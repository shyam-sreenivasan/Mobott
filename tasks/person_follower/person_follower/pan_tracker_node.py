"""Rotate in place to keep a person centred. Never drives anywhere.

Built for a stationary use: the robot sits on a desk or floor during a
video call and turns to keep whoever is in the room framed. Distance is
irrelevant here, so linear.x stays 0 at all times -- unlike
follow_controller, which also closes distance.

State machine:

  TRACKING  person visible -> turn to centre them
  SEARCHING person lost    -> sweep in the direction they were last
                              heading, since that is where they went
  COOLDOWN  a full sweep found nobody -> pause, then sweep again
  IDLE      all sweeps failed -> return to the home heading and wait,
                                 still watching, resuming the moment
                                 anyone appears

The robot has no encoders or IMU, so every angle here is dead-reckoned
from how long a turn was commanded (MS_PER_DEGREE in roam.py, measured
at 7.0 ms per degree). "A full 360" is really "turned for 2.5 seconds",
and the home heading drifts over a long session. That is accurate enough
to decide when to stop sweeping, which is all it is used for.

Run with:  ros2 run person_follower pan_tracker
"""

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

TARGET_TOPIC = "/target_person"
CMD_TOPIC = "/follow_cmd_vel"
IMAGE_TOPIC = "/conduit/camera/front/image_raw/compressed"

CALIBRATING = "CALIBRATING"
TRACKING = "TRACKING"
SEARCHING = "SEARCHING"
COOLDOWN = "COOLDOWN"
IDLE = "IDLE"
HOMING = "HOMING"


class PanTracker(Node):
    def __init__(self):
        super().__init__("pan_tracker")

        # --- tracking ---
        self.declare_parameter("deadzone_deg", 8.0)
        self.declare_parameter("turn_speed", 0.6)
        self.declare_parameter("min_turn", 0.35)
        self.declare_parameter("full_turn_deg", 35.0)
        self.declare_parameter("min_confidence", 0.4)
        # Person counts as lost after this long with no fresh detection.
        self.declare_parameter("lost_after", 1.5)

        # --- searching ---
        self.declare_parameter("search_speed", 0.5)
        # Degrees to sweep before declaring one attempt failed.
        self.declare_parameter("search_arc_deg", 360.0)
        self.declare_parameter("search_attempts", 3)
        self.declare_parameter("cooldown_secs", 10.0)
        # ms of commanded turn per degree of rotation. Same open-loop
        # calibration roam.py uses; retune if sweeps over/undershoot.
        self.declare_parameter("ms_per_degree", 7.0)

        # --- home ---
        self.declare_parameter("return_home", True)
        self.declare_parameter("home_tolerance_deg", 10.0)
        # Visual homing: save a reference frame at startup and rotate
        # until the live view matches it again. Real feedback, so unlike
        # the dead-reckoned offset it does not drift over a long session.
        # Falls back to dead reckoning when no frame has arrived.
        self.declare_parameter("visual_home", True)
        # Sweep a full circle at startup, recording a view signature every
        # few degrees. The resulting map lets homing estimate the current
        # heading from the camera instead of sweeping blindly for one
        # remembered frame.
        self.declare_parameter("calibrate", True)
        self.declare_parameter("calibration_bins", 36)   # every 10 degrees
        self.declare_parameter("image_topic", IMAGE_TOPIC)
        # Correlation above this counts as "back at the home view". Too
        # high and it never settles; too low and it stops early.
        self.declare_parameter("home_match_threshold", 0.80)

        self.declare_parameter("rate", 20.0)

        g = self.get_parameter
        self.deadzone = math.radians(g("deadzone_deg").value)
        self.turn_speed = g("turn_speed").value
        self.min_turn = g("min_turn").value
        self.full_turn = math.radians(g("full_turn_deg").value)
        self.min_conf = g("min_confidence").value
        self.lost_after = g("lost_after").value
        self.search_speed = g("search_speed").value
        self.search_arc = g("search_arc_deg").value
        self.max_attempts = g("search_attempts").value
        self.cooldown = g("cooldown_secs").value
        self.ms_per_degree = g("ms_per_degree").value
        self.return_home = g("return_home").value
        self.home_tol = g("home_tolerance_deg").value
        self.visual_home = g("visual_home").value
        self.calibrate = g("calibrate").value
        self.n_bins = max(8, int(g("calibration_bins").value))
        self.home_match_threshold = g("home_match_threshold").value

        self.create_subscription(Point, TARGET_TOPIC, self.on_target, 10)
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        self.home_view = None      # grayscale signature of the home frame
        self.live_view = None      # most recent frame, same form
        self.best_match = -1.0     # best correlation seen this homing run
        # Room map: one view signature per heading bin, filled by the
        # startup sweep. Index i covers heading i * (360 / n_bins).
        self.room_map = [None] * self.n_bins
        self.calib_swept = 0.0
        if self.visual_home:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                CompressedImage, g("image_topic").value, self.on_image, qos
            )

        self.bearing = None
        self.last_target = None
        self.state = TRACKING
        # Dead-reckoned heading relative to startup, in degrees. Positive
        # is left, matching the ROS angular.z convention.
        self.offset_deg = 0.0
        self.search_swept = 0.0
        self.attempts = 0
        self.state_since = self.get_clock().now()
        # Which way the person was last drifting; the search starts that
        # way because it is the better guess.
        self.last_direction = 1.0

        self.rate = g("rate").value
        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            f"Pan tracking only (linear.x always 0). "
            f"deadzone={math.degrees(self.deadzone):.0f}deg "
            f"search={self.search_arc:.0f}deg x{self.max_attempts} "
            f"cooldown={self.cooldown:.0f}s"
        )
        if self.calibrate and self.visual_home:
            self.state = CALIBRATING
            self.get_logger().info(
                f"Calibrating: one 360deg sweep, {self.n_bins} view samples"
            )
        else:
            self.get_logger().info("Home heading set to current position")

    # ---------------- input ----------------

    def on_target(self, msg):
        if msg.z < self.min_conf:
            return
        self.bearing = msg.x
        self.last_target = self.get_clock().now()

    def on_image(self, msg):
        """Keep a small grayscale signature of the latest frame.

        Downscaled hard: the comparison only needs coarse structure, and
        a small image makes the correlation cheap and tolerant of noise,
        exposure shifts and people moving through the scene.
        """
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            return
        small = cv2.resize(frame, (64, 64), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)
        # Zero-mean so overall brightness changes do not affect the score.
        self.live_view = gray - gray.mean()

        if self.home_view is None:
            self.home_view = self.live_view.copy()
            self.get_logger().info("Home view captured from camera")

    def _home_match(self):
        """Correlation between the live frame and the home frame, -1..1."""
        if self.home_view is None or self.live_view is None:
            return None
        a, b = self.home_view, self.live_view
        denom = float(np.linalg.norm(a) * np.linalg.norm(b))
        if denom < 1e-6:
            return None
        return float((a * b).sum() / denom)

    def _seen_recently(self):
        if self.last_target is None:
            return False
        age = (self.get_clock().now() - self.last_target).nanoseconds / 1e9
        return age <= self.lost_after

    def _in_state_for(self):
        return (self.get_clock().now() - self.state_since).nanoseconds / 1e9

    def _enter(self, state):
        if state != self.state:
            self.get_logger().info(f"{self.state} -> {state}")
            self.state = state
            self.state_since = self.get_clock().now()

    # ---------------- main loop ----------------

    def tick(self):
        """Fixed-rate so the firmware's 300ms failsafe never trips mid-turn."""
        w = 0.0

        if self.state == CALIBRATING:
            w = self._do_calibrating()
        elif self.state == TRACKING:
            w = self._do_tracking()
        elif self.state == SEARCHING:
            w = self._do_searching()
        elif self.state == COOLDOWN:
            w = self._do_cooldown()
        elif self.state == HOMING:
            w = self._do_homing()
        elif self.state == IDLE:
            w = self._do_idle()

        # Integrate the commanded turn into the dead-reckoned heading.
        # w is a normalised motor command, not rad/s, so convert via the
        # same ms-per-degree calibration the sweep uses.
        if w != 0.0 and self.ms_per_degree > 0:
            dt = 1.0 / self.rate
            self.offset_deg += math.copysign(
                (dt * 1000.0 / self.ms_per_degree) * (abs(w) / self.turn_speed),
                w,
            )

        cmd = Twist()
        cmd.angular.z = w
        cmd.linear.x = 0.0   # never drives, by design
        self.cmd_pub.publish(cmd)

    # ---------------- states ----------------

    def _do_calibrating(self):
        """One slow full turn, sampling the view into heading bins.

        Runs before tracking so the map reflects the room as it is now --
        this lighting, this furniture, this starting pose. Sampling is
        driven by the dead-reckoned angle, which is accurate enough over
        a single uninterrupted sweep; it is the later *matching* that has
        to be drift-free, and that uses the camera.
        """
        if self.live_view is None:
            # No frames yet. Hold still rather than sweeping blind.
            return 0.0

        bin_width = 360.0 / self.n_bins
        idx = int(self.calib_swept / bin_width)
        if 0 <= idx < self.n_bins and self.room_map[idx] is None:
            self.room_map[idx] = self.live_view.copy()

        step = (1.0 / self.rate) * 1000.0 / self.ms_per_degree
        step *= self.search_speed / self.turn_speed
        self.calib_swept += step

        if self.calib_swept >= 360.0:
            filled = sum(1 for v in self.room_map if v is not None)
            self.home_view = self.room_map[0]
            self.offset_deg = 0.0
            self.get_logger().info(
                f"Calibration done: {filled}/{self.n_bins} bins captured"
            )
            self._enter(TRACKING)
            return 0.0

        return self.search_speed   # always one consistent direction

    def _heading_from_map(self):
        """Best-matching heading in degrees, or None if the map cannot say.

        Returns the bin centre whose signature correlates best with the
        live view, provided it beats the runner-up clearly enough to be
        trusted -- a featureless room makes every bin score alike, and a
        confident wrong answer is worse than no answer.
        """
        if self.live_view is None:
            return None
        scores = []
        for i, view in enumerate(self.room_map):
            if view is None:
                continue
            denom = float(np.linalg.norm(view) * np.linalg.norm(self.live_view))
            if denom < 1e-6:
                continue
            scores.append(((view * self.live_view).sum() / denom, i))
        if len(scores) < 2:
            return None

        scores.sort(reverse=True)
        best, best_i = scores[0]
        if best < self.home_match_threshold * 0.8:
            return None
        # Require a margin over the best bin that is well away from the
        # winner, so a featureless scene does not produce a coin-flip.
        # Nearby bins are *expected* to score alike -- their views
        # overlap, and a heading between two bins ties them legitimately
        # -- so only a distant rival counts as genuine ambiguity.
        bin_width = 360.0 / self.n_bins
        neighbourhood = max(2, int(round(self.n_bins / 12)))
        for score, i in scores[1:]:
            gap = min(abs(i - best_i), self.n_bins - abs(i - best_i))
            if gap > neighbourhood:
                if best - score < 0.05:
                    return None
                break
        return best_i * bin_width

    def _do_tracking(self):
        if not self._seen_recently():
            self.attempts = 0
            self.search_swept = 0.0
            self._enter(SEARCHING)
            return 0.0

        if abs(self.bearing) <= self.deadzone:
            return 0.0

        # Remember which way we are turning: if the person then leaves
        # frame, they most likely carried on that way.
        self.last_direction = math.copysign(1.0, self.bearing)

        excess = abs(self.bearing) - self.deadzone
        span = max(self.full_turn - self.deadzone, 1e-6)
        frac = min(1.0, excess / span)
        mag = self.min_turn + frac * (self.turn_speed - self.min_turn)
        return math.copysign(mag, self.bearing)

    def _do_searching(self):
        if self._seen_recently():
            self.get_logger().info("Found a person; resuming tracking")
            self.attempts = 0
            self._enter(TRACKING)
            return 0.0

        swept_this_tick = (1.0 / self.rate) * 1000.0 / self.ms_per_degree
        swept_this_tick *= self.search_speed / self.turn_speed
        self.search_swept += swept_this_tick

        if self.search_swept >= self.search_arc:
            self.attempts += 1
            self.search_swept = 0.0
            if self.attempts >= self.max_attempts:
                self.get_logger().info(
                    f"No person after {self.attempts} sweeps; "
                    + ("returning home" if self.return_home else "waiting")
                )
                self._enter(HOMING if self.return_home else IDLE)
            else:
                self.get_logger().info(
                    f"Sweep {self.attempts}/{self.max_attempts} found nobody; "
                    f"pausing {self.cooldown:.0f}s"
                )
                self._enter(COOLDOWN)
            return 0.0

        return math.copysign(self.search_speed, self.last_direction)

    def _do_cooldown(self):
        if self._seen_recently():
            self.get_logger().info("Person appeared during cooldown")
            self.attempts = 0
            self._enter(TRACKING)
            return 0.0

        if self._in_state_for() >= self.cooldown:
            self._enter(SEARCHING)
        return 0.0

    def _do_homing(self):
        # Still worth watching on the way back.
        if self._seen_recently():
            self.get_logger().info("Person appeared while homing")
            self.attempts = 0
            self._enter(TRACKING)
            return 0.0

        # Preferred: ask the room map which way we are facing, then turn
        # straight to 0. This is closed-loop, so it ignores accumulated
        # dead-reckoning drift.
        heading = self._heading_from_map() if self.visual_home else None
        if heading is not None:
            # Shortest way round: +170 means turn -170, not +190.
            error = -((heading + 180.0) % 360.0 - 180.0)
            if abs(error) <= self.home_tol:
                self.get_logger().info(
                    f"At home (camera says {heading:.0f}deg); waiting for a person"
                )
                self.offset_deg = 0.0   # a real fix, so re-zero the estimate
                self._enter(IDLE)
                return 0.0
            return math.copysign(self.search_speed, error)

        # Map could not place us -- too few bins, or a featureless view.
        # Fall back to the dead-reckoned offset.
        error = -self.offset_deg
        if abs(error) <= self.home_tol:
            self.get_logger().info(
                f"At home (dead reckoning, offset {self.offset_deg:+.0f}deg); "
                "waiting for a person"
            )
            self._enter(IDLE)
            return 0.0

        if self._in_state_for() > (720.0 * self.ms_per_degree / 1000.0):
            self.get_logger().warn("Homing timed out; stopping where we are")
            self._enter(IDLE)
            return 0.0

        return math.copysign(self.search_speed, error)

    def _do_idle(self):
        """Wait, watching. Any confident detection restarts tracking."""
        if self._seen_recently():
            self.get_logger().info("Person detected; resuming tracking")
            self.attempts = 0
            self._enter(TRACKING)
        return 0.0


def main():
    rclpy.init()
    node = PanTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
