"""Module 1: establish a home heading, sweep 360 degrees, return home.

Runs on its own and exits with a verdict. Three phases:

  1. SPINNING  turn continuously for ~30s and count how many times the
               view comes back round. That measures how fast the robot
               actually rotates on this surface, rather than trusting a
               constant.
  2. SWEEPING  one accurate turn using the measured rate, sampling the
               room into heading bins.
  3. RETURNING come home by matching the live view against that map.

Phase 1 exists because the robot has no encoders or IMU. roam.py
assumes 7.0 ms per degree, but the true figure depends on the surface,
battery charge and motor wear -- and a sweep built on a wrong constant
silently covers more or less than a full circle. Counting revolutions
with the camera measures it instead: if the view returns N times in T
seconds, one turn took T/N.

Homing (phase 3) is closed-loop on the camera, so it does not inherit
any residual timing error.

The report at the end gives a match score for the returned view against
the startup view, where 1.0 is identical. Anything above ~0.8 means the
robot is looking at what it started at.

Run with:
  ros2 run person_follower calibrate_room
"""

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

CMD_TOPIC = "/follow_cmd_vel"
IMAGE_TOPIC = "/conduit/camera/front/image_raw/compressed"

WAITING = "WAITING"      # no frames yet
SPINNING = "SPINNING"    # measuring the true rotation rate
SWEEPING = "SWEEPING"    # the 360 degree turn
RETURNING = "RETURNING"  # homing on the map
DONE = "DONE"


class CalibrateRoom(Node):
    def __init__(self):
        super().__init__("calibrate_room")

        self.declare_parameter("image_topic", IMAGE_TOPIC)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("bins", 36)
        self.declare_parameter("ms_per_degree", 7.0)
        self.declare_parameter("home_tolerance_deg", 8.0)
        self.declare_parameter("rate", 20.0)
        # Give up rather than spinning forever if homing cannot settle.
        self.declare_parameter("return_timeout", 20.0)
        # Phase 1: spin this long, counting revolutions, to measure the
        # true rotation rate. Longer means more revolutions and a better
        # estimate; 30s gives roughly 10 at the default speed.
        self.declare_parameter("spin_secs", 30.0)
        self.declare_parameter("measure_rate", True)
        # Correlation thresholds for counting a revolution. The view
        # correlates ~0.8 at 2deg from home and ~0 by 10deg, so these sit
        # comfortably inside that peak.
        self.declare_parameter("peak_enter", 0.55)
        self.declare_parameter("peak_exit", 0.35)
        # How far the best-matching bin must stand out from unrelated
        # bins, in standard deviations. Absolute correlation is not
        # usable here because the robot drifts as it turns; see
        # _heading_from_map.
        self.declare_parameter("match_sigma", 3.0)
        # How fast the peak-detection ceiling decays, in correlation
        # units per second. Too slow and an early high score keeps the
        # gate above every later peak.
        self.declare_parameter("ceiling_decay", 0.08)
        # Shortest believable time for one revolution, in seconds. The
        # robot physically cannot spin faster than this, so anything
        # sooner is a false peak rather than a lap.
        self.declare_parameter("min_period", 1.5)
        # Write the captured bins out as one strip image, so you can see
        # what the robot saw at each heading and judge whether the room
        # has enough distinct texture for homing to work.
        self.declare_parameter("save_map_image", "/tmp/room_map.png")

        g = self.get_parameter
        self.turn_speed = g("turn_speed").value
        self.n_bins = max(8, int(g("bins").value))
        self.ms_per_degree = g("ms_per_degree").value
        self.home_tol = g("home_tolerance_deg").value
        self.rate = g("rate").value
        self.return_timeout = g("return_timeout").value
        self.spin_secs = g("spin_secs").value
        self.measure_rate = g("measure_rate").value
        self.peak_enter = g("peak_enter").value
        self.peak_exit = g("peak_exit").value
        self.match_sigma = g("match_sigma").value
        self._ceiling_decay = g("ceiling_decay").value
        self.min_period = g("min_period").value
        self.map_image_path = g("save_map_image").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage, g("image_topic").value, self.on_image, qos
        )
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        self.live_view = None
        self.home_view = None
        self.room_map = [None] * self.n_bins
        self.swept = 0.0
        self.peaks = []          # times the view came back round
        self._armed = False
        self._best = -1.0
        self._best_t = 0.0
        self.measured_ms = None
        self.state = WAITING
        self.state_since = self.get_clock().now()

        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            f"Room calibration: {self.n_bins} bins, "
            f"{360.0 / self.n_bins:.0f}deg apart, turn_speed={self.turn_speed}"
        )
        self.get_logger().info("Waiting for camera frames...")

    # ---------------- camera ----------------

    def on_image(self, msg):
        frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            return
        small = cv2.resize(frame, (64, 64), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)
        # Zero-mean so overall brightness changes do not drive the score.
        self.live_view = gray - gray.mean()

    @staticmethod
    def _correlate(a, b):
        if a is None or b is None:
            return None
        denom = float(np.linalg.norm(a) * np.linalg.norm(b))
        if denom < 1e-6:
            return None
        return float((a * b).sum() / denom)

    def _heading_from_map(self):
        """Best-matching heading in degrees, or None if the map cannot say.

        Judged on how far the best bin stands out from the rest of the
        room, not on its absolute correlation. The robot does not pivot
        cleanly about a point -- one shared motor PWM and four-wheel
        skid steer make it walk as it turns -- so the view never repeats
        exactly and absolute scores sag badly. Measured on a simulated
        room, a drift of 10% of the field of view drops correlation from
        1.00 to 0.43 while the correct bin still ranks first every time.
        Ranking survives what thresholds do not.
        """
        scores = []
        for i, view in enumerate(self.room_map):
            c = self._correlate(view, self.live_view)
            if c is not None:
                scores.append((c, i))
        if len(scores) < 3:
            return None

        scores.sort(reverse=True)
        best, best_i = scores[0]

        # Nearby bins overlap and are meant to score alike, so a rival
        # only counts as ambiguity if it is from elsewhere in the room.
        neighbourhood = max(2, int(round(self.n_bins / 12)))
        far = [c for c, i in scores
               if min(abs(i - best_i), self.n_bins - abs(i - best_i)) > neighbourhood]
        if not far:
            return None

        # Compare the winner against the spread of unrelated bins. In a
        # featureless room every bin scores alike and this margin
        # vanishes, which is exactly when the answer should be withheld.
        import statistics
        bg_mean = statistics.fmean(far)
        bg_sd = statistics.pstdev(far) if len(far) > 1 else 0.0
        if bg_sd < 1e-6:
            return None
        if (best - bg_mean) / bg_sd < self.match_sigma:
            return None

        return best_i * (360.0 / self.n_bins)

    def _elapsed(self):
        return (self.get_clock().now() - self.state_since).nanoseconds / 1e9

    def _enter(self, state):
        self.state = state
        self.state_since = self.get_clock().now()

    # ---------------- main loop ----------------

    def tick(self):
        w = 0.0

        if self.state == WAITING:
            if self.live_view is not None:
                self.home_view = self.live_view.copy()
                if self.measure_rate:
                    self.get_logger().info(
                        f"Home view captured. Spinning {self.spin_secs:.0f}s "
                        "to measure the rotation rate..."
                    )
                    self._enter(SPINNING)
                else:
                    self.room_map[0] = self.home_view
                    self.get_logger().info("Home view captured. Sweeping 360deg...")
                    self._enter(SWEEPING)

        elif self.state == SPINNING:
            w = self._do_spin()

        elif self.state == SWEEPING:
            w = self._do_sweep()

        elif self.state == RETURNING:
            w = self._do_return()

        cmd = Twist()
        cmd.angular.z = w
        cmd.linear.x = 0.0   # rotation only, always
        self.cmd_pub.publish(cmd)

    def _do_spin(self):
        """Turn continuously, timing each return to the starting view.

        Correlation against the home view rises to a sharp peak once per
        revolution. Each peak is logged at its maximum rather than at the
        threshold crossing, so the timing does not depend on how steeply
        the robot passes through.
        """
        t = self._elapsed()
        c = self._correlate(self.home_view, self.live_view)

        if c is not None:
            # Gate relative to a DECAYING ceiling. The first revolution
            # often matches near 1.0 because nothing has moved yet, while
            # later ones peak far lower as the robot drifts off its pivot.
            # A plain running maximum locks the gate to that first score
            # and then rejects every real peak after it -- observed on
            # hardware as 2 detections in 30s where there should be ~12.
            # Decaying lets the threshold follow the view as it degrades.
            ceiling = getattr(self, "_ceiling", None)
            if ceiling is None:
                ceiling = c
            else:
                # Fall slowly, rise immediately.
                ceiling = max(c, ceiling - self._ceiling_decay / self.rate)
            self._ceiling = ceiling

            enter = max(self.peak_enter * ceiling, 0.40)
            leave = max(self.peak_exit * ceiling, 0.25)

            if c >= enter:
                self._armed = True
                if c > self._best:
                    self._best, self._best_t = c, t
            elif self._armed and c < leave:
                # Refractory gate: a genuine revolution cannot arrive
                # sooner than this. Without it, ordinary scene variation
                # crossing a low threshold is counted as a revolution --
                # observed on hardware as 25 "revolutions" in 30s with
                # gaps as short as 0.4s, which halved the measured rate.
                too_soon = (
                    self.peaks and
                    self._best_t - self.peaks[-1] < self.min_period
                )
                if too_soon:
                    # Keep whichever candidate matched better.
                    if self._best > getattr(self, "_last_peak_score", 0.0):
                        self.peaks[-1] = self._best_t
                        self._last_peak_score = self._best
                    self._armed = False
                    self._best = -1.0
                    return self.turn_speed
                self._last_peak_score = self._best
                self.peaks.append(self._best_t)
                self.get_logger().info(
                    f"  revolution {len(self.peaks)} at {self._best_t:.1f}s "
                    f"(peak {self._best:.2f})"
                )
                self._armed = False
                self._best = -1.0

        if t < self.spin_secs:
            return self.turn_speed

        # Spin over: turn the peak times into a rate.
        #
        # Peaks are only trustworthy if they are periodic -- real
        # revolutions land on multiples of one period, false ones land
        # anywhere. Rather than averaging gaps (which a single missed or
        # spurious peak skews badly), find the period that best explains
        # the peak times, then check how cleanly it does so. If the
        # peaks are not convincingly periodic the measurement is refused
        # outright: a wrong rate is worse than the default, because it
        # silently makes every later sweep cover the wrong arc.
        period, quality, explained = self._fit_period(self.peaks)

        if period is None:
            self.get_logger().warn(
                f"Only {len(self.peaks)} peaks in {self.spin_secs:.0f}s -- too few "
                f"to measure from. Keeping ms_per_degree={self.ms_per_degree:.2f}."
            )
            self.get_logger().warn(
                "  Lower peak_enter (try 0.35) if the view degrades as the "
                "robot drifts, or raise spin_secs for more revolutions."
            )
        elif quality > 0.12:
            self.get_logger().warn(
                f"  {len(self.peaks)} peaks, but they are not cleanly periodic "
                f"(best period {period:.2f}s explains only "
                f"{explained}/{len(self.peaks)}, scatter {quality:.2f})."
            )
            self.get_logger().warn(
                f"  Refusing to measure; keeping ms_per_degree="
                f"{self.ms_per_degree:.2f}. The robot is probably drifting off "
                "its pivot so the view never repeats cleanly, or the room has "
                "repeating features."
            )
        else:
            measured = (period * 1000.0) / 360.0
            self.measured_ms = measured
            self.get_logger().info(
                f"  {len(self.peaks)} peaks, period {period:.2f}s "
                f"({explained} fit it, scatter {quality:.2f})"
            )
            self.get_logger().info(
                f"  measured ms_per_degree = {measured:.2f} "
                f"(was {self.ms_per_degree:.2f})"
            )
            self.ms_per_degree = measured

        self.room_map[0] = self.live_view.copy()
        self.home_view = self.room_map[0]
        self.swept = 0.0
        self.get_logger().info("Sweeping 360deg with the measured rate...")
        self._enter(SWEEPING)
        return 0.0

    def _fit_period(self, peaks):
        """Best period for these peak times, plus how well it fits.

        Returns (period, scatter, n_explained). Scatter is the mean
        distance from a peak to its nearest multiple of the period, as a
        fraction of the period -- near 0 for clean data, large when the
        peaks are not really periodic. A period is only accepted if it
        explains most of the peaks, so a short period cannot win simply
        by fitting noise.
        """
        if len(peaks) < 4:
            return None, 1.0, 0

        span = peaks[-1] - peaks[0]
        best = (None, 1.0, 0)
        p = self.min_period
        while p <= span / 3.0:
            errs = [abs(t - round(t / p) * p) for t in peaks]
            fit = [e for e in errs if e <= 0.3 * p]
            # Demand that most peaks fit, so noise cannot be explained
            # away by choosing a very short period.
            if len(fit) >= max(4, int(0.75 * len(peaks))):
                scatter = (sum(fit) / len(fit)) / p
                if scatter < best[1]:
                    best = (p, scatter, len(fit))
            p += 0.01
        return best

    def _do_sweep(self):
        bin_width = 360.0 / self.n_bins
        idx = int(self.swept / bin_width)
        if 0 <= idx < self.n_bins and self.room_map[idx] is None:
            self.room_map[idx] = self.live_view.copy()

        self.swept += (1.0 / self.rate) * 1000.0 / self.ms_per_degree

        if self.swept >= 360.0:
            filled = sum(1 for v in self.room_map if v is not None)
            self.get_logger().info(
                f"Sweep complete: {filled}/{self.n_bins} bins captured"
            )
            self.get_logger().info("Returning home using the room map...")
            self._enter(RETURNING)
            return 0.0

        return self.turn_speed

    def _do_return(self):
        heading = self._heading_from_map()

        if heading is None:
            if self._elapsed() > self.return_timeout:
                self._report(None, "map could not place the view")
                return 0.0
            # Keep turning slowly; a different view may be recognisable.
            return self.turn_speed * 0.6

        # Shortest way round: +350 means turn -10, not +350.
        error = -((heading + 180.0) % 360.0 - 180.0)

        if abs(error) <= self.home_tol:
            self._report(heading, None)
            return 0.0

        if self._elapsed() > self.return_timeout:
            self._report(heading, "timed out before settling")
            return 0.0

        return math.copysign(self.turn_speed * 0.6, error)

    # ---------------- report ----------------

    def _report(self, heading, problem):
        """Print the verdict.

        Heading error and view match measure different things and are
        reported separately. Correlation falls off steeply with angle --
        on a textured room roughly 0.8 at 2deg, 0.5 at 5deg, nothing by
        10deg -- so a modest match is normal when the robot is correctly
        home to within a bin width, and is not a failure. What matters
        is whether the map placed the heading at all.
        """
        match = self._correlate(self.home_view, self.live_view)
        filled = sum(1 for v in self.room_map if v is not None)
        bin_width = 360.0 / self.n_bins
        error = None
        if heading is not None:
            error = abs((heading + 180.0) % 360.0 - 180.0)

        self.get_logger().info("=" * 56)
        self.get_logger().info(f"  bins captured  : {filled}/{self.n_bins}")
        if error is not None:
            self.get_logger().info(
                f"  heading error  : {error:.0f}deg  "
                f"(bin width {bin_width:.0f}deg, tolerance {self.home_tol:.0f}deg)"
            )
        else:
            self.get_logger().info("  heading error  : unknown, map could not place the view")
        if match is not None:
            self.get_logger().info(
                f"  view match     : {match:.2f}  "
                "(falls off fast with angle; low is normal within a bin)"
            )

        if problem:
            self.get_logger().warn(f"  RESULT: FAIL -- {problem}")
            self.get_logger().warn(
                "  Either the room is too featureless to localise in, or "
                "ms_per_degree is wrong so the sweep missed a full circle."
            )
        elif filled < self.n_bins * 0.8:
            self.get_logger().warn(
                f"  RESULT: FAIL -- only {filled}/{self.n_bins} bins captured. "
                "The camera may be dropping frames, or the sweep was too fast."
            )
        elif error is not None and error <= self.home_tol:
            self.get_logger().info(
                f"  RESULT: PASS -- swept a full circle and found home "
                f"to within {error:.0f}deg"
            )
        else:
            self.get_logger().warn("  RESULT: FAIL -- did not settle at home")
        self.get_logger().info("=" * 56)

        self._save_map_image()
        self._enter(DONE)
        raise SystemExit

    def _save_map_image(self):
        """Lay every captured bin side by side, labelled by heading."""
        if not self.map_image_path:
            return
        tiles, labels = [], []
        for i, view in enumerate(self.room_map):
            if view is None:
                continue
            # Signatures are zero-mean floats; rescale to viewable gray.
            v = view - view.min()
            peak = v.max()
            v = (v / peak * 255.0) if peak > 1e-6 else v
            tiles.append(cv2.resize(v.astype(np.uint8), (64, 64)))
            labels.append(i * (360.0 / self.n_bins))
        if not tiles:
            return

        strip = cv2.cvtColor(np.hstack(tiles), cv2.COLOR_GRAY2BGR)
        header = np.zeros((20, strip.shape[1], 3), dtype=np.uint8)
        for n, deg in enumerate(labels):
            cv2.putText(
                header, f"{deg:.0f}", (n * 64 + 4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1,
            )
        try:
            cv2.imwrite(self.map_image_path, np.vstack([header, strip]))
            self.get_logger().info(f"  room map image: {self.map_image_path}")
        except Exception as e:
            self.get_logger().warn(f"  could not write map image: {e}")


def main():
    rclpy.init()
    node = CalibrateRoom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Leave the robot stopped whatever happened.
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
