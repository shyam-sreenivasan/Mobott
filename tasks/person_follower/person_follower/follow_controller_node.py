"""Stage 4: steer toward the target and hold a following distance.

Bearing drives angular.z, distance error drives linear.x:
  person too far  -> forward
  at target range -> stop
  person too close-> back up

Sign convention, consistent from tracker to firmware:
  person LEFT of centre  -> bearing > 0 -> angular.z > 0 -> turn left
The car's motionUpdate() computes left = v - w, right = v + w, so a
positive w already turns left. Nothing is negated along the way.

Distance arrives on /target_person as msg.y, in one of two forms:
  y < 0  -> box height in pixels (Stage 4, no depth sensor)
  y > 0  -> real metres (Stage 5 onward)
Both are handled, so swapping in a depth sensor needs no change here.

Set linear_enabled:=false to get the Stage 3 rotate-in-place behaviour.

Run with:  ros2 run person_follower follow_controller
"""

import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node

from person_follower.pid import PID

TARGET_TOPIC = "/target_person"
CMD_TOPIC = "/follow_cmd_vel"


class FollowController(Node):
    def __init__(self):
        super().__init__("follow_controller")

        self.declare_parameter("kp", 1.2)
        # The firmware's deadband is 0.05, so anything smaller is a command
        # the car physically ignores. Stopping short of it avoids buzzing
        # the motors with commands that cannot move the wheels.
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("min_angular", 0.08)
        # Bearing within this many degrees counts as centred.
        self.declare_parameter("deadzone_deg", 4.0)
        self.declare_parameter("min_confidence", 0.35)
        # No target for this long -> stop. Shorter than the firmware's own
        # 300ms failsafe would fight it; longer means coasting blind.
        # Kept short on purpose: the controller holds the LAST bearing and
        # box height, so a long timeout means the car keeps driving toward
        # where the person used to be after they leave frame.
        self.declare_parameter("target_timeout", 0.25)
        self.declare_parameter("rate", 20.0)

        # --- forward/backward (Stage 4) ---
        self.declare_parameter("linear_enabled", True)
        # Desired box height in pixels. On a 640-tall portrait frame a
        # standing adult is roughly this tall at a couple of metres; it is
        # a crude proxy and is meant to be tuned by eye, not calculated.
        self.declare_parameter("target_box_h", 260.0)
        # Real distance in metres, used instead once Stage 5 supplies depth.
        self.declare_parameter("target_distance_m", 2.0)
        self.declare_parameter("kp_linear", 0.0035)      # per pixel of error
        self.declare_parameter("kp_linear_metric", 0.6)  # per metre of error
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("min_linear", 0.10)
        self.declare_parameter("max_reverse", 0.25)
        # Tolerance band around the target: inside it the car holds still
        # rather than hunting back and forth.
        self.declare_parameter("distance_deadzone_px", 35.0)
        self.declare_parameter("distance_deadzone_m", 0.25)
        # Do not drive forward while still swinging hard toward the target;
        # turning first, then advancing, gives a much cleaner path.
        self.declare_parameter("turn_first_deg", 25.0)

        # "pivot"   -- turn in place, then drive straight, never both at
        #              once. Required on this car: one shared PWM line
        #              means a blended v,w cannot curve, it only pivots.
        # "blended" -- classic simultaneous v and w, for hardware with
        #              independent per-side speed control.
        self.declare_parameter("drive_mode", "pivot")

        # Grace period between the first detection and the first movement,
        # so you can step into frame and get clear before the car starts.
        # One-shot: once it has elapsed the car follows continuously, and
        # later losses of the target do not re-arm it.
        self.declare_parameter("start_delay", 5.0)

        # Fixed speeds for pivot mode, matching the values roam.py already
        # uses successfully on this car. Lower values may not overcome
        # friction once the car is on the floor.
        self.declare_parameter("cruise_speed", 1.0)
        self.declare_parameter("turn_speed", 0.6)
        # Ramp the turn with the bearing error instead of always pivoting at
        # turn_speed. Full-speed pivots overshoot on a car with no encoders
        # and no feedback, which shows up as hunting around centre.
        self.declare_parameter("proportional_turn", True)
        # Turn magnitude at the deadzone edge. Must clear the firmware's
        # 0.05 deadband with margin, or the wheels buzz without moving.
        self.declare_parameter("min_turn", 0.35)
        # Bearing error at which the turn reaches full turn_speed.
        self.declare_parameter("full_turn_deg", 40.0)
        # Cap how fast the turn command may change, in units per second.
        # Without this the car slams straight to full turn the instant a
        # target reappears at the frame edge; ramping in makes it ease
        # into the correction instead. 0 disables the limit.
        self.declare_parameter("turn_slew", 1.2)

        # --- PID on the bearing (drive_mode "pid") ---
        # Tuned for a plant with ~280ms of lag: the firmware's ALPHA=0.2
        # smoothing means a commanded turn takes that long to take effect,
        # so kd matters more than usual and ki is kept small.
        self.declare_parameter("pid_kp", 1.1)
        self.declare_parameter("pid_ki", 0.15)
        self.declare_parameter("pid_kd", 0.35)
        self.declare_parameter("pid_integral_limit", 0.4)
        self.declare_parameter("reverse_speed", 1.0)

        self.kp = self.get_parameter("kp").value
        self.max_angular = self.get_parameter("max_angular").value
        self.min_angular = self.get_parameter("min_angular").value
        self.deadzone = math.radians(self.get_parameter("deadzone_deg").value)
        self.min_conf = self.get_parameter("min_confidence").value
        self.timeout = self.get_parameter("target_timeout").value

        self.linear_enabled = self.get_parameter("linear_enabled").value
        self.target_box_h = self.get_parameter("target_box_h").value
        self.target_distance_m = self.get_parameter("target_distance_m").value
        self.kp_linear = self.get_parameter("kp_linear").value
        self.kp_linear_metric = self.get_parameter("kp_linear_metric").value
        self.max_linear = self.get_parameter("max_linear").value
        self.min_linear = self.get_parameter("min_linear").value
        self.max_reverse = self.get_parameter("max_reverse").value
        self.dz_px = self.get_parameter("distance_deadzone_px").value
        self.dz_m = self.get_parameter("distance_deadzone_m").value
        self.turn_first = math.radians(self.get_parameter("turn_first_deg").value)
        self.drive_mode = self.get_parameter("drive_mode").value
        self.start_delay = self.get_parameter("start_delay").value
        self.cruise_speed = self.get_parameter("cruise_speed").value
        self.turn_speed = self.get_parameter("turn_speed").value
        self.proportional_turn = self.get_parameter("proportional_turn").value
        self.min_turn = self.get_parameter("min_turn").value
        self.full_turn = math.radians(self.get_parameter("full_turn_deg").value)
        self.turn_slew = self.get_parameter("turn_slew").value
        self._last_w = 0.0
        self._last_tick = None

        self.pid = PID(
            kp=self.get_parameter("pid_kp").value,
            ki=self.get_parameter("pid_ki").value,
            kd=self.get_parameter("pid_kd").value,
            out_limit=self.get_parameter("turn_speed").value,
            integral_limit=self.get_parameter("pid_integral_limit").value,
        )
        self.reverse_speed = self.get_parameter("reverse_speed").value

        if self.drive_mode not in ("pivot", "blended", "pid"):
            self.get_logger().warn(
                f"Unknown drive_mode '{self.drive_mode}', falling back to pivot"
            )
            self.drive_mode = "pivot"

        if self.drive_mode in ("pivot", "pid"):
            # A narrow deadzone makes the car pivot at every small wobble
            # and never get around to driving, so widen it unless the user
            # has deliberately set their own.
            if self.get_parameter("deadzone_deg").value == 4.0:
                self.deadzone = math.radians(10.0)
            # Turning already excludes driving in this mode.
            self.turn_first = math.pi

        self.create_subscription(Point, TARGET_TOPIC, self.on_target, 10)
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        self.bearing = None
        self.distance = None   # metres if positive, box-height px if negative
        self.last_target = None
        # Set on the first detection; the car stays still until start_delay
        # has passed since then. Once armed it never disarms.
        self.first_seen = None
        self.armed = self.start_delay <= 0.0
        self._last_countdown = None

        self.rate = self.get_parameter("rate").value
        self.create_timer(1.0 / self.rate, self.tick)

        if self.linear_enabled:
            self.get_logger().info(
                f"Following in '{self.drive_mode}' mode. kp={self.kp} "
                f"target_box_h={self.target_box_h}px max_linear={self.max_linear} "
                f"deadzone={math.degrees(self.deadzone):.0f}deg"
            )
            if self.drive_mode == "pivot":
                self.get_logger().info(
                    f"Turn-in-place then drive straight (shared-PWM hardware). "
                    f"cruise={self.cruise_speed} turn={self.turn_speed} "
                    f"reverse={self.reverse_speed}"
                )
        else:
            self.get_logger().info("Steering only (linear.x=0)")
        self.get_logger().info(f"{TARGET_TOPIC} -> {CMD_TOPIC}")

    def on_target(self, msg):
        if msg.z < self.min_conf:
            return   # too unsure to steer on
        self.bearing = msg.x
        self.distance = msg.y
        self.last_target = self.get_clock().now()

    def tick(self):
        """Publish on a fixed clock, not per detection.

        The car stops on its own if it hears nothing for 300ms, so commands
        have to keep flowing at a steady rate even while the target is
        briefly missing -- otherwise steering stutters.
        """
        cmd = Twist()

        if self.last_target is None:
            self._last_w = 0.0
            self.pid.reset()
            self.cmd_pub.publish(cmd)
            return

        age = (self.get_clock().now() - self.last_target).nanoseconds / 1e9
        if age > self.timeout:
            # Target lost: publish zeros rather than the last command, so a
            # stale bearing can never spin the car indefinitely. Reset the
            # ramp too, so coming back starts from a standstill.
            self._last_w = 0.0
            self.pid.reset()
            self.cmd_pub.publish(cmd)
            return

        if not self._ready():
            self._last_w = 0.0
            self.pid.reset()
            self.cmd_pub.publish(cmd)   # zeros: still counting down
            return

        if self.drive_mode == "pid":
            v, w = self._pid_command()
        elif self.drive_mode == "pivot":
            v, w = self._pivot_command()
        else:
            w = 0.0
            if abs(self.bearing) > self.deadzone:
                w = self.kp * self.bearing
                w = max(-self.max_angular, min(self.max_angular, w))
                if 0 < abs(w) < self.min_angular:
                    w = math.copysign(self.min_angular, w)
            v = self._linear() if self.linear_enabled else 0.0

        cmd.angular.z = self._slew(w)
        cmd.linear.x = v
        self.cmd_pub.publish(cmd)

    def _slew(self, w):
        """Rate-limit the turn command so it eases in rather than jumping.

        Matters most right after a target is re-acquired: the bearing can
        legitimately jump 40+ degrees in one frame, and slamming to full
        turn on that single reading is what makes the car lurch.
        """
        if self.turn_slew <= 0:
            self._last_w = w
            return w

        step = self.turn_slew / max(self.rate, 1e-6)
        delta = w - self._last_w
        if delta > step:
            w = self._last_w + step
        elif delta < -step:
            w = self._last_w - step
        self._last_w = w
        return w

    def _ready(self):
        """False while the startup grace period is still running."""
        if self.armed:
            return True

        now = self.get_clock().now()
        if self.first_seen is None:
            self.first_seen = now
            self.get_logger().info(
                f"Target acquired -- starting in {self.start_delay:.0f}s"
            )

        remaining = self.start_delay - (now - self.first_seen).nanoseconds / 1e9
        if remaining <= 0:
            self.armed = True
            self.get_logger().info("Following now")
            return True

        # One line per whole second, rather than one per 20Hz tick.
        whole = int(remaining) + 1
        if whole != self._last_countdown:
            self._last_countdown = whole
            self.get_logger().info(f"  starting in {whole}...")
        return False

    def _pid_command(self):
        """PID on bearing, still one motion at a time.

        The hardware constraint from pivot mode has not gone away -- a
        blended v,w cannot curve on a shared PWM line -- so the PID governs
        how hard to turn, not whether to turn and drive at once.
        """
        now = self.get_clock().now()
        dt = 0.05 if self._last_tick is None else \
            (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now

        if abs(self.bearing) <= self.deadzone:
            # Centred. Let the integral decay rather than holding a term
            # that will fire the moment the person drifts off centre.
            self.pid.reset()
            if not self.linear_enabled:
                return 0.0, 0.0
            v = self._linear()
            if v > 0:
                return self.cruise_speed, 0.0
            if v < 0:
                return -self.reverse_speed, 0.0
            return 0.0, 0.0

        w = self.pid.step(self.bearing, dt)

        # Anything below the firmware's deadband just buzzes the motors
        # without turning the car, which reads as the PID doing nothing.
        if 0 < abs(w) < self.min_turn:
            w = math.copysign(self.min_turn, w)
        return 0.0, w

    def _pivot_command(self):
        """One motion at a time, at roam.py's speeds.

        The car has a single shared PWM line for all four motors (see the
        shift-register wiring in motor.cpp), so it cannot run one side
        slower than the other. A blended v,w just drops the weak side
        under the firmware deadband and the car spins. roam.py sidesteps
        this by only ever driving straight or turning in place, at fixed
        speeds -- this does the same.

        Off-centre wins over distance: driving along a stale heading is
        the worse mistake.
        """
        if abs(self.bearing) > self.deadzone:
            if self.proportional_turn:
                # Scale the turn with how far off we are, so a small error
                # gets a gentle nudge rather than a full-speed pivot that
                # overshoots and starts hunting. Ramps from min_turn at the
                # deadzone edge up to turn_speed at full_turn_deg.
                excess = abs(self.bearing) - self.deadzone
                span = max(self.full_turn - self.deadzone, 1e-6)
                frac = min(1.0, excess / span)
                mag = self.min_turn + frac * (self.turn_speed - self.min_turn)
            else:
                mag = self.turn_speed
            return 0.0, math.copysign(mag, self.bearing)

        if not self.linear_enabled:
            return 0.0, 0.0

        v = self._linear()
        if v > 0:
            return self.cruise_speed, 0.0
        if v < 0:
            return -self.reverse_speed, 0.0
        return 0.0, 0.0

    def _linear(self):
        """Forward speed from the distance error, 0 if we should hold."""
        if self.distance is None or math.isnan(self.distance):
            return 0.0

        # Swing toward the target before advancing, so the car does not
        # drive off along a stale heading while still turning.
        if abs(self.bearing) > self.turn_first:
            return 0.0

        if self.distance > 0:
            # Metres (Stage 5): positive error means too far away.
            error = self.distance - self.target_distance_m
            if abs(error) < self.dz_m:
                return 0.0
            v = self.kp_linear_metric * error
        else:
            # Box height in pixels: a SMALLER box means further away, so
            # the error is inverted relative to the metric case.
            box_h = -self.distance
            error = self.target_box_h - box_h
            if abs(error) < self.dz_px:
                return 0.0
            v = self.kp_linear * error

        v = max(-self.max_reverse, min(self.max_linear, v))
        if 0 < abs(v) < self.min_linear:
            v = math.copysign(self.min_linear, v)
        return v


def main():
    rclpy.init()
    node = FollowController()
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
