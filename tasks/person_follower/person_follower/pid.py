"""A small PID with the guards this application actually needs.

Two things matter more than the maths here:

  * Anti-windup. The car saturates at turn_speed, and the firmware ignores
    anything under its 0.05 deadband. Without clamping, the integral keeps
    growing while the output is pinned and then dumps that accumulated
    error the moment the error changes sign -- a violent swing in the
    wrong direction.

  * Reset. When the target is lost the error becomes meaningless. Carrying
    the integral and the previous error across a gap makes the car lurch
    when the person reappears, which is worse than having no PID at all.
"""


class PID:
    def __init__(self, kp, ki, kd, out_limit, integral_limit=None,
                 derivative_alpha=0.7):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = out_limit
        # Cap the integral itself, not just the output: it is the stored
        # term that causes the delayed swing.
        self.integral_limit = (
            integral_limit if integral_limit is not None else out_limit
        )
        # The bearing signal is noisy and dt is small, so a raw derivative
        # is mostly noise amplification. Low-pass it.
        self.derivative_alpha = derivative_alpha
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.derivative = 0.0

    def step(self, error, dt):
        if dt <= 0:
            return 0.0

        p = self.kp * error

        # Only integrate while inside the output range. Integrating during
        # saturation is exactly what winds the term up.
        tentative = p + self.ki * self.integral
        if abs(tentative) < self.out_limit:
            self.integral += error * dt
            self.integral = max(-self.integral_limit,
                                min(self.integral_limit, self.integral))
        i = self.ki * self.integral

        if self.prev_error is None:
            # First sample after a reset: no history, so no derivative.
            # Guessing one from a single point produces a large spurious
            # kick precisely when the target has just been reacquired.
            raw_d = 0.0
        else:
            raw_d = (error - self.prev_error) / dt
        a = self.derivative_alpha
        self.derivative = a * self.derivative + (1 - a) * raw_d
        d = self.kd * self.derivative

        self.prev_error = error

        out = p + i + d
        return max(-self.out_limit, min(self.out_limit, out))
