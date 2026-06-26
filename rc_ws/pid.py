"""pid.py — small PID with clamping, anti-windup, derivative-on-measurement."""

import math


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def angle_wrap_deg(deg):
    """Wrap to (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def euler_to_quaternion(roll, pitch, yaw):
    """radians -> (x, y, z, w)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class PID:
    def __init__(self, kp, ki, kd, out_min=-1e9, out_max=1e9,
                 i_limit=None, deriv_on_measurement=True):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.i_limit = i_limit
        self.deriv_on_measurement = deriv_on_measurement
        self.reset()

    def reset(self):
        self._i = 0.0
        self._prev_err = 0.0
        self._prev_meas = None

    def update(self, error, measurement, dt):
        p = self.kp * error
        d = 0.0
        if dt > 0:
            self._i += self.ki * error * dt
            if self.i_limit is not None:
                self._i = clamp(self._i, -self.i_limit, self.i_limit)
            if self.deriv_on_measurement and self._prev_meas is not None:
                d = -self.kd * (measurement - self._prev_meas) / dt
            else:
                d = self.kd * (error - self._prev_err) / dt
        out = p + self._i + d
        out_c = clamp(out, self.out_min, self.out_max)
        if self.ki != 0.0 and out != out_c:        # back-calc anti-windup
            self._i -= (out - out_c)
            if self.i_limit is not None:
                self._i = clamp(self._i, -self.i_limit, self.i_limit)
        self._prev_err = error
        self._prev_meas = measurement
        return out_c
