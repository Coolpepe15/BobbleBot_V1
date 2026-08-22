"""Complementary filter tilt estimation for a two-wheeled balancing robot.

The original bobble_controllers project used a full Madgwick AHRS filter
plus a fixed quaternion-to-Euler axis remap that was tuned for one specific
IMU mounting orientation in the Gazebo simulation. Because we don't know how
the MPU6050 will physically be mounted on your chassis, that fixed remap
can't be ported safely - getting it wrong on a real balancing robot means it
falls over immediately. A complementary filter needs only one accelerometer
plane and one gyro axis, and the axis/sign mapping is exposed as ROS
parameters (see config/balance_control.yaml) so it can be calibrated by
observation instead of guessed. See the README calibration section.
"""

import math


class ComplementaryFilter:
    """angle = alpha*(angle + gyro_rate*dt) + (1-alpha)*accel_angle"""

    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.angle = 0.0
        self._initialized = False

    def update(self, accel_angle, gyro_rate, dt):
        if not self._initialized or dt <= 0.0:
            self.angle = accel_angle
            self._initialized = True
            return self.angle
        gyro_estimate = self.angle + gyro_rate * dt
        # accel_angle comes from atan2() and is always wrapped to (-pi, pi].
        # gyro_estimate is an unwrapped running integral and can end up on the
        # opposite side of the +-pi branch cut (e.g. gyro_estimate=+3.10 rad,
        # accel_angle=-3.10 rad - physically 0.08 rad apart, numerically far
        # apart). Blending them directly would pull the estimate toward the
        # wrong side. Instead, blend the shortest wrapped angular error so the
        # correction always moves toward the accelerometer along the short way
        # around, then re-wrap the result so self.angle never drifts outside
        # (-pi, pi] regardless of which direction the robot fell.
        error = math.atan2(
            math.sin(accel_angle - gyro_estimate), math.cos(accel_angle - gyro_estimate)
        )
        angle = gyro_estimate + (1.0 - self.alpha) * error
        self.angle = math.atan2(math.sin(angle), math.cos(angle))
        return self.angle

    def reset(self, angle=0.0):
        self.angle = angle
        self._initialized = True


def accel_tilt_angle(accel_fall_axis, accel_vertical_axis):
    """Tilt angle (rad) of the body from vertical, computed from the
    accelerometer axis that reads ~0 when upright and grows as the robot
    tips over (accel_fall_axis) and the axis that reads ~+-g when upright
    (accel_vertical_axis)."""
    return math.atan2(accel_fall_axis, accel_vertical_axis)


_AXES = ('x', 'y', 'z')


def select_axis(vector_xyz, axis_name, sign=1.0):
    """vector_xyz: (x, y, z) tuple. axis_name: 'x'/'y'/'z'. Returns the
    signed component."""
    if axis_name not in _AXES:
        raise ValueError(f"axis must be one of {_AXES}, got {axis_name!r}")
    return sign * vector_xyz[_AXES.index(axis_name)]
