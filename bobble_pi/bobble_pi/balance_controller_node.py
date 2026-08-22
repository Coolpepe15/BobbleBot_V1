"""Cascade PID self-balance controller for the BobbleBot on a Raspberry Pi 4.

This is a Python port of the control logic in bobble_controllers'
BalanceBaseController.cpp (velocity -> desired tilt -> tilt effort cascade,
plus a turning-rate loop), adapted for real hardware:

  * Orientation comes from a complementary filter (see tilt_estimator.py)
    instead of the Madgwick+quaternion pipeline the original used, because
    that pipeline baked in axis assumptions specific to one simulated IMU
    mounting that we cannot safely assume for your physical build.
  * Wheel velocity feedback is optional (`use_wheel_odometry`). Without
    encoders wired up, the velocity PID cascade is bypassed in favor of a
    simple open-loop tilt bias driven directly by the commanded velocity -
    the robot will still balance and drive, but won't hold a precise speed.
  * Runs as a single ROS 2 timer callback instead of a hard real-time
    ros_control loop. Default 100 Hz is realistic for CPython on a Pi 4;
    the original ran at 500 Hz inside a real-time control loop in C++.

See config/balance_control.yaml for every tunable parameter and the
top-level README for the axis calibration procedure.
"""

import math
import time
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

from bobble_interfaces.msg import BobbleBotStatus, ControlCommands
from bobble_pi.difference_filter import LowPassFilter
from bobble_pi.motor_driver import L298nMotor, WheelEncoder
from bobble_pi.pid_control import PidControl
from bobble_pi.tilt_estimator import ComplementaryFilter, accel_tilt_angle, select_axis

IDLE = 0
STARTUP = 1
BALANCE = 2
DIAGNOSTIC = 3


def clamp(value, limit):
    if value < -limit:
        return -limit
    if value > limit:
        return limit
    return value


class BalanceControllerNode(Node):
    def __init__(self):
        super().__init__('balance_controller_node')

        self._reset_state()
        self._load_config()
        self._setup_filters()
        self._setup_controllers()
        self._setup_hardware()

        self._latest_imu = None
        self._last_control_time = None
        self._tilt_filter = ComplementaryFilter(self.p.complementary_filter_alpha)
        self._heading = 0.0  # telemetry only - drifts, no magnetometer on the MPU6050

        self._received_cmds = {'startup': False, 'idle': False, 'diagnostic': False}
        self._received_vel = {'linear_x': 0.0, 'angular_z': 0.0}

        self._status_pub = self.create_publisher(BobbleBotStatus, 'bobble/bb_controller_status', 10)
        self.create_subscription(Imu, 'imu/data_raw', self._imu_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(ControlCommands, 'bobble/bb_cmd', self._bb_cmd_callback, 10)

        self._timer = self.create_timer(1.0 / self.p.control_loop_frequency, self._control_loop)
        self.get_logger().info(
            'Balance controller ready (mode: IDLE). Publish ControlCommands.startup_cmd=true '
            'on bobble/bb_cmd to begin.'
        )

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _load_config(self):
        p = SimpleNamespace()
        p.control_loop_frequency = self._param('control_loop_frequency_hz', 100.0)
        p.starting_tilt_safety_limit_deg = self._param('starting_tilt_safety_limit_deg', 4.0)
        p.max_tilt_safety_limit_deg = self._param('max_tilt_safety_limit_deg', 20.0)
        p.controller_effort_max = self._param('controller_effort_max', 0.4)
        p.wheel_velocity_adjustment = self._param('wheel_velocity_adjustment', 1.0)

        p.measured_tilt_filter_hz = self._param('measured_tilt_filter_hz', 100.0)
        p.measured_tilt_dot_filter_hz = self._param('measured_tilt_dot_filter_hz', 100.0)
        p.measured_turn_rate_filter_hz = self._param('measured_turn_rate_filter_hz', 50.0)
        p.left_wheel_velocity_filter_hz = self._param('left_wheel_velocity_filter_hz', 10.0)
        p.right_wheel_velocity_filter_hz = self._param('right_wheel_velocity_filter_hz', 10.0)
        p.desired_forward_velocity_filter_hz = self._param('desired_forward_velocity_filter_hz', 10.0)
        p.desired_turn_rate_filter_hz = self._param('desired_turn_rate_filter_hz', 50.0)

        p.max_velocity_cmd = self._param('max_velocity_cmd', 0.5)
        p.max_turn_rate_cmd = self._param('max_turn_rate_cmd', 2.0)
        p.wheel_radius_m = self._param('wheel_radius_m', 0.05)
        p.velocity_cmd_scale = self._param('velocity_cmd_scale', 1.0)
        p.turn_cmd_scale = self._param('turn_cmd_scale', 1.0)

        p.velocity_control_kp = self._param('velocity_control_kp', 0.125)
        p.velocity_control_kd = self._param('velocity_control_kd', 0.05)
        p.velocity_control_deriv_filter_hz = self._param('velocity_control_deriv_filter_hz', 20.0)
        p.velocity_control_ki = self._param('velocity_control_ki', 0.01)
        p.velocity_control_alpha_filter = self._param('velocity_control_alpha_filter', 0.99)
        p.velocity_control_max_integral_output = self._param('velocity_control_max_integral_output', 0.025)
        p.velocity_control_output_limit_deg = self._param('velocity_control_output_limit_deg', 8.5)

        p.tilt_control_kp = self._param('tilt_control_kp', 2.5)
        p.tilt_control_kd = self._param('tilt_control_kd', 0.5)
        p.tilt_control_alpha_filter = self._param('tilt_control_alpha_filter', 0.05)

        p.turning_control_kp = self._param('turning_control_kp', 0.25)
        p.turning_control_ki = self._param('turning_control_ki', 0.05)
        p.turning_control_kd = self._param('turning_control_kd', 0.0)
        p.turning_control_deriv_filter_hz = self._param('turning_control_deriv_filter_hz', 50.0)
        p.turning_output_filter = self._param('turning_output_filter', 0.5)

        p.complementary_filter_alpha = self._param('complementary_filter_alpha', 0.98)
        p.accel_fall_axis = self._param('accel_fall_axis', 'x')
        p.accel_fall_sign = self._param('accel_fall_sign', 1.0)
        p.accel_vertical_axis = self._param('accel_vertical_axis', 'z')
        p.accel_vertical_sign = self._param('accel_vertical_sign', 1.0)
        p.gyro_tilt_axis = self._param('gyro_tilt_axis', 'y')
        p.gyro_tilt_sign = self._param('gyro_tilt_sign', 1.0)
        p.gyro_yaw_axis = self._param('gyro_yaw_axis', 'z')
        p.gyro_yaw_sign = self._param('gyro_yaw_sign', 1.0)
        p.tilt_offset_deg = self._param('tilt_offset_deg', 0.0)

        p.use_wheel_odometry = self._param('use_wheel_odometry', False)
        p.pulses_per_revolution = self._param('pulses_per_revolution', 780.0)
        p.left_encoder_pin_a = self._param('left_encoder_pin_a', -1)
        p.left_encoder_pin_b = self._param('left_encoder_pin_b', -1)
        p.right_encoder_pin_a = self._param('right_encoder_pin_a', -1)
        p.right_encoder_pin_b = self._param('right_encoder_pin_b', -1)
        p.left_encoder_invert = self._param('left_encoder_invert', False)
        p.right_encoder_invert = self._param('right_encoder_invert', False)

        p.open_loop_tilt_bias_scale = self._param('open_loop_tilt_bias_scale', 0.05)
        p.open_loop_max_tilt_bias_deg = self._param('open_loop_max_tilt_bias_deg', 6.0)

        p.left_motor_in1_pin = self._param('left_motor_in1_pin', 5)
        p.left_motor_in2_pin = self._param('left_motor_in2_pin', 6)
        p.left_motor_pwm_pin = self._param('left_motor_pwm_pin', 12)
        p.left_motor_invert = self._param('left_motor_invert', False)
        p.right_motor_in1_pin = self._param('right_motor_in1_pin', 20)
        p.right_motor_in2_pin = self._param('right_motor_in2_pin', 21)
        p.right_motor_pwm_pin = self._param('right_motor_pwm_pin', 13)
        p.right_motor_invert = self._param('right_motor_invert', False)
        p.pwm_frequency_hz = self._param('pwm_frequency_hz', 1000)

        self.p = p

    def _setup_filters(self):
        p = self.p
        ts = 1.0 / p.control_loop_frequency
        self.filters = SimpleNamespace(
            measured_tilt=LowPassFilter(ts, p.measured_tilt_filter_hz, 1.0),
            measured_tilt_dot=LowPassFilter(ts, p.measured_tilt_dot_filter_hz, 1.0),
            measured_turn_rate=LowPassFilter(ts, p.measured_turn_rate_filter_hz, 1.0),
            left_wheel_velocity=LowPassFilter(ts, p.left_wheel_velocity_filter_hz, 1.0),
            right_wheel_velocity=LowPassFilter(ts, p.right_wheel_velocity_filter_hz, 1.0),
            desired_forward_velocity=LowPassFilter(ts, p.desired_forward_velocity_filter_hz, 1.0),
            desired_turn_rate=LowPassFilter(ts, p.desired_turn_rate_filter_hz, 1.0),
        )

    def _setup_controllers(self):
        p = self.p
        ts = 1.0 / p.control_loop_frequency

        self.velocity_pid = PidControl()
        self.velocity_pid.set_pid(
            p.velocity_control_kp, p.velocity_control_ki, p.velocity_control_kd,
            p.velocity_control_deriv_filter_hz, ts,
        )
        self.velocity_pid.set_output_filter(p.velocity_control_alpha_filter)
        self.velocity_pid.set_max_i_output(p.velocity_control_max_integral_output)
        self.velocity_pid.set_output_limits(
            -math.radians(p.velocity_control_output_limit_deg),
            math.radians(p.velocity_control_output_limit_deg),
        )
        self.velocity_pid.set_direction(True)

        self.tilt_pid = PidControl()
        self.tilt_pid.set_pid(p.tilt_control_kp, 0.0, p.tilt_control_kd, 0.0, ts)
        self.tilt_pid.set_external_derivative_error(lambda: self.state.tilt_dot)
        self.tilt_pid.set_output_filter(p.tilt_control_alpha_filter)
        self.tilt_pid.set_output_limits(-p.controller_effort_max, p.controller_effort_max)
        self.tilt_pid.set_direction(True)
        self.tilt_pid.set_setpoint_range(math.radians(20.0))

        self.turning_pid = PidControl()
        self.turning_pid.set_pid(
            p.turning_control_kp, p.turning_control_ki, p.turning_control_kd,
            p.turning_control_deriv_filter_hz, ts,
        )
        self.turning_pid.set_output_filter(p.turning_output_filter)
        self.turning_pid.set_max_i_output(1.0)
        self.turning_pid.set_output_limits(-p.controller_effort_max / 2.0, p.controller_effort_max / 2.0)
        self.turning_pid.set_direction(True)
        self.turning_pid.set_setpoint_range(math.radians(45.0))

    def _setup_hardware(self):
        p = self.p
        self.left_motor = L298nMotor(
            p.left_motor_in1_pin, p.left_motor_in2_pin, p.left_motor_pwm_pin,
            pwm_frequency=p.pwm_frequency_hz, invert=p.left_motor_invert,
        )
        self.right_motor = L298nMotor(
            p.right_motor_in1_pin, p.right_motor_in2_pin, p.right_motor_pwm_pin,
            pwm_frequency=p.pwm_frequency_hz, invert=p.right_motor_invert,
        )

        self.left_encoder = None
        self.right_encoder = None
        if p.use_wheel_odometry:
            if p.left_encoder_pin_a < 0 or p.right_encoder_pin_a < 0:
                self.get_logger().error(
                    'use_wheel_odometry is true but left_encoder_pin_a / '
                    'right_encoder_pin_a are not set; falling back to open-loop '
                    'tilt-bias driving. Set the encoder pin parameters to enable it.'
                )
                p.use_wheel_odometry = False
            else:
                chan_b_left = p.left_encoder_pin_b if p.left_encoder_pin_b >= 0 else None
                chan_b_right = p.right_encoder_pin_b if p.right_encoder_pin_b >= 0 else None
                self.left_encoder = WheelEncoder(
                    p.left_encoder_pin_a, p.pulses_per_revolution,
                    channel_b_pin=chan_b_left, invert=p.left_encoder_invert,
                )
                self.right_encoder = WheelEncoder(
                    p.right_encoder_pin_a, p.pulses_per_revolution,
                    channel_b_pin=chan_b_right, invert=p.right_encoder_invert,
                )

    def _reset_state(self):
        self.state = SimpleNamespace(
            active_control_mode=IDLE,
            measured_tilt=0.0, measured_tilt_dot=0.0, measured_turn_rate=0.0,
            tilt=0.0, tilt_dot=0.0, turn_rate=0.0,
            desired_tilt=0.0, forward_velocity=0.0,
            left_wheel_velocity=0.0, right_wheel_velocity=0.0,
            measured_left_motor_position=0.0, measured_left_motor_velocity=0.0,
            measured_right_motor_position=0.0, measured_right_motor_velocity=0.0,
            cmds=SimpleNamespace(
                startup_cmd=False, idle_cmd=False, diagnostic_cmd=False,
                desired_velocity_raw=0.0, desired_turn_rate_raw=0.0,
                desired_velocity=0.0, desired_turn_rate=0.0,
            ),
        )
        self.outputs = SimpleNamespace(
            tilt_effort=0.0, heading_effort=0.0,
            left_motor_effort_cmd=0.0, right_motor_effort_cmd=0.0,
        )

    # ------------------------------------------------------------------
    # Subscriptions (single-threaded executor: no locking needed, these
    # run interleaved with the control timer on the same thread)
    # ------------------------------------------------------------------
    def _imu_callback(self, msg):
        self._latest_imu = (
            (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
        )

    def _cmd_vel_callback(self, msg):
        self._received_vel['linear_x'] = msg.linear.x
        self._received_vel['angular_z'] = msg.angular.z

    def _bb_cmd_callback(self, msg):
        self._received_cmds['startup'] = msg.startup_cmd
        self._received_cmds['idle'] = msg.idle_cmd
        self._received_cmds['diagnostic'] = msg.diagnostic_cmd

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _control_loop(self):
        self._populate_commands()
        self._estimate_state()
        self._apply_filters()
        self._run_state_logic()

        self.outputs.right_motor_effort_cmd = -self.outputs.tilt_effort + self.outputs.heading_effort
        self.outputs.left_motor_effort_cmd = -self.outputs.tilt_effort - self.outputs.heading_effort

        self._apply_safety()
        self._send_motor_commands()
        self._publish_status()

    def _populate_commands(self):
        cmds = self.state.cmds
        cmds.startup_cmd = self._received_cmds['startup']
        cmds.idle_cmd = self._received_cmds['idle']
        cmds.diagnostic_cmd = self._received_cmds['diagnostic']
        cmds.desired_velocity_raw = self._received_vel['linear_x'] * self.p.velocity_cmd_scale
        cmds.desired_turn_rate_raw = self._received_vel['angular_z'] * self.p.turn_cmd_scale

    def _estimate_state(self):
        now = time.monotonic()
        dt = (1.0 / self.p.control_loop_frequency) if self._last_control_time is None else now - self._last_control_time
        self._last_control_time = now

        if self._latest_imu is not None:
            p = self.p
            accel, gyro = self._latest_imu
            fall = select_axis(accel, p.accel_fall_axis, p.accel_fall_sign)
            vertical = select_axis(accel, p.accel_vertical_axis, p.accel_vertical_sign)
            accel_angle = accel_tilt_angle(fall, vertical)
            tilt_rate = select_axis(gyro, p.gyro_tilt_axis, p.gyro_tilt_sign)
            yaw_rate = select_axis(gyro, p.gyro_yaw_axis, p.gyro_yaw_sign)

            raw_tilt = self._tilt_filter.update(accel_angle, tilt_rate, dt)
            trimmed_tilt = raw_tilt - math.radians(p.tilt_offset_deg)
            self.state.measured_tilt = math.atan2(math.sin(trimmed_tilt), math.cos(trimmed_tilt))
            self.state.measured_tilt_dot = tilt_rate
            self.state.measured_turn_rate = yaw_rate
            self._heading += yaw_rate * dt

        if self.p.use_wheel_odometry and self.left_encoder is not None:
            left_vel = self.left_encoder.read_velocity_rad_s(
                assumed_direction=self.outputs.left_motor_effort_cmd)
            right_vel = self.right_encoder.read_velocity_rad_s(
                assumed_direction=self.outputs.right_motor_effort_cmd)
            self.state.measured_left_motor_velocity = left_vel
            self.state.measured_right_motor_velocity = right_vel
            self.state.measured_left_motor_position += left_vel * dt
            self.state.measured_right_motor_position += right_vel * dt

    def _apply_filters(self):
        p = self.p
        f = self.filters
        s = self.state

        s.tilt = f.measured_tilt.filter(s.measured_tilt)
        s.tilt_dot = f.measured_tilt_dot.filter(s.measured_tilt_dot)
        s.turn_rate = f.measured_turn_rate.filter(s.measured_turn_rate)

        if p.use_wheel_odometry:
            s.left_wheel_velocity = (
                f.left_wheel_velocity.filter(s.measured_left_motor_velocity) * p.wheel_velocity_adjustment
            )
            s.right_wheel_velocity = (
                f.right_wheel_velocity.filter(s.measured_right_motor_velocity) * p.wheel_velocity_adjustment
            )
            s.forward_velocity = p.wheel_radius_m * (s.right_wheel_velocity + s.left_wheel_velocity) / 2.0

        s.cmds.desired_velocity = f.desired_forward_velocity.filter(s.cmds.desired_velocity_raw)
        s.cmds.desired_turn_rate = f.desired_turn_rate.filter(s.cmds.desired_turn_rate_raw)
        s.cmds.desired_velocity = clamp(s.cmds.desired_velocity, p.max_velocity_cmd)
        s.cmds.desired_turn_rate = clamp(s.cmds.desired_turn_rate, p.max_turn_rate_cmd)

    def _apply_safety(self):
        p = self.p
        if abs(self.state.tilt) >= math.radians(p.max_tilt_safety_limit_deg):
            self.outputs.right_motor_effort_cmd = 0.0
            self.outputs.left_motor_effort_cmd = 0.0
        self.outputs.right_motor_effort_cmd = clamp(self.outputs.right_motor_effort_cmd, p.controller_effort_max)
        self.outputs.left_motor_effort_cmd = clamp(self.outputs.left_motor_effort_cmd, p.controller_effort_max)

    def _run_state_logic(self):
        mode = self.state.active_control_mode
        if mode == IDLE:
            self._idle_mode()
        elif mode == DIAGNOSTIC:
            self._diagnostic_mode()
        elif mode == STARTUP:
            self._startup_mode()
        elif mode == BALANCE:
            self._balance_mode()
        else:
            self.outputs.tilt_effort = 0.0
            self.outputs.heading_effort = 0.0

    def _idle_mode(self):
        self.velocity_pid.reset()
        self.tilt_pid.reset()
        self.turning_pid.reset()
        self.state.desired_tilt = 0.0
        self.outputs.tilt_effort = 0.0
        self.outputs.heading_effort = 0.0
        if self.state.cmds.startup_cmd:
            self.state.active_control_mode = STARTUP
        if self.state.cmds.diagnostic_cmd:
            self.state.active_control_mode = DIAGNOSTIC

    def _diagnostic_mode(self):
        # Raw pass-through: DesiredVelocity/DesiredTurnRate drive the motors
        # directly as effort commands, bypassing balance control entirely.
        # Useful for confirming wiring/polarity with the wheels off the
        # ground before ever attempting to balance.
        self.outputs.tilt_effort = self.state.cmds.desired_velocity
        self.outputs.heading_effort = self.state.cmds.desired_turn_rate
        if self.state.cmds.idle_cmd:
            self.state.active_control_mode = IDLE

    def _startup_mode(self):
        p = self.p
        if abs(self.state.tilt) >= math.radians(p.starting_tilt_safety_limit_deg):
            self.outputs.tilt_effort = 0.0
            self.outputs.heading_effort = 0.0
        else:
            self.state.active_control_mode = BALANCE

    def _balance_mode(self):
        p = self.p
        if p.use_wheel_odometry:
            self.state.desired_tilt = self.velocity_pid.get_output(
                self.state.cmds.desired_velocity, self.state.forward_velocity
            )
        else:
            self.state.desired_tilt = clamp(
                self.state.cmds.desired_velocity * p.open_loop_tilt_bias_scale,
                math.radians(p.open_loop_max_tilt_bias_deg),
            )
        self.outputs.tilt_effort = self.tilt_pid.get_output(self.state.desired_tilt, self.state.tilt)
        self.outputs.heading_effort = self.turning_pid.get_output(
            self.state.cmds.desired_turn_rate, self.state.turn_rate
        )
        if self.state.cmds.idle_cmd:
            self.state.active_control_mode = IDLE

    def _send_motor_commands(self):
        self.left_motor.set_effort(self.outputs.left_motor_effort_cmd)
        self.right_motor.set_effort(self.outputs.right_motor_effort_cmd)

    def _publish_status(self):
        s = self.state
        o = self.outputs
        msg = BobbleBotStatus()
        msg.control_mode = s.active_control_mode
        msg.measured_tilt_dot = math.degrees(s.measured_tilt_dot)
        msg.measured_turn_rate = math.degrees(s.measured_turn_rate)
        msg.filtered_tilt_dot = math.degrees(s.tilt_dot)
        msg.filtered_turn_rate = math.degrees(s.turn_rate)
        msg.tilt = math.degrees(s.tilt)
        msg.tilt_rate = math.degrees(s.tilt_dot)
        msg.heading = math.degrees(self._heading)
        msg.turn_rate = math.degrees(s.turn_rate)
        msg.forward_velocity = s.forward_velocity
        msg.desired_velocity = s.cmds.desired_velocity
        msg.desired_tilt = math.degrees(s.desired_tilt)
        msg.desired_turn_rate = math.degrees(s.cmds.desired_turn_rate)
        msg.left_motor_position = math.degrees(s.measured_left_motor_position)
        msg.left_motor_velocity = math.degrees(s.measured_left_motor_velocity)
        msg.right_motor_position = math.degrees(s.measured_right_motor_position)
        msg.right_motor_velocity = math.degrees(s.measured_right_motor_velocity)
        msg.tilt_effort = o.tilt_effort
        msg.heading_effort = o.heading_effort
        msg.left_motor_effort_cmd = o.left_motor_effort_cmd
        msg.right_motor_effort_cmd = o.right_motor_effort_cmd
        self._status_pub.publish(msg)

    def shutdown_hardware(self):
        self.left_motor.close()
        self.right_motor.close()
        if self.left_encoder is not None:
            self.left_encoder.close()
        if self.right_encoder is not None:
            self.right_encoder.close()


def main(args=None):
    rclpy.init(args=args)
    node = BalanceControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hardware()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
