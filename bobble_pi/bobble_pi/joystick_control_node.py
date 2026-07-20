"""Joystick teleop for BobbleBot, ported from the ROS 1
bobble_controllers/src/nodes/JoystickControl script. Requires the ROS 2
'joy' package to publish sensor_msgs/Joy:

    ros2 run joy joy_node
    ros2 run bobble_pi joystick_control_node

Default axis/button indices match an Xbox One controller under Linux;
adjust the parameters below if yours differs.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from bobble_interfaces.msg import ControlCommands

JOYSTICK_MIN = -1.0
JOYSTICK_MAX = 1.0
JOYSTICK_RANGE = JOYSTICK_MAX - JOYSTICK_MIN


class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')

        self.declare_parameter('fwd_vel_axis', 1)
        self.declare_parameter('turn_rate_axis', 3)
        self.declare_parameter('activate_button', 0)
        self.declare_parameter('reset_button', 4)
        self.declare_parameter('output_min', -0.2)
        self.declare_parameter('output_max', 0.2)

        self._fwd_axis = self.get_parameter('fwd_vel_axis').value
        self._turn_axis = self.get_parameter('turn_rate_axis').value
        self._activate_button = self.get_parameter('activate_button').value
        self._reset_button = self.get_parameter('reset_button').value
        self._output_min = self.get_parameter('output_min').value
        self._output_max = self.get_parameter('output_max').value
        self._output_range = self._output_max - self._output_min

        self.create_subscription(Joy, 'joy', self._joystick_callback, 10)
        self._mode_cmd_pub = self.create_publisher(ControlCommands, 'bobble/bb_cmd', 1)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self._mode_cmd = ControlCommands()
        self._cmd_vel = Twist()

    def _remap_joystick_range(self, axis_value):
        return (((axis_value - JOYSTICK_MIN) * self._output_range) / JOYSTICK_RANGE) + self._output_min

    def _joystick_callback(self, joy_msg):
        for axis_number, axis_value in enumerate(joy_msg.axes):
            if axis_number == self._fwd_axis:
                self._cmd_vel.linear.x = self._remap_joystick_range(axis_value)
            elif axis_number == self._turn_axis:
                self._cmd_vel.angular.z = self._remap_joystick_range(axis_value)

        for button_number, button_value in enumerate(joy_msg.buttons):
            if button_number == self._activate_button and button_value:
                self._mode_cmd.idle_cmd = False
                self._mode_cmd.startup_cmd = True
                self._mode_cmd.diagnostic_cmd = False
                self._mode_cmd_pub.publish(self._mode_cmd)
                self.get_logger().info('BobbleBot Active!')
            elif button_number == self._reset_button and button_value:
                self._mode_cmd.idle_cmd = True
                self._mode_cmd.startup_cmd = False
                self._mode_cmd.diagnostic_cmd = False
                self._mode_cmd_pub.publish(self._mode_cmd)
                self.get_logger().info('Resetting BobbleBot')

        self._cmd_vel_pub.publish(self._cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
