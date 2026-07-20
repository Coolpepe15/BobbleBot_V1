"""Keyboard teleop for BobbleBot, ported from the ROS 1
bobble_controllers/src/nodes/KeyboardControl script. Run this in its own
terminal (it needs an interactive TTY with raw-mode reads) - it must have
the active window focus while driving:

    ros2 run bobble_pi keyboard_control_node
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from bobble_interfaces.msg import ControlCommands

MSG = """
BobbleBot Keyboard Controller
---------------------------
Activate/Deactivate command:
    Activate/Shutdown: space bar
Moving around:
    Forward : w
    Backward : s
    Left : a
    Right : d
Speed Up/Down:
    15% Increase: q
    15% Decrease: e
CTRL-C to quit
"""

MOVE_BINDINGS = {
    'a': (0, 1),
    'w': (1, 0),
    's': (-1, 0),
    'd': (0, -1),
}

SPEED_BINDINGS = {
    'q': (1.15, 1.15),
    'e': (0.85, 0.85),
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = Node('keyboard_control_node')
    mode_cmd_pub = node.create_publisher(ControlCommands, 'bobble/bb_cmd', 1)
    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 1)

    settings = termios.tcgetattr(sys.stdin)
    drive_fwd = 0
    turn_rate = 0
    fwd_speed = 0.25
    turn_speed = 0.25
    active = False
    mode_cmd = ControlCommands()

    print(MSG)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == ' ':
                if active:
                    mode_cmd.idle_cmd = True
                    mode_cmd.startup_cmd = False
                    mode_cmd.diagnostic_cmd = False
                    print('Reset state. Hit space bar again to resume.')
                    mode_cmd_pub.publish(mode_cmd)
                    active = False
                else:
                    mode_cmd.idle_cmd = False
                    mode_cmd.startup_cmd = True
                    mode_cmd.diagnostic_cmd = False
                    print('BobbleBot Active! Hit space bar again to shutdown.')
                    mode_cmd_pub.publish(mode_cmd)
                    active = True
            if key in MOVE_BINDINGS:
                drive_fwd = MOVE_BINDINGS[key][0]
                turn_rate = MOVE_BINDINGS[key][1]
            elif key in SPEED_BINDINGS:
                fwd_speed *= SPEED_BINDINGS[key][0]
                turn_speed *= SPEED_BINDINGS[key][1]
            else:
                drive_fwd = 0.0
                turn_rate = 0.0
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = drive_fwd * fwd_speed
            twist.angular.z = turn_rate * turn_speed
            print(f'Forward Velocity Cmd : {twist.linear.x}')
            print(f'Turn Rate Cmd : {twist.angular.z}')
            cmd_vel_pub.publish(twist)
    except Exception as exc:  # noqa: BLE001 - mirrors the original script's catch-all
        print(exc)
    finally:
        cmd_vel_pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
