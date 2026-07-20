"""Publishes raw IMU data from an MPU6050 on the 'imu/data_raw' topic.

No orientation is computed here (orientation_covariance[0] is set to -1 to
signal "orientation not provided", per REP 145) - the balance controller
node consumes the raw angular velocity / linear acceleration directly and
runs its own complementary tilt filter, because it needs the IMU signal in
lock-step with its own control loop timing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from bobble_pi.mpu6050 import Mpu6050


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate_hz', 200.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('gyro_calibration_samples', 400)

        i2c_bus = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self._frame_id = self.get_parameter('frame_id').value
        calib_samples = self.get_parameter('gyro_calibration_samples').value

        self._imu = Mpu6050(bus_num=i2c_bus, address=i2c_address)

        self.get_logger().info(
            f'Calibrating MPU6050 gyro bias ({calib_samples} samples). '
            'Keep the robot completely still and level...'
        )
        bias = self._imu.calibrate_gyro(samples=calib_samples)
        self.get_logger().info(f'Gyro bias (rad/s): {bias}')

        self._pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_timer(self):
        (ax, ay, az), (gx, gy, gz) = self._imu.read()

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Orientation is not computed by this node.
        msg.orientation_covariance[0] = -1.0

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
