#!/usr/bin/env python3
"""Converts Gazebo's sensor_msgs/Imu into the real robot's hdas_msg/Imu contract, so
/hdas/imu_chassis matches the real HDAS driver's topic name and message type exactly
(see nav2_bringup_plan.md Part 2). ros_gz_bridge can only produce sensor_msgs/Imu from
Gazebo's IMU plugin, so this conversion has to happen in a separate node."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from hdas_msg.msg import Imu as HdasImu


def _quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuHdasTranslator(Node):

    def __init__(self):
        super().__init__('imu_hdas_translator')
        self.pub = self.create_publisher(HdasImu, '/hdas/imu_chassis', 10)
        self.create_subscription(
            Imu, '/_gz/imu_chassis_raw', self._imu_cb, 10)

    def _imu_cb(self, msg):
        out = HdasImu()
        out.header = msg.header
        q = msg.orientation
        out.roll, out.pitch, out.yaw = _quaternion_to_euler(q.x, q.y, q.z, q.w)
        out.groy_x = msg.angular_velocity.x
        out.groy_y = msg.angular_velocity.y
        out.groy_z = msg.angular_velocity.z
        out.acc_x = msg.linear_acceleration.x
        out.acc_y = msg.linear_acceleration.y
        out.acc_z = msg.linear_acceleration.z
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuHdasTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
