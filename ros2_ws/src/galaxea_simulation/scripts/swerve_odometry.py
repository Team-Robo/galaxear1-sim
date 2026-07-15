#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class SwerveOdometry(Node):

    MODULES = [
        ('steer_motor_joint1', 'wheel_motor_joint1', 0.21516,  0.28),
        ('steer_motor_joint2', 'wheel_motor_joint2', 0.21516, -0.28),
        ('steer_motor_joint3', 'wheel_motor_joint3', -0.28084, 0.0),
    ]

    def __init__(self):
        super().__init__('swerve_odometry')

        self.declare_parameter('wheel_radius', 0.07)
        self.declare_parameter('publish_rate', 50.0)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        rate = self.get_parameter('publish_rate').value

        n = len(self.MODULES)
        self.steer_angle = [0.0] * n
        self.wheel_velocity = [0.0] * n
        self._prev_wheel_pos = [None] * n
        self._prev_wheel_time = [None] * n

        # Fixed 6x3 kinematic Jacobian: module ground-velocity components (mx_i, my_i) as
        # a linear function of body twist (vx, vy, wz) — same geometry as the IK in
        # swerve_controller.py, inverted. Precomputed once since module offsets are fixed.
        rows = []
        for _, _, lx, ly in self.MODULES:
            rows.append([1.0, 0.0, -ly])
            rows.append([0.0, 1.0, lx])
        jacobian = np.array(rows)
        self._jacobian_pinv = np.linalg.pinv(jacobian)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)
        self.create_timer(1.0 / rate, self._publish_odom)

        self.get_logger().info(
            f'Swerve odometry ready (wheel_r={self.wheel_radius})')

    def _joint_states_cb(self, msg):
        has_velocity = len(msg.velocity) == len(msg.name)
        now = self.get_clock().now()
        for i, (steer_jnt, wheel_jnt, _, _) in enumerate(self.MODULES):
            if steer_jnt in msg.name:
                self.steer_angle[i] = msg.position[msg.name.index(steer_jnt)]

            if wheel_jnt not in msg.name:
                continue
            idx = msg.name.index(wheel_jnt)
            pos = msg.position[idx]

            if has_velocity:
                # Preferred path — Ignition's JointStatePublisher populates velocity[].
                self.wheel_velocity[i] = msg.velocity[idx]
            else:
                # Fallback — finite-difference position if velocity[] isn't published.
                # Safe for a continuous joint: Ignition reports unwrapped accumulated
                # position, so there's no 2*pi wraparound discontinuity to guard against.
                prev_pos = self._prev_wheel_pos[i]
                prev_time = self._prev_wheel_time[i]
                if prev_pos is not None:
                    dt = (now - prev_time).nanoseconds * 1e-9
                    if dt > 1e-6:
                        self.wheel_velocity[i] = (pos - prev_pos) / dt
                self._prev_wheel_pos[i] = pos
                self._prev_wheel_time[i] = now

    @staticmethod
    def _normalize(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        b = []
        for i in range(len(self.MODULES)):
            speed = self.wheel_radius * self.wheel_velocity[i]
            angle = self.steer_angle[i]
            b.append(speed * math.cos(angle))
            b.append(speed * math.sin(angle))

        vx, vy, wz = self._jacobian_pinv @ np.array(b)

        # Integrate using the heading at the start of this interval.
        theta = self.theta
        self.x += (vx * math.cos(theta) - vy * math.sin(theta)) * dt
        self.y += (vx * math.sin(theta) + vy * math.cos(theta)) * dt
        self.theta = self._normalize(theta + wz * dt)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.pose.covariance[0] = 0.01    # x
        odom.pose.covariance[7] = 0.01    # y
        odom.pose.covariance[35] = 0.02   # yaw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.02

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
