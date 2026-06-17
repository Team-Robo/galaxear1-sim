#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class SwerveController(Node):

    MODULES = [
        ('steer_motor_joint1', 'wheel_motor_joint1', 0.21516,  0.28),
        ('steer_motor_joint2', 'wheel_motor_joint2', 0.21516, -0.28),
        ('steer_motor_joint3', 'wheel_motor_joint3', -0.28084, 0.0),
    ]

    def __init__(self):
        super().__init__('swerve_controller')

        self.declare_parameter('wheel_radius', 0.025)
        self.declare_parameter('deadband', 0.001)
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_angular_accel', 1.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.deadband = self.get_parameter('deadband').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.max_lin_accel = self.get_parameter('max_linear_accel').value
        self.max_ang_accel = self.get_parameter('max_angular_accel').value
        rate = self.get_parameter('rate').value
        self.dt = 1.0 / rate

        self.target_cmd = Twist()
        self.ramped = [0.0, 0.0, 0.0]  # vx, vy, wz
        self.last_cmd_time = self.get_clock().now()
        self.current_steer = [0.0] * len(self.MODULES)

        self.steer_pubs = []
        self.wheel_pubs = []
        for steer_jnt, wheel_jnt, _, _ in self.MODULES:
            self.steer_pubs.append(
                self.create_publisher(
                    Float64,
                    f'/model/r1/joint/{steer_jnt}/cmd_pos', 10))
            self.wheel_pubs.append(
                self.create_publisher(
                    Float64,
                    f'/model/r1/joint/{wheel_jnt}/cmd_vel', 10))

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)
        self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'Swerve controller ready (wheel_r={self.wheel_radius}, '
            f'max_lin_accel={self.max_lin_accel})')

    def _cmd_vel_cb(self, msg):
        self.target_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _joint_states_cb(self, msg):
        for i, (steer_jnt, _, _, _) in enumerate(self.MODULES):
            if steer_jnt in msg.name:
                self.current_steer[i] = msg.position[
                    msg.name.index(steer_jnt)]

    @staticmethod
    def _normalize(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _ramp(self, current, target, max_step):
        diff = target - current
        return current + max(-max_step, min(max_step, diff))

    def _control_loop(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.target_cmd = Twist()

        lin_step = self.max_lin_accel * self.dt
        ang_step = self.max_ang_accel * self.dt
        self.ramped[0] = self._ramp(self.ramped[0], self.target_cmd.linear.x, lin_step)
        self.ramped[1] = self._ramp(self.ramped[1], self.target_cmd.linear.y, lin_step)
        self.ramped[2] = self._ramp(self.ramped[2], self.target_cmd.angular.z, ang_step)

        vx, vy, wz = self.ramped

        for i, (_, _, lx, ly) in enumerate(self.MODULES):
            mx = vx - wz * ly
            my = vy + wz * lx
            speed = math.hypot(mx, my)

            if speed < self.deadband:
                angle = self.current_steer[i]
                wheel_vel = 0.0
            else:
                angle = math.atan2(my, mx)
                wheel_vel = speed / self.wheel_radius

                delta = self._normalize(angle - self.current_steer[i])
                if abs(delta) > math.pi / 2.0:
                    angle = self._normalize(angle + math.pi)
                    wheel_vel = -wheel_vel




            s = Float64()
            s.data = angle
            self.steer_pubs[i].publish(s)

            w = Float64()
            w.data = wheel_vel
            self.wheel_pubs[i].publish(w)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
