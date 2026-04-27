#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_ackermann")

        self.declare_parameter("wheel_base", 0.44)
        self.declare_parameter("track_width", 0.38)
        self.declare_parameter("wheel_radius", 0.07)
        self.declare_parameter("max_steer_angle", 0.6)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("publish_rate", 50.0)

        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.max_steer_angle = float(self.get_parameter("max_steer_angle").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.latest_linear_v = 0.0
        self.latest_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            "/front_steer_position_controller/commands",
            10
        )

        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            "/rear_wheel_velocity_controller/commands",
            10
        )

        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.create_timer(timer_period, self.publish_control_commands)

        self.get_logger().info(
            "cmd_vel_to_ackermann node started: "
            f"wheel_base={self.wheel_base}, "
            f"track_width={self.track_width}, "
            f"wheel_radius={self.wheel_radius}"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.latest_linear_v = msg.linear.x
        self.latest_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def calculate_ackermann(self, linear_v, angular_z):
        # Ackermann 车辆不能原地自转；linear.x 接近 0 时直接停车。
        if abs(linear_v) < 1e-4:
            return [0.0, 0.0], [0.0, 0.0]

        # 直行
        if abs(angular_z) < 1e-4:
            wheel_velocity = linear_v / self.wheel_radius
            return [0.0, 0.0], [wheel_velocity, wheel_velocity]

        # 曲率：k = omega / v
        curvature = angular_z / linear_v

        # 车辆中心转弯半径
        center_radius = abs(1.0 / curvature)

        # 避免转弯半径小于半轮距导致几何异常
        min_radius = self.track_width / 2.0 + 0.05
        center_radius = max(center_radius, min_radius)

        turn_sign = 1.0 if curvature > 0.0 else -1.0

        inner_radius = center_radius - self.track_width / 2.0
        outer_radius = center_radius + self.track_width / 2.0

        inner_angle = math.atan(self.wheel_base / inner_radius)
        outer_angle = math.atan(self.wheel_base / outer_radius)

        # ROS 坐标约定：angular.z > 0 为左转
        if turn_sign > 0.0:
            left_steer = inner_angle
            right_steer = outer_angle
        else:
            left_steer = -outer_angle
            right_steer = -inner_angle

        left_steer = self.clamp(left_steer, -self.max_steer_angle, self.max_steer_angle)
        right_steer = self.clamp(right_steer, -self.max_steer_angle, self.max_steer_angle)

        # 后轮左右线速度：左转时右后轮更快，右转时左后轮更快
        left_rear_linear_v = linear_v - angular_z * self.track_width / 2.0
        right_rear_linear_v = linear_v + angular_z * self.track_width / 2.0

        left_rear_wheel_velocity = left_rear_linear_v / self.wheel_radius
        right_rear_wheel_velocity = right_rear_linear_v / self.wheel_radius

        return [left_steer, right_steer], [left_rear_wheel_velocity, right_rear_wheel_velocity]

    def publish_control_commands(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.command_timeout:
            linear_v = 0.0
            angular_z = 0.0
        else:
            linear_v = self.latest_linear_v
            angular_z = self.latest_angular_z

        steer_values, wheel_values = self.calculate_ackermann(linear_v, angular_z)

        steer_msg = Float64MultiArray()
        steer_msg.data = steer_values

        wheel_msg = Float64MultiArray()
        wheel_msg.data = wheel_values

        self.steer_pub.publish(steer_msg)
        self.wheel_pub.publish(wheel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
