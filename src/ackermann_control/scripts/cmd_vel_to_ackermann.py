#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelToAckermann(Node):
    """Convert /cmd_vel Twist commands into ros2_control controller commands."""

    def __init__(self):
        super().__init__("cmd_vel_to_ackermann")

        # 车辆几何参数。这里的默认值要尽量和 xacro 模型里的尺寸保持一致，
        # 否则仿真车的转向几何会和控制计算不匹配。
        self.declare_parameter("wheel_base", 0.44)
        self.declare_parameter("track_width", 0.38)
        self.declare_parameter("wheel_radius", 0.07)
        self.declare_parameter("max_steer_angle", 0.6)

        # command_timeout 用来做安全停车：超过这个时间没收到 /cmd_vel，就发布 0 命令。
        # publish_rate 是本节点向两个控制器重复发布命令的频率。
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("publish_rate", 50.0)

        # 从 ROS 参数服务器读取参数值。launch 文件里可以覆盖这些默认值。
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.max_steer_angle = float(self.get_parameter("max_steer_angle").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        # 保存最近一次收到的 /cmd_vel。定时器会按固定频率把它转换并发布出去。
        self.latest_linear_v = 0.0
        self.latest_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()

        # 前轮转向位置控制器的命令话题。
        # data[0] -> front_left_steer_joint
        # data[1] -> front_right_steer_joint
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            "/front_steer_position_controller/commands",
            10
        )

        # 后轮速度控制器的命令话题。
        # data[0] -> rear_left_wheel_joint
        # data[1] -> rear_right_wheel_joint
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            "/rear_wheel_velocity_controller/commands",
            10
        )

        # 上层通常发布 geometry_msgs/Twist 到 /cmd_vel：
        # linear.x 表示期望前进速度，angular.z 表示期望绕 z 轴转向角速度。
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # 不在 callback 里直接发布控制命令，而是用定时器按固定频率发布。
        # 这样即使 /cmd_vel 低频或短暂停顿，控制器也能持续收到最近一次命令。
        timer_period = 1.0 / self.publish_rate
        self.create_timer(timer_period, self.publish_control_commands)

        self.get_logger().info(
            "cmd_vel_to_ackermann node started: "
            f"wheel_base={self.wheel_base}, "
            f"track_width={self.track_width}, "
            f"wheel_radius={self.wheel_radius}"
        )

    def cmd_vel_callback(self, msg: Twist):
        # 这里只缓存最新速度命令，真正的 Ackermann 计算由定时器函数周期执行。
        self.latest_linear_v = msg.linear.x
        self.latest_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def clamp(self, value, min_value, max_value):
        # 把数值限制在 [min_value, max_value]，用于限制最大转角。
        return max(min_value, min(max_value, value))

    def calculate_ackermann(self, linear_v, angular_z):
        # 输入：
        #   linear_v  -> 车辆中心期望线速度，来自 /cmd_vel.linear.x
        #   angular_z -> 车辆中心期望角速度，来自 /cmd_vel.angular.z
        # 输出：
        #   steer_values -> 左右前轮转角，发给 position controller
        #   wheel_values -> 左右后轮角速度，发给 velocity controller

        # Ackermann 车辆不能原地自转；linear.x 接近 0 时直接停车。
        if abs(linear_v) < 1e-4:
            return [0.0, 0.0], [0.0, 0.0]

        # 直行时没有转向角，后轮角速度 = 车辆线速度 / 轮子半径。
        if abs(angular_z) < 1e-4:
            wheel_velocity = linear_v / self.wheel_radius
            return [0.0, 0.0], [wheel_velocity, wheel_velocity]

        # 曲率：k = omega / v。曲率越大，转弯半径越小。
        curvature = angular_z / linear_v

        # 车辆中心点的转弯半径：R = 1 / |k|。
        center_radius = abs(1.0 / curvature)

        # 避免转弯半径小于半轮距，导致内侧轮半径接近 0 或为负。
        min_radius = self.track_width / 2.0 + 0.05
        center_radius = max(center_radius, min_radius)

        # ROS 坐标约定：angular.z > 0 为左转，angular.z < 0 为右转。
        turn_sign = 1.0 if curvature > 0.0 else -1.0

        # Ackermann 转向中，内侧前轮转角更大，外侧前轮转角更小。
        inner_radius = center_radius - self.track_width / 2.0
        outer_radius = center_radius + self.track_width / 2.0

        # tan(delta) = wheel_base / turn_radius，所以 delta = atan(wheel_base / R)。
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

        # 后轮左右线速度：左转时右后轮半径更大，所以右后轮更快；右转反过来。
        left_rear_linear_v = linear_v - angular_z * self.track_width / 2.0
        right_rear_linear_v = linear_v + angular_z * self.track_width / 2.0

        # ros2_control 的后轮控制器接收的是“关节角速度”，单位近似 rad/s。
        # 轮子角速度 = 轮子线速度 / 轮子半径。
        left_rear_wheel_velocity = left_rear_linear_v / self.wheel_radius
        right_rear_wheel_velocity = right_rear_linear_v / self.wheel_radius

        return [left_steer, right_steer], [left_rear_wheel_velocity, right_rear_wheel_velocity]

    def publish_control_commands(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        # 如果上层长时间没发 /cmd_vel，就认为命令失效，主动发布停车命令。
        if elapsed > self.command_timeout:
            linear_v = 0.0
            angular_z = 0.0
        else:
            linear_v = self.latest_linear_v
            angular_z = self.latest_angular_z

        steer_values, wheel_values = self.calculate_ackermann(linear_v, angular_z)

        # 两个 controller 都使用 Float64MultiArray：
        # 前轮数组表示两个转向关节的位置命令，后轮数组表示两个轮关节的速度命令。
        steer_msg = Float64MultiArray()
        steer_msg.data = steer_values

        wheel_msg = Float64MultiArray()
        wheel_msg.data = wheel_values

        self.steer_pub.publish(steer_msg)
        self.wheel_pub.publish(wheel_msg)


def main(args=None):
    # ROS 2 Python 节点标准入口：初始化 -> 创建节点 -> spin 循环 -> 退出清理。
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
