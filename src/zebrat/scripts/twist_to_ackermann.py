#!/usr/bin/env python3

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class TwistToAckermann(Node):
    def __init__(self):
        super().__init__("twist_to_ackermann")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/ackermann_cmd")
        self.declare_parameter("stamped_input", False)
        self.declare_parameter("wheelbase", 0.41893)
        self.declare_parameter("max_speed", 0.22)
        self.declare_parameter("max_steering_angle", 0.45)
        self.declare_parameter("cmd_angle_instead_rotvel", False)

        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_speed = abs(float(self.get_parameter("max_speed").value))
        self.max_steering = abs(float(self.get_parameter("max_steering_angle").value))
        self.cmd_angle_instead_rotvel = bool(self.get_parameter("cmd_angle_instead_rotvel").value)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, self.get_parameter("output_topic").value, 10
        )

        topic = self.get_parameter("input_topic").value
        if bool(self.get_parameter("stamped_input").value):
            self.create_subscription(TwistStamped, topic, self._stamped_callback, 10)
        else:
            self.create_subscription(Twist, topic, self._twist_callback, 10)

    def _stamped_callback(self, message):
        self._publish(message.twist)

    def _twist_callback(self, message):
        self._publish(message)

    def _publish(self, twist):
        speed = _clamp(twist.linear.x, -self.max_speed, self.max_speed)
        if self.cmd_angle_instead_rotvel:
            steering = twist.angular.z
        elif abs(speed) < 1e-5:
            steering = 0.0
        else:
            steering = math.atan2(self.wheelbase * twist.angular.z, speed)
        steering = _clamp(steering, -self.max_steering, self.max_steering)

        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.drive.speed = speed
        command.drive.steering_angle = steering
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermann()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
