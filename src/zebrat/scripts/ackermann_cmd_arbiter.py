#!/usr/bin/env python3

import copy
import math
import threading
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


def _command_magnitude(message):
    return abs(message.drive.speed) + abs(message.drive.steering_angle)


def _same_command(left, right):
    return (
        math.isclose(left.drive.speed, right.drive.speed, abs_tol=1e-6)
        and math.isclose(left.drive.steering_angle, right.drive.steering_angle, abs_tol=1e-6)
    )


class AckermannCmdArbiter(Node):
    def __init__(self):
        super().__init__("ackermann_cmd_arbiter")

        self.declare_parameter("nav_topic", "/ackermann_cmd_nav")
        self.declare_parameter("teleop_topic", "/ackermann_cmd_teleop")
        self.declare_parameter("output_topic", "/ackermann_cmd")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("teleop_timeout", 0.75)
        self.declare_parameter("nav_timeout", 0.5)
        self.declare_parameter("publish_rate", 20.0)

        self.nav_topic = self.get_parameter("nav_topic").value
        self.teleop_topic = self.get_parameter("teleop_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.teleop_timeout = float(self.get_parameter("teleop_timeout").value)
        self.nav_timeout = float(self.get_parameter("nav_timeout").value)

        self._lock = threading.Lock()
        self._latest_nav = AckermannDriveStamped()
        self._latest_teleop = AckermannDriveStamped()
        self._last_nav_wall_time = 0.0
        self._last_teleop_wall_time = 0.0
        self._last_output = AckermannDriveStamped()
        self._have_last_output = False

        self._publisher = self.create_publisher(AckermannDriveStamped, self.output_topic, 10)
        self.create_subscription(AckermannDriveStamped, self.nav_topic, self._nav_callback, 10)
        self.create_subscription(AckermannDriveStamped, self.teleop_topic, self._teleop_callback, 10)

        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.create_timer(1.0 / rate, self._timer_callback)
        self.get_logger().info(
            f"Forwarding teleop={self.teleop_topic} over nav={self.nav_topic} to {self.output_topic}"
        )

    def _stamp(self, command):
        command.header.stamp = self.get_clock().now().to_msg()
        if not command.header.frame_id:
            command.header.frame_id = self.frame_id
        return command

    def _zero_command(self):
        return self._stamp(AckermannDriveStamped())

    def _nav_callback(self, message):
        with self._lock:
            self._latest_nav = copy.deepcopy(message)
            self._last_nav_wall_time = time.monotonic()

    def _teleop_callback(self, message):
        with self._lock:
            self._latest_teleop = copy.deepcopy(message)
            self._last_teleop_wall_time = time.monotonic()

    def _select_command(self):
        now = time.monotonic()
        with self._lock:
            teleop_active = (
                self._last_teleop_wall_time > 0.0
                and now - self._last_teleop_wall_time <= self.teleop_timeout
            )
            nav_active = self._last_nav_wall_time > 0.0 and now - self._last_nav_wall_time <= self.nav_timeout

            if teleop_active:
                return copy.deepcopy(self._latest_teleop)
            if nav_active:
                return copy.deepcopy(self._latest_nav)
            return self._zero_command()

    def _timer_callback(self):
        command = self._stamp(self._select_command())
        if (
            self._have_last_output
            and _same_command(command, self._last_output)
            and _command_magnitude(command) == 0.0
        ):
            return

        self._publisher.publish(command)
        self._last_output = copy.deepcopy(command)
        self._have_last_output = True


def main(args=None):
    rclpy.init(args=args)
    node = AckermannCmdArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
