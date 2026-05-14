#!/usr/bin/env python3

import copy
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def _command_nonzero(message):
    return abs(message.drive.speed) > 1e-4


def _wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_in_sector(angle, center, half_width):
    return abs(_wrap_to_pi(angle - center)) <= half_width


def _usable_range(message, value):
    if math.isnan(value) or value <= 0.0:
        return None
    if math.isinf(value):
        if value > 0.0 and math.isfinite(message.range_max) and message.range_max > 0.0:
            return message.range_max
        return None
    return value


class AckermannCmdSafetySupervisor(Node):
    def __init__(self):
        super().__init__("ackermann_cmd_safety_supervisor")

        self.declare_parameter("input_topic", "/ackermann_cmd_safety_in")
        self.declare_parameter("output_topic", "/ackermann_cmd")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("front_angle", 0.65)
        self.declare_parameter("rear_angle", 0.65)
        self.declare_parameter("hard_stop_distance", 0.45)
        self.declare_parameter("slowdown_distance", 0.85)
        self.declare_parameter("reaction_time", 0.35)
        self.declare_parameter("max_deceleration", 0.45)
        self.declare_parameter("ttc_stop_time", 1.0)
        self.declare_parameter("scan_timeout", 0.5)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.front_angle = float(self.get_parameter("front_angle").value)
        self.rear_angle = float(self.get_parameter("rear_angle").value)
        self.hard_stop_distance = float(self.get_parameter("hard_stop_distance").value)
        self.slowdown_distance = float(self.get_parameter("slowdown_distance").value)
        self.reaction_time = float(self.get_parameter("reaction_time").value)
        self.max_deceleration = max(0.05, float(self.get_parameter("max_deceleration").value))
        self.ttc_stop_time = float(self.get_parameter("ttc_stop_time").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)

        self._front_min = float("inf")
        self._rear_min = float("inf")
        self._front_closing_speed = 0.0
        self._rear_closing_speed = 0.0
        self._last_scan_wall = 0.0
        self._warn_times = {}

        self._publisher = self.create_publisher(AckermannDriveStamped, self.output_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self.create_subscription(AckermannDriveStamped, self.input_topic, self._cmd_callback, 10)
        self.get_logger().info(
            f"Forwarding {self.input_topic} to {self.output_topic} with scan safety from {self.scan_topic}"
        )

    def _warn_throttled(self, key, period, message):
        now = time.monotonic()
        if now - self._warn_times.get(key, 0.0) >= period:
            self.get_logger().warn(message)
            self._warn_times[key] = now

    def _stamp(self, command):
        command.header.stamp = self.get_clock().now().to_msg()
        if not command.header.frame_id:
            command.header.frame_id = self.frame_id
        return command

    def _zero_command(self):
        return self._stamp(AckermannDriveStamped())

    @staticmethod
    def _closing_speed(previous_min, current_min, previous_wall, current_wall):
        if (
            not math.isfinite(previous_min)
            or not math.isfinite(current_min)
            or previous_wall <= 0.0
            or current_wall <= previous_wall
        ):
            return 0.0
        return max(0.0, (previous_min - current_min) / (current_wall - previous_wall))

    def _scan_callback(self, message):
        front_ranges = []
        rear_ranges = []
        for index, value in enumerate(message.ranges):
            usable = _usable_range(message, value)
            if usable is None:
                continue
            angle = message.angle_min + index * message.angle_increment
            if _angle_in_sector(angle, 0.0, self.front_angle):
                front_ranges.append(usable)
            if _angle_in_sector(angle, math.pi, self.rear_angle):
                rear_ranges.append(usable)

        now = time.monotonic()
        current_front_min = min(front_ranges) if front_ranges else float("inf")
        current_rear_min = min(rear_ranges) if rear_ranges else float("inf")

        self._front_closing_speed = self._closing_speed(
            self._front_min,
            current_front_min,
            self._last_scan_wall,
            now,
        )
        self._rear_closing_speed = self._closing_speed(
            self._rear_min,
            current_rear_min,
            self._last_scan_wall,
            now,
        )
        self._front_min = current_front_min
        self._rear_min = current_rear_min
        self._last_scan_wall = now

    def _scan_is_fresh(self):
        return self._last_scan_wall > 0.0 and time.monotonic() - self._last_scan_wall <= self.scan_timeout

    def _required_stop_distance(self, speed):
        return self.hard_stop_distance + speed * self.reaction_time + (speed * speed) / (2.0 * self.max_deceleration)

    def _time_to_collision(self, speed, obstacle_distance, closing_speed):
        closing_speed = max(0.0, speed, closing_speed)
        remaining_distance = obstacle_distance - self.hard_stop_distance
        if closing_speed <= 1e-3 or remaining_distance <= 0.0:
            return 0.0 if remaining_distance <= 0.0 else float("inf")
        return remaining_distance / closing_speed

    def _sector_for_speed(self, speed):
        if speed < -1e-4:
            return "rear", self._rear_min, self._rear_closing_speed
        return "front", self._front_min, self._front_closing_speed

    def _should_emergency_stop(self, command):
        if not self._scan_is_fresh():
            if _command_nonzero(command):
                self._warn_throttled("stale_scan", 2.0, f"Stopping command because {self.scan_topic} is stale")
            return _command_nonzero(command)

        speed = abs(command.drive.speed)
        if speed <= 1e-4:
            return False

        direction, obstacle_distance, closing_speed = self._sector_for_speed(command.drive.speed)
        if not math.isfinite(obstacle_distance):
            self._warn_throttled(
                "no_ranges",
                2.0,
                f"Stopping command because {direction} sector has no valid ranges from {self.scan_topic}",
            )
            return True

        if obstacle_distance <= self.hard_stop_distance:
            self._warn_throttled(
                "hard_stop",
                1.0,
                f"Emergency stop: {direction} obstacle {obstacle_distance:.2f}m <= {self.hard_stop_distance:.2f}m",
            )
            return True

        required_distance = self._required_stop_distance(speed)
        ttc = self._time_to_collision(speed, obstacle_distance, closing_speed)
        if obstacle_distance <= required_distance:
            self._warn_throttled(
                "required_stop",
                1.0,
                f"Emergency stop: {direction} obstacle {obstacle_distance:.2f}m <= {required_distance:.2f}m",
            )
            return True
        if ttc <= self.ttc_stop_time:
            self._warn_throttled("ttc", 1.0, f"Emergency stop: {direction} obstacle TTC {ttc:.2f}s")
            return True
        return False

    def _apply_slowdown(self, command):
        if abs(command.drive.speed) <= 1e-4 or not self._scan_is_fresh():
            return command
        _direction, obstacle_distance, _closing_speed = self._sector_for_speed(command.drive.speed)
        if obstacle_distance >= self.slowdown_distance:
            return command

        available = max(0.0, obstacle_distance - self.hard_stop_distance)
        span = max(0.05, self.slowdown_distance - self.hard_stop_distance)
        scale = max(0.0, min(1.0, available / span))
        limited = copy.deepcopy(command)
        limited.drive.speed *= scale
        return limited

    def _cmd_callback(self, message):
        if self._should_emergency_stop(message):
            self._publisher.publish(self._zero_command())
            return

        command = self._stamp(self._apply_slowdown(copy.deepcopy(message)))
        self._publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannCmdSafetySupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
