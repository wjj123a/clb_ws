#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


HELP = """R1 Ackermann teleop

w/s: speed up/down
a/d: steer left/right
space: stop
q: quit
"""


class KeyboardAckermannTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_ackermann_teleop")
        self.declare_parameter("output_topic", "/ackermann_cmd_teleop")
        self.declare_parameter("speed_step", 0.04)
        self.declare_parameter("steering_step", 0.06)
        self.declare_parameter("max_speed", 0.22)
        self.declare_parameter("max_steering_angle", 0.45)
        self.declare_parameter("publish_rate", 20.0)

        self.speed = 0.0
        self.steering = 0.0
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.steering_step = float(self.get_parameter("steering_step").value)
        self.max_speed = abs(float(self.get_parameter("max_speed").value))
        self.max_steering = abs(float(self.get_parameter("max_steering_angle").value))
        self.publisher = self.create_publisher(
            AckermannDriveStamped, self.get_parameter("output_topic").value, 10
        )
        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.create_timer(1.0 / rate, self._publish)

    def update_from_key(self, key):
        if key == "w":
            self.speed = min(self.speed + self.speed_step, self.max_speed)
        elif key == "s":
            self.speed = max(self.speed - self.speed_step, -self.max_speed)
        elif key == "a":
            self.steering = min(self.steering + self.steering_step, self.max_steering)
        elif key == "d":
            self.steering = max(self.steering - self.steering_step, -self.max_steering)
        elif key == " ":
            self.speed = 0.0
            self.steering = 0.0

    def _publish(self):
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed = self.speed
        message.drive.steering_angle = self.steering
        self.publisher.publish(message)


def _read_key(timeout=0.05):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if not readable:
        return ""
    return sys.stdin.read(1)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardAckermannTeleop()
    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = _read_key()
            if key == "q":
                break
            if key:
                node.update_from_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
