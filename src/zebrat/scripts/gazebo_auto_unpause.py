#!/usr/bin/env python3

import subprocess
import time

import rclpy
from rclpy.node import Node


class GazeboAutoUnpause(Node):
    def __init__(self):
        super().__init__("gazebo_auto_unpause")
        self.declare_parameter("world", "default")
        self.declare_parameter("delay", 2.0)
        self.declare_parameter("retries", 5)
        self._timer = self.create_timer(float(self.get_parameter("delay").value), self._unpause_once)

    def _unpause_once(self):
        self._timer.cancel()
        world = self.get_parameter("world").value
        service = f"/world/{world}/control"
        command = [
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1000",
            "--req",
            "pause: false",
        ]
        for attempt in range(int(self.get_parameter("retries").value)):
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            if result.returncode == 0:
                self.get_logger().info(f"Unpaused Gazebo world '{world}'")
                rclpy.shutdown()
                return
            self.get_logger().warn(f"Unpause attempt {attempt + 1} failed: {result.stderr.strip()}")
            time.sleep(0.5)
        self.get_logger().error(f"Could not unpause Gazebo world '{world}'")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboAutoUnpause()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
