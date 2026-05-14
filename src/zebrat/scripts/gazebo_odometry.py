#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GazeboOdometryRelay(Node):
    def __init__(self):
        super().__init__("gazebo_odometry")
        self.declare_parameter("input_topic", "/model/r1/odometry")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.publisher = self.create_publisher(Odometry, self.get_parameter("output_topic").value, 10)
        self.create_subscription(Odometry, self.get_parameter("input_topic").value, self._callback, 10)

    def _callback(self, message):
        message.header.frame_id = self.get_parameter("odom_frame").value
        message.child_frame_id = self.get_parameter("base_frame").value
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboOdometryRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
