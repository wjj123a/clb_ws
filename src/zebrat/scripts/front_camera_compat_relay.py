#!/usr/bin/env python3

from copy import deepcopy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class FrontCameraCompatRelay(Node):
    def __init__(self):
        super().__init__("front_camera_compat_relay")

        self.declare_parameter("frame_id", "camera_optical_link")
        self.declare_parameter("depth_image_in", "/camera/depth/image_raw")
        self.declare_parameter("depth_info_in", "/camera/camera_info")
        self.declare_parameter("rgb_image_in", "/camera/rgb/image_raw")
        self.declare_parameter("rgb_info_in", "/camera/rgb/camera_info")
        self.declare_parameter("depth_image_out", "/front_camera/depth/image_raw")
        self.declare_parameter("depth_viz_out", "/front_camera/depth/image_viz")
        self.declare_parameter("depth_info_out", "/front_camera/depth/camera_info")
        self.declare_parameter("rgb_image_out", "/front_camera/rgb/image_raw")
        self.declare_parameter("rgb_info_out", "/front_camera/rgb/camera_info")
        self.declare_parameter("mirror_depth_info_to_rgb", True)
        self.declare_parameter("publish_depth_viz", True)
        self.declare_parameter("depth_viz_min", 0.10)
        self.declare_parameter("depth_viz_max", 8.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.mirror_depth_info_to_rgb = bool(self.get_parameter("mirror_depth_info_to_rgb").value)
        self.publish_depth_viz = bool(self.get_parameter("publish_depth_viz").value)
        self.depth_viz_min = float(self.get_parameter("depth_viz_min").value)
        self.depth_viz_max = float(self.get_parameter("depth_viz_max").value)
        self.seen_rgb_info = False

        self.depth_image_pub = self.create_publisher(
            Image, self.get_parameter("depth_image_out").value, qos_profile_sensor_data
        )
        self.depth_viz_pub = self.create_publisher(
            Image, self.get_parameter("depth_viz_out").value, qos_profile_sensor_data
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("depth_info_out").value, qos_profile_sensor_data
        )
        self.rgb_image_pub = self.create_publisher(
            Image, self.get_parameter("rgb_image_out").value, qos_profile_sensor_data
        )
        self.rgb_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("rgb_info_out").value, qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            self.get_parameter("depth_image_in").value,
            self._relay_depth_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("depth_info_in").value,
            self._relay_depth_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("rgb_image_in").value,
            lambda msg: self._relay_image(msg, self.rgb_image_pub),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("rgb_info_in").value,
            self._relay_rgb_info,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Publishing ROS1-style front camera topics in frame '{self.frame_id}'"
        )

    def _stamp_frame(self, msg):
        msg.header.frame_id = self.frame_id
        return msg

    def _relay_image(self, msg, publisher):
        publisher.publish(self._stamp_frame(msg))

    def _relay_depth_image(self, msg):
        out = self._stamp_frame(msg)
        if out.encoding.upper() in ("R_FLOAT32", "FLOAT32"):
            out.encoding = "32FC1"
            out.step = out.width * 4
        self.depth_image_pub.publish(out)
        if self.publish_depth_viz:
            self._publish_depth_viz(out)

    def _depth_array(self, msg):
        encoding = msg.encoding.upper()
        scale = 1.0
        if encoding in ("32FC1", "R_FLOAT32", "TYPE_32FC1"):
            dtype = np.dtype(np.float32)
        elif encoding in ("16UC1", "MONO16", "TYPE_16UC1"):
            dtype = np.dtype(np.uint16)
            scale = 0.001
        else:
            self.get_logger().warn(f"Unsupported depth encoding '{msg.encoding}'", throttle_duration_sec=5.0)
            return None

        if msg.is_bigendian:
            dtype = dtype.newbyteorder(">")

        row_width = msg.step // dtype.itemsize
        depth = np.frombuffer(msg.data, dtype=dtype)
        try:
            depth = depth.reshape((msg.height, row_width))[:, : msg.width].astype(np.float32)
        except ValueError:
            self.get_logger().warn("Depth image stride does not match image dimensions", throttle_duration_sec=5.0)
            return None
        return depth * scale

    def _publish_depth_viz(self, msg):
        depth = self._depth_array(msg)
        if depth is None:
            return

        valid = np.isfinite(depth) & (depth >= self.depth_viz_min) & (depth <= self.depth_viz_max)
        if not np.any(valid):
            return

        normalized = np.zeros(depth.shape, dtype=np.float32)
        normalized[valid] = (depth[valid] - self.depth_viz_min) / (self.depth_viz_max - self.depth_viz_min)
        normalized = np.clip(normalized, 0.0, 1.0)

        viz = Image()
        viz.header = msg.header
        viz.height = msg.height
        viz.width = msg.width
        viz.encoding = "mono8"
        viz.is_bigendian = 0
        viz.step = msg.width
        viz.data = (255.0 * (1.0 - normalized)).astype(np.uint8).tobytes()
        self.depth_viz_pub.publish(viz)

    def _relay_depth_info(self, msg):
        out = self._stamp_frame(msg)
        self.depth_info_pub.publish(out)
        if self.mirror_depth_info_to_rgb and not self.seen_rgb_info:
            self.rgb_info_pub.publish(deepcopy(out))

    def _relay_rgb_info(self, msg):
        self.seen_rgb_info = True
        self.rgb_info_pub.publish(self._stamp_frame(msg))


def main():
    rclpy.init()
    node = FrontCameraCompatRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
