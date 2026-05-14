#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__("depth_to_pointcloud")

        self.declare_parameter("depth_topic", "/front_camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/front_camera/depth/camera_info")
        self.declare_parameter("points_topic", "/front_camera/depth/points")
        self.declare_parameter("frame_id", "camera_optical_link")
        self.declare_parameter("downsample", 4)
        self.declare_parameter("min_depth", 0.10)
        self.declare_parameter("max_depth", 8.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.downsample = max(1, int(self.get_parameter("downsample").value))
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.camera_info = None

        self.points_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("points_topic").value,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self._depth_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Publishing depth points on {self.get_parameter('points_topic').value}"
        )

    def _camera_info_cb(self, msg):
        self.camera_info = msg

    def _depth_array(self, msg):
        encoding = msg.encoding.upper()
        if encoding in ("32FC1", "R_FLOAT32", "TYPE_32FC1"):
            dtype = np.dtype(np.float32)
            scale = 1.0
        elif encoding in ("16UC1", "MONO16", "TYPE_16UC1"):
            dtype = np.dtype(np.uint16)
            scale = 0.001
        else:
            self.get_logger().warn(f"Unsupported depth encoding '{msg.encoding}'", throttle_duration_sec=5.0)
            return None

        if msg.is_bigendian:
            dtype = dtype.newbyteorder(">")

        row_width = msg.step // dtype.itemsize
        arr = np.frombuffer(msg.data, dtype=dtype)
        try:
            arr = arr.reshape((msg.height, row_width))[:, : msg.width].astype(np.float32)
        except ValueError:
            self.get_logger().warn("Depth image stride does not match image dimensions", throttle_duration_sec=5.0)
            return None

        return arr * scale

    def _depth_cb(self, msg):
        info = self.camera_info
        if info is None:
            return

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx <= 0.0 or fy <= 0.0 or not math.isfinite(fx) or not math.isfinite(fy):
            self.get_logger().warn("CameraInfo has invalid intrinsics", throttle_duration_sec=5.0)
            return

        depth = self._depth_array(msg)
        if depth is None:
            return

        ds = self.downsample
        depth = depth[::ds, ::ds]
        v_coords = np.arange(0, msg.height, ds, dtype=np.float32)
        u_coords = np.arange(0, msg.width, ds, dtype=np.float32)
        u_grid, v_grid = np.meshgrid(u_coords, v_coords)

        valid = np.isfinite(depth) & (depth >= self.min_depth) & (depth <= self.max_depth)
        if not np.any(valid):
            return

        z = depth[valid]
        x = (u_grid[valid] - cx) * z / fx
        y = (v_grid[valid] - cy) * z / fy
        points = np.column_stack((x, y, z)).astype(np.float32)

        header = msg.header
        header.frame_id = self.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self.points_pub.publish(cloud)


def main():
    rclpy.init()
    node = DepthToPointCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
