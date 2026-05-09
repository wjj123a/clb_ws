#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _signed(value, sign_source):
    return -abs(value) if sign_source < 0.0 else abs(value)


def _yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class R1AckermannController(Node):
    def __init__(self):
        super().__init__("r1_ackermann_controller")

        self.declare_parameter("input_topic", "/ackermann_cmd")
        self.declare_parameter("steering_controller_topic", "/front_steering_controller/commands")
        self.declare_parameter("wheel_controller_topic", "/front_wheel_controller/commands")
        self.declare_parameter("wheelbase", 0.41893)
        self.declare_parameter("front_track", 0.26462)
        self.declare_parameter("wheel_radius", 0.04075)
        self.declare_parameter("max_speed", 0.22)
        self.declare_parameter("max_steering_angle", 0.45)
        self.declare_parameter("command_timeout", 0.45)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("hold_steering_on_timeout", False)
        self.declare_parameter("left_wheel_speed_sign", 1.0)
        self.declare_parameter("right_wheel_speed_sign", -1.0)
        self.declare_parameter("left_steering_sign", 1.0)
        self.declare_parameter("right_steering_sign", 1.0)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.input_topic = self.get_parameter("input_topic").value
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.front_track = float(self.get_parameter("front_track").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.max_speed = abs(float(self.get_parameter("max_speed").value))
        self.max_steering_angle = abs(float(self.get_parameter("max_steering_angle").value))
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.hold_steering_on_timeout = bool(self.get_parameter("hold_steering_on_timeout").value)
        self.left_wheel_sign = float(self.get_parameter("left_wheel_speed_sign").value)
        self.right_wheel_sign = float(self.get_parameter("right_wheel_speed_sign").value)
        self.left_steering_sign = float(self.get_parameter("left_steering_sign").value)
        self.right_steering_sign = float(self.get_parameter("right_steering_sign").value)
        self.publish_odom = bool(self.get_parameter("publish_odom").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self._lock = threading.Lock()
        self._last_command = AckermannDriveStamped()
        self._last_command_wall = 0.0
        self._last_update_wall = time.monotonic()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._steering_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("steering_controller_topic").value, 10
        )
        self._wheel_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("wheel_controller_topic").value, 10
        )
        self._odom_pub = (
            self.create_publisher(Odometry, self.get_parameter("odom_topic").value, 10)
            if self.publish_odom
            else None
        )
        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(AckermannDriveStamped, self.input_topic, self._command_callback, 10)
        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.create_timer(1.0 / rate, self._timer_callback)
        self.get_logger().info(f"Consuming {self.input_topic} and commanding ros2_control group controllers")

    def _command_callback(self, message):
        with self._lock:
            self._last_command = message
            self._last_command_wall = time.monotonic()

    def _front_wheel_angle(self, rear_center_radius, lateral_offset):
        angle = math.atan2(self.wheelbase, rear_center_radius - lateral_offset)
        if angle > math.pi / 2.0:
            angle -= math.pi
        elif angle < -math.pi / 2.0:
            angle += math.pi
        return angle

    def _front_wheel_linear_speed(self, speed, curvature, rear_center_radius, lateral_offset):
        if abs(curvature) <= 1e-6:
            return speed
        yaw_rate = abs(speed * curvature)
        path_radius = math.hypot(self.wheelbase, rear_center_radius - lateral_offset)
        return _signed(yaw_rate * path_radius, speed)

    def _targets_from_command(self, command, fresh):
        speed = _clamp(command.drive.speed, -self.max_speed, self.max_speed) if fresh else 0.0
        steering = _clamp(command.drive.steering_angle, -self.max_steering_angle, self.max_steering_angle)
        if not fresh and not self.hold_steering_on_timeout:
            steering = 0.0

        if abs(steering) <= 1e-5:
            left_steering = 0.0
            right_steering = 0.0
            left_linear_speed = speed
            right_linear_speed = speed
        else:
            curvature = math.tan(steering) / self.wheelbase
            rear_center_radius = 1.0 / curvature
            half_track = self.front_track / 2.0
            left_steering = self._front_wheel_angle(rear_center_radius, half_track)
            right_steering = self._front_wheel_angle(rear_center_radius, -half_track)
            left_linear_speed = self._front_wheel_linear_speed(speed, curvature, rear_center_radius, half_track)
            right_linear_speed = self._front_wheel_linear_speed(speed, curvature, rear_center_radius, -half_track)

        left_wheel_velocity = self.left_wheel_sign * left_linear_speed / self.wheel_radius
        right_wheel_velocity = self.right_wheel_sign * right_linear_speed / self.wheel_radius
        return (
            self.left_steering_sign * left_steering,
            self.right_steering_sign * right_steering,
            left_wheel_velocity,
            right_wheel_velocity,
            speed,
            steering,
        )

    def _timer_callback(self):
        now_wall = time.monotonic()
        with self._lock:
            command = self._last_command
            fresh = self._last_command_wall > 0.0 and now_wall - self._last_command_wall <= self.command_timeout

        left_steering, right_steering, left_wheel, right_wheel, speed, steering = self._targets_from_command(
            command, fresh
        )
        steering_msg = Float64MultiArray()
        steering_msg.data = [left_steering, right_steering]
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [left_wheel, right_wheel]
        self._steering_pub.publish(steering_msg)
        self._wheel_pub.publish(wheel_msg)

        dt = max(0.0, min(now_wall - self._last_update_wall, 0.1))
        self._last_update_wall = now_wall
        self._integrate_and_publish_odom(speed, steering, dt)

    def _integrate_and_publish_odom(self, speed, steering, dt):
        if not self.publish_odom:
            return
        yaw_rate = speed * math.tan(steering) / self.wheelbase if abs(steering) > 1e-6 else 0.0
        self._yaw += yaw_rate * dt
        self._x += speed * math.cos(self._yaw) * dt
        self._y += speed * math.sin(self._yaw) * dt
        qx, qy, qz, qw = _yaw_to_quaternion(self._yaw)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = yaw_rate
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self._x
            transform.transform.translation.y = self._y
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = R1AckermannController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
