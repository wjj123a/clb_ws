#!/usr/bin/env python3

import math
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def _yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wheelbase", 0.41893)
        self.declare_parameter("wheel_radius", 0.04075)
        self.declare_parameter("left_wheel_joint", "front_left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "front_right_wheel_joint")
        self.declare_parameter("left_steering_joint", "front_left_steering_joint")
        self.declare_parameter("right_steering_joint", "front_right_steering_joint")
        self.declare_parameter("left_wheel_speed_sign", 1.0)
        self.declare_parameter("right_wheel_speed_sign", -1.0)
        self.declare_parameter("publish_tf", False)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.left_wheel_joint = self.get_parameter("left_wheel_joint").value
        self.right_wheel_joint = self.get_parameter("right_wheel_joint").value
        self.left_steering_joint = self.get_parameter("left_steering_joint").value
        self.right_steering_joint = self.get_parameter("right_steering_joint").value
        self.left_wheel_sign = float(self.get_parameter("left_wheel_speed_sign").value)
        self.right_wheel_sign = float(self.get_parameter("right_wheel_speed_sign").value)

        self._lock = threading.Lock()
        self._last_stamp = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._publisher = self.create_publisher(Odometry, self.odom_topic, 20)
        self._tf_broadcaster = TransformBroadcaster(self) if bool(self.get_parameter("publish_tf").value) else None
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self._joint_state_callback,
            20,
        )
        self.get_logger().info(f"Publishing wheel odometry on {self.odom_topic}")

    def _joint_value(self, msg, name, values):
        try:
            index = msg.name.index(name)
        except ValueError:
            return None
        if index >= len(values):
            return None
        return values[index]

    def _joint_state_callback(self, msg):
        left_wheel = self._joint_value(msg, self.left_wheel_joint, msg.velocity)
        right_wheel = self._joint_value(msg, self.right_wheel_joint, msg.velocity)
        left_steering = self._joint_value(msg, self.left_steering_joint, msg.position)
        right_steering = self._joint_value(msg, self.right_steering_joint, msg.position)
        if None in (left_wheel, right_wheel, left_steering, right_steering):
            self.get_logger().warn("JointState is missing wheel or steering joints", throttle_duration_sec=2.0)
            return

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()

        with self._lock:
            if self._last_stamp is None:
                self._last_stamp = stamp
                return

            dt = (stamp - self._last_stamp).nanoseconds * 1e-9
            self._last_stamp = stamp
            if dt <= 0.0 or dt > 0.5:
                return

            left_speed = self.left_wheel_sign * float(left_wheel) * self.wheel_radius
            right_speed = self.right_wheel_sign * float(right_wheel) * self.wheel_radius
            speed = 0.5 * (left_speed + right_speed)
            steering = 0.5 * (float(left_steering) + float(right_steering))
            yaw_rate = speed * math.tan(steering) / self.wheelbase if abs(steering) > 1e-6 else 0.0

            self._yaw += yaw_rate * dt
            self._x += speed * math.cos(self._yaw) * dt
            self._y += speed * math.sin(self._yaw) * dt
            self._publish(stamp, speed, yaw_rate)

    def _publish(self, stamp, speed, yaw_rate):
        qx, qy, qz, qw = _yaw_to_quaternion(self._yaw)
        stamp_msg = stamp.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.03
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[35] = 0.20
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = yaw_rate
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.10
        self._publisher.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp_msg
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
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
