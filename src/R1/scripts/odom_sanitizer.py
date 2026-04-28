#!/usr/bin/env python3

import copy
import math

import rospy
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def is_finite(value):
    return math.isfinite(value)


def vector_is_finite(vector):
    return is_finite(vector.x) and is_finite(vector.y) and is_finite(vector.z)


def quaternion_is_valid(quaternion):
    if not (
        is_finite(quaternion.x)
        and is_finite(quaternion.y)
        and is_finite(quaternion.z)
        and is_finite(quaternion.w)
    ):
        return False
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    return norm > 1e-9


def normalized_quaternion(quaternion):
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    normalized = copy.deepcopy(quaternion)
    normalized.x /= norm
    normalized.y /= norm
    normalized.z /= norm
    normalized.w /= norm
    return normalized


class OdomSanitizer(object):
    def __init__(self):
        self.last_valid_pose = None
        self.last_planar_state = None

        output_topic = rospy.get_param("~output_topic", "ground_truth_odom_sanitized")
        input_topic = rospy.get_param("~input_topic", "ground_truth_odom")
        self.publish_tf = rospy.get_param("~publish_tf", False)
        self.output_frame_id = rospy.get_param("~output_frame_id", "")
        self.output_child_frame_id = rospy.get_param("~output_child_frame_id", "")

        self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=10)
        self.subscriber = rospy.Subscriber(input_topic, Odometry, self.handle_odom, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster() if self.publish_tf else None

        rospy.loginfo("odom_sanitizer relaying %s -> %s", input_topic, output_topic)

    def handle_odom(self, message):
        sanitized = copy.deepcopy(message)
        if self.output_frame_id:
            sanitized.header.frame_id = self.output_frame_id
        if self.output_child_frame_id:
            sanitized.child_frame_id = self.output_child_frame_id

        pose = sanitized.pose.pose
        if vector_is_finite(pose.position) and quaternion_is_valid(pose.orientation):
            self.last_valid_pose = copy.deepcopy(pose)
        elif self.last_valid_pose is not None:
            sanitized.pose.pose = copy.deepcopy(self.last_valid_pose)
            rospy.logwarn_throttle(2.0, "odom_sanitizer replaced invalid pose with last valid pose")
        else:
            sanitized.pose.pose.position.x = 0.0
            sanitized.pose.pose.position.y = 0.0
            sanitized.pose.pose.position.z = 0.0
            sanitized.pose.pose.orientation.x = 0.0
            sanitized.pose.pose.orientation.y = 0.0
            sanitized.pose.pose.orientation.z = 0.0
            sanitized.pose.pose.orientation.w = 1.0
            rospy.logwarn_throttle(2.0, "odom_sanitizer replaced invalid pose with identity")

        planar_quaternion = normalized_quaternion(sanitized.pose.pose.orientation)
        _, _, yaw = euler_from_quaternion(
            [
                planar_quaternion.x,
                planar_quaternion.y,
                planar_quaternion.z,
                planar_quaternion.w,
            ]
        )
        planar_orientation = quaternion_from_euler(0.0, 0.0, yaw)
        sanitized.pose.pose.position.z = 0.0
        sanitized.pose.pose.orientation.x = planar_orientation[0]
        sanitized.pose.pose.orientation.y = planar_orientation[1]
        sanitized.pose.pose.orientation.z = planar_orientation[2]
        sanitized.pose.pose.orientation.w = planar_orientation[3]

        planar_x = sanitized.pose.pose.position.x
        planar_y = sanitized.pose.pose.position.y

        sanitized.twist.twist.linear.x = 0.0
        sanitized.twist.twist.linear.y = 0.0
        sanitized.twist.twist.linear.z = 0.0
        sanitized.twist.twist.angular.x = 0.0
        sanitized.twist.twist.angular.y = 0.0
        sanitized.twist.twist.angular.z = 0.0

        stamp = sanitized.header.stamp if sanitized.header.stamp != rospy.Time() else rospy.Time.now()
        if self.last_planar_state is not None:
            last_x, last_y, last_yaw, last_stamp = self.last_planar_state
            dt = (stamp - last_stamp).to_sec()
            if dt > 1e-6:
                dx = planar_x - last_x
                dy = planar_y - last_y
                dyaw = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
                sanitized.twist.twist.linear.x = (math.cos(yaw) * dx + math.sin(yaw) * dy) / dt
                sanitized.twist.twist.angular.z = dyaw / dt

        self.last_planar_state = (planar_x, planar_y, yaw, stamp)

        if rospy.is_shutdown():
            return

        try:
            self.publisher.publish(sanitized)
        except rospy.ROSException:
            return

        if self.tf_broadcaster is not None:
            pose = sanitized.pose.pose
            self.tf_broadcaster.sendTransform(
                (pose.position.x, pose.position.y, pose.position.z),
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
                stamp,
                sanitized.child_frame_id,
                sanitized.header.frame_id,
            )


if __name__ == "__main__":
    rospy.init_node("odom_sanitizer")
    OdomSanitizer()
    rospy.spin()
