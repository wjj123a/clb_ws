#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformBroadcaster,
    TransformListener,
)
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    _, _, yaw = euler_from_quaternion(
        [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
    )
    return yaw


def inverse_planar(transform):
    x, y, yaw = transform
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        -cos_yaw * x - sin_yaw * y,
        sin_yaw * x - cos_yaw * y,
        -yaw,
    )


def compose_planar(lhs, rhs):
    lx, ly, lyaw = lhs
    rx, ry, ryaw = rhs
    cos_yaw = math.cos(lyaw)
    sin_yaw = math.sin(lyaw)
    return (
        lx + cos_yaw * rx - sin_yaw * ry,
        ly + sin_yaw * rx + cos_yaw * ry,
        normalize_angle(lyaw + ryaw),
    )


class GroundTruthLocalization(object):
    def __init__(self):
        self.map_frame_id = rospy.get_param("~map_frame_id", "map")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_footprint")
        self.ground_truth_topic = rospy.get_param("~ground_truth_topic", "ground_truth_odom")
        self.localized_pose_topic = rospy.get_param(
            "~localized_pose_topic", "ground_truth_localized_pose"
        )
        self.transform_tolerance = rospy.Duration(
            rospy.get_param("~transform_tolerance", 0.1)
        )

        self.listener = TransformListener()
        self.broadcaster = TransformBroadcaster()
        self.pose_publisher = rospy.Publisher(
            self.localized_pose_topic, PoseWithCovarianceStamped, queue_size=10
        )
        self.subscriber = rospy.Subscriber(
            self.ground_truth_topic, Odometry, self.handle_ground_truth, queue_size=10
        )

        rospy.loginfo(
            "ground_truth_localization publishing %s -> %s from %s with %.3fs tolerance",
            self.map_frame_id,
            self.odom_frame_id,
            self.ground_truth_topic,
            self.transform_tolerance.to_sec(),
        )

    def handle_ground_truth(self, message):
        pose = message.pose.pose
        stamp = message.header.stamp if message.header.stamp != rospy.Time() else rospy.Time.now()

        ground_truth_yaw = yaw_from_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        map_to_base = (
            pose.position.x,
            pose.position.y,
            normalize_angle(ground_truth_yaw),
        )

        try:
            odom_translation, odom_rotation = self.listener.lookupTransform(
                self.odom_frame_id, self.base_frame_id, rospy.Time(0)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            rospy.logwarn_throttle(
                2.0,
                "ground_truth_localization waiting for TF %s -> %s",
                self.odom_frame_id,
                self.base_frame_id,
            )
            return

        odom_to_base = (
            odom_translation[0],
            odom_translation[1],
            normalize_angle(yaw_from_quaternion(odom_rotation)),
        )
        map_to_odom = compose_planar(map_to_base, inverse_planar(odom_to_base))
        quaternion = quaternion_from_euler(0.0, 0.0, map_to_odom[2])
        transform_stamp = stamp + self.transform_tolerance

        localized_pose = PoseWithCovarianceStamped()
        localized_pose.header.stamp = stamp
        localized_pose.header.frame_id = self.map_frame_id
        localized_pose.pose.pose.position.x = map_to_base[0]
        localized_pose.pose.pose.position.y = map_to_base[1]
        localized_pose.pose.pose.position.z = 0.0
        localized_pose.pose.pose.orientation.x = 0.0
        localized_pose.pose.pose.orientation.y = 0.0
        localized_pose.pose.pose.orientation.z = quaternion_from_euler(
            0.0, 0.0, map_to_base[2]
        )[2]
        localized_pose.pose.pose.orientation.w = quaternion_from_euler(
            0.0, 0.0, map_to_base[2]
        )[3]
        localized_pose.pose.covariance[0] = 1e-4
        localized_pose.pose.covariance[7] = 1e-4
        localized_pose.pose.covariance[35] = 1e-4
        try:
            self.pose_publisher.publish(localized_pose)
            self.broadcaster.sendTransform(
                (map_to_odom[0], map_to_odom[1], 0.0),
                quaternion,
                transform_stamp,
                self.odom_frame_id,
                self.map_frame_id,
            )
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("ground_truth_localization")
    GroundTruthLocalization()
    rospy.spin()
