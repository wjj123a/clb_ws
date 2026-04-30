#!/usr/bin/env python3

import math
import time

import rospy
import tf
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class TwistToAckermann:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_nav")
        self.output_topic = rospy.get_param("~output_topic", "/ackermann_cmd_nav")
        self.frame_id = rospy.get_param("~frame_id", "base_footprint")

        self.wheelbase = float(rospy.get_param("~wheelbase", 0.41893))
        self.max_speed = abs(float(rospy.get_param("~max_speed", 0.22)))
        self.max_steering_angle = abs(float(rospy.get_param("~max_steering_angle", 0.45)))
        self.planner_max_steering_angle = min(
            abs(float(rospy.get_param("~planner_max_steering_angle", self.max_steering_angle))),
            self.max_steering_angle,
        )
        self.min_turn_speed = abs(float(rospy.get_param("~min_turn_speed", 0.06)))
        self.angular_deadband = abs(float(rospy.get_param("~angular_deadband", 1e-3)))
        self.allow_rotate_crawl = bool(rospy.get_param("~allow_rotate_crawl", True))
        self.angular_z_is_steering_angle = bool(
            rospy.get_param("~angular_z_is_steering_angle", False)
        )
        self.use_local_plan_steering = bool(
            rospy.get_param("~use_local_plan_steering", True)
        )
        self.local_plan_topic = rospy.get_param(
            "~local_plan_topic", "/move_base/TebLocalPlannerROS/local_plan"
        )
        self.local_plan_timeout = max(
            0.0, float(rospy.get_param("~local_plan_timeout", 0.50))
        )
        self.path_lookahead_base = abs(
            float(rospy.get_param("~path_lookahead_base", 0.75))
        )
        self.path_lookahead_speed_gain = abs(
            float(rospy.get_param("~path_lookahead_speed_gain", 0.9))
        )
        self.path_lookahead_min = abs(
            float(rospy.get_param("~path_lookahead_min", 0.55))
        )
        self.path_lookahead_max = abs(
            float(rospy.get_param("~path_lookahead_max", 1.20))
        )
        if self.path_lookahead_max < self.path_lookahead_min:
            self.path_lookahead_max = self.path_lookahead_min
        self.min_forward_tracking_speed = abs(
            float(rospy.get_param("~min_forward_tracking_speed", 0.05))
        )
        self.steering_rate_limit = abs(float(rospy.get_param("~steering_rate_limit", 0.0)))
        self.steering_deadband = abs(
            float(rospy.get_param("~steering_deadband", self.angular_deadband))
        )
        self.steering_deadband_release = abs(
            float(rospy.get_param("~steering_deadband_release", self.steering_deadband))
        )
        if self.steering_deadband_release < self.steering_deadband:
            self.steering_deadband_release = self.steering_deadband
        self._steering_deadband_active = False
        self._last_steering = 0.0
        self._last_steering_wall = None
        self._local_plan = None
        self._local_plan_wall = 0.0
        self._tf_listener = tf.TransformListener() if self.use_local_plan_steering else None

        self.publisher = rospy.Publisher(self.output_topic, AckermannDriveStamped, queue_size=1)
        rospy.Subscriber(self.input_topic, Twist, self.callback, queue_size=1)
        if self.use_local_plan_steering:
            rospy.Subscriber(self.local_plan_topic, Path, self._local_plan_callback, queue_size=1)
        rospy.loginfo(
            (
                "twist_to_ackermann forwarding %s to %s "
                "(%s, steering rate limit %.2f rad/s, deadband %.3f/%.3f rad, "
                "local plan steering %s)"
            ),
            self.input_topic,
            self.output_topic,
            "angular.z as steering angle"
            if self.angular_z_is_steering_angle
            else "angular.z as yaw rate",
            self.steering_rate_limit,
            self.steering_deadband,
            self.steering_deadband_release,
            "enabled" if self.use_local_plan_steering else "disabled",
        )

    def _local_plan_callback(self, message):
        self._local_plan = message
        self._local_plan_wall = time.monotonic()

    def _local_plan_is_fresh(self):
        if self._local_plan is None or self._local_plan_wall <= 0.0:
            return False
        return time.monotonic() - self._local_plan_wall <= self.local_plan_timeout

    def _lookahead_distance(self, speed):
        lookahead = self.path_lookahead_base + self.path_lookahead_speed_gain * abs(speed)
        return _clamp(lookahead, self.path_lookahead_min, self.path_lookahead_max)

    def _pose_in_base_frame(self, plan, pose):
        frame_id = pose.header.frame_id or plan.header.frame_id
        if not frame_id:
            return None

        if frame_id == self.frame_id:
            return pose.pose.position.x, pose.pose.position.y

        transformed = PoseStamped()
        transformed.header.frame_id = frame_id
        transformed.header.stamp = rospy.Time(0)
        transformed.pose = pose.pose
        base_pose = self._tf_listener.transformPose(self.frame_id, transformed)
        return base_pose.pose.position.x, base_pose.pose.position.y

    def _local_plan_steering(self, speed):
        if (
            not self.use_local_plan_steering
            or speed < -1e-4
            or not self._local_plan_is_fresh()
            or len(self._local_plan.poses) < 2
        ):
            return None

        lookahead = self._lookahead_distance(speed)
        best_point = None
        best_over_distance = float("inf")
        fallback_point = None
        fallback_distance = 0.0

        try:
            for pose in self._local_plan.poses:
                point = self._pose_in_base_frame(self._local_plan, pose)
                if point is None:
                    continue
                x, y = point
                if x <= 0.02:
                    continue
                distance = math.hypot(x, y)
                if distance < 0.05:
                    continue
                if distance >= lookahead:
                    over_distance = distance - lookahead
                    if over_distance < best_over_distance:
                        best_point = point
                        best_over_distance = over_distance
                elif distance > fallback_distance:
                    fallback_point = point
                    fallback_distance = distance
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "Falling back to TEB steering because local plan transform failed: %s",
                exc,
            )
            return None

        point = best_point
        if point is None:
            # 预瞄点不足时只接受仍有前方路径的短预瞄，避免到终点附近强行前进。
            if fallback_point is None or fallback_distance < max(0.25, 0.5 * lookahead):
                return None
            point = fallback_point

        _x, y = point
        steering = math.atan2(2.0 * self.wheelbase * y, lookahead * lookahead)
        return steering

    def _twist_steering(self, speed, yaw_rate):
        if self.angular_z_is_steering_angle:
            return yaw_rate, speed

        if abs(speed) <= 1e-4:
            if self.allow_rotate_crawl and abs(yaw_rate) > self.angular_deadband:
                speed = self.min_turn_speed
            else:
                yaw_rate = 0.0

        steering = 0.0
        if abs(speed) > 1e-4 and abs(yaw_rate) > self.angular_deadband:
            steering = math.atan(self.wheelbase * yaw_rate / speed)
        return steering, speed

    def _filter_steering(self, steering):
        steering = _clamp(
            steering,
            -self.planner_max_steering_angle,
            self.planner_max_steering_angle,
        )
        if self.steering_deadband > 1e-6:
            steering_abs = abs(steering)
            if self._steering_deadband_active:
                if steering_abs < self.steering_deadband_release:
                    steering = 0.0
                else:
                    self._steering_deadband_active = False
            elif steering_abs <= self.steering_deadband:
                steering = 0.0
                self._steering_deadband_active = True

        now = time.monotonic()
        if self.steering_rate_limit <= 1e-6 or self._last_steering_wall is None:
            self._last_steering = steering
            self._last_steering_wall = now
            return steering

        elapsed = max(0.0, now - self._last_steering_wall)
        max_delta = self.steering_rate_limit * elapsed
        steering = _clamp(
            steering,
            self._last_steering - max_delta,
            self._last_steering + max_delta,
        )
        self._last_steering = steering
        self._last_steering_wall = now
        return steering

    def callback(self, message):
        speed = _clamp(message.linear.x, -self.max_speed, self.max_speed)
        yaw_rate = message.angular.z

        if abs(message.linear.y) > 1e-4:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring cmd_vel linear.y %.3f for Ackermann drive",
                message.linear.y,
            )

        steering = self._local_plan_steering(speed)
        if steering is not None:
            if 0.0 <= speed < self.min_forward_tracking_speed:
                speed = min(self.min_forward_tracking_speed, self.max_speed)
        else:
            steering, speed = self._twist_steering(speed, yaw_rate)

        steering = self._filter_steering(steering)

        command = AckermannDriveStamped()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = self.frame_id
        command.drive.speed = speed
        command.drive.steering_angle = steering
        self.publisher.publish(command)


def main():
    rospy.init_node("twist_to_ackermann")
    TwistToAckermann()
    rospy.spin()


if __name__ == "__main__":
    main()
