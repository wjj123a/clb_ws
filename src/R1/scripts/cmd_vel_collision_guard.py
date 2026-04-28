#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class CmdVelCollisionGuard:
    def __init__(self):
        self.front_angle = rospy.get_param("~front_angle", 0.65)
        self.stop_distance = rospy.get_param("~stop_distance", 0.45)
        self.slow_distance = rospy.get_param("~slow_distance", 0.85)
        self.self_filter_distance = rospy.get_param("~self_filter_distance", 0.20)
        self.scan_timeout = rospy.get_param("~scan_timeout", 0.50)

        self.last_scan_time = rospy.Time(0)
        self.front_min_range = float("inf")

        self.publisher = rospy.Publisher("cmd_vel_out", Twist, queue_size=1)
        self.scan_subscriber = rospy.Subscriber("scan", LaserScan, self.handle_scan, queue_size=1)
        self.cmd_subscriber = rospy.Subscriber("cmd_vel_in", Twist, self.handle_cmd, queue_size=1)

        rospy.on_shutdown(self.publish_zero)

    def handle_scan(self, scan):
        min_range = float("inf")
        angle = scan.angle_min
        for distance in scan.ranges:
            if -self.front_angle <= angle <= self.front_angle and self._valid_range(scan, distance):
                min_range = min(min_range, distance)
            angle += scan.angle_increment

        self.front_min_range = min_range
        self.last_scan_time = rospy.Time.now()

    def _valid_range(self, scan, distance):
        if not math.isfinite(distance):
            return False
        if distance < scan.range_min:
            return False
        if distance < self.self_filter_distance:
            return False
        if distance > scan.range_max:
            return False
        return True

    def handle_cmd(self, cmd):
        guarded = Twist()
        guarded.linear.x = cmd.linear.x
        guarded.linear.y = cmd.linear.y
        guarded.linear.z = 0.0
        guarded.angular.x = 0.0
        guarded.angular.y = 0.0
        guarded.angular.z = cmd.angular.z

        if cmd.linear.x > 0.0:
            scale = self._forward_scale()
            if scale <= 0.0:
                guarded.linear.x = 0.0
                rospy.logwarn_throttle(
                    1.0,
                    "cmd_vel_collision_guard stopped forward motion; front obstacle %.2fm",
                    self.front_min_range,
                )
            elif scale < 1.0:
                guarded.linear.x = cmd.linear.x * scale
                rospy.logwarn_throttle(
                    1.0,
                    "cmd_vel_collision_guard slowing forward motion to %.0f%%; front obstacle %.2fm",
                    scale * 100.0,
                    self.front_min_range,
                )

        self.publisher.publish(guarded)

    def _forward_scale(self):
        if self.last_scan_time == rospy.Time(0):
            return 0.0

        scan_age = (rospy.Time.now() - self.last_scan_time).to_sec()
        if scan_age > self.scan_timeout:
            return 0.0

        if self.front_min_range <= self.stop_distance:
            return 0.0

        if self.front_min_range >= self.slow_distance:
            return 1.0

        span = self.slow_distance - self.stop_distance
        if span <= 0.0:
            return 1.0
        return clamp((self.front_min_range - self.stop_distance) / span, 0.0, 1.0)

    def publish_zero(self):
        self.publisher.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("cmd_vel_collision_guard")
    CmdVelCollisionGuard()
    rospy.spin()
