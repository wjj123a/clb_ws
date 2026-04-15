#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist


def clamp(value, limit):
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, value))


def slew_limit(current, target, accel_limit, dt):
    if accel_limit <= 0.0 or dt <= 0.0:
        return target
    max_delta = accel_limit * dt
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


class CmdVelLimiter:
    def __init__(self):
        self.max_linear_x = rospy.get_param("~max_linear_x", 0.15)
        self.max_linear_y = rospy.get_param("~max_linear_y", 0.0)
        self.max_angular_z = rospy.get_param("~max_angular_z", 0.35)
        self.max_linear_accel = rospy.get_param("~max_linear_accel", 0.25)
        self.max_angular_accel = rospy.get_param("~max_angular_accel", 0.60)
        self.timeout = rospy.get_param("~timeout", 0.40)
        publish_rate = rospy.get_param("~publish_rate", 30.0)

        self.target = Twist()
        self.current = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.last_update_time = rospy.Time.now()

        self.publisher = rospy.Publisher("cmd_vel_out", Twist, queue_size=1)
        self.subscriber = rospy.Subscriber("cmd_vel_in", Twist, self.handle_cmd, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / publish_rate), self.update)

        rospy.on_shutdown(self.publish_zero)

    def handle_cmd(self, message):
        self.last_cmd_time = rospy.Time.now()
        self.target.linear.x = clamp(message.linear.x, self.max_linear_x)
        self.target.linear.y = clamp(message.linear.y, self.max_linear_y)
        self.target.linear.z = 0.0
        self.target.angular.x = 0.0
        self.target.angular.y = 0.0
        self.target.angular.z = clamp(message.angular.z, self.max_angular_z)

    def update(self, _event):
        now = rospy.Time.now()
        dt = (now - self.last_update_time).to_sec()
        self.last_update_time = now

        timed_out = self.last_cmd_time == rospy.Time(0) or (now - self.last_cmd_time).to_sec() > self.timeout

        desired_linear_x = 0.0 if timed_out else self.target.linear.x
        desired_linear_y = 0.0 if timed_out else self.target.linear.y
        desired_angular_z = 0.0 if timed_out else self.target.angular.z

        limited = Twist()
        limited.linear.x = slew_limit(self.current.linear.x, desired_linear_x, self.max_linear_accel, dt)
        limited.linear.y = slew_limit(self.current.linear.y, desired_linear_y, self.max_linear_accel, dt)
        limited.linear.z = 0.0
        limited.angular.x = 0.0
        limited.angular.y = 0.0
        limited.angular.z = slew_limit(self.current.angular.z, desired_angular_z, self.max_angular_accel, dt)

        if math.isfinite(limited.linear.x) and math.isfinite(limited.linear.y) and math.isfinite(limited.angular.z):
            self.current = limited
            self.publisher.publish(limited)

    def publish_zero(self):
        self.publisher.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("cmd_vel_limiter")
    CmdVelLimiter()
    rospy.spin()
