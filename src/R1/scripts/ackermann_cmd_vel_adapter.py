#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class AckermannCmdVelAdapter(object):
    def __init__(self):
        self.wheelbase = rospy.get_param("~wheelbase", 0.41893)
        self.max_steer_angle = abs(rospy.get_param("~max_steer_angle", 0.55))
        self.min_turning_speed = abs(rospy.get_param("~min_turning_speed", 0.04))
        self.angular_deadband = abs(rospy.get_param("~angular_deadband", 1e-4))
        self.linear_deadband = abs(rospy.get_param("~linear_deadband", 1e-4))
        self.lateral_deadband = abs(rospy.get_param("~lateral_deadband", 1e-4))

        self.publisher = rospy.Publisher("cmd_vel_out", Twist, queue_size=1)
        self.subscriber = rospy.Subscriber("cmd_vel_in", Twist, self.handle_cmd, queue_size=1)

        rospy.loginfo(
            "ackermann_cmd_vel_adapter using wheelbase %.3fm, max steer %.3frad",
            self.wheelbase,
            self.max_steer_angle,
        )

    def handle_cmd(self, message):
        output = Twist()
        linear = message.linear.x
        yaw_rate = message.angular.z
        lateral = message.linear.y

        if abs(lateral) > self.lateral_deadband:
            rospy.logwarn_throttle(
                2.0,
                "ackermann_cmd_vel_adapter ignored lateral cmd_vel %.3fm/s",
                lateral,
            )

        if abs(yaw_rate) <= self.angular_deadband:
            output.linear.x = linear
            output.angular.z = 0.0
            self.publisher.publish(output)
            return

        if abs(linear) < self.min_turning_speed:
            direction = -1.0 if linear < -self.linear_deadband else 1.0
            linear = direction * self.min_turning_speed
            rospy.logwarn_throttle(
                2.0,
                "ackermann_cmd_vel_adapter converted near-zero spin command into a slow forward arc",
            )

        steer_angle = math.atan((self.wheelbase * yaw_rate) / linear)
        output.linear.x = linear
        output.angular.z = clamp(steer_angle, -self.max_steer_angle, self.max_steer_angle)
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("ackermann_cmd_vel_adapter")
    AckermannCmdVelAdapter()
    rospy.spin()
