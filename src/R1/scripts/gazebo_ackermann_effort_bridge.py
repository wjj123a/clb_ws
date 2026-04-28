#!/usr/bin/env python3

import math
import threading

import rospy
from gazebo_msgs.srv import ApplyJointEffort
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def shortest_angle_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class GazeboAckermannEffortBridge(object):
    def __init__(self):
        self.wheel_joint = rospy.get_param("~wheel_joint", "front_drive_wheel_joint")
        self.steer_joint = rospy.get_param("~steer_joint", "front_steer_joint")
        self.gazebo_wheel_joint = rospy.get_param("~gazebo_wheel_joint", "R1::" + self.wheel_joint)
        self.gazebo_steer_joint = rospy.get_param("~gazebo_steer_joint", "R1::" + self.steer_joint)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.04075)
        self.max_wheel_speed = rospy.get_param("~max_wheel_speed", 30.0)
        self.max_steer_angle = rospy.get_param("~max_steer_angle", 0.55)
        self.command_timeout = rospy.Duration(rospy.get_param("~command_timeout", 0.35))
        self.update_rate = rospy.get_param("~update_rate", 50.0)
        self.drive_effort_per_mps = rospy.get_param("~drive_effort_per_mps", 50.0)
        self.drive_kp = rospy.get_param("~drive_kp", 80.0)
        self.drive_damping = rospy.get_param("~drive_damping", 0.05)
        self.idle_drive_damping = rospy.get_param("~idle_drive_damping", 0.0)
        self.idle_linear_brake = rospy.get_param("~idle_linear_brake", 120.0)
        self.idle_linear_deadband = rospy.get_param("~idle_linear_deadband", 0.01)
        self.idle_drive_effort_limit = rospy.get_param("~idle_drive_effort_limit", 5.0)
        self.drive_effort_limit = rospy.get_param("~drive_effort_limit", 35.0)
        self.odom_timeout = rospy.Duration(rospy.get_param("~odom_timeout", 0.25))
        self.steer_kp = rospy.get_param("~steer_kp", 700.0)
        self.steer_kd = rospy.get_param("~steer_kd", 45.0)
        self.steer_effort_limit = rospy.get_param("~steer_effort_limit", 350.0)

        self.lock = threading.Lock()
        self.command = Twist()
        self.command_stamp = rospy.Time(0)
        self.joint_position = {}
        self.joint_velocity = {}
        self.measured_linear = 0.0
        self.measured_linear_stamp = rospy.Time(0)
        self.effort_duration = rospy.Duration(1.5 / self.update_rate)

        rospy.wait_for_service("/gazebo/apply_joint_effort")
        self.apply_effort = rospy.ServiceProxy(
            "/gazebo/apply_joint_effort",
            ApplyJointEffort,
            persistent=True,
        )

        rospy.Subscriber("cmd_vel", Twist, self.handle_cmd_vel, queue_size=1)
        rospy.Subscriber("joint_states", JointState, self.handle_joint_states, queue_size=10)
        rospy.Subscriber("ground_truth_odom", Odometry, self.handle_odom, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.update)

        rospy.loginfo(
            "gazebo_ackermann_effort_bridge applying effort to %s and %s",
            self.gazebo_wheel_joint,
            self.gazebo_steer_joint,
        )

    def handle_cmd_vel(self, message):
        with self.lock:
            self.command = message
            self.command_stamp = rospy.Time.now()

    def handle_joint_states(self, message):
        with self.lock:
            for index, name in enumerate(message.name):
                if index < len(message.position):
                    self.joint_position[name] = message.position[index]
                if index < len(message.velocity):
                    self.joint_velocity[name] = message.velocity[index]

    def handle_odom(self, message):
        linear = message.twist.twist.linear.x
        if not math.isfinite(linear):
            return
        with self.lock:
            self.measured_linear = linear
            self.measured_linear_stamp = rospy.Time.now()

    def update(self, _event):
        with self.lock:
            command = self.command
            command_stamp = self.command_stamp
            wheel_velocity = self.joint_velocity.get(self.wheel_joint)
            steer_position = self.joint_position.get(self.steer_joint)
            steer_velocity = self.joint_velocity.get(self.steer_joint, 0.0)
            measured_linear = self.measured_linear
            measured_linear_stamp = self.measured_linear_stamp

        if wheel_velocity is None or steer_position is None:
            return

        now = rospy.Time.now()
        command_active = command_stamp != rospy.Time(0) and now - command_stamp <= self.command_timeout
        odom_active = (
            measured_linear_stamp != rospy.Time(0)
            and now - measured_linear_stamp <= self.odom_timeout
        )
        if not odom_active:
            measured_linear = 0.0

        if not command_active:
            target_linear = 0.0
            target_steer = 0.0
        else:
            target_linear = clamp(
                command.linear.x,
                -self.max_wheel_speed * self.wheel_radius,
                self.max_wheel_speed * self.wheel_radius,
            )
            target_steer = clamp(command.angular.z, -self.max_steer_angle, self.max_steer_angle)

        if not all(
            math.isfinite(value)
            for value in (
                wheel_velocity,
                steer_position,
                steer_velocity,
                measured_linear,
                target_linear,
                target_steer,
            )
        ):
            return

        if command_active:
            linear_error = target_linear - measured_linear
            drive_effort = clamp(
                self.drive_effort_per_mps * target_linear
                + self.drive_kp * linear_error
                - self.drive_damping * wheel_velocity,
                -self.drive_effort_limit,
                self.drive_effort_limit,
            )
        else:
            linear_brake = 0.0
            if abs(measured_linear) > self.idle_linear_deadband:
                linear_brake = -self.idle_linear_brake * measured_linear
            drive_effort = clamp(
                linear_brake - self.idle_drive_damping * wheel_velocity,
                -self.idle_drive_effort_limit,
                self.idle_drive_effort_limit,
            )
        steer_error = shortest_angle_error(target_steer, steer_position)
        steer_effort = clamp(
            self.steer_kp * steer_error - self.steer_kd * steer_velocity,
            -self.steer_effort_limit,
            self.steer_effort_limit,
        )

        try:
            self.apply_effort(self.gazebo_wheel_joint, drive_effort, rospy.Time(0), self.effort_duration)
            self.apply_effort(self.gazebo_steer_joint, steer_effort, rospy.Time(0), self.effort_duration)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(1.0, "gazebo_ackermann_effort_bridge apply effort failed: %s", exc)


if __name__ == "__main__":
    rospy.init_node("gazebo_ackermann_effort_bridge")
    GazeboAckermannEffortBridge()
    rospy.spin()
