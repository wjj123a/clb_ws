#!/usr/bin/env python3

import math

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PlanarPoseGuard(object):
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "R1")
        self.reference_z = rospy.get_param("~reference_z", float("nan"))
        self.roll_threshold = rospy.get_param("~roll_threshold", 0.08)
        self.pitch_threshold = rospy.get_param("~pitch_threshold", 0.08)
        self.z_threshold = rospy.get_param("~z_threshold", 0.03)
        self.min_correction_interval = rospy.get_param("~min_correction_interval", 0.05)

        self.model_index = None
        self.last_correction_time = rospy.Time(0)
        self.set_model_state = None

        rospy.loginfo("planar_pose_guard waiting for /gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.handle_model_states, queue_size=1)
        rospy.loginfo("planar_pose_guard guarding model %s", self.model_name)

    def handle_model_states(self, message):
        if self.model_index is None:
            try:
                self.model_index = message.name.index(self.model_name)
            except ValueError:
                return

        pose = message.pose[self.model_index]
        twist = message.twist[self.model_index]

        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        if not math.isfinite(self.reference_z):
            self.reference_z = pose.position.z

        need_correction = (
            abs(roll) > self.roll_threshold
            or abs(pitch) > self.pitch_threshold
            or abs(pose.position.z - self.reference_z) > self.z_threshold
        )

        if not need_correction:
            return

        now = rospy.Time.now()
        if (now - self.last_correction_time).to_sec() < self.min_correction_interval:
            return

        planar_orientation = quaternion_from_euler(0.0, 0.0, yaw)

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = pose.position.x
        state.pose.position.y = pose.position.y
        state.pose.position.z = self.reference_z
        state.pose.orientation.x = planar_orientation[0]
        state.pose.orientation.y = planar_orientation[1]
        state.pose.orientation.z = planar_orientation[2]
        state.pose.orientation.w = planar_orientation[3]
        state.twist.linear.x = twist.linear.x
        state.twist.linear.y = twist.linear.y
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = twist.angular.z

        try:
            self.set_model_state(state)
            self.last_correction_time = now
            rospy.logwarn_throttle(
                2.0,
                "planar_pose_guard corrected roll=%.3f pitch=%.3f z=%.3f",
                roll,
                pitch,
                pose.position.z,
            )
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "planar_pose_guard failed to correct pose: %s", exc)


if __name__ == "__main__":
    rospy.init_node("planar_pose_guard")
    PlanarPoseGuard()
    rospy.spin()
