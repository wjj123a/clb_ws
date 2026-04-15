#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState


def main():
    rospy.init_node("static_joint_state_publisher")

    joint_names = rospy.get_param("~joint_names", ["J1", "J2", "J3", "J4"])
    rate_hz = rospy.get_param("~rate", 10.0)

    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rate = rospy.Rate(rate_hz)

    msg = JointState()
    msg.name = list(joint_names)
    msg.position = [0.0] * len(joint_names)
    msg.velocity = [0.0] * len(joint_names)
    msg.effort = [0.0] * len(joint_names)

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == "__main__":
    main()
