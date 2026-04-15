#!/usr/bin/env python3

import math
import threading

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler


GOAL_STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def goal_status_name(state):
    return GOAL_STATUS_NAMES.get(state, "UNKNOWN")


class DockingController(object):
    def __init__(self):
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.start_srv = rospy.Service("start_docking", Trigger, self.start_docking)
        self.undock_srv = rospy.Service("undock", Trigger, self.undock)

        self.tf_listener = tf.TransformListener()

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.move_base_action = rospy.get_param("~move_base_action", "move_base")
        self.move_base_wait_timeout = rospy.get_param("~move_base_wait_timeout", 3.0)
        self.pose_timeout = rospy.get_param("~pose_timeout", 5.0)
        self.handoff_delay = rospy.get_param("~handoff_delay", 0.25)

        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action, MoveBaseAction
        )

        self.state_lock = threading.Lock()
        self.active_thread = None
        self.active_operation = None

        self.dock_x = rospy.get_param("~dock_x", 2.8)
        self.dock_y = rospy.get_param("~dock_y", 0.0)
        self.dock_yaw = rospy.get_param("~dock_yaw", math.pi)
        self.pre_dock_offset = rospy.get_param("~pre_dock_offset", 0.60)
        self.goal_timeout = rospy.get_param("~goal_timeout", 120.0)
        self.align_timeout = rospy.get_param("~align_timeout", 45.0)
        self.control_rate = rospy.get_param("~control_rate", 15.0)
        self.linear_gain = rospy.get_param("~linear_gain", 0.60)
        self.lateral_gain = rospy.get_param("~lateral_gain", 1.20)
        self.angular_gain = rospy.get_param("~angular_gain", 1.80)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.15)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.70)
        self.dock_tolerance_x = rospy.get_param("~dock_tolerance_x", 0.04)
        self.dock_tolerance_y = rospy.get_param("~dock_tolerance_y", 0.03)
        self.dock_tolerance_yaw = rospy.get_param("~dock_tolerance_yaw", 0.05)
        self.undock_distance = rospy.get_param("~undock_distance", 0.50)
        self.pre_dock_xy_tolerance = rospy.get_param("~pre_dock_xy_tolerance", 0.20)
        self.pre_dock_yaw_tolerance = rospy.get_param("~pre_dock_yaw_tolerance", 0.20)

        rospy.loginfo("Docking controller ready")

    def start_docking(self, _request):
        reserved, message = self.reserve_operation("docking")
        if not reserved:
            return TriggerResponse(success=False, message=message)

        if not self.ensure_navigation_ready():
            self.release_operation("docking")
            return TriggerResponse(
                success=False,
                message="move_base or map pose is not available",
            )

        self.active_thread = threading.Thread(
            target=self.run_docking,
            name="docking_thread",
        )
        self.active_thread.daemon = True
        self.active_thread.start()
        return TriggerResponse(success=True, message="Docking sequence started")

    def undock(self, _request):
        reserved, message = self.reserve_operation("undocking")
        if not reserved:
            return TriggerResponse(success=False, message=message)

        try:
            self.release_navigation_control()
            duration = self.undock_distance / max(self.max_linear_speed, 1e-3)
            self.drive_for_duration(-self.max_linear_speed, 0.0, duration)
            self.stop_robot()
            return TriggerResponse(success=True, message="Undock completed")
        finally:
            self.release_operation("undocking")

    def reserve_operation(self, operation_name):
        with self.state_lock:
            if self.active_operation is not None:
                return False, "%s already in progress" % self.active_operation
            self.active_operation = operation_name
            return True, ""

    def release_operation(self, operation_name):
        with self.state_lock:
            if self.active_operation == operation_name:
                self.active_operation = None
            if operation_name == "docking":
                self.active_thread = None

    def ensure_navigation_ready(self):
        if not self.move_base_client.wait_for_server(
            rospy.Duration(self.move_base_wait_timeout)
        ):
            rospy.logwarn(
                "Docking aborted: move_base action '%s' is not available",
                self.move_base_action,
            )
            return False

        if not self.wait_for_pose(self.map_frame, self.pose_timeout):
            rospy.logwarn(
                "Docking aborted: transform %s -> %s is not available",
                self.map_frame,
                self.base_frame,
            )
            return False

        return True

    def current_pose(self, frame_id):
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                frame_id, self.base_frame, rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            return None

        yaw = euler_from_quaternion(rotation)[2]
        return (translation[0], translation[1], yaw)

    def build_goal(self, x_pos, y_pos, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_pos
        goal.target_pose.pose.position.y = y_pos
        goal.target_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, yaw)
        )
        return goal

    def run_docking(self):
        try:
            pre_dock_x = self.dock_x - self.pre_dock_offset * math.cos(self.dock_yaw)
            pre_dock_y = self.dock_y - self.pre_dock_offset * math.sin(self.dock_yaw)
            rospy.loginfo("Sending pre-dock goal to (%.2f, %.2f)", pre_dock_x, pre_dock_y)

            goal = self.build_goal(pre_dock_x, pre_dock_y, self.dock_yaw)
            if not self.send_pre_dock_goal(goal):
                return

            if not self.wait_for_pre_dock(pre_dock_x, pre_dock_y, self.dock_yaw, 2.0):
                rospy.logwarn(
                    "Docking aborted: robot pose did not settle at the pre-dock target"
                )
                return

            self.release_navigation_control()

            rospy.loginfo("Pre-dock pose reached, starting fine alignment")
            if self.align_to_dock():
                rospy.loginfo("Docking alignment completed")
            else:
                rospy.logwarn("Docking alignment timed out")
        except Exception as exc:  # pragma: no cover - ROS runtime safety net
            rospy.logerr("Docking controller crashed: %s", exc)
        finally:
            self.release_navigation_control()
            self.stop_robot()
            self.release_operation("docking")

    def send_pre_dock_goal(self, goal):
        self.move_base_client.send_goal(goal)

        if not self.move_base_client.wait_for_result(rospy.Duration(self.goal_timeout)):
            self.move_base_client.cancel_all_goals()
            rospy.logwarn("Docking aborted: move_base pre-dock goal timed out")
            return False

        state = self.move_base_client.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn(
                "Docking aborted: move_base finished in state %s",
                goal_status_name(state),
            )
            return False

        return True

    def align_to_dock(self):
        deadline = rospy.Time.now() + rospy.Duration(self.align_timeout)
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            pose = self.current_pose(self.map_frame)
            if pose is None:
                rate.sleep()
                continue

            x_pos, y_pos, yaw = pose
            dx = self.dock_x - x_pos
            dy = self.dock_y - y_pos

            forward_error = math.cos(self.dock_yaw) * dx + math.sin(self.dock_yaw) * dy
            lateral_error = -math.sin(self.dock_yaw) * dx + math.cos(self.dock_yaw) * dy
            yaw_error = normalize_angle(self.dock_yaw - yaw)

            if (
                abs(forward_error) < self.dock_tolerance_x
                and abs(lateral_error) < self.dock_tolerance_y
                and abs(yaw_error) < self.dock_tolerance_yaw
            ):
                return True

            cmd = Twist()
            cmd.linear.x = clamp(
                self.linear_gain * forward_error,
                -self.max_linear_speed,
                self.max_linear_speed,
            )
            cmd.angular.z = clamp(
                self.angular_gain * yaw_error + self.lateral_gain * lateral_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.cmd_pub.publish(cmd)
            rate.sleep()

        return False

    def wait_for_pre_dock(self, goal_x, goal_y, goal_yaw, timeout_seconds):
        deadline = rospy.Time.now() + rospy.Duration(timeout_seconds)
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            pose = self.current_pose(self.map_frame)
            if pose is None:
                rate.sleep()
                continue

            x_pos, y_pos, yaw = pose
            xy_error = math.hypot(goal_x - x_pos, goal_y - y_pos)
            yaw_error = abs(normalize_angle(goal_yaw - yaw))

            if (
                xy_error < self.pre_dock_xy_tolerance
                and yaw_error < self.pre_dock_yaw_tolerance
            ):
                return True

            rate.sleep()

        return False

    def release_navigation_control(self):
        self.move_base_client.cancel_all_goals()
        self.stop_robot()
        rospy.sleep(self.handoff_delay)

    def drive_for_duration(self, linear_speed, angular_speed, duration):
        rate = rospy.Rate(self.control_rate)
        deadline = rospy.Time.now() + rospy.Duration(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            cmd = Twist()
            cmd.linear.x = linear_speed
            cmd.angular.z = angular_speed
            self.cmd_pub.publish(cmd)
            rate.sleep()

    def stop_robot(self):
        zero_cmd = Twist()
        publish_count = 3
        for index in range(publish_count):
            self.cmd_pub.publish(zero_cmd)
            if index + 1 < publish_count:
                rospy.sleep(1.0 / max(self.control_rate, 1.0))

    def wait_for_pose(self, frame_id, timeout_seconds):
        deadline = rospy.Time.now() + rospy.Duration(timeout_seconds)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.current_pose(frame_id) is not None:
                return True
            rate.sleep()
        return False


if __name__ == "__main__":
    rospy.init_node("docking_controller")
    DockingController()
    rospy.spin()
