#!/usr/bin/env python3

import copy
import math
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import LaserScan


def _command_nonzero(message):
    return abs(message.drive.speed) > 1e-4


def _zero_command(frame_id):
    command = AckermannDriveStamped()
    command.header.stamp = rospy.Time.now()
    command.header.frame_id = frame_id
    return command


def _wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_in_sector(angle, center, half_width):
    return abs(_wrap_to_pi(angle - center)) <= half_width


class AckermannCmdSafetySupervisor:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/ackermann_cmd_safety_in")
        self.output_topic = rospy.get_param("~output_topic", "/ackermann_cmd")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cancel_topic = rospy.get_param("~cancel_topic", "/move_base/cancel")
        self.frame_id = rospy.get_param("~frame_id", "base_footprint")

        self.front_angle = float(rospy.get_param("~front_angle", 0.65))
        self.rear_angle = float(rospy.get_param("~rear_angle", self.front_angle))
        self.hard_stop_distance = float(rospy.get_param("~hard_stop_distance", 0.45))
        self.slowdown_distance = float(rospy.get_param("~slowdown_distance", 0.85))
        self.reaction_time = float(rospy.get_param("~reaction_time", 0.35))
        self.max_deceleration = max(0.05, float(rospy.get_param("~max_deceleration", 0.45)))
        self.ttc_stop_time = float(rospy.get_param("~ttc_stop_time", 1.0))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.5))
        self.cancel_on_emergency = bool(rospy.get_param("~cancel_on_emergency", False))

        self._front_min = float("inf")
        self._rear_min = float("inf")
        self._front_closing_speed = 0.0
        self._rear_closing_speed = 0.0
        self._last_scan_wall = 0.0
        self._emergency_active = False

        self._publisher = rospy.Publisher(self.output_topic, AckermannDriveStamped, queue_size=1)
        self._cancel_publisher = rospy.Publisher(self.cancel_topic, GoalID, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber(self.input_topic, AckermannDriveStamped, self._cmd_callback, queue_size=1)
        rospy.loginfo(
            "ackermann safety supervisor forwarding %s to %s using %s",
            self.input_topic,
            self.output_topic,
            self.scan_topic,
        )

    @staticmethod
    def _closing_speed(previous_min, current_min, previous_wall, current_wall):
        if (
            not math.isfinite(previous_min)
            or not math.isfinite(current_min)
            or previous_wall <= 0.0
            or current_wall <= previous_wall
        ):
            return 0.0
        return max(0.0, (previous_min - current_min) / (current_wall - previous_wall))

    def _scan_callback(self, message):
        front_ranges = []
        rear_ranges = []
        for index, value in enumerate(message.ranges):
            if not math.isfinite(value) or value <= 0.0:
                continue
            angle = message.angle_min + index * message.angle_increment
            if _angle_in_sector(angle, 0.0, self.front_angle):
                front_ranges.append(value)
            if _angle_in_sector(angle, math.pi, self.rear_angle):
                rear_ranges.append(value)

        now = time.monotonic()
        current_front_min = min(front_ranges) if front_ranges else float("inf")
        current_rear_min = min(rear_ranges) if rear_ranges else float("inf")
        previous_front_min = self._front_min
        previous_rear_min = self._rear_min
        previous_wall = self._last_scan_wall

        self._front_min = current_front_min
        self._rear_min = current_rear_min
        self._front_closing_speed = self._closing_speed(
            previous_front_min,
            current_front_min,
            previous_wall,
            now,
        )
        self._rear_closing_speed = self._closing_speed(
            previous_rear_min,
            current_rear_min,
            previous_wall,
            now,
        )
        self._last_scan_wall = now

    def _scan_is_fresh(self):
        return self._last_scan_wall > 0.0 and time.monotonic() - self._last_scan_wall <= self.scan_timeout

    def _required_stop_distance(self, speed):
        return self.hard_stop_distance + speed * self.reaction_time + (speed * speed) / (2.0 * self.max_deceleration)

    def _time_to_collision(self, speed, obstacle_distance, closing_speed):
        closing_speed = max(0.0, speed, closing_speed)
        remaining_distance = obstacle_distance - self.hard_stop_distance
        if closing_speed <= 1e-3 or remaining_distance <= 0.0:
            return 0.0 if remaining_distance <= 0.0 else float("inf")
        return remaining_distance / closing_speed

    def _sector_for_speed(self, speed):
        if speed < -1e-4:
            return "rear", self._rear_min, self._rear_closing_speed
        return "front", self._front_min, self._front_closing_speed

    def _should_emergency_stop(self, command):
        if not self._scan_is_fresh():
            if _command_nonzero(command):
                rospy.logwarn_throttle(
                    2.0,
                    "Stopping ackermann_cmd because %s is stale",
                    self.scan_topic,
                )
            return _command_nonzero(command)

        speed = abs(command.drive.speed)
        if speed <= 1e-4:
            return False

        direction, obstacle_distance, closing_speed = self._sector_for_speed(command.drive.speed)
        if not math.isfinite(obstacle_distance):
            rospy.logwarn_throttle(
                2.0,
                "Stopping ackermann_cmd because %s sector has no valid ranges from %s",
                direction,
                self.scan_topic,
            )
            return True

        if obstacle_distance <= self.hard_stop_distance:
            rospy.logwarn_throttle(
                1.0,
                "Emergency stop: %s obstacle %.2fm <= hard stop %.2fm",
                direction,
                obstacle_distance,
                self.hard_stop_distance,
            )
            return True

        required_distance = self._required_stop_distance(speed)
        ttc = self._time_to_collision(speed, obstacle_distance, closing_speed)
        if obstacle_distance <= required_distance:
            rospy.logwarn_throttle(
                1.0,
                "Emergency stop: %s obstacle %.2fm <= required %.2fm",
                direction,
                obstacle_distance,
                required_distance,
            )
            return True
        if ttc <= self.ttc_stop_time:
            rospy.logwarn_throttle(
                1.0,
                "Emergency stop: %s obstacle TTC %.2fs <= %.2fs",
                direction,
                ttc,
                self.ttc_stop_time,
            )
            return True
        return False

    def _apply_slowdown(self, command):
        if abs(command.drive.speed) <= 1e-4 or not self._scan_is_fresh():
            return command
        _direction, obstacle_distance, _closing_speed = self._sector_for_speed(
            command.drive.speed
        )
        if obstacle_distance >= self.slowdown_distance:
            return command

        available = max(0.0, obstacle_distance - self.hard_stop_distance)
        span = max(0.05, self.slowdown_distance - self.hard_stop_distance)
        scale = max(0.0, min(1.0, available / span))
        limited = copy.deepcopy(command)
        limited.drive.speed *= scale
        return limited

    def _publish_emergency_stop(self):
        self._publisher.publish(_zero_command(self.frame_id))
        if self.cancel_on_emergency and not self._emergency_active:
            self._cancel_publisher.publish(GoalID())
        self._emergency_active = True

    def _cmd_callback(self, message):
        if self._should_emergency_stop(message):
            self._publish_emergency_stop()
            return

        command = self._apply_slowdown(message)
        self._emergency_active = False
        command.header.stamp = rospy.Time.now()
        if not command.header.frame_id:
            command.header.frame_id = self.frame_id
        self._publisher.publish(command)


def main():
    rospy.init_node("ackermann_cmd_safety_supervisor")
    AckermannCmdSafetySupervisor()
    rospy.spin()


if __name__ == "__main__":
    main()
