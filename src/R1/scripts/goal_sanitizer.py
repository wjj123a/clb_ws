#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf2_ros


class GoalSanitizer:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.input_topic = rospy.get_param("~input_topic", "/move_base_simple/goal")
        self.output_topic = rospy.get_param(
            "~output_topic", "/move_base_simple/goal_sanitized"
        )
        self.occupancy_threshold = rospy.get_param("~occupancy_threshold", 50)
        self.clearance_radius = rospy.get_param("~clearance_radius", 0.38)
        self.search_radius = rospy.get_param("~search_radius", 0.80)
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_footprint")
        self.arrival_radius = rospy.get_param("~arrival_radius", 0.38)
        self.sanitize_min_shift = rospy.get_param("~sanitize_min_shift", 0.05)
        self.approach_clearance_radius = rospy.get_param("~approach_clearance_radius", 0.24)
        self.robot_distance_weight = rospy.get_param("~robot_distance_weight", 0.20)
        self.blocked_approach_penalty = rospy.get_param("~blocked_approach_penalty", 2.0)
        self.open_space_weight = rospy.get_param("~open_space_weight", 0.80)
        self.open_space_radius = rospy.get_param("~open_space_radius", 0.80)

        self.map_msg = None
        self.cached_offsets = []
        self.cached_resolution = None
        self.active_goal = None
        self.completion_sent = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.goal_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=5)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb, queue_size=1)
        rospy.Subscriber(self.input_topic, PoseStamped, self._goal_cb, queue_size=5)
        rospy.Timer(rospy.Duration(0.1), self._finish_goal_if_needed)

    def _map_cb(self, msg):
        self.map_msg = msg
        if self.cached_resolution != msg.info.resolution:
            self.cached_resolution = msg.info.resolution
            self.cached_offsets = self._build_offsets(msg.info.resolution)

    def _build_offsets(self, resolution):
        max_cells = int(math.ceil(self.search_radius / resolution))
        offsets = []
        for dx in range(-max_cells, max_cells + 1):
            for dy in range(-max_cells, max_cells + 1):
                dist = math.hypot(dx, dy)
                if dist <= max_cells:
                    offsets.append((dist, dx, dy))
        offsets.sort(key=lambda item: item[0])
        return offsets

    def _goal_cb(self, goal):
        if self.map_msg is None:
            rospy.logwarn_throttle(5.0, "goal_sanitizer waiting for map; forwarding raw goal")
            self.goal_pub.publish(goal)
            return

        if goal.header.frame_id != self.map_msg.header.frame_id:
            rospy.logwarn_throttle(
                5.0,
                "goal_sanitizer only adjusts goals in %s; forwarding %s",
                self.map_msg.header.frame_id,
                goal.header.frame_id,
            )
            self.goal_pub.publish(goal)
            return

        mx, my = self._world_to_map(goal.pose.position.x, goal.pose.position.y)
        if mx is None:
            rospy.logwarn("goal_sanitizer received out-of-map goal; forwarding raw goal")
            self.goal_pub.publish(goal)
            return

        robot_cell = self._get_robot_map_cell(goal.header.frame_id)
        best = self._find_radial_escape_cell(mx, my, robot_cell)
        if best is None:
            best = self._find_safe_cell(mx, my, robot_cell)
        if best is None:
            rospy.logwarn("goal_sanitizer could not find a safe nearby goal; forwarding raw goal")
            self.goal_pub.publish(goal)
            return

        safe_goal = PoseStamped()
        safe_goal.header = goal.header
        safe_goal.pose = goal.pose
        safe_goal.pose.position.x, safe_goal.pose.position.y = self._map_to_world(*best)

        moved = math.hypot(
            safe_goal.pose.position.x - goal.pose.position.x,
            safe_goal.pose.position.y - goal.pose.position.y,
        )
        if moved > 0.02:
            rospy.loginfo(
                "goal_sanitizer adjusted goal from (%.2f, %.2f) to (%.2f, %.2f)",
                goal.pose.position.x,
                goal.pose.position.y,
                safe_goal.pose.position.x,
                safe_goal.pose.position.y,
            )

        if moved >= self.sanitize_min_shift:
            self.active_goal = safe_goal
            self.completion_sent = False
        else:
            self.active_goal = None
            self.completion_sent = False

        self.goal_pub.publish(safe_goal)

    def _finish_goal_if_needed(self, _event):
        if self.active_goal is None or self.completion_sent:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.active_goal.header.frame_id,
                self.base_frame_id,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception:
            return

        dx = self.active_goal.pose.position.x - transform.transform.translation.x
        dy = self.active_goal.pose.position.y - transform.transform.translation.y
        if math.hypot(dx, dy) > self.arrival_radius:
            return

        completion_goal = PoseStamped()
        completion_goal.header.frame_id = self.active_goal.header.frame_id
        completion_goal.header.stamp = rospy.Time.now()
        completion_goal.pose.position.x = transform.transform.translation.x
        completion_goal.pose.position.y = transform.transform.translation.y
        completion_goal.pose.position.z = transform.transform.translation.z
        completion_goal.pose.orientation = transform.transform.rotation

        rospy.loginfo(
            "goal_sanitizer finalizing sanitized goal at current pose (%.2f, %.2f)",
            completion_goal.pose.position.x,
            completion_goal.pose.position.y,
        )
        self.goal_pub.publish(completion_goal)
        self.completion_sent = True

    def _get_robot_map_cell(self, goal_frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                goal_frame_id,
                self.base_frame_id,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception:
            return None

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        rx, ry = self._world_to_map(robot_x, robot_y)
        if rx is None:
            return None
        return rx, ry

    def _find_safe_cell(self, mx, my, robot_cell=None):
        best_score = None
        best_cell = None
        resolution = self.map_msg.info.resolution
        approach_cells = max(
            1, int(math.ceil(self.approach_clearance_radius / resolution))
        )
        for _, dx, dy in self.cached_offsets:
            candidate_x = mx + dx
            candidate_y = my + dy
            if not self._cell_has_clearance(candidate_x, candidate_y):
                continue

            score = math.hypot(dx, dy) * resolution
            if robot_cell is not None:
                rx, ry = robot_cell
                score += (
                    math.hypot(candidate_x - rx, candidate_y - ry)
                    * resolution
                    * self.robot_distance_weight
                )
                if not self._line_has_clearance(
                    rx, ry, candidate_x, candidate_y, approach_cells
                ):
                    score += self.blocked_approach_penalty

            score -= self._orthogonal_clearance(candidate_x, candidate_y) * self.open_space_weight

            if best_score is None or score < best_score:
                best_score = score
                best_cell = (candidate_x, candidate_y)
        return best_cell

    def _find_radial_escape_cell(self, mx, my, robot_cell=None):
        info = self.map_msg.info
        max_cells = int(math.ceil(self.search_radius / info.resolution))
        nearest_occupied = None
        for dist, dx, dy in self.cached_offsets:
            if dist > max_cells:
                break
            cx = mx + dx
            cy = my + dy
            if not (0 <= cx < info.width and 0 <= cy < info.height):
                continue
            cost = self.map_msg.data[cy * info.width + cx]
            if cost < 0 or cost >= self.occupancy_threshold:
                nearest_occupied = (cx, cy)
                break

        if nearest_occupied is None:
            return None

        occ_x, occ_y = nearest_occupied
        step_x = float(mx - occ_x)
        step_y = float(my - occ_y)
        if abs(step_x) < 1e-6 and abs(step_y) < 1e-6:
            if robot_cell is None:
                return None
            step_x = float(mx - robot_cell[0])
            step_y = float(my - robot_cell[1])
            if abs(step_x) < 1e-6 and abs(step_y) < 1e-6:
                return None

        length = math.hypot(step_x, step_y)
        unit_x = step_x / length
        unit_y = step_y / length
        for step in range(1, max_cells + 1):
            candidate_x = int(round(mx + unit_x * step))
            candidate_y = int(round(my + unit_y * step))
            if self._cell_has_clearance(candidate_x, candidate_y):
                return candidate_x, candidate_y
        return None

    def _cell_has_clearance(self, mx, my, clearance_cells=None):
        info = self.map_msg.info
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return False

        center_cost = self.map_msg.data[my * info.width + mx]
        if center_cost < 0 or center_cost >= self.occupancy_threshold:
            return False

        if clearance_cells is None:
            clearance_cells = int(math.ceil(self.clearance_radius / info.resolution))
        for dx in range(-clearance_cells, clearance_cells + 1):
            for dy in range(-clearance_cells, clearance_cells + 1):
                if math.hypot(dx, dy) > clearance_cells:
                    continue
                cx = mx + dx
                cy = my + dy
                if not (0 <= cx < info.width and 0 <= cy < info.height):
                    return False
                cost = self.map_msg.data[cy * info.width + cx]
                if cost < 0 or cost >= self.occupancy_threshold:
                    return False
        return True

    def _line_has_clearance(self, start_x, start_y, end_x, end_y, clearance_cells):
        steps = max(abs(end_x - start_x), abs(end_y - start_y)) * 2 + 1
        for i in range(steps + 1):
            ratio = float(i) / float(steps)
            mx = int(round(start_x + (end_x - start_x) * ratio))
            my = int(round(start_y + (end_y - start_y) * ratio))
            if not self._cell_has_clearance(mx, my, clearance_cells):
                return False
        return True

    def _orthogonal_clearance(self, mx, my):
        info = self.map_msg.info
        max_cells = max(1, int(math.ceil(self.open_space_radius / info.resolution)))
        distances = []
        for step_x, step_y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            distance = 0.0
            for step in range(1, max_cells + 1):
                cx = mx + step_x * step
                cy = my + step_y * step
                if not (0 <= cx < info.width and 0 <= cy < info.height):
                    break
                cost = self.map_msg.data[cy * info.width + cx]
                if cost < 0 or cost >= self.occupancy_threshold:
                    break
                distance = step * info.resolution
            distances.append(distance)
        return min(distances) if distances else 0.0

    def _world_to_map(self, x, y):
        info = self.map_msg.info
        mx = int(math.floor((x - info.origin.position.x) / info.resolution))
        my = int(math.floor((y - info.origin.position.y) / info.resolution))
        if 0 <= mx < info.width and 0 <= my < info.height:
            return mx, my
        return None, None

    def _map_to_world(self, mx, my):
        info = self.map_msg.info
        x = info.origin.position.x + (mx + 0.5) * info.resolution
        y = info.origin.position.y + (my + 0.5) * info.resolution
        return x, y


if __name__ == "__main__":
    rospy.init_node("goal_sanitizer")
    GoalSanitizer()
    rospy.spin()
