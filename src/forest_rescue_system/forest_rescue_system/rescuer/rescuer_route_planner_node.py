#!/usr/bin/env python3
"""확정된 조난자 위치까지 구조자 보행 경로를 생성한다."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from forest_rescue_system.common.log_utils import TimestampedNode
from forest_rescue_system.rescuer.rescuer_route_utils import (
    astar_to_goal_set,
    build_rescuer_grid_map,
    goal_cells_around_victim,
    nearest_walkable_cell,
    path_length_m,
    simplify_grid_path,
    validate_bridge_usage,
)


class RescuerRoutePlannerNode(TimestampedNode):
    """강은 막고 다리만 열어 둔 2D A* 경로를 발행한다."""

    def __init__(self):
        super().__init__("rescuer_route_planner_node")

        self.declare_parameter("operation_mode", "rescue_search")
        self.declare_parameter(
            "navigation_surface_path",
            "~/b3_cobot3_ws/isaac_sim/generated_navigation_surface.npz",
        )
        self.declare_parameter(
            "environment_mesh_path",
            "~/b3_cobot3_ws/isaac_sim/generated_environment_meshes.npz",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("victim_goal_topic", "/rescue/victim_goal")
        self.declare_parameter("rescuer_pose_topic", "/rescue/rescuer/position")
        self.declare_parameter("rescuer_path_topic", "/rescue/rescuer_path")
        self.declare_parameter(
            "route_status_topic", "/rescue/rescuer/route_status"
        )
        self.declare_parameter(
            "selected_bridge_topic", "/rescue/rescuer/selected_bridge"
        )
        self.declare_parameter(
            "route_marker_topic", "/rescue/rescuer_route_markers"
        )
        self.declare_parameter("max_slope_deg", 35.0)
        self.declare_parameter("river_clearance_m", 0.75)
        self.declare_parameter("bridge_expansion_m", 1.5)
        self.declare_parameter("obstacle_clearance_m", 0.8)
        self.declare_parameter("block_rocks", True)
        self.declare_parameter("block_vegetation", False)
        self.declare_parameter("goal_min_standoff_m", 1.2)
        self.declare_parameter("goal_max_standoff_m", 2.2)
        self.declare_parameter("slope_cost_weight", 1.5)
        self.declare_parameter("map_retry_period_sec", 1.0)
        self.declare_parameter("replan_position_change_m", 1.0)

        self.operation_mode = str(
            self.get_parameter("operation_mode").value
        ).strip().lower()
        if self.operation_mode != "rescue_search":
            raise RuntimeError(
                "rescuer_route_planner는 rescue_search에서만 실행할 수 "
                f"있습니다: {self.operation_mode!r}"
            )

        self.navigation_surface_path = Path(
            str(self.get_parameter("navigation_surface_path").value)
        ).expanduser()
        self.environment_mesh_path = Path(
            str(self.get_parameter("environment_mesh_path").value)
        ).expanduser()
        self.map_frame = str(self.get_parameter("map_frame").value)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path_publisher = self.create_publisher(
            PathMessage,
            str(self.get_parameter("rescuer_path_topic").value),
            transient_qos,
        )
        self.route_status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("route_status_topic").value),
            transient_qos,
        )
        self.selected_bridge_publisher = self.create_publisher(
            String,
            str(self.get_parameter("selected_bridge_topic").value),
            transient_qos,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("route_marker_topic").value),
            transient_qos,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("victim_goal_topic").value),
            self._victim_goal_callback,
            transient_qos,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("rescuer_pose_topic").value),
            self._rescuer_pose_callback,
            10,
        )

        self.grid_map = None
        self.latest_rescuer_pose = None
        self.pending_victim_goal = None
        self.last_planned_goal = None
        self.last_map_wait_log_at = float("-inf")
        self._publish_route_status("WAITING_FOR_MAP")

        retry_period = max(
            0.2, float(self.get_parameter("map_retry_period_sec").value)
        )
        self.map_timer = self.create_timer(retry_period, self._load_map_if_ready)
        self.get_logger().info(
            "구조자 경로계획 노드 시작: "
            f"navigation={self.navigation_surface_path}, "
            f"environment={self.environment_mesh_path}"
        )

    def _publish_route_status(self, status: str):
        message = String()
        message.data = str(status)
        self.route_status_publisher.publish(message)

    def _load_map_if_ready(self):
        if self.grid_map is not None:
            return
        if not self.navigation_surface_path.is_file() or not self.environment_mesh_path.is_file():
            now = time.monotonic()
            if now - self.last_map_wait_log_at >= 5.0:
                self.get_logger().info(
                    "구조자 지도 파일 대기 중: "
                    f"{self.navigation_surface_path}, {self.environment_mesh_path}"
                )
                self.last_map_wait_log_at = now
            return

        try:
            self.grid_map = build_rescuer_grid_map(
                self.navigation_surface_path,
                self.environment_mesh_path,
                max_slope_deg=float(self.get_parameter("max_slope_deg").value),
                river_clearance_m=float(
                    self.get_parameter("river_clearance_m").value
                ),
                bridge_expansion_m=float(
                    self.get_parameter("bridge_expansion_m").value
                ),
                obstacle_clearance_m=float(
                    self.get_parameter("obstacle_clearance_m").value
                ),
                block_rocks=bool(self.get_parameter("block_rocks").value),
                block_vegetation=bool(
                    self.get_parameter("block_vegetation").value
                ),
            )
        except (OSError, KeyError, TypeError, ValueError, RuntimeError) as error:
            self.get_logger().warning(
                f"구조자 보행 지도 생성 실패, 다음 주기에 재시도: {error}"
            )
            return

        walkable_count = int(np.count_nonzero(self.grid_map.walkable))
        river_count = int(np.count_nonzero(self.grid_map.river_mask))
        bridge_count = int(np.count_nonzero(self.grid_map.bridge_mask))
        self._publish_route_status("MAP_READY")
        self.get_logger().info(
            "구조자 보행 지도 준비 완료: "
            f"shape={self.grid_map.shape}, walkable={walkable_count}, "
            f"river={river_count}, bridge={bridge_count}"
        )
        self._try_plan()

    def _rescuer_pose_callback(self, message: PointStamped):
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                "구조자 위치 frame 불일치: "
                f"{message.header.frame_id} != {self.map_frame}"
            )
            return
        self.latest_rescuer_pose = np.asarray(
            [message.point.x, message.point.y, message.point.z],
            dtype=np.float64,
        )
        self._try_plan()

    def _victim_goal_callback(self, message: PointStamped):
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().error(
                "조난자 목표 frame 불일치: "
                f"{message.header.frame_id} != {self.map_frame}"
            )
            self._publish_route_status("PATH_FAILED:FRAME_MISMATCH")
            return
        goal = np.asarray(
            [message.point.x, message.point.y, message.point.z],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(goal)):
            self._publish_route_status("PATH_FAILED:INVALID_GOAL")
            return
        if self.last_planned_goal is not None:
            change = float(np.linalg.norm(goal - self.last_planned_goal))
            threshold = float(
                self.get_parameter("replan_position_change_m").value
            )
            if change < threshold:
                return
        self.pending_victim_goal = goal
        self._publish_route_status("PLANNING_PENDING")
        self.get_logger().warning(
            "확정 조난자 목표 수신: "
            f"({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f})"
        )
        self._try_plan()

    def _try_plan(self):
        if (
            self.grid_map is None
            or self.latest_rescuer_pose is None
            or self.pending_victim_goal is None
        ):
            return

        goal = self.pending_victim_goal.copy()
        start_position = self.latest_rescuer_pose.copy()
        self.pending_victim_goal = None
        self._publish_route_status("PLANNING")

        try:
            requested_start = self.grid_map.world_to_grid(
                start_position[0], start_position[1]
            )
            start_cell = nearest_walkable_cell(
                self.grid_map, requested_start, max_radius_cells=20
            )
            min_standoff = float(
                self.get_parameter("goal_min_standoff_m").value
            )
            max_standoff = float(
                self.get_parameter("goal_max_standoff_m").value
            )
            goal_cells = goal_cells_around_victim(
                self.grid_map,
                goal,
                min_standoff,
                max_standoff,
            )
            raw_path = astar_to_goal_set(
                self.grid_map,
                start_cell,
                goal_cells,
                goal,
                max_standoff_m=max_standoff,
                slope_cost_weight=float(
                    self.get_parameter("slope_cost_weight").value
                ),
            )
            simplified_path = simplify_grid_path(self.grid_map, raw_path)
            bridge_required, selected_bridge = validate_bridge_usage(
                self.grid_map, raw_path
            )
            length_m = path_length_m(self.grid_map, simplified_path)
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(f"구조자 경로 생성 실패: {error}")
            self._publish_route_status(f"PATH_FAILED:{error}")
            return

        path_message = self._build_path_message(simplified_path)
        self.path_publisher.publish(path_message)
        selected_message = String()
        selected_message.data = (
            f"bridge_component_{selected_bridge}"
            if selected_bridge > 0
            else "NONE"
        )
        self.selected_bridge_publisher.publish(selected_message)
        self.marker_publisher.publish(
            self._build_marker_array(
                start_cell,
                raw_path[-1],
                goal,
                selected_bridge,
            )
        )
        self.last_planned_goal = goal
        status_payload = {
            "state": "PATH_READY",
            "path_points": len(simplified_path),
            "raw_grid_points": len(raw_path),
            "length_m": round(length_m, 3),
            "bridge_required": bool(bridge_required),
            "selected_bridge": int(selected_bridge),
        }
        self._publish_route_status(
            "PATH_READY:" + json.dumps(status_payload, ensure_ascii=False)
        )
        self.get_logger().warning(
            "구조자 경로 생성 완료: "
            f"points={len(simplified_path)}, length={length_m:.1f}m, "
            f"bridge_required={bridge_required}, "
            f"selected_bridge={selected_message.data}"
        )

    def _build_path_message(self, cells):
        message = PathMessage()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        for cell in cells:
            world = self.grid_map.grid_to_world(cell)
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(world[0])
            pose.pose.position.y = float(world[1])
            pose.pose.position.z = float(world[2])
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    @staticmethod
    def _point(x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _build_marker_array(
        self,
        start_cell,
        goal_cell,
        victim_goal,
        selected_bridge,
    ):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        delete_all = Marker()
        delete_all.header.frame_id = self.map_frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        start = self.grid_map.grid_to_world(start_cell)
        goal = self.grid_map.grid_to_world(goal_cell)
        for marker_id, name, point, color in (
            (0, "rescuer_start", start, (1.0, 0.65, 0.0)),
            (1, "rescuer_goal", goal, (0.0, 1.0, 0.8)),
        ):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = name
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = float(point[2]) + 0.4
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.8
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            markers.markers.append(marker)

        victim_marker = Marker()
        victim_marker.header.frame_id = self.map_frame
        victim_marker.header.stamp = stamp
        victim_marker.ns = "rescuer_victim_goal"
        victim_marker.id = 2
        victim_marker.type = Marker.CYLINDER
        victim_marker.action = Marker.ADD
        victim_marker.pose.position.x = float(victim_goal[0])
        victim_marker.pose.position.y = float(victim_goal[1])
        victim_marker.pose.position.z = float(victim_goal[2]) + 0.05
        victim_marker.pose.orientation.w = 1.0
        victim_marker.scale.x = 4.4
        victim_marker.scale.y = 4.4
        victim_marker.scale.z = 0.1
        victim_marker.color.r = 1.0
        victim_marker.color.g = 0.1
        victim_marker.color.b = 0.1
        victim_marker.color.a = 0.25
        markers.markers.append(victim_marker)

        if selected_bridge > 0:
            cells = np.argwhere(
                self.grid_map.bridge_labels == int(selected_bridge)
            )
            bridge_marker = Marker()
            bridge_marker.header.frame_id = self.map_frame
            bridge_marker.header.stamp = stamp
            bridge_marker.ns = "selected_bridge"
            bridge_marker.id = 3
            bridge_marker.type = Marker.POINTS
            bridge_marker.action = Marker.ADD
            bridge_marker.pose.orientation.w = 1.0
            bridge_marker.scale.x = min(
                self.grid_map.spacing_x, self.grid_map.spacing_y
            ) * 0.7
            bridge_marker.scale.y = bridge_marker.scale.x
            bridge_marker.color.r = 1.0
            bridge_marker.color.g = 0.85
            bridge_marker.color.b = 0.0
            bridge_marker.color.a = 0.9
            bridge_marker.points = [
                self._point(*self.grid_map.grid_to_world((int(row), int(column))))
                for row, column in cells
            ]
            markers.markers.append(bridge_marker)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = RescuerRoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
