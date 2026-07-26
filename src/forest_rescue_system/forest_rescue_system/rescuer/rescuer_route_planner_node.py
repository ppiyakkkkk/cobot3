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
    astar_via_best_bridge,
    build_rescuer_grid_map,
    component_id_at,
    goal_cells_around_victim,
    nearest_walkable_cell,
    path_height_statistics,
    path_length_m,
    route_requires_bridge,
    simplify_grid_path,
    validate_bridge_usage,
    walkable_component_labels,
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
        self.declare_parameter("max_slope_deg", 45.0)
        self.declare_parameter("max_step_height_m", 1.25)
        self.declare_parameter("bridge_max_slope_deg", 60.0)
        self.declare_parameter("bridge_max_step_height_m", 1.5)
        self.declare_parameter("bridge_connector_max_length_m", 15.0)
        self.declare_parameter("bridge_connector_half_width_m", 4.5)
        self.declare_parameter("river_clearance_m", 0.75)
        self.declare_parameter("bridge_expansion_m", 1.5)
        self.declare_parameter("obstacle_clearance_m", 0.8)
        self.declare_parameter("platform_clearance_m", 1.0)
        self.declare_parameter("block_rocks", True)
        self.declare_parameter("block_vegetation", False)
        self.declare_parameter("goal_min_standoff_m", 1.2)
        self.declare_parameter("goal_max_standoff_m", 2.2)
        self.declare_parameter("slope_cost_weight", 1.5)
        self.declare_parameter("map_retry_period_sec", 1.0)
        self.declare_parameter("replan_position_change_m", 1.0)
        self.declare_parameter("fixed_bridge_enabled", True)
        self.declare_parameter("fixed_bridge_force_for_demo", False)
        self.declare_parameter("fixed_bridge_entry_xy", [7.0, 16.0])
        self.declare_parameter("fixed_bridge_exit_xy", [23.5, 6.5])
        self.declare_parameter("fixed_bridge_center_count", 3)
        self.declare_parameter("fixed_bridge_exit_clearance_m", 2.5)
        self.declare_parameter("fixed_bridge_forbidden_half_width_m", 2.0)
        self.declare_parameter("fixed_bridge_required_goal_xy", [21.0, 18.0])
        self.declare_parameter("fixed_bridge_required_radius_m", 4.0)
        self.declare_parameter("fixed_bridge_bypass_goal_xy", [-2.0, 35.0])
        self.declare_parameter("fixed_bridge_bypass_radius_m", 3.0)
        self.declare_parameter("fixed_bridge_terminal_search_radius_m", 15.0)
        self.declare_parameter("fixed_bridge_terminal_spacing_m", 0.75)

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
                max_step_height_m=float(
                    self.get_parameter("max_step_height_m").value
                ),
                bridge_max_slope_deg=float(
                    self.get_parameter("bridge_max_slope_deg").value
                ),
                bridge_max_step_height_m=float(
                    self.get_parameter("bridge_max_step_height_m").value
                ),
                bridge_connector_max_length_m=float(
                    self.get_parameter(
                        "bridge_connector_max_length_m"
                    ).value
                ),
                bridge_connector_half_width_m=float(
                    self.get_parameter(
                        "bridge_connector_half_width_m"
                    ).value
                ),
                river_clearance_m=float(
                    self.get_parameter("river_clearance_m").value
                ),
                bridge_expansion_m=float(
                    self.get_parameter("bridge_expansion_m").value
                ),
                obstacle_clearance_m=float(
                    self.get_parameter("obstacle_clearance_m").value
                ),
                platform_clearance_m=float(
                    self.get_parameter("platform_clearance_m").value
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
        bridge_core_count = int(
            np.count_nonzero(self.grid_map.bridge_core_mask)
        )
        bridge_access_count = int(
            np.count_nonzero(self.grid_map.bridge_access_mask)
        )
        bridge_connector_count = int(
            np.count_nonzero(self.grid_map.bridge_connector_mask)
        )
        self._publish_route_status("MAP_READY")
        self.get_logger().info(
            "구조자 보행 지도 준비 완료: "
            f"shape={self.grid_map.shape}, walkable={walkable_count}, "
            f"river={river_count}, bridge={bridge_count}, "
            f"bridge_core={bridge_core_count}, "
            f"bridge_access={bridge_access_count}, "
            f"bridge_connector={bridge_connector_count}, "
            f"max_slope={float(self.get_parameter('max_slope_deg').value):.1f}deg, "
            f"max_step={self.grid_map.max_step_height_m:.2f}m, "
            f"bridge_max_step="
            f"{self.grid_map.bridge_max_step_height_m:.2f}m"
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
                self.grid_map,
                requested_start,
                max_radius_cells=20,
                requested_z=float(start_position[2]),
                max_height_difference_m=max(
                    0.35,
                    float(self.grid_map.bridge_max_step_height_m),
                ),
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
            slope_cost_weight = float(
                self.get_parameter("slope_cost_weight").value
            )
            bridge_required = route_requires_bridge(
                self.grid_map, start_cell, goal_cells
            )
            bridge_bypass = self._fixed_bridge_bypass_requested(goal)
            fixed_bridge_goal = self._fixed_bridge_goal_requested(goal)
            effective_bridge_required = bool(
                (bridge_required or fixed_bridge_goal) and not bridge_bypass
            )
            use_fixed_bridge = bool(
                self.get_parameter("fixed_bridge_enabled").value
            ) and (
                effective_bridge_required
                or bool(
                    self.get_parameter(
                        "fixed_bridge_force_for_demo"
                    ).value
                )
            ) and not bridge_bypass
            if not use_fixed_bridge:
                self.get_logger().info(
                    "다리가 필요 없는 목표이므로 직접 A* 경로를 생성합니다: "
                    f"victim=({goal[0]:.2f}, {goal[1]:.2f}), "
                    f"explicit_bypass={bridge_bypass}, "
                    f"fixed_bridge_goal={fixed_bridge_goal}"
                )
            fixed_world_path = None
            if use_fixed_bridge:
                (
                    raw_path,
                    fixed_world_path,
                    maximum_slope,
                    maximum_step,
                ) = self._plan_via_fixed_bridge(
                    start_cell,
                    goal_cells,
                    goal,
                    max_standoff,
                    slope_cost_weight,
                )
                selected_bridge = 0
                simplified_path = None
                length_m = float(
                    np.sum(
                        np.linalg.norm(
                            np.diff(np.asarray(fixed_world_path), axis=0),
                            axis=1,
                        )
                    )
                )
            elif effective_bridge_required:
                raw_path, selected_bridge = astar_via_best_bridge(
                    self.grid_map,
                    start_cell,
                    goal_cells,
                    goal,
                    max_standoff_m=max_standoff,
                    slope_cost_weight=slope_cost_weight,
                )
            else:
                raw_path = self._plan_direct_without_bridge(
                    start_cell,
                    goal_cells,
                    goal,
                    max_standoff_m=max_standoff,
                    slope_cost_weight=slope_cost_weight,
                )
                _unused_required, selected_bridge = validate_bridge_usage(
                    self.grid_map, raw_path
                )
            if fixed_world_path is None:
                simplified_path = simplify_grid_path(self.grid_map, raw_path)
                length_m = path_length_m(self.grid_map, simplified_path)
                maximum_slope, maximum_step = path_height_statistics(
                    self.grid_map, raw_path
                )
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(f"구조자 경로 생성 실패: {error}")
            self._publish_route_status(f"PATH_FAILED:{error}")
            return

        path_message = (
            self._build_world_path_message(fixed_world_path)
            if fixed_world_path is not None
            else self._build_path_message(simplified_path)
        )
        self.path_publisher.publish(path_message)
        selected_message = String()
        if fixed_world_path is not None:
            selected_message.data = "fixed_Wooden_bridge2"
        else:
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
            "path_points": len(path_message.poses),
            "raw_grid_points": len(raw_path),
            "length_m": round(length_m, 3),
            "max_slope_deg": round(maximum_slope, 2),
            "max_step_height_m": round(maximum_step, 3),
            "bridge_required": bool(bridge_required),
            "fixed_bridge_goal": bool(fixed_bridge_goal),
            "bridge_bypass": bool(bridge_bypass),
            "selected_bridge": int(selected_bridge),
            "fixed_bridge_route": bool(fixed_world_path is not None),
        }
        self._publish_route_status(
            "PATH_READY:" + json.dumps(status_payload, ensure_ascii=False)
        )
        self.get_logger().warning(
            "구조자 경로 생성 완료: "
            f"points={len(path_message.poses)}, length={length_m:.1f}m, "
            f"max_slope={maximum_slope:.1f}deg, "
            f"max_step={maximum_step:.2f}m, "
            f"bridge_required={bridge_required}, "
            f"fixed_bridge_goal={fixed_bridge_goal}, "
            f"bridge_bypass={bridge_bypass}, "
            f"selected_bridge={selected_message.data}"
        )

    def _goal_is_within_configured_radius(
        self,
        victim_goal,
        xy_parameter,
        radius_parameter,
    ):
        """설정된 XY 목표 영역 안에 조난자가 있는지 확인한다."""
        target_xy = np.asarray(
            self.get_parameter(xy_parameter).value,
            dtype=np.float64,
        )
        if target_xy.shape != (2,) or not np.all(np.isfinite(target_xy)):
            return False
        radius = max(
            0.0,
            float(self.get_parameter(radius_parameter).value),
        )
        return bool(
            np.linalg.norm(
                np.asarray(victim_goal[:2], dtype=np.float64) - target_xy
            )
            <= radius
        )

    def _fixed_bridge_goal_requested(self, victim_goal):
        """고정 Wooden_bridge2를 사용하기로 지정한 목표 영역인지 확인한다."""
        return self._goal_is_within_configured_radius(
            victim_goal,
            "fixed_bridge_required_goal_xy",
            "fixed_bridge_required_radius_m",
        )

    def _fixed_bridge_bypass_requested(self, victim_goal):
        """다리를 사용하지 않기로 지정한 목표 영역인지 확인한다."""
        return self._goal_is_within_configured_radius(
            victim_goal,
            "fixed_bridge_bypass_goal_xy",
            "fixed_bridge_bypass_radius_m",
        )

    def _plan_direct_without_bridge(
        self,
        start_cell,
        goal_cells,
        victim_goal,
        *,
        max_standoff_m,
        slope_cost_weight,
    ):
        """다리가 필요 없는 목표는 다리 셀을 닫고 직접 A*로 계획한다."""
        direct_goals = {
            cell
            for cell in goal_cells
            if not bool(self.grid_map.bridge_access_mask[cell])
        }
        if not direct_goals:
            raise RuntimeError(
                "직접 경로 목표 주변에 다리가 아닌 보행 셀이 없습니다."
            )

        original_walkable = self.grid_map.walkable
        self.grid_map.walkable = (
            original_walkable & ~self.grid_map.bridge_access_mask
        )
        try:
            return astar_to_goal_set(
                self.grid_map,
                start_cell,
                direct_goals,
                victim_goal,
                max_standoff_m=max_standoff_m,
                slope_cost_weight=slope_cost_weight,
            )
        finally:
            self.grid_map.walkable = original_walkable

    def _plan_via_fixed_bridge(
        self,
        start_cell,
        goal_cells,
        victim_goal,
        max_standoff,
        slope_cost_weight,
    ):
        """입구 전·출구 후 A*를 고정 다리 중심선으로 연결한다."""
        entry_xy = [
            float(value)
            for value in self.get_parameter("fixed_bridge_entry_xy").value
        ]
        exit_xy = [
            float(value)
            for value in self.get_parameter("fixed_bridge_exit_xy").value
        ]
        if len(entry_xy) != 2 or len(exit_xy) != 2:
            raise ValueError("fixed bridge 입구/출구는 XY 두 값이어야 합니다.")

        entry_cell = nearest_walkable_cell(
            self.grid_map,
            self.grid_map.world_to_grid(*entry_xy),
            max_radius_cells=20,
        )
        exit_cell = nearest_walkable_cell(
            self.grid_map,
            self.grid_map.world_to_grid(*exit_xy),
            max_radius_cells=20,
        )
        entry_world = self.grid_map.grid_to_world(entry_cell).copy()
        exit_world = self.grid_map.grid_to_world(exit_cell).copy()
        # 사용자가 측정한 정확한 중심선 XY를 유지하고 Z만 지도에서 얻는다.
        entry_world[:2] = entry_xy
        exit_world[:2] = exit_xy

        approach = astar_to_goal_set(
            self.grid_map,
            start_cell,
            {entry_cell},
            entry_world,
            max_standoff_m=0.0,
            slope_cost_weight=slope_cost_weight,
        )

        # 출구에서 다리 진행 방향으로 먼저 완전히 빠져나간 뒤 A*를 시작한다.
        # 출구 셀을 곧바로 A* 시작점으로 쓰면 최근접 보행 셀이 다리 안쪽으로
        # 스냅되거나, 이후 경로가 다시 다리 중심부를 사용하는 경우가 있었다.
        bridge_direction = np.asarray(exit_xy, dtype=np.float64) - np.asarray(
            entry_xy, dtype=np.float64
        )
        bridge_length = float(np.linalg.norm(bridge_direction))
        if bridge_length <= 1.0e-6:
            raise ValueError("fixed bridge 입구와 출구가 같은 좌표입니다.")
        bridge_direction /= bridge_length
        exit_clearance_m = max(
            0.5,
            float(
                self.get_parameter("fixed_bridge_exit_clearance_m").value
            ),
        )
        recovery_xy = (
            np.asarray(exit_xy, dtype=np.float64)
            + bridge_direction * exit_clearance_m
        )
        forbidden_mask = self._fixed_bridge_forbidden_mask(
            np.asarray(entry_xy, dtype=np.float64),
            np.asarray(exit_xy, dtype=np.float64),
        )
        recovery_cell = self._nearest_non_bridge_walkable_cell(
            recovery_xy,
            max_radius_cells=24,
            forbidden_mask=forbidden_mask,
        )
        recovery_world = self.grid_map.grid_to_world(recovery_cell).copy()

        if forbidden_mask[recovery_cell]:
            raise RuntimeError(
                "다리 출구 바깥 안전 복귀점이 금지 구역 안에 있습니다: "
                f"recovery=({recovery_world[0]:.2f}, "
                f"{recovery_world[1]:.2f})"
            )

        original_walkable = self.grid_map.walkable
        self.grid_map.walkable = original_walkable & ~forbidden_mask
        try:
            (
                departure,
                terminal_world_points,
            ) = self._plan_bridge_departure(
                recovery_cell,
                goal_cells,
                victim_goal,
                max_standoff,
                slope_cost_weight,
            )
        finally:
            self.grid_map.walkable = original_walkable

        if any(bool(forbidden_mask[cell]) for cell in departure):
            raise RuntimeError(
                "출구 이후 경로가 다리 금지 구역으로 되돌아갑니다."
            )
        approach_simple = simplify_grid_path(self.grid_map, approach)
        departure_simple = simplify_grid_path(self.grid_map, departure)
        world_path = [
            self.grid_map.grid_to_world(cell).copy()
            for cell in approach_simple[:-1]
        ]
        world_path.append(entry_world)

        center_count = max(
            1,
            int(self.get_parameter("fixed_bridge_center_count").value),
        )
        for index in range(1, center_count + 1):
            ratio = index / float(center_count + 1)
            world_path.append(
                entry_world * (1.0 - ratio) + exit_world * ratio
            )
        world_path.append(exit_world)
        if float(np.linalg.norm(recovery_world[:2] - exit_world[:2])) >= 0.05:
            world_path.append(recovery_world)
        world_path.extend(
            self.grid_map.grid_to_world(cell).copy()
            for cell in departure_simple[1:]
        )
        for point in terminal_world_points:
            if (
                not world_path
                or float(
                    np.linalg.norm(point[:2] - world_path[-1][:2])
                )
                >= 0.05
            ):
                world_path.append(point)

        approach_slope, approach_step = path_height_statistics(
            self.grid_map, approach
        )
        departure_slope, departure_step = path_height_statistics(
            self.grid_map, departure
        )
        self.get_logger().warning(
            "Wooden_bridge2 고정 경로 연결: "
            f"entry=({entry_world[0]:.2f}, {entry_world[1]:.2f}, "
            f"{entry_world[2]:.2f}), "
            f"exit=({exit_world[0]:.2f}, {exit_world[1]:.2f}, "
            f"{exit_world[2]:.2f}), "
            f"recovery=({recovery_world[0]:.2f}, "
            f"{recovery_world[1]:.2f}, {recovery_world[2]:.2f}), "
            f"departure_points={len(departure_simple)}, "
            f"terminal_points={len(terminal_world_points)}, "
            f"victim=({victim_goal[0]:.2f}, {victim_goal[1]:.2f})"
        )
        return (
            approach + departure[1:],
            world_path,
            max(approach_slope, departure_slope),
            max(approach_step, departure_step),
        )

    def _plan_bridge_departure(
        self,
        recovery_cell,
        requested_goal_cells,
        victim_goal,
        max_standoff,
        slope_cost_weight,
    ):
        """출구에서 안전 A* 후 실제 조난자 좌표까지 마지막 경로를 잇는다."""
        usable_goals = {
            cell
            for cell in requested_goal_cells
            if (
                self.grid_map.in_bounds(cell)
                and bool(self.grid_map.walkable[cell])
            )
        }
        if usable_goals:
            try:
                departure = astar_to_goal_set(
                    self.grid_map,
                    recovery_cell,
                    usable_goals,
                    victim_goal,
                    max_standoff_m=max_standoff,
                    slope_cost_weight=slope_cost_weight,
                )
                return departure, []
            except RuntimeError:
                pass

        # 조난자 주변이 급경사·바위 마스크 등으로 모두 닫혀 있으면, A*로
        # 도달 가능한 가장 가까운 안전 지점까지 먼저 간다. 기존처럼 멀리
        # 떨어진 다리 셀을 최종 목표로 오인하지 않는다.
        search_limit = max(
            float(max_standoff) + 1.0,
            float(
                self.get_parameter(
                    "fixed_bridge_terminal_search_radius_m"
                ).value
            ),
        )
        grid_x, grid_y = np.meshgrid(
            self.grid_map.x_values,
            self.grid_map.y_values,
        )
        victim_distance = np.hypot(
            grid_x - float(victim_goal[0]),
            grid_y - float(victim_goal[1]),
        )
        component_labels = walkable_component_labels(self.grid_map)
        recovery_component = component_id_at(
            component_labels,
            recovery_cell,
        )
        rows, columns = np.where(
            component_labels == int(recovery_component)
        )
        if recovery_component <= 0 or len(rows) == 0:
            raise RuntimeError(
                "다리 출구에서 조난자 방향으로 연결되는 안전 A* 접근점을 "
                "찾지 못했습니다."
            )
        staging_candidates = [
            (
                float(victim_distance[row, column]),
                (int(row), int(column)),
            )
            for row, column in zip(rows, columns)
            if (int(row), int(column)) != recovery_cell
        ]
        if not staging_candidates:
            raise RuntimeError(
                "다리 출구 안전 복귀점에서 이동 가능한 Terrain 셀이 없습니다."
            )
        staging_candidates.sort(key=lambda item: item[0])
        used_radius, staging_cell = staging_candidates[0]
        if used_radius > search_limit:
            raise RuntimeError(
                "안전 A* 접근점과 조난자의 거리가 마지막 Terrain 추종 "
                f"허용값을 초과합니다: distance={used_radius:.2f}m, "
                f"limit={search_limit:.2f}m"
            )
        departure = astar_to_goal_set(
            self.grid_map,
            recovery_cell,
            {staging_cell},
            victim_goal,
            max_standoff_m=used_radius,
            slope_cost_weight=slope_cost_weight,
        )

        staging_world = self.grid_map.grid_to_world(departure[-1]).copy()
        terminal_points = self._sample_terminal_victim_path(
            staging_world,
            victim_goal,
        )
        self.get_logger().warning(
            "조난자 주변 엄격 보행 셀이 없어 마지막 Terrain 추종 경로를 "
            "연결합니다: "
            f"staging=({staging_world[0]:.2f}, "
            f"{staging_world[1]:.2f}), "
            f"search_radius={used_radius:.1f}m, "
            f"terminal_points={len(terminal_points)}"
        )
        return departure, terminal_points

    def _sample_terminal_victim_path(self, start_world, victim_goal):
        """안전 A* 끝점부터 실제 조난자 XY까지 촘촘한 지형 경로를 만든다."""
        start_xy = np.asarray(start_world[:2], dtype=np.float64)
        goal_xy = np.asarray(victim_goal[:2], dtype=np.float64)
        distance = float(np.linalg.norm(goal_xy - start_xy))
        spacing = max(
            0.25,
            float(
                self.get_parameter(
                    "fixed_bridge_terminal_spacing_m"
                ).value
            ),
        )
        count = max(1, int(math.ceil(distance / spacing)))
        points = []
        for index in range(1, count + 1):
            ratio = index / float(count)
            xy = start_xy * (1.0 - ratio) + goal_xy * ratio
            z = self._interpolated_navigation_z(
                float(xy[0]),
                float(xy[1]),
            )
            points.append(
                np.asarray([float(xy[0]), float(xy[1]), z])
            )
        return points

    def _interpolated_navigation_z(self, x, y):
        """격자점 사이 높이를 이중선형 보간해 큰 Z 계단을 만들지 않는다."""
        x_values = self.grid_map.x_values
        y_values = self.grid_map.y_values
        column_high = int(np.searchsorted(x_values, float(x), side="right"))
        row_high = int(np.searchsorted(y_values, float(y), side="right"))
        column_high = min(max(1, column_high), len(x_values) - 1)
        row_high = min(max(1, row_high), len(y_values) - 1)
        column_low = column_high - 1
        row_low = row_high - 1

        x0 = float(x_values[column_low])
        x1 = float(x_values[column_high])
        y0 = float(y_values[row_low])
        y1 = float(y_values[row_high])
        tx = 0.0 if x1 == x0 else (float(x) - x0) / (x1 - x0)
        ty = 0.0 if y1 == y0 else (float(y) - y0) / (y1 - y0)
        tx = float(np.clip(tx, 0.0, 1.0))
        ty = float(np.clip(ty, 0.0, 1.0))

        z00 = float(self.grid_map.z_grid[row_low, column_low])
        z10 = float(self.grid_map.z_grid[row_low, column_high])
        z01 = float(self.grid_map.z_grid[row_high, column_low])
        z11 = float(self.grid_map.z_grid[row_high, column_high])
        return (
            z00 * (1.0 - tx) * (1.0 - ty)
            + z10 * tx * (1.0 - ty)
            + z01 * (1.0 - tx) * ty
            + z11 * tx * ty
        )

    def _nearest_non_bridge_walkable_cell(
        self,
        requested_xy,
        *,
        max_radius_cells,
        forbidden_mask=None,
    ):
        """요청 좌표 주변에서 다리 영역이 아닌 보행 셀을 찾는다."""
        requested = self.grid_map.world_to_grid(
            float(requested_xy[0]),
            float(requested_xy[1]),
        )
        row0, column0 = requested
        for radius in range(0, int(max_radius_cells) + 1):
            candidates = []
            for row in range(row0 - radius, row0 + radius + 1):
                for column in range(
                    column0 - radius,
                    column0 + radius + 1,
                ):
                    cell = (row, column)
                    if not self.grid_map.in_bounds(cell):
                        continue
                    if not self.grid_map.walkable[cell]:
                        continue
                    if self.grid_map.bridge_access_mask[cell]:
                        continue
                    if (
                        forbidden_mask is not None
                        and bool(forbidden_mask[cell])
                    ):
                        continue
                    world = self.grid_map.grid_to_world(cell)
                    distance = float(
                        np.linalg.norm(world[:2] - requested_xy[:2])
                    )
                    candidates.append((distance, cell))
            if candidates:
                candidates.sort(key=lambda item: item[0])
                return candidates[0][1]
        raise RuntimeError(
            "다리 출구 주변에서 일반 Terrain 안전 복귀점을 찾지 못했습니다: "
            f"requested=({requested_xy[0]:.2f}, {requested_xy[1]:.2f})"
        )

    def _fixed_bridge_forbidden_mask(self, entry_xy, exit_xy):
        """출구 이후 A*가 다리 중심선·접속부로 복귀하지 못하게 막는다."""
        grid_x, grid_y = np.meshgrid(
            self.grid_map.x_values,
            self.grid_map.y_values,
        )
        segment = exit_xy - entry_xy
        length_squared = float(np.dot(segment, segment))
        if length_squared <= 1.0e-9:
            return self.grid_map.bridge_access_mask.copy()

        relative_x = grid_x - float(entry_xy[0])
        relative_y = grid_y - float(entry_xy[1])
        ratio = np.clip(
            (
                relative_x * float(segment[0])
                + relative_y * float(segment[1])
            )
            / length_squared,
            0.0,
            1.0,
        )
        nearest_x = float(entry_xy[0]) + ratio * float(segment[0])
        nearest_y = float(entry_xy[1]) + ratio * float(segment[1])
        distance = np.hypot(grid_x - nearest_x, grid_y - nearest_y)
        half_width = max(
            0.5,
            float(
                self.get_parameter(
                    "fixed_bridge_forbidden_half_width_m"
                ).value
            ),
        )
        return (
            self.grid_map.bridge_access_mask.copy()
            | (distance <= half_width)
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

    def _build_world_path_message(self, points):
        message = PathMessage()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        for world in points:
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
