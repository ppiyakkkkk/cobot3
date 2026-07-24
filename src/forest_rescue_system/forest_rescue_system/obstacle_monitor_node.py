#!/usr/bin/env python3

"""360° LiDAR로 팽창 costmap과 로컬 A* 우회 경로를 만든다."""

import heapq
import math
import time

import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, String

from forest_rescue_system.log_utils import TimestampedNode


class ObstacleMonitorNode(TimestampedNode):
    def __init__(self):
        super().__init__("obstacle_monitor_node")

        self.declare_parameter("drone_id", "quadrotor_01")
        self.declare_parameter(
            "point_cloud_topic", "/quadrotor_01/point_cloud"
        )
        self.declare_parameter(
            "accumulated_point_cloud_topic",
            "/drone_01/obstacle/accumulated_cloud_body",
        )
        self.declare_parameter("use_accumulated_cloud_for_astar", True)
        self.declare_parameter("accumulated_cloud_max_age_sec", 0.8)
        self.declare_parameter(
            "local_astar_path_topic",
            "/drone_01/obstacle/local_astar_path",
        )
        self.declare_parameter(
            "blocked_topic", "/drone_01/obstacle/blocked"
        )
        self.declare_parameter(
            "distance_topic", "/drone_01/obstacle/min_distance"
        )
        self.declare_parameter(
            "planning_obstacle_distance_topic",
            "/drone_01/obstacle/planning_distance",
        )
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("safety_distance_m", 4.0)
        self.declare_parameter("emergency_distance_m", 1.2)
        self.declare_parameter("front_sector_half_angle_deg", 45.0)
        self.declare_parameter("side_sector_offset_deg", 70.0)
        self.declare_parameter("side_sector_half_angle_deg", 30.0)
        self.declare_parameter("minimum_obstacle_points", 3)
        self.declare_parameter("blocked_confirm_scans", 2)
        self.declare_parameter("clear_confirm_scans", 3)
        self.declare_parameter("processing_period_sec", 0.10)
        self.declare_parameter("minimum_height_m", -0.8)
        self.declare_parameter("maximum_height_m", 0.8)
        self.declare_parameter(
            "movement_direction_topic",
            "/drone_01/navigation/direction_body_rad",
        )
        self.declare_parameter(
            "clearances_topic",
            "/drone_01/obstacle/clearances",
        )
        self.declare_parameter(
            "avoidance_vector_topic",
            "/drone_01/obstacle/avoidance_vector",
        )
        self.declare_parameter(
            "local_detour_topic",
            "/drone_01/obstacle/local_detour_body",
        )
        self.declare_parameter(
            "candidate_offsets_deg",
            [0.0, -25.0, 25.0, -50.0, 50.0, -75.0, 75.0, -100.0, 100.0],
        )
        self.declare_parameter("candidate_sector_half_angle_deg", 15.0)
        self.declare_parameter("candidate_min_clearance_m", 3.5)
        self.declare_parameter("candidate_score_distance_cap_m", 15.0)
        self.declare_parameter("candidate_turn_penalty", 1.2)
        # 빈 공간만 넓다고 원래 목표에서 멀어지는 방향을 고르지 않도록
        # 목표 진행 성분과 이전 회피 방향 유지 성향을 점수에 반영한다.
        self.declare_parameter("candidate_forward_progress_weight", 3.0)
        self.declare_parameter("candidate_side_switch_penalty", 4.0)
        self.declare_parameter("candidate_side_hold_sec", 2.0)
        self.declare_parameter("candidate_max_offset_deg", 80.0)
        self.declare_parameter("local_grid_size_m", 30.0)
        self.declare_parameter("local_grid_resolution_m", 0.25)
        self.declare_parameter("obstacle_inflation_radius_m", 1.1)
        self.declare_parameter("local_planner_goal_distance_m", 10.0)
        self.declare_parameter("local_planner_lookahead_m", 3.5)
        self.declare_parameter("local_planner_period_sec", 0.40)
        # 누적 코스트맵에서 원래 진행 통로가 앞으로 막힐 것으로 보이면
        # 근거리 blocked 판정 전에도 A*를 선제적으로 요청한다.
        self.declare_parameter("proactive_avoidance_enabled", True)
        self.declare_parameter("proactive_planning_distance_m", 10.0)
        self.declare_parameter("proactive_corridor_half_width_m", 1.10)
        self.declare_parameter("proactive_ignore_near_m", 0.80)
        self.declare_parameter("proactive_min_obstacle_voxels", 2)
        self.declare_parameter("planner_start_release_radius_m", 0.50)
        # A* 경로의 첫 구간이 목표 반대편으로 과도하게 후퇴하면
        # 로컬 우회점으로 사용하지 않는다. 0은 측면 이동까지 허용한다.
        self.declare_parameter("local_planner_min_forward_progress_m", -0.10)
        self.declare_parameter(
            "active_mission_states",
            [
                "INITIAL_TAKEOFF",
                "INITIAL_HOVER",
                "READY",
                "SEARCHING",
                "VICTIM_DETECTED",
                "RETURNING_NO_VICTIM",
                "COMPLETE",
                "MISSION_FAILED",
            ],
        )
        self.declare_parameter("warning_period_sec", 10.0)

        self.blocked_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("blocked_topic").value),
            10,
        )
        self.distance_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("distance_topic").value),
            10,
        )
        self.planning_distance_publisher = self.create_publisher(
            Float32,
            str(
                self.get_parameter(
                    "planning_obstacle_distance_topic"
                ).value
            ),
            10,
        )
        self.clearances_publisher = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("clearances_topic").value),
            10,
        )
        self.avoidance_vector_publisher = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("avoidance_vector_topic").value),
            10,
        )
        self.local_detour_publisher = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("local_detour_topic").value),
            10,
        )
        self.local_astar_path_publisher = self.create_publisher(
            NavPath,
            str(self.get_parameter("local_astar_path_topic").value),
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("point_cloud_topic").value,
            self._point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("accumulated_point_cloud_topic").value),
            self._accumulated_point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("movement_direction_topic").value),
            self._movement_direction_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            self._mission_state_callback,
            10,
        )

        self.last_blocked = False
        self.blocked_scan_count = 0
        self.clear_scan_count = 0
        self.monitoring_enabled = False
        self.last_warning_time = float("-inf")
        self.last_processing_time = float("-inf")
        self.last_local_plan_time = float("-inf")
        self.local_detour_x_m = 0.0
        self.local_detour_y_m = 0.0
        self.local_detour_valid = False
        self.local_astar_path_body = []
        self.local_plan_source = "NONE"
        self.latest_accumulated_x = np.empty(0, dtype=np.float32)
        self.latest_accumulated_y = np.empty(0, dtype=np.float32)
        self.latest_accumulated_stamp_sec = float("-inf")
        self.latest_accumulated_receive_wall = float("-inf")
        # VFH가 매 스캔마다 좌우를 바꾸지 않도록 최근 회피 측을 기억한다.
        # +1은 목표 방향 기준 왼쪽, -1은 오른쪽이다.
        self.preferred_avoidance_side = 0
        self.preferred_avoidance_side_until = float("-inf")
        self.processing_period_sec = max(
            0.02,
            float(self.get_parameter("processing_period_sec").value),
        )
        # LiDAR body frame의 +X축을 0 rad로 사용한다. 컨트롤러가 다음
        # Waypoint 방향을 body frame 각도로 계속 갱신한다.
        self.movement_direction_rad = 0.0
        self.warning_period_sec = max(
            0.1,
            float(self.get_parameter("warning_period_sec").value),
        )
        self.active_mission_states = {
            str(state).upper()
            for state in self.get_parameter("active_mission_states").value
        }
        self.get_logger().info(
            f"{self.get_parameter('drone_id').value} LiDAR 장애물 감시 시작"
        )

    def _movement_direction_callback(self, message):
        new_direction = self._wrap_angle(float(message.data))
        if abs(self._wrap_angle(new_direction - self.movement_direction_rad)) > math.radians(8.0):
            self.local_detour_valid = False
            self.last_local_plan_time = float("-inf")
        self.movement_direction_rad = new_direction

    def _mission_state_callback(self, message):
        state = message.data.strip().upper()
        enabled = state in self.active_mission_states
        if enabled == self.monitoring_enabled:
            return

        self.monitoring_enabled = enabled
        self.last_blocked = False
        self.blocked_scan_count = 0
        self.clear_scan_count = 0
        self.preferred_avoidance_side = 0
        self.preferred_avoidance_side_until = float("-inf")
        self.local_astar_path_body = []
        self.local_plan_source = "NONE"
        self.latest_accumulated_x = np.empty(0, dtype=np.float32)
        self.latest_accumulated_y = np.empty(0, dtype=np.float32)
        self.latest_accumulated_stamp_sec = float("-inf")
        self.latest_accumulated_receive_wall = float("-inf")
        if enabled:
            self.get_logger().info(
                f"LiDAR 장애물 감시 활성화: mission_state={state}"
            )
        else:
            # 감시가 꺼질 때 이전 차단 상태가 남지 않게 해제한다.
            self._publish_result(
                float("inf"), float("inf"), float("inf"), False,
                allow_warning=False,
            )
            self.get_logger().info(
                f"LiDAR 장애물 감시 비활성화: mission_state={state}"
            )

    def _accumulated_point_cloud_callback(self, message):
        """현재 body frame으로 변환된 최근 누적 장애물 점군을 보관한다."""
        if not self.monitoring_enabled:
            return
        try:
            points = point_cloud2.read_points_numpy(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        except (AttributeError, ValueError):
            points = np.asarray(
                list(
                    point_cloud2.read_points(
                        message,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    )
                ),
                dtype=np.float32,
            )
        points = np.asarray(points, dtype=np.float32)
        if points.size == 0:
            self.latest_accumulated_x = np.empty(0, dtype=np.float32)
            self.latest_accumulated_y = np.empty(0, dtype=np.float32)
        else:
            points = points.reshape(-1, 3)
            finite = np.all(np.isfinite(points[:, :2]), axis=1)
            self.latest_accumulated_x = points[finite, 0].copy()
            self.latest_accumulated_y = points[finite, 1].copy()
        self.latest_accumulated_stamp_sec = self._stamp_to_seconds(
            message.header.stamp
        )
        self.latest_accumulated_receive_wall = time.monotonic()

    def _point_cloud_callback(self, message):
        if not self.monitoring_enabled:
            return
        # CPU 처리량 제한은 실제 시간, 경로 재계획과 회피 방향 유지시간은
        # LiDAR가 측정된 Isaac Sim 시간으로 각각 분리한다.
        wall_now = time.monotonic()
        if wall_now - self.last_processing_time < self.processing_period_sec:
            return
        self.last_processing_time = wall_now
        measurement_time = self._stamp_to_seconds(message.header.stamp)

        try:
            points = point_cloud2.read_points_numpy(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        except (AttributeError, ValueError):
            # 일부 Humble 버전에는 read_points_numpy가 없을 수 있다.
            points = np.asarray(
                list(
                    point_cloud2.read_points(
                        message,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    )
                ),
                dtype=np.float32,
            )

        points = np.asarray(points)
        if points.size == 0:
            self._publish_result(
                float("inf"), float("inf"), float("inf"), False
            )
            return
        points = points.reshape(-1, 3)

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        minimum_height = float(
            self.get_parameter("minimum_height_m").value
        )
        maximum_height = float(
            self.get_parameter("maximum_height_m").value
        )

        height_mask = (z >= minimum_height) & (z <= maximum_height)
        if not np.any(height_mask):
            self._publish_result(float("inf"), float("inf"), float("inf"), False)
            return

        # x>0 고정 필터를 제거하고 360도 포인트의 각도를 모두 계산한다.
        angles = np.arctan2(y[height_mask], x[height_mask])
        distances = np.hypot(x[height_mask], y[height_mask])
        center = self.movement_direction_rad
        front_half = math.radians(
            float(self.get_parameter("front_sector_half_angle_deg").value)
        )
        side_offset = math.radians(
            float(self.get_parameter("side_sector_offset_deg").value)
        )
        side_half = math.radians(
            float(self.get_parameter("side_sector_half_angle_deg").value)
        )

        front_mask = np.abs(self._angle_difference(angles, center)) <= front_half
        left_mask = (
            np.abs(self._angle_difference(angles, center + side_offset))
            <= side_half
        )
        right_mask = (
            np.abs(self._angle_difference(angles, center - side_offset))
            <= side_half
        )

        nearest_360_distance = float(np.min(distances))
        front_distance = self._minimum_distance(distances, front_mask)
        left_distance = self._minimum_distance(distances, left_mask)
        right_distance = self._minimum_distance(distances, right_mask)
        (
            recommended_direction_rad,
            recommended_clearance_m,
            recommended_valid,
        ) = self._select_avoidance_direction(
            angles,
            distances,
            center,
            measurement_time,
        )
        safety_distance = float(
            self.get_parameter("safety_distance_m").value
        )
        minimum_points = int(
            self.get_parameter("minimum_obstacle_points").value
        )
        close_front_points = int(
            np.count_nonzero(front_mask & (distances < safety_distance))
        )
        emergency_distance = float(
            self.get_parameter("emergency_distance_m").value
        )
        close_360_points = int(
            np.count_nonzero(distances < emergency_distance)
        )
        front_blocked = close_front_points >= minimum_points
        emergency_blocked = close_360_points >= minimum_points
        reactive_blocked = front_blocked or emergency_blocked
        (
            proactive_blocked,
            proactive_nearest_m,
            proactive_voxel_count,
        ) = self._proactive_corridor_check(center, measurement_time)

        # 누적 PointCloud의 선제 통로 판정은 경로계획에만 사용한다.
        # 오래 남은 voxel이나 시야 전환 흔적만으로 드론을 Hover시키지 않는다.
        # 실제 blocked 토픽은 현재 LiDAR의 전방 장애물 또는 360° 비상
        # 근접 장애물만으로 결정한다.
        if emergency_blocked:
            # 충돌 임박은 확인 스캔을 기다리지 않고 즉시 정지시킨다.
            self.last_blocked = True
            self.blocked_scan_count = max(
                self.blocked_scan_count,
                int(self.get_parameter("blocked_confirm_scans").value),
            )
            self.clear_scan_count = 0
            blocked = True
        else:
            blocked = self._apply_blocked_hysteresis(front_blocked)

        # 현재 스캔이 아직 blocked 확정 전이거나 누적맵만 통로를 예고해도
        # 로컬 A*는 미리 계산한다. 따라서 실제 근거리 장애물이 확인되는
        # 순간에는 이미 준비된 우회점을 사용할 수 있다.
        planning_requested = (
            blocked or reactive_blocked or proactive_blocked
        )
        if planning_requested:
            planner_period = float(
                self.get_parameter("local_planner_period_sec").value
            )
            if measurement_time < self.last_local_plan_time:
                # Isaac 재시작 등으로 시간이 되감기면 이전 실행의 제한값을 폐기한다.
                self.last_local_plan_time = float("-inf")
            if measurement_time - self.last_local_plan_time >= planner_period:
                obstacle_x = x[height_mask]
                obstacle_y = y[height_mask]
                self.local_plan_source = "latest_scan"
                if self._accumulated_cloud_is_fresh(measurement_time):
                    obstacle_x = self.latest_accumulated_x
                    obstacle_y = self.latest_accumulated_y
                    self.local_plan_source = "accumulated_3s"
                (
                    self.local_detour_x_m,
                    self.local_detour_y_m,
                    self.local_detour_valid,
                ) = self._plan_local_detour(
                    obstacle_x,
                    obstacle_y,
                    center,
                )
                self.last_local_plan_time = measurement_time
        else:
            self.local_detour_valid = False
            self.local_astar_path_body = []
            self.local_plan_source = "NONE"

        self._publish_result(
            front_distance,
            left_distance,
            right_distance,
            blocked,
            nearest_360_distance=nearest_360_distance,
            recommended_direction_rad=recommended_direction_rad,
            recommended_clearance_m=recommended_clearance_m,
            recommended_valid=recommended_valid,
            reactive_blocked=reactive_blocked,
            emergency_blocked=emergency_blocked,
            proactive_blocked=proactive_blocked,
            proactive_nearest_m=proactive_nearest_m,
            proactive_voxel_count=proactive_voxel_count,
        )

    def _proactive_corridor_check(self, center_angle, measurement_time):
        """누적 점군에서 앞으로 지나갈 통로가 막혔는지 확인한다.

        A*를 실제 충돌 직전에만 호출하지 않도록, 목표 진행축 기준의
        캡슐형 통로를 근사한 직사각형 영역 안에 확정 voxel이 있는지 본다.
        누적맵이 오래됐거나 준비되지 않았으면 선제 판정을 사용하지 않는다.
        """
        if not bool(
            self.get_parameter("proactive_avoidance_enabled").value
        ):
            return False, float("inf"), 0
        if not self._accumulated_cloud_is_fresh(measurement_time):
            return False, float("inf"), 0

        planning_distance = max(
            2.0,
            float(
                self.get_parameter("proactive_planning_distance_m").value
            ),
        )
        half_width = max(
            0.3,
            float(
                self.get_parameter("proactive_corridor_half_width_m").value
            ),
        )
        ignore_near = max(
            0.0,
            float(self.get_parameter("proactive_ignore_near_m").value),
        )
        minimum_voxels = max(
            1,
            int(
                self.get_parameter("proactive_min_obstacle_voxels").value
            ),
        )

        cos_center = math.cos(center_angle)
        sin_center = math.sin(center_angle)
        forward = (
            self.latest_accumulated_x * cos_center
            + self.latest_accumulated_y * sin_center
        )
        lateral = (
            -self.latest_accumulated_x * sin_center
            + self.latest_accumulated_y * cos_center
        )
        corridor_mask = (
            (forward >= ignore_near)
            & (forward <= planning_distance)
            & (np.abs(lateral) <= half_width)
        )
        voxel_count = int(np.count_nonzero(corridor_mask))
        if voxel_count <= 0:
            return False, float("inf"), 0
        nearest = float(np.min(forward[corridor_mask]))
        return voxel_count >= minimum_voxels, nearest, voxel_count

    def _accumulated_cloud_is_fresh(self, measurement_time):
        if not bool(
            self.get_parameter("use_accumulated_cloud_for_astar").value
        ):
            return False
        if self.latest_accumulated_x.size == 0:
            return False
        max_age = max(
            0.05,
            float(self.get_parameter("accumulated_cloud_max_age_sec").value),
        )
        sim_age = abs(
            float(measurement_time) - self.latest_accumulated_stamp_sec
        )
        wall_age = time.monotonic() - self.latest_accumulated_receive_wall
        return sim_age <= max_age and wall_age <= max_age * 2.0

    def _publish_local_astar_path(self, header):
        message = NavPath()
        message.header = header
        if self.local_detour_valid:
            for x_m, y_m in self.local_astar_path_body:
                pose = PoseStamped()
                pose.header = header
                pose.pose.position.x = float(x_m)
                pose.pose.position.y = float(y_m)
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                message.poses.append(pose)
        self.local_astar_path_publisher.publish(message)

    def _publish_result(
        self,
        front_distance,
        left_distance,
        right_distance,
        blocked,
        allow_warning=True,
        nearest_360_distance=float("inf"),
        recommended_direction_rad=0.0,
        recommended_clearance_m=0.0,
        recommended_valid=False,
        reactive_blocked=False,
        emergency_blocked=False,
        proactive_blocked=False,
        proactive_nearest_m=float("inf"),
        proactive_voxel_count=0,
    ):
        # 선제 통로에서 계산한 A* 경로는 blocked=False여도 유지한다.
        # 컨트롤러는 계속 수색하고, 실제 근거리 차단 시 이 최신 경로를 쓴다.
        if not blocked and not proactive_blocked:
            self.local_detour_valid = False
            self.local_astar_path_body = []
            self.local_plan_source = "NONE"
        distance_message = Float32()
        distance_message.data = front_distance
        self.distance_publisher.publish(distance_message)

        planning_distance_message = Float32()
        planning_distance_message.data = (
            float(proactive_nearest_m)
            if proactive_blocked
            else float("inf")
        )
        self.planning_distance_publisher.publish(planning_distance_message)

        clearances = Vector3Stamped()
        clearances.header.stamp = self.get_clock().now().to_msg()
        clearances.header.frame_id = str(
            self.get_parameter("drone_id").value
        ) + "/base_scan"
        clearances.vector.x = front_distance
        clearances.vector.y = left_distance
        clearances.vector.z = right_distance
        self.clearances_publisher.publish(clearances)

        avoidance = Vector3Stamped()
        avoidance.header = clearances.header
        # x: LiDAR body frame 기준 추천 진행각(rad)
        # y: 해당 후보 섹터의 실제 최소 여유거리(m)
        # z: 1.0이면 유효, 0.0이면 검증된 회피 방향 없음
        avoidance.vector.x = float(recommended_direction_rad)
        avoidance.vector.y = float(recommended_clearance_m)
        avoidance.vector.z = 1.0 if recommended_valid else 0.0
        self.avoidance_vector_publisher.publish(avoidance)

        local_detour = Vector3Stamped()
        local_detour.header = clearances.header
        # body frame 좌표: x=전방(m), y=좌측(m),
        # z의 절댓값=360° 최소거리(m), 부호=경로 유효 여부다.
        local_detour.vector.x = float(self.local_detour_x_m)
        local_detour.vector.y = float(self.local_detour_y_m)
        local_detour.vector.z = (
            float(nearest_360_distance)
            if self.local_detour_valid
            else -(float(nearest_360_distance) + 1.0e-6)
        )
        self.local_detour_publisher.publish(local_detour)
        self._publish_local_astar_path(clearances.header)

        blocked_message = Bool()
        blocked_message.data = blocked
        self.blocked_publisher.publish(blocked_message)

        now = time.monotonic()
        if (
            allow_warning
            and (blocked or proactive_blocked)
            and now - self.last_warning_time >= self.warning_period_sec
        ):
            recommendation = (
                f"{math.degrees(recommended_direction_rad):.0f}°/"
                f"{recommended_clearance_m:.2f}m"
                if recommended_valid
                else "NONE"
            )
            local_plan = (
                f"{self.local_plan_source}:"
                f"({self.local_detour_x_m:.2f}, "
                f"{self.local_detour_y_m:.2f})m"
                if self.local_detour_valid
                else "NONE"
            )
            if blocked:
                trigger_parts = []
                if emergency_blocked:
                    trigger_parts.append("비상근접")
                elif reactive_blocked:
                    trigger_parts.append("현재전방")
                if proactive_blocked:
                    trigger_parts.append(
                        f"선제통로={proactive_nearest_m:.2f}m/"
                        f"{int(proactive_voxel_count)}vox"
                    )
                trigger_text = (
                    "+".join(trigger_parts) if trigger_parts else "유지"
                )
                self.get_logger().warning(
                    f"이동 방향 장애물[{trigger_text}]: "
                    f"전방={front_distance:.2f}m, "
                    f"좌측={left_distance:.2f}m, "
                    f"우측={right_distance:.2f}m, "
                    f"360°최소={nearest_360_distance:.2f}m, "
                    f"VFH참고={recommendation}, 로컬A*={local_plan}"
                )
            else:
                self.get_logger().info(
                    "선제 통로 장애물 예고(정지하지 않음): "
                    f"거리={proactive_nearest_m:.2f}m/"
                    f"{int(proactive_voxel_count)}vox, "
                    f"VFH참고={recommendation}, 로컬A*={local_plan}"
                )
            self.last_warning_time = now
        self.last_blocked = blocked

    @staticmethod
    def _wrap_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _angle_difference(angles, center):
        return (angles - center + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _minimum_distance(distances, mask):
        if not np.any(mask):
            return float("inf")
        return float(np.min(distances[mask]))

    def _apply_blocked_hysteresis(self, raw_blocked):
        """한두 프레임의 LiDAR 잡음으로 회피 상태가 진동하지 않게 한다."""
        if raw_blocked:
            self.blocked_scan_count += 1
            self.clear_scan_count = 0
            if self.blocked_scan_count >= max(
                1,
                int(self.get_parameter("blocked_confirm_scans").value),
            ):
                return True
            return self.last_blocked

        self.clear_scan_count += 1
        self.blocked_scan_count = 0
        if self.clear_scan_count >= max(
            1,
            int(self.get_parameter("clear_confirm_scans").value),
        ):
            return False
        return self.last_blocked

    def _select_avoidance_direction(
        self,
        angles,
        distances,
        target_center,
        measurement_time,
    ):
        """목표 진행성과 좌우 일관성을 포함해 VFH 후보를 고른다.

        후보는 목표 방향을 중심으로 평가한다. 뒤쪽에 가까운 큰 회전은
        제외하고, 장애물 여유거리뿐 아니라 목표 방향으로 전진하는 성분과
        직전 회피 측을 유지하는 정도를 함께 반영한다.
        """
        offsets_deg = [
            float(value)
            for value in self.get_parameter("candidate_offsets_deg").value
        ]
        half_angle = math.radians(
            float(
                self.get_parameter("candidate_sector_half_angle_deg").value
            )
        )
        minimum_clearance = float(
            self.get_parameter("candidate_min_clearance_m").value
        )
        score_cap = float(
            self.get_parameter("candidate_score_distance_cap_m").value
        )
        turn_penalty = float(
            self.get_parameter("candidate_turn_penalty").value
        )
        forward_weight = float(
            self.get_parameter("candidate_forward_progress_weight").value
        )
        switch_penalty = float(
            self.get_parameter("candidate_side_switch_penalty").value
        )
        max_offset_deg = abs(
            float(self.get_parameter("candidate_max_offset_deg").value)
        )
        now = measurement_time
        side_hold_active = (
            self.preferred_avoidance_side != 0
            and now < self.preferred_avoidance_side_until
        )

        candidates = []
        for offset_deg in offsets_deg:
            if abs(offset_deg) > max_offset_deg:
                continue
            offset_rad = math.radians(offset_deg)
            candidate_angle = self._wrap_angle(target_center + offset_rad)
            mask = (
                np.abs(self._angle_difference(angles, candidate_angle))
                <= half_angle
            )
            clearance = self._minimum_distance(distances, mask)
            if math.isnan(clearance):
                continue
            effective_clearance = (
                score_cap if math.isinf(clearance) else clearance
            )
            if effective_clearance < minimum_clearance:
                continue

            candidate_side = 0
            if offset_deg > 1.0e-6:
                candidate_side = 1
            elif offset_deg < -1.0e-6:
                candidate_side = -1

            # cos(offset)는 목표 방향으로 실제 전진하는 비율이다.
            forward_progress = math.cos(offset_rad)
            side_change_cost = 0.0
            if (
                side_hold_active
                and candidate_side != 0
                and candidate_side != self.preferred_avoidance_side
            ):
                side_change_cost = switch_penalty

            score = (
                min(effective_clearance, score_cap)
                + forward_weight * forward_progress
                - turn_penalty * abs(offset_rad)
                - side_change_cost
            )
            candidates.append(
                (
                    score,
                    candidate_angle,
                    effective_clearance,
                    candidate_side,
                    abs(offset_rad),
                )
            )

        if not candidates:
            return 0.0, 0.0, False

        # 동일 점수면 회전량이 작은 후보를 우선한다.
        best = max(candidates, key=lambda item: (item[0], -item[4]))
        selected_side = int(best[3])
        if selected_side != 0:
            hold_sec = max(
                0.0,
                float(self.get_parameter("candidate_side_hold_sec").value),
            )
            self.preferred_avoidance_side = selected_side
            self.preferred_avoidance_side_until = now + hold_sec

        return float(best[1]), float(best[2]), True

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1.0e9

    def _plan_local_detour(self, obstacle_x, obstacle_y, target_angle):
        """팽창된 로컬 격자에서 목표 방향까지 A* 우회점을 구한다."""
        self.local_astar_path_body = []
        grid_size_m = float(self.get_parameter("local_grid_size_m").value)
        resolution = float(
            self.get_parameter("local_grid_resolution_m").value
        )
        if grid_size_m <= 2.0 or resolution <= 0.05:
            return 0.0, 0.0, False

        cell_count = max(21, int(round(grid_size_m / resolution)))
        if cell_count % 2 == 0:
            cell_count += 1
        center_cell = cell_count // 2
        half_size = center_cell * resolution
        occupied = np.zeros((cell_count, cell_count), dtype=bool)

        valid = (
            np.isfinite(obstacle_x)
            & np.isfinite(obstacle_y)
            & (np.abs(obstacle_x) <= half_size)
            & (np.abs(obstacle_y) <= half_size)
        )
        if np.any(valid):
            columns = np.rint(obstacle_x[valid] / resolution).astype(int) + center_cell
            rows = center_cell - np.rint(obstacle_y[valid] / resolution).astype(int)
            inside = (
                (rows >= 0) & (rows < cell_count)
                & (columns >= 0) & (columns < cell_count)
            )
            occupied[rows[inside], columns[inside]] = True

        occupied = self._inflate_grid(
            occupied,
            int(math.ceil(
                float(
                    self.get_parameter("obstacle_inflation_radius_m").value
                ) / resolution
            )),
        )
        start = (center_cell, center_cell)
        # 이미 장애물 팽창영역 가장자리에 들어온 경우 시작 셀까지 막히면
        # A*가 탈출 경로를 전혀 만들 수 없다. 실제 충돌 임박 여부는 별도의
        # emergency_distance_m으로 판정하므로, 드론 중심 주변의 작은 원만
        # 출발 가능 영역으로 복구해 장애물 반대편으로 빠져나오게 한다.
        release_radius_cells = max(
            0,
            int(math.floor(
                float(
                    self.get_parameter(
                        "planner_start_release_radius_m"
                    ).value
                ) / resolution
            )),
        )
        for row_offset in range(
            -release_radius_cells,
            release_radius_cells + 1,
        ):
            for column_offset in range(
                -release_radius_cells,
                release_radius_cells + 1,
            ):
                if (
                    row_offset ** 2 + column_offset ** 2
                    > release_radius_cells ** 2
                ):
                    continue
                occupied[
                    center_cell + row_offset,
                    center_cell + column_offset,
                ] = False

        goal_distance = min(
            float(
                self.get_parameter("local_planner_goal_distance_m").value
            ),
            half_size - resolution,
        )
        goal_x = math.cos(target_angle) * goal_distance
        goal_y = math.sin(target_angle) * goal_distance
        goal = (
            center_cell - int(round(goal_y / resolution)),
            center_cell + int(round(goal_x / resolution)),
        )
        goal = self._nearest_free_cell(occupied, goal, max_radius_cells=16)
        if goal is None:
            return 0.0, 0.0, False

        path = self._astar_grid(occupied, start, goal)
        if len(path) < 2:
            return 0.0, 0.0, False

        self.local_astar_path_body = [
            (
                (cell[1] - center_cell) * resolution,
                (center_cell - cell[0]) * resolution,
            )
            for cell in path
        ]

        lookahead = max(
            resolution,
            float(self.get_parameter("local_planner_lookahead_m").value),
        )

        # A*의 꺾인 경로에서 단순히 몇 번째 셀을 Position setpoint로
        # 보내면 PX4는 그 셀까지 직선으로 가면서 장애물 모서리를 자를 수
        # 있다. 시작 셀에서 직선 가시성이 확인되는 가장 먼 셀만 선택한다.
        selected = path[1]
        traveled = 0.0
        previous = path[0]
        for cell in path[1:]:
            traveled += math.hypot(
                cell[0] - previous[0],
                cell[1] - previous[1],
            ) * resolution
            if traveled > lookahead + 1.0e-9:
                break
            if self._grid_line_is_free(occupied, start, cell):
                selected = cell
            previous = cell

        detour_x = (selected[1] - center_cell) * resolution
        detour_y = (center_cell - selected[0]) * resolution
        detour_distance = math.hypot(detour_x, detour_y)
        if detour_distance < 0.5:
            self.local_astar_path_body = []
            return 0.0, 0.0, False

        target_forward_progress = (
            detour_x * math.cos(target_angle)
            + detour_y * math.sin(target_angle)
        )
        minimum_progress = float(
            self.get_parameter(
                "local_planner_min_forward_progress_m"
            ).value
        )
        if target_forward_progress < minimum_progress:
            self.local_astar_path_body = []
            return 0.0, 0.0, False

        return float(detour_x), float(detour_y), True

    @staticmethod
    def _grid_line_is_free(occupied, start, end):
        """두 격자 셀 사이 직선이 팽창 장애물을 지나지 않는지 검사한다."""
        row_delta = int(end[0]) - int(start[0])
        col_delta = int(end[1]) - int(start[1])
        step_count = max(abs(row_delta), abs(col_delta)) * 2 + 1
        rows, columns = occupied.shape
        for ratio in np.linspace(0.0, 1.0, max(2, step_count)):
            row = int(round(start[0] + row_delta * ratio))
            col = int(round(start[1] + col_delta * ratio))
            if not (0 <= row < rows and 0 <= col < columns):
                return False
            if occupied[row, col]:
                return False
        return True

    @staticmethod
    def _inflate_grid(occupied, radius_cells):
        if radius_cells <= 0 or not np.any(occupied):
            return occupied
        inflated = occupied.copy()
        rows, columns = occupied.shape
        for row_offset in range(-radius_cells, radius_cells + 1):
            for column_offset in range(-radius_cells, radius_cells + 1):
                if row_offset ** 2 + column_offset ** 2 > radius_cells ** 2:
                    continue
                source_row_start = max(0, -row_offset)
                source_row_end = min(rows, rows - row_offset)
                source_col_start = max(0, -column_offset)
                source_col_end = min(columns, columns - column_offset)
                target_row_start = source_row_start + row_offset
                target_row_end = source_row_end + row_offset
                target_col_start = source_col_start + column_offset
                target_col_end = source_col_end + column_offset
                inflated[
                    target_row_start:target_row_end,
                    target_col_start:target_col_end,
                ] |= occupied[
                    source_row_start:source_row_end,
                    source_col_start:source_col_end,
                ]
        return inflated

    @staticmethod
    def _nearest_free_cell(occupied, goal, max_radius_cells):
        rows, columns = occupied.shape
        goal_row = min(rows - 1, max(0, int(goal[0])))
        goal_col = min(columns - 1, max(0, int(goal[1])))
        if not occupied[goal_row, goal_col]:
            return goal_row, goal_col
        for radius in range(1, max_radius_cells + 1):
            candidates = []
            for row in range(max(0, goal_row - radius), min(rows, goal_row + radius + 1)):
                for col in range(max(0, goal_col - radius), min(columns, goal_col + radius + 1)):
                    if max(abs(row - goal_row), abs(col - goal_col)) != radius:
                        continue
                    if not occupied[row, col]:
                        candidates.append((row, col))
            if candidates:
                return min(
                    candidates,
                    key=lambda cell: (cell[0] - goal_row) ** 2 + (cell[1] - goal_col) ** 2,
                )
        return None

    @staticmethod
    def _astar_grid(occupied, start, goal):
        rows, columns = occupied.shape
        neighbors = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        )
        queue = [(0.0, start)]
        came_from = {}
        cost_so_far = {start: 0.0}
        while queue:
            _priority, current = heapq.heappop(queue)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            for row_delta, col_delta, step_cost in neighbors:
                neighbor = (current[0] + row_delta, current[1] + col_delta)
                if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < columns):
                    continue
                if occupied[neighbor]:
                    continue
                # 대각선으로 장애물 모서리를 가로지르지 않는다.
                if row_delta and col_delta:
                    if occupied[current[0] + row_delta, current[1]]:
                        continue
                    if occupied[current[0], current[1] + col_delta]:
                        continue
                new_cost = cost_so_far[current] + step_cost
                if new_cost >= cost_so_far.get(neighbor, float("inf")):
                    continue
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                heuristic = math.hypot(
                    goal[0] - neighbor[0],
                    goal[1] - neighbor[1],
                )
                heapq.heappush(queue, (new_cost + heuristic, neighbor))
        return []


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
