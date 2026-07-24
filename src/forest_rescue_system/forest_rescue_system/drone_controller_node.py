#!/usr/bin/env python3

"""드론 한 대의 ROS 2 명령을 MAVSDK/PX4 Offboard 제어로 변환한다."""

import asyncio
import json
import math
from pathlib import Path
import threading

import numpy as np
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    TransformStamped,
    Vector3Stamped,
)
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.param import ParamError
from mavsdk.telemetry import TelemetryError
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from tf2_ros import TransformBroadcaster

from forest_rescue_system.log_utils import TimestampedNode


class DroneControllerNode(TimestampedNode):
    """namespace와 설정을 통해 PX4 드론 한 대를 독립적으로 제어한다."""

    def __init__(self):
        super().__init__("drone_controller_node")

        self.declare_parameter("drone_id", "quadrotor_01")
        self.declare_parameter("system_address", "udpin://0.0.0.0:14540")
        self.declare_parameter("mavsdk_server_port", 50051)
        self.declare_parameter("takeoff_altitude_m", 6.0)
        self.declare_parameter("altitude_acceptance_radius_m", 0.1)
        self.declare_parameter("takeoff_tolerance_m", 0.2)
        self.declare_parameter("search_yaw_deg", 0.0)
        self.declare_parameter("waypoint_hold_seconds", 0.2)
        self.declare_parameter("waypoint_acceptance_radius_m", 0.8)
        self.declare_parameter("waypoint_altitude_tolerance_m", 0.8)
        self.declare_parameter("waypoint_timeout_sec", 75.0)
        self.declare_parameter("return_timeout_sec", 120.0)
        self.declare_parameter(
            "terrain_mesh_path",
            "~/b3_cobot3_ws/isaac_sim/generated_terrain_mesh.npz",
        )
        self.declare_parameter(
            "environment_mesh_path",
            "~/b3_cobot3_ws/isaac_sim/generated_environment_meshes.npz",
        )
        self.declare_parameter("return_path_clearance_m", 8.0)
        self.declare_parameter("return_path_corridor_radius_m", 2.0)
        self.declare_parameter("return_obstacle_clearance_m", 3.0)

        # 복귀 후 높은 안전고도에서 바로 LAND하지 않고 홈 상공의 낮은
        # 접근고도까지 Offboard로 정밀 하강한 뒤 착륙한다.
        self.declare_parameter("landing_approach_altitude_m", 3.0)
        self.declare_parameter("landing_approach_timeout_sec", 45.0)
        self.declare_parameter("landing_descent_start_timeout_sec", 8.0)
        self.declare_parameter("landing_min_descent_progress_m", 0.30)
        self.declare_parameter("landing_fallback_to_px4_land", True)
        self.declare_parameter("landing_xy_tolerance_m", 0.35)
        self.declare_parameter("landing_altitude_tolerance_m", 0.30)
        self.declare_parameter("landing_stopped_speed_m_s", 0.40)
        self.declare_parameter("landing_settle_sec", 2.0)
        self.declare_parameter("landing_timeout_sec", 35.0)
        self.declare_parameter("post_touchdown_settle_sec", 1.0)
        self.declare_parameter("disarm_timeout_sec", 8.0)
        self.declare_parameter("landing_command_retries", 2)

        # PX4 Position Offboard 모드의 이동 제한값이다. Position setpoint에는
        # 속도 필드가 없으므로 PX4 파라미터를 통해 실제 비행 속도를 정한다.
        self.declare_parameter("search_horizontal_speed_m_s", 2.0)
        self.declare_parameter("search_horizontal_acceleration_m_s2", 2.0)
        self.declare_parameter("search_horizontal_position_gain", 0.8)
        self.declare_parameter("search_vertical_speed_up_m_s", 1.5)
        self.declare_parameter("search_vertical_speed_down_m_s", 1.5)
        self.declare_parameter("avoidance_climb_step_m", 2.0)
        self.declare_parameter("avoidance_max_climb_m", 12.0)
        self.declare_parameter("avoidance_settle_sec", 1.0)
        self.declare_parameter("avoidance_climb_timeout_sec", 15.0)
        self.declare_parameter("avoidance_altitude_tolerance_m", 0.5)
        self.declare_parameter("avoidance_climb_step_retries", 2)
        self.declare_parameter("avoidance_replan_attempts", 1)
        self.declare_parameter("avoidance_retry_hover_sec", 0.3)
        self.declare_parameter("avoidance_lateral_offset_m", 5.0)
        self.declare_parameter("avoidance_forward_offset_m", 2.0)
        self.declare_parameter("avoidance_side_clearance_m", 5.0)
        self.declare_parameter("avoidance_xy_timeout_sec", 8.0)
        self.declare_parameter("avoidance_brake_timeout_sec", 2.5)
        self.declare_parameter("avoidance_stopped_speed_m_s", 0.60)
        self.declare_parameter("avoidance_direction_check_sec", 0.25)
        self.declare_parameter("avoidance_front_clearance_m", 3.5)
        self.declare_parameter("avoidance_probe_distance_m", 4.0)
        # 한 Waypoint에서 A*와 VFH를 반복해 수색 경로에서 멀어지지 않도록
        # A* 1회와 VFH 1회 정도만 검사한 뒤 상승 회피로 전환한다.
        self.declare_parameter("avoidance_direction_attempts", 2)
        self.declare_parameter(
            "search_horizontal_avoidance_budget_per_waypoint",
            1,
        )
        # 높은 회피 고도에서는 좌우로 다시 빠지기보다 원래 수색점의 XY를
        # 향하도록 수평 우회를 생략하고 추가 상승을 우선한다.
        self.declare_parameter(
            "search_high_altitude_skip_horizontal_avoidance",
            True,
        )
        self.declare_parameter("search_high_altitude_threshold_m", 1.0)
        # 수색 Waypoint는 XY 방문을 우선한다. 계획 고도보다 이미 높다면
        # 내려오지 않고 현재 높은 고도를 유지하며 다음 점으로 이동한다.
        self.declare_parameter("search_xy_priority_enabled", True)
        self.declare_parameter(
            "search_keep_high_altitude_after_escape",
            True,
        )
        # 원래 Waypoint 방향으로 최소 이만큼 전진하지 못하는 순수 측면
        # 우회는 거부하고, A* → VFH → 상승 회피 순서로 빠르게 전환한다.
        self.declare_parameter("avoidance_min_forward_progress_m", 0.75)
        # A*/VFH가 전진하더라도 측면 이동량이 지나치면 해당 후보를 버린다.
        self.declare_parameter("avoidance_max_lateral_offset_m", 2.0)
        self.declare_parameter(
            "avoidance_max_lateral_to_forward_ratio",
            0.80,
        )
        self.declare_parameter("avoidance_vector_max_age_sec", 1.2)
        self.declare_parameter("local_detour_max_age_sec", 1.2)
        self.declare_parameter("local_detour_hard_stop_distance_m", 0.75)
        # 짧은 로컬 우회에서는 body 기준 경로가 바뀌지 않도록 현재 Yaw를
        # 유지한다. 이동 직후 센서 방향이 안정될 때까지 일반 차단 판정은
        # 잠시 유예하고, 이후에도 일정 시간 연속 차단일 때만 재계획한다.
        self.declare_parameter("avoidance_keep_yaw_during_detour", True)
        self.declare_parameter("avoidance_commit_sec", 0.35)
        self.declare_parameter("avoidance_block_confirm_sec", 0.20)
        # 수평 A*/VFH가 모두 실패했을 때 충분히 상승한 뒤 원래 진행
        # 방향으로 장애물을 건넌다. LiDAR가 잠깐 clear가 되더라도 최소
        # 상승량을 채우며, 전진 중 다시 막히면 추가 상승 후 재시도한다.
        self.declare_parameter("vertical_escape_min_climb_m", 5.0)
        self.declare_parameter("vertical_escape_retry_climb_step_m", 2.0)
        self.declare_parameter("vertical_escape_cross_retries", 3)
        self.declare_parameter(
            "vertical_escape_keep_high_on_descent_failure",
            True,
        )
        self.declare_parameter("vertical_escape_min_forward_m", 6.0)
        self.declare_parameter("vertical_escape_max_forward_m", 16.0)
        self.declare_parameter("vertical_escape_obstacle_pass_margin_m", 2.5)
        self.declare_parameter("vertical_escape_extra_forward_m", 3.0)
        self.declare_parameter("vertical_escape_clear_hold_sec", 1.2)
        self.declare_parameter("vertical_escape_cross_timeout_sec", 20.0)
        self.declare_parameter("vertical_escape_cross_tolerance_m", 0.8)
        self.declare_parameter("vertical_escape_descent_step_m", 1.0)
        self.declare_parameter("vertical_escape_descent_timeout_sec", 12.0)
        self.declare_parameter("vertical_escape_descent_retries", 2)
        self.declare_parameter("vertical_escape_descent_clear_sec", 0.6)
        self.declare_parameter("victim_approach_timeout_sec", 120.0)

        # 수평 이동 전에 기체 전방과 RGB 카메라가 목표 진행방향을
        # 바라보도록 Yaw를 먼저 정렬한다. 별도 패치 실행 없이 이 노드가
        # 시작될 때부터 모든 수색·우회·접근·복귀 이동에 적용된다.
        self.declare_parameter("direction_yaw_enabled", True)
        self.declare_parameter("direction_yaw_min_distance_m", 0.50)
        self.declare_parameter("direction_yaw_tolerance_deg", 6.0)
        self.declare_parameter("direction_yaw_timeout_sec", 8.0)
        self.declare_parameter("direction_yaw_stable_samples", 2)
        self.declare_parameter("direction_yaw_settle_sec", 0.20)
        self.declare_parameter(
            "search_plan_path",
            "~/b3_cobot3_ws/isaac_sim/generated_search_plan.json",
        )
        self.declare_parameter("search_waypoints", "0,0,-6")
        self.declare_parameter("home_world_enu", [0.0, 0.0, 0.0])
        self.declare_parameter("safe_return_down_m", -20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "quadrotor_01/base_link")
        self.declare_parameter("command_topic", "/drone_01/command")
        self.declare_parameter("status_topic", "/drone_01/status")
        self.declare_parameter(
            "position_topic", "/drone_01/local_position_ned"
        )
        self.declare_parameter(
            "obstacle_topic", "/drone_01/obstacle/blocked"
        )
        self.declare_parameter(
            "obstacle_clearances_topic",
            "/drone_01/obstacle/clearances",
        )
        self.declare_parameter(
            "planning_obstacle_distance_topic",
            "/drone_01/obstacle/planning_distance",
        )
        self.declare_parameter(
            "movement_direction_topic",
            "/drone_01/navigation/direction_body_rad",
        )
        self.declare_parameter(
            "avoidance_vector_topic",
            "/drone_01/obstacle/avoidance_vector",
        )
        self.declare_parameter(
            "local_detour_topic",
            "/drone_01/obstacle/local_detour_body",
        )
        self.declare_parameter("path_topic", "/drone_01/search_path")
        self.declare_parameter(
            "cooperative_plan_topic",
            "/drone_01/mission/cooperative_plan",
        )
        self.declare_parameter(
            "cooperative_plan_ack_topic",
            "/drone_01/mission/cooperative_plan_ack",
        )
        self.declare_parameter(
            "cooperative_transit_path_topic",
            "/drone_01/cooperative_transit_path",
        )
        self.declare_parameter(
            "cooperative_search_path_topic",
            "/drone_01/cooperative_search_path",
        )
        # 수색 경로 반복 방식이다.
        # - once: 정방향 한 번
        # - forward_reverse_once: 정방향 한 번 + 역방향 한 번
        # - infinite: 정방향과 역방향을 조난자 탐지 또는 외부 명령 전까지 반복
        self.declare_parameter("search_repeat_mode", "infinite")

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.home_world_enu = [
            float(value)
            for value in self.get_parameter("home_world_enu").value
        ]
        self.safe_return_down_m = float(
            self.get_parameter("safe_return_down_m").value
        )
        self.primary_waypoints = []
        self.search_waypoints = []
        self.primary_next_index = 0
        self.primary_search_direction = "FORWARD"
        self.primary_search_completed = False
        self.search_repeat_mode = str(
            self.get_parameter("search_repeat_mode").value
        ).strip().lower()
        supported_repeat_modes = {
            "once",
            "forward_reverse_once",
            "infinite",
        }
        if self.search_repeat_mode not in supported_repeat_modes:
            raise ValueError(
                "search_repeat_mode는 once, forward_reverse_once, "
                f"infinite 중 하나여야 합니다: {self.search_repeat_mode!r}"
            )
        self.search_plan_loaded = self._load_search_plan(log_failure=True)
        self.return_terrain_xy = None
        self.return_terrain_z = None
        self.return_obstacle_xy = None
        self.return_obstacle_z = None
        self._load_return_terrain()

        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.position_publisher = self.create_publisher(
            PointStamped,
            str(self.get_parameter("position_topic").value),
            10,
        )
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(
            NavPath,
            str(self.get_parameter("path_topic").value),
            path_qos,
        )
        self.cooperative_transit_path_publisher = self.create_publisher(
            NavPath,
            str(
                self.get_parameter(
                    "cooperative_transit_path_topic"
                ).value
            ),
            path_qos,
        )
        self.cooperative_search_path_publisher = self.create_publisher(
            NavPath,
            str(
                self.get_parameter(
                    "cooperative_search_path_topic"
                ).value
            ),
            path_qos,
        )
        self.cooperative_plan_ack_publisher = self.create_publisher(
            String,
            str(
                self.get_parameter(
                    "cooperative_plan_ack_topic"
                ).value
            ),
            10,
        )
        self.movement_direction_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("movement_direction_topic").value),
            10,
        )
        self.transform_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._command_callback,
            10,
        )
        self.create_subscription(
            String,
            str(
                self.get_parameter(
                    "cooperative_plan_topic"
                ).value
            ),
            self._cooperative_plan_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("obstacle_topic").value),
            self._obstacle_callback,
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("obstacle_clearances_topic").value),
            self._obstacle_clearances_callback,
            10,
        )
        self.create_subscription(
            Float32,
            str(
                self.get_parameter(
                    "planning_obstacle_distance_topic"
                ).value
            ),
            self._planning_obstacle_distance_callback,
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("avoidance_vector_topic").value),
            self._avoidance_vector_callback,
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("local_detour_topic").value),
            self._local_detour_callback,
            10,
        )

        server_port = int(
            self.get_parameter("mavsdk_server_port").value
        )
        self.drone = System(port=server_port)
        self.connected = False
        self.health_ready = False
        self.offboard_started = False
        self.flight_limits_applied = False
        self.search_task = None
        self.active_search_mode = "NONE"
        self.cooperative_plan_id = None
        self.cooperative_transit_waypoints = []
        self.cooperative_search_waypoints = []
        self.cooperative_assignment = None
        self.cooperative_repeat_mode = self.search_repeat_mode
        self.approach_task = None
        self.return_task = None
        self.landing_in_progress = False
        self.stop_search_event = None
        self.latest_north_m = 0.0
        self.latest_east_m = 0.0
        self.latest_down_m = 0.0
        self.latest_relative_altitude_m = 0.0
        self.latest_velocity_north_m_s = 0.0
        self.latest_velocity_east_m_s = 0.0
        self.latest_velocity_down_m_s = 0.0
        self.latest_yaw_deg = 0.0
        self.obstacle_blocked = False
        self.proactive_obstacle_distance_m = float("inf")
        self.front_clearance_m = float("inf")
        self.left_clearance_m = float("inf")
        self.right_clearance_m = float("inf")
        self.avoidance_direction_body_rad = 0.0
        self.avoidance_direction_clearance_m = 0.0
        self.avoidance_direction_valid = False
        self.avoidance_direction_received_at = float("-inf")
        self.local_detour_body_x_m = 0.0
        self.local_detour_body_y_m = 0.0
        self.local_detour_valid = False
        self.local_detour_nearest_360_m = float("inf")
        self.local_detour_received_at = float("-inf")
        self.current_status = "CREATED"

        self.async_loop = asyncio.new_event_loop()
        self.async_thread = threading.Thread(
            target=self._run_async_loop,
            daemon=True,
        )
        self.async_thread.start()
        self._submit(self._initialize_mavsdk())

        self.path_timer = self.create_timer(1.0, self._publish_search_path)
        self.status_timer = self.create_timer(1.0, self._republish_status)
        self.get_logger().info(
            f"{self.drone_id} MAVSDK 제어 노드 시작: "
            f"server_port={server_port}, waypoints={len(self.search_waypoints)}"
        )

    def _load_search_plan(self, log_failure=True):
        """현재 드론의 동적 수색 계획과 홈 좌표를 JSON에서 다시 읽는다."""
        plan_path = Path(
            str(self.get_parameter("search_plan_path").value)
        ).expanduser()
        if plan_path.is_file():
            try:
                plan = json.loads(plan_path.read_text(encoding="utf-8"))
                drone_plans = plan["drones"]
                drone_plan = drone_plans[self.drone_id]

                declared_count = int(
                    plan.get("drone_count", len(drone_plans))
                )
                declared_ids = [
                    str(value)
                    for value in plan.get("drone_ids", drone_plans.keys())
                ]
                if declared_count != len(drone_plans):
                    raise ValueError(
                        "drone_count와 drones 항목 수가 다릅니다: "
                        f"count={declared_count}, entries={len(drone_plans)}"
                    )
                if self.drone_id not in declared_ids:
                    raise KeyError(
                        f"drone_ids에 {self.drone_id}가 없습니다"
                    )

                self.home_world_enu = [
                    float(value)
                    for value in drone_plan["home_world_enu"]
                ]
                if len(self.home_world_enu) != 3:
                    raise ValueError(
                        f"home_world_enu 형식 오류: {self.home_world_enu}"
                    )

                # format_version 2 계획과의 호환용이다. 새 계획은 지도
                # 전체 최고점 기반 safe_return_down_m을 저장하지 않는다.
                if "safe_return_down_m" in drone_plan:
                    self.safe_return_down_m = float(
                        drone_plan["safe_return_down_m"]
                    )

                self.return_path_clearance_m = float(
                    plan.get(
                        "return_path_clearance_m",
                        self.get_parameter("return_path_clearance_m").value,
                    )
                )
                self.return_path_corridor_radius_m = float(
                    plan.get(
                        "return_path_corridor_radius_m",
                        self.get_parameter(
                            "return_path_corridor_radius_m"
                        ).value,
                    )
                )
                self.return_obstacle_clearance_m = float(
                    plan.get(
                        "return_obstacle_clearance_m",
                        self.get_parameter(
                            "return_obstacle_clearance_m"
                        ).value,
                    )
                )
                waypoints = [
                    (
                        float(item["north_m"]),
                        float(item["east_m"]),
                        float(item["down_m"]),
                    )
                    for item in drone_plan["waypoints"]
                ]
                if not waypoints:
                    raise ValueError(f"{self.drone_id} waypoints가 비어 있습니다")
                self.search_waypoints = waypoints
                self.primary_waypoints = list(waypoints)
                if self.primary_next_index > len(self.primary_waypoints):
                    self.primary_next_index = 0
                self.search_plan_loaded = True
                self.get_logger().info(
                    f"동적 수색 계획 로드: {self.drone_id}, "
                    f"fleet={declared_count}, waypoints={len(waypoints)}, "
                    f"home={self.home_world_enu}"
                )
                return True
            except (
                KeyError,
                TypeError,
                ValueError,
                json.JSONDecodeError,
            ) as error:
                if log_failure:
                    self.get_logger().warning(
                        "수색 계획 파일 파싱 실패: "
                        f"{plan_path}: {error}"
                    )
        elif log_failure:
            self.get_logger().warning(
                f"수색 계획 파일이 아직 없습니다: {plan_path}"
            )

        # 초기 TF와 안전 대기를 위해 YAML의 한 점 경로는 유지하지만,
        # 실제 START_SEARCH는 유효한 동적 JSON이 없으면 시작하지 않는다.
        self.search_waypoints = self._parse_waypoints(
            str(self.get_parameter("search_waypoints").value)
        )
        self.primary_waypoints = list(self.search_waypoints)
        self.search_plan_loaded = False
        return False

    def _load_return_terrain(self):
        """Isaac Sim이 내보낸 Terrain 표면을 복귀 경로 계산용으로 읽는다."""
        if not hasattr(self, "return_path_clearance_m"):
            self.return_path_clearance_m = float(
                self.get_parameter("return_path_clearance_m").value
            )
        if not hasattr(self, "return_path_corridor_radius_m"):
            self.return_path_corridor_radius_m = float(
                self.get_parameter("return_path_corridor_radius_m").value
            )
        if not hasattr(self, "return_obstacle_clearance_m"):
            self.return_obstacle_clearance_m = float(
                self.get_parameter("return_obstacle_clearance_m").value
            )

        mesh_path = Path(
            str(self.get_parameter("terrain_mesh_path").value)
        ).expanduser()
        try:
            with np.load(mesh_path, allow_pickle=False) as mesh:
                vertices = np.asarray(mesh["vertices"], dtype=np.float64)
            if vertices.ndim != 2 or vertices.shape[1] != 3:
                raise ValueError(f"vertices shape={vertices.shape}")
            self.return_terrain_xy = vertices[:, :2]
            self.return_terrain_z = vertices[:, 2]
            self.get_logger().info(
                "복귀 경로 Terrain 로드: "
                f"{mesh_path}, vertices={len(vertices)}"
            )
        except (OSError, KeyError, ValueError) as error:
            self.get_logger().warning(
                "복귀 Terrain 로드 실패, 기존 안전고도를 대체값으로 사용: "
                f"{mesh_path}: {error}"
            )

        environment_path = Path(
            str(self.get_parameter("environment_mesh_path").value)
        ).expanduser()
        try:
            vertex_groups = []
            with np.load(environment_path, allow_pickle=False) as meshes:
                for key in meshes.files:
                    if not key.endswith("_vertices"):
                        continue
                    vertices = np.asarray(meshes[key], dtype=np.float64)
                    if vertices.ndim == 2 and vertices.shape[1] == 3:
                        vertex_groups.append(vertices)
            if not vertex_groups:
                raise ValueError("환경 vertices 배열이 없습니다.")
            vertices = np.concatenate(vertex_groups, axis=0)
            self.return_obstacle_xy = vertices[:, :2]
            self.return_obstacle_z = vertices[:, 2]
            self.get_logger().info(
                "복귀 경로 환경 Mesh 로드: "
                f"{environment_path}, vertices={len(vertices)}"
            )
        except (OSError, KeyError, ValueError) as error:
            self.get_logger().warning(
                "복귀 환경 Mesh 로드 실패, Terrain만 사용: "
                f"{environment_path}: {error}"
            )

    @staticmethod
    def _distance_to_segment(points_xy, start_xy, end_xy):
        """여러 XY 점과 유한 선분 사이의 최단거리를 계산한다."""
        segment = end_xy - start_xy
        length_squared = float(np.dot(segment, segment))
        if length_squared < 1.0e-6:
            return np.linalg.norm(points_xy - end_xy, axis=1)
        relative = points_xy - start_xy
        ratios = np.clip(
            (relative @ segment) / length_squared,
            0.0,
            1.0,
        )
        closest = start_xy + ratios[:, None] * segment
        return np.linalg.norm(points_xy - closest, axis=1)

    def _calculate_return_down_m(self):
        """현재 위치~홈 회랑의 최고 지형을 이용해 이번 복귀 고도를 정한다."""
        if self.return_terrain_xy is None or self.return_terrain_z is None:
            fallback_down = min(
                float(self.latest_down_m),
                float(self.safe_return_down_m),
            )
            self.get_logger().warning(
                "Terrain이 없어 기존 안전 복귀고도를 사용: "
                f"target_D={fallback_down:.2f}"
            )
            return fallback_down

        start_xy = np.asarray(
            [
                self.home_world_enu[0] + self.latest_east_m,
                self.home_world_enu[1] + self.latest_north_m,
            ],
            dtype=np.float64,
        )
        home_xy = np.asarray(self.home_world_enu[:2], dtype=np.float64)
        distance_to_path = self._distance_to_segment(
            self.return_terrain_xy,
            start_xy,
            home_xy,
        )

        corridor_radius = max(0.5, self.return_path_corridor_radius_m)
        corridor_mask = distance_to_path <= corridor_radius
        if np.any(corridor_mask):
            terrain_max_z = float(np.max(self.return_terrain_z[corridor_mask]))
        else:
            terrain_max_z = float(
                self.return_terrain_z[int(np.argmin(distance_to_path))]
            )

        clearance = max(3.0, self.return_path_clearance_m)
        terrain_safe_z = terrain_max_z + clearance
        obstacle_max_z = None
        obstacle_safe_z = float("-inf")
        if self.return_obstacle_xy is not None:
            obstacle_distance = self._distance_to_segment(
                self.return_obstacle_xy,
                start_xy,
                home_xy,
            )
            obstacle_mask = obstacle_distance <= corridor_radius
            if np.any(obstacle_mask):
                obstacle_max_z = float(
                    np.max(self.return_obstacle_z[obstacle_mask])
                )
                obstacle_safe_z = obstacle_max_z + max(
                    1.0,
                    self.return_obstacle_clearance_m,
                )

        required_world_z = max(terrain_safe_z, obstacle_safe_z)
        required_down_m = -(
            required_world_z - float(self.home_world_enu[2])
        )

        # 현재 고도가 이미 안전고도보다 높으면 더 상승시키지 않는다.
        # 현재 고도를 유지한 채 곧바로 홈 방향 수평 복귀를 시작한다.
        selected_down_m = min(float(self.latest_down_m), required_down_m)
        current_world_z = (
            float(self.home_world_enu[2]) - float(self.latest_down_m)
        )
        selected_world_z = (
            float(self.home_world_enu[2]) - selected_down_m
        )
        self.get_logger().info(
            "동적 복귀 고도 계산: "
            f"path_terrain_max_Z={terrain_max_z:.2f}m, "
            f"clearance={clearance:.2f}m, "
            f"path_obstacle_max_Z="
            f"{obstacle_max_z if obstacle_max_z is not None else 'NONE'}, "
            f"required_Z={required_world_z:.2f}m, "
            f"current_Z={current_world_z:.2f}m, "
            f"selected_Z={selected_world_z:.2f}m "
            f"(target_D={selected_down_m:.2f})"
        )
        return selected_down_m

    def _run_async_loop(self):
        asyncio.set_event_loop(self.async_loop)
        self.async_loop.run_forever()

    def _submit(self, coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine, self.async_loop)

    async def _initialize_mavsdk(self):
        self.stop_search_event = asyncio.Event()
        system_address = str(self.get_parameter("system_address").value)
        self._publish_status("CONNECTING")
        await self.drone.connect(system_address=system_address)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.connected = True
                self._publish_status("CONNECTED")
                self.get_logger().info(
                    f"{self.drone_id} PX4 연결 성공: {system_address}"
                )
                break

        await self._configure_telemetry_rates()
        asyncio.create_task(self._position_telemetry_loop())
        asyncio.create_task(self._attitude_telemetry_loop())
        asyncio.create_task(self._relative_altitude_loop())

    async def _configure_telemetry_rates(self):
        """다중 드론 운용 시 MAVSDK callback queue가 밀리지 않게 제한한다."""
        rate_setters = [
            ("position_velocity_ned", 10.0),
            ("attitude_euler", 5.0),
            ("position", 5.0),
        ]
        for stream_name, rate_hz in rate_setters:
            setter = getattr(
                self.drone.telemetry,
                f"set_rate_{stream_name}",
                None,
            )
            if setter is None:
                continue
            try:
                await setter(rate_hz)
            except TelemetryError as error:
                self.get_logger().warning(
                    f"Telemetry rate 설정 실패({stream_name}): {error}"
                )

    async def _position_telemetry_loop(self):
        async for telemetry in self.drone.telemetry.position_velocity_ned():
            position = telemetry.position
            velocity = telemetry.velocity
            self.latest_north_m = float(position.north_m)
            self.latest_east_m = float(position.east_m)
            self.latest_down_m = float(position.down_m)
            self.latest_velocity_north_m_s = float(velocity.north_m_s)
            self.latest_velocity_east_m_s = float(velocity.east_m_s)
            self.latest_velocity_down_m_s = float(velocity.down_m_s)
            self._publish_local_position()
            self._publish_map_to_base_tf()

    async def _attitude_telemetry_loop(self):
        async for attitude in self.drone.telemetry.attitude_euler():
            self.latest_yaw_deg = float(attitude.yaw_deg)

    async def _relative_altitude_loop(self):
        async for position in self.drone.telemetry.position():
            self.latest_relative_altitude_m = float(
                position.relative_altitude_m
            )

    def _command_callback(self, message):
        raw_command = message.data.strip()
        command = raw_command.upper()
        self.get_logger().info(f"{self.drone_id} 명령 수신: {command}")
        if command == "TAKEOFF":
            self._submit(self._takeoff())
        elif command == "START_SEARCH":
            self._submit(self._start_search(resume=False))
        elif command == "RESUME_SEARCH":
            self._submit(self._start_search(resume=True))
        elif command.startswith("START_COOPERATIVE_SEARCH:"):
            plan_id = raw_command.split(":", 1)[1].strip()
            self._submit(self._start_cooperative_search(plan_id))
        elif command == "MARK_PRIMARY_COMPLETE":
            self.primary_next_index = len(self.primary_waypoints)
            self.primary_search_completed = True
            self._clear_cooperative_paths()
            self._submit(self._hover())
        elif command.startswith("APPROACH_VICTIM:"):
            self._clear_cooperative_paths()
            try:
                fields = raw_command.split(":", 1)[1].split(",")
                if len(fields) != 3:
                    raise ValueError("좌표 필드는 3개여야 합니다")
                target_world_enu = tuple(float(value) for value in fields)
            except ValueError as error:
                self.get_logger().error(f"조난자 접근 좌표 파싱 실패: {error}")
                return
            self._submit(self._start_victim_approach(target_world_enu))
        elif command == "HOVER":
            self._submit(self._hover())
        elif command == "RETURN_HOME":
            self._clear_cooperative_paths()
            self._submit(self._start_return_home())
        elif command == "LAND":
            self._clear_cooperative_paths()
            self._submit(self._land())
        else:
            self.get_logger().warning(f"알 수 없는 명령: {command}")

    def _cooperative_plan_callback(self, message):
        """Mission Manager가 만든 드론별 협동 계획을 검증해 보관한다."""
        try:
            payload = json.loads(message.data)
            if str(payload["drone_id"]) != self.drone_id:
                return
            plan_id = str(payload["plan_id"])
            transit_world = payload["transit_waypoints_world_enu"]
            search_world = payload["search_waypoints_world_enu"]
            if not transit_world or not search_world:
                raise ValueError("협동 계획 Waypoint가 비어 있습니다.")
            self.cooperative_transit_waypoints = [
                self._world_enu_to_local_ned(item)
                for item in transit_world
            ]
            self.cooperative_search_waypoints = [
                self._world_enu_to_local_ned(item)
                for item in search_world
            ]
            repeat_mode = str(
                payload.get("search_repeat_mode", self.search_repeat_mode)
            ).strip().lower()
            if repeat_mode not in {
                "once",
                "forward_reverse_once",
                "infinite",
            }:
                raise ValueError(
                    f"협동 search_repeat_mode 형식 오류: {repeat_mode!r}"
                )
            self.cooperative_plan_id = plan_id
            self.cooperative_assignment = payload
            self.cooperative_repeat_mode = repeat_mode
            self._publish_cooperative_paths()

            ack = String()
            ack.data = plan_id
            self.cooperative_plan_ack_publisher.publish(ack)
            self.get_logger().info(
                f"협동 수색 계획 수신: plan_id={plan_id}, "
                f"transit={len(self.cooperative_transit_waypoints)}, "
                f"search={len(self.cooperative_search_waypoints)}"
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"협동 수색 계획 파싱 실패: {error}")

    def _world_enu_to_local_ned(self, world_enu):
        if not isinstance(world_enu, (list, tuple)) or len(world_enu) != 3:
            raise ValueError(f"world_enu 형식 오류: {world_enu}")
        world_x, world_y, world_z = [float(value) for value in world_enu]
        return (
            world_y - self.home_world_enu[1],
            world_x - self.home_world_enu[0],
            -(world_z - self.home_world_enu[2]),
        )

    async def _start_cooperative_search(self, plan_id):
        if plan_id != self.cooperative_plan_id:
            self.get_logger().error(
                "협동 수색 시작 거부: 계획 ID 불일치 "
                f"requested={plan_id}, loaded={self.cooperative_plan_id}"
            )
            self._publish_status("ERROR_COOP_PLAN_MISMATCH")
            return
        if not self.cooperative_search_waypoints:
            self._publish_status("ERROR_COOP_PLAN_EMPTY")
            return
        await self._cancel_active_search()
        self.stop_search_event.clear()
        self.active_search_mode = "COOPERATIVE"
        self.search_task = asyncio.create_task(self._cooperative_search_path())

    async def _cooperative_search_path(self):
        """협동 소구역을 설정 방식에 따라 반복 수색한다."""
        try:
            await self._ensure_offboard()
            yaw_deg = float(self.get_parameter("search_yaw_deg").value)
            hold_seconds = float(
                self.get_parameter("waypoint_hold_seconds").value
            )
            self._publish_status("COOP_TRANSIT")
            completed = await self._execute_waypoint_pass(
                self.cooperative_transit_waypoints,
                yaw_deg,
                hold_seconds,
                label="협동 진입",
                status="COOP_TRANSIT",
                allow_waypoint_skip=False,
            )
            if not completed:
                return

            mode = self.cooperative_repeat_mode
            pass_count = 0
            direction = "FORWARD"
            cycle_count = 1

            while not self.stop_search_event.is_set():
                if direction == "FORWARD":
                    # 역순 끝점에서 다시 출발할 때 첫 점 중복 명령을 피한다.
                    waypoints = (
                        self.cooperative_search_waypoints
                        if pass_count == 0
                        else self.cooperative_search_waypoints[1:]
                    )
                    status = "COOP_SEARCHING_FORWARD"
                    label = f"협동 정방향 {cycle_count}회차"
                else:
                    # 정방향 마지막 점은 현재 위치이므로 제외한다.
                    waypoints = list(
                        reversed(self.cooperative_search_waypoints[:-1])
                    )
                    status = "COOP_SEARCHING_REVERSE"
                    label = f"협동 역방향 {cycle_count}회차"

                if waypoints:
                    self._publish_status(status)
                    completed = await self._execute_waypoint_pass(
                        waypoints,
                        yaw_deg,
                        hold_seconds,
                        label=label,
                        status=status,
                        allow_waypoint_skip=True,
                    )
                    if not completed:
                        return

                pass_count += 1
                if mode == "once":
                    break
                if mode == "forward_reverse_once" and pass_count >= 2:
                    break

                if direction == "FORWARD":
                    direction = "REVERSE"
                    self.get_logger().warning(
                        f"협동 수색 정방향 {cycle_count}회차 미탐: "
                        "같은 소구역을 역방향으로 수색합니다."
                    )
                else:
                    direction = "FORWARD"
                    cycle_count += 1
                    self.get_logger().warning(
                        f"협동 수색 역방향 {cycle_count - 1}회차 미탐: "
                        f"정방향 {cycle_count}회차를 계속합니다."
                    )

            if self.stop_search_event.is_set():
                return

            # 유한 반복 모드에서만 정상 완료 상태를 발행한다.
            await self._hold_current_position(stop_search=False)
            self._publish_status("COOP_SEARCH_FINISHED")
        except OffboardError as error:
            self.get_logger().error(f"협동 수색 Offboard 실패: {error}")
            self._publish_status("ERROR_COOP_OFFBOARD")
        except asyncio.CancelledError:
            return
        finally:
            self.search_task = None
            if self.active_search_mode == "COOPERATIVE":
                self.active_search_mode = "NONE"

    async def _execute_waypoint_pass(
        self,
        waypoints,
        yaw_deg,
        hold_seconds,
        label,
        status,
        allow_waypoint_skip,
        primary_indices=None,
        primary_direction=None,
    ):
        total = len(waypoints)
        for relative_index, waypoint in enumerate(waypoints, start=1):
            if self.stop_search_event.is_set():
                return False
            north_m, east_m, down_m = waypoint
            if primary_indices is not None:
                waypoint_index = int(primary_indices[relative_index - 1])
                self.primary_next_index = waypoint_index
                self.primary_search_direction = str(primary_direction)
            self.get_logger().info(
                f"{label} {relative_index}/{total}: "
                f"N={north_m:.1f}, E={east_m:.1f}, D={down_m:.1f}"
            )
            reached = await self._go_to_setpoint(
                north_m,
                east_m,
                down_m,
                yaw_deg,
                float(self.get_parameter("waypoint_timeout_sec").value),
                allow_avoidance=True,
                resume_status=status,
                allow_waypoint_skip=allow_waypoint_skip,
            )
            if not reached:
                return False
            if primary_indices is not None:
                waypoint_index = int(primary_indices[relative_index - 1])
                if str(primary_direction) == "FORWARD":
                    self.primary_next_index = waypoint_index + 1
                else:
                    self.primary_next_index = waypoint_index - 1
            await self._sleep_sim_time(max(0.0, hold_seconds))
        return True

    def _obstacle_callback(self, message):
        self.obstacle_blocked = bool(message.data)

    def _obstacle_clearances_callback(self, message):
        self.front_clearance_m = float(message.vector.x)
        self.left_clearance_m = float(message.vector.y)
        self.right_clearance_m = float(message.vector.z)

    def _planning_obstacle_distance_callback(self, message):
        value = float(message.data)
        self.proactive_obstacle_distance_m = (
            value if math.isfinite(value) and value >= 0.0 else float("inf")
        )

    def _avoidance_vector_callback(self, message):
        self.avoidance_direction_body_rad = float(message.vector.x)
        self.avoidance_direction_clearance_m = float(message.vector.y)
        self.avoidance_direction_valid = bool(message.vector.z >= 0.5)
        self.avoidance_direction_received_at = self._stamp_to_seconds(
            message.header.stamp
        )

    def _local_detour_callback(self, message):
        self.local_detour_body_x_m = float(message.vector.x)
        self.local_detour_body_y_m = float(message.vector.y)
        self.local_detour_valid = bool(message.vector.z >= 0.0)
        self.local_detour_nearest_360_m = abs(float(message.vector.z))
        self.local_detour_received_at = self._stamp_to_seconds(
            message.header.stamp
        )

    async def _wait_for_health(self, timeout_sec=60.0):
        async def wait_stream():
            async for health in self.drone.telemetry.health():
                ready = (
                    health.is_global_position_ok
                    and health.is_home_position_ok
                    and health.is_local_position_ok
                )
                if ready:
                    self.health_ready = True
                    return

        await asyncio.wait_for(wait_stream(), timeout=timeout_sec)

    async def _takeoff(self):
        if not self.connected:
            self._publish_status("ERROR_NOT_CONNECTED")
            return
        try:
            self._publish_status("PREPARING")
            if not self.health_ready:
                await self._wait_for_health()

            altitude = float(
                self.get_parameter("takeoff_altitude_m").value
            )
            await self.drone.param.set_param_float(
                "NAV_MC_ALT_RAD",
                float(
                    self.get_parameter(
                        "altitude_acceptance_radius_m"
                    ).value
                ),
            )
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.arm()
            self._publish_status("ARMED")
            await self.drone.action.takeoff()
            self._publish_status("TAKING_OFF")

            tolerance = float(
                self.get_parameter("takeoff_tolerance_m").value
            )
            deadline = self._sim_time_sec() + 40.0
            while self._sim_time_sec() < deadline:
                if self.latest_relative_altitude_m >= altitude - tolerance:
                    self._publish_status("AIRBORNE")
                    return
                await asyncio.sleep(0.2)
            raise asyncio.TimeoutError("목표 이륙 고도 도달 시간 초과")
        except (ActionError, ParamError, asyncio.TimeoutError) as error:
            self.get_logger().error(f"이륙 실패: {error}")
            self._publish_status(f"ERROR_TAKEOFF_{type(error).__name__}")

    async def _ensure_offboard(self):
        if self.offboard_started:
            return
        await self._configure_flight_limits()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(
                self.latest_north_m,
                self.latest_east_m,
                self.latest_down_m,
                self.latest_yaw_deg,
            )
        )
        await self.drone.offboard.start()
        self.offboard_started = True

    async def _configure_flight_limits(self):
        """수색 속도 제한을 PX4 멀티콥터 위치제어기에 한 번 적용한다."""
        if self.flight_limits_applied:
            return

        parameter_values = {
            "MPC_XY_VEL_MAX": float(
                self.get_parameter("search_horizontal_speed_m_s").value
            ),
            "MPC_ACC_HOR": float(
                self.get_parameter("search_horizontal_acceleration_m_s2").value
            ),
            "MPC_XY_P": float(
                self.get_parameter("search_horizontal_position_gain").value
            ),
            "MPC_Z_VEL_MAX_UP": float(
                self.get_parameter("search_vertical_speed_up_m_s").value
            ),
            "MPC_Z_VEL_MAX_DN": float(
                self.get_parameter("search_vertical_speed_down_m_s").value
            ),
        }
        applied = []
        for name, value in parameter_values.items():
            try:
                await self.drone.param.set_param_float(name, value)
                applied.append(f"{name}={value:.1f}")
            except ParamError as error:
                # PX4 빌드별 파라미터 차이가 있어도 비행 자체는 계속한다.
                self.get_logger().warning(
                    f"PX4 속도 파라미터 적용 실패({name}): {error}"
                )

        self.flight_limits_applied = True
        if applied:
            self.get_logger().info(
                "PX4 수색 속도 설정: " + ", ".join(applied)
            )

    async def _start_search(self, resume=False):
        if self.search_task and not self.search_task.done():
            self.get_logger().warning("이미 수색 경로를 실행 중입니다.")
            return

        # 첫 시작에서만 최신 JSON을 읽고 진행 방향과 인덱스를 초기화한다.
        # 협동 수색 후 RESUME_SEARCH에서는 중단 전 방향과 목표 인덱스를 보존한다.
        if not resume:
            if not self._load_search_plan(log_failure=True):
                self.get_logger().error(
                    "동적 수색 계획이 없어 START_SEARCH를 거부합니다. "
                    "Isaac Sim과 ROS launch의 drone_count가 같은지 확인하세요."
                )
                self._publish_status("ERROR_SEARCH_PLAN")
                return
            self.primary_next_index = 0
            self.primary_search_direction = "FORWARD"
            self.primary_search_completed = False
        elif self.primary_search_completed:
            self.get_logger().info("기본 담당 구역은 이미 완료 처리됐습니다.")
            self._publish_status("SEARCH_FINISHED_NO_VICTIM")
            return

        self._clear_cooperative_paths()
        self._publish_search_path()
        self.stop_search_event.clear()
        self.active_search_mode = "PRIMARY"
        self.search_task = asyncio.create_task(self._search_path())

    async def _search_path(self):
        """기본 담당 구역을 정방향·역방향으로 반복 수색한다."""
        try:
            await self._ensure_offboard()
            yaw_deg = float(self.get_parameter("search_yaw_deg").value)
            hold_seconds = float(
                self.get_parameter("waypoint_hold_seconds").value
            )
            mode = self.search_repeat_mode
            pass_count = 0
            cycle_count = 1

            while not self.stop_search_event.is_set():
                waypoint_count = len(self.primary_waypoints)
                if waypoint_count == 0:
                    self._publish_status("ERROR_SEARCH_PLAN_EMPTY")
                    return

                direction = self.primary_search_direction
                next_index = int(self.primary_next_index)

                if direction == "FORWARD":
                    if next_index >= waypoint_count:
                        if mode == "once":
                            break
                        self.primary_search_direction = "REVERSE"
                        self.primary_next_index = max(0, waypoint_count - 2)
                        direction = "REVERSE"
                        next_index = self.primary_next_index
                    indices = list(range(max(0, next_index), waypoint_count))
                    status = "SEARCHING_FORWARD"
                    label = f"기본 정방향 {cycle_count}회차"
                else:
                    if next_index < 0:
                        if mode == "forward_reverse_once":
                            break
                        self.primary_search_direction = "FORWARD"
                        self.primary_next_index = 1 if waypoint_count > 1 else 0
                        direction = "FORWARD"
                        next_index = self.primary_next_index
                        cycle_count += 1
                    indices = list(range(min(waypoint_count - 1, next_index), -1, -1))
                    status = "SEARCHING_REVERSE"
                    label = f"기본 역방향 {cycle_count}회차"

                if not indices:
                    # 한 점짜리 경로에서도 반복 루프가 과도하게 회전하지 않게 한다.
                    await self._sleep_sim_time(max(0.1, hold_seconds))
                    if direction == "FORWARD":
                        self.primary_search_direction = "REVERSE"
                        self.primary_next_index = waypoint_count - 1
                    else:
                        self.primary_search_direction = "FORWARD"
                        self.primary_next_index = 0
                        cycle_count += 1
                    continue

                waypoints = [self.primary_waypoints[index] for index in indices]
                self._publish_status(status)
                completed = await self._execute_waypoint_pass(
                    waypoints,
                    yaw_deg,
                    hold_seconds,
                    label=label,
                    status=status,
                    allow_waypoint_skip=True,
                    primary_indices=indices,
                    primary_direction=direction,
                )
                if not completed:
                    return

                pass_count += 1
                if mode == "once":
                    break
                if mode == "forward_reverse_once" and pass_count >= 2:
                    break

                if direction == "FORWARD":
                    self.primary_search_direction = "REVERSE"
                    self.primary_next_index = max(0, waypoint_count - 2)
                    self.get_logger().warning(
                        f"기본 수색 정방향 {cycle_count}회차 미탐: "
                        "같은 경로를 역방향으로 수색합니다."
                    )
                else:
                    self.primary_search_direction = "FORWARD"
                    self.primary_next_index = 1 if waypoint_count > 1 else 0
                    cycle_count += 1
                    self.get_logger().warning(
                        f"기본 수색 역방향 {cycle_count - 1}회차 미탐: "
                        f"정방향 {cycle_count}회차를 계속합니다."
                    )

            if self.stop_search_event.is_set():
                return

            # 유한 반복 모드에서만 담당 구역 완료 상태를 발행한다.
            self.primary_search_completed = True
            self.primary_next_index = len(self.primary_waypoints)
            await self._hold_current_position(stop_search=False)
            self._publish_status("SEARCH_FINISHED_NO_VICTIM")
        except OffboardError as error:
            self.get_logger().error(f"Offboard 수색 실패: {error}")
            self._publish_status("ERROR_OFFBOARD")
        except asyncio.CancelledError:
            return
        finally:
            self.search_task = None
            if self.active_search_mode == "PRIMARY":
                self.active_search_mode = "NONE"

    @staticmethod
    def _normalize_angle_deg(angle_deg):
        """각도를 -180~180도 범위로 정규화한다."""
        return (float(angle_deg) + 180.0) % 360.0 - 180.0

    def _target_yaw_deg(self, target_north_m, target_east_m):
        """현재 위치에서 목표점으로 향하는 PX4 NED Yaw를 계산한다."""
        delta_north = float(target_north_m) - self.latest_north_m
        delta_east = float(target_east_m) - self.latest_east_m
        horizontal_distance = math.hypot(delta_north, delta_east)

        minimum_distance = max(
            0.0,
            float(
                self.get_parameter(
                    "direction_yaw_min_distance_m"
                ).value
            ),
        )
        if horizontal_distance < minimum_distance:
            return None

        # PX4 Local NED에서 Yaw 0도는 North, +90도는 East이다.
        return math.degrees(math.atan2(delta_east, delta_north))

    async def _align_yaw_to_target(
        self,
        target_north_m,
        target_east_m,
        fallback_yaw_deg,
    ):
        """현재 위치를 유지한 채 목표 진행방향으로 먼저 회전한다.

        드론이 자동차처럼 진행방향을 바라본 뒤 이동하게 하여, body에
        고정된 RGB/Depth 카메라 영상에서도 양옆 구조물이 뒤로 흐르는
        형태가 되도록 한다. 수평 이동이 거의 없으면 현재 Yaw를 유지한다.
        """
        enabled = bool(
            self.get_parameter("direction_yaw_enabled").value
        )
        target_yaw_deg = self._target_yaw_deg(
            target_north_m,
            target_east_m,
        )

        if not enabled:
            return float(fallback_yaw_deg)
        if target_yaw_deg is None:
            return float(self.latest_yaw_deg)

        tolerance_deg = max(
            0.5,
            float(
                self.get_parameter(
                    "direction_yaw_tolerance_deg"
                ).value
            ),
        )
        timeout_sec = max(
            0.5,
            float(
                self.get_parameter(
                    "direction_yaw_timeout_sec"
                ).value
            ),
        )
        required_stable_samples = max(
            1,
            int(
                self.get_parameter(
                    "direction_yaw_stable_samples"
                ).value
            ),
        )
        settle_sec = max(
            0.0,
            float(
                self.get_parameter(
                    "direction_yaw_settle_sec"
                ).value
            ),
        )

        initial_error_deg = self._normalize_angle_deg(
            target_yaw_deg - self.latest_yaw_deg
        )
        if abs(initial_error_deg) <= tolerance_deg:
            return float(target_yaw_deg)

        # 회전 중에는 현재 위치와 고도를 유지한다. 위치 이동 명령과 Yaw
        # 회전을 동시에 보내지 않아 옆걸음처럼 보이는 구간을 최소화한다.
        hold_north_m = float(self.latest_north_m)
        hold_east_m = float(self.latest_east_m)
        hold_down_m = float(self.latest_down_m)

        self.get_logger().info(
            f"진행방향 Yaw 정렬: current={self.latest_yaw_deg:.1f}°, "
            f"target={target_yaw_deg:.1f}°, "
            f"error={initial_error_deg:.1f}°"
        )

        deadline = self._sim_time_sec() + timeout_sec
        stable_samples = 0

        while self._sim_time_sec() < deadline:
            self._publish_movement_direction(
                target_north_m,
                target_east_m,
            )
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    hold_north_m,
                    hold_east_m,
                    hold_down_m,
                    float(target_yaw_deg),
                )
            )

            yaw_error_deg = abs(
                self._normalize_angle_deg(
                    target_yaw_deg - self.latest_yaw_deg
                )
            )
            if yaw_error_deg <= tolerance_deg:
                stable_samples += 1
                if stable_samples >= required_stable_samples:
                    if settle_sec > 0.0:
                        await self._sleep_sim_time(settle_sec)
                    self.get_logger().info(
                        "진행방향 Yaw 정렬 완료: "
                        f"yaw={self.latest_yaw_deg:.1f}°, "
                        f"target={target_yaw_deg:.1f}°"
                    )
                    return float(target_yaw_deg)
            else:
                stable_samples = 0

            await asyncio.sleep(0.1)

        # 회전이 조금 늦더라도 목표 Yaw가 포함된 이동 Setpoint를 계속
        # 사용하므로 임무 전체를 정지시키지 않는다.
        remaining_error_deg = self._normalize_angle_deg(
            target_yaw_deg - self.latest_yaw_deg
        )
        self.get_logger().warning(
            "진행방향 Yaw 정렬 시간 초과, 목표 Yaw를 유지하며 이동: "
            f"current={self.latest_yaw_deg:.1f}°, "
            f"target={target_yaw_deg:.1f}°, "
            f"error={remaining_error_deg:.1f}°"
        )
        return float(target_yaw_deg)

    async def _go_to_setpoint(
        self,
        north_m,
        east_m,
        down_m,
        yaw_deg,
        timeout_sec,
        allow_avoidance=False,
        resume_status="SEARCHING",
        allow_waypoint_skip=False,
    ):
        # 수색 Waypoint에서는 XY 방문을 우선한다. 현재 고도가 계획 고도보다
        # 높다면 굳이 내려오지 않고, 지형 안전고도보다 낮을 때만 상승한다.
        xy_priority = (
            allow_avoidance
            and allow_waypoint_skip
            and bool(
                self.get_parameter(
                    "search_xy_priority_enabled"
                ).value
            )
        )
        keep_high_altitude = (
            xy_priority
            and bool(
                self.get_parameter(
                    "search_keep_high_altitude_after_escape"
                ).value
            )
        )
        commanded_down_m = (
            min(float(down_m), float(self.latest_down_m))
            if keep_high_altitude
            else float(down_m)
        )
        horizontal_avoidance_successes = 0
        avoidance_replans = 0

        # 초기 Hover가 낮은 지형 Waypoint보다 이미 높을 수 있다. 상승
        # 한도는 Waypoint와 진입 고도 중 더 높은 쪽을 기준으로 한 번만
        # 계산해 같은 Waypoint의 반복 회피가 한도를 계속 올리지 못하게 한다.
        avoidance_ceiling_down_m = None
        if allow_avoidance:
            reference_down_m = min(
                float(down_m),
                float(self.latest_down_m),
            )
            avoidance_ceiling_down_m = max(
                self.safe_return_down_m,
                reference_down_m
                - float(
                    self.get_parameter("avoidance_max_climb_m").value
                ),
            )

        command_yaw_deg = await self._align_yaw_to_target(
            north_m,
            east_m,
            yaw_deg,
        )
        self._publish_movement_direction(north_m, east_m)

        # 새 이동 방향이 LiDAR 필터에 반영되기 전에 고속 이동 명령을
        # 보내지 않는다. 첫 PointCloud 판정을 기다린 뒤 경로가 열려
        # 있을 때만 원래 Waypoint를 명령한다.
        if allow_avoidance:
            await self._sleep_sim_time(0.35)
        if not (allow_avoidance and self.obstacle_blocked):
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    north_m,
                    east_m,
                    commanded_down_m,
                    command_yaw_deg,
                )
            )

        deadline = self._sim_time_sec() + timeout_sec
        while self._sim_time_sec() < deadline:
            self._publish_movement_direction(north_m, east_m)
            if self.stop_search_event.is_set() and allow_avoidance:
                return False

            if allow_avoidance and self.obstacle_blocked:
                avoidance_started = self._sim_time_sec()
                horizontal_budget = max(
                    0,
                    int(
                        self.get_parameter(
                            "search_horizontal_avoidance_budget_per_waypoint"
                        ).value
                    ),
                )
                high_threshold = max(
                    0.0,
                    float(
                        self.get_parameter(
                            "search_high_altitude_threshold_m"
                        ).value
                    ),
                )
                already_high = (
                    xy_priority
                    and float(self.latest_down_m)
                    < float(down_m) - high_threshold
                )
                skip_horizontal_when_high = bool(
                    self.get_parameter(
                        "search_high_altitude_skip_horizontal_avoidance"
                    ).value
                )
                try_horizontal = (
                    not xy_priority
                    or (
                        horizontal_avoidance_successes < horizontal_budget
                        and not (
                            already_high
                            and skip_horizontal_when_high
                        )
                    )
                )

                if try_horizontal:
                    avoided_horizontally = (
                        await self._perform_horizontal_avoidance(
                            north_m,
                            east_m,
                            command_yaw_deg,
                            resume_status,
                        )
                    )
                    if avoided_horizontally is None:
                        return False
                else:
                    avoided_horizontally = False
                    reason = (
                        "이미 높은 회피 고도"
                        if already_high
                        else "수평 회피 예산 소진"
                    )
                    self.get_logger().warning(
                        f"XY 우선 수색({reason}): 추가 좌우 우회를 생략하고 "
                        "원래 Waypoint 방향의 상승 회피로 전환합니다."
                    )

                if avoided_horizontally:
                    horizontal_avoidance_successes += 1
                    if keep_high_altitude:
                        commanded_down_m = min(
                            float(commanded_down_m),
                            float(self.latest_down_m),
                            float(down_m),
                        )
                    else:
                        commanded_down_m = float(down_m)
                else:
                    vertical_down_m = await self._perform_vertical_avoidance(
                        planned_down_m=down_m,
                        target_north_m=north_m,
                        target_east_m=east_m,
                        yaw_deg=command_yaw_deg,
                        resume_status=resume_status,
                        highest_allowed_down=avoidance_ceiling_down_m,
                        maintain_high_for_search=keep_high_altitude,
                    )
                    if vertical_down_m is None:
                        if (
                            self.stop_search_event is not None
                            and self.stop_search_event.is_set()
                        ):
                            return False

                        # 구조 수색 Waypoint는 한 지점에서 여러 차례
                        # Hover 재계획하지 않는다. 수평·수직 회피가 모두
                        # 실패하면 현재 점만 포기하고 다음 수색점으로 이어간다.
                        if allow_waypoint_skip:
                            self.get_logger().warning(
                                "수평·수직 회피가 모두 실패한 수색 Waypoint를 "
                                "즉시 건너뛰고 다음 수색점으로 이동합니다."
                            )
                            self._publish_status(resume_status)
                            return True

                        avoidance_replans += 1
                        max_replans = max(
                            0,
                            int(
                                self.get_parameter(
                                    "avoidance_replan_attempts"
                                ).value
                            ),
                        )
                        await self._hold_current_position(
                            stop_search=False
                        )
                        if avoidance_replans <= max_replans:
                            self._publish_status(
                                "AVOIDANCE_REPLANNING"
                            )
                            retry_hover = max(
                                0.1,
                                float(
                                    self.get_parameter(
                                        "avoidance_retry_hover_sec"
                                    ).value
                                ),
                            )
                            self.get_logger().warning(
                                f"필수 이동 회피 경로 재계획 "
                                f"{avoidance_replans}/{max_replans}: "
                                f"{retry_hover:.1f}초 Hover 후 LiDAR 재검사"
                            )
                            await self._sleep_sim_time(retry_hover)
                            deadline += (
                                self._sim_time_sec()
                                - avoidance_started
                            )
                            continue

                        self._publish_status(
                            "ERROR_AVOIDANCE_EXHAUSTED"
                        )
                        await self._hold_current_position(
                            stop_search=True
                        )
                        return False

                    commanded_down_m = float(vertical_down_m)
                    horizontal_avoidance_successes = horizontal_budget

                avoidance_replans = 0

                # 장애물 회피에 사용한 시간은 Waypoint 이동 제한시간에서
                # 제외한다. 드론 정지가 아니라 타이머만 연장하는 처리다.
                deadline += self._sim_time_sec() - avoidance_started

                # 우회가 끝나면 원래 Waypoint 방향으로 다시 회전한 뒤,
                # 유지 중인 안전고도에서 원래 XY를 직접 명령한다.
                command_yaw_deg = await self._align_yaw_to_target(
                    north_m,
                    east_m,
                    command_yaw_deg,
                )
                self._publish_movement_direction(north_m, east_m)
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(
                        north_m,
                        east_m,
                        commanded_down_m,
                        command_yaw_deg,
                    )
                )

            horizontal_error = math.hypot(
                self.latest_north_m - north_m,
                self.latest_east_m - east_m,
            )
            altitude_error = abs(
                self.latest_down_m - commanded_down_m
            )
            horizontal_tolerance = float(
                self.get_parameter(
                    "waypoint_acceptance_radius_m"
                ).value
            )
            altitude_tolerance = float(
                self.get_parameter(
                    "waypoint_altitude_tolerance_m"
                ).value
            )

            if xy_priority:
                # 계획 고도는 최소 안전고도로만 사용한다. 현재 고도가 그보다
                # 높다면 하강을 기다리지 않고 XY 도달만으로 방문을 완료한다.
                safe_altitude_reached = (
                    self.latest_down_m
                    <= float(down_m) + altitude_tolerance
                )
                if (
                    horizontal_error <= horizontal_tolerance
                    and safe_altitude_reached
                ):
                    if (
                        self.latest_down_m
                        < float(down_m) - altitude_tolerance
                    ):
                        self.get_logger().info(
                            "XY 우선 수색 Waypoint 도달: "
                            f"N={north_m:.1f}, E={east_m:.1f}, "
                            f"계획D={down_m:.1f}, "
                            f"유지고도D={self.latest_down_m:.1f}"
                        )
                    return True
            else:
                # 조난자 접근·복귀 같은 필수 이동은 기존처럼 XYZ를 모두
                # 만족한다. 높은 회피 고도는 목표 XY에서 안전할 때 복귀한다.
                if (
                    horizontal_error <= horizontal_tolerance
                    and commanded_down_m
                    < down_m - altitude_tolerance
                    and not self.obstacle_blocked
                ):
                    commanded_down_m = float(down_m)
                    self.get_logger().info(
                        f"회피 종료 후 계획 고도 복귀: D={down_m:.1f}"
                    )
                    await self.drone.offboard.set_position_ned(
                        PositionNedYaw(
                            north_m,
                            east_m,
                            commanded_down_m,
                            command_yaw_deg,
                        )
                    )
                    await asyncio.sleep(0.1)
                    continue

                if (
                    horizontal_error <= horizontal_tolerance
                    and altitude_error <= altitude_tolerance
                ):
                    return True

            await asyncio.sleep(0.2)

        if allow_waypoint_skip:
            self.get_logger().warning(
                f"수색 Waypoint 시간 초과로 건너뜀: N={north_m:.1f}, "
                f"E={east_m:.1f}, D={down_m:.1f}"
            )
            await self._hold_current_position(stop_search=False)
            self._publish_status(resume_status)
            return True

        self.get_logger().error(
            f"Waypoint 시간 초과: N={north_m:.1f}, "
            f"E={east_m:.1f}, D={down_m:.1f}"
        )
        self._publish_status("ERROR_WAYPOINT_TIMEOUT")
        await self._hold_current_position(stop_search=True)
        return False

    async def _perform_horizontal_avoidance(
        self,
        target_north_m,
        target_east_m,
        yaw_deg,
        resume_status="SEARCHING",
    ):
        """A*를 먼저 쓰고 VFH 좌우 우회까지 실패하면 상승 회피로 넘긴다."""
        if not await self._brake_before_avoidance():
            # 감속 실패 한 번을 드론 치명 오류로 취급하지 않는다.
            self._publish_status("AVOIDANCE_REPLANNING")
            return False

        attempted_targets = set()
        ignore_local_detour = False
        attempt_count = max(
            1,
            int(self.get_parameter("avoidance_direction_attempts").value),
        )
        minimum_forward_progress = max(
            0.0,
            float(
                self.get_parameter(
                    "avoidance_min_forward_progress_m"
                ).value
            ),
        )

        for _ in range(attempt_count):
            detour_age = self._sim_time_sec() - self.local_detour_received_at
            detour_max_age = float(
                self.get_parameter("local_detour_max_age_sec").value
            )
            local_detour_fresh = (
                self.local_detour_valid
                and detour_age >= 0.0
                and detour_age <= detour_max_age
            )
            vector_age = (
                self._sim_time_sec()
                - self.avoidance_direction_received_at
            )
            vector_fresh = (
                self.avoidance_direction_valid
                and vector_age >= 0.0
                and vector_age
                <= float(
                    self.get_parameter(
                        "avoidance_vector_max_age_sec"
                    ).value
                )
            )

            if local_detour_fresh and not ignore_local_detour:
                body_x = float(self.local_detour_body_x_m)
                body_y = float(self.local_detour_body_y_m)
                path_source = "로컬A*"
            elif vector_fresh:
                # A*가 없거나 전진성이 부족하면 VFH의 좌우 빈 섹터를
                # 실제 fallback으로 사용한다.
                body_angle = float(self.avoidance_direction_body_rad)
                probe_distance = max(
                    1.0,
                    float(
                        self.get_parameter(
                            "avoidance_probe_distance_m"
                        ).value
                    ),
                )
                clearance = max(
                    1.0,
                    float(self.avoidance_direction_clearance_m),
                )
                detour_distance = min(
                    probe_distance,
                    clearance * 0.6,
                )
                body_x = math.cos(body_angle) * detour_distance
                body_y = math.sin(body_angle) * detour_distance
                path_source = "VFH"
            else:
                self.get_logger().warning(
                    "로컬 A*와 VFH 모두 안전한 수평 경로를 찾지 못함"
                )
                return False

            detour_distance = math.hypot(body_x, body_y)
            if detour_distance < 0.5:
                if path_source == "로컬A*":
                    ignore_local_detour = True
                    continue
                return False

            body_angle = math.atan2(body_y, body_x)
            target_key = (
                path_source,
                round(body_x, 1),
                round(body_y, 1),
            )
            if target_key in attempted_targets:
                if path_source == "로컬A*":
                    ignore_local_detour = True
                await asyncio.sleep(0.2)
                continue
            attempted_targets.add(target_key)

            # ROS LiDAR body의 +Y는 왼쪽(CCW), PX4 NED yaw의 +는
            # 오른쪽(CW)이므로 body 각도의 부호를 뒤집는다.
            world_heading_rad = (
                math.radians(self.latest_yaw_deg) - body_angle
            )
            direction_north = math.cos(world_heading_rad)
            direction_east = math.sin(world_heading_rad)
            detour_north = (
                self.latest_north_m
                + direction_north * detour_distance
            )
            detour_east = (
                self.latest_east_m
                + direction_east * detour_distance
            )
            detour_down = self.latest_down_m

            # 순수 측면 또는 후진 우회는 조밀한 산림에서 다시 끼기 쉽다.
            # 원래 Waypoint 방향 성분이 기준보다 작으면 해당 후보를 버리고
            # A* 후보라면 VFH로, VFH 후보라면 상승 회피로 넘어간다.
            target_delta_n = target_north_m - self.latest_north_m
            target_delta_e = target_east_m - self.latest_east_m
            remaining_to_target = math.hypot(
                target_delta_n,
                target_delta_e,
            )
            if remaining_to_target > 1.0e-6:
                target_unit_n = target_delta_n / remaining_to_target
                target_unit_e = target_delta_e / remaining_to_target
                movement_n = detour_north - self.latest_north_m
                movement_e = detour_east - self.latest_east_m
                forward_progress = (
                    movement_n * target_unit_n
                    + movement_e * target_unit_e
                )
                lateral_progress = abs(
                    movement_n * (-target_unit_e)
                    + movement_e * target_unit_n
                )
            else:
                forward_progress = detour_distance
                lateral_progress = 0.0
            required_progress = min(
                minimum_forward_progress,
                max(0.0, remaining_to_target - 0.4),
            )
            if forward_progress < required_progress:
                self.get_logger().warning(
                    f"{path_source} 후보 전진성 부족: "
                    f"진행={forward_progress:.2f}m, "
                    f"필요={required_progress:.2f}m → "
                    + ("VFH 재검사" if path_source == "로컬A*" else "상승 회피")
                )
                if path_source == "로컬A*":
                    ignore_local_detour = True
                    self._publish_movement_direction(
                        target_north_m,
                        target_east_m,
                    )
                    await self._sleep_sim_time(0.1)
                    continue
                return False

            max_lateral = max(
                0.2,
                float(
                    self.get_parameter(
                        "avoidance_max_lateral_offset_m"
                    ).value
                ),
            )
            max_lateral_ratio = max(
                0.0,
                float(
                    self.get_parameter(
                        "avoidance_max_lateral_to_forward_ratio"
                    ).value
                ),
            )
            ratio_limit = max(
                0.5,
                max(0.0, forward_progress) * max_lateral_ratio,
            )
            if (
                lateral_progress > max_lateral
                or lateral_progress > ratio_limit
            ):
                self.get_logger().warning(
                    f"{path_source} 후보 측면 편차 과다: "
                    f"전진={forward_progress:.2f}m, "
                    f"측면={lateral_progress:.2f}m → "
                    + (
                        "VFH 재검사"
                        if path_source == "로컬A*"
                        else "상승 회피"
                    )
                )
                if path_source == "로컬A*":
                    ignore_local_detour = True
                    self._publish_movement_direction(
                        target_north_m,
                        target_east_m,
                    )
                    await self._sleep_sim_time(0.1)
                    continue
                return False

            # 아직 이동하지 않고 분석 중심만 추천 방향으로 바꿔 재검증한다.
            self._publish_movement_direction(detour_north, detour_east)
            await self._sleep_sim_time(
                float(
                    self.get_parameter(
                        "avoidance_direction_check_sec"
                    ).value
                )
            )
            required_front = float(
                self.get_parameter("avoidance_front_clearance_m").value
            )
            direction_is_clear = (
                self.local_detour_nearest_360_m
                >= float(
                    self.get_parameter(
                        "local_detour_hard_stop_distance_m"
                    ).value
                )
                and (
                    (
                        math.isfinite(self.front_clearance_m)
                        and self.front_clearance_m >= required_front
                    )
                    or (
                        math.isinf(self.front_clearance_m)
                        and detour_distance >= 0.5
                    )
                )
            )
            if not direction_is_clear:
                self.get_logger().warning(
                    f"{path_source} 경로각 "
                    f"{math.degrees(body_angle):.0f}° 사전검사 실패: "
                    f"전방={self.front_clearance_m:.2f}m"
                )
                if path_source == "로컬A*":
                    ignore_local_detour = True
                self._publish_movement_direction(
                    target_north_m,
                    target_east_m,
                )
                await self._sleep_sim_time(0.1)
                continue

            self._publish_status("AVOIDING_OBSTACLE_XY")
            side_name = (
                "LEFT"
                if body_y > 0.15
                else "RIGHT"
                if body_y < -0.15
                else "FORWARD"
            )
            self.get_logger().warning(
                f"{path_source} {side_name} 우회 승인: body="
                f"({body_x:.2f}, {body_y:.2f})m, "
                f"전진성={forward_progress:.2f}m, "
                f"임시점 N={detour_north:.1f}, "
                f"E={detour_east:.1f}, D={detour_down:.1f}"
            )

            # 짧은 로컬 우회 중에는 현재 Yaw를 유지해, 계산한 body 기준
            # 경로와 LiDAR 방향이 함께 회전하는 문제를 막는다.
            keep_yaw = bool(
                self.get_parameter(
                    "avoidance_keep_yaw_during_detour"
                ).value
            )
            detour_yaw_deg = (
                float(self.latest_yaw_deg)
                if keep_yaw
                else await self._align_yaw_to_target(
                    detour_north,
                    detour_east,
                    yaw_deg,
                )
            )
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    detour_north,
                    detour_east,
                    detour_down,
                    detour_yaw_deg,
                )
            )

            movement_started_at = self._sim_time_sec()
            blocked_since = None
            commit_sec = max(
                0.0,
                float(
                    self.get_parameter("avoidance_commit_sec").value
                ),
            )
            block_confirm_sec = max(
                0.0,
                float(
                    self.get_parameter(
                        "avoidance_block_confirm_sec"
                    ).value
                ),
            )
            deadline = movement_started_at + float(
                self.get_parameter("avoidance_xy_timeout_sec").value
            )
            while self._sim_time_sec() < deadline:
                if self.stop_search_event.is_set():
                    return False
                self._publish_movement_direction(
                    detour_north,
                    detour_east,
                )
                horizontal_error = math.hypot(
                    self.latest_north_m - detour_north,
                    self.latest_east_m - detour_east,
                )
                if horizontal_error <= 1.0:
                    self._publish_status(resume_status)
                    return True

                hard_stop = float(
                    self.get_parameter(
                        "local_detour_hard_stop_distance_m"
                    ).value
                )
                now = self._sim_time_sec()
                emergency_blocked = (
                    self.local_detour_nearest_360_m < hard_stop
                )
                front_blocked = (
                    math.isfinite(self.front_clearance_m)
                    and self.front_clearance_m
                    < float(
                        self.get_parameter(
                            "avoidance_front_clearance_m"
                        ).value
                    )
                )

                confirmed_blocked = emergency_blocked
                if (
                    not emergency_blocked
                    and now - movement_started_at >= commit_sec
                    and front_blocked
                ):
                    if blocked_since is None:
                        blocked_since = now
                    elif now - blocked_since >= block_confirm_sec:
                        confirmed_blocked = True
                else:
                    blocked_since = None

                if confirmed_blocked:
                    self.get_logger().warning(
                        f"{path_source} {side_name} 우회 중 "
                        "지속 장애물 감지 → 다음 수평 후보 재탐색"
                    )
                    await self._brake_before_avoidance()
                    if path_source == "로컬A*":
                        ignore_local_detour = True
                    self._publish_movement_direction(
                        target_north_m,
                        target_east_m,
                    )
                    await self._sleep_sim_time(0.1)
                    break
                await asyncio.sleep(0.1)

        self.get_logger().warning(
            "A*와 VFH 좌우 수평 경로가 모두 부적합 → 상승 회피"
        )
        return False

    async def _brake_before_avoidance(self):
        """현재 위치 Setpoint로 감속하고 거의 정지한 뒤 회피를 계획한다."""
        hold_north = self.latest_north_m
        hold_east = self.latest_east_m
        hold_down = self.latest_down_m
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(
                hold_north,
                hold_east,
                hold_down,
                self.latest_yaw_deg,
            )
        )
        deadline = self._sim_time_sec() + float(
            self.get_parameter("avoidance_brake_timeout_sec").value
        )
        stopped_speed = float(
            self.get_parameter("avoidance_stopped_speed_m_s").value
        )
        while self._sim_time_sec() < deadline:
            speed = math.sqrt(
                self.latest_velocity_north_m_s ** 2
                + self.latest_velocity_east_m_s ** 2
                + self.latest_velocity_down_m_s ** 2
            )
            if speed <= stopped_speed:
                return True
            await asyncio.sleep(0.1)
        self.get_logger().error("장애물 앞 감속 실패: 안전 속도에 도달하지 못함")
        return False

    async def _perform_vertical_avoidance(
        self,
        planned_down_m,
        target_north_m,
        target_east_m,
        yaw_deg,
        resume_status="SEARCHING",
        highest_allowed_down=None,
        maintain_high_for_search=False,
    ):
        """수평 우회 실패 시 충분히 상승해 장애물 위로 통과한다.

        반환값
        ------
        float:
            이후 원래 Waypoint까지 유지할 NED Down 고도다. 안전하게
            하강했으면 ``planned_down_m``, 하강 지점을 찾지 못했으면 높은
            회피 고도를 반환한다.
        None:
            최대 상승고도에서도 전진 통로를 확보하지 못한 경우다.
        """
        self._publish_status("AVOIDING_OBSTACLE_VERTICAL_CLIMB")

        initial_front_clearance_m = float(self.front_clearance_m)
        initial_proactive_distance_m = float(
            self.proactive_obstacle_distance_m
        )
        start_down_m = float(self.latest_down_m)

        step = max(
            0.5,
            float(self.get_parameter("avoidance_climb_step_m").value),
        )
        max_climb = max(
            step,
            float(self.get_parameter("avoidance_max_climb_m").value),
        )
        minimum_climb = max(
            step,
            float(
                self.get_parameter(
                    "vertical_escape_min_climb_m"
                ).value
            ),
        )
        retry_climb_step = max(
            step,
            float(
                self.get_parameter(
                    "vertical_escape_retry_climb_step_m"
                ).value
            ),
        )
        cross_retries = max(
            0,
            int(
                self.get_parameter(
                    "vertical_escape_cross_retries"
                ).value
            ),
        )
        keep_high_on_descent_failure = bool(
            self.get_parameter(
                "vertical_escape_keep_high_on_descent_failure"
            ).value
        )
        settle = max(
            0.2,
            float(self.get_parameter("avoidance_settle_sec").value),
        )
        climb_timeout = max(
            2.0,
            float(
                self.get_parameter(
                    "avoidance_climb_timeout_sec"
                ).value
            ),
        )
        altitude_tolerance = max(
            0.1,
            float(
                self.get_parameter(
                    "avoidance_altitude_tolerance_m"
                ).value
            ),
        )
        step_retries = max(
            0,
            int(
                self.get_parameter(
                    "avoidance_climb_step_retries"
                ).value
            ),
        )

        if highest_allowed_down is None:
            reference_down_m = min(planned_down_m, start_down_m)
            highest_allowed_down = max(
                self.safe_return_down_m,
                reference_down_m - max_climb,
            )

        # 장애물이 한 프레임 clear가 됐다는 이유로 현재 고도에서 바로
        # 전진하지 않는다. 시작 고도보다 minimum_climb만큼 높은 위치를
        # 반드시 먼저 확보한다.
        minimum_escape_down_m = max(
            float(highest_allowed_down),
            start_down_m - minimum_climb,
        )
        self.get_logger().info(
            f"회피 상승 범위: 현재D={start_down_m:.1f}, "
            f"WaypointD={planned_down_m:.1f}, "
            f"최소회피D={minimum_escape_down_m:.1f}, "
            f"최고D={float(highest_allowed_down):.1f}"
        )

        # 1) 최소 회피 고도까지는 LiDAR clear 여부와 무관하게 상승한다.
        # 그 높이에서도 막혀 있으면 최대 상승한도까지 단계적으로 올린다.
        while True:
            if (
                self.stop_search_event is not None
                and self.stop_search_event.is_set()
            ):
                return None

            minimum_height_reached = (
                self.latest_down_m
                <= minimum_escape_down_m + altitude_tolerance
            )
            if minimum_height_reached:
                clear_confirmed = await self._wait_for_clear_corridor(
                    settle,
                    reactive_only=True,
                )
                if clear_confirmed:
                    break

            if (
                self.latest_down_m
                <= float(highest_allowed_down) + altitude_tolerance
            ):
                self.get_logger().warning(
                    "최대 회피 상승고도에서도 현재 LiDAR 통로가 차단됨: "
                    "현재 위치에서 안전하게 전진할 수 없습니다."
                )
                self._publish_status("AVOIDANCE_REPLANNING")
                await self._hold_current_position(stop_search=False)
                return None

            next_down = max(
                float(highest_allowed_down),
                self.latest_down_m - step,
            )
            self.get_logger().warning(
                f"장애물 회피 강제 상승: "
                f"D={self.latest_down_m:.1f} → {next_down:.1f}"
            )
            reached = await self._command_vertical_level(
                self.latest_north_m,
                self.latest_east_m,
                next_down,
                self.latest_yaw_deg,
                climb_timeout,
                altitude_tolerance,
                ascending=True,
                retries=step_retries,
            )
            if not reached:
                self.get_logger().warning(
                    "상승 회피 단계 고도 도달 실패 → 현재 위치 Hover"
                )
                self._publish_status("AVOIDANCE_REPLANNING")
                await self._hold_current_position(stop_search=False)
                return None

        escape_down_m = float(self.latest_down_m)
        self.get_logger().info(
            f"수직 회피 고도 확보: 시작D={start_down_m:.1f}, "
            f"회피D={escape_down_m:.1f}, "
            f"실제상승={start_down_m - escape_down_m:.1f}m"
        )

        min_forward = max(
            1.0,
            float(
                self.get_parameter(
                    "vertical_escape_min_forward_m"
                ).value
            ),
        )
        max_forward = max(
            min_forward,
            float(
                self.get_parameter(
                    "vertical_escape_max_forward_m"
                ).value
            ),
        )
        pass_margin = max(
            0.5,
            float(
                self.get_parameter(
                    "vertical_escape_obstacle_pass_margin_m"
                ).value
            ),
        )
        obstacle_distance_candidates = [
            value
            for value in (
                initial_front_clearance_m,
                initial_proactive_distance_m,
            )
            if math.isfinite(value) and value >= 0.0
        ]
        if obstacle_distance_candidates:
            estimated_obstacle_distance = min(
                obstacle_distance_candidates
            )
            initial_cross_distance = min(
                max_forward,
                max(
                    min_forward,
                    estimated_obstacle_distance + pass_margin,
                ),
            )
        else:
            estimated_obstacle_distance = float("inf")
            initial_cross_distance = min_forward
        self.get_logger().info(
            "수직 회피 통과거리 계산: "
            f"장애물거리="
            f"{estimated_obstacle_distance if math.isfinite(estimated_obstacle_distance) else 'UNKNOWN'}, "
            f"통과거리={initial_cross_distance:.1f}m"
        )

        extra_forward = max(
            0.5,
            float(
                self.get_parameter(
                    "vertical_escape_extra_forward_m"
                ).value
            ),
        )
        descent_retries = max(
            0,
            int(
                self.get_parameter(
                    "vertical_escape_descent_retries"
                ).value
            ),
        )

        # 2) 높은 고도에서 원래 Waypoint 방향으로 통과한다. 전진 중 다시
        # 막히면 추가 상승 후 같은 방향으로 재시도한다.
        for descent_attempt in range(descent_retries + 1):
            requested_forward = (
                initial_cross_distance
                if descent_attempt == 0
                else extra_forward
            )
            cross_completed = False

            for cross_attempt in range(cross_retries + 1):
                remaining_to_target = math.hypot(
                    target_north_m - self.latest_north_m,
                    target_east_m - self.latest_east_m,
                )
                if remaining_to_target <= 0.4:
                    cross_completed = True
                    break

                cross_distance = min(
                    remaining_to_target,
                    requested_forward,
                )
                if await self._cross_obstacle_at_escape_altitude(
                    target_north_m=target_north_m,
                    target_east_m=target_east_m,
                    escape_down_m=escape_down_m,
                    yaw_deg=yaw_deg,
                    cross_distance_m=cross_distance,
                ):
                    cross_completed = True
                    break

                if (
                    cross_attempt >= cross_retries
                    or self.latest_down_m
                    <= float(highest_allowed_down)
                    + altitude_tolerance
                ):
                    break

                next_escape_down = max(
                    float(highest_allowed_down),
                    self.latest_down_m - retry_climb_step,
                )
                self.get_logger().warning(
                    "상승 고도 전진이 막혀 추가 상승 후 재시도: "
                    f"{cross_attempt + 1}/{cross_retries}, "
                    f"D={self.latest_down_m:.1f} → "
                    f"{next_escape_down:.1f}"
                )
                self._publish_status(
                    "AVOIDING_OBSTACLE_VERTICAL_CLIMB"
                )
                reached = await self._command_vertical_level(
                    self.latest_north_m,
                    self.latest_east_m,
                    next_escape_down,
                    self.latest_yaw_deg,
                    climb_timeout,
                    altitude_tolerance,
                    ascending=True,
                    retries=step_retries,
                )
                if not reached:
                    break
                escape_down_m = float(self.latest_down_m)
                await self._wait_for_clear_corridor(
                    settle,
                    reactive_only=True,
                )

            if not cross_completed:
                self.get_logger().warning(
                    "최대 허용 상승고도까지 재시도했지만 "
                    "상승 고도 전진 통로를 확보하지 못했습니다."
                )
                self._publish_status("AVOIDANCE_REPLANNING")
                await self._hold_current_position(stop_search=False)
                return None

            clear_hold = max(
                0.2,
                float(
                    self.get_parameter(
                        "vertical_escape_clear_hold_sec"
                    ).value
                ),
            )
            if not await self._wait_for_clear_corridor(
                clear_hold,
                reactive_only=True,
            ):
                self.get_logger().warning(
                    "통과 후 현재 고도 통로가 다시 막힘 → "
                    "추가 전진 또는 상승 재검사"
                )
                if descent_attempt >= descent_retries:
                    break
                continue

            if maintain_high_for_search:
                self.get_logger().warning(
                    "XY 우선 수색: 장애물 상공 통과 후 계획 고도로 "
                    f"하강하지 않고 D={escape_down_m:.1f}를 유지한 채 "
                    "원래 Waypoint XY로 이동합니다."
                )
                self._publish_status(resume_status)
                return float(escape_down_m)

            self._publish_status(
                "AVOIDING_OBSTACLE_VERTICAL_DESCENT"
            )
            descent_ok = await self._descend_after_vertical_escape(
                planned_down_m=planned_down_m,
                escape_down_m=escape_down_m,
                yaw_deg=yaw_deg,
                altitude_tolerance=altitude_tolerance,
            )
            if descent_ok:
                self.get_logger().info(
                    f"수직 회피 완료: 장애물 통과 후 계획 고도 "
                    f"D={planned_down_m:.1f} 복귀"
                )
                self._publish_status(resume_status)
                return float(planned_down_m)

            if descent_attempt >= descent_retries:
                break

            self.get_logger().warning(
                f"하강 중 장애물 재감지: 회피 고도로 복귀 후 "
                f"추가 전진 {descent_attempt + 1}/{descent_retries}"
            )
            if not await self._command_vertical_level(
                self.latest_north_m,
                self.latest_east_m,
                escape_down_m,
                self.latest_yaw_deg,
                climb_timeout,
                altitude_tolerance,
                ascending=True,
                retries=step_retries,
            ):
                return None

        # 3) 숲이 조밀해 안전한 하강 지점을 못 찾았으면 Waypoint를
        # 건너뛰지 않고 높은 회피 고도를 유지한 채 목표 XY까지 간다.
        if keep_high_on_descent_failure:
            if self.latest_down_m > escape_down_m + altitude_tolerance:
                reached = await self._command_vertical_level(
                    self.latest_north_m,
                    self.latest_east_m,
                    escape_down_m,
                    self.latest_yaw_deg,
                    climb_timeout,
                    altitude_tolerance,
                    ascending=True,
                    retries=step_retries,
                )
                if not reached:
                    return None

            self.get_logger().warning(
                "안전한 하강 지점을 찾지 못해 수색점을 건너뛰지 않습니다. "
                f"높은 회피 고도 D={escape_down_m:.1f}를 유지한 채 "
                "원래 Waypoint까지 비행합니다."
            )
            self._publish_status(resume_status)
            return float(escape_down_m)

        self.get_logger().warning(
            "수직 회피 후 안전한 하강 지점을 찾지 못함 → 재계획"
        )
        self._publish_status("AVOIDANCE_REPLANNING")
        await self._hold_current_position(stop_search=False)
        return None

    async def _cross_obstacle_at_escape_altitude(
        self,
        target_north_m,
        target_east_m,
        escape_down_m,
        yaw_deg,
        cross_distance_m,
    ):
        """높은 회피 고도를 유지한 채 원래 목표 방향으로 전진한다."""
        remaining = math.hypot(
            target_north_m - self.latest_north_m,
            target_east_m - self.latest_east_m,
        )
        if remaining <= 0.4 or cross_distance_m <= 0.2:
            return True
        direction_north = (target_north_m - self.latest_north_m) / remaining
        direction_east = (target_east_m - self.latest_east_m) / remaining
        travel = min(float(cross_distance_m), remaining)
        cross_north = self.latest_north_m + direction_north * travel
        cross_east = self.latest_east_m + direction_east * travel
        cross_tolerance = max(
            0.3,
            float(
                self.get_parameter("vertical_escape_cross_tolerance_m").value
            ),
        )
        timeout = max(
            3.0,
            float(
                self.get_parameter("vertical_escape_cross_timeout_sec").value
            ),
        )
        self._publish_status("AVOIDING_OBSTACLE_VERTICAL_CROSS")
        self.get_logger().warning(
            f"상승 고도 장애물 통과: {travel:.1f}m → "
            f"N={cross_north:.1f}, E={cross_east:.1f}, D={escape_down_m:.1f}"
        )
        self._publish_movement_direction(cross_north, cross_east)
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(
                cross_north,
                cross_east,
                escape_down_m,
                yaw_deg,
            )
        )
        deadline = self._sim_time_sec() + timeout
        blocked_since = None
        while self._sim_time_sec() < deadline:
            if self.stop_search_event is not None and self.stop_search_event.is_set():
                return False
            self._publish_movement_direction(cross_north, cross_east)
            horizontal_error = math.hypot(
                self.latest_north_m - cross_north,
                self.latest_east_m - cross_east,
            )
            hard_stop = float(
                self.get_parameter("local_detour_hard_stop_distance_m").value
            )
            if self.local_detour_nearest_360_m < hard_stop:
                self.get_logger().warning(
                    "상승 고도 전진 중 근접 장애물 감지 → 즉시 Hover"
                )
                await self._brake_before_avoidance()
                return False
            if self.obstacle_blocked:
                if blocked_since is None:
                    blocked_since = self._sim_time_sec()
                elif self._sim_time_sec() - blocked_since >= 0.5:
                    self.get_logger().warning(
                        "상승 고도 전진 통로가 계속 차단됨 → 더 높은 고도 재계획"
                    )
                    await self._brake_before_avoidance()
                    return False
            else:
                blocked_since = None
            if horizontal_error <= cross_tolerance:
                await self._hold_current_position(stop_search=False)
                return True
            await asyncio.sleep(0.1)
        self.get_logger().warning("상승 고도 장애물 통과 시간 초과")
        await self._hold_current_position(stop_search=False)
        return False

    async def _descend_after_vertical_escape(
        self,
        planned_down_m,
        escape_down_m,
        yaw_deg,
        altitude_tolerance,
    ):
        """현재 XY에서 계획 고도로 단계 하강하고 각 단계의 안전을 확인한다."""
        descent_step = max(
            0.3,
            float(
                self.get_parameter("vertical_escape_descent_step_m").value
            ),
        )
        descent_timeout = max(
            2.0,
            float(
                self.get_parameter("vertical_escape_descent_timeout_sec").value
            ),
        )
        clear_sec = max(
            0.1,
            float(
                self.get_parameter("vertical_escape_descent_clear_sec").value
            ),
        )
        while self.latest_down_m < planned_down_m - altitude_tolerance:
            if self.stop_search_event is not None and self.stop_search_event.is_set():
                return False
            next_down = min(planned_down_m, self.latest_down_m + descent_step)
            self.get_logger().info(
                f"회피 후 단계 하강: D={self.latest_down_m:.1f} → "
                f"{next_down:.1f}"
            )
            reached = await self._command_vertical_level(
                self.latest_north_m,
                self.latest_east_m,
                next_down,
                yaw_deg,
                descent_timeout,
                altitude_tolerance,
                ascending=False,
                retries=0,
                abort_on_obstacle=True,
            )
            if not reached:
                # 하강 중 obstacle_blocked 또는 hard stop이 잡힌 경우
                # 현재 고도에서 멈춘 뒤 호출자가 escape_down으로 재상승한다.
                await self._hold_current_position(stop_search=False)
                return False
            if not await self._wait_for_clear_corridor(
                clear_sec, reactive_only=True
            ):
                await self._hold_current_position(stop_search=False)
                return False
        return True

    async def _command_vertical_level(
        self,
        north_m,
        east_m,
        target_down_m,
        yaw_deg,
        timeout_sec,
        altitude_tolerance,
        ascending,
        retries=0,
        abort_on_obstacle=False,
    ):
        """한 단계 수직 이동 명령을 재전송하며 목표 고도 도달을 확인한다."""
        vertical_speed = max(
            0.2,
            float(
                self.get_parameter(
                    "search_vertical_speed_up_m_s"
                    if ascending
                    else "search_vertical_speed_down_m_s"
                ).value
            ),
        )
        for retry_index in range(max(0, int(retries)) + 1):
            start_down = self.latest_down_m
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(north_m, east_m, target_down_m, yaw_deg)
            )
            expected = abs(start_down - target_down_m) / vertical_speed
            deadline = self._sim_time_sec() + max(
                float(timeout_sec), expected * 3.0 + 2.0
            )
            while self._sim_time_sec() < deadline:
                if self.stop_search_event is not None and self.stop_search_event.is_set():
                    return False
                if abort_on_obstacle:
                    hard_stop = float(
                        self.get_parameter(
                            "local_detour_hard_stop_distance_m"
                        ).value
                    )
                    if self._reactive_obstacle_present(hard_stop):
                        self.get_logger().warning(
                            "단계 하강 중 근거리 장애물 감지 → 하강 중단"
                        )
                        return False
                if ascending:
                    reached = self.latest_down_m <= target_down_m + altitude_tolerance
                else:
                    reached = self.latest_down_m >= target_down_m - altitude_tolerance
                if reached:
                    return True
                await asyncio.sleep(0.1)
            if retry_index < max(0, int(retries)):
                self.get_logger().warning(
                    f"수직 단계 미도달: 명령 재전송 "
                    f"{retry_index + 1}/{int(retries)}"
                )
                await self._hold_current_position(stop_search=False)
        return False

    def _reactive_obstacle_present(self, hard_stop_distance=None):
        """현재 고도에서 실제 근접 충돌 위험이 있는지 확인한다.

        선제 A*용 10m 통로 차단은 포함하지 않는다. 수직 회피 후 하강
        가능 여부는 먼 다음 장애물이 아니라 현재 위치 주변의 LiDAR
        여유거리로 판단해야 불필요하게 높은 고도에 머무르지 않는다.
        """
        hard_stop = (
            float(hard_stop_distance)
            if hard_stop_distance is not None
            else float(
                self.get_parameter("local_detour_hard_stop_distance_m").value
            )
        )
        required_front = float(
            self.get_parameter("avoidance_front_clearance_m").value
        )
        front_blocked = (
            math.isfinite(self.front_clearance_m)
            and self.front_clearance_m < required_front
        )
        emergency_blocked = self.local_detour_nearest_360_m < hard_stop
        return bool(front_blocked or emergency_blocked)

    async def _wait_for_clear_corridor(
        self, required_clear_sec, reactive_only=False
    ):
        """진행 통로가 연속해서 clear인 시간을 확인한다."""
        required = max(0.0, float(required_clear_sec))
        deadline = self._sim_time_sec() + max(2.0, required * 4.0)
        clear_since = None
        while self._sim_time_sec() < deadline:
            if self.stop_search_event is not None and self.stop_search_event.is_set():
                return False
            blocked = (
                self._reactive_obstacle_present()
                if reactive_only
                else self.obstacle_blocked
            )
            if blocked:
                clear_since = None
            else:
                if clear_since is None:
                    clear_since = self._sim_time_sec()
                elif self._sim_time_sec() - clear_since >= required:
                    return True
            await asyncio.sleep(0.1)
        return False

    async def _hover(self):
        if self.stop_search_event is not None:
            self.stop_search_event.set()
        try:
            await self._cancel_active_search()
            await self._cancel_active_approach()
            await self._ensure_offboard()
            await self._hold_current_position(stop_search=False)
            self._publish_status("HOVERING")
        except OffboardError as error:
            self.get_logger().error(f"Hover 실패: {error}")
            self._publish_status("ERROR_HOVER")

    async def _hold_current_position(self, stop_search):
        if stop_search and self.stop_search_event is not None:
            self.stop_search_event.set()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(
                self.latest_north_m,
                self.latest_east_m,
                self.latest_down_m,
                self.latest_yaw_deg,
            )
        )

    async def _start_return_home(self):
        if self.return_task and not self.return_task.done():
            return
        if self.stop_search_event is not None:
            self.stop_search_event.set()
        await self._cancel_active_search()
        await self._cancel_active_approach()
        self.return_task = asyncio.create_task(self._return_home())

    async def _cancel_active_search(self):
        """진행 중인 수색/회피가 Hover·Return 상태를 덮어쓰지 않게 취소한다."""
        task = self.search_task
        if task is None or task.done() or task is asyncio.current_task():
            return
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        self.search_task = None

    async def _start_victim_approach(self, target_world_enu):
        """확정된 조난자 위치의 안전 상공으로 LiDAR 회피하며 접근한다."""
        if self.stop_search_event is not None:
            self.stop_search_event.set()
        await self._cancel_active_search()
        await self._cancel_active_approach()
        self.stop_search_event.clear()
        self.approach_task = asyncio.create_task(
            self._approach_victim(target_world_enu)
        )

    async def _approach_victim(self, target_world_enu):
        try:
            await self._ensure_offboard()
            world_x, world_y, world_z = target_world_enu
            target_north = world_y - self.home_world_enu[1]
            target_east = world_x - self.home_world_enu[0]
            target_down = -(world_z - self.home_world_enu[2])
            self._publish_status("APPROACHING_VICTIM")
            self.get_logger().warning(
                f"조난자 안전 상공 접근: world=({world_x:.2f}, "
                f"{world_y:.2f}, {world_z:.2f}), NED=({target_north:.2f}, "
                f"{target_east:.2f}, {target_down:.2f})"
            )
            reached = await self._go_to_setpoint(
                target_north,
                target_east,
                target_down,
                float(self.get_parameter("search_yaw_deg").value),
                float(
                    self.get_parameter("victim_approach_timeout_sec").value
                ),
                allow_avoidance=True,
                resume_status="APPROACHING_VICTIM",
            )
            if not reached:
                return
            await self._hold_current_position(stop_search=False)
            self._publish_status("HOVERING")
            self.get_logger().info("조난자 안전 상공 도착 및 Hover")
        except OffboardError as error:
            self.get_logger().error(f"조난자 접근 실패: {error}")
            self._publish_status("ERROR_VICTIM_APPROACH")
        except asyncio.CancelledError:
            return
        finally:
            self.approach_task = None

    async def _cancel_active_approach(self):
        task = self.approach_task
        if task is None or task.done() or task is asyncio.current_task():
            return
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        self.approach_task = None

    async def _return_home(self):
        """현재~홈 경로의 지형 안전고도로 복귀한 뒤 자동 착륙한다."""
        try:
            await self._ensure_offboard()
            self._publish_status("RETURNING")
            timeout = float(
                self.get_parameter("return_timeout_sec").value
            )

            return_down_m = self._calculate_return_down_m()

            # 1. 경로 안전고도보다 낮을 때만 현재 XY에서 필요한 만큼
            # 상승한다. 이미 충분히 높으면 이 단계를 건너뛴다.
            if return_down_m < self.latest_down_m - 0.3:
                if not await self._go_to_setpoint(
                    self.latest_north_m,
                    self.latest_east_m,
                    return_down_m,
                    self.latest_yaw_deg,
                    timeout,
                ):
                    return
            else:
                self.get_logger().info(
                    "현재 고도가 복귀 경로 안전고도 이상: 추가 상승 없이 복귀"
                )

            # 2. 선택된 안전고도를 유지하며 PX4 Local NED 홈으로 이동한다.
            if not await self._go_to_setpoint(
                0.0,
                0.0,
                return_down_m,
                self.latest_yaw_deg,
                timeout,
            ):
                return

            # 3. 홈 XY를 유지한 채 지면 위 낮은 접근고도까지 정밀 하강한다.
            approach_altitude = max(
                1.5,
                float(
                    self.get_parameter(
                        "landing_approach_altitude_m"
                    ).value
                ),
            )
            approach_down = -approach_altitude
            self._publish_status("LANDING_APPROACH")

            if not await self._go_to_landing_approach(
                approach_down,
                self.latest_yaw_deg,
            ):
                # 홈 XY에는 이미 도착한 상태다. Offboard 고도 setpoint가
                # PX4에서 적용되지 않더라도 공중 Hover로 임무를 끝내지
                # 않고, PX4 자체 LAND 모드가 하강과 접지를 맡도록 한다.
                if bool(
                    self.get_parameter(
                        "landing_fallback_to_px4_land"
                    ).value
                ):
                    self.get_logger().warning(
                        "착륙 접근 하강 실패, 홈 상공에서 PX4 LAND로 전환"
                    )
                    self._publish_status("LANDING_APPROACH_FALLBACK")
                    await self._land()
                    return

                self._publish_status("ERROR_LANDING_APPROACH")
                return

            self._publish_status("HOME_REACHED")
            await self._land()
        except OffboardError as error:
            self.get_logger().error(f"자동 복귀 실패: {error}")
            self._publish_status("ERROR_RETURN_HOME")
        except asyncio.CancelledError:
            return
        finally:
            self.return_task = None

    async def _go_to_landing_approach(self, target_down_m, yaw_deg):
        """홈 상공의 낮은 착륙 접근점에서 위치와 속도가 안정될 때까지 기다린다."""
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(
                0.0,
                0.0,
                float(target_down_m),
                float(yaw_deg),
            )
        )

        timeout = float(
            self.get_parameter(
                "landing_approach_timeout_sec"
            ).value
        )
        xy_tolerance = float(
            self.get_parameter("landing_xy_tolerance_m").value
        )
        altitude_tolerance = float(
            self.get_parameter(
                "landing_altitude_tolerance_m"
            ).value
        )
        stopped_speed = float(
            self.get_parameter("landing_stopped_speed_m_s").value
        )
        deadline = self._sim_time_sec() + timeout
        descent_start_deadline = self._sim_time_sec() + max(
            1.0,
            float(
                self.get_parameter(
                    "landing_descent_start_timeout_sec"
                ).value
            ),
        )
        initial_down_m = float(self.latest_down_m)
        minimum_progress_m = max(
            0.05,
            float(
                self.get_parameter(
                    "landing_min_descent_progress_m"
                ).value
            ),
        )
        stable_samples = 0

        while self._sim_time_sec() < deadline:
            horizontal_error = math.hypot(
                self.latest_north_m,
                self.latest_east_m,
            )
            altitude_error = abs(
                self.latest_down_m - target_down_m
            )
            total_speed = math.sqrt(
                self.latest_velocity_north_m_s ** 2
                + self.latest_velocity_east_m_s ** 2
                + self.latest_velocity_down_m_s ** 2
            )

            if (
                horizontal_error <= xy_tolerance
                and altitude_error <= altitude_tolerance
                and total_speed <= stopped_speed
            ):
                stable_samples += 1
                if stable_samples >= 5:
                    settle_sec = max(
                        0.0,
                        float(
                            self.get_parameter(
                                "landing_settle_sec"
                            ).value
                        ),
                    )
                    self.get_logger().info(
                        "착륙 접근점 안정화 완료: "
                        f"XY오차={horizontal_error:.2f}m, "
                        f"고도오차={altitude_error:.2f}m, "
                        f"속도={total_speed:.2f}m/s, "
                        f"settle={settle_sec:.1f}s"
                    )
                    await self._sleep_sim_time(settle_sec)
                    return True
            else:
                stable_samples = 0

            # 접근 고도 명령 후에도 실제 D가 전혀 증가하지 않으면 남은
            # 전체 타임아웃을 기다리지 않고 PX4 LAND 대체 경로로 넘긴다.
            if (
                self._sim_time_sec() >= descent_start_deadline
                and target_down_m > initial_down_m
                and self.latest_down_m - initial_down_m < minimum_progress_m
            ):
                self.get_logger().warning(
                    "착륙 접근 하강이 시작되지 않음: "
                    f"initial_D={initial_down_m:.2f}, "
                    f"current_D={self.latest_down_m:.2f}, "
                    f"target_D={target_down_m:.2f}"
                )
                return False

            # PX4가 setpoint를 놓치지 않도록 같은 목표를 계속 갱신한다.
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    0.0,
                    0.0,
                    float(target_down_m),
                    float(yaw_deg),
                )
            )
            await asyncio.sleep(0.1)

        self.get_logger().error(
            "착륙 접근점 도달 실패: "
            f"N={self.latest_north_m:.2f}, "
            f"E={self.latest_east_m:.2f}, "
            f"D={self.latest_down_m:.2f}, "
            f"target_D={target_down_m:.2f}"
        )
        return False

    async def _land(self):
        """LAND 명령, 접지, 자동/강제 무장 해제까지 순서대로 확인한다."""
        if self.current_status == "LANDED":
            return
        if self.landing_in_progress:
            self.get_logger().warning("이미 착륙 절차가 진행 중입니다.")
            return

        self.landing_in_progress = True
        if self.stop_search_event is not None:
            self.stop_search_event.set()

        try:
            # Offboard stop 실패가 실제 LAND 명령을 막지 않도록 분리한다.
            if self.offboard_started:
                try:
                    await self.drone.offboard.stop()
                except OffboardError as error:
                    self.get_logger().warning(
                        "Offboard stop 실패, PX4 LAND 명령은 계속합니다: "
                        f"{error}"
                    )
                finally:
                    self.offboard_started = False

            retries = max(
                1,
                int(
                    self.get_parameter(
                        "landing_command_retries"
                    ).value
                ),
            )
            landing_timeout = float(
                self.get_parameter("landing_timeout_sec").value
            )
            touchdown_confirmed = False
            last_error = None

            for attempt in range(1, retries + 1):
                try:
                    await self.drone.action.land()
                    self._publish_status("LANDING")
                    self.get_logger().info(
                        f"PX4 LAND 명령 전송: {attempt}/{retries}"
                    )

                    await asyncio.wait_for(
                        self._wait_until_not_in_air(),
                        timeout=landing_timeout,
                    )
                    touchdown_confirmed = True
                    break
                except (ActionError, asyncio.TimeoutError) as error:
                    last_error = error
                    self.get_logger().warning(
                        f"착륙 명령 {attempt}/{retries} 실패 또는 시간 초과: "
                        f"{error}"
                    )
                    if attempt < retries:
                        await asyncio.sleep(1.0)

            if not touchdown_confirmed:
                raise asyncio.TimeoutError(
                    f"접지 확인 실패: {last_error}"
                )

            await asyncio.sleep(
                max(
                    0.0,
                    float(
                        self.get_parameter(
                            "post_touchdown_settle_sec"
                        ).value
                    ),
                )
            )

            # PX4 자동 disarm을 기다리고, 계속 무장 상태면 지상에서만
            # 명시적 disarm을 한 번 수행한다.
            disarm_timeout = float(
                self.get_parameter("disarm_timeout_sec").value
            )
            try:
                await asyncio.wait_for(
                    self._wait_until_disarmed(),
                    timeout=disarm_timeout,
                )
            except asyncio.TimeoutError:
                self.get_logger().warning(
                    "접지는 확인됐지만 자동 무장 해제가 늦어 "
                    "명시적 DISARM을 수행합니다."
                )
                try:
                    await self.drone.action.disarm()
                except ActionError as error:
                    self.get_logger().warning(
                        f"명시적 DISARM 응답: {error}"
                    )
                await asyncio.wait_for(
                    self._wait_until_disarmed(),
                    timeout=5.0,
                )

            self._publish_status("LANDED")
            self.get_logger().info(
                "착륙 완료: 접지 및 무장 해제 확인"
            )
        except (
            ActionError,
            OffboardError,
            asyncio.TimeoutError,
            TelemetryError,
        ) as error:
            self.get_logger().error(f"착륙 실패: {error}")
            self._publish_status("ERROR_LAND")
        finally:
            self.landing_in_progress = False

    async def _wait_until_not_in_air(self):
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                return

    async def _wait_until_disarmed(self):
        async for armed in self.drone.telemetry.armed():
            if not armed:
                return

    def _sim_time_sec(self):
        """use_sim_time 적용 시 /clock 기준 현재 시각을 초로 반환한다."""
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1.0e9

    async def _sleep_sim_time(self, duration_sec):
        """Pause 중 진행되지 않는 임무용 대기시간이다.

        asyncio 자체는 실제 시간을 사용하므로 짧게 양보하면서 ROS clock의
        경과량을 확인한다. MAVSDK 통신 타임아웃에는 이 함수를 쓰지 않는다.
        """
        duration_sec = max(0.0, float(duration_sec))
        deadline = self._sim_time_sec() + duration_sec
        while self._sim_time_sec() < deadline:
            await asyncio.sleep(0.05)

    @staticmethod
    def _parse_waypoints(value):
        waypoints = []
        for item in value.split(";"):
            fields = [field.strip() for field in item.split(",")]
            if len(fields) != 3:
                continue
            waypoints.append(tuple(float(field) for field in fields))
        if not waypoints:
            raise ValueError("3차원 수색 Waypoint가 비어 있습니다.")
        return waypoints

    def _publish_status(self, status):
        self.current_status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info(f"{self.drone_id} 상태: {status}")

    def _republish_status(self):
        message = String()
        message.data = self.current_status
        self.status_publisher.publish(message)

    def _publish_local_position(self):
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = f"{self.drone_id}/local_ned"
        message.point.x = self.latest_north_m
        message.point.y = self.latest_east_m
        message.point.z = self.latest_down_m
        self.position_publisher.publish(message)

    def _publish_movement_direction(self, target_north_m, target_east_m):
        """다음 목표 방향을 LiDAR body frame 기준 각도로 발행한다."""
        delta_north = target_north_m - self.latest_north_m
        delta_east = target_east_m - self.latest_east_m
        if math.hypot(delta_north, delta_east) < 0.05:
            return
        target_heading_deg = math.degrees(
            math.atan2(delta_east, delta_north)
        )
        # PX4 NED yaw는 시계방향 +, ROS LiDAR body 각도는 반시계방향 +다.
        relative_rad = math.radians(self.latest_yaw_deg - target_heading_deg)
        relative_rad = (
            relative_rad + math.pi
        ) % (2.0 * math.pi) - math.pi
        message = Float32()
        message.data = float(relative_rad)
        self.movement_direction_publisher.publish(message)

    def _publish_map_to_base_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter("map_frame").value)
        transform.child_frame_id = str(
            self.get_parameter("base_frame").value
        )
        transform.transform.translation.x = (
            self.home_world_enu[0] + self.latest_east_m
        )
        transform.transform.translation.y = (
            self.home_world_enu[1] + self.latest_north_m
        )
        transform.transform.translation.z = (
            self.home_world_enu[2] - self.latest_down_m
        )
        yaw_enu = math.radians(90.0 - self.latest_yaw_deg)
        transform.transform.rotation.z = math.sin(0.5 * yaw_enu)
        transform.transform.rotation.w = math.cos(0.5 * yaw_enu)
        self.transform_broadcaster.sendTransform(transform)

    def _publish_search_path(self):
        message = NavPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        for north_m, east_m, down_m in self.search_waypoints:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = self.home_world_enu[0] + east_m
            pose.pose.position.y = self.home_world_enu[1] + north_m
            pose.pose.position.z = self.home_world_enu[2] - down_m
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_publisher.publish(message)
        if self.path_timer is not None:
            self.path_timer.cancel()

    def _make_nav_path(self, waypoints):
        message = NavPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        for north_m, east_m, down_m in waypoints:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = self.home_world_enu[0] + east_m
            pose.pose.position.y = self.home_world_enu[1] + north_m
            pose.pose.position.z = self.home_world_enu[2] - down_m
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def _publish_cooperative_paths(self):
        self.cooperative_transit_path_publisher.publish(
            self._make_nav_path(self.cooperative_transit_waypoints)
        )
        self.cooperative_search_path_publisher.publish(
            self._make_nav_path(self.cooperative_search_waypoints)
        )

    def _clear_cooperative_paths(self):
        self.cooperative_transit_path_publisher.publish(
            self._make_nav_path([])
        )
        self.cooperative_search_path_publisher.publish(
            self._make_nav_path([])
        )

    def destroy_node(self):
        if self.stop_search_event is not None:
            self.async_loop.call_soon_threadsafe(self.stop_search_event.set)
        self.async_loop.call_soon_threadsafe(self.async_loop.stop)
        self.async_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
