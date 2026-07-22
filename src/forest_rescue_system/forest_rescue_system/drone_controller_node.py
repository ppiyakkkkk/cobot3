#!/usr/bin/env python3

"""드론 한 대의 ROS 2 명령을 MAVSDK/PX4 Offboard 제어로 변환한다."""

import asyncio
import json
import math
from pathlib import Path
import threading
import time

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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from tf2_ros import TransformBroadcaster


class DroneControllerNode(Node):
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
        self.declare_parameter("avoidance_climb_step_m", 1.5)
        self.declare_parameter("avoidance_max_climb_m", 8.0)
        self.declare_parameter("avoidance_settle_sec", 1.0)
        self.declare_parameter("avoidance_climb_timeout_sec", 15.0)
        self.declare_parameter("avoidance_altitude_tolerance_m", 0.5)
        self.declare_parameter("avoidance_climb_step_retries", 2)
        self.declare_parameter("avoidance_replan_attempts", 4)
        self.declare_parameter("avoidance_retry_hover_sec", 2.0)
        self.declare_parameter("avoidance_lateral_offset_m", 5.0)
        self.declare_parameter("avoidance_forward_offset_m", 2.0)
        self.declare_parameter("avoidance_side_clearance_m", 5.0)
        self.declare_parameter("avoidance_xy_timeout_sec", 15.0)
        self.declare_parameter("avoidance_brake_timeout_sec", 8.0)
        self.declare_parameter("avoidance_stopped_speed_m_s", 0.60)
        self.declare_parameter("avoidance_direction_check_sec", 0.8)
        self.declare_parameter("avoidance_front_clearance_m", 3.5)
        self.declare_parameter("avoidance_probe_distance_m", 5.0)
        self.declare_parameter("avoidance_direction_attempts", 4)
        self.declare_parameter("avoidance_vector_max_age_sec", 1.2)
        self.declare_parameter("local_detour_max_age_sec", 1.2)
        self.declare_parameter("local_detour_hard_stop_distance_m", 0.9)
        # 짧은 로컬 우회에서는 body 기준 경로가 바뀌지 않도록 현재 Yaw를
        # 유지한다. 이동 직후 센서 방향이 안정될 때까지 일반 차단 판정은
        # 잠시 유예하고, 이후에도 일정 시간 연속 차단일 때만 재계획한다.
        self.declare_parameter("avoidance_keep_yaw_during_detour", True)
        self.declare_parameter("avoidance_commit_sec", 0.70)
        self.declare_parameter("avoidance_block_confirm_sec", 0.35)
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

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.home_world_enu = [
            float(value)
            for value in self.get_parameter("home_world_enu").value
        ]
        self.safe_return_down_m = float(
            self.get_parameter("safe_return_down_m").value
        )
        self.search_waypoints = []
        self._load_search_plan()
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
        self.local_detour_nearest_360_m = 0.0
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

    def _load_search_plan(self):
        plan_path = Path(
            str(self.get_parameter("search_plan_path").value)
        ).expanduser()
        if plan_path.is_file():
            try:
                plan = json.loads(plan_path.read_text(encoding="utf-8"))
                drone_plan = plan["drones"][self.drone_id]
                self.home_world_enu = [
                    float(value)
                    for value in drone_plan["home_world_enu"]
                ]
                # format_version 2 계획과의 호환용이다. 새 계획(v3)은 지도
                # 전체 최고점 기반 safe_return_down_m을 저장하지 않는다.
                if "safe_return_down_m" in drone_plan:
                    self.safe_return_down_m = float(
                        drone_plan["safe_return_down_m"]
                    )
                if "return_path_clearance_m" in plan:
                    self.return_path_clearance_m = float(
                        plan["return_path_clearance_m"]
                    )
                else:
                    self.return_path_clearance_m = float(
                        self.get_parameter("return_path_clearance_m").value
                    )
                if "return_path_corridor_radius_m" in plan:
                    self.return_path_corridor_radius_m = float(
                        plan["return_path_corridor_radius_m"]
                    )
                else:
                    self.return_path_corridor_radius_m = float(
                        self.get_parameter(
                            "return_path_corridor_radius_m"
                        ).value
                    )
                if "return_obstacle_clearance_m" in plan:
                    self.return_obstacle_clearance_m = float(
                        plan["return_obstacle_clearance_m"]
                    )
                else:
                    self.return_obstacle_clearance_m = float(
                        self.get_parameter(
                            "return_obstacle_clearance_m"
                        ).value
                    )
                self.search_waypoints = [
                    (
                        float(item["north_m"]),
                        float(item["east_m"]),
                        float(item["down_m"]),
                    )
                    for item in drone_plan["waypoints"]
                ]
                return
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                self.get_logger().warning(
                    f"수색 계획 파일 파싱 실패, YAML 경로 사용: {error}"
                )

        self.search_waypoints = self._parse_waypoints(
            str(self.get_parameter("search_waypoints").value)
        )

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
        """3대 동시 운용 시 MAVSDK callback queue가 밀리지 않게 제한한다."""
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
            self._submit(self._start_search())
        elif command.startswith("APPROACH_VICTIM:"):
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
            self._submit(self._start_return_home())
        elif command == "LAND":
            self._submit(self._land())
        else:
            self.get_logger().warning(f"알 수 없는 명령: {command}")

    def _obstacle_callback(self, message):
        self.obstacle_blocked = bool(message.data)

    def _obstacle_clearances_callback(self, message):
        self.front_clearance_m = float(message.vector.x)
        self.left_clearance_m = float(message.vector.y)
        self.right_clearance_m = float(message.vector.z)

    def _avoidance_vector_callback(self, message):
        self.avoidance_direction_body_rad = float(message.vector.x)
        self.avoidance_direction_clearance_m = float(message.vector.y)
        self.avoidance_direction_valid = bool(message.vector.z >= 0.5)
        self.avoidance_direction_received_at = time.monotonic()

    def _local_detour_callback(self, message):
        self.local_detour_body_x_m = float(message.vector.x)
        self.local_detour_body_y_m = float(message.vector.y)
        self.local_detour_valid = bool(message.vector.z >= 0.0)
        self.local_detour_nearest_360_m = abs(float(message.vector.z))
        self.local_detour_received_at = time.monotonic()

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
            deadline = time.monotonic() + 40.0
            while time.monotonic() < deadline:
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

    async def _start_search(self):
        if self.search_task and not self.search_task.done():
            self.get_logger().warning("이미 수색 경로를 실행 중입니다.")
            return
        self.stop_search_event.clear()
        self.search_task = asyncio.create_task(self._search_path())

    async def _search_path(self):
        try:
            await self._ensure_offboard()
            self._publish_status("SEARCHING")
            yaw_deg = float(self.get_parameter("search_yaw_deg").value)
            hold_seconds = float(
                self.get_parameter("waypoint_hold_seconds").value
            )

            for index, waypoint in enumerate(self.search_waypoints, start=1):
                if self.stop_search_event.is_set():
                    return
                north_m, east_m, down_m = waypoint
                self.get_logger().info(
                    f"수색 {index}/{len(self.search_waypoints)}: "
                    f"N={north_m:.1f}, E={east_m:.1f}, D={down_m:.1f}"
                )
                reached = await self._go_to_setpoint(
                    north_m,
                    east_m,
                    down_m,
                    yaw_deg,
                    float(self.get_parameter("waypoint_timeout_sec").value),
                    allow_avoidance=True,
                    allow_waypoint_skip=True,
                )
                if not reached:
                    return
                await asyncio.sleep(max(0.0, hold_seconds))

            await self._hold_current_position(stop_search=False)
            self.search_task = None
            self._publish_status("SEARCH_FINISHED_NO_VICTIM")
        except OffboardError as error:
            self.get_logger().error(f"Offboard 수색 실패: {error}")
            self._publish_status("ERROR_OFFBOARD")
        except asyncio.CancelledError:
            # Hover/Return 명령이 오면 진행 중인 회피와 Waypoint 명령을
            # 즉시 끝낸다. 취소는 오류가 아니다.
            return

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

        deadline = time.monotonic() + timeout_sec
        stable_samples = 0

        while time.monotonic() < deadline:
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
                        await asyncio.sleep(settle_sec)
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
        commanded_down_m = down_m
        avoidance_replans = 0
        # 초기 Hover가 낮은 지형 Waypoint보다 이미 높을 수 있다. 상승
        # 한도는 Waypoint와 진입 고도 중 더 높은 쪽을 기준으로 한 번만
        # 계산해 같은 Waypoint의 반복 회피가 한도를 계속 올리지 못하게 한다.
        avoidance_ceiling_down_m = None
        if allow_avoidance:
            reference_down_m = min(down_m, self.latest_down_m)
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
            await asyncio.sleep(0.35)
        if not (allow_avoidance and self.obstacle_blocked):
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    north_m,
                    east_m,
                    commanded_down_m,
                    command_yaw_deg,
                )
            )
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._publish_movement_direction(north_m, east_m)
            if self.stop_search_event.is_set() and allow_avoidance:
                return False
            if allow_avoidance and self.obstacle_blocked:
                avoidance_started = time.monotonic()
                avoided_horizontally = await self._perform_horizontal_avoidance(
                    north_m,
                    east_m,
                    command_yaw_deg,
                    resume_status,
                )
                if avoided_horizontally is None:
                    return False
                if not avoided_horizontally:
                    if not await self._perform_vertical_avoidance(
                        down_m,
                        resume_status,
                        avoidance_ceiling_down_m,
                    ):
                        if (
                            self.stop_search_event is not None
                            and self.stop_search_event.is_set()
                        ):
                            return False
                        avoidance_replans += 1
                        max_replans = max(
                            1,
                            int(
                                self.get_parameter(
                                    "avoidance_replan_attempts"
                                ).value
                            ),
                        )
                        await self._hold_current_position(stop_search=False)
                        if avoidance_replans <= max_replans:
                            self._publish_status("AVOIDANCE_REPLANNING")
                            retry_hover = max(
                                0.2,
                                float(
                                    self.get_parameter(
                                        "avoidance_retry_hover_sec"
                                    ).value
                                ),
                            )
                            self.get_logger().warning(
                                f"회피 경로 재계획 "
                                f"{avoidance_replans}/{max_replans}: "
                                f"{retry_hover:.1f}초 Hover 후 LiDAR 재검사"
                            )
                            await asyncio.sleep(retry_hover)
                            deadline += time.monotonic() - avoidance_started
                            continue

                        if allow_waypoint_skip:
                            self.get_logger().warning(
                                "반복 회피가 불가능한 수색 Waypoint를 건너뛰고 "
                                "다음 수색점에서 경로를 다시 연결합니다."
                            )
                            self._publish_status(resume_status)
                            return True

                        self._publish_status("ERROR_AVOIDANCE_EXHAUSTED")
                        await self._hold_current_position(stop_search=True)
                        return False
                    # 상승 회피 후에는 원래 낮은 고도로 즉시 복귀하지 않는다.
                    commanded_down_m = min(down_m, self.latest_down_m)
                else:
                    commanded_down_m = down_m
                avoidance_replans = 0

                # 장애물 회피에 사용한 시간은 Waypoint 이동 제한시간에서
                # 제외한다. 드론 정지가 아니라 타이머만 연장하는 처리다.
                deadline += time.monotonic() - avoidance_started

                # 수평 우회 중에는 임시점 방향을 바라본다. 우회가 끝나면
                # 원래 Waypoint 방향으로 다시 회전한 뒤 수색을 재개한다.
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
            altitude_error = abs(self.latest_down_m - commanded_down_m)
            horizontal_tolerance = float(
                self.get_parameter("waypoint_acceptance_radius_m").value
            )
            altitude_tolerance = float(
                self.get_parameter("waypoint_altitude_tolerance_m").value
            )

            # 상승 회피 후 목표 XY까지는 높은 고도를 유지한다. 목표점에
            # 도착하고 LiDAR가 깨끗할 때만 계획 고도로 수직 복귀하여,
            # 나무를 넘자마자 대각선으로 급강하하는 동작을 막는다.
            if (
                horizontal_error <= horizontal_tolerance
                and commanded_down_m < down_m - altitude_tolerance
                and not self.obstacle_blocked
            ):
                commanded_down_m = down_m
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
        """정지 후 팽창된 로컬 costmap의 A* 경로로 짧게 우회한다."""
        if not await self._brake_before_avoidance():
            # 감속 실패 한 번을 드론 치명 오류로 취급하지 않는다. 현재
            # 위치 Setpoint를 계속 유지한 뒤 상승/재계획 단계로 넘긴다.
            self._publish_status("AVOIDANCE_REPLANNING")
            return False

        attempted_targets = set()
        attempt_count = int(
            self.get_parameter("avoidance_direction_attempts").value
        )
        for _ in range(max(1, attempt_count)):
            detour_age = time.monotonic() - self.local_detour_received_at
            detour_max_age = float(
                self.get_parameter("local_detour_max_age_sec").value
            )
            local_detour_fresh = (
                self.local_detour_valid
                and detour_age <= detour_max_age
            )
            vector_age = time.monotonic() - self.avoidance_direction_received_at
            vector_fresh = (
                self.avoidance_direction_valid
                and vector_age
                <= float(
                    self.get_parameter("avoidance_vector_max_age_sec").value
                )
            )

            if local_detour_fresh:
                body_x = float(self.local_detour_body_x_m)
                body_y = float(self.local_detour_body_y_m)
                path_source = "로컬A*"
            elif vector_fresh:
                # A* 시작 셀이 팽창 장애물에 막힌 경우에도 VFH의 검증된
                # 빈 섹터로 짧게 빠져나갈 수 있게 실제 fallback으로 쓴다.
                body_angle = float(self.avoidance_direction_body_rad)
                probe_distance = max(
                    1.0,
                    float(
                        self.get_parameter("avoidance_probe_distance_m").value
                    ),
                )
                clearance = max(
                    1.0,
                    float(self.avoidance_direction_clearance_m),
                )
                detour_distance = min(probe_distance, clearance * 0.6)
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
                return False
            body_angle = math.atan2(body_y, body_x)
            target_key = (
                round(body_x, 1),
                round(body_y, 1),
            )
            if target_key in attempted_targets:
                await asyncio.sleep(0.2)
                continue
            attempted_targets.add(target_key)

            # ROS LiDAR body의 +Y는 왼쪽(CCW), PX4 NED yaw의 +는
            # 오른쪽(CW)이므로 body 각도의 부호를 뒤집는다.
            world_heading_rad = math.radians(self.latest_yaw_deg) - body_angle
            direction_north = math.cos(world_heading_rad)
            direction_east = math.sin(world_heading_rad)
            detour_north = self.latest_north_m + direction_north * detour_distance
            detour_east = self.latest_east_m + direction_east * detour_distance
            detour_down = self.latest_down_m

            # 아직 이동하지 않고 분석 중심만 추천 방향으로 바꿔 재검증한다.
            self._publish_movement_direction(detour_north, detour_east)
            await asyncio.sleep(
                float(
                    self.get_parameter("avoidance_direction_check_sec").value
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
                    f"로컬 경로각 {math.degrees(body_angle):.0f}° "
                    "사전검사 실패: "
                    f"전방={self.front_clearance_m:.2f}m"
                )
                continue

            self._publish_status("AVOIDING_OBSTACLE_XY")
            self.get_logger().warning(
                f"{path_source} 우회 승인: body="
                f"({body_x:.2f}, {body_y:.2f})m, "
                f"임시점 N={detour_north:.1f}, E={detour_east:.1f}, "
                f"D={detour_down:.1f}"
            )

            # 로컬 A*/VFH 결과는 현재 body frame을 기준으로 계산됐다.
            # 여기서 임시점 방향으로 Yaw를 먼저 돌리면 센서 기준과 경로가
            # 함께 회전해 방금 안전하다고 계산한 우회가 무효가 된다. 따라서
            # 짧은 우회 동안에는 현재 Yaw를 유지하고 위치만 이동한다.
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

            movement_started_at = time.monotonic()
            blocked_since = None
            commit_sec = max(
                0.0,
                float(self.get_parameter("avoidance_commit_sec").value),
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
            while time.monotonic() < deadline:
                if self.stop_search_event.is_set():
                    return False
                self._publish_movement_direction(detour_north, detour_east)
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
                now = time.monotonic()
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

                # 360° 최소거리가 hard stop보다 작으면 즉시 정지한다.
                # 그 외 전방 차단은 이동 시작 직후 센서 전환 시간을 제외하고
                # 일정 시간 연속될 때만 실제 새 장애물로 확정한다.
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
                        "추천 우회 중 지속 장애물 감지 → 감속·재탐색"
                    )
                    await self._brake_before_avoidance()
                    break
                await asyncio.sleep(0.1)

        self.get_logger().warning("검증된 로컬 XY 경로 없음 → 상승 회피")
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
        deadline = time.monotonic() + float(
            self.get_parameter("avoidance_brake_timeout_sec").value
        )
        stopped_speed = float(
            self.get_parameter("avoidance_stopped_speed_m_s").value
        )
        while time.monotonic() < deadline:
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
        resume_status="SEARCHING",
        highest_allowed_down=None,
    ):
        """나무를 만나면 현재 XY에서 단계적으로 상승한다."""
        self._publish_status("AVOIDING_OBSTACLE")
        step = float(self.get_parameter("avoidance_climb_step_m").value)
        max_climb = float(
            self.get_parameter("avoidance_max_climb_m").value
        )
        settle = float(self.get_parameter("avoidance_settle_sec").value)
        climb_timeout = float(
            self.get_parameter("avoidance_climb_timeout_sec").value
        )
        altitude_tolerance = float(
            self.get_parameter("avoidance_altitude_tolerance_m").value
        )
        step_retries = max(
            0,
            int(
                self.get_parameter("avoidance_climb_step_retries").value
            ),
        )
        if highest_allowed_down is None:
            reference_down_m = min(planned_down_m, self.latest_down_m)
            highest_allowed_down = max(
                self.safe_return_down_m,
                reference_down_m - max_climb,
            )

        self.get_logger().info(
            f"회피 상승 한도: 현재D={self.latest_down_m:.1f}, "
            f"WaypointD={planned_down_m:.1f}, "
            f"최고D={highest_allowed_down:.1f}"
        )

        while self.obstacle_blocked:
            if self.stop_search_event is not None and self.stop_search_event.is_set():
                return False
            next_down = max(
                highest_allowed_down,
                self.latest_down_m - step,
            )
            if (
                self.latest_down_m
                <= highest_allowed_down + altitude_tolerance
                and self.obstacle_blocked
            ):
                self.get_logger().warning(
                    "최대 회피 상승고도에서도 장애물 감지: "
                    "현재 고도에서 수평 경로를 다시 계획합니다."
                )
                self._publish_status("AVOIDANCE_REPLANNING")
                await self._hold_current_position(stop_search=False)
                return False

            self.get_logger().warning(
                f"장애물 회피 상승: D={self.latest_down_m:.1f} → "
                f"{next_down:.1f}"
            )
            step_reached = False
            for retry_index in range(step_retries + 1):
                climb_start_down = self.latest_down_m
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(
                        self.latest_north_m,
                        self.latest_east_m,
                        next_down,
                        self.latest_yaw_deg,
                    )
                )
                # 설정 속도와 이동거리를 함께 반영해 고정 8초보다 현실적인
                # 단계 제한시간을 만든다. PX4 위치 정착 오차도 0.5m까지 허용한다.
                vertical_speed = max(
                    0.2,
                    float(
                        self.get_parameter(
                            "search_vertical_speed_up_m_s"
                        ).value
                    ),
                )
                expected_travel_sec = (
                    abs(climb_start_down - next_down) / vertical_speed
                )
                step_deadline = time.monotonic() + max(
                    climb_timeout,
                    expected_travel_sec * 3.0 + 3.0,
                    settle * 4.0,
                )
                while time.monotonic() < step_deadline:
                    if (
                        self.stop_search_event is not None
                        and self.stop_search_event.is_set()
                    ):
                        return False
                    if self.latest_down_m <= next_down + altitude_tolerance:
                        step_reached = True
                        break
                    await asyncio.sleep(0.1)
                if step_reached:
                    break
                if retry_index < step_retries:
                    self.get_logger().warning(
                        f"상승 단계 미도달: 명령 재전송 "
                        f"{retry_index + 1}/{step_retries}"
                    )
                    await self._hold_current_position(stop_search=False)
                    await asyncio.sleep(max(0.2, settle))
            if not step_reached:
                self.get_logger().warning(
                    "상승 회피 단계 고도 도달 실패: "
                    "Hover 후 수평 경로를 다시 계획합니다."
                )
                self._publish_status("AVOIDANCE_REPLANNING")
                await self._hold_current_position(stop_search=False)
                return False

            # 목표 고도 도달 후에도 LiDAR가 settle 시간 동안 연속해서
            # 깨끗해야만 원래 진행 방향을 다시 명령한다.
            clear_since = None
            validation_deadline = time.monotonic() + max(2.0, settle * 3.0)
            while time.monotonic() < validation_deadline:
                if (
                    self.stop_search_event is not None
                    and self.stop_search_event.is_set()
                ):
                    return False
                if self.obstacle_blocked:
                    clear_since = None
                    await asyncio.sleep(0.1)
                    break
                if clear_since is None:
                    clear_since = time.monotonic()
                elif time.monotonic() - clear_since >= settle:
                    self._publish_status(resume_status)
                    return True
                await asyncio.sleep(0.1)

        if self.stop_search_event is not None and self.stop_search_event.is_set():
            return False
        self._publish_status(resume_status)
        return True

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
        deadline = time.monotonic() + timeout
        descent_start_deadline = time.monotonic() + max(
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

        while time.monotonic() < deadline:
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
                    await asyncio.sleep(settle_sec)
                    return True
            else:
                stable_samples = 0

            # 접근 고도 명령 후에도 실제 D가 전혀 증가하지 않으면 남은
            # 전체 타임아웃을 기다리지 않고 PX4 LAND 대체 경로로 넘긴다.
            if (
                time.monotonic() >= descent_start_deadline
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
