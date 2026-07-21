#!/usr/bin/env python3

"""드론 한 대의 ROS 2 명령을 MAVSDK/PX4 Offboard 제어로 변환한다."""

import asyncio
import json
import math
from pathlib import Path
import threading
import time

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
        self.declare_parameter("local_detour_hard_stop_distance_m", 0.8)
        self.declare_parameter("victim_approach_timeout_sec", 120.0)
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
                self.safe_return_down_m = float(
                    drone_plan["safe_return_down_m"]
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
        self._publish_movement_direction(north_m, east_m)
        # 새 이동 방향이 LiDAR 필터에 반영되기 전에 고속 이동 명령을
        # 보내지 않는다. 첫 PointCloud 판정을 기다린 뒤 경로가 열려
        # 있을 때만 원래 Waypoint를 명령한다.
        if allow_avoidance:
            await asyncio.sleep(0.35)
        if not (allow_avoidance and self.obstacle_blocked):
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(north_m, east_m, commanded_down_m, yaw_deg)
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
                    yaw_deg,
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
                self._publish_movement_direction(north_m, east_m)
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(
                        north_m,
                        east_m,
                        commanded_down_m,
                        yaw_deg,
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
                        yaw_deg,
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
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    detour_north,
                    detour_east,
                    detour_down,
                    yaw_deg,
                )
            )

            deadline = time.monotonic() + float(
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
                path_is_blocked = (
                    self.local_detour_nearest_360_m < hard_stop
                    or (
                        math.isfinite(self.front_clearance_m)
                        and self.front_clearance_m
                        < float(
                            self.get_parameter(
                                "avoidance_front_clearance_m"
                            ).value
                        )
                    )
                )
                if path_is_blocked:
                    self.get_logger().warning(
                        "추천 우회 중 새 장애물 감지 → 즉시 감속·재탐색"
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
        try:
            await self._ensure_offboard()
            self._publish_status("RETURNING")
            timeout = float(self.get_parameter("return_timeout_sec").value)

            # 먼저 지역 최고점보다 높은 공통 안전고도로 상승한다.
            if not await self._go_to_setpoint(
                self.latest_north_m,
                self.latest_east_m,
                self.safe_return_down_m,
                self.latest_yaw_deg,
                timeout,
            ):
                return
            # 안전고도에서 각 PX4의 Local NED 원점(자기 홈)으로 이동한다.
            if not await self._go_to_setpoint(
                0.0,
                0.0,
                self.safe_return_down_m,
                self.latest_yaw_deg,
                timeout,
            ):
                return
            self._publish_status("HOME_REACHED")
            await self._land()
        except OffboardError as error:
            self.get_logger().error(f"자동 복귀 실패: {error}")
            self._publish_status("ERROR_RETURN_HOME")

    async def _land(self):
        if self.stop_search_event is not None:
            self.stop_search_event.set()
        try:
            if self.offboard_started:
                await self.drone.offboard.stop()
                self.offboard_started = False
            await self.drone.action.land()
            self._publish_status("LANDING")

            async def wait_until_landed():
                async for in_air in self.drone.telemetry.in_air():
                    if not in_air:
                        return

            await asyncio.wait_for(wait_until_landed(), timeout=45.0)
            self._publish_status("LANDED")
        except (ActionError, OffboardError, asyncio.TimeoutError) as error:
            self.get_logger().error(f"착륙 실패: {error}")
            self._publish_status("ERROR_LAND")

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
