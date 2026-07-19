#!/usr/bin/env python3

"""ROS 2 명령을 MAVSDK/PX4 Offboard 제어로 변환한다."""

import asyncio
import math
import threading
import time

from geometry_msgs.msg import PointStamped, TransformStamped
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.param import ParamError
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


class DroneControllerNode(Node):
    """검증된 MAVSDK 비행 코드를 비동기 ROS 2 노드로 제공한다."""

    def __init__(self):
        super().__init__("drone_controller_node")

        self.declare_parameter(
            "system_address",
            "udpin://0.0.0.0:14540",
        )
        self.declare_parameter("takeoff_altitude_m", 5.0)
        self.declare_parameter("altitude_acceptance_radius_m", 0.1)
        self.declare_parameter("takeoff_tolerance_m", 0.15)
        self.declare_parameter("search_yaw_deg", 0.0)
        self.declare_parameter("waypoint_hold_seconds", 4.0)
        self.declare_parameter("waypoint_acceptance_radius_m", 0.5)
        self.declare_parameter("waypoint_altitude_tolerance_m", 0.5)
        self.declare_parameter("waypoint_timeout_sec", 15.0)
        self.declare_parameter(
            "search_waypoints",
            "0,0;4,0;8,0;8,4;4,4;0,4;0,8;4,8;8,8",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.status_publisher = self.create_publisher(
            String,
            "/drone/status",
            10,
        )
        self.position_publisher = self.create_publisher(
            PointStamped,
            "/drone/local_position_ned",
            10,
        )
        self.transform_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            String,
            "/drone/command",
            self._command_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/obstacle/blocked",
            self._obstacle_callback,
            10,
        )

        self.drone = System()
        self.connected = False
        self.health_ready = False
        self.offboard_started = False
        self.search_task = None
        self.stop_search_event = None
        self.latest_north_m = 0.0
        self.latest_east_m = 0.0
        self.latest_down_m = 0.0
        self.latest_relative_altitude_m = 0.0
        self.latest_yaw_deg = 0.0
        self.obstacle_blocked = False
        self.last_obstacle_command_time = 0.0

        # MAVSDK는 asyncio 기반이므로 ROS executor와 별도 루프에서 실행한다.
        self.async_loop = asyncio.new_event_loop()
        self.async_thread = threading.Thread(
            target=self._run_async_loop,
            daemon=True,
        )
        self.async_thread.start()
        self._submit(self._initialize_mavsdk())

        self.get_logger().info("MAVSDK 드론 제어 노드 시작")

    def _run_async_loop(self):
        asyncio.set_event_loop(self.async_loop)
        self.async_loop.run_forever()

    def _submit(self, coroutine):
        return asyncio.run_coroutine_threadsafe(
            coroutine,
            self.async_loop,
        )

    async def _initialize_mavsdk(self):
        self.stop_search_event = asyncio.Event()
        system_address = str(
            self.get_parameter("system_address").value
        )
        self._publish_status("CONNECTING")
        await self.drone.connect(system_address=system_address)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.connected = True
                self._publish_status("CONNECTED")
                self.get_logger().info("PX4 연결 성공")
                break

        asyncio.create_task(self._position_telemetry_loop())
        asyncio.create_task(self._attitude_telemetry_loop())
        asyncio.create_task(self._relative_altitude_loop())

    async def _position_telemetry_loop(self):
        async for telemetry in self.drone.telemetry.position_velocity_ned():
            position = telemetry.position
            self.latest_north_m = float(position.north_m)
            self.latest_east_m = float(position.east_m)
            self.latest_down_m = float(position.down_m)
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
        command = message.data.strip().upper()
        self.get_logger().info(f"드론 명령 수신: {command}")

        if command == "TAKEOFF":
            self._submit(self._takeoff())
        elif command == "START_SEARCH":
            self._submit(self._start_search())
        elif command == "HOVER":
            self._submit(self._hover())
        elif command == "LAND":
            self._submit(self._land())
        else:
            self.get_logger().warning(f"알 수 없는 드론 명령: {command}")

    def _obstacle_callback(self, message):
        self.obstacle_blocked = bool(message.data)
        if not self.obstacle_blocked or self.search_task is None:
            return

        now = time.monotonic()
        if now - self.last_obstacle_command_time < 1.0:
            return
        self.last_obstacle_command_time = now
        self.get_logger().warning("전방 장애물 감지: Hover 명령")
        self._submit(self._hover())

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
            radius = float(
                self.get_parameter(
                    "altitude_acceptance_radius_m"
                ).value
            )
            tolerance = float(
                self.get_parameter("takeoff_tolerance_m").value
            )

            await self.drone.param.set_param_float(
                "NAV_MC_ALT_RAD",
                radius,
            )
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.arm()
            self._publish_status("ARMED")
            await self.drone.action.takeoff()
            self._publish_status("TAKING_OFF")

            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                if self.latest_relative_altitude_m >= altitude - tolerance:
                    self._publish_status("AIRBORNE")
                    return
                await asyncio.sleep(0.2)
            raise asyncio.TimeoutError("목표 이륙 고도 도달 시간 초과")
        except (
            ActionError,
            ParamError,
            asyncio.TimeoutError,
        ) as error:
            self.get_logger().error(f"이륙 실패: {error}")
            self._publish_status(f"ERROR_TAKEOFF_{type(error).__name__}")

    async def _start_search(self):
        if self.search_task and not self.search_task.done():
            self.get_logger().warning("이미 수색 경로를 실행 중입니다.")
            return

        self.stop_search_event.clear()
        self.search_task = asyncio.create_task(self._search_path())

    async def _search_path(self):
        try:
            altitude = float(
                self.get_parameter("takeoff_altitude_m").value
            )
            yaw_deg = float(
                self.get_parameter("search_yaw_deg").value
            )
            hold_seconds = float(
                self.get_parameter("waypoint_hold_seconds").value
            )
            acceptance_radius = float(
                self.get_parameter(
                    "waypoint_acceptance_radius_m"
                ).value
            )
            altitude_tolerance = float(
                self.get_parameter(
                    "waypoint_altitude_tolerance_m"
                ).value
            )
            waypoint_timeout = float(
                self.get_parameter("waypoint_timeout_sec").value
            )
            waypoints = self._parse_waypoints(
                str(self.get_parameter("search_waypoints").value)
            )

            initial = PositionNedYaw(
                self.latest_north_m,
                self.latest_east_m,
                -altitude,
                yaw_deg,
            )
            await self.drone.offboard.set_position_ned(initial)
            if not self.offboard_started:
                await self.drone.offboard.start()
                self.offboard_started = True

            self._publish_status("SEARCHING")
            for index, (north_m, east_m) in enumerate(waypoints, start=1):
                if self.stop_search_event.is_set():
                    return
                if self.obstacle_blocked:
                    await self._hover()
                    return

                self.get_logger().info(
                    f"수색 지점 {index}/{len(waypoints)}: "
                    f"N={north_m:.1f}, E={east_m:.1f}, Yaw={yaw_deg:.0f}"
                )
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(
                        north_m,
                        east_m,
                        -altitude,
                        yaw_deg,
                    )
                )

                deadline = time.monotonic() + waypoint_timeout
                while True:
                    if self.stop_search_event.is_set():
                        return
                    if self.obstacle_blocked:
                        await self._hover()
                        return

                    horizontal_error = math.hypot(
                        self.latest_north_m - north_m,
                        self.latest_east_m - east_m,
                    )
                    altitude_error = abs(
                        self.latest_down_m - (-altitude)
                    )
                    if (
                        horizontal_error <= acceptance_radius
                        and altitude_error <= altitude_tolerance
                    ):
                        self.get_logger().info(
                            f"수색 지점 {index}/{len(waypoints)} 도착: "
                            f"수평오차={horizontal_error:.2f}m, "
                            f"고도오차={altitude_error:.2f}m"
                        )
                        break

                    if time.monotonic() >= deadline:
                        self.get_logger().error(
                            f"수색 지점 {index}/{len(waypoints)} "
                            f"도착 시간 초과: 수평오차={horizontal_error:.2f}m, "
                            f"고도오차={altitude_error:.2f}m"
                        )
                        await self._hover()
                        self._publish_status("ERROR_WAYPOINT_TIMEOUT")
                        return

                    await asyncio.sleep(0.2)

                elapsed = 0.0
                while elapsed < hold_seconds:
                    if self.stop_search_event.is_set():
                        return
                    if self.obstacle_blocked:
                        await self._hover()
                        return
                    await asyncio.sleep(0.2)
                    elapsed += 0.2

            await self._hover()
            self.search_task = None
            self._publish_status("SEARCH_FINISHED_NO_VICTIM")
        except OffboardError as error:
            self.get_logger().error(f"Offboard 수색 실패: {error}")
            self._publish_status("ERROR_OFFBOARD")

    async def _hover(self):
        if self.stop_search_event is not None:
            self.stop_search_event.set()

        try:
            altitude = float(
                self.get_parameter("takeoff_altitude_m").value
            )
            # Hover는 현재 PX4 Yaw를 유지해 불필요한 초기 회전을 막는다.
            # search_yaw_deg는 실제 수색을 시작할 때만 적용한다.
            yaw_deg = self.latest_yaw_deg
            setpoint = PositionNedYaw(
                self.latest_north_m,
                self.latest_east_m,
                -altitude,
                yaw_deg,
            )
            await self.drone.offboard.set_position_ned(setpoint)
            if not self.offboard_started:
                await self.drone.offboard.start()
                self.offboard_started = True
            self._publish_status("HOVERING")
        except OffboardError as error:
            self.get_logger().error(f"Hover 실패: {error}")
            self._publish_status("ERROR_HOVER")

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

            await asyncio.wait_for(wait_until_landed(), timeout=30.0)
            self._publish_status("LANDED")
        except (
            ActionError,
            OffboardError,
            asyncio.TimeoutError,
        ) as error:
            self.get_logger().error(f"착륙 실패: {error}")
            self._publish_status("ERROR_LAND")

    @staticmethod
    def _parse_waypoints(value):
        waypoints = []
        for pair in value.split(";"):
            pair = pair.strip()
            if not pair:
                continue
            north_text, east_text = pair.split(",", maxsplit=1)
            waypoints.append((float(north_text), float(east_text)))
        if not waypoints:
            raise ValueError("수색 Waypoint가 비어 있습니다.")
        return waypoints

    def _publish_status(self, status):
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info(f"드론 상태: {status}")

    def _publish_local_position(self):
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "local_ned"
        message.point.x = self.latest_north_m
        message.point.y = self.latest_east_m
        message.point.z = self.latest_down_m
        self.position_publisher.publish(message)

    def _publish_map_to_base_tf(self):
        """PX4 NED 위치를 ROS ENU map 좌표로 변환해 발행한다."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(
            self.get_parameter("map_frame").value
        )
        transform.child_frame_id = str(
            self.get_parameter("base_frame").value
        )

        transform.transform.translation.x = self.latest_east_m
        transform.transform.translation.y = self.latest_north_m
        transform.transform.translation.z = -self.latest_down_m

        # NED heading을 ENU yaw로 변환한다. 기본 시스템은 roll/pitch를 생략한다.
        yaw_enu = math.radians(90.0 - self.latest_yaw_deg)
        transform.transform.rotation.z = math.sin(0.5 * yaw_enu)
        transform.transform.rotation.w = math.cos(0.5 * yaw_enu)
        self.transform_broadcaster.sendTransform(transform)

    def destroy_node(self):
        if self.stop_search_event is not None:
            self.async_loop.call_soon_threadsafe(
                self.stop_search_event.set
            )
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
