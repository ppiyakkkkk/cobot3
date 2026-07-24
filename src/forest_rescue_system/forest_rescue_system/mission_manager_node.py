#!/usr/bin/env python3

"""설정된 N대 드론의 수색·탐지·복귀 상태를 중앙에서 관리한다."""

from functools import partial
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from forest_rescue_interfaces.msg import VictimDetection
from forest_rescue_system.cooperative_search_planner import (
    CooperativeSearchPlanner,
)
from forest_rescue_system.log_utils import TimestampedNode


class MissionManagerNode(TimestampedNode):
    """탐지 드론은 Hover, 나머지 드론은 자동 복귀·착륙시킨다."""

    def __init__(self):
        super().__init__("mission_manager_node")

        self.declare_parameter(
            "drone_ids",
            ["quadrotor_01", "quadrotor_02", "quadrotor_03"],
        )
        self.declare_parameter("operation_mode", "rescue_search")
        self.declare_parameter(
            "search_plan_path",
            "~/b3_cobot3_ws/isaac_sim/generated_search_plan.json",
        )
        self.declare_parameter("require_search_plan_match", True)
        self.declare_parameter(
            "fallback_search_area_bounds_xy",
            [-46.7, 46.7, -46.7, 46.7],
        )
        self.declare_parameter("required_detection_frames", 3)
        self.declare_parameter("minimum_detection_confidence", 0.40)
        self.declare_parameter("detection_start_delay_sec", 60.0)
        self.declare_parameter("require_map_position_before_detection", True)
        self.declare_parameter("start_exclusion_radius_m", 15.0)
        self.declare_parameter("detection_position_consistency_radius_m", 4.0)
        self.declare_parameter("detection_position_timeout_sec", 1.0)
        # 완전히 연속된 프레임만 요구하지 않고, 짧은 시간창 안에서
        # 위치가 일치하는 양성 탐지를 누적한다.
        self.declare_parameter("detection_window_sec", 2.0)
        self.declare_parameter("maximum_missed_detections", 2)
        self.declare_parameter("victim_approach_height_m", 7.0)
        self.declare_parameter(
            "search_zone_bounds_xy",
            [
                -46.7, 46.7, 13.55, 46.7,
                -46.7, 46.7, -13.55, 13.55,
                -46.7, 46.7, -46.7, -13.55,
            ],
        )
        self.declare_parameter(
            "start_exclusion_centers_xy",
            [-34.0, 40.0, -29.0, 40.0, -39.0, 40.0],
        )
        self.declare_parameter("auto_takeoff_on_connect", True)

        # 특정 드론 담당 구역을 런타임에 다시 나누는 협동 수색 설정이다.
        self.declare_parameter(
            "terrain_mesh_path",
            "~/b3_cobot3_ws/isaac_sim/generated_terrain_mesh.npz",
        )
        self.declare_parameter(
            "navigation_surface_path",
            "~/b3_cobot3_ws/isaac_sim/generated_navigation_surface.npz",
        )
        self.declare_parameter("cooperative_lane_spacing_m", 4.0)
        self.declare_parameter("cooperative_sample_spacing_m", 4.0)
        self.declare_parameter(
            "cooperative_terrain_profile_spacing_m",
            1.0,
        )
        self.declare_parameter("cooperative_transit_altitude_step_m", 1.0)
        self.declare_parameter("cooperative_transit_profile_spacing_m", 3.0)
        self.declare_parameter("cooperative_max_climb_step_m", 2.5)
        self.declare_parameter("cooperative_max_descent_step_m", 2.0)
        self.declare_parameter("cooperative_transit_climb_only", True)
        self.declare_parameter(
            "cooperative_transit_skip_current_waypoint",
            True,
        )
        self.declare_parameter("cooperative_subzone_margin_m", 0.6)
        self.declare_parameter("cooperative_prepare_timeout_sec", 20.0)
        self.declare_parameter(
            "cooperative_marker_topic",
            "/mission/cooperative_search/markers",
        )

        self.operation_mode = str(
            self.get_parameter("operation_mode").value
        ).strip().lower()
        if self.operation_mode != "rescue_search":
            raise RuntimeError(
                "mission_manager는 operation_mode=rescue_search에서만 "
                f"실행할 수 있습니다: {self.operation_mode!r}"
            )

        self.drone_ids = [
            str(value) for value in self.get_parameter("drone_ids").value
        ]
        if not self.drone_ids:
            raise RuntimeError("drone_ids가 비어 있습니다.")
        self.search_zone_bounds = {}
        self.drone_home_world_enu = {}
        self.start_exclusion_centers = []
        self._load_search_plan_metadata(log_result=True)
        self.cooperative_planner = CooperativeSearchPlanner(
            search_plan_path=str(
                self.get_parameter("search_plan_path").value
            ),
            terrain_mesh_path=str(
                self.get_parameter("terrain_mesh_path").value
            ),
            navigation_surface_path=str(
                self.get_parameter("navigation_surface_path").value
            ),
            lane_spacing_m=float(
                self.get_parameter("cooperative_lane_spacing_m").value
            ),
            sample_spacing_m=float(
                self.get_parameter("cooperative_sample_spacing_m").value
            ),
            terrain_profile_spacing_m=float(
                self.get_parameter(
                    "cooperative_terrain_profile_spacing_m"
                ).value
            ),
            transit_altitude_step_m=float(
                self.get_parameter(
                    "cooperative_transit_altitude_step_m"
                ).value
            ),
            transit_profile_spacing_m=float(
                self.get_parameter(
                    "cooperative_transit_profile_spacing_m"
                ).value
            ),
            max_climb_step_m=float(
                self.get_parameter(
                    "cooperative_max_climb_step_m"
                ).value
            ),
            max_descent_step_m=float(
                self.get_parameter(
                    "cooperative_max_descent_step_m"
                ).value
            ),
            transit_climb_only=bool(
                self.get_parameter(
                    "cooperative_transit_climb_only"
                ).value
            ),
            transit_skip_current_waypoint=bool(
                self.get_parameter(
                    "cooperative_transit_skip_current_waypoint"
                ).value
            ),
            subzone_margin_m=float(
                self.get_parameter("cooperative_subzone_margin_m").value
            ),
        )
        self.command_publishers = {}
        self.cooperative_plan_publishers = {}
        self.drone_status = {drone_id: "UNKNOWN" for drone_id in self.drone_ids}
        self.detection_counts = {drone_id: 0 for drone_id in self.drone_ids}
        self.detection_miss_counts = {
            drone_id: 0 for drone_id in self.drone_ids
        }
        self.search_finished = set()
        self.failed_drones = set()
        self.latest_camera_positions = {}
        self.latest_map_positions = {}
        self.pending_detection_stamps = {
            drone_id: {} for drone_id in self.drone_ids
        }
        self.confirmed_map_sequences = {
            drone_id: [] for drone_id in self.drone_ids
        }
        self.recent_map_positions = {
            drone_id: {} for drone_id in self.drone_ids
        }
        self.search_started_at = None
        self.last_detection_delay_log_at = 0.0
        self.last_exclusion_log_at = {drone_id: 0.0 for drone_id in self.drone_ids}
        self.last_zone_log_at = {drone_id: 0.0 for drone_id in self.drone_ids}
        self.latest_drone_local_positions = {}
        self.cooperative_plan = None
        self.cooperative_active_drones = []
        self.cooperative_plan_acks = set()
        self.cooperative_finished_drones = set()
        self.cooperative_started_at = None
        self.cooperative_owner_drone = None
        self.cooperative_target_bounds = None

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cooperative_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("cooperative_marker_topic").value),
            marker_qos,
        )

        for index, drone_id in enumerate(self.drone_ids, start=1):
            prefix = f"/drone_{index:02d}"
            self.command_publishers[drone_id] = self.create_publisher(
                String,
                f"{prefix}/command",
                10,
            )
            self.cooperative_plan_publishers[drone_id] = self.create_publisher(
                String,
                f"{prefix}/mission/cooperative_plan",
                10,
            )
            self.create_subscription(
                String,
                f"{prefix}/status",
                partial(self._drone_status_callback, drone_id),
                10,
            )
            self.create_subscription(
                VictimDetection,
                f"{prefix}/victim/detection",
                partial(self._detection_callback, drone_id),
                10,
            )
            self.create_subscription(
                PointStamped,
                f"{prefix}/victim/position_camera",
                partial(self._camera_position_callback, drone_id),
                10,
            )
            self.create_subscription(
                PointStamped,
                f"{prefix}/victim/position_map",
                partial(self._map_position_callback, drone_id),
                10,
            )
            self.create_subscription(
                PointStamped,
                f"{prefix}/local_position_ned",
                partial(self._drone_position_callback, drone_id),
                10,
            )
            self.create_subscription(
                String,
                f"{prefix}/mission/cooperative_plan_ack",
                partial(self._cooperative_plan_ack_callback, drone_id),
                10,
            )

        self.state_publisher = self.create_publisher(
            String,
            "/mission/state",
            10,
        )
        self.mode_publisher = self.create_publisher(
            String,
            "/mission/mode",
            10,
        )
        self.finder_publisher = self.create_publisher(
            String,
            "/mission/finder_drone",
            10,
        )
        self.start_service = self.create_service(
            Trigger,
            "/mission/start",
            self._start_mission_callback,
        )
        self.land_service = self.create_service(
            Trigger,
            "/mission/land",
            self._land_callback,
        )
        self.cooperative_services = []
        for index, drone_id in enumerate(self.drone_ids, start=1):
            self.cooperative_services.append(
                self.create_service(
                    Trigger,
                    f"/mission/cooperative_search/drone_{index:02d}",
                    partial(self._cooperative_search_callback, drone_id),
                )
            )

        self.state = "IDLE"
        self.initial_takeoff_requested = False
        self.finder_drone = None
        self.finder_hovering = False
        self.camera_position_received = False
        self.map_position_received = False
        self._publish_mode()
        self._publish_state("IDLE")
        self.republish_timer = self.create_timer(
            1.0,
            self._republish_mode_and_state,
        )
        self.cooperative_timer = self.create_timer(
            0.2,
            self._advance_cooperative_search,
        )
        self.get_logger().info(
            "구조 수색 모드 시작: "
            f"operation_mode={self.operation_mode}, drones={self.drone_ids}"
        )

    def _start_mission_callback(self, _request, response):
        if self.state != "READY":
            response.success = False
            response.message = f"READY 상태에서만 시작할 수 있습니다: {self.state}"
            return response

        # 시뮬레이션 재실행으로 수색계획이 갱신됐을 수 있으므로 다시 읽는다.
        plan_ready = self._load_search_plan_metadata(log_result=True)
        if (
            bool(self.get_parameter("require_search_plan_match").value)
            and not plan_ready
        ):
            response.success = False
            response.message = (
                "현재 operation_mode와 drone_count에 일치하는 "
                "generated_search_plan.json이 없습니다. Isaac Sim을 같은 "
                "모드와 드론 수로 먼저 실행하세요."
            )
            self.get_logger().error(response.message)
            return response

        self._reset_detection_state()
        self.search_started_at = self._sim_time_sec()
        self.last_detection_delay_log_at = 0.0
        self._publish_state("SEARCHING")
        self._send_all("START_SEARCH")
        response.success = True
        response.message = (
            f"드론 {len(self.drone_ids)}대의 분할 구역 수색을 시작했습니다."
        )
        return response

    def _land_callback(self, _request, response):
        self._publish_state("LANDING")
        self._send_all("LAND")
        response.success = True
        response.message = "모든 드론에 착륙 명령을 전송했습니다."
        return response

    def _cooperative_search_callback(self, target_drone_id, _request, response):
        """선택한 드론의 기존 담당 구역을 활성 드론 수만큼 재분할한다."""
        if self.state != "SEARCHING":
            response.success = False
            response.message = (
                "기본 SEARCHING 상태에서만 협동 수색을 요청할 수 있습니다: "
                f"{self.state}"
            )
            return response
        if self.finder_drone is not None or self.cooperative_plan is not None:
            response.success = False
            response.message = "이미 탐지 또는 협동 수색 절차가 진행 중입니다."
            return response

        active_drones = [
            drone_id
            for drone_id in self.drone_ids
            if drone_id not in self.failed_drones
            and self.drone_status.get(drone_id) != "LANDED"
            and not self.drone_status.get(drone_id, "").startswith("ERROR")
        ]
        if target_drone_id not in active_drones:
            response.success = False
            response.message = (
                f"{target_drone_id}가 현재 협동 수색에 참여할 수 없습니다."
            )
            return response

        missing_positions = [
            drone_id
            for drone_id in active_drones
            if drone_id not in self.latest_drone_local_positions
            or drone_id not in self.drone_home_world_enu
        ]
        if missing_positions:
            response.success = False
            response.message = (
                "현재 위치 또는 홈 좌표를 아직 받지 못한 드론: "
                + ", ".join(missing_positions)
            )
            return response

        try:
            self.cooperative_planner.reload()
            world_positions = {
                drone_id: self._drone_world_position(drone_id)
                for drone_id in active_drones
            }
            plan = self.cooperative_planner.create_plan(
                target_drone_id=target_drone_id,
                active_drone_ids=active_drones,
                world_positions=world_positions,
            )
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            response.success = False
            response.message = f"협동 수색 계획 생성 실패: {error}"
            self.get_logger().error(response.message)
            return response

        self.cooperative_plan = plan
        self.cooperative_active_drones = list(active_drones)
        self.cooperative_plan_acks.clear()
        self.cooperative_finished_drones.clear()
        self.cooperative_started_at = time.monotonic()
        self.cooperative_owner_drone = target_drone_id
        self.cooperative_target_bounds = tuple(
            float(value) for value in plan["target_zone_bounds_xy"]
        )
        self._publish_state("COOP_SEARCH_PREPARING")

        # 각 드론은 중앙점으로 모이지 않는다. 계획을 먼저 보관한 뒤 Hover가
        # 확인되면 자기 소구역의 첫 웨이포인트로 직접 이동한다.
        for drone_id in active_drones:
            self._send_command(drone_id, "HOVER")
            assignment = dict(plan["assignments"][drone_id])
            assignment.update(
                {
                    "plan_id": plan["plan_id"],
                    "plan_type": plan["plan_type"],
                    "target_drone_id": target_drone_id,
                    "target_zone_bounds_xy": plan[
                        "target_zone_bounds_xy"
                    ],
                    "search_repeat_mode": "infinite",
                }
            )
            message = String()
            message.data = json.dumps(
                assignment,
                ensure_ascii=False,
                separators=(",", ":"),
            )
            self.cooperative_plan_publishers[drone_id].publish(message)

        self.cooperative_marker_publisher.publish(
            self._build_cooperative_markers(plan)
        )
        response.success = True
        response.message = (
            f"{target_drone_id} 담당 구역을 {len(active_drones)}개로 재분할했습니다. "
            f"plan_id={plan['plan_id']}"
        )
        self.get_logger().warning(response.message)
        return response

    def _drone_position_callback(self, drone_id, message):
        self.latest_drone_local_positions[drone_id] = message

    def _drone_world_position(self, drone_id):
        local = self.latest_drone_local_positions[drone_id].point
        home = self.drone_home_world_enu[drone_id]
        return [
            float(home[0]) + float(local.y),
            float(home[1]) + float(local.x),
            float(home[2]) - float(local.z),
        ]

    def _cooperative_plan_ack_callback(self, drone_id, message):
        if self.cooperative_plan is None:
            return
        if message.data.strip() != self.cooperative_plan["plan_id"]:
            return
        self.cooperative_plan_acks.add(drone_id)
        self.get_logger().info(
            "협동 계획 ACK: "
            f"{drone_id} ({len(self.cooperative_plan_acks)}/"
            f"{len(self.cooperative_active_drones)})"
        )

    def _advance_cooperative_search(self):
        if self.cooperative_plan is None:
            return
        active = [
            drone_id
            for drone_id in self.cooperative_active_drones
            if drone_id not in self.failed_drones
        ]
        if not active:
            self._abort_cooperative_search("협동 수색 가능한 드론이 없습니다.")
            return

        if self.state == "COOP_SEARCH_PREPARING":
            timeout = float(
                self.get_parameter("cooperative_prepare_timeout_sec").value
            )
            if (
                self.cooperative_started_at is not None
                and time.monotonic() - self.cooperative_started_at > timeout
            ):
                self._abort_cooperative_search(
                    "Hover 또는 협동 계획 ACK 준비시간을 초과했습니다."
                )
                return
            all_hovering = all(
                self.drone_status.get(drone_id) == "HOVERING"
                for drone_id in active
            )
            all_acked = all(
                drone_id in self.cooperative_plan_acks
                for drone_id in active
            )
            if all_hovering and all_acked:
                plan_id = self.cooperative_plan["plan_id"]
                self._publish_state("COOP_SEARCH_TRANSIT")
                for drone_id in active:
                    self._send_command(
                        drone_id,
                        f"START_COOPERATIVE_SEARCH:{plan_id}",
                    )
                self.get_logger().warning(
                    "협동 수색 진입 시작: 현재 절대고도보다 하강하지 "
                    "않고, 앞쪽 지형·장애물에 필요할 때만 상승하며 자기 "
                    "소구역 첫 웨이포인트로 이동합니다."
                )
            return

        if self.state in {"COOP_SEARCH_TRANSIT", "COOP_SEARCHING"}:
            resolved = all(
                drone_id in self.cooperative_finished_drones
                or drone_id in self.failed_drones
                for drone_id in self.cooperative_active_drones
            )
            if resolved:
                self._finish_cooperative_search()

    def _finish_cooperative_search(self):
        owner = self.cooperative_owner_drone
        active = list(self.cooperative_active_drones)
        self._publish_state("COOP_SEARCH_COMPLETE")

        # 대상 구역은 모든 참여 드론이 정방향+역순으로 다시 확인했으므로
        # 원래 담당 드론의 기본 구역은 완료 처리한다. 지원 드론은 중단 전
        # 기본 웨이포인트 인덱스부터 수색을 재개한다.
        if owner and owner not in self.failed_drones:
            self.search_finished.add(owner)
            self._send_command(owner, "MARK_PRIMARY_COMPLETE")
        for drone_id in active:
            if drone_id == owner or drone_id in self.failed_drones:
                continue
            self._send_command(drone_id, "RESUME_SEARCH")

        self._clear_cooperative_markers()
        self._clear_cooperative_context()
        self._publish_state("SEARCHING")
        self._try_finish_search_without_victim()

    def _abort_cooperative_search(self, reason):
        self.get_logger().error(f"협동 수색 취소: {reason}")
        for drone_id in self.cooperative_active_drones:
            if drone_id not in self.failed_drones:
                self._send_command(drone_id, "RESUME_SEARCH")
        self._clear_cooperative_markers()
        self._clear_cooperative_context()
        self._publish_state("SEARCHING")

    def _clear_cooperative_context(self):
        self.cooperative_plan = None
        self.cooperative_active_drones = []
        self.cooperative_plan_acks.clear()
        self.cooperative_finished_drones.clear()
        self.cooperative_started_at = None
        self.cooperative_owner_drone = None
        self.cooperative_target_bounds = None

    def _clear_cooperative_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.cooperative_marker_publisher.publish(marker_array)

    @staticmethod
    def _marker_point(x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _build_cooperative_markers(self, plan):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        colors = (
            (1.0, 0.20, 0.20),
            (0.20, 1.0, 0.20),
            (0.20, 0.45, 1.0),
            (1.0, 0.75, 0.10),
        )
        marker_id = 0

        target = plan["target_zone_bounds_xy"]
        target_z = max(
            item["entry_world_enu"][2]
            for item in plan["assignments"].values()
        ) + 1.0
        outline = Marker()
        outline.header.frame_id = "map"
        outline.header.stamp = stamp
        outline.ns = "cooperative_target"
        outline.id = marker_id
        marker_id += 1
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.45
        outline.color.r = 1.0
        outline.color.g = 1.0
        outline.color.b = 1.0
        outline.color.a = 1.0
        x_min, x_max, y_min, y_max = target
        outline.points = [
            self._marker_point(x_min, y_min, target_z),
            self._marker_point(x_max, y_min, target_z),
            self._marker_point(x_max, y_max, target_z),
            self._marker_point(x_min, y_max, target_z),
            self._marker_point(x_min, y_min, target_z),
        ]
        marker_array.markers.append(outline)

        for drone_index, drone_id in enumerate(plan["active_drone_ids"]):
            assignment = plan["assignments"][drone_id]
            bounds = assignment["subzone_bounds_xy"]
            entry = assignment["entry_world_enu"]
            red, green, blue = colors[drone_index % len(colors)]
            x_min, x_max, y_min, y_max = bounds

            zone = Marker()
            zone.header.frame_id = "map"
            zone.header.stamp = stamp
            zone.ns = "cooperative_subzones"
            zone.id = marker_id
            marker_id += 1
            zone.type = Marker.CUBE
            zone.action = Marker.ADD
            zone.pose.orientation.w = 1.0
            zone.pose.position.x = (x_min + x_max) * 0.5
            zone.pose.position.y = (y_min + y_max) * 0.5
            zone.pose.position.z = target_z - 0.5
            zone.scale.x = max(0.1, x_max - x_min)
            zone.scale.y = max(0.1, y_max - y_min)
            zone.scale.z = 0.18
            zone.color.r = red
            zone.color.g = green
            zone.color.b = blue
            zone.color.a = 0.28
            marker_array.markers.append(zone)

            entry_marker = Marker()
            entry_marker.header.frame_id = "map"
            entry_marker.header.stamp = stamp
            entry_marker.ns = "cooperative_entries"
            entry_marker.id = marker_id
            marker_id += 1
            entry_marker.type = Marker.SPHERE
            entry_marker.action = Marker.ADD
            entry_marker.pose.orientation.w = 1.0
            entry_marker.pose.position.x = float(entry[0])
            entry_marker.pose.position.y = float(entry[1])
            entry_marker.pose.position.z = float(entry[2])
            entry_marker.scale.x = 1.2
            entry_marker.scale.y = 1.2
            entry_marker.scale.z = 1.2
            entry_marker.color.r = red
            entry_marker.color.g = green
            entry_marker.color.b = blue
            entry_marker.color.a = 1.0
            marker_array.markers.append(entry_marker)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = stamp
            label.ns = "cooperative_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.orientation.w = 1.0
            label.pose.position.x = (x_min + x_max) * 0.5
            label.pose.position.y = (y_min + y_max) * 0.5
            label.pose.position.z = target_z + 1.2
            label.scale.z = 1.5
            label.color.r = red
            label.color.g = green
            label.color.b = blue
            label.color.a = 1.0
            label.text = f"{drone_id}\nENTRY"
            marker_array.markers.append(label)
        return marker_array

    def _reset_detection_state(self):
        self.finder_drone = None
        self.finder_hovering = False
        self.camera_position_received = False
        self.map_position_received = False
        self.search_finished.clear()
        self.failed_drones.clear()
        self.latest_camera_positions.clear()
        self.latest_map_positions.clear()
        for drone_id in self.drone_ids:
            self.detection_counts[drone_id] = 0
            self.detection_miss_counts[drone_id] = 0
            self.pending_detection_stamps[drone_id].clear()
            self.confirmed_map_sequences[drone_id].clear()
            self.recent_map_positions[drone_id].clear()

    def _drone_status_callback(self, drone_id, message):
        status = message.data.strip().upper()
        previous = self.drone_status[drone_id]
        self.drone_status[drone_id] = status
        if status != previous:
            self.get_logger().info(f"{drone_id} 상태: {status}")

        if (
            bool(self.get_parameter("auto_takeoff_on_connect").value)
            and not self.initial_takeoff_requested
            and all(
                self.drone_status[item] == "CONNECTED"
                for item in self.drone_ids
            )
        ):
            self.initial_takeoff_requested = True
            self._publish_state("INITIAL_TAKEOFF")
            self._send_all("TAKEOFF")
            return

        airborne_states = {
            "AIRBORNE",
            "HOVERING",
            "SEARCHING",
            "AVOIDING_OBSTACLE",
            "AVOIDING_OBSTACLE_XY",
        }
        if self.state == "INITIAL_TAKEOFF" and all(
            self.drone_status[item] in airborne_states
            for item in self.drone_ids
        ):
            self._publish_state("INITIAL_HOVER")
            self._send_all("HOVER")
            return

        if self.state == "INITIAL_HOVER" and all(
            self.drone_status[item] == "HOVERING"
            for item in self.drone_ids
        ):
            self._publish_state("READY")
            self.get_logger().info(
                f"드론 {len(self.drone_ids)}대 초기 이륙 완료: "
                "/mission/start 대기"
            )
            return

        if status.startswith("COOP_SEARCHING"):
            if self.state == "COOP_SEARCH_TRANSIT":
                self._publish_state("COOP_SEARCHING")

        if status == "COOP_SEARCH_FINISHED":
            self.cooperative_finished_drones.add(drone_id)
            self._advance_cooperative_search()
            return

        if status == "SEARCH_FINISHED_NO_VICTIM":
            self.search_finished.add(drone_id)
            self._try_finish_search_without_victim()
            return

        if self.finder_drone is not None:
            if drone_id == self.finder_drone and status == "HOVERING":
                self.finder_hovering = True
            self._try_complete_mission()
        elif self.state == "RETURNING_NO_VICTIM" and all(
            self.drone_status[item] == "LANDED" for item in self.drone_ids
        ):
            self._publish_state("SEARCH_COMPLETE_NOT_FOUND")

        if status.startswith("ERROR"):
            if drone_id not in self.failed_drones:
                self.failed_drones.add(drone_id)

                # 착륙 실패 직후 지면 근처 드론에 HOVER를 보내면
                # 다시 들썩이거나 Offboard가 재시작될 수 있으므로 금지한다.
                if status == "ERROR_LAND":
                    self.get_logger().error(
                        f"{drone_id} 착륙 오류: HOVER 재명령 없이 격리"
                    )
                else:
                    self._send_command(drone_id, "HOVER")
                    self.get_logger().error(
                        f"{drone_id} 오류 격리: 해당 드론만 Hover, "
                        "나머지 드론은 계속 동작"
                    )

            active_drones = [
                item for item in self.drone_ids
                if item not in self.failed_drones
            ]
            if not active_drones:
                self._publish_state("MISSION_FAILED")
            else:
                self._try_finish_search_without_victim()
                self._try_complete_mission()
                self._advance_cooperative_search()

    def _detection_callback(self, drone_id, detection):
        if self.state not in {
            "SEARCHING",
            "COOP_SEARCH_TRANSIT",
            "COOP_SEARCHING",
        } or self.finder_drone is not None:
            return
        if drone_id in self.failed_drones:
            return

        delay_sec = float(
            self.get_parameter("detection_start_delay_sec").value
        )
        elapsed = (
            self._sim_time_sec() - self.search_started_at
            if self.search_started_at is not None
            else 0.0
        )
        if elapsed < delay_sec:
            # 탐지 노드 설정이 잘못되거나 이전 메시지가 남아 있어도
            # 중앙 관리자에서 한 번 더 초기 구조자 오탐을 차단한다.
            now = time.monotonic()
            if detection.detected and now - self.last_detection_delay_log_at >= 5.0:
                self.get_logger().warning(
                    f"{drone_id} 초기 탐지 무시: "
                    f"유효화까지 {delay_sec - elapsed:.1f}초"
                )
                self.last_detection_delay_log_at = now
            self._reset_drone_detection_sequence(drone_id)
            return

        if not detection.detected:
            self._register_detection_miss(
                drone_id,
                self._stamp_to_seconds(detection.header.stamp),
            )
            return

        minimum_confidence = float(
            self.get_parameter("minimum_detection_confidence").value
        )
        if float(detection.confidence) < minimum_confidence:
            self._register_detection_miss(
                drone_id,
                self._stamp_to_seconds(detection.header.stamp),
            )
            return

        # Detection callback과 Localizer callback은 서로 다른 구독이다.
        # 예전처럼 latest_map_positions를 즉시 읽으면 이전 bbox의 위치를
        # 현재 bbox에 잘못 연결할 수 있다. 같은 Header stamp의 map 위치가
        # 도착할 때까지 이번 양성 탐지를 보류한다.
        stamp_ns = self._stamp_to_nanoseconds(detection.header.stamp)
        now = time.monotonic()
        self._prune_detection_window(
            drone_id,
            self._stamp_to_seconds(detection.header.stamp),
        )
        self.detection_miss_counts[drone_id] = 0
        pending = self.pending_detection_stamps[drone_id]
        pending[stamp_ns] = (
            now,
            float(detection.confidence),
            (
                int(detection.x_min),
                int(detection.y_min),
                int(detection.x_max),
                int(detection.y_max),
            ),
        )
        timeout = float(
            self.get_parameter("detection_position_timeout_sec").value
        )
        for old_stamp, record in list(pending.items()):
            inserted_at = record[0]
            if now - inserted_at > timeout:
                pending.pop(old_stamp, None)
        # 프로세스 스케줄링에 따라 Localizer의 map 메시지가 Detection보다
        # 먼저 도착할 수도 있으므로 짧게 보관한 동일 stamp 결과도 확인한다.
        cached_map = self.recent_map_positions[drone_id].get(stamp_ns)
        if cached_map is not None:
            self._validate_synchronized_detection(drone_id, cached_map[0])

    def _camera_position_callback(self, drone_id, message):
        self.latest_camera_positions[drone_id] = message
        if drone_id != self.finder_drone:
            return
        if not self.camera_position_received:
            self.get_logger().info(
                f"조난자 카메라 위치: ({message.point.x:.2f}, "
                f"{message.point.y:.2f}, {message.point.z:.2f})"
            )
        self.camera_position_received = True
        self._try_complete_mission()

    def _map_position_callback(self, drone_id, message):
        self.latest_map_positions[drone_id] = message
        stamp_ns = self._stamp_to_nanoseconds(message.header.stamp)
        now = time.monotonic()
        recent = self.recent_map_positions[drone_id]
        recent[stamp_ns] = (message, now)
        timeout = float(
            self.get_parameter("detection_position_timeout_sec").value
        )
        for old_stamp, (_old_message, inserted_at) in list(recent.items()):
            if now - inserted_at > timeout:
                recent.pop(old_stamp, None)
        self._validate_synchronized_detection(drone_id, message)
        if drone_id != self.finder_drone:
            return
        if not self.map_position_received:
            self.get_logger().info(
                f"조난자 map 위치: ({message.point.x:.2f}, "
                f"{message.point.y:.2f}, {message.point.z:.2f})"
            )
        self.map_position_received = True
        self._try_complete_mission()

    def _validate_synchronized_detection(self, drone_id, message):
        """동일 영상 stamp의 위치가 연속해서 일치할 때만 확정한다."""
        if self.state not in {
            "SEARCHING",
            "COOP_SEARCH_TRANSIT",
            "COOP_SEARCHING",
        } or self.finder_drone is not None:
            return
        if drone_id in self.failed_drones:
            return

        stamp_ns = self._stamp_to_nanoseconds(message.header.stamp)
        pending = self.pending_detection_stamps[drone_id]
        detection_record = pending.pop(stamp_ns, None)
        if detection_record is None:
            return
        _inserted_at, confidence, bbox = detection_record

        x = float(message.point.x)
        y = float(message.point.y)
        z = float(message.point.z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            self._reset_drone_detection_sequence(drone_id)
            return
        if self._is_in_start_exclusion_zone(x, y):
            self._reset_drone_detection_sequence(drone_id)
            now = time.monotonic()
            if now - self.last_exclusion_log_at[drone_id] >= 5.0:
                self.get_logger().warning(
                    f"{drone_id} 시작 구역 person 무시: map=({x:.2f}, {y:.2f})"
                )
                self.last_exclusion_log_at[drone_id] = now
            return
        if not self._is_in_assigned_search_zone(drone_id, x, y):
            self._reset_drone_detection_sequence(drone_id)
            now = time.monotonic()
            if now - self.last_zone_log_at[drone_id] >= 5.0:
                self.get_logger().warning(
                    f"{drone_id} 담당 구역 밖 person 무시: "
                    f"map=({x:.2f}, {y:.2f}), conf={confidence:.2f}"
                )
                self.last_zone_log_at[drone_id] = now
            return

        measurement_time = self._stamp_to_seconds(message.header.stamp)
        self._prune_detection_window(drone_id, measurement_time)
        sequence = self.confirmed_map_sequences[drone_id]
        consistency_radius = float(
            self.get_parameter(
                "detection_position_consistency_radius_m"
            ).value
        )
        if sequence:
            center_x = sum(record[1] for record in sequence) / len(sequence)
            center_y = sum(record[2] for record in sequence) / len(sequence)
            center_z = sum(record[3] for record in sequence) / len(sequence)
            position_error = math.sqrt(
                (x - center_x) ** 2
                + (y - center_y) ** 2
                + (z - center_z) ** 2
            )
            if position_error > consistency_radius:
                self.get_logger().warning(
                    f"{drone_id} 탐지 위치 불일치: {position_error:.2f}m "
                    "→ 시간창 탐지 누적 재시작"
                )
                sequence.clear()

        # (RGB 촬영시각, x, y, z) 형태로 저장한다. 처리 부하나 YOLO
        # 추론 지연이 아니라 실제 영상 간 시각 차이로 2초 창을 판정한다.
        sequence.append((measurement_time, x, y, z))
        self.detection_miss_counts[drone_id] = 0
        required = int(
            self.get_parameter("required_detection_frames").value
        )
        if len(sequence) > required:
            del sequence[:-required]
        self.detection_counts[drone_id] = len(sequence)
        window_sec = float(
            self.get_parameter("detection_window_sec").value
        )
        self.get_logger().info(
            f"{drone_id} 시간창 위치 일치 조난자 탐지: "
            f"{len(sequence)}/{required}, conf={confidence:.2f}, "
            f"window={window_sec:.1f}s, bbox={bbox}, "
            f"map=({x:.2f}, {y:.2f}, {z:.2f})"
        )
        if len(sequence) < required:
            return

        victim_x = sum(record[1] for record in sequence) / len(sequence)
        victim_y = sum(record[2] for record in sequence) / len(sequence)
        victim_z = sum(record[3] for record in sequence) / len(sequence)
        approach_z = victim_z + float(
            self.get_parameter("victim_approach_height_m").value
        )

        if self.cooperative_plan is not None:
            self._clear_cooperative_markers()
            self._clear_cooperative_context()
        self.finder_drone = drone_id
        self.camera_position_received = drone_id in self.latest_camera_positions
        self.map_position_received = True
        self._publish_finder(drone_id)
        self._publish_state("VICTIM_DETECTED")
        self._send_command(
            drone_id,
            f"APPROACH_VICTIM:{victim_x:.3f},{victim_y:.3f},{approach_z:.3f}",
        )
        for other_id in self.drone_ids:
            if other_id != drone_id and other_id not in self.failed_drones:
                self._send_command(other_id, "RETURN_HOME")
        self.get_logger().warning(
            f"조난자 확정 map=({victim_x:.2f}, {victim_y:.2f}, "
            f"{victim_z:.2f}); {drone_id}는 상공 {approach_z:.2f}m로 접근, "
            "나머지 드론은 자동 복귀"
        )

    def _prune_detection_window(self, drone_id, now=None):
        """설정된 시간창 밖의 양성 탐지를 누적 목록에서 제거한다."""
        if now is None:
            now = self._sim_time_sec()
        window_sec = max(0.1, float(
            self.get_parameter("detection_window_sec").value
        ))
        sequence = self.confirmed_map_sequences[drone_id]
        if any(record[0] > now for record in sequence):
            # 시뮬레이션 시간이 되감긴 경우 이전 실행의 탐지 기록을 폐기한다.
            sequence.clear()
        sequence[:] = [
            record for record in sequence
            if now - record[0] <= window_sec
        ]
        self.detection_counts[drone_id] = len(sequence)
        if not sequence:
            self.detection_miss_counts[drone_id] = 0

    def _register_detection_miss(self, drone_id, measurement_time=None):
        """중간 미탐은 허용하되, 너무 많이 연속되면 누적을 해제한다."""
        self._prune_detection_window(drone_id, measurement_time)
        sequence = self.confirmed_map_sequences[drone_id]
        if not sequence:
            return

        self.detection_miss_counts[drone_id] += 1
        maximum_misses = max(0, int(
            self.get_parameter("maximum_missed_detections").value
        ))
        if self.detection_miss_counts[drone_id] <= maximum_misses:
            return

        self.get_logger().info(
            f"{drone_id} 탐지 미확인 {self.detection_miss_counts[drone_id]}회 "
            f"> 허용 {maximum_misses}회: 시간창 누적 초기화"
        )
        self._reset_drone_detection_sequence(drone_id)

    def _reset_drone_detection_sequence(self, drone_id):
        self.detection_counts[drone_id] = 0
        self.detection_miss_counts[drone_id] = 0
        self.pending_detection_stamps[drone_id].clear()
        self.confirmed_map_sequences[drone_id].clear()
        self.recent_map_positions[drone_id].clear()

    @staticmethod
    def _stamp_to_nanoseconds(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1.0e9

    def _sim_time_sec(self):
        """use_sim_time 적용 시 /clock 기준 현재 시각을 초로 반환한다."""
        return self.get_clock().now().nanoseconds / 1.0e9

    def _try_complete_mission(self):
        if self.finder_drone is None or self.state == "COMPLETE":
            return
        return_drone_ids = [
            item
            for item in self.drone_ids
            if item != self.finder_drone
        ]
        returners_resolved = all(
            item in self.failed_drones
            or self.drone_status[item] == "LANDED"
            for item in return_drone_ids
        )
        if (
            self.finder_hovering
            and self.camera_position_received
            and self.map_position_received
            and returners_resolved
        ):
            landing_failures = [
                item
                for item in return_drone_ids
                if self.drone_status[item] != "LANDED"
            ]
            if landing_failures:
                self._publish_state("COMPLETE_WITH_LANDING_ERROR")
                self.get_logger().error(
                    "조난자 위치 Hover는 완료했지만 홈 착륙 실패: "
                    + ", ".join(landing_failures)
                )
            else:
                self._publish_state("COMPLETE")
                self.get_logger().info(
                    f"임무 완료: {self.finder_drone}는 조난자 위치 Hover, "
                    "나머지 드론은 홈 착륙 완료"
                )

    def _load_search_plan_metadata(self, log_result=True):
        """생성 JSON이 현재 launch의 함대 구성과 정확히 일치하는지 확인한다."""
        plan_path = Path(
            str(self.get_parameter("search_plan_path").value)
        ).expanduser()
        try:
            plan = json.loads(plan_path.read_text(encoding="utf-8"))
            drone_plans = plan["drones"]
            if not isinstance(drone_plans, dict):
                raise TypeError("drones는 객체여야 합니다")

            plan_mode = str(
                plan.get("operation_mode", "rescue_search")
            ).strip().lower()
            if plan_mode != self.operation_mode:
                raise ValueError(
                    "Isaac/ROS operation_mode이 다릅니다: "
                    f"plan={plan_mode}, launch={self.operation_mode}"
                )

            plan_ids = [
                str(value)
                for value in plan.get("drone_ids", drone_plans.keys())
            ]
            plan_count = int(plan.get("drone_count", len(plan_ids)))
            if plan_count != len(plan_ids):
                raise ValueError(
                    "drone_count와 drone_ids 길이가 다릅니다: "
                    f"count={plan_count}, ids={plan_ids}"
                )
            if set(drone_plans) != set(plan_ids):
                raise ValueError(
                    "drone_ids와 drones 키가 다릅니다: "
                    f"ids={plan_ids}, keys={list(drone_plans)}"
                )
            if plan_ids != self.drone_ids:
                raise ValueError(
                    "Isaac/ROS 함대 구성이 다릅니다: "
                    f"plan={plan_ids}, launch={self.drone_ids}"
                )
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
        ) as error:
            self.search_zone_bounds = {}
            self.drone_home_world_enu = {}
            self.start_exclusion_centers = []
            if log_result:
                self.get_logger().warning(
                    "현재 함대와 일치하는 수색 계획을 읽지 못했습니다: "
                    f"{plan_path}: {error}"
                )
            return False

        loaded_zones = {}
        loaded_homes = {}
        loaded_centers = []
        terrain_bounds = plan.get("terrain_bounds", {})
        terrain_x_min = terrain_bounds.get("x_min")
        terrain_x_max = terrain_bounds.get("x_max")

        for drone_id in self.drone_ids:
            drone_plan = drone_plans.get(drone_id)
            if not isinstance(drone_plan, dict):
                if log_result:
                    self.get_logger().warning(
                        f"수색 계획에 {drone_id} 항목이 없습니다: {plan_path}"
                    )
                return False

            zone = drone_plan.get("zone_bounds_xy")
            if not isinstance(zone, list) or len(zone) != 4:
                y_min = drone_plan.get("zone_y_min")
                y_max = drone_plan.get("zone_y_max")
                if None not in (terrain_x_min, terrain_x_max, y_min, y_max):
                    zone = [terrain_x_min, terrain_x_max, y_min, y_max]
            if not isinstance(zone, list) or len(zone) != 4:
                if log_result:
                    self.get_logger().warning(
                        f"{drone_id} 담당 구역 형식이 잘못됐습니다: {zone}"
                    )
                return False
            loaded_zones[drone_id] = tuple(float(value) for value in zone)

            home = drone_plan.get("home_world_enu")
            if not isinstance(home, list) or len(home) < 2:
                if log_result:
                    self.get_logger().warning(
                        f"{drone_id} 홈 좌표 형식이 잘못됐습니다: {home}"
                    )
                return False
            loaded_home = [float(value) for value in home[:3]]
            loaded_homes[drone_id] = loaded_home
            loaded_centers.extend([loaded_home[0], loaded_home[1]])

        self.search_zone_bounds = loaded_zones
        self.drone_home_world_enu = loaded_homes
        self.start_exclusion_centers = loaded_centers
        if log_result:
            self.get_logger().info(
                "동적 수색 계획 검증 완료: "
                f"drones={len(self.drone_ids)}, ids={self.drone_ids}, "
                f"path={plan_path}"
            )
        return True

    def _is_in_start_exclusion_zone(self, x, y):
        values = self.start_exclusion_centers
        if not values:
            fallback_values = [
                float(value)
                for value in self.get_parameter(
                    "start_exclusion_centers_xy"
                ).value
            ]
            # 존재하지 않는 드론의 시작점까지 오탐 제외 영역으로 쓰지 않는다.
            values = fallback_values[: 2 * len(self.drone_ids)]
        radius = float(
            self.get_parameter("start_exclusion_radius_m").value
        )
        for index in range(0, len(values) - 1, 2):
            if math.hypot(x - values[index], y - values[index + 1]) <= radius:
                return True
        return False

    def _is_in_assigned_search_zone(self, drone_id, x, y):
        if (
            self.cooperative_target_bounds is not None
            and self.state in {
                "COOP_SEARCH_PREPARING",
                "COOP_SEARCH_TRANSIT",
                "COOP_SEARCHING",
            }
        ):
            x_min, x_max, y_min, y_max = self.cooperative_target_bounds
            return x_min <= x <= x_max and y_min <= y <= y_max

        zone = self.search_zone_bounds.get(drone_id)
        if zone is None:
            # Isaac Sim이 JSON을 생성한 직후 ROS가 시작되는 경우를 고려해 재시도한다.
            self._load_search_plan_metadata(log_result=False)
            zone = self.search_zone_bounds.get(drone_id)

        if zone is None:
            # JSON을 필수로 하지 않는 호환 모드에서는 현재 N대 수에 맞춰
            # 전체 영역을 Y축으로 균등 분할한다. 작은 ID가 상단을 맡는다.
            bounds = [
                float(value)
                for value in self.get_parameter(
                    "fallback_search_area_bounds_xy"
                ).value
            ]
            if len(bounds) != 4:
                self.get_logger().error(
                    "fallback_search_area_bounds_xy는 4개 값이어야 합니다."
                )
                return False
            try:
                drone_index = self.drone_ids.index(drone_id)
            except ValueError:
                return False

            x_min, x_max, y_min, y_max = bounds
            if x_min >= x_max or y_min >= y_max:
                return False
            zone_height = (y_max - y_min) / len(self.drone_ids)
            reverse_index = len(self.drone_ids) - 1 - drone_index
            zone = (
                x_min,
                x_max,
                y_min + reverse_index * zone_height,
                y_min + (reverse_index + 1) * zone_height,
            )

        x_min, x_max, y_min, y_max = zone
        return x_min <= x <= x_max and y_min <= y <= y_max

    def _try_finish_search_without_victim(self):
        """오류 드론을 제외한 정상 드론이 모두 수색을 마쳤는지 확인한다."""
        if self.finder_drone is not None or self.state != "SEARCHING":
            return
        active_drones = [
            item for item in self.drone_ids
            if item not in self.failed_drones
        ]
        if not active_drones:
            return
        if not all(item in self.search_finished for item in active_drones):
            return
        self._publish_state("RETURNING_NO_VICTIM")
        for item in active_drones:
            self._send_command(item, "RETURN_HOME")

    def _send_command(self, drone_id, command):
        message = String()
        message.data = command
        self.command_publishers[drone_id].publish(message)
        self.get_logger().info(f"{drone_id} 명령: {command}")

    def _send_all(self, command):
        for drone_id in self.drone_ids:
            self._send_command(drone_id, command)

    def _publish_finder(self, drone_id):
        message = String()
        message.data = drone_id
        self.finder_publisher.publish(message)

    def _publish_mode(self):
        message = String()
        message.data = self.operation_mode
        self.mode_publisher.publish(message)

    def _publish_state(self, state):
        if state == self.state and state != "IDLE":
            return
        self.state = state
        message = String()
        message.data = state
        self.state_publisher.publish(message)
        self.get_logger().info(f"임무 상태: {state}")

    def _republish_mode_and_state(self):
        self._publish_mode()
        message = String()
        message.data = self.state
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
