#!/usr/bin/env python3

"""세 드론의 수색·탐지·복귀 상태를 중앙에서 관리한다."""

from functools import partial
import math
import time

from geometry_msgs.msg import PointStamped
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from forest_rescue_interfaces.msg import VictimDetection
from forest_rescue_system.log_utils import TimestampedNode


class MissionManagerNode(TimestampedNode):
    """탐지 드론은 Hover, 나머지 드론은 자동 복귀·착륙시킨다."""

    def __init__(self):
        super().__init__("mission_manager_node")

        self.declare_parameter(
            "drone_ids",
            ["quadrotor_01", "quadrotor_02", "quadrotor_03"],
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

        self.drone_ids = [
            str(value) for value in self.get_parameter("drone_ids").value
        ]
        self.command_publishers = {}
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

        for index, drone_id in enumerate(self.drone_ids, start=1):
            prefix = f"/drone_{index:02d}"
            self.command_publishers[drone_id] = self.create_publisher(
                String,
                f"{prefix}/command",
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

        self.state_publisher = self.create_publisher(
            String,
            "/mission/state",
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

        self.state = "IDLE"
        self.initial_takeoff_requested = False
        self.finder_drone = None
        self.finder_hovering = False
        self.camera_position_received = False
        self.map_position_received = False
        self._publish_state("IDLE")

    def _start_mission_callback(self, _request, response):
        if self.state != "READY":
            response.success = False
            response.message = f"READY 상태에서만 시작할 수 있습니다: {self.state}"
            return response

        self._reset_detection_state()
        self.search_started_at = self._sim_time_sec()
        self.last_detection_delay_log_at = 0.0
        self._publish_state("SEARCHING")
        self._send_all("START_SEARCH")
        response.success = True
        response.message = "드론 3대의 분할 구역 수색을 시작했습니다."
        return response

    def _land_callback(self, _request, response):
        self._publish_state("LANDING")
        self._send_all("LAND")
        response.success = True
        response.message = "모든 드론에 착륙 명령을 전송했습니다."
        return response

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
                "드론 3대 초기 이륙 완료: /mission/start 대기"
            )
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

    def _detection_callback(self, drone_id, detection):
        if self.state != "SEARCHING" or self.finder_drone is not None:
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
        if self.state != "SEARCHING" or self.finder_drone is not None:
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

    def _is_in_start_exclusion_zone(self, x, y):
        values = [
            float(value)
            for value in self.get_parameter(
                "start_exclusion_centers_xy"
            ).value
        ]
        radius = float(
            self.get_parameter("start_exclusion_radius_m").value
        )
        for index in range(0, len(values) - 1, 2):
            if math.hypot(x - values[index], y - values[index + 1]) <= radius:
                return True
        return False

    def _is_in_assigned_search_zone(self, drone_id, x, y):
        values = [
            float(value)
            for value in self.get_parameter("search_zone_bounds_xy").value
        ]
        try:
            drone_index = self.drone_ids.index(drone_id)
        except ValueError:
            return False
        offset = drone_index * 4
        if offset + 3 >= len(values):
            self.get_logger().error(
                "search_zone_bounds_xy 설정이 드론 수보다 짧습니다."
            )
            return False
        x_min, x_max, y_min, y_max = values[offset:offset + 4]
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

    def _publish_state(self, state):
        if state == self.state and state != "IDLE":
            return
        self.state = state
        message = String()
        message.data = state
        self.state_publisher.publish(message)
        self.get_logger().info(f"임무 상태: {state}")


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
