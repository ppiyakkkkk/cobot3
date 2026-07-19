#!/usr/bin/env python3

"""탐색 임무의 상태 전환과 드론 명령을 관리한다."""

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from forest_rescue_interfaces.msg import VictimDetection


class MissionManagerNode(Node):
    """초기 자동 이륙·Hover와 탐색 임무 상태를 관리한다."""

    def __init__(self):
        super().__init__("mission_manager_node")

        self.declare_parameter("required_detection_frames", 3)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("auto_takeoff_on_connect", True)
        self.declare_parameter("keep_hover_after_detection", True)

        self.command_publisher = self.create_publisher(
            String,
            "/drone/command",
            10,
        )
        self.state_publisher = self.create_publisher(
            String,
            "/mission/state",
            10,
        )

        self.create_subscription(
            VictimDetection,
            "/victim/detection",
            self._detection_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/victim/position_camera",
            self._camera_position_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/victim/position_map",
            self._map_position_callback,
            10,
        )
        self.create_subscription(
            String,
            "/drone/status",
            self._drone_status_callback,
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
        self.detection_count = 0
        self.victim_confirmed = False
        self.hovering = False
        self.camera_position_received = False
        self.map_position_received = False
        self.initial_takeoff_requested = False
        self._publish_state("IDLE")

        self.auto_start_timer = None
        if bool(self.get_parameter("auto_start").value):
            self.auto_start_timer = self.create_timer(
                2.0,
                self._auto_start_once,
            )

    def _auto_start_once(self):
        # auto_takeoff_on_connect가 켜져 있으면 READY 이후 사용자가
        # /mission/start를 호출하는 흐름을 우선한다.
        if (
            self.state == "IDLE"
            and not bool(
                self.get_parameter("auto_takeoff_on_connect").value
            )
        ):
            self._begin_mission()
        if self.auto_start_timer is not None:
            self.auto_start_timer.cancel()

    def _start_mission_callback(self, _request, response):
        if self.state not in (
            "IDLE",
            "READY",
            "COMPLETE",
            "SEARCH_COMPLETE_NOT_FOUND",
            "ERROR",
        ):
            response.success = False
            response.message = (
                "현재 임무 상태에서는 시작할 수 없습니다: "
                f"{self.state}"
            )
            return response

        self._begin_mission()
        response.success = True
        response.message = "산림 조난자 탐색 임무를 시작했습니다."
        return response

    def _begin_mission(self):
        # READY/COMPLETE/미탐지 종료이면 이미 공중 Hover 중이다.
        already_airborne = self.hovering or self.state in (
            "READY",
            "COMPLETE",
            "SEARCH_COMPLETE_NOT_FOUND",
        )

        self.detection_count = 0
        self.victim_confirmed = False
        self.camera_position_received = False
        self.map_position_received = False

        if already_airborne:
            self.hovering = False
            self._publish_state("SEARCHING")
            self._send_drone_command("START_SEARCH")
        else:
            self.hovering = False
            self._publish_state("TAKEOFF")
            self._send_drone_command("TAKEOFF")

    def _land_callback(self, _request, response):
        self._publish_state("LANDING")
        self._send_drone_command("LAND")
        response.success = True
        response.message = "착륙 명령을 전송했습니다."
        return response

    def _drone_status_callback(self, message):
        status = message.data.upper()
        self.get_logger().info(f"드론 상태 수신: {status}")

        if (
            status == "CONNECTED"
            and self.state == "IDLE"
            and not self.initial_takeoff_requested
            and bool(
                self.get_parameter("auto_takeoff_on_connect").value
            )
        ):
            self.initial_takeoff_requested = True
            self._publish_state("INITIAL_TAKEOFF")
            self._send_drone_command("TAKEOFF")

        elif status == "AIRBORNE" and self.state == "INITIAL_TAKEOFF":
            # 목표 고도에 도착하면 수색을 시작하지 않고 제자리에서 대기한다.
            self._publish_state("INITIAL_HOVER")
            self._send_drone_command("HOVER")

        elif status == "AIRBORNE" and self.state == "TAKEOFF":
            self._publish_state("SEARCHING")
            self._send_drone_command("START_SEARCH")

        elif status == "HOVERING":
            self.hovering = True
            if self.state == "INITIAL_HOVER":
                self._publish_state("READY")
                self.get_logger().info(
                    "초기 이륙 완료: 제자리 Hover 상태에서 임무 시작을 "
                    "대기합니다."
                )
            elif self.victim_confirmed:
                self._publish_state("VICTIM_LOCATED")
                self._try_complete_mission()

        elif status == "LANDED":
            self.hovering = False
            self._publish_state("IDLE")

        elif (
            status == "SEARCH_FINISHED_NO_VICTIM"
            and self.state == "SEARCHING"
            and not self.victim_confirmed
        ):
            self.hovering = True
            self._publish_state("SEARCH_COMPLETE_NOT_FOUND")
            self.get_logger().warning(
                "전체 수색 경로를 완료했지만 조난자를 탐지하지 "
                "못했습니다. 현 위치에서 Hover를 유지합니다."
            )

        elif status.startswith("ERROR"):
            self._publish_state("ERROR")

    def _detection_callback(self, detection):
        if self.state != "SEARCHING" or self.victim_confirmed:
            return

        if detection.detected:
            self.detection_count += 1
            required = int(
                self.get_parameter("required_detection_frames").value
            )
            self.get_logger().info(
                "조난자 연속 탐지: "
                f"{self.detection_count}/{required}"
            )
            if self.detection_count >= required:
                self.victim_confirmed = True
                self._publish_state("VICTIM_DETECTED")
                self._send_drone_command("HOVER")
        else:
            self.detection_count = 0

    def _camera_position_callback(self, message):
        first_position = not self.camera_position_received
        self.camera_position_received = True
        if first_position and self.state in (
            "SEARCHING",
            "VICTIM_DETECTED",
            "VICTIM_LOCATED",
        ):
            self.get_logger().info(
                "조난자 카메라 위치 최초 수신: "
                f"({message.point.x:.2f}, {message.point.y:.2f}, "
                f"{message.point.z:.2f})"
            )
        self._try_complete_mission()

    def _map_position_callback(self, message):
        first_position = not self.map_position_received
        self.map_position_received = True
        if first_position and self.state in (
            "SEARCHING",
            "VICTIM_DETECTED",
            "VICTIM_LOCATED",
        ):
            self.get_logger().info(
                "조난자 map 위치 최초 수신: "
                f"({message.point.x:.2f}, {message.point.y:.2f}, "
                f"{message.point.z:.2f})"
            )
        self._try_complete_mission()

    def _try_complete_mission(self):
        # 위치 토픽이 계속 들어와도 COMPLETE 처리는 임무당 한 번만 수행한다.
        if self.state == "COMPLETE":
            return

        # TF가 아직 없어도 카메라 위치까지 계산되면 센서 파이프라인은 완료다.
        if not (
            self.victim_confirmed
            and self.hovering
            and self.camera_position_received
        ):
            return

        self._publish_state("COMPLETE")
        if bool(self.get_parameter("keep_hover_after_detection").value):
            self.get_logger().info(
                "조난자 위치에서 Hover를 유지합니다. "
                "착륙은 /mission/land 서비스를 호출하세요."
            )
        else:
            self._publish_state("LANDING")
            self._send_drone_command("LAND")

    def _send_drone_command(self, command):
        message = String()
        message.data = command
        self.command_publisher.publish(message)
        self.get_logger().info(f"드론 명령 전송: {command}")

    def _publish_state(self, state):
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
