#!/usr/bin/env python3

"""3D 매핑 운용 모드의 상태와 공통 드론 명령을 관리하는 골격 노드."""

from functools import partial
import json
from pathlib import Path

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from forest_rescue_system.log_utils import TimestampedNode


class MappingManagerNode(TimestampedNode):
    """수색 상태와 분리된 3D 매핑 전용 상태 머신 골격이다.

    현재 단계에서는 이륙, 초기 Hover, 모드 상태 발행, 전체 착륙까지만
    제공한다. 실제 PointCloud 누적과 매핑 경로 비행은 다음 단계에서
    별도 매핑 노드에 연결한다.
    """

    def __init__(self):
        super().__init__("mapping_manager_node")

        self.declare_parameter("operation_mode", "mapping_3d")
        self.declare_parameter(
            "drone_ids",
            ["quadrotor_01", "quadrotor_02", "quadrotor_03"],
        )
        self.declare_parameter("auto_takeoff_on_connect", True)
        self.declare_parameter(
            "ground_truth_path",
            "~/b3_cobot3_ws/isaac_sim/generated_ground_truth.json",
        )
        self.declare_parameter("require_simulation_mode_match", True)

        self.operation_mode = str(
            self.get_parameter("operation_mode").value
        ).strip().lower()
        if self.operation_mode != "mapping_3d":
            raise RuntimeError(
                "mapping_manager는 operation_mode=mapping_3d에서만 "
                f"실행할 수 있습니다: {self.operation_mode!r}"
            )

        self.drone_ids = [
            str(value) for value in self.get_parameter("drone_ids").value
        ]
        if not self.drone_ids:
            raise RuntimeError("drone_ids가 비어 있습니다.")

        self.command_publishers = {}
        self.drone_status = {
            drone_id: "UNKNOWN" for drone_id in self.drone_ids
        }
        self.failed_drones = set()

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
        self.start_service = self.create_service(
            Trigger,
            "/mission/start",
            self._start_mapping_callback,
        )
        self.land_service = self.create_service(
            Trigger,
            "/mission/land",
            self._land_callback,
        )

        self.state = "IDLE"
        self.initial_takeoff_requested = False
        self._publish_mode()
        self._publish_state("IDLE")
        self.republish_timer = self.create_timer(
            1.0,
            self._republish_mode_and_state,
        )

        self.get_logger().info(
            "3D 매핑 모드 골격 시작: "
            f"drones={self.drone_ids}, "
            "실제 PointCloud 누적/매핑 비행은 아직 연결되지 않았습니다."
        )

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
            self._publish_state("MAPPING_READY")
            self.get_logger().info(
                f"드론 {len(self.drone_ids)}대 초기 이륙 완료: "
                "3D 매핑 모드 골격 준비"
            )
            return

        if status.startswith("ERROR"):
            self.failed_drones.add(drone_id)
            if status != "ERROR_LAND":
                self._send_command(drone_id, "HOVER")
            self._publish_state("MAPPING_FAILED")
            self.get_logger().error(
                f"{drone_id} 오류로 3D 매핑 모드를 중단했습니다: {status}"
            )
            return

        if self.state == "MAPPING_RETURNING":
            resolved = all(
                self.drone_status[item] == "LANDED"
                or item in self.failed_drones
                for item in self.drone_ids
            )
            if resolved:
                if self.failed_drones:
                    self._publish_state("MAPPING_COMPLETE_WITH_ERROR")
                else:
                    self._publish_state("MAPPING_COMPLETE")

    def _start_mapping_callback(self, _request, response):
        if self.state != "MAPPING_READY":
            response.success = False
            response.message = (
                "MAPPING_READY 상태에서만 시작할 수 있습니다: "
                f"{self.state}"
            )
            return response

        if (
            bool(
                self.get_parameter(
                    "require_simulation_mode_match"
                ).value
            )
            and not self._simulation_configuration_matches()
        ):
            response.success = False
            response.message = (
                "Isaac Sim의 operation_mode 또는 drone_count가 ROS Launch와 "
                "다릅니다. 같은 값으로 Isaac Sim을 다시 실행하세요."
            )
            self.get_logger().error(response.message)
            return response

        # 이 단계에서는 상태 구조와 모드 분리만 제공한다. START_SEARCH를
        # 잘못 보내 기존 구조 수색 경로가 실행되지 않도록 Hover를 유지한다.
        self._publish_state("MAPPING_PREPARING")
        self._send_all("HOVER")
        self._publish_state("MAPPING_NOT_IMPLEMENTED")
        response.success = False
        response.message = (
            "mapping_3d 모드 선택 골격은 정상입니다. 실제 PointCloud 누적, "
            "지도 저장, 매핑 경로 비행은 다음 구현 단계입니다."
        )
        self.get_logger().warning(response.message)
        return response

    def _simulation_configuration_matches(self):
        """Isaac Ground Truth의 모드와 함대가 현재 Launch와 같은지 확인한다."""
        path = Path(
            str(self.get_parameter("ground_truth_path").value)
        ).expanduser()
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            sim_mode = str(payload["operation_mode"]).strip().lower()
            sim_ids = [str(value) for value in payload["drone_ids"]]
            sim_count = int(payload.get("drone_count", len(sim_ids)))
            people_spawned = bool(payload.get("people_spawned", False))

            if sim_mode != self.operation_mode:
                raise ValueError(
                    f"mode: sim={sim_mode}, launch={self.operation_mode}"
                )
            if sim_ids != self.drone_ids or sim_count != len(self.drone_ids):
                raise ValueError(
                    f"fleet: sim={sim_ids}, launch={self.drone_ids}"
                )
            if people_spawned:
                raise ValueError(
                    "mapping_3d인데 Ground Truth에 사람이 생성되어 있습니다"
                )
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
        ) as error:
            self.get_logger().warning(
                "Isaac Sim 매핑 모드 구성 검증 실패: "
                f"{path}: {error}"
            )
            return False

        self.get_logger().info(
            "Isaac Sim 매핑 모드 구성 검증 완료: "
            f"operation_mode={sim_mode}, drone_ids={sim_ids}"
        )
        return True

    def _land_callback(self, _request, response):
        self._publish_state("MAPPING_RETURNING")
        self._send_all("LAND")
        response.success = True
        response.message = "모든 매핑 드론에 착륙 명령을 전송했습니다."
        return response

    def _send_command(self, drone_id, command):
        message = String()
        message.data = command
        self.command_publishers[drone_id].publish(message)
        self.get_logger().info(f"{drone_id} 명령: {command}")

    def _send_all(self, command):
        for drone_id in self.drone_ids:
            self._send_command(drone_id, command)

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
        self.get_logger().info(f"매핑 임무 상태: {state}")

    def _republish_mode_and_state(self):
        self._publish_mode()
        message = String()
        message.data = self.state
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MappingManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
