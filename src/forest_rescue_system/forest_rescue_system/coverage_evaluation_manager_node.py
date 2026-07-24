#!/usr/bin/env python3

"""정방향 1회와 역방향 1회의 카메라 커버리지를 단계별 평가한다."""

from datetime import datetime
from functools import partial
import json
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from forest_rescue_system.log_utils import TimestampedNode


_KST = ZoneInfo("Asia/Seoul")


class CoverageEvaluationManagerNode(TimestampedNode):
    """다중 드론의 왕복 수색과 두 번의 커버리지 스냅샷을 관리한다."""

    def __init__(self):
        super().__init__("coverage_evaluation_manager_node")

        self.declare_parameter("operation_mode", "eval_coverage")
        self.declare_parameter(
            "drone_ids",
            ["quadrotor_01", "quadrotor_02", "quadrotor_03"],
        )
        self.declare_parameter("auto_takeoff_on_connect", True)
        self.declare_parameter(
            "ground_truth_path",
            "~/b3_cobot3_ws/isaac_sim/generated_ground_truth.json",
        )
        self.declare_parameter(
            "search_plan_path",
            "~/b3_cobot3_ws/isaac_sim/generated_search_plan.json",
        )
        self.declare_parameter("require_simulation_mode_match", True)
        self.declare_parameter(
            "coverage_statistics_topic",
            "/forest_rescue/coverage/statistics",
        )
        self.declare_parameter("coverage_settle_sec", 1.5)
        self.declare_parameter("coverage_snapshot_timeout_sec", 6.0)
        self.declare_parameter("auto_return_after_evaluation", True)
        self.declare_parameter(
            "result_directory",
            "~/b3_cobot3_ws/coverage_results",
        )
        self.declare_parameter(
            "evaluation_result_topic",
            "/forest_rescue/coverage/evaluation_result",
        )

        self.operation_mode = str(
            self.get_parameter("operation_mode").value
        ).strip().lower()
        if self.operation_mode != "eval_coverage":
            raise RuntimeError(
                "coverage_evaluation_manager는 eval_coverage 모드에서만 "
                f"실행합니다: {self.operation_mode!r}"
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
        self.result_publisher = self.create_publisher(
            String,
            str(self.get_parameter("evaluation_result_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("coverage_statistics_topic").value),
            self._coverage_statistics_callback,
            10,
        )
        self.start_service = self.create_service(
            Trigger,
            "/mission/start",
            self._start_evaluation_callback,
        )
        self.land_service = self.create_service(
            Trigger,
            "/mission/land",
            self._land_callback,
        )

        self.state = "IDLE"
        self.initial_takeoff_requested = False
        self.failed_drones = set()
        self.forward_finished = set()
        self.reverse_finished = set()
        self.latest_coverage_statistics = None
        self.latest_statistics_received_wall = float("-inf")
        self.expected_coverage_session = None
        self.snapshot_phase = None
        self.snapshot_started_wall = None
        self.forward_snapshot = None
        self.final_snapshot = None
        self.evaluation_started_sim_sec = None
        self.evaluation_started_wall_kst = None
        self.result_path = None

        self._publish_mode()
        self._publish_state("IDLE")
        self.create_timer(1.0, self._republish_mode_and_state)
        self.create_timer(0.1, self._advance_snapshot)
        self.get_logger().info(
            "커버리지 평가 관리자 시작: "
            f"drones={self.drone_ids}, passes=forward+reverse"
        )

    def _coverage_statistics_callback(self, message):
        try:
            payload = json.loads(message.data)
            if str(payload.get("operation_mode", "")) != self.operation_mode:
                return
            if [str(value) for value in payload.get("drone_ids", [])] != self.drone_ids:
                return
            required = (
                "session_index",
                "terrain_total_area_m2",
                "terrain_covered_area_m2",
                "terrain_coverage_percent",
            )
            if any(key not in payload for key in required):
                raise ValueError(f"필수 통계 키가 없습니다: {required}")
            self.latest_coverage_statistics = payload
            self.latest_statistics_received_wall = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"커버리지 통계 파싱 실패: {error}")

    def _drone_status_callback(self, drone_id, message):
        status = str(message.data).strip().upper()
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
            "SEARCHING_FORWARD",
            "SEARCHING_REVERSE",
            "SEARCH_FORWARD_FINISHED",
            "AVOIDING_OBSTACLE",
            "AVOIDING_OBSTACLE_XY",
            "AVOIDING_OBSTACLE_VERTICAL_CLIMB",
            "AVOIDING_OBSTACLE_VERTICAL_CROSS",
        }
        if self.state == "INITIAL_TAKEOFF" and all(
            self.drone_status[item] in airborne_states
            for item in self._active_drones()
        ):
            self._publish_state("INITIAL_HOVER")
            self._send_active("HOVER")
            return

        if self.state == "INITIAL_HOVER" and self._active_drones() and all(
            self.drone_status[item] == "HOVERING"
            for item in self._active_drones()
        ):
            self._publish_state("EVAL_READY")
            self.get_logger().info(
                f"드론 {len(self._active_drones())}대 초기 이륙 완료: "
                "/mission/start 대기"
            )
            return

        if status == "SEARCH_FORWARD_FINISHED":
            self.forward_finished.add(drone_id)
            self._try_begin_forward_snapshot()
            return

        if status == "SEARCH_FINISHED_NO_VICTIM":
            self.reverse_finished.add(drone_id)
            self._try_begin_final_snapshot()
            return

        if status.startswith("ERROR"):
            self._handle_drone_error(drone_id, status)
            return

        if self.state == "EVAL_RETURNING":
            self._try_finish_return()

    def _handle_drone_error(self, drone_id, status):
        if drone_id in self.failed_drones:
            return
        self.failed_drones.add(drone_id)
        if status != "ERROR_LAND":
            self._send_command(drone_id, "HOVER")
        self.get_logger().error(
            f"{drone_id} 평가 오류 격리: {status}. "
            "나머지 정상 드론으로 평가를 계속합니다."
        )

        if not self._active_drones():
            self._publish_state("EVAL_FAILED")
            self._write_result_file(incomplete_reason="모든 드론 오류")
            return

        if self.state == "EVAL_FORWARD":
            self._try_begin_forward_snapshot()
        elif self.state == "EVAL_REVERSE":
            self._try_begin_final_snapshot()
        elif self.state == "EVAL_RETURNING":
            self._try_finish_return()

    def _start_evaluation_callback(self, _request, response):
        if self.state != "EVAL_READY":
            response.success = False
            response.message = (
                "EVAL_READY 상태에서만 평가를 시작할 수 있습니다: "
                f"{self.state}"
            )
            return response

        if bool(
            self.get_parameter("require_simulation_mode_match").value
        ) and not self._simulation_configuration_matches():
            response.success = False
            response.message = (
                "Isaac Sim의 operation_mode 또는 drone_count가 ROS Launch와 "
                "다릅니다. 같은 값으로 Isaac Sim을 다시 실행하세요."
            )
            return response

        if not self._search_plan_matches():
            response.success = False
            response.message = (
                "eval_coverage 함대와 일치하는 generated_search_plan.json을 "
                "찾지 못했습니다. Isaac Sim을 먼저 실행하세요."
            )
            return response

        self.failed_drones.clear()
        self.forward_finished.clear()
        self.reverse_finished.clear()
        self.forward_snapshot = None
        self.final_snapshot = None
        self.snapshot_phase = None
        self.snapshot_started_wall = None
        self.result_path = None
        latest_session = int(
            (self.latest_coverage_statistics or {}).get("session_index", 0)
        )
        self.expected_coverage_session = latest_session + 1
        self.evaluation_started_sim_sec = self._sim_time_sec()
        self.evaluation_started_wall_kst = datetime.now(_KST)

        self._publish_state("EVAL_FORWARD")
        self._send_active("START_SEARCH")
        response.success = True
        response.message = (
            f"드론 {len(self._active_drones())}대의 정방향 커버리지 평가를 "
            "시작했습니다. 정방향 완료 후 자동으로 역방향 1회를 수행합니다."
        )
        return response

    def _try_begin_forward_snapshot(self):
        if self.state != "EVAL_FORWARD":
            return
        active = self._active_drones()
        if not active or not all(item in self.forward_finished for item in active):
            return
        self.snapshot_phase = "forward"
        self.snapshot_started_wall = time.monotonic()
        self._publish_state("EVAL_FORWARD_SNAPSHOT")
        self.get_logger().warning(
            "모든 정상 드론의 정방향 수색 완료: 마지막 카메라 프레임을 "
            "반영한 뒤 정방향 커버리지를 저장합니다."
        )

    def _try_begin_final_snapshot(self):
        if self.state != "EVAL_REVERSE":
            return
        active = self._active_drones()
        if not active or not all(item in self.reverse_finished for item in active):
            return
        self.snapshot_phase = "final"
        self.snapshot_started_wall = time.monotonic()
        self._publish_state("EVAL_FINAL_SNAPSHOT")
        self.get_logger().warning(
            "모든 정상 드론의 역방향 수색 완료: 왕복 최종 커버리지를 "
            "저장합니다."
        )

    def _advance_snapshot(self):
        if self.snapshot_phase is None or self.snapshot_started_wall is None:
            return
        elapsed = time.monotonic() - self.snapshot_started_wall
        settle = max(
            0.0, float(self.get_parameter("coverage_settle_sec").value)
        )
        timeout = max(
            settle,
            float(
                self.get_parameter("coverage_snapshot_timeout_sec").value
            ),
        )
        statistics = self.latest_coverage_statistics
        valid_session = (
            statistics is not None
            and self.expected_coverage_session is not None
            and int(statistics.get("session_index", -1))
            >= self.expected_coverage_session
        )
        new_statistics = (
            valid_session
            and self.latest_statistics_received_wall
            >= self.snapshot_started_wall
        )

        if elapsed < settle or (not new_statistics and elapsed < timeout):
            return
        if not valid_session:
            self.get_logger().error(
                "현재 평가 세션의 커버리지 통계를 받지 못했습니다."
            )
            self._publish_state("EVAL_FAILED")
            self._write_result_file(
                incomplete_reason="커버리지 통계 수신 실패"
            )
            self.snapshot_phase = None
            return
        if not new_statistics:
            self.get_logger().warning(
                "스냅샷 대기시간 내 새 통계가 없어 최신 유효 통계를 사용합니다."
            )

        snapshot = self._snapshot_from_statistics(statistics)
        phase = self.snapshot_phase
        self.snapshot_phase = None
        self.snapshot_started_wall = None

        if phase == "forward":
            self.forward_snapshot = snapshot
            self.get_logger().warning(
                "[COVERAGE EVAL] 정방향 완료: "
                f"{snapshot['coverage_percent']:.2f}% "
                f"({snapshot['covered_area_m2']:.2f}/"
                f"{snapshot['total_area_m2']:.2f}m²)"
            )
            self._publish_state("EVAL_REVERSE")
            self._send_active("CONTINUE_REVERSE")
            return

        self.final_snapshot = snapshot
        self.get_logger().warning(
            "[COVERAGE EVAL] 왕복 완료: "
            f"{snapshot['coverage_percent']:.2f}% "
            f"({snapshot['covered_area_m2']:.2f}/"
            f"{snapshot['total_area_m2']:.2f}m²)"
        )
        self._write_result_file()
        if bool(
            self.get_parameter("auto_return_after_evaluation").value
        ):
            self._publish_state("EVAL_RETURNING")
            self._send_active("RETURN_HOME")
        else:
            self._publish_state(
                "EVAL_COMPLETE_WITH_ERROR"
                if self.failed_drones
                else "EVAL_COMPLETE"
            )

    @staticmethod
    def _snapshot_from_statistics(statistics):
        return {
            "total_area_m2": float(statistics["terrain_total_area_m2"]),
            "covered_area_m2": float(
                statistics["terrain_covered_area_m2"]
            ),
            "coverage_percent": float(
                statistics["terrain_coverage_percent"]
            ),
            "covered_triangles": int(
                statistics.get("terrain_covered_triangles", 0)
            ),
            "total_triangles": int(
                statistics.get("terrain_total_triangles", 0)
            ),
            "per_drone": statistics.get("per_drone", {}),
            "sim_time_sec": float(statistics.get("sim_time_sec", 0.0)),
        }

    def _try_finish_return(self):
        active = self._active_drones()
        resolved = all(
            self.drone_status[item] == "LANDED"
            for item in active
        )
        if not resolved:
            return
        # 착륙 단계에서 새 오류가 발생했을 수 있으므로 같은 결과 파일을
        # 최신 failed_drones와 종료 시각으로 한 번 더 갱신한다.
        self._write_result_file()
        self._publish_state(
            "EVAL_COMPLETE_WITH_ERROR"
            if self.failed_drones
            else "EVAL_COMPLETE"
        )
        self.get_logger().info(
            "커버리지 평가와 정상 드론의 홈 복귀·착륙이 완료됐습니다."
        )

    def _land_callback(self, _request, response):
        self._publish_state("EVAL_RETURNING")
        self._send_active("LAND")
        response.success = True
        response.message = "모든 정상 평가 드론에 착륙 명령을 전송했습니다."
        return response

    def _simulation_configuration_matches(self):
        path = Path(
            str(self.get_parameter("ground_truth_path").value)
        ).expanduser()
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            mode = str(payload["operation_mode"]).strip().lower()
            ids = [str(value) for value in payload["drone_ids"]]
            count = int(payload.get("drone_count", len(ids)))
            people_spawned = bool(payload.get("people_spawned", False))
            if mode != self.operation_mode:
                raise ValueError(
                    f"mode: sim={mode}, launch={self.operation_mode}"
                )
            if ids != self.drone_ids or count != len(self.drone_ids):
                raise ValueError(
                    f"fleet: sim={ids}, launch={self.drone_ids}"
                )
            if people_spawned or payload.get("victim") is not None:
                raise ValueError("eval_coverage에서 사람이 생성되어 있습니다.")
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
        ) as error:
            self.get_logger().error(
                f"Isaac Sim 평가 구성 검증 실패: {path}: {error}"
            )
            return False
        return True

    def _search_plan_matches(self):
        path = Path(
            str(self.get_parameter("search_plan_path").value)
        ).expanduser()
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            mode = str(payload["operation_mode"]).strip().lower()
            ids = [str(value) for value in payload["drone_ids"]]
            plans = payload["drones"]
            if mode != self.operation_mode or ids != self.drone_ids:
                raise ValueError(
                    f"mode={mode}, ids={ids}, expected={self.drone_ids}"
                )
            for drone_id in self.drone_ids:
                waypoints = plans[drone_id]["waypoints"]
                if not isinstance(waypoints, list) or len(waypoints) < 2:
                    raise ValueError(f"{drone_id} 경로가 비어 있습니다.")
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
        ) as error:
            self.get_logger().error(f"평가 수색 계획 검증 실패: {path}: {error}")
            return False
        return True

    def _write_result_file(self, incomplete_reason=None):
        started = self.evaluation_started_wall_kst or datetime.now(_KST)
        finished = datetime.now(_KST)
        forward = self.forward_snapshot
        final = self.final_snapshot
        reverse_gain = None
        if forward is not None and final is not None:
            reverse_gain = {
                "area_m2": float(
                    final["covered_area_m2"] - forward["covered_area_m2"]
                ),
                "percentage_points": float(
                    final["coverage_percent"] - forward["coverage_percent"]
                ),
            }

        payload = {
            "format_version": 1,
            "operation_mode": self.operation_mode,
            "status": (
                "incomplete" if incomplete_reason else "completed"
            ),
            "incomplete_reason": incomplete_reason,
            "drone_count": len(self.drone_ids),
            "drone_ids": list(self.drone_ids),
            "failed_drones": sorted(self.failed_drones),
            "started_at_kst": started.isoformat(timespec="milliseconds"),
            "finished_at_kst": finished.isoformat(timespec="milliseconds"),
            "elapsed_sim_sec": (
                self._sim_time_sec() - self.evaluation_started_sim_sec
                if self.evaluation_started_sim_sec is not None
                else None
            ),
            "forward": forward,
            "forward_reverse": final,
            "reverse_gain": reverse_gain,
            "search_repeat_mode": "forward_reverse_once",
            "coverage_denominator": "terrain_triangle_surface_area_m2",
        }

        directory = Path(
            str(self.get_parameter("result_directory").value)
        ).expanduser()
        try:
            directory.mkdir(parents=True, exist_ok=True)
            if self.result_path is not None:
                output_path = Path(self.result_path)
            else:
                filename = (
                    "coverage_eval_"
                    + finished.strftime("%Y%m%d_%H%M%S_%f")[:-3]
                    + ".json"
                )
                output_path = directory / filename
            temporary_path = output_path.with_suffix(".json.tmp")
            temporary_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            temporary_path.replace(output_path)
            self.result_path = output_path
            self.get_logger().warning(
                f"커버리지 평가 결과 저장: {output_path}"
            )
        except OSError as error:
            self.get_logger().error(f"커버리지 결과 파일 저장 실패: {error}")

        message = String()
        result_payload = dict(payload)
        result_payload["result_path"] = (
            str(self.result_path) if self.result_path else None
        )
        message.data = json.dumps(
            result_payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.result_publisher.publish(message)

    def _active_drones(self):
        return [
            drone_id
            for drone_id in self.drone_ids
            if drone_id not in self.failed_drones
        ]

    def _send_command(self, drone_id, command):
        message = String()
        message.data = command
        self.command_publishers[drone_id].publish(message)
        self.get_logger().info(f"{drone_id} 명령: {command}")

    def _send_active(self, command):
        for drone_id in self._active_drones():
            self._send_command(drone_id, command)

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
        self.get_logger().info(f"커버리지 평가 상태: {state}")

    def _republish_mode_and_state(self):
        self._publish_mode()
        message = String()
        message.data = self.state
        self.state_publisher.publish(message)

    def _sim_time_sec(self):
        return self.get_clock().now().nanoseconds / 1.0e9


def main(args=None):
    rclpy.init(args=args)
    node = CoverageEvaluationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
