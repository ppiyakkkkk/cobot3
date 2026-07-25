#!/usr/bin/env python3

"""Bounding Box와 Depth를 이용해 조난자의 3차원 위치를 계산한다."""

import math
import time
from collections import OrderedDict, deque

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

# PointStamped 변환 형식을 tf2에 등록한다.
import tf2_geometry_msgs  # noqa: F401

from forest_rescue_interfaces.msg import VictimDetection
from forest_rescue_system.common.log_utils import TimestampedNode


class VictimLocalizerNode(TimestampedNode):
    """사람 영역의 유효 Depth 중앙값으로 위치를 역투영한다."""

    def __init__(self):
        super().__init__("victim_localizer_node")

        self.declare_parameter("drone_id", "quadrotor_01")

        self.declare_parameter(
            "depth_topic",
            "/quadrotor_01/Camera/depth",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/quadrotor_01/Camera/camera_info",
        )
        self.declare_parameter(
            "detection_topic", "/drone_01/victim/detection"
        )
        self.declare_parameter(
            "camera_position_topic",
            "/drone_01/victim/position_camera",
        )
        self.declare_parameter(
            "map_position_topic",
            "/drone_01/victim/position_map",
        )
        self.declare_parameter("camera_frame_override", "Camera")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("roi_center_ratio", 0.5)
        self.declare_parameter("minimum_depth_m", 0.2)
        self.declare_parameter("maximum_depth_m", 30.0)
        # YOLO 추론 결과는 원본 RGB 촬영 시점보다 늦게 도착한다. 최신
        # Depth 한 장만 사용하지 않고 최근 프레임을 보관한 뒤 RGB stamp와
        # 가장 가까운 Depth를 선택한다.
        self.declare_parameter("depth_buffer_duration_sec", 3.0)
        self.declare_parameter("depth_buffer_max_frames", 120)
        self.declare_parameter("max_depth_detection_time_delta_sec", 0.35)
        # Detection이 Depth보다 먼저 도착하거나 YOLO 처리 지연으로 대응
        # 프레임이 아직 버퍼에 없을 수 있다. 양성 탐지를 즉시 버리지 않고
        # 잠시 보관한 뒤 Depth/CameraInfo가 들어올 때마다 다시 처리한다.
        self.declare_parameter("detection_retry_timeout_sec", 3.0)
        self.declare_parameter("detection_retry_period_sec", 0.05)
        self.declare_parameter("detection_retry_max_items", 80)
        # Detection 콜백 안에서 TF를 기다리면 단일 실행기가 막혀 TF 수신
        # 콜백도 처리되지 못한다. 요청 시각의 TF가 조금 늦게 도착하는 경우를
        # 위해 Point를 잠시 보관하고 별도 타이머에서 비동기로 재시도한다.
        self.declare_parameter("tf_retry_timeout_sec", 2.0)
        self.declare_parameter("tf_retry_period_sec", 0.02)
        self.declare_parameter("tf_retry_max_points", 40)
        self.declare_parameter("log_period_sec", 10.0)
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter(
            "active_localization_states",
            [
                "SEARCHING",
                "COOP_SEARCH_TRANSIT",
                "COOP_SEARCHING",
                "VICTIM_DETECTED",
                "VICTIM_LOCATED",
            ],
        )

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.active_localization_states = {
            str(value).strip().upper()
            for value in self.get_parameter(
                "active_localization_states"
            ).value
        }

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_header = None
        self.depth_buffer = deque()
        self.camera_info = None
        self.log_period_sec = max(
            0.1,
            float(self.get_parameter("log_period_sec").value),
        )
        self.last_log_times = {}
        self.mission_state = "IDLE"
        self.position_locked = False
        self.pending_detections = OrderedDict()
        self.pending_tf_points = OrderedDict()
        self.last_tf_error = None
        self.depth_rx_count = 0
        self.camera_info_rx_count = 0
        self.detection_rx_count = 0
        self.positive_detection_rx_count = 0
        self.map_position_tx_count = 0

        self.camera_position_publisher = self.create_publisher(
            PointStamped,
            self.get_parameter("camera_position_topic").value,
            10,
        )
        self.map_position_publisher = self.create_publisher(
            PointStamped,
            self.get_parameter("map_position_topic").value,
            10,
        )

        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VictimDetection,
            self.get_parameter("detection_topic").value,
            self._detection_callback,
            10,
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            self._mission_state_callback,
            state_qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_retry_timer = self.create_timer(
            max(
                0.01,
                float(self.get_parameter("tf_retry_period_sec").value),
            ),
            self._retry_pending_transforms,
        )
        self.detection_retry_timer = self.create_timer(
            max(
                0.02,
                float(
                    self.get_parameter(
                        "detection_retry_period_sec"
                    ).value
                ),
            ),
            self._retry_pending_detections,
        )
        self.diagnostics_timer = self.create_timer(
            5.0,
            self._log_pipeline_diagnostics,
        )

        self.get_logger().info(
            f"{self.drone_id} Depth 기반 조난자 위치 계산 노드 시작"
        )

    def _mission_state_callback(self, message):
        state = message.data.strip().upper()
        previous_state = self.mission_state
        if (
            self._localization_is_active(state)
            and not self._localization_is_active(self.mission_state)
        ):
            self.position_locked = False
            self.pending_detections.clear()
            self.pending_tf_points.clear()
            self.last_tf_error = None
            self.last_log_times.clear()
            self.get_logger().info(
                "새 수색 임무 시작: 조난자 위치 고정을 해제합니다."
            )
        self.mission_state = state
        if state != previous_state:
            self.get_logger().info(
                f"위치 계산 임무 상태 수신: {previous_state} → {state}"
            )

    def _depth_callback(self, message):
        self.depth_rx_count += 1
        try:
            depth = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="passthrough",
            )
        except CvBridgeError as error:
            if self._should_log("depth_conversion_error"):
                self.get_logger().error(f"Depth 변환 실패: {error}")
            return

        # CvBridge 결과가 메시지 메모리를 참조할 수 있으므로 버퍼에 넣을
        # 프레임은 복사해 콜백 종료 후에도 안전하게 유지한다.
        self.latest_depth = np.asarray(depth, dtype=np.float32).copy()
        self.latest_depth_header = message.header
        stamp_ns = self._stamp_to_nanoseconds(message.header.stamp)

        # Isaac Sim을 재시작하거나 시뮬레이션 시간이 되감긴 경우에는 이전
        # 실행의 Depth가 선택되지 않도록 버퍼를 즉시 비운다.
        if self.depth_buffer and stamp_ns < self.depth_buffer[-1][0]:
            self.depth_buffer.clear()
        self.depth_buffer.append(
            (stamp_ns, self.latest_depth_header, self.latest_depth)
        )
        self._prune_depth_buffer(stamp_ns)
        # Detection이 먼저 도착한 경우 새 Depth를 받은 즉시 다시 매칭한다.
        self._retry_pending_detections()

    def _camera_info_callback(self, message):
        self.camera_info_rx_count += 1
        self.camera_info = message
        self._retry_pending_detections()

    def _detection_callback(self, detection):
        self.detection_rx_count += 1
        if detection.detected:
            self.positive_detection_rx_count += 1
        if not self._localization_is_active(self.mission_state):
            if detection.detected and self._should_log("inactive_positive"):
                self.get_logger().warning(
                    "양성 탐지 메시지를 받았지만 현재 임무 상태가 위치 계산 "
                    f"대상이 아니어서 보류합니다: state={self.mission_state}"
                )
            return
        if self.position_locked:
            return
        if not detection.detected:
            return

        detection_stamp_ns = self._stamp_to_nanoseconds(
            detection.header.stamp
        )
        # 동일 stamp가 중복 수신되면 최초 대기 시각은 유지한다.
        if detection_stamp_ns not in self.pending_detections:
            self.pending_detections[detection_stamp_ns] = {
                "detection": detection,
                "queued_at": time.monotonic(),
                "last_reason": "QUEUED",
            }
            self.get_logger().info(
                f"[stamp={detection_stamp_ns}] localizer 양성 탐지 수신: "
                "Depth 위치 계산 대기열에 등록"
            )

        maximum_items = max(
            1,
            int(
                self.get_parameter(
                    "detection_retry_max_items"
                ).value
            ),
        )
        while len(self.pending_detections) > maximum_items:
            old_stamp, old_record = self.pending_detections.popitem(
                last=False
            )
            self.get_logger().warning(
                f"[stamp={old_stamp}] 위치 계산 실패: "
                "PENDING_QUEUE_OVERFLOW "
                f"(last_reason={old_record['last_reason']})"
            )

        self._retry_pending_detections()

    def _retry_pending_detections(self):
        """늦게 도착한 Depth/CameraInfo로 보류된 양성 탐지를 재처리한다."""
        if not self.pending_detections:
            return
        if self.position_locked:
            self.pending_detections.clear()
            return
        if not self._localization_is_active(self.mission_state):
            return

        timeout_sec = max(
            0.1,
            float(
                self.get_parameter(
                    "detection_retry_timeout_sec"
                ).value
            ),
        )
        now = time.monotonic()
        for stamp_ns, record in list(self.pending_detections.items()):
            processed, reason = self._try_localize_detection(
                record["detection"]
            )
            if processed:
                self.pending_detections.pop(stamp_ns, None)
                if reason not in {"MAP_PUBLISHED", "TF_PENDING"}:
                    self.get_logger().warning(
                        f"[stamp={stamp_ns}] 위치 계산 실패: {reason}"
                    )
                continue

            record["last_reason"] = reason
            if now - record["queued_at"] < timeout_sec:
                continue

            self.pending_detections.pop(stamp_ns, None)
            self.get_logger().warning(
                f"[stamp={stamp_ns}] 위치 계산 실패: {reason} "
                f"({timeout_sec:.1f}초 재시도 만료)"
            )

    def _try_localize_detection(self, detection):
        """한 양성 탐지의 Depth 역투영을 시도한다.

        반환값은 (처리 완료 여부, 현재 실패/대기 사유)다. 처리 완료에는
        map TF 즉시 발행뿐 아니라 TF 재시도 큐 등록도 포함된다.
        """
        if self.latest_depth is None:
            return False, "DEPTH_NOT_RECEIVED"
        if self.camera_info is None:
            return False, "CAMERA_INFO_MISSING"
        if not self.depth_buffer:
            return False, "DEPTH_BUFFER_EMPTY"

        # YOLO 추론 지연 때문에 콜백 시점의 최신 Depth는 원본 RGB보다 훨씬
        # 뒤일 수 있다. 최근 버퍼에서 RGB stamp와 가장 가까운 Depth를 찾는다.
        detection_stamp_ns = self._stamp_to_nanoseconds(detection.header.stamp)
        depth_stamp_ns, depth_header, selected_depth = min(
            self.depth_buffer,
            key=lambda item: abs(item[0] - detection_stamp_ns),
        )
        stamp_delta_sec = abs(detection_stamp_ns - depth_stamp_ns) / 1.0e9
        max_delta_sec = float(
            self.get_parameter("max_depth_detection_time_delta_sec").value
        )
        if stamp_delta_sec > max_delta_sec:
            return (
                False,
                "DEPTH_TIMESTAMP_MISMATCH"
                f"(delta={stamp_delta_sec:.3f}s,"
                f" depth_stamp={depth_stamp_ns})",
            )

        roi, u, v = self._extract_center_roi(detection, selected_depth)
        if roi.size == 0:
            return True, "INVALID_BOUNDING_BOX"

        minimum_depth = float(
            self.get_parameter("minimum_depth_m").value
        )
        maximum_depth = float(
            self.get_parameter("maximum_depth_m").value
        )
        valid = roi[
            np.isfinite(roi)
            & (roi >= minimum_depth)
            & (roi <= maximum_depth)
        ]
        if valid.size == 0:
            # Detection이 Depth보다 먼저 도착한 경우 현재 가장 가까운
            # 프레임에는 값이 없어도 다음 Depth 프레임에는 생길 수 있다.
            return False, "INVALID_DEPTH_PIXELS"

        depth_m = float(np.median(valid))
        self.get_logger().info(
            f"[stamp={detection_stamp_ns}] Depth 선택 성공: "
            f"delta={stamp_delta_sec:.3f}초, depth={depth_m:.2f}m, "
            f"valid_pixels={valid.size}"
        )

        # CameraInfo가 Depth와 다른 해상도로 발행되면 내부 파라미터도
        # 현재 Depth 영상 크기에 맞춰 스케일링한다.
        depth_height, depth_width = selected_depth.shape[:2]
        info_width = int(self.camera_info.width) or depth_width
        info_height = int(self.camera_info.height) or depth_height
        scale_x = depth_width / float(info_width)
        scale_y = depth_height / float(info_height)
        fx = float(self.camera_info.k[0]) * scale_x
        fy = float(self.camera_info.k[4]) * scale_y
        cx = float(self.camera_info.k[2]) * scale_x
        cy = float(self.camera_info.k[5]) * scale_y
        if fx <= 0.0 or fy <= 0.0:
            return False, "INVALID_CAMERA_INTRINSICS"

        # ROS optical frame: X=오른쪽, Y=아래, Z=카메라 전방
        point = PointStamped()
        point.header = detection.header
        frame_override = str(
            self.get_parameter("camera_frame_override").value
        )
        if frame_override:
            point.header.frame_id = frame_override
        elif not point.header.frame_id and depth_header:
            point.header.frame_id = depth_header.frame_id

        point.point.x = (u - cx) * depth_m / fx
        point.point.y = (v - cy) * depth_m / fy
        point.point.z = depth_m
        self.camera_position_publisher.publish(point)

        if self._should_log("camera_position"):
            self.get_logger().info(
                "조난자 카메라 좌표: "
                f"x={point.point.x:.2f}, "
                f"y={point.point.y:.2f}, "
                f"z={point.point.z:.2f}m"
            )
        map_published = self._publish_map_position(point)
        if (
            map_published
            and self.mission_state in ("VICTIM_DETECTED", "VICTIM_LOCATED")
        ):
            self.position_locked = True
            self.get_logger().info(
                "조난자 위치를 현재 임무의 확정 위치로 고정했습니다."
            )
        if not map_published:
            self.get_logger().info(
                f"[stamp={detection_stamp_ns}] Camera 좌표 계산 성공, "
                "Camera→map TF 비동기 재시도 대기"
            )
        return True, "MAP_PUBLISHED" if map_published else "TF_PENDING"

    def _localization_is_active(self, state):
        """전역 수색 상태와 향후 세부 수색/회피 상태를 모두 허용한다."""
        normalized = str(state).strip().upper()
        if normalized in self.active_localization_states:
            return True
        return normalized.startswith(
            (
                "COOP_SEARCHING_",
                "SEARCHING_",
                "AVOIDING_OBSTACLE",
            )
        )

    def _should_log(self, key):
        """토픽 계산은 유지하면서 같은 종류의 로그만 주기 제한한다."""
        now = time.monotonic()
        last = self.last_log_times.get(key)
        if last is not None and now - last < self.log_period_sec:
            return False
        self.last_log_times[key] = now
        return True

    def _log_pipeline_diagnostics(self):
        """탐지 이후 어느 입력 단계가 끊겼는지 주기적으로 표시한다."""
        if not self._localization_is_active(self.mission_state):
            return
        self.get_logger().info(
            "조난자 위치 파이프라인: "
            f"state={self.mission_state}, "
            f"depth_rx={self.depth_rx_count}, "
            f"camera_info_rx={self.camera_info_rx_count}, "
            f"detection_rx={self.detection_rx_count}, "
            f"positive_rx={self.positive_detection_rx_count}, "
            f"pending_depth={len(self.pending_detections)}, "
            f"pending_tf={len(self.pending_tf_points)}, "
            f"map_tx={self.map_position_tx_count}"
        )

    @staticmethod
    def _stamp_to_nanoseconds(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _prune_depth_buffer(self, newest_stamp_ns):
        """설정 시간과 최대 프레임 수를 넘은 Depth를 제거한다."""
        duration_sec = max(
            0.1,
            float(self.get_parameter("depth_buffer_duration_sec").value),
        )
        maximum_frames = max(
            2,
            int(self.get_parameter("depth_buffer_max_frames").value),
        )
        oldest_allowed_ns = newest_stamp_ns - int(duration_sec * 1.0e9)
        while (
            self.depth_buffer
            and self.depth_buffer[0][0] < oldest_allowed_ns
        ):
            self.depth_buffer.popleft()
        while len(self.depth_buffer) > maximum_frames:
            self.depth_buffer.popleft()

    def _extract_center_roi(self, detection, depth_image):
        height, width = depth_image.shape[:2]

        # Detection 좌표는 RGB 기준이므로 실제 Depth 해상도로 변환한다.
        source_width = int(detection.image_width) or width
        source_height = int(detection.image_height) or height
        bbox_scale_x = width / float(source_width)
        bbox_scale_y = height / float(source_height)
        x_min = max(
            0,
            min(width - 1, detection.x_min * bbox_scale_x),
        )
        y_min = max(
            0,
            min(height - 1, detection.y_min * bbox_scale_y),
        )
        x_max = max(
            0,
            min(width, detection.x_max * bbox_scale_x),
        )
        y_max = max(
            0,
            min(height, detection.y_max * bbox_scale_y),
        )
        if x_max <= x_min or y_max <= y_min:
            return np.array([], dtype=np.float32), 0.0, 0.0

        ratio = float(self.get_parameter("roi_center_ratio").value)
        ratio = max(0.1, min(1.0, ratio))
        center_x = 0.5 * (x_min + x_max)
        center_y = 0.5 * (y_min + y_max)
        half_width = 0.5 * (x_max - x_min) * ratio
        half_height = 0.5 * (y_max - y_min) * ratio

        roi_x_min = max(0, int(math.floor(center_x - half_width)))
        roi_x_max = min(width, int(math.ceil(center_x + half_width)))
        roi_y_min = max(0, int(math.floor(center_y - half_height)))
        roi_y_max = min(height, int(math.ceil(center_y + half_height)))
        roi = depth_image[
            roi_y_min:roi_y_max,
            roi_x_min:roi_x_max,
        ]
        return roi, center_x, center_y

    def _publish_map_position(self, camera_point):
        map_frame = str(self.get_parameter("map_frame").value)
        # YOLO가 처리한 원본 RGB 촬영 시각을 그대로 사용한다. 최신 TF를
        # 사용하면 추론 지연 동안 이동한 드론 위치가 적용되어 오차가 난다.
        point_for_tf = PointStamped()
        point_for_tf.header = camera_point.header
        point_for_tf.point = camera_point.point
        try:
            map_point = self.tf_buffer.transform(
                point_for_tf,
                map_frame,
                # 여기서 기다리면 같은 실행기의 TF 수신 콜백도 막힌다.
                # 즉시 확인하고 아직 없으면 아래 재시도 큐로 넘긴다.
                timeout=Duration(seconds=0.0),
            )
        except TransformException as error:
            self.last_tf_error = str(error)
            self._queue_tf_retry(point_for_tf)
            return False

        self._publish_transformed_map_point(map_point)
        return True

    def _queue_tf_retry(self, camera_point):
        """동일 촬영 시각 Point를 중복 없이 TF 재시도 큐에 보관한다."""
        stamp_ns = self._stamp_to_nanoseconds(camera_point.header.stamp)
        if stamp_ns not in self.pending_tf_points:
            self.pending_tf_points[stamp_ns] = (
                camera_point,
                time.monotonic(),
            )

        maximum_points = max(
            1,
            int(self.get_parameter("tf_retry_max_points").value),
        )
        while len(self.pending_tf_points) > maximum_points:
            self.pending_tf_points.popitem(last=False)

    def _retry_pending_transforms(self):
        """TF 수신 뒤 원본 RGB 촬영 시각으로 변환을 다시 시도한다."""
        if not self.pending_tf_points:
            return
        if self.position_locked:
            self.pending_tf_points.clear()
            return

        map_frame = str(self.get_parameter("map_frame").value)
        timeout_sec = max(
            0.1,
            float(self.get_parameter("tf_retry_timeout_sec").value),
        )
        now = time.monotonic()

        for stamp_ns, (camera_point, queued_at) in list(
            self.pending_tf_points.items()
        ):
            try:
                map_point = self.tf_buffer.transform(
                    camera_point,
                    map_frame,
                    timeout=Duration(seconds=0.0),
                )
            except TransformException as error:
                self.last_tf_error = str(error)
                if now - queued_at < timeout_sec:
                    continue
                self.pending_tf_points.pop(stamp_ns, None)
                if self._should_log("tf_warning"):
                    self.get_logger().warning(
                        "Camera→map TF가 재시도 시간 안에 도착하지 않아 "
                        "map 위치를 발행하지 못했습니다: "
                        f"{self.last_tf_error}"
                    )
                continue

            self.pending_tf_points.pop(stamp_ns, None)
            self._publish_transformed_map_point(map_point)
            if self.mission_state in (
                "VICTIM_DETECTED",
                "VICTIM_LOCATED",
            ):
                self.position_locked = True
                self.pending_tf_points.clear()
                self.get_logger().info(
                    "조난자 위치를 현재 임무의 확정 위치로 고정했습니다."
                )
                break

    def _publish_transformed_map_point(self, map_point):
        """변환 완료된 map 좌표를 발행하고 공통 로그를 출력한다."""
        self.map_position_publisher.publish(map_point)
        self.map_position_tx_count += 1
        stamp_ns = self._stamp_to_nanoseconds(map_point.header.stamp)
        self.get_logger().info(
            f"[stamp={stamp_ns}] map 위치 발행 성공"
        )
        if self._should_log("map_position"):
            self.get_logger().info(
                "조난자 map 좌표: "
                f"x={map_point.point.x:.2f}, "
                f"y={map_point.point.y:.2f}, "
                f"z={map_point.point.z:.2f}m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = VictimLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
