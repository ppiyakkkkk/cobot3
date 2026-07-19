#!/usr/bin/env python3

"""Bounding Box와 Depth를 이용해 조난자의 3차원 위치를 계산한다."""

import math
import time

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

# PointStamped 변환 형식을 tf2에 등록한다.
import tf2_geometry_msgs  # noqa: F401

from forest_rescue_interfaces.msg import VictimDetection


class VictimLocalizerNode(Node):
    """사람 영역의 유효 Depth 중앙값으로 위치를 역투영한다."""

    def __init__(self):
        super().__init__("victim_localizer_node")

        self.declare_parameter(
            "depth_topic",
            "/quadrotor/Camera/depth",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/quadrotor/Camera/camera_info",
        )
        self.declare_parameter("detection_topic", "/victim/detection")
        self.declare_parameter(
            "camera_position_topic",
            "/victim/position_camera",
        )
        self.declare_parameter(
            "map_position_topic",
            "/victim/position_map",
        )
        self.declare_parameter("camera_frame_override", "Camera")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("roi_center_ratio", 0.5)
        self.declare_parameter("minimum_depth_m", 0.2)
        self.declare_parameter("maximum_depth_m", 30.0)
        self.declare_parameter("log_period_sec", 10.0)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_header = None
        self.camera_info = None
        self.log_period_sec = max(
            0.1,
            float(self.get_parameter("log_period_sec").value),
        )
        self.last_log_times = {}
        self.mission_state = "IDLE"
        self.position_locked = False

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
        self.create_subscription(
            String,
            "/mission/state",
            self._mission_state_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Depth 기반 조난자 위치 계산 노드 시작")

    def _mission_state_callback(self, message):
        state = message.data.strip().upper()
        if state == "SEARCHING" and self.mission_state != "SEARCHING":
            self.position_locked = False
            self.last_log_times.clear()
            self.get_logger().info(
                "새 수색 임무 시작: 조난자 위치 고정을 해제합니다."
            )
        self.mission_state = state

    def _depth_callback(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="passthrough",
            )
        except CvBridgeError as error:
            if self._should_log("depth_conversion_error"):
                self.get_logger().error(f"Depth 변환 실패: {error}")
            return

        self.latest_depth = np.asarray(depth, dtype=np.float32)
        self.latest_depth_header = message.header

    def _camera_info_callback(self, message):
        self.camera_info = message

    def _detection_callback(self, detection):
        if self.mission_state not in (
            "SEARCHING",
            "VICTIM_DETECTED",
            "VICTIM_LOCATED",
        ):
            return
        if self.position_locked:
            return
        if not detection.detected:
            return
        if self.latest_depth is None or self.camera_info is None:
            if self._should_log("missing_depth_or_info"):
                self.get_logger().warning(
                    "Depth 또는 CameraInfo를 아직 받지 못했습니다."
                )
            return

        roi, u, v = self._extract_center_roi(detection)
        if roi.size == 0:
            if self._should_log("invalid_bbox"):
                self.get_logger().warning(
                    "탐지 Bounding Box가 유효하지 않습니다."
                )
            return

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
            if self._should_log("invalid_depth"):
                self.get_logger().warning(
                    "사람 영역에서 유효한 Depth 값을 찾지 못했습니다."
                )
            return

        depth_m = float(np.median(valid))

        # CameraInfo가 Depth와 다른 해상도로 발행되면 내부 파라미터도
        # 현재 Depth 영상 크기에 맞춰 스케일링한다.
        depth_height, depth_width = self.latest_depth.shape[:2]
        info_width = int(self.camera_info.width) or depth_width
        info_height = int(self.camera_info.height) or depth_height
        scale_x = depth_width / float(info_width)
        scale_y = depth_height / float(info_height)
        fx = float(self.camera_info.k[0]) * scale_x
        fy = float(self.camera_info.k[4]) * scale_y
        cx = float(self.camera_info.k[2]) * scale_x
        cy = float(self.camera_info.k[5]) * scale_y
        if fx <= 0.0 or fy <= 0.0:
            if self._should_log("invalid_camera_info"):
                self.get_logger().error(
                    "CameraInfo 내부 초점거리가 잘못됐습니다."
                )
            return

        # ROS optical frame: X=오른쪽, Y=아래, Z=카메라 전방
        point = PointStamped()
        point.header = detection.header
        frame_override = str(
            self.get_parameter("camera_frame_override").value
        )
        if frame_override:
            point.header.frame_id = frame_override
        elif not point.header.frame_id and self.latest_depth_header:
            point.header.frame_id = self.latest_depth_header.frame_id

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
        if self._publish_map_position(point):
            self.position_locked = True
            self.get_logger().info(
                "조난자 위치를 현재 임무의 확정 위치로 고정했습니다."
            )

    def _should_log(self, key):
        """토픽 계산은 유지하면서 같은 종류의 로그만 주기 제한한다."""
        now = time.monotonic()
        last = self.last_log_times.get(key)
        if last is not None and now - last < self.log_period_sec:
            return False
        self.last_log_times[key] = now
        return True

    def _extract_center_roi(self, detection):
        height, width = self.latest_depth.shape[:2]

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
        roi = self.latest_depth[
            roi_y_min:roi_y_max,
            roi_x_min:roi_x_max,
        ]
        return roi, center_x, center_y

    def _publish_map_position(self, camera_point):
        map_frame = str(self.get_parameter("map_frame").value)
        # Isaac Sim 영상 stamp와 ROS 노드의 wall time이 다를 수 있으므로
        # 기본 시스템에서는 가장 최근 TF를 사용한다.
        point_for_tf = PointStamped()
        point_for_tf.header.frame_id = camera_point.header.frame_id
        point_for_tf.point = camera_point.point
        try:
            map_point = self.tf_buffer.transform(
                point_for_tf,
                map_frame,
                timeout=Duration(seconds=0.1),
            )
        except TransformException as error:
            if self._should_log("tf_warning"):
                self.get_logger().warning(
                    "Camera→map TF가 아직 없어 map 위치를 발행하지 "
                    f"못했습니다: {error}"
                )
            return False

        self.map_position_publisher.publish(map_point)
        if self._should_log("map_position"):
            self.get_logger().info(
                "조난자 map 좌표: "
                f"x={map_point.point.x:.2f}, "
                f"y={map_point.point.y:.2f}, "
                f"z={map_point.point.z:.2f}m"
            )
        return True


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
