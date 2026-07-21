#!/usr/bin/env python3

"""RGB 영상에서 조난자 후보를 찾는 교체 가능한 탐지 노드."""

import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from forest_rescue_interfaces.msg import VictimDetection


class HumanDetectorNode(Node):
    """Mock 또는 Ultralytics YOLO로 사람 Bounding Box를 발행한다."""

    def __init__(self):
        super().__init__("human_detector_node")

        self.declare_parameter("image_topic", "/quadrotor_01/Camera/rgb")
        self.declare_parameter("drone_id", "quadrotor_01")
        self.declare_parameter(
            "detection_topic", "/drone_01/victim/detection"
        )
        self.declare_parameter(
            "annotated_image_topic",
            "/drone_01/victim/annotated_image",
        )
        self.declare_parameter("detector_mode", "yolo")
        self.declare_parameter(
            "model_path",
            "~/b3_cobot3_ws/models/yolo11s.pt",
        )
        self.declare_parameter("person_class_id", 0)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("inference_period_sec", 0.2)
        self.declare_parameter("mission_state_topic", "/mission/state")
        # 시작 지점의 구조자를 조난자로 확정하지 않도록, 수색 시작 후
        # 일정 시간 동안 실제/Mock 탐지를 모두 비활성화한다.
        self.declare_parameter("detection_start_delay_sec", 60.0)
        self.declare_parameter("detect_only_while_searching", True)
        self.declare_parameter("mock_delay_sec", 8.0)
        self.declare_parameter("mock_confidence", 0.95)
        self.declare_parameter("mock_x_min_ratio", 0.46)
        self.declare_parameter("mock_y_min_ratio", 0.38)
        self.declare_parameter("mock_x_max_ratio", 0.54)
        self.declare_parameter("mock_y_max_ratio", 0.66)

        self.image_topic = self.get_parameter("image_topic").value
        self.drone_id = str(self.get_parameter("drone_id").value)
        self.detector_mode = str(
            self.get_parameter("detector_mode").value
        ).lower()
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.inference_period_sec = float(
            self.get_parameter("inference_period_sec").value
        )
        self.mock_delay_sec = float(
            self.get_parameter("mock_delay_sec").value
        )
        self.detection_start_delay_sec = float(
            self.get_parameter("detection_start_delay_sec").value
        )
        self.detect_only_while_searching = bool(
            self.get_parameter("detect_only_while_searching").value
        )

        self.bridge = CvBridge()
        self.last_inference_time = 0.0
        self.model = None
        self.mission_state = "IDLE"
        self.search_started_at = None
        self.delay_notice_printed = False

        self.detection_publisher = self.create_publisher(
            VictimDetection,
            self.get_parameter("detection_topic").value,
            10,
        )
        self.annotated_image_publisher = self.create_publisher(
            Image,
            self.get_parameter("annotated_image_topic").value,
            10,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            self._mission_state_callback,
            10,
        )

        if self.detector_mode == "yolo":
            self._load_yolo_model()
        elif self.detector_mode != "mock":
            raise ValueError(
                "detector_mode는 'mock' 또는 'yolo'여야 합니다."
            )

        self.get_logger().info(
            f"{self.drone_id} 탐지 모드={self.detector_mode}, "
            f"입력={self.image_topic}"
        )
        if self.detector_mode == "mock":
            self.get_logger().warning(
                "Mock 모드는 연결 시험용입니다. 실제 탐지 성능을 의미하지 "
                "않습니다."
            )

    def _mission_state_callback(self, message):
        state = message.data.strip().upper()
        if state == "SEARCHING" and self.mission_state != "SEARCHING":
            self.search_started_at = time.monotonic()
            self.delay_notice_printed = False
            self.get_logger().info(
                "SEARCHING 진입: "
                f"{self.detection_start_delay_sec:.1f}초 후 사람 탐지를 시작합니다."
            )
        elif state != "SEARCHING":
            self.search_started_at = None
            self.delay_notice_printed = False
        self.mission_state = state

    def _load_yolo_model(self):
        """Ultralytics 모델을 지연 없이 한 번만 불러온다."""
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                "YOLO 모드를 사용하려면 `pip install ultralytics`가 "
                "필요합니다."
            ) from error

        model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(
                f"YOLO 가중치 파일을 찾을 수 없습니다: {model_path}"
            )

        self.model = YOLO(str(model_path))
        self.get_logger().info(f"YOLO 모델 로드 완료: {model_path}")

    def _image_callback(self, message):
        """설정된 주기에 맞춰 최신 RGB 프레임을 처리한다."""
        now = time.monotonic()
        if now - self.last_inference_time < self.inference_period_sec:
            return
        self.last_inference_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except CvBridgeError as error:
            self.get_logger().error(f"RGB 변환 실패: {error}")
            return

        if not self._detection_is_enabled(now):
            detection = self._empty_detection(message)
        elif self.detector_mode == "mock":
            detection = self._run_mock_detection(message, image)
        else:
            detection = self._run_yolo_detection(message, image)

        # Localizer가 RGB와 Depth 해상도가 달라도 bbox를 올바르게
        # 스케일링할 수 있도록 원본 RGB 크기를 함께 전달한다.
        detection.image_width = int(image.shape[1])
        detection.image_height = int(image.shape[0])

        annotated = image.copy()
        if detection.detected:
            self._draw_detection(annotated, detection)

        self.detection_publisher.publish(detection)

        annotated_message = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding="bgr8",
        )
        annotated_message.header = message.header
        self.annotated_image_publisher.publish(annotated_message)

    def _detection_is_enabled(self, now):
        """수색 전 또는 초기 유예시간에는 사람 탐지 결과를 차단한다."""
        if self.detect_only_while_searching and self.mission_state != "SEARCHING":
            return False
        if self.search_started_at is None:
            return not self.detect_only_while_searching

        elapsed = now - self.search_started_at
        if elapsed < self.detection_start_delay_sec:
            if not self.delay_notice_printed:
                self.get_logger().info(
                    "초기 구조자 오탐 방지 중: "
                    f"탐지 활성화까지 {self.detection_start_delay_sec - elapsed:.1f}초"
                )
                self.delay_notice_printed = True
            return False

        if self.delay_notice_printed:
            self.get_logger().info("사람 탐지를 활성화했습니다.")
            self.delay_notice_printed = False
        return True

    def _empty_detection(self, image_message):
        detection = VictimDetection()
        detection.header = image_message.header
        detection.detected = False
        detection.class_name = "person"
        return detection

    def _run_mock_detection(self, image_message, image):
        """전체 파이프라인 검증을 위한 가상 Bounding Box를 만든다."""
        detection = self._empty_detection(image_message)
        if self.mission_state != "SEARCHING":
            return detection
        if self.search_started_at is None:
            return detection
        if time.monotonic() - self.search_started_at < self.mock_delay_sec:
            return detection

        height, width = image.shape[:2]
        detection.detected = True
        detection.confidence = float(
            self.get_parameter("mock_confidence").value
        )
        detection.x_min = int(
            width * self.get_parameter("mock_x_min_ratio").value
        )
        detection.y_min = int(
            height * self.get_parameter("mock_y_min_ratio").value
        )
        detection.x_max = int(
            width * self.get_parameter("mock_x_max_ratio").value
        )
        detection.y_max = int(
            height * self.get_parameter("mock_y_max_ratio").value
        )
        return detection

    def _run_yolo_detection(self, image_message, image):
        """YOLO 결과 중 confidence가 가장 높은 person을 선택한다."""
        detection = self._empty_detection(image_message)
        person_class_id = int(
            self.get_parameter("person_class_id").value
        )

        results = self.model.predict(
            source=image,
            conf=self.confidence_threshold,
            verbose=False,
        )

        best = None
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls.item())
                confidence = float(box.conf.item())
                if class_id != person_class_id:
                    continue
                if best is None or confidence > best[0]:
                    coordinates = box.xyxy[0].tolist()
                    best = (confidence, coordinates)

        if best is None:
            return detection

        confidence, coordinates = best
        x_min, y_min, x_max, y_max = coordinates
        detection.detected = True
        detection.confidence = confidence
        detection.x_min = int(round(x_min))
        detection.y_min = int(round(y_min))
        detection.x_max = int(round(x_max))
        detection.y_max = int(round(y_max))
        return detection

    @staticmethod
    def _draw_detection(image, detection):
        color = (0, 255, 0)
        cv2.rectangle(
            image,
            (detection.x_min, detection.y_min),
            (detection.x_max, detection.y_max),
            color,
            2,
        )
        label = f"person {detection.confidence:.2f}"
        cv2.putText(
            image,
            label,
            (detection.x_min, max(25, detection.y_min - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
