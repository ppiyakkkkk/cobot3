#!/usr/bin/env python3

"""드론 body와 카메라/LiDAR 사이의 정적 TF를 발행한다."""

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from forest_rescue_system.log_utils import TimestampedNode


class SensorTfNode(TimestampedNode):
    def __init__(self):
        super().__init__("sensor_tf_node")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "Camera")
        self.declare_parameter("lidar_frame", "base_scan")
        self.declare_parameter("camera_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_down_tilt_deg", 40.0)
        self.declare_parameter("derive_camera_quaternion_from_tilt", True)
        # 이전 설정과의 호환용이다. derive_camera_quaternion_from_tilt가
        # false일 때만 아래 Quaternion을 직접 사용한다.
        self.declare_parameter(
            "camera_quaternion_xyzw",
            [-0.640856, 0.640856, -0.298836, 0.298836],
        )
        self.declare_parameter("lidar_translation", [0.0, 0.0, 0.15])
        self.declare_parameter(
            "lidar_quaternion_xyzw",
            [0.0, 0.0, 0.0, 1.0],
        )

        if bool(
            self.get_parameter("derive_camera_quaternion_from_tilt").value
        ):
            camera_quaternion = self._camera_optical_quaternion_from_tilt(
                float(self.get_parameter("camera_down_tilt_deg").value)
            )
        else:
            camera_quaternion = list(
                self.get_parameter("camera_quaternion_xyzw").value
            )

        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = [
            self._make_transform(
                str(self.get_parameter("camera_frame").value),
                list(self.get_parameter("camera_translation").value),
                camera_quaternion,
            ),
            self._make_transform(
                str(self.get_parameter("lidar_frame").value),
                list(self.get_parameter("lidar_translation").value),
                list(
                    self.get_parameter(
                        "lidar_quaternion_xyzw"
                    ).value
                ),
            ),
        ]
        self.broadcaster.sendTransform(transforms)
        self.get_logger().info(
            "정적 센서 TF 발행: "
            f"{self.get_parameter('base_frame').value}→"
            f"{self.get_parameter('camera_frame').value}, "
            f"{self.get_parameter('base_frame').value}→"
            f"{self.get_parameter('lidar_frame').value}, "
            f"camera_down_tilt="
            f"{float(self.get_parameter('camera_down_tilt_deg').value):.1f}°"
        )

    @staticmethod
    def _camera_optical_quaternion_from_tilt(down_tilt_deg):
        """ROS optical frame을 body 전방에서 아래로 기울인 Quaternion.

        body 좌표는 +X 전방, +Y 좌측, +Z 위쪽이고 ROS optical frame은
        +X 오른쪽, +Y 아래쪽, +Z 영상 전방이다. Isaac의 실제 카메라
        하향각과 같은 값을 사용해 Depth 역투영 좌표와 map TF를 일치시킨다.
        """
        theta = math.radians(float(down_tilt_deg))
        sine = math.sin(theta)
        cosine = math.cos(theta)

        # optical frame의 각 축을 body frame으로 나타낸 회전행렬이다.
        rotation = (
            (0.0, -sine, cosine),
            (-1.0, 0.0, 0.0),
            (0.0, -cosine, -sine),
        )
        return SensorTfNode._quaternion_from_rotation_matrix(rotation)

    @staticmethod
    def _quaternion_from_rotation_matrix(matrix):
        """3x3 회전행렬을 normalized XYZW Quaternion으로 변환한다."""
        m00, m01, m02 = matrix[0]
        m10, m11, m12 = matrix[1]
        m20, m21, m22 = matrix[2]
        trace = m00 + m11 + m22

        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (m21 - m12) / scale
            y = (m02 - m20) / scale
            z = (m10 - m01) / scale
        elif m00 > m11 and m00 > m22:
            scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            w = (m21 - m12) / scale
            x = 0.25 * scale
            y = (m01 + m10) / scale
            z = (m02 + m20) / scale
        elif m11 > m22:
            scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            w = (m02 - m20) / scale
            x = (m01 + m10) / scale
            y = 0.25 * scale
            z = (m12 + m21) / scale
        else:
            scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            w = (m10 - m01) / scale
            x = (m02 + m20) / scale
            y = (m12 + m21) / scale
            z = 0.25 * scale

        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1.0e-12:
            raise RuntimeError("카메라 Quaternion 계산 결과가 유효하지 않습니다.")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

        # 같은 회전을 나타내는 q와 -q 중 w가 양수인 표현을 사용한다.
        if w < 0.0:
            x, y, z, w = -x, -y, -z, -w
        return [x, y, z, w]

    def _make_transform(self, child_frame, translation, quaternion):
        if len(translation) != 3:
            raise ValueError(f"translation은 3개 값이어야 합니다: {translation}")
        if len(quaternion) != 4:
            raise ValueError(f"quaternion은 4개 값이어야 합니다: {quaternion}")

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(
            self.get_parameter("base_frame").value
        )
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        return transform


def main(args=None):
    rclpy.init(args=args)
    node = SensorTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
