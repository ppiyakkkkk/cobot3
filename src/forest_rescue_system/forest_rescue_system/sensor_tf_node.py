#!/usr/bin/env python3

"""드론 body와 카메라/LiDAR 사이의 정적 TF를 발행한다."""

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
        self.declare_parameter("camera_translation", [0.3, 0.0, -0.07])
        self.declare_parameter(
            "camera_quaternion_xyzw",
            [-0.612372, 0.612372, -0.353553, 0.353553],
        )
        self.declare_parameter("lidar_translation", [0.0, 0.0, 0.15])
        self.declare_parameter(
            "lidar_quaternion_xyzw",
            [0.0, 0.0, 0.0, 1.0],
        )

        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = [
            self._make_transform(
                str(self.get_parameter("camera_frame").value),
                list(self.get_parameter("camera_translation").value),
                list(
                    self.get_parameter(
                        "camera_quaternion_xyzw"
                    ).value
                ),
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
            "정적 센서 TF 발행: base_link→Camera, base_link→base_scan"
        )

    def _make_transform(self, child_frame, translation, quaternion):
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
