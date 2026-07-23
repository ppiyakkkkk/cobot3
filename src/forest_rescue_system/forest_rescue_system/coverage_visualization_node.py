#!/usr/bin/env python3

"""드론 3대가 depth로 실제 확인한 지형/식생을 누적 표시한다."""

from functools import partial
from pathlib import Path
import time

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from forest_rescue_system import coverage_geometry, coverage_mesh
from forest_rescue_system.coverage_ownership import TriangleOwnership
from forest_rescue_system.log_utils import TimestampedNode

_COLOR_PARAMETERS = (
    "drone_01_color_rgb",
    "drone_02_color_rgb",
    "drone_03_color_rgb",
)


class CoverageVisualizationNode(TimestampedNode):
    """카메라 depth 기반 실제 가시성으로 커버리지를 누적 표시한다."""

    def __init__(self, **kwargs):
        super().__init__("coverage_visualization_node", **kwargs)

        self.declare_parameter(
            "drone_ids",
            ["quadrotor_01", "quadrotor_02", "quadrotor_03"],
        )
        self.declare_parameter(
            "terrain_mesh_path",
            "~/b3_cobot3_ws/isaac_sim/generated_terrain_mesh.npz",
        )
        self.declare_parameter(
            "environment_mesh_path",
            "~/b3_cobot3_ws/isaac_sim/generated_environment_meshes.npz",
        )
        self.declare_parameter(
            "coverage_marker_topic", "/forest_rescue/coverage_markers"
        )
        self.declare_parameter(
            "coverage_area_topic", "/forest_rescue/coverage_area_m2"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("refresh_period_sec", 1.0)
        self.declare_parameter("area_publish_period_sec", 1.0)
        self.declare_parameter("visibility_tolerance_m", 0.5)
        self.declare_parameter("minimum_depth_m", 0.20)
        self.declare_parameter("maximum_depth_m", 20.0)
        self.declare_parameter("coverage_z_offset_m", 0.05)
        self.declare_parameter("drone_01_color_rgb", [0.55, 0.0, 0.85])
        self.declare_parameter("drone_02_color_rgb", [0.73, 0.33, 0.83])
        self.declare_parameter("drone_03_color_rgb", [0.60, 0.0, 0.50])

        self.drone_ids = [
            str(value) for value in self.get_parameter("drone_ids").value
        ]
        self.terrain_mesh_path = Path(
            str(self.get_parameter("terrain_mesh_path").value)
        ).expanduser()
        self.environment_mesh_path = Path(
            str(self.get_parameter("environment_mesh_path").value)
        ).expanduser()
        self.map_frame = str(self.get_parameter("map_frame").value)

        self.scene = None
        self.ownership = None
        self.last_mesh_wait_log_at = float("-inf")

        self.bridge = CvBridge()
        self.camera_info_by_drone = {}
        self.depth_by_drone = {}

        for drone_id in self.drone_ids:
            self.create_subscription(
                CameraInfo,
                f"/{drone_id}/Camera/camera_info",
                partial(self._camera_info_callback, drone_id),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                f"/{drone_id}/Camera/depth",
                partial(self._depth_callback, drone_id),
                qos_profile_sensor_data,
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("coverage_marker_topic").value),
            marker_qos,
        )
        self.area_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("coverage_area_topic").value),
            10,
        )

        self._load_mesh_if_ready()

        refresh_period = max(
            0.1, float(self.get_parameter("refresh_period_sec").value)
        )
        self.refresh_timer = self.create_timer(
            refresh_period, self._refresh_coverage
        )
        area_period = max(
            0.1, float(self.get_parameter("area_publish_period_sec").value)
        )
        self.area_timer = self.create_timer(area_period, self._publish_area)

        self.get_logger().info("커버리지 시각화 노드 시작")

    def _camera_info_callback(self, drone_id, message):
        self.camera_info_by_drone[drone_id] = message

    def _depth_callback(self, drone_id, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="passthrough"
            )
        except CvBridgeError as error:
            self.get_logger().error(f"{drone_id} Depth 변환 실패: {error}")
            return
        self.depth_by_drone[drone_id] = np.asarray(
            depth, dtype=np.float32
        ).copy()

    def _load_mesh_if_ready(self):
        if self.scene is not None:
            return
        if (
            not self.terrain_mesh_path.is_file()
            or not self.environment_mesh_path.is_file()
        ):
            now = time.monotonic()
            if now - self.last_mesh_wait_log_at >= 5.0:
                self.get_logger().info(
                    "지형/환경 Mesh 파일 대기 중: "
                    f"{self.terrain_mesh_path}, {self.environment_mesh_path}"
                )
                self.last_mesh_wait_log_at = now
            return

        try:
            groups = {}
            groups.update(
                coverage_mesh.load_terrain_group(self.terrain_mesh_path)
            )
            groups.update(
                coverage_mesh.load_environment_groups(
                    self.environment_mesh_path
                )
            )
        except (OSError, KeyError, ValueError) as error:
            self.get_logger().warning(
                f"Mesh 읽기 실패, 다음 주기에 재시도: {error}"
            )
            return

        if not groups:
            self.get_logger().warning(
                "Mesh 파일에 표시 가능한 그룹이 없습니다."
            )
            return

        self.scene = coverage_geometry.assemble_scene(groups)
        self.ownership = TriangleOwnership(len(self.scene.centroids))
        self.get_logger().info(
            "Mesh 로드 완료: "
            f"groups={self.scene.group_names}, "
            f"triangles={len(self.scene.centroids)}"
        )

    def _refresh_coverage(self):
        self._load_mesh_if_ready()
        if self.scene is None or self.ownership is None:
            return

        any_newly_claimed = False
        for drone_index, drone_id in enumerate(self.drone_ids):
            any_newly_claimed |= self._process_drone(drone_index, drone_id)

        if any_newly_claimed:
            self._publish_markers()

    def _process_drone(self, drone_index, drone_id):
        camera_info = self.camera_info_by_drone.get(drone_id)
        depth_image = self.depth_by_drone.get(drone_id)
        if camera_info is None or depth_image is None:
            return False

        candidate_indices = np.where(self.ownership.unclaimed_mask())[0]
        if candidate_indices.size == 0:
            return False

        camera_frame = f"{drone_id}/camera_optical_frame"
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                camera_frame,
                self.map_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return False

        matrix = coverage_geometry.transform_matrix_from_tf(
            (
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z,
            ),
            (
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w,
            ),
        )
        sample_points = coverage_geometry.triangle_sample_points(
            self.scene.triangle_positions[candidate_indices]
        )
        sample_points_camera = coverage_geometry.apply_transform(
            sample_points.reshape(-1, 3), matrix
        ).reshape(sample_points.shape)

        depth_height, depth_width = depth_image.shape[:2]
        info_width = int(camera_info.width) or depth_width
        info_height = int(camera_info.height) or depth_height
        fx, fy, cx, cy = coverage_geometry.scaled_intrinsics(
            camera_info.k, info_width, info_height, depth_width, depth_height
        )

        visible = coverage_geometry.visibility_mask_multi_sample(
            sample_points_camera,
            fx,
            fy,
            cx,
            cy,
            depth_image,
            float(self.get_parameter("visibility_tolerance_m").value),
            float(self.get_parameter("minimum_depth_m").value),
            float(self.get_parameter("maximum_depth_m").value),
        )
        visible_global_indices = candidate_indices[visible]
        newly_claimed = np.asarray([], dtype=np.int64)
        if visible_global_indices.size:
            newly_claimed = self.ownership.claim(
                visible_global_indices, drone_index
            )
        return bool(newly_claimed.size)

    def _build_coverage_marker_array(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        z_offset = float(self.get_parameter("coverage_z_offset_m").value)

        for drone_index in range(len(self.drone_ids)):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = f"coverage_drone_{drone_index + 1:02d}"
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.z = z_offset
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.frame_locked = True

            color = [
                float(value)
                for value in self.get_parameter(
                    _COLOR_PARAMETERS[drone_index]
                ).value
            ]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0

            marker.points = []
            for triangle_index in self.ownership.indices_for_drone(
                drone_index
            ):
                for vertex in self.scene.triangle_positions[triangle_index]:
                    point = Point()
                    point.x = float(vertex[0])
                    point.y = float(vertex[1])
                    point.z = float(vertex[2])
                    marker.points.append(point)

            marker_array.markers.append(marker)

        return marker_array

    def _publish_markers(self):
        self.marker_publisher.publish(self._build_coverage_marker_array())

    def _compute_total_area(self):
        if self.scene is None or self.ownership is None:
            return 0.0
        owned_mask = self.ownership.owner_ids >= 0
        return float(np.sum(self.scene.areas[owned_mask]))

    def _publish_area(self):
        message = Float32()
        message.data = self._compute_total_area()
        self.area_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
