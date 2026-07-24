#!/usr/bin/env python3

"""드론 3대가 depth로 실제 확인한 지형/식생을 누적 표시한다."""

from functools import partial
from pathlib import Path
import time

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

_FLASHLIGHT_CONE_ALPHA = 0.15
_FLASHLIGHT_POINT_ALPHA = 0.6
_FLASHLIGHT_LINE_WIDTH_M = 0.02
_FLASHLIGHT_POINT_SIZE_M = 0.1


def _to_point(vector):
    point = Point()
    point.x = float(vector[0])
    point.y = float(vector[1])
    point.z = float(vector[2])
    return point


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
            "coverage_progress_topic",
            "/forest_rescue/coverage_progress_percent",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("refresh_period_sec", 1.0)
        self.declare_parameter("area_publish_period_sec", 1.0)
        self.declare_parameter("ray_grid_step_px", 4)
        self.declare_parameter(
            "flashlight_marker_topic", "/forest_rescue/flashlight_markers"
        )
        self.declare_parameter("flashlight_color_rgb", [1.0, 0.95, 0.7])
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
        self.raycasting_scene = None
        self.flashlight_state = {}
        self._flashlight_published_drones = set()
        self.ownership = None
        self.total_floor_area_m2 = 0.0
        self.last_mesh_wait_log_at = float("-inf")

        self.camera_info_by_drone = {}
        self.depth_shape_by_drone = {}
        self.depth_stamp_by_drone = {}

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
        self.progress_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("coverage_progress_topic").value),
            10,
        )

        flashlight_qos = QoSProfile(depth=1)
        flashlight_qos.reliability = ReliabilityPolicy.RELIABLE
        flashlight_qos.durability = DurabilityPolicy.VOLATILE
        self.flashlight_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("flashlight_marker_topic").value),
            flashlight_qos,
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
        self.progress_timer = self.create_timer(
            area_period, self._publish_progress
        )

        self.get_logger().info("커버리지 시각화 노드 시작")

    def _camera_info_callback(self, drone_id, message):
        self.camera_info_by_drone[drone_id] = message

    def _depth_callback(self, drone_id, message):
        self.depth_shape_by_drone[drone_id] = (
            int(message.height), int(message.width)
        )
        self.depth_stamp_by_drone[drone_id] = message.header.stamp

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
        self.raycasting_scene = coverage_geometry.build_raycasting_scene(
            self.scene.triangle_positions
        )
        self.ownership = TriangleOwnership(len(self.scene.centroids))
        terrain_slice = self.scene.group_slices.get("terrain")
        if terrain_slice is not None:
            self.total_floor_area_m2 = float(
                np.sum(self.scene.areas[terrain_slice])
            )
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
        self._publish_flashlight_markers()

    def _process_drone(self, drone_index, drone_id):
        camera_info = self.camera_info_by_drone.get(drone_id)
        depth_shape = self.depth_shape_by_drone.get(drone_id)
        depth_stamp = self.depth_stamp_by_drone.get(drone_id)
        if camera_info is None or depth_shape is None or depth_stamp is None:
            self.flashlight_state.pop(drone_id, None)
            return False
        if self.raycasting_scene is None:
            self.flashlight_state.pop(drone_id, None)
            return False

        camera_frame = f"{drone_id}/camera_optical_frame"
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                camera_frame,
                Time.from_msg(depth_stamp),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            self.flashlight_state.pop(drone_id, None)
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
        camera_origin = matrix[:3, 3]

        depth_height, depth_width = depth_shape
        info_width = int(camera_info.width) or depth_width
        info_height = int(camera_info.height) or depth_height
        fx, fy, cx, cy = coverage_geometry.scaled_intrinsics(
            camera_info.k, info_width, info_height, depth_width, depth_height
        )

        grid_u, grid_v = coverage_geometry.pixel_grid_uv(
            depth_width,
            depth_height,
            int(self.get_parameter("ray_grid_step_px").value),
        )
        ray_directions_camera = coverage_geometry.pixel_to_camera_ray(
            grid_u, grid_v, fx, fy, cx, cy
        )
        ray_directions_map = coverage_geometry.transform_direction(
            ray_directions_camera, matrix
        )

        hit_points, triangle_indices = coverage_geometry.cast_visibility_rays(
            self.raycasting_scene,
            camera_origin,
            ray_directions_map,
            float(self.get_parameter("minimum_depth_m").value),
            float(self.get_parameter("maximum_depth_m").value),
        )

        corner_u = np.array([0.0, float(depth_width), float(depth_width), 0.0])
        corner_v = np.array(
            [0.0, 0.0, float(depth_height), float(depth_height)]
        )
        corner_directions_camera = coverage_geometry.pixel_to_camera_ray(
            corner_u, corner_v, fx, fy, cx, cy
        )
        corner_directions_map = coverage_geometry.transform_direction(
            corner_directions_camera, matrix
        )
        self.flashlight_state[drone_id] = {
            "origin": camera_origin,
            "corner_directions": corner_directions_map,
            "hit_points": hit_points,
        }

        newly_claimed = np.asarray([], dtype=np.int64)
        if triangle_indices.size:
            newly_claimed = self.ownership.claim(
                np.unique(triangle_indices), drone_index
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

    def _delete_marker(self, ns, marker_id, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _build_flashlight_marker_array(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        color = [
            float(value)
            for value in self.get_parameter("flashlight_color_rgb").value
        ]
        max_depth = float(self.get_parameter("maximum_depth_m").value)

        published_drones = set()
        for drone_index, drone_id in enumerate(self.drone_ids):
            ns = f"flashlight_drone_{drone_index + 1:02d}"
            state = self.flashlight_state.get(drone_id)
            if state is None:
                if drone_id in self._flashlight_published_drones:
                    marker_array.markers.append(
                        self._delete_marker(ns, 0, stamp)
                    )
                    marker_array.markers.append(
                        self._delete_marker(ns, 1, stamp)
                    )
                continue

            published_drones.add(drone_id)
            origin = state["origin"]
            far_points = origin + state["corner_directions"] * max_depth

            cone_marker = Marker()
            cone_marker.header.frame_id = self.map_frame
            cone_marker.header.stamp = stamp
            cone_marker.ns = ns
            cone_marker.id = 0
            cone_marker.type = Marker.LINE_LIST
            cone_marker.action = Marker.ADD
            cone_marker.pose.orientation.w = 1.0
            cone_marker.scale.x = _FLASHLIGHT_LINE_WIDTH_M
            cone_marker.color.r = color[0]
            cone_marker.color.g = color[1]
            cone_marker.color.b = color[2]
            cone_marker.color.a = _FLASHLIGHT_CONE_ALPHA
            cone_marker.frame_locked = True
            cone_marker.points = []
            for far_point in far_points:
                cone_marker.points.append(_to_point(origin))
                cone_marker.points.append(_to_point(far_point))
            for i in range(4):
                cone_marker.points.append(_to_point(far_points[i]))
                cone_marker.points.append(
                    _to_point(far_points[(i + 1) % 4])
                )
            marker_array.markers.append(cone_marker)

            hit_marker = Marker()
            hit_marker.header.frame_id = self.map_frame
            hit_marker.header.stamp = stamp
            hit_marker.ns = ns
            hit_marker.id = 1
            hit_marker.type = Marker.POINTS
            hit_marker.action = Marker.ADD
            hit_marker.pose.orientation.w = 1.0
            hit_marker.scale.x = _FLASHLIGHT_POINT_SIZE_M
            hit_marker.scale.y = _FLASHLIGHT_POINT_SIZE_M
            hit_marker.color.r = color[0]
            hit_marker.color.g = color[1]
            hit_marker.color.b = color[2]
            hit_marker.color.a = _FLASHLIGHT_POINT_ALPHA
            hit_marker.frame_locked = True
            hit_marker.points = [
                _to_point(point) for point in state["hit_points"]
            ]
            marker_array.markers.append(hit_marker)

        self._flashlight_published_drones = published_drones
        return marker_array

    def _publish_flashlight_markers(self):
        self.flashlight_marker_publisher.publish(
            self._build_flashlight_marker_array()
        )

    def _compute_floor_coverage_percent(self):
        if self.scene is None or self.ownership is None:
            return 0.0
        if self.total_floor_area_m2 <= 0.0:
            return 0.0
        terrain_slice = self.scene.group_slices.get("terrain")
        if terrain_slice is None:
            return 0.0
        owned_mask = self.ownership.owner_ids[terrain_slice] >= 0
        covered_floor_area_m2 = float(
            np.sum(self.scene.areas[terrain_slice][owned_mask])
        )
        return covered_floor_area_m2 / self.total_floor_area_m2 * 100.0

    def _publish_progress(self):
        percent = self._compute_floor_coverage_percent()
        message = Float32()
        message.data = percent
        self.progress_publisher.publish(message)
        print(f"[커버리지 진행도] {percent:.1f}%")


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
