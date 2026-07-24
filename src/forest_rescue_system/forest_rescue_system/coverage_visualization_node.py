#!/usr/bin/env python3

"""다중 드론 카메라의 누적 가시영역과 지면 커버리지를 계산한다."""

from functools import partial
import json
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
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from forest_rescue_system import coverage_utils
from forest_rescue_system.log_utils import TimestampedNode


_COLOR_PARAMETERS = (
    "drone_01_color_rgb",
    "drone_02_color_rgb",
    "drone_03_color_rgb",
    "drone_04_color_rgb",
)
_FLASHLIGHT_CONE_ALPHA = 0.15
_FLASHLIGHT_POINT_ALPHA = 0.60
_FLASHLIGHT_LINE_WIDTH_M = 0.03
_FLASHLIGHT_POINT_SIZE_M = 0.10


def _point(vector):
    message = Point()
    message.x = float(vector[0])
    message.y = float(vector[1])
    message.z = float(vector[2])
    return message


def _stamp_to_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class CoverageVisualizationNode(TimestampedNode):
    """레이캐스팅으로 실제 보이는 삼각형을 최초 관측 드론에 누적한다."""

    def __init__(self):
        super().__init__("coverage_visualization_node")

        self.declare_parameter("operation_mode", "eval_coverage")
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
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter(
            "active_coverage_states",
            [
                "EVAL_FORWARD",
                "EVAL_FORWARD_SNAPSHOT",
                "EVAL_REVERSE",
                "EVAL_FINAL_SNAPSHOT",
            ],
        )
        self.declare_parameter(
            "coverage_marker_topic", "/forest_rescue/coverage_markers"
        )
        self.declare_parameter(
            "flashlight_marker_topic", "/forest_rescue/flashlight_markers"
        )
        self.declare_parameter(
            "coverage_progress_topic",
            "/forest_rescue/coverage_progress_percent",
        )
        self.declare_parameter(
            "coverage_area_topic", "/forest_rescue/coverage_area_m2"
        )
        self.declare_parameter(
            "coverage_total_area_topic",
            "/forest_rescue/coverage_total_area_m2",
        )
        self.declare_parameter(
            "coverage_statistics_topic",
            "/forest_rescue/coverage/statistics",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("refresh_period_sec", 0.50)
        self.declare_parameter("statistics_publish_period_sec", 0.50)
        self.declare_parameter("ray_grid_step_px", 4)
        self.declare_parameter("minimum_depth_m", 0.20)
        self.declare_parameter("maximum_depth_m", 30.0)
        self.declare_parameter("coverage_z_offset_m", 0.05)
        self.declare_parameter("flashlight_color_rgb", [1.0, 0.95, 0.70])
        self.declare_parameter("drone_01_color_rgb", [0.55, 0.00, 0.85])
        self.declare_parameter("drone_02_color_rgb", [0.73, 0.33, 0.83])
        self.declare_parameter("drone_03_color_rgb", [0.60, 0.00, 0.50])
        self.declare_parameter("drone_04_color_rgb", [0.90, 0.25, 0.65])
        self.declare_parameter("warning_period_sec", 5.0)

        self.operation_mode = str(
            self.get_parameter("operation_mode").value
        ).strip().lower()
        if self.operation_mode != "eval_coverage":
            raise RuntimeError(
                "coverage_visualization은 operation_mode=eval_coverage에서만 "
                f"실행합니다: {self.operation_mode!r}"
            )

        self.drone_ids = [
            str(value) for value in self.get_parameter("drone_ids").value
        ]
        if not 1 <= len(self.drone_ids) <= len(_COLOR_PARAMETERS):
            raise RuntimeError(
                "커버리지 노드는 드론 1~4대를 지원합니다: "
                f"{self.drone_ids}"
            )
        self.terrain_mesh_path = Path(
            str(self.get_parameter("terrain_mesh_path").value)
        ).expanduser()
        self.environment_mesh_path = Path(
            str(self.get_parameter("environment_mesh_path").value)
        ).expanduser()
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.active_states = {
            str(value).strip().upper()
            for value in self.get_parameter("active_coverage_states").value
        }
        self.warning_period_sec = max(
            0.1, float(self.get_parameter("warning_period_sec").value)
        )

        self.scene = None
        self.raycasting_scene = None
        self.ownership = None
        self.total_floor_area_m2 = 0.0
        self.mission_state = "IDLE"
        self.session_index = 0
        self.last_mesh_wait_wall = float("-inf")
        self.last_warning_wall = {}

        self.camera_info_by_drone = {}
        self.depth_shape_by_drone = {}
        self.depth_stamp_by_drone = {}
        self.flashlight_state = {}
        self._flashlight_published_drones = set()

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
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            self._mission_state_callback,
            10,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.coverage_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("coverage_marker_topic").value),
            marker_qos,
        )

        flashlight_qos = QoSProfile(depth=1)
        flashlight_qos.reliability = ReliabilityPolicy.RELIABLE
        flashlight_qos.durability = DurabilityPolicy.VOLATILE
        self.flashlight_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("flashlight_marker_topic").value),
            flashlight_qos,
        )
        self.progress_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("coverage_progress_topic").value),
            10,
        )
        self.area_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("coverage_area_topic").value),
            10,
        )
        self.total_area_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("coverage_total_area_topic").value),
            10,
        )
        self.statistics_publisher = self.create_publisher(
            String,
            str(self.get_parameter("coverage_statistics_topic").value),
            10,
        )

        self._load_mesh_if_ready()
        self.create_timer(
            max(0.1, float(self.get_parameter("refresh_period_sec").value)),
            self._refresh_coverage,
        )
        self.create_timer(
            max(
                0.1,
                float(
                    self.get_parameter("statistics_publish_period_sec").value
                ),
            ),
            self._publish_statistics,
        )
        self.get_logger().info(
            "커버리지 평가 노드 시작: "
            f"drones={self.drone_ids}, active_states={sorted(self.active_states)}"
        )

    def _camera_info_callback(self, drone_id, message):
        self.camera_info_by_drone[drone_id] = message

    def _depth_callback(self, drone_id, message):
        self.depth_shape_by_drone[drone_id] = (
            int(message.height),
            int(message.width),
        )
        self.depth_stamp_by_drone[drone_id] = message.header.stamp

    def _mission_state_callback(self, message):
        state = str(message.data).strip().upper()
        previous = self.mission_state
        self.mission_state = state

        if state == "EVAL_FORWARD" and previous != "EVAL_FORWARD":
            # READY 동안 도착한 마지막 Depth를 평가 첫 프레임으로 재사용하지 않는다.
            self.depth_stamp_by_drone.clear()
            self._reset_coverage()
            self.get_logger().info("정방향 커버리지 누적을 0%에서 시작합니다.")

        if state not in self.active_states:
            self.flashlight_state.clear()

    def _load_mesh_if_ready(self):
        if self.scene is not None:
            return True
        if (
            not self.terrain_mesh_path.is_file()
            or not self.environment_mesh_path.is_file()
        ):
            wall_now = time.monotonic()
            if wall_now - self.last_mesh_wait_wall >= 5.0:
                self.get_logger().info(
                    "커버리지 Terrain/환경 Mesh 대기 중: "
                    f"{self.terrain_mesh_path}, {self.environment_mesh_path}"
                )
                self.last_mesh_wait_wall = wall_now
            return False

        try:
            groups = {}
            groups.update(
                coverage_utils.load_terrain_group(self.terrain_mesh_path)
            )
            groups.update(
                coverage_utils.load_environment_groups(
                    self.environment_mesh_path
                )
            )
            scene = coverage_utils.assemble_scene(groups)
            if "terrain" not in scene.group_slices:
                raise ValueError("Terrain 그룹을 찾지 못했습니다.")
            raycasting_scene = coverage_utils.build_raycasting_scene(
                scene.triangle_positions
            )
        except (OSError, KeyError, TypeError, ValueError, RuntimeError) as error:
            self._warn_throttled("mesh_load", f"커버리지 Mesh 로드 실패: {error}")
            return False

        self.scene = scene
        self.raycasting_scene = raycasting_scene
        self.ownership = coverage_utils.TriangleOwnership(len(scene.areas))
        terrain_slice = scene.group_slices["terrain"]
        self.total_floor_area_m2 = float(np.sum(scene.areas[terrain_slice]))
        self.get_logger().info(
            "커버리지 레이캐스팅 장면 생성 완료: "
            f"groups={scene.group_names}, triangles={len(scene.areas)}, "
            f"terrain_area={self.total_floor_area_m2:.2f}m²"
        )
        self._publish_coverage_markers()
        self._publish_statistics()
        return True

    def _reset_coverage(self):
        self.session_index += 1
        if self.ownership is not None:
            self.ownership.reset()
        self.flashlight_state.clear()
        self._flashlight_published_drones.clear()
        self._publish_coverage_markers()
        self._publish_flashlight_markers()
        self._publish_statistics()

    def _refresh_coverage(self):
        if not self._load_mesh_if_ready():
            return
        if self.mission_state not in self.active_states:
            self.flashlight_state.clear()
            self._publish_flashlight_markers()
            return

        newly_claimed = False
        for drone_index, drone_id in enumerate(self.drone_ids):
            try:
                newly_claimed |= self._process_drone(drone_index, drone_id)
            except (TypeError, ValueError) as error:
                self._warn_throttled(
                    f"process_{drone_id}",
                    f"{drone_id} 커버리지 계산 입력 오류: {error}",
                )
                self.flashlight_state.pop(drone_id, None)

        if newly_claimed:
            self._publish_coverage_markers()
        self._publish_flashlight_markers()

    def _process_drone(self, drone_index, drone_id):
        camera_info = self.camera_info_by_drone.get(drone_id)
        depth_shape = self.depth_shape_by_drone.get(drone_id)
        depth_stamp = self.depth_stamp_by_drone.get(drone_id)
        if camera_info is None or depth_shape is None or depth_stamp is None:
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
        except TransformException as error:
            self.flashlight_state.pop(drone_id, None)
            self._warn_throttled(
                f"tf_{drone_id}",
                f"{drone_id} Camera→map exact TF 대기 중: {error}",
            )
            return False

        transform = transform_stamped.transform
        matrix = coverage_utils.transform_matrix_from_tf(
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )
        camera_origin = matrix[:3, 3]

        image_height, image_width = depth_shape
        fx, fy, cx, cy = coverage_utils.scaled_intrinsics(
            camera_info.k,
            int(camera_info.width),
            int(camera_info.height),
            image_width,
            image_height,
        )
        grid_u, grid_v = coverage_utils.pixel_grid_uv(
            image_width,
            image_height,
            int(self.get_parameter("ray_grid_step_px").value),
        )
        ray_camera = coverage_utils.pixel_to_camera_ray(
            grid_u, grid_v, fx, fy, cx, cy
        )
        ray_map = coverage_utils.transform_direction(ray_camera, matrix)
        hit_points, triangle_indices = coverage_utils.cast_visibility_rays(
            self.raycasting_scene,
            camera_origin,
            ray_map,
            float(self.get_parameter("minimum_depth_m").value),
            float(self.get_parameter("maximum_depth_m").value),
        )

        corner_u = np.asarray(
            [0.0, float(image_width), float(image_width), 0.0]
        )
        corner_v = np.asarray(
            [0.0, 0.0, float(image_height), float(image_height)]
        )
        corner_map = coverage_utils.transform_direction(
            coverage_utils.pixel_to_camera_ray(
                corner_u, corner_v, fx, fy, cx, cy
            ),
            matrix,
        )
        self.flashlight_state[drone_id] = {
            "origin": camera_origin,
            "corner_directions": corner_map,
            "hit_points": hit_points,
            "stamp_ns": _stamp_to_nanoseconds(depth_stamp),
        }

        newly_claimed = self.ownership.claim(
            triangle_indices,
            drone_index,
        )
        return bool(newly_claimed.size)

    def _coverage_color(self, drone_index):
        values = [
            float(value)
            for value in self.get_parameter(
                _COLOR_PARAMETERS[drone_index]
            ).value
        ]
        if len(values) != 3:
            raise ValueError(
                f"{_COLOR_PARAMETERS[drone_index]}는 RGB 3개여야 합니다."
            )
        return [max(0.0, min(1.0, value)) for value in values]

    def _build_coverage_marker_array(self):
        marker_array = MarkerArray()
        if self.scene is None or self.ownership is None:
            return marker_array
        stamp = self.get_clock().now().to_msg()
        z_offset = float(self.get_parameter("coverage_z_offset_m").value)

        for drone_index, _drone_id in enumerate(self.drone_ids):
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
            red, green, blue = self._coverage_color(drone_index)
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = 1.0
            marker.points = [
                _point(vertex)
                for triangle_index in self.ownership.indices_for_drone(
                    drone_index
                )
                for vertex in self.scene.triangle_positions[triangle_index]
            ]
            marker_array.markers.append(marker)
        return marker_array

    def _publish_coverage_markers(self):
        self.coverage_marker_publisher.publish(
            self._build_coverage_marker_array()
        )

    def _delete_marker(self, namespace, marker_id, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.action = Marker.DELETE
        return marker

    def _build_flashlight_marker_array(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        color = [
            float(value)
            for value in self.get_parameter("flashlight_color_rgb").value
        ]
        if len(color) != 3:
            color = [1.0, 0.95, 0.70]
        max_depth = float(self.get_parameter("maximum_depth_m").value)
        published_drones = set()

        for drone_index, drone_id in enumerate(self.drone_ids):
            namespace = f"flashlight_drone_{drone_index + 1:02d}"
            state = self.flashlight_state.get(drone_id)
            if state is None:
                if drone_id in self._flashlight_published_drones:
                    marker_array.markers.extend(
                        [
                            self._delete_marker(namespace, 0, stamp),
                            self._delete_marker(namespace, 1, stamp),
                        ]
                    )
                continue

            published_drones.add(drone_id)
            origin = state["origin"]
            far_points = (
                origin + state["corner_directions"] * max_depth
            )

            cone = Marker()
            cone.header.frame_id = self.map_frame
            cone.header.stamp = stamp
            cone.ns = namespace
            cone.id = 0
            cone.type = Marker.LINE_LIST
            cone.action = Marker.ADD
            cone.pose.orientation.w = 1.0
            cone.scale.x = _FLASHLIGHT_LINE_WIDTH_M
            cone.color.r = color[0]
            cone.color.g = color[1]
            cone.color.b = color[2]
            cone.color.a = _FLASHLIGHT_CONE_ALPHA
            cone.frame_locked = True
            for far_point in far_points:
                cone.points.extend((_point(origin), _point(far_point)))
            for index in range(4):
                cone.points.extend(
                    (
                        _point(far_points[index]),
                        _point(far_points[(index + 1) % 4]),
                    )
                )
            marker_array.markers.append(cone)

            hits = Marker()
            hits.header.frame_id = self.map_frame
            hits.header.stamp = stamp
            hits.ns = namespace
            hits.id = 1
            hits.type = Marker.POINTS
            hits.action = Marker.ADD
            hits.pose.orientation.w = 1.0
            hits.scale.x = _FLASHLIGHT_POINT_SIZE_M
            hits.scale.y = _FLASHLIGHT_POINT_SIZE_M
            hits.color.r = color[0]
            hits.color.g = color[1]
            hits.color.b = color[2]
            hits.color.a = _FLASHLIGHT_POINT_ALPHA
            hits.frame_locked = True
            hits.points = [_point(value) for value in state["hit_points"]]
            marker_array.markers.append(hits)

        self._flashlight_published_drones = published_drones
        return marker_array

    def _publish_flashlight_markers(self):
        self.flashlight_marker_publisher.publish(
            self._build_flashlight_marker_array()
        )

    def _statistics_payload(self):
        payload = {
            "format_version": 1,
            "operation_mode": self.operation_mode,
            "session_index": int(self.session_index),
            "mission_state": self.mission_state,
            "sim_time_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "drone_ids": list(self.drone_ids),
            "terrain_total_area_m2": float(self.total_floor_area_m2),
            "terrain_covered_area_m2": 0.0,
            "terrain_coverage_percent": 0.0,
            "terrain_total_triangles": 0,
            "terrain_covered_triangles": 0,
            "scene_total_triangles": 0,
            "scene_covered_triangles": 0,
            "per_drone": {},
        }
        if self.scene is None or self.ownership is None:
            return payload

        owner_ids = self.ownership.owner_ids
        terrain_slice = self.scene.group_slices.get("terrain")
        payload["scene_total_triangles"] = int(len(owner_ids))
        payload["scene_covered_triangles"] = int(np.count_nonzero(owner_ids >= 0))
        if terrain_slice is None:
            return payload

        terrain_owners = owner_ids[terrain_slice]
        terrain_areas = self.scene.areas[terrain_slice]
        covered_mask = terrain_owners >= 0
        covered_area = float(np.sum(terrain_areas[covered_mask]))
        percent = (
            covered_area / self.total_floor_area_m2 * 100.0
            if self.total_floor_area_m2 > 0.0
            else 0.0
        )
        payload.update(
            {
                "terrain_covered_area_m2": covered_area,
                "terrain_coverage_percent": percent,
                "terrain_total_triangles": int(len(terrain_owners)),
                "terrain_covered_triangles": int(np.count_nonzero(covered_mask)),
            }
        )

        for drone_index, drone_id in enumerate(self.drone_ids):
            terrain_mask = terrain_owners == drone_index
            scene_mask = owner_ids == drone_index
            payload["per_drone"][drone_id] = {
                "terrain_covered_area_m2": float(
                    np.sum(terrain_areas[terrain_mask])
                ),
                "terrain_covered_triangles": int(
                    np.count_nonzero(terrain_mask)
                ),
                "scene_covered_triangles": int(
                    np.count_nonzero(scene_mask)
                ),
            }
        return payload

    def _publish_statistics(self):
        payload = self._statistics_payload()

        progress = Float32()
        progress.data = float(payload["terrain_coverage_percent"])
        self.progress_publisher.publish(progress)

        area = Float32()
        area.data = float(payload["terrain_covered_area_m2"])
        self.area_publisher.publish(area)

        total_area = Float32()
        total_area.data = float(payload["terrain_total_area_m2"])
        self.total_area_publisher.publish(total_area)

        statistics = String()
        statistics.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.statistics_publisher.publish(statistics)

    def _warn_throttled(self, key, message):
        wall_now = time.monotonic()
        last = self.last_warning_wall.get(key, float("-inf"))
        if wall_now - last < self.warning_period_sec:
            return
        self.last_warning_wall[key] = wall_now
        self.get_logger().warning(message)


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
