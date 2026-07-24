#!/usr/bin/env python3

"""Start, Victim, 산과 식생·바위·강·다리를 RViz에 함께 표시한다."""

from functools import partial
import json
from pathlib import Path
import time

from geometry_msgs.msg import Point, PointStamped
import numpy as np
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray

from forest_rescue_system.log_utils import TimestampedNode


class RvizVisualizationNode(TimestampedNode):
    """실제 스폰 정보와 USD 환경 Mesh를 RViz Marker로 변환한다."""

    def __init__(self):
        super().__init__("rviz_visualization_node")

        self.declare_parameter(
            "ground_truth_path",
            "~/b3_cobot3_ws/isaac_sim/generated_ground_truth.json",
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
            "scene_marker_topic",
            "/forest_rescue/scene_markers",
        )
        self.declare_parameter(
            "terrain_marker_topic",
            "/forest_rescue/terrain_mesh",
        )
        self.declare_parameter(
            "environment_marker_topic",
            "/forest_rescue/environment_meshes",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("operation_mode", "rescue_search")
        self.declare_parameter(
            "drone_ids",
            ["quadrotor_01", "quadrotor_02", "quadrotor_03"],
        )
        self.declare_parameter(
            "drone_marker_topic",
            "/forest_rescue/drone_markers",
        )
        self.declare_parameter("refresh_period_sec", 0.5)
        self.declare_parameter("terrain_color_rgb", [0.10, 0.55, 0.12])
        self.declare_parameter("terrain_alpha", 0.72)
        self.declare_parameter("terrain_z_offset_m", 0.0)

        # 실제 USD 그룹별 RViz 색상과 투명도
        self.declare_parameter(
            "pineforest_color_rgb",
            [0.16, 0.55, 0.18],
        )
        self.declare_parameter("pineforest_alpha", 1.0)
        self.declare_parameter(
            "broadleafforest_color_rgb",
            [0.28, 0.68, 0.22],
        )
        self.declare_parameter("broadleafforest_alpha", 1.0)
        self.declare_parameter(
            "bushes_color_rgb",
            [0.55, 0.76, 0.25],
        )
        self.declare_parameter("bushes_alpha", 1.0)
        self.declare_parameter(
            "rocks_color_rgb",
            [0.56, 0.57, 0.60],
        )
        self.declare_parameter("rocks_alpha", 1.0)
        self.declare_parameter(
            "bridge_color_rgb",
            [0.42, 0.24, 0.08],
        )
        self.declare_parameter("bridge_alpha", 1.0)
        self.declare_parameter("bridge_z_offset_m", 0.02)
        self.declare_parameter(
            "river_color_rgb",
            [0.02, 0.45, 0.95],
        )
        self.declare_parameter("river_alpha", 0.95)
        # Terrain과 수면이 같은 높이일 때 생기는 Z-fighting을 방지한다.
        self.declare_parameter("river_z_offset_m", 0.08)

        self.ground_truth_path = Path(
            str(self.get_parameter("ground_truth_path").value)
        ).expanduser()
        self.terrain_mesh_path = Path(
            str(self.get_parameter("terrain_mesh_path").value)
        ).expanduser()
        self.environment_mesh_path = Path(
            str(self.get_parameter("environment_mesh_path").value)
        ).expanduser()
        self.scene_marker_topic = str(
            self.get_parameter("scene_marker_topic").value
        )
        self.terrain_marker_topic = str(
            self.get_parameter("terrain_marker_topic").value
        )
        self.environment_marker_topic = str(
            self.get_parameter("environment_marker_topic").value
        )
        self.operation_mode = str(
            self.get_parameter("operation_mode").value
        ).strip().lower()
        self.drone_ids = [
            str(value) for value in self.get_parameter("drone_ids").value
        ]
        if not self.drone_ids:
            raise RuntimeError("RViz drone_ids가 비어 있습니다.")
        self.drone_marker_topic = str(
            self.get_parameter("drone_marker_topic").value
        )
        self.drone_home_positions = {}
        self.latest_drone_local_positions = {}

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.scene_publisher = self.create_publisher(
            MarkerArray,
            self.scene_marker_topic,
            qos,
        )
        self.terrain_publisher = self.create_publisher(
            Marker,
            self.terrain_marker_topic,
            qos,
        )
        self.environment_publisher = self.create_publisher(
            MarkerArray,
            self.environment_marker_topic,
            qos,
        )
        self.drone_publisher = self.create_publisher(
            MarkerArray,
            self.drone_marker_topic,
            qos,
        )
        for index, drone_id in enumerate(self.drone_ids, start=1):
            self.create_subscription(
                PointStamped,
                f"/drone_{index:02d}/local_position_ned",
                partial(self._drone_position_callback, drone_id),
                10,
            )

        self.scene_signature = None
        self.terrain_signature = None
        self.environment_signature = None
        self.scene_published = False
        self.terrain_published = False
        self.environment_published = False
        self.last_scene_wait_log_at = float("-inf")
        self.last_terrain_wait_log_at = float("-inf")
        self.last_environment_wait_log_at = float("-inf")

        refresh_period = max(
            0.1,
            float(self.get_parameter("refresh_period_sec").value),
        )
        self.timer = self.create_timer(
            refresh_period,
            self._refresh_visualization,
        )

        # 파일이 이미 만들어져 있다면 spin 시작 전에 즉시 발행한다.
        self._refresh_visualization()

        self.get_logger().info(
            "RViz 시각화 노드 시작: "
            f"scene={self.scene_marker_topic}, "
            f"terrain={self.terrain_marker_topic}, "
            f"environment={self.environment_marker_topic}, "
            f"drones={self.drone_marker_topic}, "
            f"operation_mode={self.operation_mode}, "
            f"drone_ids={self.drone_ids}"
        )

    def _refresh_visualization(self):
        self._refresh_scene_markers()
        self._refresh_drone_markers()
        self._refresh_terrain_marker()
        self._refresh_environment_markers()

    def _refresh_scene_markers(self):
        if not self.ground_truth_path.is_file():
            if self.scene_published:
                self._clear_scene_markers()
                self._clear_drone_markers()
                self.scene_published = False
                self.scene_signature = None
                self.drone_home_positions = {}

            now = time.monotonic()
            if now - self.last_scene_wait_log_at >= 5.0:
                self.get_logger().info(
                    "Isaac Sim 실제 스폰 좌표 파일 대기 중: "
                    f"{self.ground_truth_path}"
                )
                self.last_scene_wait_log_at = now
            return

        try:
            stat = self.ground_truth_path.stat()
            signature = (stat.st_mtime_ns, stat.st_size)
            if signature == self.scene_signature and self.scene_published:
                return

            payload = json.loads(
                self.ground_truth_path.read_text(encoding="utf-8")
            )
            payload_mode = str(
                payload.get("operation_mode", "rescue_search")
            ).strip().lower()
            if payload_mode != self.operation_mode:
                raise ValueError(
                    "Ground Truth와 ROS operation_mode이 다릅니다: "
                    f"ground_truth={payload_mode}, "
                    f"launch={self.operation_mode}"
                )
            marker_array = self._build_scene_marker_array(payload)
        except (
            OSError,
            json.JSONDecodeError,
            KeyError,
            TypeError,
            ValueError,
        ) as error:
            self.get_logger().warning(
                f"Ground Truth 파일 읽기 실패, 다음 주기에 재시도: {error}"
            )
            return

        self.scene_publisher.publish(marker_array)
        self.scene_signature = signature
        self.scene_published = True

        victim = payload.get("victim")
        victim_text = "NONE"
        if isinstance(victim, dict) and "world_enu" in victim:
            position = victim["world_enu"]
            victim_text = (
                f"({position[0]:.2f}, "
                f"{position[1]:.2f}, {position[2]:.2f})"
            )
        self.get_logger().info(
            "RViz Scene Marker 발행: "
            f"operation_mode={self.operation_mode}, "
            f"Start={len(self.drone_home_positions)}, "
            f"Victim={victim_text}"
        )

    def _refresh_terrain_marker(self):
        if not self.terrain_mesh_path.is_file():
            if self.terrain_published:
                self._delete_terrain_marker()
                self.terrain_published = False
                self.terrain_signature = None

            now = time.monotonic()
            if now - self.last_terrain_wait_log_at >= 5.0:
                self.get_logger().info(
                    "Isaac Sim Terrain Mesh 파일 대기 중: "
                    f"{self.terrain_mesh_path}"
                )
                self.last_terrain_wait_log_at = now
            return

        try:
            stat = self.terrain_mesh_path.stat()
            signature = (stat.st_mtime_ns, stat.st_size)
            if signature == self.terrain_signature and self.terrain_published:
                return

            marker, vertex_count, triangle_count = (
                self._load_terrain_marker()
            )
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
        ) as error:
            self.get_logger().warning(
                f"Terrain Mesh 읽기 실패, 다음 주기에 재시도: {error}"
            )
            return

        self.terrain_publisher.publish(marker)
        self.terrain_signature = signature
        self.terrain_published = True
        self.get_logger().info(
            "RViz 초록색 Terrain 발행: "
            f"vertices={vertex_count}, triangles={triangle_count}"
        )

    def _refresh_environment_markers(self):
        if not self.environment_mesh_path.is_file():
            if self.environment_published:
                self._clear_environment_markers()
                self.environment_published = False
                self.environment_signature = None

            now = time.monotonic()
            if now - self.last_environment_wait_log_at >= 5.0:
                self.get_logger().info(
                    "Isaac Sim 환경 그룹 Mesh 파일 대기 중: "
                    f"{self.environment_mesh_path}"
                )
                self.last_environment_wait_log_at = now
            return

        try:
            stat = self.environment_mesh_path.stat()
            signature = (stat.st_mtime_ns, stat.st_size)
            if (
                signature == self.environment_signature
                and self.environment_published
            ):
                return

            marker_array, summaries = (
                self._load_environment_marker_array()
            )
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
        ) as error:
            self.get_logger().warning(
                "환경 그룹 Mesh 읽기 실패, 다음 주기에 재시도: "
                f"{error}"
            )
            return

        self.environment_publisher.publish(marker_array)
        self.environment_signature = signature
        self.environment_published = True

        summary_text = ", ".join(
            f"{name}={triangle_count}"
            for name, triangle_count in summaries.items()
        )
        self.get_logger().info(
            f"RViz 환경 그룹 Marker 발행: {summary_text}"
        )

    def _load_environment_marker_array(self):
        group_specs = (
            (
                "pineforest",
                "PineForest",
                "pineforest_color_rgb",
                "pineforest_alpha",
            ),
            (
                "broadleafforest",
                "BroadleafForest",
                "broadleafforest_color_rgb",
                "broadleafforest_alpha",
            ),
            (
                "bushes",
                "Bushes",
                "bushes_color_rgb",
                "bushes_alpha",
            ),
            (
                "rocks",
                "Rocks",
                "rocks_color_rgb",
                "rocks_alpha",
            ),
            (
                "bridges",
                "Bridges",
                "bridge_color_rgb",
                "bridge_alpha",
            ),
            (
                "river",
                "River",
                "river_color_rgb",
                "river_alpha",
            ),
        )

        marker_array = MarkerArray()
        summaries = {}

        with np.load(
            self.environment_mesh_path,
            allow_pickle=False,
        ) as data:
            map_frame_value = data.get("map_frame")
            if map_frame_value is not None and len(map_frame_value):
                map_frame = str(map_frame_value[0])
            else:
                map_frame = str(
                    self.get_parameter("map_frame").value
                )

            stamp = self.get_clock().now().to_msg()

            for marker_id, (
                key,
                display_name,
                color_parameter,
                alpha_parameter,
            ) in enumerate(group_specs):
                vertices_value = data.get(f"{key}_vertices")
                triangles_value = data.get(f"{key}_triangles")
                # 이전 버전 NPZ에는 river 키가 없을 수 있다. Isaac Sim을
                # 다시 실행해 파일이 갱신될 때까지 기존 그룹은 정상 표시한다.
                if vertices_value is None or triangles_value is None:
                    summaries[display_name] = 0
                    continue

                vertices = np.asarray(
                    vertices_value,
                    dtype=np.float64,
                )
                triangles = np.asarray(
                    triangles_value,
                    dtype=np.int64,
                )

                if vertices.size == 0 or triangles.size == 0:
                    summaries[display_name] = 0
                    continue

                self._validate_triangle_mesh(
                    vertices,
                    triangles,
                    display_name,
                )

                marker = Marker()
                marker.header.frame_id = map_frame
                marker.header.stamp = stamp
                marker.ns = key
                marker.id = marker_id
                marker.type = Marker.TRIANGLE_LIST
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                if key == "river":
                    marker.pose.position.z = float(
                        self.get_parameter("river_z_offset_m").value
                    )
                elif key == "bridges":
                    marker.pose.position.z = float(
                        self.get_parameter("bridge_z_offset_m").value
                    )
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.frame_locked = True

                color = [
                    float(value)
                    for value in self.get_parameter(
                        color_parameter
                    ).value
                ]
                if len(color) != 3:
                    raise ValueError(
                        f"{color_parameter}는 RGB 3개여야 합니다."
                    )
                marker.color.r = max(0.0, min(1.0, color[0]))
                marker.color.g = max(0.0, min(1.0, color[1]))
                marker.color.b = max(0.0, min(1.0, color[2]))
                marker.color.a = max(
                    0.05,
                    min(
                        1.0,
                        float(
                            self.get_parameter(
                                alpha_parameter
                            ).value
                        ),
                    ),
                )

                marker.points = []

                # RViz의 TRIANGLE_LIST는 면의 정점 순서에 따라 한쪽 면만
                # 보일 수 있다. River 원본 Mesh의 면 방향이 아래쪽을 향하면
                # 아래에서는 파랗지만 위에서는 지형에 가려진 것처럼 보인다.
                # 강에만 역방향 면을 하나 더 만들어 위·아래 양쪽에서 표시한다.
                render_triangles = triangles
                if key == "river":
                    reverse_triangles = triangles[:, [0, 2, 1]]
                    render_triangles = np.concatenate(
                        (triangles, reverse_triangles),
                        axis=0,
                    )

                marker.points.extend(
                    self._point_from_vertex(vertices[index])
                    for triangle in render_triangles
                    for index in triangle
                )

                marker_array.markers.append(marker)
                summaries[display_name] = len(triangles)

        if not marker_array.markers:
            raise ValueError(
                "환경 그룹 파일에 표시 가능한 Mesh가 없습니다."
            )

        return marker_array, summaries

    @staticmethod
    def _validate_triangle_mesh(vertices, triangles, name):
        if (
            vertices.ndim != 2
            or vertices.shape[1] != 3
            or not np.all(np.isfinite(vertices))
        ):
            raise ValueError(
                f"{name} vertices 형식이 잘못됐습니다: "
                f"{vertices.shape}"
            )
        if (
            triangles.ndim != 2
            or triangles.shape[1] != 3
            or triangles.size == 0
        ):
            raise ValueError(
                f"{name} triangles 형식이 잘못됐습니다: "
                f"{triangles.shape}"
            )
        if np.min(triangles) < 0 or np.max(triangles) >= len(vertices):
            raise ValueError(
                f"{name} triangle index가 정점 범위를 벗어났습니다."
            )

    def _load_terrain_marker(self):
        with np.load(
            self.terrain_mesh_path,
            allow_pickle=False,
        ) as data:
            vertices = np.asarray(data["vertices"], dtype=np.float64)
            triangles = np.asarray(data["triangles"], dtype=np.int64)
            map_frame_value = data.get("map_frame")

            if map_frame_value is not None and len(map_frame_value):
                map_frame = str(map_frame_value[0])
            else:
                map_frame = str(
                    self.get_parameter("map_frame").value
                )

        self._validate_triangle_mesh(
            vertices,
            triangles,
            "Terrain",
        )

        marker = Marker()
        marker.header.frame_id = map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "forest_terrain"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = float(
            self.get_parameter("terrain_z_offset_m").value
        )
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.frame_locked = True

        color = [
            float(value)
            for value in self.get_parameter("terrain_color_rgb").value
        ]
        if len(color) != 3:
            raise ValueError("terrain_color_rgb는 RGB 값 3개여야 합니다.")
        marker.color.r = max(0.0, min(1.0, color[0]))
        marker.color.g = max(0.0, min(1.0, color[1]))
        marker.color.b = max(0.0, min(1.0, color[2]))
        marker.color.a = max(
            0.05,
            min(
                1.0,
                float(self.get_parameter("terrain_alpha").value),
            ),
        )

        # TRIANGLE_LIST는 점 3개를 순서대로 묶어 한 면으로 그린다.
        marker.points = []
        marker.points.extend(
            self._point_from_vertex(vertices[index])
            for triangle in triangles
            for index in triangle
        )

        return marker, len(vertices), len(triangles)

    def _build_scene_marker_array(self, payload):
        """현재 N대의 시작점과 실제 조난자 Marker를 만든다."""
        map_frame = str(
            payload.get(
                "map_frame",
                self.get_parameter("map_frame").value,
            )
        )
        drone_starts = payload["drone_starts"]
        victim = payload.get("victim")

        if not isinstance(drone_starts, list) or not drone_starts:
            raise ValueError("drone_starts 목록이 비어 있습니다.")

        start_by_id = {
            str(item.get("drone_id", "")): item
            for item in drone_starts
            if isinstance(item, dict)
        }
        missing = [
            drone_id for drone_id in self.drone_ids
            if drone_id not in start_by_id
        ]
        if missing:
            raise ValueError(
                f"Ground Truth에 현재 함대 시작점이 없습니다: {missing}"
            )

        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        delete_all = Marker()
        delete_all.header.frame_id = map_frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        self.drone_home_positions = {}
        palette = (
            (1.00, 0.20, 0.20),
            (0.20, 1.00, 0.25),
            (0.20, 0.45, 1.00),
            (1.00, 0.75, 0.10),
        )
        for index, drone_id in enumerate(self.drone_ids):
            start_item = start_by_id[drone_id]
            start_position = self._read_position(start_item["world_enu"])
            self.drone_home_positions[drone_id] = start_position
            red, green, blue = palette[index % len(palette)]
            base_id = index * 10

            start_area = self._base_marker(
                map_frame,
                stamp,
                namespace="drone_start_areas",
                marker_id=base_id,
                marker_type=Marker.CYLINDER,
            )
            start_area.pose.position.x = start_position[0]
            start_area.pose.position.y = start_position[1]
            start_area.pose.position.z = start_position[2] + 0.08
            start_area.scale.x = 3.0
            start_area.scale.y = 3.0
            start_area.scale.z = 0.16
            start_area.color.r = red
            start_area.color.g = green
            start_area.color.b = blue
            start_area.color.a = 0.45
            markers.markers.append(start_area)

            start_label = self._base_marker(
                map_frame,
                stamp,
                namespace="drone_start_labels",
                marker_id=base_id + 1,
                marker_type=Marker.TEXT_VIEW_FACING,
            )
            start_label.pose.position.x = start_position[0]
            start_label.pose.position.y = start_position[1]
            start_label.pose.position.z = start_position[2] + 2.2
            start_label.scale.z = 1.2
            start_label.color.r = red
            start_label.color.g = green
            start_label.color.b = blue
            start_label.color.a = 1.0
            start_label.text = f"Start {index + 1:02d}"
            markers.markers.append(start_label)

            markers.markers.append(
                self._vertical_line_marker(
                    map_frame=map_frame,
                    stamp=stamp,
                    namespace="drone_start_height_guides",
                    marker_id=base_id + 2,
                    x=start_position[0],
                    y=start_position[1],
                    z_top=start_position[2],
                    red=red,
                    green=green,
                    blue=blue,
                )
            )

        if victim is not None:
            if not isinstance(victim, dict) or "world_enu" not in victim:
                raise ValueError(f"victim 형식이 잘못됐습니다: {victim}")
            victim_position = self._read_position(victim["world_enu"])
            victim_x, victim_y, victim_z = victim_position

            head = self._base_marker(
                map_frame,
                stamp,
                namespace="victim_human",
                marker_id=100,
                marker_type=Marker.SPHERE,
            )
            head.pose.position.x = victim_x
            head.pose.position.y = victim_y
            head.pose.position.z = victim_z + 1.75
            head.scale.x = 0.70
            head.scale.y = 0.70
            head.scale.z = 0.70
            head.color.r = 1.0
            head.color.g = 0.28
            head.color.b = 0.18
            head.color.a = 1.0
            markers.markers.append(head)

            torso = self._base_marker(
                map_frame,
                stamp,
                namespace="victim_human",
                marker_id=101,
                marker_type=Marker.CYLINDER,
            )
            torso.pose.position.x = victim_x
            torso.pose.position.y = victim_y
            torso.pose.position.z = victim_z + 1.02
            torso.scale.x = 0.85
            torso.scale.y = 0.85
            torso.scale.z = 1.35
            torso.color.r = 1.0
            torso.color.g = 0.06
            torso.color.b = 0.06
            torso.color.a = 1.0
            markers.markers.append(torso)

            limbs = self._base_marker(
                map_frame,
                stamp,
                namespace="victim_human",
                marker_id=102,
                marker_type=Marker.LINE_LIST,
            )
            limbs.scale.x = 0.24
            limbs.color.r = 1.0
            limbs.color.g = 0.06
            limbs.color.b = 0.06
            limbs.color.a = 1.0
            shoulder_z = victim_z + 1.35
            hip_z = victim_z + 0.55
            limbs.points = [
                self._make_point(victim_x, victim_y, shoulder_z),
                self._make_point(victim_x - 0.70, victim_y, victim_z + 0.85),
                self._make_point(victim_x, victim_y, shoulder_z),
                self._make_point(victim_x + 0.70, victim_y, victim_z + 0.85),
                self._make_point(victim_x - 0.16, victim_y, hip_z),
                self._make_point(victim_x - 0.35, victim_y, victim_z),
                self._make_point(victim_x + 0.16, victim_y, hip_z),
                self._make_point(victim_x + 0.35, victim_y, victim_z),
            ]
            markers.markers.append(limbs)

            victim_label = self._base_marker(
                map_frame,
                stamp,
                namespace="scene_labels",
                marker_id=103,
                marker_type=Marker.TEXT_VIEW_FACING,
            )
            victim_label.pose.position.x = victim_x
            victim_label.pose.position.y = victim_y
            victim_label.pose.position.z = victim_z + 9.5
            victim_label.scale.z = 2.2
            victim_label.color.r = 1.0
            victim_label.color.g = 0.20
            victim_label.color.b = 0.20
            victim_label.color.a = 1.0
            victim_label.text = "Victim"
            markers.markers.append(victim_label)

            markers.markers.append(
                self._vertical_line_marker(
                    map_frame=map_frame,
                    stamp=stamp,
                    namespace="height_guides",
                    marker_id=104,
                    x=victim_x,
                    y=victim_y,
                    z_top=victim_z,
                    red=1.0,
                    green=0.10,
                    blue=0.10,
                )
            )

            victim_beacon_line = self._base_marker(
                map_frame,
                stamp,
                namespace="victim_beacon",
                marker_id=105,
                marker_type=Marker.LINE_LIST,
            )
            victim_beacon_line.scale.x = 0.28
            victim_beacon_line.color.r = 1.0
            victim_beacon_line.color.g = 0.05
            victim_beacon_line.color.b = 0.05
            victim_beacon_line.color.a = 1.0
            victim_beacon_line.points = [
                self._make_point(victim_x, victim_y, victim_z + 1.8),
                self._make_point(victim_x, victim_y, victim_z + 8.0),
            ]
            markers.markers.append(victim_beacon_line)

            victim_beacon = self._base_marker(
                map_frame,
                stamp,
                namespace="victim_beacon",
                marker_id=106,
                marker_type=Marker.SPHERE,
            )
            victim_beacon.pose.position.x = victim_x
            victim_beacon.pose.position.y = victim_y
            victim_beacon.pose.position.z = victim_z + 8.0
            victim_beacon.scale.x = 1.6
            victim_beacon.scale.y = 1.6
            victim_beacon.scale.z = 1.6
            victim_beacon.color.r = 1.0
            victim_beacon.color.g = 0.08
            victim_beacon.color.b = 0.05
            victim_beacon.color.a = 1.0
            markers.markers.append(victim_beacon)

        return markers

    def _drone_position_callback(self, drone_id, message):
        self.latest_drone_local_positions[drone_id] = message

    def _refresh_drone_markers(self):
        """현재 함대 N대의 실시간 위치를 큰 드론 형태로 표시한다."""
        if not self.drone_home_positions:
            return

        marker_array = MarkerArray()
        map_frame = str(self.get_parameter("map_frame").value)
        stamp = self.get_clock().now().to_msg()
        delete_all = Marker()
        delete_all.header.frame_id = map_frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        palette = (
            (1.00, 0.20, 0.20),
            (0.20, 1.00, 0.25),
            (0.20, 0.45, 1.00),
            (1.00, 0.75, 0.10),
        )
        for index, drone_id in enumerate(self.drone_ids):
            home = self.drone_home_positions.get(drone_id)
            if home is None:
                continue
            local = self.latest_drone_local_positions.get(drone_id)
            if local is None:
                world_x, world_y, world_z = home
            else:
                world_x = home[0] + float(local.point.y)
                world_y = home[1] + float(local.point.x)
                world_z = home[2] - float(local.point.z)

            red, green, blue = palette[index % len(palette)]
            base_id = index * 20

            body = self._base_marker(
                map_frame,
                stamp,
                namespace="live_drones",
                marker_id=base_id,
                marker_type=Marker.CUBE,
            )
            body.pose.position.x = world_x
            body.pose.position.y = world_y
            body.pose.position.z = world_z
            body.scale.x = 1.20
            body.scale.y = 0.75
            body.scale.z = 0.30
            body.color.r = red
            body.color.g = green
            body.color.b = blue
            body.color.a = 1.0
            marker_array.markers.append(body)

            arms = self._base_marker(
                map_frame,
                stamp,
                namespace="live_drone_arms",
                marker_id=base_id + 1,
                marker_type=Marker.LINE_LIST,
            )
            arms.scale.x = 0.16
            arms.color.r = red
            arms.color.g = green
            arms.color.b = blue
            arms.color.a = 1.0
            arms.points = [
                self._make_point(world_x - 0.75, world_y - 0.75, world_z),
                self._make_point(world_x + 0.75, world_y + 0.75, world_z),
                self._make_point(world_x - 0.75, world_y + 0.75, world_z),
                self._make_point(world_x + 0.75, world_y - 0.75, world_z),
            ]
            marker_array.markers.append(arms)

            for rotor_index, (offset_x, offset_y) in enumerate(
                ((-0.75, -0.75), (-0.75, 0.75), (0.75, -0.75), (0.75, 0.75))
            ):
                rotor = self._base_marker(
                    map_frame,
                    stamp,
                    namespace="live_drone_rotors",
                    marker_id=base_id + 2 + rotor_index,
                    marker_type=Marker.CYLINDER,
                )
                rotor.pose.position.x = world_x + offset_x
                rotor.pose.position.y = world_y + offset_y
                rotor.pose.position.z = world_z + 0.05
                rotor.scale.x = 0.48
                rotor.scale.y = 0.48
                rotor.scale.z = 0.08
                rotor.color.r = 0.10
                rotor.color.g = 0.10
                rotor.color.b = 0.10
                rotor.color.a = 1.0
                marker_array.markers.append(rotor)

            label = self._base_marker(
                map_frame,
                stamp,
                namespace="live_drone_labels",
                marker_id=base_id + 10,
                marker_type=Marker.TEXT_VIEW_FACING,
            )
            label.pose.position.x = world_x
            label.pose.position.y = world_y
            label.pose.position.z = world_z + 1.8
            label.scale.z = 1.0
            label.color.r = red
            label.color.g = green
            label.color.b = blue
            label.color.a = 1.0
            label.text = drone_id
            marker_array.markers.append(label)

        self.drone_publisher.publish(marker_array)

    def _vertical_line_marker(
        self,
        map_frame,
        stamp,
        namespace,
        marker_id,
        x,
        y,
        z_top,
        red,
        green,
        blue,
    ):
        """map Z=0부터 실제 위치 Z까지 수직 가이드 라인을 만든다."""
        line = self._base_marker(
            map_frame,
            stamp,
            namespace=namespace,
            marker_id=marker_id,
            marker_type=Marker.LINE_LIST,
        )
        line.scale.x = 0.12
        line.color.r = float(red)
        line.color.g = float(green)
        line.color.b = float(blue)
        line.color.a = 0.85
        line.points = [
            self._make_point(x, y, 0.0),
            self._make_point(x, y, z_top),
        ]
        return line

    @staticmethod
    def _point_from_vertex(vertex):
        return RvizVisualizationNode._make_point(
            vertex[0],
            vertex[1],
            vertex[2],
        )

    @staticmethod
    def _make_point(x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    @staticmethod
    def _read_position(values):
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise ValueError(f"world_enu 좌표 형식이 잘못됐습니다: {values}")
        return tuple(float(value) for value in values)

    @staticmethod
    def _base_marker(
        frame_id,
        stamp,
        namespace,
        marker_id,
        marker_type,
    ):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = int(marker_type)
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        return marker

    def _clear_scene_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = str(
            self.get_parameter("map_frame").value
        )
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.scene_publisher.publish(marker_array)

    def _clear_drone_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = str(
            self.get_parameter("map_frame").value
        )
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.drone_publisher.publish(marker_array)

    def _clear_environment_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = str(
            self.get_parameter("map_frame").value
        )
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.environment_publisher.publish(marker_array)

    def _delete_terrain_marker(self):
        marker = Marker()
        marker.header.frame_id = str(
            self.get_parameter("map_frame").value
        )
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "forest_terrain"
        marker.id = 0
        marker.action = Marker.DELETE
        self.terrain_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RvizVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
