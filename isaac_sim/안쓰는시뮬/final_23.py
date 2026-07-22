#!/usr/bin/env python3

"""
산림 조난자 탐지 드론 시뮬레이션.

변경 사항:
- 지정된 3개 조난자 좌표 중 1곳을 무작위로 선택해 조난자 1명을 정지 상태로 생성한다.
- 시작 좌표 주변에 Iris 드론 3대를 5 m 간격으로 생성한다.
- 각 드론의 기본 카메라를 유지하고 초점거리를 10 mm로 설정한다.
- 세 카메라의 드론 body 기준 상대 위치를 모두 (0, 0, 0)으로 맞춘다.
- 드론 무리 옆 산 표면에 구조자 1명을 정지 상태로 생성한다.
- 조난자와 구조자에 캡슐형 물리 충돌 프록시를 적용한다.
- 사람이 걸을 때 물리 충돌 프록시가 실제 캐릭터 위치를 따라가도록 동기화한다.
- 첫 번째 드론도 quadrotor_01 이름을 사용한다.
- 드론 시작점과 실제 조난자 스폰 좌표를 RViz용 JSON으로 저장한다.
- 실제 USD Terrain 높이를 RViz용 삼각형 메시로 저장한다.
- PineForest, BroadleafForest, Bushes, Rocks의 실제 Mesh도 RViz용으로 저장한다.
- 착륙 시험 시 지정된 XY 위치에 조난자를 고정 배치한다.
"""

import json
import math
import os
from pathlib import Path

import carb
import numpy as np
from isaacsim import SimulationApp


# 센서 및 시나리오 기본 설정값
CAMERA_FOCAL_LENGTH_MM = 10.0
CAMERA_DOWN_TILT_DEG = 30.0
CAMERA_RESOLUTION = [640, 480]

# 왼쪽 메인 Viewport가 따라갈 드론과 3인칭 추적 카메라 설정이다.
# 추적 대상을 바꾸려면 quadrotor_01을 quadrotor_02 또는 03으로 변경한다.
FOLLOW_DRONE_PRIM_PATH = "/World/quadrotor_01/body"
FOLLOW_CAMERA_PRIM_PATH = "/World/FollowCamera"

# 드론의 실제 진행방향을 기준으로 카메라를 뒤쪽·위쪽에 배치한다.
# 카메라는 드론보다 앞쪽 지점을 바라보므로 비행 진행방향이 화면에 보인다.
FOLLOW_CAMERA_BACK_DISTANCE_M = 12.0
FOLLOW_CAMERA_HEIGHT_M = 7.0
FOLLOW_CAMERA_LOOK_AHEAD_M = 10.0
FOLLOW_CAMERA_TARGET_HEIGHT_M = -1.0

# 위치 변화가 이 값보다 클 때만 실제 이동방향을 새로 계산한다.
# 정지 중에는 드론 body의 전방축을 사용한다.
FOLLOW_CAMERA_MIN_MOVEMENT_M = 0.01

# 방향 변화가 너무 급하게 화면에 반영되지 않도록 보간한다.
# 0에 가까울수록 부드럽고, 1에 가까울수록 즉시 방향이 바뀐다.
FOLLOW_CAMERA_DIRECTION_SMOOTHING = 0.15

# 시작 직후 서로의 LiDAR costmap을 점유하지 않도록 5 m 간격으로 배치한다.
DRONE_CONFIGS = [
    ("/World/quadrotor_01", 0, [-34.0, 40.0, 31.0]),
    ("/World/quadrotor_02", 1, [-29.0, 40.0, 31.0]),
    ("/World/quadrotor_03", 2, [-39.0, 40.0, 31.0]),
]

CAMERA_PRIM_PATHS = [
    f"{prim_path}/body/Camera"
    for prim_path, _, _ in DRONE_CONFIGS
]

# 일반 실행에서는 아래 후보 좌표 중 한 곳에 조난자를 생성한다.
VICTIM_SPAWN_POSITIONS = [
    [33.0, 29.0, 13.7],
    # [-0.9, -1.8, -0.9],
    # [33.0, -22.0, 50.6],
]

# 착륙 복귀 시험용 조난자 위치다.
# XY는 보기 쉬운 곳에 고정하고, Z만 실제 Terrain 높이에서 계산한다.
FOR_TEST_VICTIM_SPAWN_ENABLED = True
FOR_TEST_VICTIM_WORLD_XY = (0.0, 40.0)

# 구조자 충돌체도 초기 LiDAR 팽창영역에 들어오지 않도록 6 m 떨어뜨린다.
# 구조자의 발 높이는 첫 번째 드론의 초기 World Z와 동일하게 맞춘다.
RESCUER_XY = (-34.0, 34.0)
RESCUER_FOOT_Z = float(DRONE_CONFIGS[0][2][2])

# 지형 보간 오차로 발이 지면에 묻히지 않도록 아주 조금 띄운다.
PERSON_GROUND_CLEARANCE_M = 0.08

# 정지한 사람을 물리 장애물로 취급하기 위한 캡슐 충돌체 크기다.
# Capsule의 전체 높이 = cylinder height + 2 * radius = 1.8 m이다.
PERSON_COLLIDER_RADIUS_M = 0.30
PERSON_COLLIDER_CYLINDER_HEIGHT_M = 1.20
PERSON_COLLIDER_TOTAL_HEIGHT_M = (
    PERSON_COLLIDER_CYLINDER_HEIGHT_M
    + 2.0 * PERSON_COLLIDER_RADIUS_M
)

# 지형을 3개의 가로 구역으로 나눠 생성할 수색 경로 설정이다.
SEARCH_AREA_MARGIN_M = 6.0
SEARCH_LANE_SPACING_M = 7.0
SEARCH_SAMPLE_SPACING_M = 7.0
SEARCH_CLEARANCE_M = 6.0
# 두 Waypoint 사이의 직선 구간도 지형과 충돌하지 않도록 1 m 간격으로
# 지형 최고점을 검사한다. Waypoint 끝점만 높이를 재는 기존 방식은 구간
# 중간에 봉우리가 있으면 계획선이 지형 아래로 지나갈 수 있었다.
SEARCH_TERRAIN_PROFILE_SPACING_M = 1.0
SAFE_RETURN_CLEARANCE_M = 12.0

SCRIPT_DIR = Path(__file__).resolve().parent
FOREST_WORLD_PATH = SCRIPT_DIR / "worlds" / "my_forest.usd"
GENERATED_SEARCH_PLAN_PATH = SCRIPT_DIR / "generated_search_plan.json"
GENERATED_GROUND_TRUTH_PATH = SCRIPT_DIR / "generated_ground_truth.json"
GENERATED_TERRAIN_MESH_PATH = SCRIPT_DIR / "generated_terrain_mesh.npz"
GENERATED_ENVIRONMENT_MESH_PATH = (
    SCRIPT_DIR / "generated_environment_meshes.npz"
)

# RViz 지형은 실제 USD Terrain 높이를 이 간격으로 샘플링한다.
# 값이 작을수록 산이 부드럽지만 Marker 메시지가 커진다.
RVIZ_TERRAIN_SAMPLE_SPACING_M = 1.5

# RViz에 실제 형상으로 내보낼 USD 그룹이다.
# Stage 경로의 어느 조상 Prim 이름이라도 아래 이름과 일치하면 해당
# 그룹의 Mesh로 분류한다.
RVIZ_ENVIRONMENT_GROUPS = {
    "pineforest": ("pineforest",),
    "broadleafforest": ("broadleafforest",),
    "bushes": ("bushes",),
    "rocks": ("rocks",),
}

# TRIANGLE_LIST 메시지가 지나치게 커지는 것을 방지하는 그룹별 상한이다.
# 원본 삼각형 수가 이 값을 넘을 때만 균일하게 일부 면을 선택한다.
RVIZ_ENVIRONMENT_MAX_TRIANGLES_PER_GROUP = 120_000


# 대부분의 Isaac Sim 모듈보다 먼저 SimulationApp을 생성해야 한다.
simulation_app = SimulationApp({"headless": False})


from isaacsim.core.utils.extensions import enable_extension

# ROS 2 sensor topic 발행에 필요한 extension을 활성화한다.
enable_extension("isaacsim.ros2.bridge")

# RTX LiDAR 생성에 필요한 extension을 활성화한다.
enable_extension("isaacsim.sensors.rtx")

# ROS2CameraGraph가 생성하는 센서 Viewport를 도킹하기 위해 활성화한다.
enable_extension("omni.kit.viewport.window")

simulation_app.update()


# Extension을 불러온 뒤 깨끗한 Stage를 생성한다.
import omni.usd

omni.usd.get_context().new_stage()


import omni.timeline
import omni.replicator.core as rep
from omni.isaac.core.world import World
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.graphs import ROS2CameraGraph
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.people.person import Person
from pegasus.simulator.logic.vehicles.multirotor import (
    Multirotor,
    MultirotorConfig,
)


class NamespacedLidar(Lidar):
    """드론마다 독립된 ROS 2 PointCloud2 토픽을 발행하는 LiDAR."""

    def __init__(
        self,
        lidar_name,
        topic_name,
        frame_id,
        config=None,
    ):
        super().__init__(lidar_name, config=config or {})
        self._topic_name = topic_name
        self._frame_id = frame_id
        self._render_product = None
        self._writer = None

    def start(self):
        # Pegasus 기본 Lidar는 모든 드론이 /point_cloud를 사용하므로
        # ROS 2 writer를 직접 생성해 드론별 토픽과 frame_id를 지정한다.
        if not self._show_render:
            return

        self._render_product = rep.create.render_product(
            self._sensor.GetPath(),
            [1, 1],
            name="Isaac",
        )
        self._writer = rep.writers.get(
            "RtxLidarROS2PublishPointCloud"
        )
        self._writer.initialize(
            topicName=self._topic_name,
            frameId=self._frame_id,
        )
        self._writer.attach([self._render_product])


class TerrainHeightField:
    def __init__(self, stage):
        self._stage = stage
        self._terrain_prim = self._find_terrain_mesh()
        self._build_interpolator()

    def _find_terrain_mesh(self):
        meshes = [
            prim
            for prim in self._stage.Traverse()
            if prim.IsA(UsdGeom.Mesh)
        ]
        if not meshes:
            raise RuntimeError("Stage에서 Mesh Prim을 찾지 못했습니다.")

        named_candidates = [
            prim
            for prim in meshes
            if "terrain" in prim.GetName().lower()
            or "terrain" in str(prim.GetPath()).lower()
        ]

        candidates = named_candidates if named_candidates else meshes

        def point_count(prim):
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            return len(points) if points is not None else 0

        terrain_prim = max(candidates, key=point_count)
        if not named_candidates:
            carb.log_warn(
                "이름에 Terrain이 들어간 Mesh를 찾지 못해 가장 큰 Mesh를 "
                f"지형으로 사용합니다: {terrain_prim.GetPath()}"
            )

        print(f"[TERRAIN] Mesh Prim: {terrain_prim.GetPath()}")
        return terrain_prim

    def _build_interpolator(self):
        mesh = UsdGeom.Mesh(self._terrain_prim)
        local_points = mesh.GetPointsAttr().Get()
        if local_points is None or len(local_points) < 3:
            raise RuntimeError(
                f"Terrain Mesh 정점이 부족합니다: {self._terrain_prim.GetPath()}"
            )

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        world_matrix = xform_cache.GetLocalToWorldTransform(self._terrain_prim)

        world_points = np.asarray(
            [
                tuple(
                    world_matrix.Transform(
                        Gf.Vec3d(
                            float(point[0]),
                            float(point[1]),
                            float(point[2]),
                        )
                    )
                )
                for point in local_points
            ],
            dtype=np.float64,
        )

        xy = world_points[:, :2]
        z = world_points[:, 2]

        rounded_xy = np.round(xy, decimals=5)
        _, unique_indices = np.unique(
            rounded_xy,
            axis=0,
            return_index=True,
        )
        xy = xy[unique_indices]
        z = z[unique_indices]

        if len(xy) < 3:
            raise RuntimeError("Terrain 높이 보간에 사용할 정점이 부족합니다.")

        self._linear = LinearNDInterpolator(xy, z, fill_value=np.nan)
        self._nearest = NearestNDInterpolator(xy, z)

        self.x_min = float(np.min(xy[:, 0]))
        self.x_max = float(np.max(xy[:, 0]))
        self.y_min = float(np.min(xy[:, 1]))
        self.y_max = float(np.max(xy[:, 1]))
        self.z_min = float(np.min(z))
        self.z_max = float(np.max(z))

        print(
            "[TERRAIN] XY bounds: "
            f"X=({self.x_min:.2f}, {self.x_max:.2f}), "
            f"Y=({self.y_min:.2f}, {self.y_max:.2f})"
        )
        print(
            "[TERRAIN] Z bounds: "
            f"Z=({self.z_min:.2f}, {self.z_max:.2f})"
        )

    def height(self, x, y):
        if not (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
        ):
            raise ValueError(
                f"좌표가 Terrain 범위를 벗어났습니다: X={x:.2f}, Y={y:.2f}"
            )

        value = self._linear(float(x), float(y))
        value = float(np.asarray(value))
        if not np.isfinite(value):
            value = float(self._nearest(float(x), float(y)))
        if not np.isfinite(value):
            raise RuntimeError(
                f"Terrain 높이를 계산하지 못했습니다: X={x:.2f}, Y={y:.2f}"
            )
        return value

    def write_rviz_terrain_mesh(self, output_path, sample_spacing_m):
        """실제 USD Terrain 높이로 RViz용 삼각형 표면을 저장한다."""
        spacing = max(0.25, float(sample_spacing_m))

        # 양 끝 경계를 반드시 포함하는 규칙 격자를 만든다.
        x_count = max(
            2,
            int(math.ceil((self.x_max - self.x_min) / spacing)) + 1,
        )
        y_count = max(
            2,
            int(math.ceil((self.y_max - self.y_min) / spacing)) + 1,
        )
        x_values = np.linspace(self.x_min, self.x_max, x_count)
        y_values = np.linspace(self.y_min, self.y_max, y_count)

        vertices = np.empty(
            (x_count * y_count, 3),
            dtype=np.float32,
        )

        vertex_index = 0
        for world_y in y_values:
            for world_x in x_values:
                vertices[vertex_index] = (
                    float(world_x),
                    float(world_y),
                    float(self.height(world_x, world_y)),
                )
                vertex_index += 1

        # 각 격자 셀을 위쪽에서 보았을 때 반시계 방향인 삼각형 2개로 만든다.
        triangle_count = (x_count - 1) * (y_count - 1) * 2
        triangles = np.empty((triangle_count, 3), dtype=np.int32)
        triangle_index = 0

        for row in range(y_count - 1):
            for column in range(x_count - 1):
                lower_left = row * x_count + column
                lower_right = lower_left + 1
                upper_left = (row + 1) * x_count + column
                upper_right = upper_left + 1

                triangles[triangle_index] = (
                    lower_left,
                    lower_right,
                    upper_right,
                )
                triangle_index += 1
                triangles[triangle_index] = (
                    lower_left,
                    upper_right,
                    upper_left,
                )
                triangle_index += 1

        output_path = Path(output_path)
        temporary_path = output_path.with_suffix(".npz.tmp")
        with temporary_path.open("wb") as file_handle:
            np.savez_compressed(
                file_handle,
                vertices=vertices,
                triangles=triangles,
                map_frame=np.asarray(["map"]),
                coordinate_convention=np.asarray(["world_enu"]),
                source_prim=np.asarray([str(self._terrain_prim.GetPath())]),
                sample_spacing_m=np.asarray([spacing], dtype=np.float32),
                bounds=np.asarray(
                    [
                        self.x_min,
                        self.x_max,
                        self.y_min,
                        self.y_max,
                        self.z_min,
                        self.z_max,
                    ],
                    dtype=np.float32,
                ),
            )
        temporary_path.replace(output_path)

        print(
            "[INFO] RViz Terrain Mesh 저장: "
            f"{output_path}, vertices={len(vertices)}, "
            f"triangles={len(triangles)}, spacing={spacing:.2f}m"
        )

    def random_surface_position(
        self,
        rng,
        x_range,
        y_range,
        max_slope_deg,
        attempts=500,
    ):
        x_low = max(float(x_range[0]), self.x_min)
        x_high = min(float(x_range[1]), self.x_max)
        y_low = max(float(y_range[0]), self.y_min)
        y_high = min(float(y_range[1]), self.y_max)

        if x_low >= x_high or y_low >= y_high:
            raise RuntimeError(
                "PERSON_INCLUDE 영역과 Terrain 영역이 겹치지 않습니다."
            )

        for _ in range(attempts):
            x = float(rng.uniform(x_low, x_high))
            y = float(rng.uniform(y_low, y_high))
            z = self.height(x, y)
            return np.array([x, y, z], dtype=np.float64)

        raise RuntimeError(
            "산 표면 위에 사람을 생성할 수 있는 위치를 찾지 못했습니다."
        )


class EnvironmentMeshExporter:
    """지정된 USD 그룹 아래의 실제 Mesh를 RViz용으로 추출한다."""

    def __init__(self, stage):
        self._stage = stage
        self._xform_cache = UsdGeom.XformCache(
            Usd.TimeCode.Default()
        )

    @staticmethod
    def _normalize_name(value):
        """공백·언더바·대소문자를 무시할 수 있는 이름으로 바꾼다."""
        return "".join(
            character.lower()
            for character in str(value)
            if character.isalnum()
        )

    def _classify_mesh(self, prim):
        """Prim 경로의 조상 이름을 검사해 환경 그룹을 결정한다."""
        normalized_segments = [
            self._normalize_name(segment)
            for segment in str(prim.GetPath()).split("/")
            if segment
        ]

        for category, aliases in RVIZ_ENVIRONMENT_GROUPS.items():
            for segment in normalized_segments:
                if any(
                    segment == alias or segment.startswith(alias)
                    for alias in aliases
                ):
                    return category
        return None

    @staticmethod
    def _triangulate_mesh(mesh):
        """USD 다각형 Face를 삼각형 인덱스로 변환한다."""
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()

        if counts is None or indices is None:
            return np.empty((0, 3), dtype=np.int64)

        counts = [int(value) for value in counts]
        indices = [int(value) for value in indices]
        triangles = []
        offset = 0

        orientation = mesh.GetOrientationAttr().Get()
        left_handed = orientation == UsdGeom.Tokens.leftHanded

        for vertex_count in counts:
            face = indices[offset:offset + vertex_count]
            offset += vertex_count

            if vertex_count < 3 or len(face) != vertex_count:
                continue

            first = face[0]
            for index in range(1, vertex_count - 1):
                second = face[index]
                third = face[index + 1]
                if left_handed:
                    second, third = third, second
                triangles.append((first, second, third))

        if not triangles:
            return np.empty((0, 3), dtype=np.int64)
        return np.asarray(triangles, dtype=np.int64)

    def _extract_mesh(self, prim):
        """Mesh의 정점과 면을 World ENU 좌표로 변환한다."""
        mesh = UsdGeom.Mesh(prim)
        local_points = mesh.GetPointsAttr().Get()
        if local_points is None or len(local_points) < 3:
            return None

        imageable = UsdGeom.Imageable(prim)
        if imageable:
            visibility = imageable.ComputeVisibility()
            if visibility == UsdGeom.Tokens.invisible:
                return None

            purpose = imageable.ComputePurpose()
            if purpose == UsdGeom.Tokens.proxy:
                return None

        path_lower = str(prim.GetPath()).lower()
        if "collision" in path_lower or "collider" in path_lower:
            return None

        world_matrix = self._xform_cache.GetLocalToWorldTransform(prim)
        world_points = np.asarray(
            [
                tuple(
                    world_matrix.Transform(
                        Gf.Vec3d(
                            float(point[0]),
                            float(point[1]),
                            float(point[2]),
                        )
                    )
                )
                for point in local_points
            ],
            dtype=np.float32,
        )
        triangles = self._triangulate_mesh(mesh)

        if triangles.size == 0:
            return None
        if np.min(triangles) < 0 or np.max(triangles) >= len(world_points):
            carb.log_warn(
                "환경 Mesh 인덱스 범위가 잘못되어 건너뜁니다: "
                f"{prim.GetPath()}"
            )
            return None

        return world_points, triangles

    @staticmethod
    def _limit_and_compact(vertices, triangles, max_triangles):
        """삼각형 수를 제한한 뒤 사용하지 않는 정점을 제거한다."""
        original_triangle_count = len(triangles)

        if original_triangle_count > max_triangles:
            selected = np.linspace(
                0,
                original_triangle_count - 1,
                max_triangles,
                dtype=np.int64,
            )
            triangles = triangles[selected]

        used_vertices = np.unique(triangles.reshape(-1))
        remap = np.full(len(vertices), -1, dtype=np.int64)
        remap[used_vertices] = np.arange(
            len(used_vertices),
            dtype=np.int64,
        )

        compact_vertices = vertices[used_vertices]
        compact_triangles = remap[triangles].astype(np.int32)

        return (
            compact_vertices.astype(np.float32),
            compact_triangles,
            original_triangle_count,
        )

    def write(self, output_path):
        """Pine/Broadleaf/Bushes/Rocks Mesh를 하나의 NPZ로 저장한다."""
        grouped_vertices = {
            category: []
            for category in RVIZ_ENVIRONMENT_GROUPS
        }
        grouped_triangles = {
            category: []
            for category in RVIZ_ENVIRONMENT_GROUPS
        }
        grouped_paths = {
            category: []
            for category in RVIZ_ENVIRONMENT_GROUPS
        }
        vertex_offsets = {
            category: 0
            for category in RVIZ_ENVIRONMENT_GROUPS
        }

        for prim in self._stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue

            category = self._classify_mesh(prim)
            if category is None:
                continue

            extracted = self._extract_mesh(prim)
            if extracted is None:
                continue

            vertices, triangles = extracted
            offset = vertex_offsets[category]
            grouped_vertices[category].append(vertices)
            grouped_triangles[category].append(triangles + offset)
            grouped_paths[category].append(str(prim.GetPath()))
            vertex_offsets[category] += len(vertices)

        payload = {
            "format_version": np.asarray([1], dtype=np.int32),
            "map_frame": np.asarray(["map"]),
            "coordinate_convention": np.asarray(["world_enu"]),
        }

        found_group_count = 0
        for category in RVIZ_ENVIRONMENT_GROUPS:
            if not grouped_vertices[category]:
                carb.log_warn(
                    f"RViz 환경 그룹 Mesh를 찾지 못했습니다: {category}"
                )
                payload[f"{category}_vertices"] = np.empty(
                    (0, 3),
                    dtype=np.float32,
                )
                payload[f"{category}_triangles"] = np.empty(
                    (0, 3),
                    dtype=np.int32,
                )
                payload[f"{category}_source_paths"] = np.asarray(
                    [],
                    dtype="U1",
                )
                payload[f"{category}_original_triangle_count"] = (
                    np.asarray([0], dtype=np.int64)
                )
                continue

            vertices = np.concatenate(
                grouped_vertices[category],
                axis=0,
            )
            triangles = np.concatenate(
                grouped_triangles[category],
                axis=0,
            )
            (
                vertices,
                triangles,
                original_triangle_count,
            ) = self._limit_and_compact(
                vertices,
                triangles,
                RVIZ_ENVIRONMENT_MAX_TRIANGLES_PER_GROUP,
            )

            payload[f"{category}_vertices"] = vertices
            payload[f"{category}_triangles"] = triangles
            payload[f"{category}_source_paths"] = np.asarray(
                grouped_paths[category],
                dtype=str,
            )
            payload[f"{category}_original_triangle_count"] = np.asarray(
                [original_triangle_count],
                dtype=np.int64,
            )
            found_group_count += 1

            print(
                f"[INFO] RViz {category}: "
                f"meshes={len(grouped_paths[category])}, "
                f"vertices={len(vertices)}, "
                f"triangles={len(triangles)}, "
                f"original_triangles={original_triangle_count}"
            )

        if found_group_count == 0:
            raise RuntimeError(
                "PineForest/BroadleafForest/Bushes/Rocks 아래에서 "
                "RViz용 Mesh를 하나도 찾지 못했습니다."
            )

        output_path = Path(output_path)
        temporary_path = output_path.with_suffix(".npz.tmp")
        with temporary_path.open("wb") as file_handle:
            np.savez_compressed(file_handle, **payload)
        temporary_path.replace(output_path)

        print(
            "[INFO] RViz 환경 그룹 Mesh 저장: "
            f"{output_path}, groups={found_group_count}"
        )


class ForestRescueSimulation:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.rng = np.random.default_rng()
        self._person_physics_proxies = {}

        # 이전 실행의 실제 스폰 위치가 RViz에 잠시 남지 않도록 제거한다.
        try:
            GENERATED_GROUND_TRUTH_PATH.unlink(missing_ok=True)
            GENERATED_TERRAIN_MESH_PATH.unlink(missing_ok=True)
            GENERATED_ENVIRONMENT_MESH_PATH.unlink(missing_ok=True)
        except OSError as error:
            carb.log_warn(
                "이전 RViz 시각화 파일을 제거하지 못했습니다: "
                f"{error}"
            )

        # 왼쪽 메인 Viewport용 추적 카메라 상태다.
        self._follow_viewport_api = None
        self._follow_camera_ready = False
        self._follow_previous_position = None
        self._follow_direction_xy = None

        # Pegasus 인터페이스
        self.pg = PegasusInterface()

        # PX4 경로를 설정하고 SITL 바이너리가 있는지 확인한다.
        px4_path = Path(
            os.environ.get(
                "PX4_AUTOPILOT_PATH",
                str(Path.home() / "PX4-Autopilot"),
            )
        ).expanduser().resolve()

        px4_binary = px4_path / "build/px4_sitl_default/bin/px4"

        if not px4_binary.is_file():
            raise RuntimeError(
                "PX4 SITL binary was not found.\n"
                f"Expected path: {px4_binary}\n"
                "Run `make px4_sitl_default none` in PX4-Autopilot first."
            )

        self.pg.set_px4_path(str(px4_path))

        print(f"[INFO] PX4 path: {self.pg.px4_path}")
        print(f"[INFO] ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID')}")
        print(
            "[INFO] RMW_IMPLEMENTATION: "
            f"{os.environ.get('RMW_IMPLEMENTATION')}"
        )

        # Isaac Sim World를 생성한다.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # 사용자 정의 산악·산림 환경을 현재 Stage 전체로 불러온다.
        if not FOREST_WORLD_PATH.is_file():
            raise FileNotFoundError(
                "산림 환경 USD 파일을 찾을 수 없습니다.\n"
                f"Expected path: {FOREST_WORLD_PATH}"
            )

        print(f"[INFO] Forest world: {FOREST_WORLD_PATH}")

        self.pg.load_environment(str(FOREST_WORLD_PATH))
        simulation_app.update()

        # 검은 배경 대신 하늘색 Dome Light를 적용한다.
        self._configure_sky_background()
        simulation_app.update()

        self._verify_loaded_environment()
        self._fit_viewport_to_environment()

        stage = omni.usd.get_context().get_stage()
        self.terrain = TerrainHeightField(stage)
        self.terrain.write_rviz_terrain_mesh(
            GENERATED_TERRAIN_MESH_PATH,
            RVIZ_TERRAIN_SAMPLE_SPACING_M,
        )
        EnvironmentMeshExporter(stage).write(
            GENERATED_ENVIRONMENT_MESH_PATH
        )
        self._write_generated_search_plan()

        self._spawn_people()
        self._spawn_iris()
        simulation_app.update()
        self._configure_drone_cameras()

        self.world.reset()
        simulation_app.update()
        self._create_docked_camera_viewports()

        # 센서 Render Product 생성 이후에도 RTX 배경 색상을 다시 적용한다.
        self._configure_sky_background()
        simulation_app.update()

        self._setup_follow_viewport()

        print("[OK] Forest-rescue multi-drone simulation initialized")
        for prim_path, _, _ in DRONE_CONFIGS:
            drone_name = prim_path.rsplit("/", 1)[-1]
            print(f"[INFO] {drone_name} RGB: /{drone_name}/Camera/rgb")
            print(f"[INFO] {drone_name} Depth: /{drone_name}/Camera/depth")
            print(
                f"[INFO] {drone_name} Depth PointCloud: "
                f"/{drone_name}/Camera/depth_pcl"
            )
            print(
                f"[INFO] {drone_name} CameraInfo: "
                f"/{drone_name}/Camera/camera_info"
            )
            print(
                f"[INFO] {drone_name} LiDAR: "
                f"/{drone_name}/point_cloud"
            )

    @staticmethod
    def _sample_axis(start, stop, spacing):
        """양 끝점을 포함하도록 일정 간격의 좌표 목록을 만든다."""
        distance = abs(float(stop) - float(start))
        count = max(2, int(math.ceil(distance / float(spacing))) + 1)
        return [
            float(value)
            for value in np.linspace(float(start), float(stop), count)
        ]

    def _write_generated_search_plan(self):
        """Terrain 높이를 반영한 드론 3대의 3차원 지그재그 경로를 저장한다."""
        x_low = self.terrain.x_min + SEARCH_AREA_MARGIN_M
        x_high = self.terrain.x_max - SEARCH_AREA_MARGIN_M
        y_low = self.terrain.y_min + SEARCH_AREA_MARGIN_M
        y_high = self.terrain.y_max - SEARCH_AREA_MARGIN_M

        if x_low >= x_high or y_low >= y_high:
            raise RuntimeError("수색 경로를 만들 Terrain 영역이 부족합니다.")

        # 그림과 같이 위·가운데·아래의 가로 구역 3개로 균등 분할한다.
        y_edges = np.linspace(y_low, y_high, 4)
        safe_return_world_z = (
            self.terrain.z_max + SAFE_RETURN_CLEARANCE_M
        )
        plan = {
            "format_version": 2,
            "map_frame": "map",
            "coordinate_convention": "world_enu_and_local_ned",
            "terrain_bounds": {
                "x_min": self.terrain.x_min,
                "x_max": self.terrain.x_max,
                "y_min": self.terrain.y_min,
                "y_max": self.terrain.y_max,
                "z_min": self.terrain.z_min,
                "z_max": self.terrain.z_max,
            },
            "search_clearance_m": SEARCH_CLEARANCE_M,
            "safe_return_world_z": safe_return_world_z,
            "test_victim_spawn": None,
            "drones": {},
        }

        # 시험에서는 복잡한 자동 탐색 없이 지정한 XY를 그대로 사용한다.
        if FOR_TEST_VICTIM_SPAWN_ENABLED:
            test_x = float(FOR_TEST_VICTIM_WORLD_XY[0])
            test_y = float(FOR_TEST_VICTIM_WORLD_XY[1])

            if not (
                self.terrain.x_min <= test_x <= self.terrain.x_max
                and self.terrain.y_min <= test_y <= self.terrain.y_max
            ):
                raise RuntimeError(
                    "시험용 조난자 XY가 Terrain 범위를 벗어났습니다: "
                    f"XY=({test_x:.2f}, {test_y:.2f})"
                )

            test_ground_z = self.terrain.height(test_x, test_y)
            self.test_victim_spawn_world_enu = [
                test_x,
                test_y,
                float(
                    test_ground_z
                    + PERSON_GROUND_CLEARANCE_M
                ),
            ]
            plan["test_victim_spawn"] = {
                "enabled": True,
                "selection": "hardcoded_xy",
                "world_enu": list(
                    self.test_victim_spawn_world_enu
                ),
            }
            print(
                "[TEST] 조난자 시험 위치 고정: "
                f"XY=({test_x:.2f}, {test_y:.2f}), "
                f"terrain_Z={test_ground_z:.2f}, "
                f"spawn_Z={self.test_victim_spawn_world_enu[2]:.2f}"
            )

        # drone_01은 상단, 02는 중앙, 03은 하단 구역을 맡는다.
        zone_indices = (2, 1, 0)
        for config, zone_index in zip(DRONE_CONFIGS, zone_indices):
            prim_path, vehicle_id, home = config
            drone_name = prim_path.rsplit("/", 1)[-1]
            home_x, home_y, home_z = [float(value) for value in home]
            zone_min_y = float(y_edges[zone_index])
            zone_max_y = float(y_edges[zone_index + 1])

            # 모든 드론이 시작 지점에 가까운 높은 Y쪽부터 수색한다.
            rows = self._sample_axis(
                zone_max_y,
                zone_min_y,
                SEARCH_LANE_SPACING_M,
            )

            route_xy = []

            def append_route_point(world_x, world_y):
                """연속 중복을 제거해 수색 경로의 XY 뼈대를 만든다."""
                point = (float(world_x), float(world_y))
                if route_xy:
                    previous = route_xy[-1]
                    if (
                        abs(previous[0] - point[0]) < 1.0e-6
                        and abs(previous[1] - point[1]) < 1.0e-6
                    ):
                        return
                route_xy.append(point)

            # 출발점도 경로에 넣어 첫 이동 구간의 지형 최고점을 검사한다.
            append_route_point(home_x, home_y)

            # 세 드론이 출발 직후 같은 (-40.67, 40.0) 지점으로 합류하지
            # 않도록, 각자의 시작 X 통로에서 담당 구역 Y까지 먼저 분리
            # 이동한다. 그 다음 담당 구역의 Y를 유지하면서 왼쪽 경계로
            # 진입한다. 이렇게 하면 초기 경로가 서로 겹치지 않는다.
            ingress_y = min(max(home_y, y_low), y_high)
            target_ingress_y = min(max(zone_max_y, y_low), y_high)
            for world_y in self._sample_axis(
                ingress_y,
                target_ingress_y,
                SEARCH_SAMPLE_SPACING_M,
            )[1:]:
                append_route_point(home_x, world_y)
            for world_x in self._sample_axis(
                home_x,
                x_low,
                SEARCH_SAMPLE_SPACING_M,
            )[1:]:
                append_route_point(world_x, target_ingress_y)

            for row_index, world_y in enumerate(rows):
                if row_index % 2 == 0:
                    row_x = self._sample_axis(
                        x_low,
                        x_high,
                        SEARCH_SAMPLE_SPACING_M,
                    )
                else:
                    row_x = self._sample_axis(
                        x_high,
                        x_low,
                        SEARCH_SAMPLE_SPACING_M,
                    )

                for world_x in row_x:
                    append_route_point(world_x, world_y)

            # 각 선분의 최고 지형고도를 구하고 양 끝 Waypoint를 모두 그보다
            # SEARCH_CLEARANCE_M만큼 높인다. 따라서 PX4가 두 점 사이를 직선
            # 보간해도 선분 중간의 산봉우리와 충돌하지 않는다.
            point_ground_z = [
                self.terrain.height(world_x, world_y)
                for world_x, world_y in route_xy
            ]
            segment_safe_z = []
            for start_xy, end_xy in zip(route_xy, route_xy[1:]):
                distance = math.hypot(
                    end_xy[0] - start_xy[0],
                    end_xy[1] - start_xy[1],
                )
                sample_count = max(
                    2,
                    int(math.ceil(
                        distance / SEARCH_TERRAIN_PROFILE_SPACING_M
                    )) + 1,
                )
                terrain_max = max(
                    self.terrain.height(
                        start_xy[0]
                        + (end_xy[0] - start_xy[0]) * ratio,
                        start_xy[1]
                        + (end_xy[1] - start_xy[1]) * ratio,
                    )
                    for ratio in np.linspace(0.0, 1.0, sample_count)
                )
                segment_safe_z.append(terrain_max + SEARCH_CLEARANCE_M)

            waypoints = []
            for point_index, ((world_x, world_y), ground_z) in enumerate(
                zip(route_xy, point_ground_z)
            ):
                required_z = ground_z + SEARCH_CLEARANCE_M
                if point_index > 0:
                    required_z = max(
                        required_z,
                        segment_safe_z[point_index - 1],
                    )
                if point_index < len(segment_safe_z):
                    required_z = max(
                        required_z,
                        segment_safe_z[point_index],
                    )
                waypoints.append(
                    {
                        "north_m": world_y - home_y,
                        "east_m": world_x - home_x,
                        "down_m": -(required_z - home_z),
                        "world_enu": [world_x, world_y, required_z],
                    }
                )

            plan["drones"][drone_name] = {
                "vehicle_id": int(vehicle_id),
                "home_world_enu": [home_x, home_y, home_z],
                "zone_y_min": zone_min_y,
                "zone_y_max": zone_max_y,
                "safe_return_down_m": -(
                    safe_return_world_z - home_z
                ),
                "waypoints": waypoints,
            }

        GENERATED_SEARCH_PLAN_PATH.write_text(
            json.dumps(plan, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(
            "[INFO] Terrain 기반 3드론 수색 경로 저장: "
            f"{GENERATED_SEARCH_PLAN_PATH}"
        )
        for drone_name, drone_plan in plan["drones"].items():
            print(
                f"[INFO] {drone_name} zone: "
                f"Y=({drone_plan['zone_y_min']:.2f}, "
                f"{drone_plan['zone_y_max']:.2f}), "
                f"waypoints={len(drone_plan['waypoints'])}"
            )

    def _configure_sky_background(self):
        """RTX 배경을 하늘색으로 강제하고 Dome Light로 장면을 밝힌다."""
        stage = omni.usd.get_context().get_stage()

        # Dome Light는 검은색 드론과 지형을 밝히는 장면 조명으로 사용한다.
        dome_light = UsdLux.DomeLight.Define(
            stage,
            "/World/SkyDome",
        )
        dome_light.CreateColorAttr().Set(
            Gf.Vec3f(0.45, 0.70, 1.0)
        )
        dome_light.CreateIntensityAttr().Set(400.0)
        dome_light.CreateExposureAttr().Set(0.0)

        # Dome Light 색만 지정하면 RTX Real-Time에서 배경이 검정으로
        # 남을 수 있으므로, 렌더러의 배경 소스를 Color로 강제한다.
        settings = carb.settings.get_settings()
        settings.set_int(
            "/rtx/background/source/type",
            2,
        )
        settings.set_float_array(
            "/rtx/background/source/color",
            [0.45, 0.70, 1.0],
        )

        print(
            "[INFO] RTX sky background configured: "
            "source=color, color=(0.45, 0.70, 1.0), "
            "dome_intensity=400"
        )

    def _verify_loaded_environment(self):
        """요청한 USD가 실제 Root Layer로 열렸는지 확인한다."""
        stage = omni.usd.get_context().get_stage()
        root_layer = stage.GetRootLayer()
        root_path = root_layer.realPath or root_layer.identifier

        print(f"[CHECK] Stage root layer: {root_path}")

        if root_layer.realPath:
            loaded_path = Path(root_layer.realPath).resolve()
            expected_path = FOREST_WORLD_PATH.resolve()
            if loaded_path != expected_path:
                raise RuntimeError(
                    "요청한 산악 환경과 실제로 열린 Stage가 다릅니다.\n"
                    f"Expected: {expected_path}\n"
                    f"Loaded: {loaded_path}"
                )

        default_prim = stage.GetDefaultPrim()
        if default_prim and default_prim.IsValid():
            print(f"[CHECK] Default Prim: {default_prim.GetPath()}")
        else:
            carb.log_warn(
                "산악 환경 USD에 Default Prim이 없어 /World 또는 첫 번째 "
                "상위 Prim을 환경 루트로 사용합니다."
            )

        top_level_prims = [
            str(prim.GetPath())
            for prim in stage.GetPseudoRoot().GetChildren()
        ]
        print(f"[CHECK] Top-level Prims: {top_level_prims}")

    @staticmethod
    def _get_environment_root_prim():
        """환경 Bounding Box 계산에 사용할 유효한 루트 Prim을 찾는다."""
        stage = omni.usd.get_context().get_stage()

        default_prim = stage.GetDefaultPrim()
        if default_prim and default_prim.IsValid():
            return default_prim

        world_prim = stage.GetPrimAtPath("/World")
        if world_prim and world_prim.IsValid():
            return world_prim

        top_level_prims = list(stage.GetPseudoRoot().GetChildren())
        if top_level_prims:
            return top_level_prims[0]

        raise RuntimeError("산악 환경 USD에 불러올 Prim이 없습니다.")

    def _fit_viewport_to_environment(self):
        """환경 크기를 계산해 산 전체가 보이도록 Viewport를 맞춘다."""
        target_prim = self._get_environment_root_prim()

        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [
                UsdGeom.Tokens.default_,
                UsdGeom.Tokens.render,
                UsdGeom.Tokens.proxy,
            ],
        )
        aligned_range = bbox_cache.ComputeWorldBound(
            target_prim
        ).ComputeAlignedRange()

        if aligned_range.IsEmpty():
            carb.log_warn(
                "환경 Bounding Box를 계산하지 못해 기본 Viewport를 "
                "사용합니다."
            )
            self.pg.set_viewport_camera(
                [30.0, 30.0, 25.0],
                [0.0, 0.0, 0.0],
            )
            return

        minimum = aligned_range.GetMin()
        maximum = aligned_range.GetMax()
        center = 0.5 * (minimum + maximum)
        size = maximum - minimum
        largest_dimension = max(
            float(size[0]),
            float(size[1]),
            float(size[2]),
        )

        values = [
            float(center[0]),
            float(center[1]),
            float(center[2]),
            largest_dimension,
        ]
        if not all(np.isfinite(value) for value in values):
            raise RuntimeError(
                "산악 환경의 Bounding Box 값이 유효하지 않습니다: "
                f"center={center}, size={size}"
            )

        camera_distance = max(20.0, largest_dimension * 0.75)
        eye = [
            float(center[0]) + camera_distance,
            float(center[1]) + camera_distance,
            max(
                float(maximum[2]) + camera_distance * 0.35,
                float(center[2]) + camera_distance * 0.65,
            ),
        ]
        target = [
            float(center[0]),
            float(center[1]),
            float(center[2]),
        ]

        self.pg.set_viewport_camera(eye, target)
        print(
            "[CHECK] Environment bounds: "
            f"min={minimum}, max={maximum}"
        )
        print(f"[CHECK] Viewport eye={eye}, target={target}")

    @staticmethod
    def _write_ground_truth(victim_position, victim_index):
        """드론 시작점과 실제 조난자 스폰 위치를 원자적으로 저장한다."""
        drone_starts = []
        for prim_path, vehicle_id, position in DRONE_CONFIGS:
            drone_starts.append(
                {
                    "drone_id": prim_path.rsplit("/", 1)[-1],
                    "vehicle_id": int(vehicle_id),
                    "world_enu": [
                        float(position[0]),
                        float(position[1]),
                        float(position[2]),
                    ],
                }
            )

        payload = {
            "format_version": 1,
            "map_frame": "map",
            "coordinate_convention": "world_enu",
            "drone_starts": drone_starts,
            "victim": {
                "victim_id": "victim_01",
                "spawn_candidate_index": int(victim_index),
                "world_enu": [
                    float(victim_position[0]),
                    float(victim_position[1]),
                    float(victim_position[2]),
                ],
            },
        }

        # ROS 노드가 저장 중인 JSON을 읽지 않도록 임시 파일을 완성한 뒤
        # 최종 파일명으로 교체한다.
        temporary_path = GENERATED_GROUND_TRUTH_PATH.with_suffix(
            ".json.tmp"
        )
        temporary_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        temporary_path.replace(GENERATED_GROUND_TRUTH_PATH)

        print(
            "[INFO] RViz Ground Truth 저장: "
            f"{GENERATED_GROUND_TRUTH_PATH}, "
            f"victim={payload['victim']['world_enu']}"
        )

    def _spawn_people(self):
        """조난자 1명과 구조자 1명을 지정 조건에 맞게 정지 상태로 생성한다."""
        preferred_asset = "original_female_adult_business_02"
        available_assets = Person.get_character_asset_list()

        if not available_assets:
            raise RuntimeError("No Pegasus person assets were found.")

        if preferred_asset in available_assets:
            selected_asset = preferred_asset
        else:
            selected_asset = available_assets[0]
            carb.log_warn(
                f"Preferred person asset was not found. "
                f"Using {selected_asset} instead."
            )

        if FOR_TEST_VICTIM_SPAWN_ENABLED:
            victim_position = getattr(
                self,
                "test_victim_spawn_world_enu",
                None,
            )
            if victim_position is None:
                raise RuntimeError(
                    "시험용 조난자 위치가 생성되지 않았습니다. "
                    "FOR_TEST_VICTIM_WORLD_XY를 확인하세요."
                )
            victim_position = [
                float(value) for value in victim_position
            ]
            victim_index = -1
            spawn_description = (
                "TEST hardcoded "
                f"XY=({FOR_TEST_VICTIM_WORLD_XY[0]:.1f}, "
                f"{FOR_TEST_VICTIM_WORLD_XY[1]:.1f})"
            )
        else:
            victim_index = int(
                self.rng.integers(len(VICTIM_SPAWN_POSITIONS))
            )
            victim_candidate = VICTIM_SPAWN_POSITIONS[victim_index]
            victim_x = float(victim_candidate[0])
            victim_y = float(victim_candidate[1])

            # 후보 좌표의 실제 Terrain 높이를 다시 읽는다.
            victim_ground_z = self.terrain.height(victim_x, victim_y)
            victim_position = [
                victim_x,
                victim_y,
                victim_ground_z + PERSON_GROUND_CLEARANCE_M,
            ]
            spawn_description = f"candidate {victim_index + 1}"

        # Person을 실제로 생성할 때 사용하는 좌표를 그대로 RViz용
        # Ground Truth 파일에 기록한다.
        self._write_ground_truth(victim_position, victim_index)

        self.victim = Person(
            "victim_01",
            selected_asset,
            init_pos=victim_position,
            init_yaw=0.0,
        )
        self._create_person_physics_proxy(
            "victim_01",
            self.victim,
            victim_position,
        )
        print(
            f"[INFO] Spawned victim at {spawn_description}: "
            f"({victim_position[0]:.3f}, "
            f"{victim_position[1]:.3f}, "
            f"{victim_position[2]:.3f})"
        )

        rescuer_x, rescuer_y = RESCUER_XY
        rescuer_position = [
            rescuer_x,
            rescuer_y,
            RESCUER_FOOT_Z,
        ]
        self.rescuer = Person(
            "rescuer_01",
            selected_asset,
            init_pos=rescuer_position,
            init_yaw=0.0,
        )
        self._create_person_physics_proxy(
            "rescuer_01",
            self.rescuer,
            rescuer_position,
        )
        print(
            "[INFO] Spawned rescuer beside drones at "
            f"({rescuer_position[0]:.3f}, "
            f"{rescuer_position[1]:.3f}, "
            f"{rescuer_position[2]:.3f})"
        )

    def _create_person_physics_proxy(
        self,
        person_name,
        person,
        foot_position,
    ):
        """사람을 따라 움직이는 보이지 않는 캡슐형 충돌체를 만든다."""
        stage = omni.usd.get_context().get_stage()
        collider_path = f"/World/person_colliders/{person_name}"

        capsule = UsdGeom.Capsule.Define(stage, collider_path)
        capsule.CreateAxisAttr().Set(UsdGeom.Tokens.z)
        capsule.CreateRadiusAttr().Set(PERSON_COLLIDER_RADIUS_M)
        capsule.CreateHeightAttr().Set(
            PERSON_COLLIDER_CYLINDER_HEIGHT_M
        )

        # Capsule의 원점은 중심이므로 발 위치에서 사람 키의 절반만큼 올린다.
        capsule_center = Gf.Vec3d(
            float(foot_position[0]),
            float(foot_position[1]),
            float(foot_position[2])
            + PERSON_COLLIDER_TOTAL_HEIGHT_M * 0.5,
        )
        translate_op = capsule.AddTranslateOp()
        translate_op.Set(capsule_center)

        # 시각적 Person은 그대로 두고, 이 프록시만 물리 충돌에 사용한다.
        capsule.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        collider_prim = capsule.GetPrim()
        UsdPhysics.CollisionAPI.Apply(collider_prim)

        rigid_body = UsdPhysics.RigidBodyAPI.Apply(collider_prim)
        rigid_body.CreateRigidBodyEnabledAttr(True)
        rigid_body.CreateKinematicEnabledAttr(True)

        # Person은 Animation Graph를 통해 이동하므로 캐릭터와 충돌체를
        # 부모-자식으로 묶지 않고, Person.state.position을 매 프레임 따라간다.
        self._person_physics_proxies[person_name] = {
            "person": person,
            "translate_op": translate_op,
            "collider_path": collider_path,
        }

        print(
            f"[INFO] Physics proxy created: {collider_path}, "
            f"foot={foot_position}, center={tuple(capsule_center)}"
        )

    def _sync_person_physics_proxies(self):
        """걷는 사람의 현재 World 위치로 물리 충돌체를 이동한다."""
        for person_name, proxy in self._person_physics_proxies.items():
            person_position = np.asarray(
                proxy["person"].state.position,
                dtype=np.float64,
            )

            if person_position.shape != (3,) or not np.all(
                np.isfinite(person_position)
            ):
                carb.log_warn(
                    f"Invalid person position for {person_name}: "
                    f"{person_position}"
                )
                continue

            # Person.state.position은 발 기준 World 좌표다.
            # 캡슐 Prim의 원점은 중심이므로 높이 절반을 더한다.
            capsule_center = Gf.Vec3d(
                float(person_position[0]),
                float(person_position[1]),
                float(person_position[2])
                + PERSON_COLLIDER_TOTAL_HEIGHT_M * 0.5,
            )
            proxy["translate_op"].Set(capsule_center)

    def _spawn_iris(self):
        """시작 지점 주변에 카메라가 장착된 Iris 드론 3대를 생성한다."""
        self.drones = []

        for prim_path, vehicle_id, position in DRONE_CONFIGS:
            drone_name = prim_path.rsplit("/", 1)[-1]
            multirotor_config = MultirotorConfig()

            px4_config = PX4MavlinkBackendConfig(
                {
                    "vehicle_id": vehicle_id,
                    "px4_autolaunch": True,
                    "px4_dir": self.pg.px4_path,
                }
            )
            multirotor_config.backends = [
                PX4MavlinkBackend(px4_config)
            ]

            multirotor_config.graphs = [
                ROS2CameraGraph(
                    "body/Camera",
                    config={
                        "resolution": CAMERA_RESOLUTION,
                        "types": [
                            "rgb",
                            "depth",
                            "depth_pcl",
                            "camera_info",
                        ],
                        "namespace": f"/{drone_name}",
                        "topic": "/Camera",
                        "tf_frame_id": (
                            f"{drone_name}/camera_optical_frame"
                        ),
                    },
                )
            ]

            # 세 드론에 LiDAR를 각각 하나씩 장착하고 토픽을 분리한다.
            multirotor_config.graphical_sensors = [
                NamespacedLidar(
                    "lidar",
                    topic_name=f"/{drone_name}/point_cloud",
                    frame_id=f"{drone_name}/base_scan",
                    config={
                        "frequency": 10.0,
                        "position": np.array([0.0, 0.0, 0.15]),
                        "orientation": np.array([0.0, 0.0, 0.0]),
                        "sensor_configuration": {
                            "sensor_configuration": "Example_Rotary"
                        },
                        "show_render": True,
                    },
                )
            ]

            drone = Multirotor(
                prim_path,
                ROBOTS["Iris"],
                vehicle_id,
                position,
                Rotation.from_euler(
                    "XYZ",
                    [0.0, 0.0, 0.0],
                    degrees=True,
                ).as_quat(),
                config=multirotor_config,
            )
            self.drones.append(drone)
            print(
                f"[INFO] Spawned drone {vehicle_id + 1}: {prim_path} at "
                f"({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
            )

        # 기존 코드와의 호환성을 위해 1번 드론도 별도 참조로 남긴다.
        self.drone = self.drones[0]

    @staticmethod
    def _set_camera_translation_zero(xformable):
        """기존 회전은 유지하고 카메라 local Translate만 0,0,0으로 맞춘다."""
        translate_op = next(
            (
                op
                for op in xformable.GetOrderedXformOps()
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
            ),
            None,
        )

        if translate_op is None:
            xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
            return

        attribute_type = str(translate_op.GetAttr().GetTypeName())
        if attribute_type == "float3":
            zero = Gf.Vec3f(0.0, 0.0, 0.0)
        elif attribute_type == "half3":
            zero = Gf.Vec3h(0.0, 0.0, 0.0)
        else:
            zero = Gf.Vec3d(0.0, 0.0, 0.0)
        translate_op.Set(zero)

    def _configure_drone_cameras(self):
        """세 카메라의 초점거리와 body 기준 상대 위치를 동일하게 설정한다."""
        stage = omni.usd.get_context().get_stage()

        for camera_path in CAMERA_PRIM_PATHS:
            camera_prim = stage.GetPrimAtPath(camera_path)
            if not camera_prim.IsValid():
                raise RuntimeError(
                    f"Iris 카메라 Prim을 찾을 수 없습니다: {camera_path}"
                )

            camera = UsdGeom.Camera(camera_prim)
            camera.GetFocalLengthAttr().Set(CAMERA_FOCAL_LENGTH_MM)

            horizontal_aperture = float(
                camera.GetHorizontalApertureAttr().Get()
            )
            horizontal_fov_deg = math.degrees(
                2.0
                * math.atan(
                    horizontal_aperture
                    / (2.0 * CAMERA_FOCAL_LENGTH_MM)
                )
            )

            xformable = UsdGeom.Xformable(camera_prim)
            self._set_camera_translation_zero(xformable)

            # 기존 카메라 자세를 유지하면서 아래쪽 30도 회전을 추가한다.
            orient_op = next(
                (
                    op
                    for op in xformable.GetOrderedXformOps()
                    if op.GetOpType() == UsdGeom.XformOp.TypeOrient
                ),
                None,
            )

            if orient_op is None:
                carb.log_warn(
                    f"Camera orient op가 없어 하향각을 적용하지 못했습니다: "
                    f"{camera_path}"
                )
            else:
                current_quaternion = orient_op.Get()
                imaginary = current_quaternion.GetImaginary()
                current_rotation = Rotation.from_quat(
                    [
                        float(imaginary[0]),
                        float(imaginary[1]),
                        float(imaginary[2]),
                        float(current_quaternion.GetReal()),
                    ]
                )
                down_rotation = Rotation.from_euler(
                    "X",
                    -CAMERA_DOWN_TILT_DEG,
                    degrees=True,
                )
                configured_xyzw = (
                    current_rotation * down_rotation
                ).as_quat()
                x, y, z, w = [
                    float(value)
                    for value in configured_xyzw
                ]

                attribute_type = str(
                    orient_op.GetAttr().GetTypeName()
                )
                if attribute_type == "quatd":
                    configured_quaternion = Gf.Quatd(
                        w,
                        Gf.Vec3d(x, y, z),
                    )
                elif attribute_type == "quath":
                    configured_quaternion = Gf.Quath(
                        w,
                        Gf.Vec3h(x, y, z),
                    )
                else:
                    configured_quaternion = Gf.Quatf(
                        w,
                        Gf.Vec3f(x, y, z),
                    )
                orient_op.Set(configured_quaternion)

            print(
                f"[INFO] Camera configured: {camera_path}, "
                f"local_translate=(0, 0, 0), "
                f"focal={CAMERA_FOCAL_LENGTH_MM:.1f}mm, "
                f"horizontal_fov≈{horizontal_fov_deg:.1f}deg"
            )

    def _create_docked_camera_viewports(self):
        """ROS2CameraGraph의 실제 센서 Viewport 3개를 고정 배치한다.

        배치 구조는 오른쪽 영역의 상단에 Camera 01, 하단을 좌우로
        나눠 Camera 02와 Camera 03을 두는 형태다. 별도 Viewport를
        만들지 않고 ROS RGB/Depth 발행에 사용되는 640x480 Viewport
        자체를 도킹하므로 RViz 영상과 같은 카메라·화각을 사용한다.
        """
        try:
            import omni.ui as ui
            from omni.kit.viewport.utility import (
                get_active_viewport_window,
                get_viewport_from_window_name,
            )

            main_window = get_active_viewport_window(
                window_name="Viewport"
            )
            if main_window is None:
                raise RuntimeError("기본 Viewport 창을 찾지 못했습니다.")

            main_title = getattr(main_window, "title", "Viewport")
            main_window_handle = ui.Workspace.get_window(main_title)
            if main_window_handle is None:
                main_window_handle = ui.Workspace.get_window("Viewport")
            if main_window_handle is None:
                raise RuntimeError(
                    "기본 Viewport WindowHandle을 찾지 못했습니다."
                )

            sensor_viewport_specs = [
                ("/quadrotor_01/Camera", CAMERA_PRIM_PATHS[0]),
                ("/quadrotor_02/Camera", CAMERA_PRIM_PATHS[1]),
                ("/quadrotor_03/Camera", CAMERA_PRIM_PATHS[2]),
            ]

            # ROS2CameraGraph가 센서 Viewport와 WindowHandle을 모두
            # 생성할 때까지 기다린다. 새 Viewport는 만들지 않는다.
            sensor_viewports = {}
            sensor_window_handles = {}
            for _ in range(120):
                simulation_app.update()
                sensor_viewports = {
                    title: get_viewport_from_window_name(title)
                    for title, _ in sensor_viewport_specs
                }
                sensor_window_handles = {
                    title: ui.Workspace.get_window(title)
                    for title, _ in sensor_viewport_specs
                }
                if all(
                    viewport is not None
                    for viewport in sensor_viewports.values()
                ) and all(
                    handle is not None
                    for handle in sensor_window_handles.values()
                ):
                    break

            missing = [
                title
                for title, viewport in sensor_viewports.items()
                if viewport is None
            ]
            if missing:
                raise RuntimeError(
                    "ROS 센서 Viewport를 찾지 못했습니다: "
                    f"{missing}"
                )
            missing_handles = [
                title
                for title, handle in sensor_window_handles.items()
                if handle is None
            ]
            if missing_handles:
                raise RuntimeError(
                    "ROS 센서 WindowHandle을 찾지 못했습니다: "
                    f"{missing_handles}"
                )

            # Viewport 메뉴의 Camera Light와 같은 RTX 조명 모드를 켠다.
            carb.settings.get_settings().set_bool(
                "/rtx/useViewLightingMode",
                True,
            )

            # 창이 먼저 만들어지면 잠시 /OmniverseKit_Persp를 가리킬 수
            # 있다. 이 경우 실패시키지 않고 해당 ROS Render Product를
            # 실제 드론 카메라에 명시적으로 연결한다.
            for title, expected_camera_path in sensor_viewport_specs:
                viewport_api = sensor_viewports[title]
                actual_camera_path = str(viewport_api.camera_path)
                if actual_camera_path != expected_camera_path:
                    print(
                        f"[INFO] Sensor Viewport camera binding: {title}, "
                        f"from={actual_camera_path}, "
                        f"to={expected_camera_path}"
                    )
                    viewport_api.camera_path = expected_camera_path

                for _ in range(30):
                    simulation_app.update()
                    actual_camera_path = str(viewport_api.camera_path)
                    if actual_camera_path == expected_camera_path:
                        break
                if actual_camera_path != expected_camera_path:
                    raise RuntimeError(
                        f"{title} Camera 연결 실패: "
                        f"expected={expected_camera_path}, "
                        f"actual={actual_camera_path}"
                    )

                self._enable_viewport_camera_light(viewport_api, title)
                print(
                    f"[CHECK] Sensor Viewport {title}: "
                    f"camera={actual_camera_path}, "
                    f"resolution={viewport_api.resolution}"
                )

            camera_01_title = sensor_viewport_specs[0][0]
            camera_02_title = sensor_viewport_specs[1][0]
            camera_03_title = sensor_viewport_specs[2][0]
            camera_01_window = sensor_window_handles[camera_01_title]
            camera_02_window = sensor_window_handles[camera_02_title]
            camera_03_window = sensor_window_handles[camera_03_title]

            camera_01_window.visible = True
            camera_02_window.visible = True
            camera_03_window.visible = True

            # 메인 화면 오른쪽에 Camera 01 영역을 만든다.
            camera_01_window.dock_in(
                main_window_handle,
                ui.DockPosition.RIGHT,
                0.38,
            )
            simulation_app.update()

            # Camera 01 아래쪽에 Camera 02를 배치한다.
            camera_02_window.dock_in(
                camera_01_window,
                ui.DockPosition.BOTTOM,
                0.50,
            )
            simulation_app.update()

            # 하단 Camera 02 영역의 오른쪽에 Camera 03을 배치한다.
            camera_03_window.dock_in(
                camera_02_window,
                ui.DockPosition.RIGHT,
                0.50,
            )
            for _ in range(3):
                simulation_app.update()

            dock_states = {
                camera_01_title: bool(camera_01_window.docked),
                camera_02_title: bool(camera_02_window.docked),
                camera_03_title: bool(camera_03_window.docked),
            }
            if not all(dock_states.values()):
                raise RuntimeError(
                    f"WindowHandle 도킹 상태 확인 실패: {dock_states}"
                )

            print(
                "[INFO] ROS sensor Viewports docked: "
                "01=right/top, 02=right/bottom-left, "
                "03=right/bottom-right"
            )
        except Exception as error:
            carb.log_error(
                "ROS 센서 Viewport 자동 배치 실패. "
                f"기본 시뮬레이션은 계속 실행합니다: {error}"
            )

    def _setup_follow_viewport(self):
        """왼쪽 메인 Viewport를 특정 드론을 따라가는 카메라에 연결한다."""
        try:
            from omni.kit.viewport.utility import (
                get_viewport_from_window_name,
            )

            stage = omni.usd.get_context().get_stage()
            target_prim = stage.GetPrimAtPath(FOLLOW_DRONE_PRIM_PATH)
            if not target_prim.IsValid():
                raise RuntimeError(
                    "추적 대상 드론 Prim을 찾지 못했습니다: "
                    f"{FOLLOW_DRONE_PRIM_PATH}"
                )

            # 센서 카메라와 분리된 Viewport 전용 Camera Prim이다.
            follow_camera = UsdGeom.Camera.Define(
                stage,
                FOLLOW_CAMERA_PRIM_PATH,
            )
            follow_camera.GetFocalLengthAttr().Set(24.0)

            main_viewport = get_viewport_from_window_name("Viewport")
            if main_viewport is None:
                raise RuntimeError("왼쪽 메인 Viewport를 찾지 못했습니다.")

            # 오른쪽 센서 Viewport에는 영향을 주지 않고 메인 Viewport만 연결한다.
            main_viewport.camera_path = FOLLOW_CAMERA_PRIM_PATH
            self._follow_viewport_api = main_viewport
            self._follow_camera_ready = True

            # 첫 프레임부터 드론이 화면 중앙에 보이도록 즉시 위치를 맞춘다.
            self._update_follow_viewport()

            print(
                "[INFO] Main Viewport forward follow camera enabled: "
                f"target={FOLLOW_DRONE_PRIM_PATH}, "
                f"camera={FOLLOW_CAMERA_PRIM_PATH}, "
                f"back_distance={FOLLOW_CAMERA_BACK_DISTANCE_M:.1f}m, "
                f"look_ahead={FOLLOW_CAMERA_LOOK_AHEAD_M:.1f}m"
            )
        except Exception as error:
            self._follow_viewport_api = None
            self._follow_camera_ready = False
            carb.log_error(
                "메인 Viewport 추적 카메라 설정 실패. "
                f"기본 Viewport를 유지합니다: {error}"
            )

    def _update_follow_viewport(self):
        """드론 뒤에서 실제 진행방향 앞쪽을 바라보도록 카메라를 갱신한다."""
        if not self._follow_camera_ready:
            return

        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(FOLLOW_DRONE_PRIM_PATH)
        if not target_prim.IsValid():
            return

        # 동적 Prim의 최신 World Transform을 읽기 위해 매 호출마다
        # 새로운 XformCache를 만든다.
        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        world_matrix = xform_cache.GetLocalToWorldTransform(target_prim)
        drone_translation = world_matrix.ExtractTranslation()
        drone_position = np.array(
            [
                float(drone_translation[0]),
                float(drone_translation[1]),
                float(drone_translation[2]),
            ],
            dtype=np.float64,
        )

        measured_direction_xy = None

        # 이전 프레임과 현재 프레임의 XY 위치 차이로 실제 진행방향을 구한다.
        if self._follow_previous_position is not None:
            movement_xy = (
                drone_position[:2]
                - self._follow_previous_position[:2]
            )
            movement_distance = float(np.linalg.norm(movement_xy))
            if movement_distance >= FOLLOW_CAMERA_MIN_MOVEMENT_M:
                measured_direction_xy = movement_xy / movement_distance

        # 아직 이동하지 않았거나 위치 변화가 너무 작으면 드론 body의
        # 로컬 +X축을 World 좌표로 변환해 전방 방향으로 사용한다.
        if measured_direction_xy is None:
            body_forward_world = world_matrix.TransformDir(
                Gf.Vec3d(1.0, 0.0, 0.0)
            )
            body_forward_xy = np.array(
                [
                    float(body_forward_world[0]),
                    float(body_forward_world[1]),
                ],
                dtype=np.float64,
            )
            body_forward_norm = float(np.linalg.norm(body_forward_xy))
            if body_forward_norm > 1.0e-6:
                measured_direction_xy = (
                    body_forward_xy / body_forward_norm
                )
            else:
                measured_direction_xy = np.array(
                    [1.0, 0.0],
                    dtype=np.float64,
                )

        # 방향이 갑자기 바뀔 때 Viewport가 급회전하지 않도록 이전 방향과
        # 새 방향을 보간한 뒤 다시 단위 벡터로 정규화한다.
        if self._follow_direction_xy is None:
            smoothed_direction_xy = measured_direction_xy
        else:
            alpha = FOLLOW_CAMERA_DIRECTION_SMOOTHING
            smoothed_direction_xy = (
                (1.0 - alpha) * self._follow_direction_xy
                + alpha * measured_direction_xy
            )
            smoothed_norm = float(np.linalg.norm(smoothed_direction_xy))
            if smoothed_norm > 1.0e-6:
                smoothed_direction_xy /= smoothed_norm
            else:
                smoothed_direction_xy = measured_direction_xy

        self._follow_direction_xy = smoothed_direction_xy
        self._follow_previous_position = drone_position.copy()

        # 카메라는 드론의 진행방향 반대쪽에 놓고, 드론보다 앞쪽의
        # LOOK_AHEAD 지점을 바라본다.
        eye = np.array(
            [
                drone_position[0]
                - smoothed_direction_xy[0]
                * FOLLOW_CAMERA_BACK_DISTANCE_M,
                drone_position[1]
                - smoothed_direction_xy[1]
                * FOLLOW_CAMERA_BACK_DISTANCE_M,
                drone_position[2] + FOLLOW_CAMERA_HEIGHT_M,
            ],
            dtype=np.float64,
        )
        target = np.array(
            [
                drone_position[0]
                + smoothed_direction_xy[0]
                * FOLLOW_CAMERA_LOOK_AHEAD_M,
                drone_position[1]
                + smoothed_direction_xy[1]
                * FOLLOW_CAMERA_LOOK_AHEAD_M,
                drone_position[2]
                + FOLLOW_CAMERA_TARGET_HEIGHT_M,
            ],
            dtype=np.float64,
        )

        set_camera_view(
            eye=eye,
            target=target,
            camera_prim_path=FOLLOW_CAMERA_PRIM_PATH,
            viewport_api=self._follow_viewport_api,
        )

    @staticmethod
    def _enable_viewport_camera_light(viewport_api, viewport_title):
        """센서 Viewport Render Product의 Camera Light를 활성화한다."""
        stage = omni.usd.get_context().get_stage()
        render_product_path = str(
            viewport_api.get_render_product_path()
        )
        render_product_prim = stage.GetPrimAtPath(render_product_path)
        if not render_product_prim.IsValid():
            raise RuntimeError(
                f"{viewport_title} Render Product를 찾지 못했습니다: "
                f"{render_product_path}"
            )

        attribute = render_product_prim.GetAttribute(
            "omni:rtx:scene:useViewLightingMode"
        )
        if not attribute.IsValid():
            attribute = render_product_prim.CreateAttribute(
                "omni:rtx:scene:useViewLightingMode",
                Sdf.ValueTypeNames.Bool,
                custom=True,
            )
        attribute.Set(True)
        print(
            f"[CHECK] Camera Light ON: {viewport_title}, "
            f"render_product={render_product_path}"
        )

    def run(self):
        self.timeline.play()

        while simulation_app.is_running():
            self.world.step(render=True)

            # 왼쪽 메인 Viewport의 3인칭 카메라가 지정 드론을 따라간다.
            self._update_follow_viewport()

            # Person의 Animation Graph가 갱신한 World 위치를 읽어
            # 다음 물리 스텝의 kinematic 충돌체 위치에 반영한다.
            self._sync_person_physics_proxies()

        carb.log_warn("Forest-rescue simulation is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    app = ForestRescueSimulation()
    app.run()


if __name__ == "__main__":
    main()
