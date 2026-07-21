#!/usr/bin/env python3

"""
산림 조난자 탐지 드론 시뮬레이션.

변경 사항:
- 지정된 3개 조난자 좌표 중 1곳을 무작위로 선택해 조난자 1명을 정지 상태로 생성한다.
- 시작 좌표 주변에 Iris 드론 3대를 약 1.5 m 간격으로 생성한다.
- 각 드론의 기본 카메라를 유지하고 초점거리를 10 mm로 설정한다.
- 세 카메라의 드론 body 기준 상대 위치를 모두 (0, 0, 0)으로 맞춘다.
- 드론 무리 옆 산 표면에 구조자 1명을 정지 상태로 생성한다.
- 조난자와 구조자에 캡슐형 물리 충돌 프록시를 적용한다.
- 사람이 걸을 때 물리 충돌 프록시가 실제 캐릭터 위치를 따라가도록 동기화한다.
- 첫 번째 드론도 quadrotor_01 이름을 사용한다.
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

# 1번 드론은 지정한 시작 좌표에 두고, 나머지 두 대는 좌우 1.5 m에 배치한다.
DRONE_CONFIGS = [
    ("/World/quadrotor_01", 0, [-34.0, 40.0, 31.0]),
    ("/World/quadrotor_02", 1, [-32.5, 40.0, 31.0]),
    ("/World/quadrotor_03", 2, [-35.5, 40.0, 31.0]),
]

CAMERA_PRIM_PATHS = [
    f"{prim_path}/body/Camera"
    for prim_path, _, _ in DRONE_CONFIGS
]

# 아래 세 좌표 중 한 곳에 조난자 1명을 무작위 생성한다.
VICTIM_SPAWN_POSITIONS = [
    [33.0, 29.0, 13.7],
    [33.0, -22.0, 50.6],
    [-0.9, -1.8, -0.9],
]

# 구조자는 드론 중심에서 Y 방향으로 1.5 m 떨어진 위치에 생성한다.
# 구조자의 발 높이는 첫 번째 드론의 초기 World Z와 동일하게 맞춘다.
RESCUER_XY = (-34.0, 38.5)
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
SAFE_RETURN_CLEARANCE_M = 12.0

SCRIPT_DIR = Path(__file__).resolve().parent
FOREST_WORLD_PATH = SCRIPT_DIR / "worlds" / "my_forest.usd"
GENERATED_SEARCH_PLAN_PATH = SCRIPT_DIR / "generated_search_plan.json"


# 대부분의 Isaac Sim 모듈보다 먼저 SimulationApp을 생성해야 한다.
simulation_app = SimulationApp({"headless": False})


from isaacsim.core.utils.extensions import enable_extension

# ROS 2 sensor topic 발행에 필요한 extension을 활성화한다.
enable_extension("isaacsim.ros2.bridge")

# RTX LiDAR 생성에 필요한 extension을 활성화한다.
enable_extension("isaacsim.sensors.rtx")

# 카메라 상태 확인에 필요한 UI extension을 활성화한다.
enable_extension("isaacsim.sensors.camera.ui")
enable_extension("isaacsim.util.camera_inspector")
enable_extension("omni.kit.viewport.window")

simulation_app.update()


# Extension을 불러온 뒤 깨끗한 Stage를 생성한다.
import omni.usd

omni.usd.get_context().new_stage()


import omni.timeline
import omni.replicator.core as rep
from omni.isaac.core.world import World
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
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


class ForestRescueSimulation:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.rng = np.random.default_rng()
        self._person_physics_proxies = {}

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

        self._verify_loaded_environment()
        self._fit_viewport_to_environment()

        stage = omni.usd.get_context().get_stage()
        self.terrain = TerrainHeightField(stage)
        self._write_generated_search_plan()

        self._spawn_people()
        self._spawn_iris()
        simulation_app.update()
        self._configure_drone_cameras()

        self.world.reset()
        simulation_app.update()
        self._create_docked_camera_viewports()

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
            "format_version": 1,
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
            "drones": {},
        }

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
            waypoints = []

            def append_waypoint(world_x, world_y):
                """지형 상공의 일정 고도를 유지하는 NED 경유점을 추가한다."""
                world_x = float(world_x)
                world_y = float(world_y)
                world_z = (
                    self.terrain.height(world_x, world_y)
                    + SEARCH_CLEARANCE_M
                )
                waypoint = {
                    "north_m": world_y - home_y,
                    "east_m": world_x - home_x,
                    "down_m": -(world_z - home_z),
                    "world_enu": [world_x, world_y, world_z],
                }
                if waypoints:
                    previous = waypoints[-1]["world_enu"]
                    if (
                        abs(previous[0] - world_x) < 1.0e-6
                        and abs(previous[1] - world_y) < 1.0e-6
                    ):
                        waypoints[-1] = waypoint
                        return
                waypoints.append(waypoint)

            # 시작점에서 담당 구역까지 한 번에 대각선으로 가면 중간 능선을
            # 놓칠 수 있다. 먼저 Y가 높은 외곽으로 이동한 뒤, 왼쪽 경계를
            # 따라 구역 입구까지 지형 고도를 샘플링하며 진입한다.
            ingress_y = min(max(home_y, y_low), y_high)
            for world_x in self._sample_axis(
                home_x,
                x_low,
                SEARCH_SAMPLE_SPACING_M,
            )[1:]:
                append_waypoint(world_x, ingress_y)
            for world_y in self._sample_axis(
                ingress_y,
                zone_max_y,
                SEARCH_SAMPLE_SPACING_M,
            )[1:]:
                append_waypoint(x_low, world_y)

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
                    append_waypoint(world_x, world_y)

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

        victim_index = int(self.rng.integers(len(VICTIM_SPAWN_POSITIONS)))
        victim_candidate = VICTIM_SPAWN_POSITIONS[victim_index]
        victim_x = float(victim_candidate[0])
        victim_y = float(victim_candidate[1])

        # 후보 좌표의 Z를 그대로 사용하면 지형 표면과 차이가 생길 수 있다.
        # 실제 Terrain 높이를 다시 읽어 사람의 발을 지면 위에 배치한다.
        victim_ground_z = self.terrain.height(victim_x, victim_y)
        victim_position = [
            victim_x,
            victim_y,
            victim_ground_z + PERSON_GROUND_CLEARANCE_M,
        ]

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
            f"[INFO] Spawned victim at candidate {victim_index + 1}: "
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
        """드론 카메라 3개를 메인 Viewport 오른쪽에 고정 배치한다.

        배치 구조는 오른쪽 영역의 상단에 Camera 01, 하단을 좌우로
        나눠 Camera 02와 Camera 03을 두는 형태다. UI API가 바뀌거나
        Viewport 생성에 실패하더라도 시뮬레이션 자체는 계속 실행한다.
        """
        try:
            import omni.ui as ui
            from omni.kit.viewport.utility import (
                create_viewport_window,
                get_active_viewport_window,
            )

            main_window = get_active_viewport_window(
                window_name="Viewport"
            )
            if main_window is None:
                raise RuntimeError("기본 Viewport 창을 찾지 못했습니다.")

            main_title = getattr(main_window, "title", "Viewport")
            viewport_specs = [
                ("Drone Camera 01", CAMERA_PRIM_PATHS[0]),
                ("Drone Camera 02", CAMERA_PRIM_PATHS[1]),
                ("Drone Camera 03", CAMERA_PRIM_PATHS[2]),
            ]
            self.camera_viewport_windows = []

            for title, camera_path in viewport_specs:
                viewport_window = create_viewport_window(
                    name=title,
                    width=640,
                    height=360,
                    camera_path=Sdf.Path(camera_path),
                )
                if viewport_window is None:
                    raise RuntimeError(
                        f"Viewport 생성 실패: {title}"
                    )
                # 생성 시 camera_path 적용이 지연되는 Kit 버전에서도
                # 확실히 해당 드론 카메라를 사용하도록 한 번 더 지정한다.
                viewport_window.viewport_api.camera_path = Sdf.Path(
                    camera_path
                )
                self.camera_viewport_windows.append(viewport_window)
                simulation_app.update()

            camera_01_title = viewport_specs[0][0]
            camera_02_title = viewport_specs[1][0]
            camera_03_title = viewport_specs[2][0]

            # 메인 화면 오른쪽에 Camera 01 영역을 만든다.
            if not ui.dock_window_in_window(
                camera_01_title,
                main_title,
                ui.DockPosition.RIGHT,
                0.38,
            ):
                raise RuntimeError("Camera 01 도킹에 실패했습니다.")
            simulation_app.update()

            # Camera 01 아래쪽에 Camera 02를 배치한다.
            if not ui.dock_window_in_window(
                camera_02_title,
                camera_01_title,
                ui.DockPosition.BOTTOM,
                0.50,
            ):
                raise RuntimeError("Camera 02 도킹에 실패했습니다.")
            simulation_app.update()

            # 하단 Camera 02 영역의 오른쪽에 Camera 03을 배치한다.
            if not ui.dock_window_in_window(
                camera_03_title,
                camera_02_title,
                ui.DockPosition.RIGHT,
                0.50,
            ):
                raise RuntimeError("Camera 03 도킹에 실패했습니다.")
            simulation_app.update()

            print(
                "[INFO] Camera Viewports docked: "
                "01=right/top, 02=right/bottom-left, "
                "03=right/bottom-right"
            )
        except Exception as error:
            self.camera_viewport_windows = []
            carb.log_warn(
                "드론 카메라 Viewport 자동 배치 실패. "
                f"기본 시뮬레이션은 계속 실행합니다: {error}"
            )

    def run(self):
        self.timeline.play()

        while simulation_app.is_running():
            self.world.step(render=True)
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
