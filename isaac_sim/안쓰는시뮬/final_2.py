#!/usr/bin/env python3

"""
산림 조난자 탐지 드론 시뮬레이션 (final_2)

간단한 구조로 정리:
- 조난자 스폰 위치 3개 후보 중 하나를 랜덤으로 선택한다.
- 드론 3대를 생성하고 각 드론마다 카메라를 붙인다.
- 구조자는 드론과 가까운 위치에 배치한다.
"""

import math
import os
from pathlib import Path

import carb
import numpy as np
from isaacsim import SimulationApp


# ---------------------------------------------------------------------------
# 기본 설정
# ---------------------------------------------------------------------------

CAMERA_FOCAL_LENGTH_MM = 10.0
CAMERA_DOWN_TILT_DEG = 30.0

# 조난자 3개 스폰 후보 위치
VICTIM_SPAWN_OPTIONS = [
    np.array([33.0, 29.0, 13.7], dtype=np.float64),
    np.array([33.0, -22.0, 50.6], dtype=np.float64),
    np.array([-0.9, -1.8, -0.9], dtype=np.float64),
]

PERSON_GROUND_OFFSET_M = 0.05
PERSON_RANDOM_SEED = None

# 드론은 조난자와는 별개로 간단히 배치한다.
DRONE_COUNT = 3
DRONE_SPACING_M = 2.0
DRONE_HEIGHT_AGL_M = 6.0
DRONE_BASE_POSITION = np.array([8.0, -8.0, 0.0], dtype=np.float64)
RESCUER_OFFSET_FROM_DRONE = np.array([0.6, -0.4, 0.0], dtype=np.float64)

SCRIPT_DIR = Path(__file__).resolve().parent


def resolve_forest_world_path():
    env_path = os.environ.get("FOREST_WORLD_PATH")
    candidates = []
    if env_path:
        candidates.append(Path(env_path).expanduser())

    candidates.extend(
        [
            SCRIPT_DIR / "worlds" / "my_forest.usd",
            SCRIPT_DIR / "my_forest.usd",
            Path.cwd() / "worlds" / "my_forest.usd",
            Path.cwd() / "my_forest.usd",
        ]
    )

    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()

    return None


simulation_app = SimulationApp({"headless": False})


from isaacsim.core.utils.extensions import enable_extension


def try_enable_extension(extension_id):
    try:
        enable_extension(extension_id)
        print(f"[EXT] enabled: {extension_id}")
    except Exception as error:
        carb.log_warn(
            f"Extension을 활성화하지 못했습니다: {extension_id}\n{error}"
        )


try_enable_extension("isaacsim.ros2.bridge")
try_enable_extension("isaacsim.sensors.rtx")
try_enable_extension("isaacsim.sensors.camera.ui")
try_enable_extension("isaacsim.util.camera_inspector")
try_enable_extension("omni.anim.people")
try_enable_extension("omni.anim.navigation")

simulation_app.update()

import omni.usd

omni.usd.get_context().new_stage()

import omni.timeline
from omni.isaac.core.world import World
from pxr import Gf, Usd, UsdGeom
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


class TerrainHeightField:
    def __init__(self, stage):
        self._stage = stage
        self._terrain_prim = self._find_terrain_mesh()
        self._build_interpolator()

    def _find_terrain_mesh(self):
        meshes = [prim for prim in self._stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
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
            raise RuntimeError(f"Terrain Mesh 정점이 부족합니다: {self._terrain_prim.GetPath()}")

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        world_matrix = xform_cache.GetLocalToWorldTransform(self._terrain_prim)
        world_points = np.asarray(
            [
                tuple(
                    world_matrix.Transform(
                        Gf.Vec3d(float(point[0]), float(point[1]), float(point[2]))
                    )
                )
                for point in local_points
            ],
            dtype=np.float64,
        )

        xy = world_points[:, :2]
        z = world_points[:, 2]
        rounded_xy = np.round(xy, decimals=5)
        _, unique_indices = np.unique(rounded_xy, axis=0, return_index=True)
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

    def height(self, x, y):
        if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max):
            raise ValueError(f"좌표가 Terrain 범위를 벗어났습니다: X={x:.2f}, Y={y:.2f}")

        value = self._linear(float(x), float(y))
        value = float(np.asarray(value))
        if not np.isfinite(value):
            value = float(self._nearest(float(x), float(y)))
        if not np.isfinite(value):
            raise RuntimeError(f"Terrain 높이를 계산하지 못했습니다: X={x:.2f}, Y={y:.2f}")
        return value

    def slope_deg(self, x, y, sample_distance=0.5):
        x1 = np.clip(x - sample_distance, self.x_min, self.x_max)
        x2 = np.clip(x + sample_distance, self.x_min, self.x_max)
        y1 = np.clip(y - sample_distance, self.y_min, self.y_max)
        y2 = np.clip(y + sample_distance, self.y_min, self.y_max)
        dz_dx = (self.height(x2, y) - self.height(x1, y)) / max(float(x2 - x1), 1e-6)
        dz_dy = (self.height(x, y2) - self.height(x, y1)) / max(float(y2 - y1), 1e-6)
        slope = math.sqrt(dz_dx * dz_dx + dz_dy * dz_dy)
        return math.degrees(math.atan(slope))

    def random_surface_position(self, rng, x_range, y_range, max_slope_deg, attempts=400):
        x_low = max(float(x_range[0]), self.x_min)
        x_high = min(float(x_range[1]), self.x_max)
        y_low = max(float(y_range[0]), self.y_min)
        y_high = min(float(y_range[1]), self.y_max)
        if x_low >= x_high or y_low >= y_high:
            raise RuntimeError("PERSON_INCLUDE 영역과 Terrain 영역이 겹치지 않습니다.")
        for _ in range(attempts):
            x = float(rng.uniform(x_low, x_high))
            y = float(rng.uniform(y_low, y_high))
            z = self.height(x, y)
            if self.slope_deg(x, y) <= max_slope_deg:
                return np.array([x, y, z], dtype=np.float64)
        raise RuntimeError("사람이 설 수 있는 완만한 지점을 찾지 못했습니다.")


class ForestRescueSimulation:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.rng = np.random.default_rng(PERSON_RANDOM_SEED)
        self.pg = PegasusInterface()

        px4_path = Path(os.environ.get("PX4_AUTOPILOT_PATH", str(Path.home() / "PX4-Autopilot"))).expanduser().resolve()
        px4_binary = px4_path / "build/px4_sitl_default/bin/px4"
        if not px4_binary.is_file():
            raise RuntimeError(f"PX4 SITL binary was not found. Expected path: {px4_binary}")

        self.pg.set_px4_path(str(px4_path))

        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.forest_world_path = resolve_forest_world_path()
        if not self.forest_world_path:
            raise FileNotFoundError(
                "산림 환경 USD 파일을 찾을 수 없습니다.\n"
                "1) 환경 변수 FOREST_WORLD_PATH를 설정하세요.\n"
                "2) my_forest.usd 파일을 Downloads/worlds 또는 Downloads에 두세요."
            )

        print(f"[INFO] Forest world: {self.forest_world_path}")
        self.pg.load_environment(str(self.forest_world_path))
        simulation_app.update()

        self._verify_loaded_environment()
        self._fit_viewport_to_environment()

        stage = omni.usd.get_context().get_stage()
        self.terrain = TerrainHeightField(stage)

        self._select_victim_spawn()
        self._spawn_victim()
        self._spawn_iris_above_terrain()
        self._spawn_rescuer()

        simulation_app.update()
        self._configure_drone_camera()
        self.world.reset()

        print("[OK] final_2 simulation initialized")
        print(f"[INFO] Selected victim spawn index: {self.selected_victim_index}")
        print(f"[INFO] Victim spawn: ({self.selected_victim_position[0]:.2f}, {self.selected_victim_position[1]:.2f}, {self.selected_victim_position[2]:.2f})")

    def _verify_loaded_environment(self):
        stage = omni.usd.get_context().get_stage()
        root_layer = stage.GetRootLayer()
        root_path = root_layer.realPath or root_layer.identifier
        print(f"[CHECK] Stage root layer: {root_path}")

    def _get_environment_root_prim(self):
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
        target_prim = self._get_environment_root_prim()
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
        aligned_range = bbox_cache.ComputeWorldBound(target_prim).ComputeAlignedRange()
        if aligned_range.IsEmpty():
            self.pg.set_viewport_camera([30.0, 30.0, 25.0], [0.0, 0.0, 0.0])
            return
        minimum = aligned_range.GetMin()
        maximum = aligned_range.GetMax()
        center = 0.5 * (minimum + maximum)
        size = maximum - minimum
        largest_dimension = max(float(size[0]), float(size[1]), float(size[2]))
        camera_distance = max(20.0, largest_dimension * 0.75)
        eye = [float(center[0]) + camera_distance, float(center[1]) + camera_distance, max(float(maximum[2]) + camera_distance * 0.35, float(center[2]) + camera_distance * 0.65)]
        target = [float(center[0]), float(center[1]), float(center[2])]
        self.pg.set_viewport_camera(eye, target)

    def _select_victim_spawn(self):
        self.selected_victim_index = int(self.rng.integers(0, len(VICTIM_SPAWN_OPTIONS)))
        self.selected_victim_position = VICTIM_SPAWN_OPTIONS[self.selected_victim_index].copy()
        self.selected_victim_position[2] = self.terrain.height(float(self.selected_victim_position[0]), float(self.selected_victim_position[1])) + PERSON_GROUND_OFFSET_M
        return self.selected_victim_position

    def _spawn_victim(self):
        preferred_asset = "original_female_adult_business_02"
        available_assets = Person.get_character_asset_list()
        if not available_assets:
            raise RuntimeError("No Pegasus person assets were found.")
        selected_asset = preferred_asset if preferred_asset in available_assets else available_assets[0]

        self.victim = Person(
            "victim_01",
            selected_asset,
            init_pos=self.selected_victim_position.tolist(),
            init_yaw=0.0,
        )
        print(f"[INFO] Victim asset: {selected_asset}")

    def _spawn_iris_above_terrain(self):
        self.drones = []

        for drone_idx in range(DRONE_COUNT):
            drone_x = float(DRONE_BASE_POSITION[0] + drone_idx * DRONE_SPACING_M)
            drone_y = float(DRONE_BASE_POSITION[1])
            ground_z = self.terrain.height(drone_x, drone_y)
            drone_position = [drone_x, drone_y, ground_z + DRONE_HEIGHT_AGL_M]

            multirotor_config = MultirotorConfig()
            px4_config = PX4MavlinkBackendConfig({
                "vehicle_id": drone_idx,
                "px4_autolaunch": True,
                "px4_dir": self.pg.px4_path,
            })
            multirotor_config.backends = [PX4MavlinkBackend(px4_config)]
            multirotor_config.graphs = [ROS2CameraGraph("body/Camera", config={"types": ["rgb", "depth", "depth_pcl", "camera_info"]})]
            multirotor_config.graphical_sensors = [
                Lidar(
                    "lidar",
                    config={
                        "frequency": 10.0,
                        "position": np.array([0.0, 0.0, 0.15]),
                        "orientation": np.array([0.0, 0.0, 0.0]),
                        "sensor_configuration": {"sensor_configuration": "Example_Rotary"},
                        "show_render": True,
                    },
                )
            ]

            drone = Multirotor(
                f"/World/quadrotor_{drone_idx}",
                ROBOTS["Iris"],
                drone_idx,
                drone_position,
                Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
                config=multirotor_config,
            )
            self.drones.append(drone)
            print(f"[INFO] Drone {drone_idx} spawn: ({drone_position[0]:.2f}, {drone_position[1]:.2f}, {drone_position[2]:.2f})")

        self.drone = self.drones[0]

    def _spawn_rescuer(self):
        first_drone_xy = np.array([DRONE_BASE_POSITION[0], DRONE_BASE_POSITION[1]], dtype=np.float64)
        rescuer_xy = first_drone_xy + RESCUER_OFFSET_FROM_DRONE[:2]
        rescuer_ground_z = self.terrain.height(float(rescuer_xy[0]), float(rescuer_xy[1]))
        rescuer_position = np.array([rescuer_xy[0], rescuer_xy[1], rescuer_ground_z + PERSON_GROUND_OFFSET_M], dtype=np.float64)

        preferred_asset = "original_female_adult_business_02"
        available_assets = Person.get_character_asset_list()
        selected_asset = preferred_asset if preferred_asset in available_assets else available_assets[0]

        self.rescuer = Person(
            "rescuer_01",
            selected_asset,
            init_pos=rescuer_position.tolist(),
            init_yaw=0.0,
        )
        print(f"[INFO] Rescuer spawn: ({rescuer_position[0]:.2f}, {rescuer_position[1]:.2f}, {rescuer_position[2]:.2f})")

    def _configure_drone_camera(self):
        stage = omni.usd.get_context().get_stage()

        for drone_idx in range(DRONE_COUNT):
            camera_prim_path = f"/World/quadrotor_{drone_idx}/body/Camera"
            camera_prim = stage.GetPrimAtPath(camera_prim_path)
            if not camera_prim.IsValid():
                carb.log_warn(f"드론 {drone_idx}의 카메라 Prim을 찾을 수 없어 건너뜁니다: {camera_prim_path}")
                continue

            camera = UsdGeom.Camera(camera_prim)
            camera.GetFocalLengthAttr().Set(CAMERA_FOCAL_LENGTH_MM)
            horizontal_aperture = float(camera.GetHorizontalApertureAttr().Get())
            horizontal_fov_deg = math.degrees(2.0 * math.atan(horizontal_aperture / (2.0 * CAMERA_FOCAL_LENGTH_MM)))
            print(f"[INFO] Drone {drone_idx} camera: focal={CAMERA_FOCAL_LENGTH_MM:.1f}mm, horizontal_fov≈{horizontal_fov_deg:.1f}deg")

    def run(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
        carb.log_warn("Forest-rescue simulation is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    app = ForestRescueSimulation()
    app.run()


if __name__ == "__main__":
    main()
