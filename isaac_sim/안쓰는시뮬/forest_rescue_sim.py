#!/usr/bin/env python3

"""
산림 조난자 탐지 드론의 Isaac Sim 기본 환경.

구성:
- Isaac Sim 5.1
- Pegasus Simulator 5.1
- PX4 SITL
- 3DR Iris 쿼드로터
- ROS 2 RGB/Depth/PointCloud 카메라
- RTX Rotary LiDAR
- 고정 위치 조난자 1명
"""

import math
import os
import numpy as np
from pathlib import Path

import carb
from isaacsim import SimulationApp


# 센서 및 시나리오 기본 설정값
CAMERA_PRIM_PATH = "/World/quadrotor/body/Camera"
CAMERA_FOCAL_LENGTH_MM = 18.0
CAMERA_DOWN_TILT_DEG = 30.0
# PX4 Yaw 0도에서 카메라는 대략 North(+Y)를 향한다. 5m 고도와
# 하향각 30도에서 지면 중심은 드론 전방 약 8.7m이다.
# 조난자는 초기 시야 바깥쪽에 두고, 0~8m로 확장한 수색 경로의
# 중반(East 4m, North 4m 부근)에 도착했을 때 시야로 들어오게 한다.
VICTIM_POSITION = [4.0, 4.0, 0.0]

# 산악 지형의 원점 높이가 0m가 아니라면 두 위치의 Z값을 함께 조정한다.
DRONE_POSITION = [0.0, 0.0, 0.07]

# 현재 스크립트가 있는 isaac_sim 디렉터리를 기준으로 USD 경로를 찾는다.
SCRIPT_DIR = Path(__file__).resolve().parent

FOREST_WORLD_PATH = (
    SCRIPT_DIR
    / "worlds"
    / "my_forest.usd"
)


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

# Script Editor 활성화
enable_extension("omni.kit.window.script_editor")

simulation_app.update()


# Extension을 불러온 뒤 깨끗한 Stage를 생성한다.
import omni.usd

omni.usd.get_context().new_stage()


import omni.timeline
from omni.isaac.core.world import World
from pxr import Gf, Usd, UsdGeom
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


class ForestRescueSimulation:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

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

        self._spawn_stationary_victim()
        self._spawn_iris()
        simulation_app.update()
        self._configure_drone_camera()

        self.world.reset()

        print("[OK] Forest-rescue baseline initialized")
        print("[INFO] RGB topic: /quadrotor/Camera/rgb")
        print("[INFO] Depth topic: /quadrotor/Camera/depth")
        print("[INFO] CameraInfo topic: /quadrotor/Camera/camera_info")
        print("[INFO] LiDAR topic: /point_cloud")

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

    def _spawn_stationary_victim(self):
        preferred_asset = "original_female_adult_business_02"
        available_assets = Person.get_character_asset_list()

        if not available_assets:
            raise RuntimeError(
                "No Pegasus person assets were found."
            )

        if preferred_asset in available_assets:
            selected_asset = preferred_asset
        else:
            selected_asset = available_assets[0]
            carb.log_warn(
                f"Preferred person asset was not found. "
                f"Using {selected_asset} instead."
            )

        # 기본 시스템에서는 고정 위치를 사용한다.
        # 환경 담당자는 이후 랜덤 배치 모듈로 교체할 수 있다.
        self.victim = Person(
            "victim_01",
            selected_asset,
            init_pos=VICTIM_POSITION,
            init_yaw=3.14,
        )

        print(f"[INFO] Victim asset: {selected_asset}")
        print(f"[INFO] Victim position: {VICTIM_POSITION}")

    def _spawn_iris(self):
        multirotor_config = MultirotorConfig()

        px4_config = PX4MavlinkBackendConfig(
            {
                "vehicle_id": 0,
                "px4_autolaunch": True,
                "px4_dir": self.pg.px4_path,
            }
        )

        # PX4가 MAVLink를 통해 드론을 제어한다.
        multirotor_config.backends = [
            PX4MavlinkBackend(px4_config)
        ]

        # Iris 기본 카메라의 RGB/Depth/PointCloud를 ROS 2로 발행한다.
        multirotor_config.graphs = [
            ROS2CameraGraph(
                "body/Camera",
                config={
                    "types": [
                        "rgb",
                        "depth",
                        "depth_pcl",
                        "camera_info",
                    ]
                },
            )
        ]

        # 드론 body 위쪽에 RTX Rotary LiDAR를 장착한다.
        multirotor_config.graphical_sensors = [
            Lidar(
                "lidar",
                config={
                    "frequency": 10.0,
                    "position": np.array(
                        [0.0, 0.0, 0.15]
                    ),
                    "orientation": np.array(
                        [0.0, 0.0, 0.0]
                    ),
                    "sensor_configuration": {
                        "sensor_configuration": "Example_Rotary"
                    },
                    "show_render": True,
                },
            )
        ]

        self.drone = Multirotor(
            "/World/quadrotor",
            ROBOTS["Iris"],
            0,
            DRONE_POSITION,
            Rotation.from_euler(
                "XYZ",
                [0.0, 0.0, 0.0],
                degrees=True,
            ).as_quat(),
            config=multirotor_config,
        )

    def _configure_drone_camera(self):
        """카메라 시야각과 하향 장착각을 Stage에 영구 적용한다."""
        stage = omni.usd.get_context().get_stage()
        camera_prim = stage.GetPrimAtPath(CAMERA_PRIM_PATH)
        if not camera_prim.IsValid():
            raise RuntimeError(
                f"Iris 카메라 Prim을 찾을 수 없습니다: {CAMERA_PRIM_PATH}"
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

        # Iris 카메라의 기존 자세를 보존하면서 카메라 local X축을
        # 기준으로 아래쪽 30도 회전을 추가한다.
        xformable = UsdGeom.Xformable(camera_prim)
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
                "Camera orient op가 없어 하향각은 적용하지 못했습니다."
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
            x, y, z, w = [float(value) for value in configured_xyzw]

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
            "[INFO] Camera setting: "
            f"focal={CAMERA_FOCAL_LENGTH_MM:.1f}mm, "
            f"horizontal_fov≈{horizontal_fov_deg:.1f}deg, "
            f"down_tilt={CAMERA_DOWN_TILT_DEG:.1f}deg"
        )

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
