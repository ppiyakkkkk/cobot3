#!/usr/bin/env python3

"""산림 조난자 탐지 드론 시뮬레이션 실행 진입점.

실행 명령은 기존과 동일하다.

    isaac_python final_24.py

긴 구현은 역할별 모듈로 분리했다.
- sim_config.py: 모든 설정값과 경로
- sim_terrain.py: Terrain 높이 보간과 RViz Terrain/환경 Mesh 추출
- sim_utils.py: 수색 경로, Ground Truth JSON, 환경 공통 처리
- sim_people.py: 조난자·구조자와 물리 충돌 프록시
- sim_drone.py: PX4 Iris, ROS 카메라, RTX LiDAR
- sim_viewports.py: 센서 Viewport 도킹과 메인 추적 카메라

중요:
Isaac Sim/Pegasus 모듈은 대부분 SimulationApp이 생성된 뒤 import해야 한다.
따라서 이 파일에서는 SimulationApp 생성과 extension 활성화를 먼저 수행하고,
그 다음 역할별 모듈을 import한다.
"""

import os
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

# sim_config.py는 표준 라이브러리만 사용하므로 SimulationApp 전에도 안전하다.
from sim_config import (
    DRONE_CONFIGS,
    FOREST_WORLD_PATH,
    GENERATED_ENVIRONMENT_MESH_PATH,
    GENERATED_GROUND_TRUTH_PATH,
    GENERATED_TERRAIN_MESH_PATH,
    RVIZ_TERRAIN_SAMPLE_SPACING_M,
)


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


# 아래 모듈들은 반드시 SimulationApp과 필요한 extension 생성 이후 import한다.
import carb
import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from sim_drone import DroneManager, print_camera_direction_debug
from sim_people import PeopleManager
from sim_terrain import EnvironmentMeshExporter, TerrainHeightField
from sim_utils import (
    SceneEnvironmentManager,
    SearchPlanGenerator,
    remove_previous_generated_files,
)
from sim_viewports import ViewportManager


class ForestRescueSimulation:
    """각 역할별 관리 객체를 조립하고 전체 시뮬레이션 수명주기를 제어한다."""

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.rng = np.random.default_rng()

        # 이전 실행의 실제 스폰 위치와 Mesh가 RViz에 잠시 남지 않도록 제거한다.
        remove_previous_generated_files(
            GENERATED_GROUND_TRUTH_PATH,
            GENERATED_TERRAIN_MESH_PATH,
            GENERATED_ENVIRONMENT_MESH_PATH,
        )

        # ------------------------------------------------------------------
        # Pegasus / PX4 / Isaac World 초기화
        # ------------------------------------------------------------------
        self.pg = PegasusInterface()

        # 환경 변수로 PX4 경로를 바꿀 수 있으며, 없으면 기본 경로를 사용한다.
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

        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # ------------------------------------------------------------------
        # 산림 USD 로드와 RViz용 환경 데이터 생성
        # ------------------------------------------------------------------
        if not FOREST_WORLD_PATH.is_file():
            raise FileNotFoundError(
                "산림 환경 USD 파일을 찾을 수 없습니다.\n"
                f"Expected path: {FOREST_WORLD_PATH}"
            )

        print(f"[INFO] Forest world: {FOREST_WORLD_PATH}")
        self.pg.load_environment(str(FOREST_WORLD_PATH))
        simulation_app.update()

        self.scene_manager = SceneEnvironmentManager(self.pg)
        self.scene_manager.configure_sky_background()
        simulation_app.update()
        self.scene_manager.verify_loaded_environment()
        self.scene_manager.fit_viewport_to_environment()

        stage = omni.usd.get_context().get_stage()
        self.terrain = TerrainHeightField(stage)
        self.terrain.write_rviz_terrain_mesh(
            GENERATED_TERRAIN_MESH_PATH,
            RVIZ_TERRAIN_SAMPLE_SPACING_M,
        )
        EnvironmentMeshExporter(stage).write(
            GENERATED_ENVIRONMENT_MESH_PATH
        )

        search_plan_generator = SearchPlanGenerator(self.terrain)
        search_plan_generator.write_generated_search_plan()

        # ------------------------------------------------------------------
        # 사람과 드론 생성
        # ------------------------------------------------------------------
        self.people_manager = PeopleManager(
            terrain=self.terrain,
            rng=self.rng,
            test_victim_spawn_world_enu=(
                search_plan_generator.test_victim_spawn_world_enu
            ),
        )
        self.people_manager.spawn_people()
        self.victim = self.people_manager.victim
        self.rescuer = self.people_manager.rescuer

        self.drone_manager = DroneManager(self.pg)
        self.drone_manager.spawn_iris()
        self.drones = self.drone_manager.drones
        self.drone = self.drone_manager.drone

        simulation_app.update()
        self.drone_manager.configure_drone_cameras()

        # 모든 Prim과 센서가 준비된 뒤 물리 World를 한 번 초기화한다.
        self.world.reset()
        simulation_app.update()

        # 드론 body 전방과 실제 RGB/Depth 카메라 광축을 수치로 검증한다.
        print_camera_direction_debug()

        # ------------------------------------------------------------------
        # Viewport 구성
        # ------------------------------------------------------------------
        self.viewport_manager = ViewportManager(simulation_app)
        self.viewport_manager.create_docked_camera_viewports()

        # 센서 Render Product 생성 이후에도 RTX 배경 색상을 다시 적용한다.
        self.scene_manager.configure_sky_background()
        simulation_app.update()
        self.viewport_manager.setup_follow_viewport()

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

    def run(self):
        """물리, 렌더링, 추적 카메라와 사람 충돌체를 매 프레임 갱신한다."""
        self.timeline.play()

        while simulation_app.is_running():
            self.world.step(render=True)

            # 왼쪽 메인 Viewport의 3인칭 카메라가 지정 드론을 따라간다.
            self.viewport_manager.update_follow_viewport()

            # Person Animation Graph의 위치를 다음 물리 스텝 충돌체에 반영한다.
            self.people_manager.sync_person_physics_proxies()

        carb.log_warn("Forest-rescue simulation is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    app = ForestRescueSimulation()
    app.run()


if __name__ == "__main__":
    main()
