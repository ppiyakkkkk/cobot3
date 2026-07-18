#!/usr/bin/env python3

"""
Forest-rescue simulation baseline.

Components:
- Isaac Sim 5.1
- Pegasus Simulator 5.1
- PX4 SITL
- Iris quadrotor
- ROS 2 RGB camera
- One stationary victim
"""

import os
import numpy as np
from pathlib import Path

import carb
from isaacsim import SimulationApp


# SimulationApp must be created before importing most Isaac Sim modules.
simulation_app = SimulationApp({"headless": False})


# Enable the ROS 2 Bridge explicitly.
from isaacsim.core.utils.extensions import enable_extension

# ROS 2 sensor topic 발행에 필요한 extension을 활성화한다.
enable_extension("isaacsim.ros2.bridge")

# RTX LiDAR 생성에 필요한 extension을 활성화한다.
enable_extension("isaacsim.sensors.rtx")

# 카메라 상태 확인에 필요한 UI extension을 활성화한다.
enable_extension("isaacsim.sensors.camera.ui")
enable_extension("isaacsim.util.camera_inspector")

simulation_app.update()


# The Pegasus people example recreates the stage after loading extensions.
import omni.usd

omni.usd.get_context().new_stage()


import omni.timeline
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
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

        # Pegasus interface
        self.pg = PegasusInterface()

        # Set and validate the PX4 path.
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

        # Create Isaac Sim world.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load a simple environment for the first integration test.
        self.pg.load_asset(
            SIMULATION_ENVIRONMENTS["Rough Plane"],
            "/World/layout",
        )

        self._spawn_stationary_victim()
        self._spawn_iris()

        # Viewport camera only affects what is shown in the Isaac Sim window.
        self.pg.set_viewport_camera(
            [7.0, 7.0, 5.0],
            [0.0, 0.0, 0.0],
        )

        self.world.reset()

        print("[OK] Forest-rescue baseline initialized")
        print("[INFO] RGB topic: /quadrotor/Camera/rgb")
        print("[INFO] CameraInfo topic: /quadrotor/Camera/camera_info")

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

        # Fixed position for the first test.
        # Random placement will be added after this baseline works.
        self.victim = Person(
            "victim_01",
            selected_asset,
            init_pos=[4.0, 0.0, 0.0],
            init_yaw=3.14,
        )

        print(f"[INFO] Victim asset: {selected_asset}")
        print("[INFO] Victim position: [4.0, 0.0, 0.0]")

    def _spawn_iris(self):
        multirotor_config = MultirotorConfig()

        px4_config = PX4MavlinkBackendConfig(
            {
                "vehicle_id": 0,
                "px4_autolaunch": True,
                "px4_dir": self.pg.px4_path,
            }
        )

        # PX4 controls the drone through MAVLink.
        multirotor_config.backends = [
            PX4MavlinkBackend(px4_config)
        ]

        # Publish the existing Iris camera through ROS 2.
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
            [0.0, 0.0, 0.07],
            Rotation.from_euler(
                "XYZ",
                [0.0, 0.0, 0.0],
                degrees=True,
            ).as_quat(),
            config=multirotor_config,
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