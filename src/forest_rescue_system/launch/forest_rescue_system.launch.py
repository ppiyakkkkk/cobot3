#!/usr/bin/env python3

"""산림 조난자 탐지 드론 기본 시스템을 한 번에 실행한다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("forest_rescue_system")
    default_config = os.path.join(
        package_share,
        "config",
        "forest_rescue.yaml",
    )

    config = LaunchConfiguration("config")
    mavsdk_python = LaunchConfiguration("mavsdk_python")
    detector_python = LaunchConfiguration("detector_python")

    node_names = (
        ("sensor_tf", "sensor_tf_node"),
        ("obstacle_monitor", "obstacle_monitor_node"),
        ("human_detector", "human_detector_node"),
        ("victim_localizer", "victim_localizer_node"),
        ("drone_controller", "drone_controller_node"),
        ("mission_manager", "mission_manager_node"),
    )

    nodes = []
    for executable, node_name in node_names:
        options = dict(
            package="forest_rescue_system",
            executable=executable,
            name=node_name,
            output="screen",
            parameters=[config],
        )

        # MAVSDK가 필요한 드론 제어 노드만 pegasus_control을 사용한다.
        if executable == "drone_controller":
            options["prefix"] = [mavsdk_python]

        # YOLO와 MAVSDK가 설치된 pegasus_control Python을 사용한다.
        if executable == "human_detector":
            options["prefix"] = [detector_python]

        nodes.append(Node(**options))

    return LaunchDescription(
        [
            # ~/.local의 NumPy 2.x가 ROS Humble cv_bridge보다 먼저 로드되는 것을 막는다.
            SetEnvironmentVariable(
                name="PYTHONNOUSERSITE",
                value="1",
            ),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="통합 시스템 YAML 설정 파일",
            ),
            DeclareLaunchArgument(
                "mavsdk_python",
                default_value=os.path.expanduser(
                    "~/venvs/pegasus_control/bin/python"
                ),
                description="MAVSDK가 설치된 Python 실행 파일",
            ),
            DeclareLaunchArgument(
                "detector_python",
                default_value=os.path.expanduser(
                    "~/venvs/pegasus_control/bin/python"
                ),
                description=(
                    "Ultralytics와 ROS 영상 의존성이 설치된 탐지용 Python"
                ),
            ),
            *nodes,
        ]
    )
