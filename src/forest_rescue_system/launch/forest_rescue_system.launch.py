#!/usr/bin/env python3

"""산림 구조용 드론 3대의 ROS 2 노드를 한 번에 실행한다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
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
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = os.path.join(
        package_share,
        "config",
        "forest_rescue_multi.rviz",
    )

    nodes = [
        # 중앙 관리자를 먼저 시작하고 각 드론의 주기 상태를 받는다.
        Node(
            package="forest_rescue_system",
            executable="mission_manager",
            name="mission_manager_node",
            output="screen",
            parameters=[config],
        )
    ]

    for index in range(1, 4):
        suffix = f"{index:02d}"
        common = dict(
            package="forest_rescue_system",
            output="screen",
            parameters=[config],
        )
        nodes.extend(
            [
                Node(
                    **common,
                    executable="sensor_tf",
                    name=f"sensor_tf_{suffix}",
                ),
                Node(
                    **common,
                    executable="obstacle_monitor",
                    name=f"obstacle_monitor_{suffix}",
                ),
                Node(
                    **common,
                    executable="human_detector",
                    name=f"human_detector_{suffix}",
                    prefix=[detector_python],
                ),
                Node(
                    **common,
                    executable="victim_localizer",
                    name=f"victim_localizer_{suffix}",
                ),
                Node(
                    **common,
                    executable="drone_controller",
                    name=f"drone_controller_{suffix}",
                    prefix=[mavsdk_python],
                ),
            ]
        )

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="PYTHONNOUSERSITE", value="1"),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="3드론 통합 시스템 YAML 설정 파일",
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
                description="Ultralytics가 설치된 Python 실행 파일",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="세 드론 통합 RViz 화면을 함께 실행",
            ),
            *nodes,
            Node(
                package="rviz2",
                executable="rviz2",
                name="forest_rescue_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
