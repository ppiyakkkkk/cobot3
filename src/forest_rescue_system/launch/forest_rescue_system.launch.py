#!/usr/bin/env python3

"""드론 수와 운용 모드에 맞춰 산림 구조 ROS 2 노드를 실행한다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MIN_DRONE_COUNT = 1
MAX_DRONE_COUNT = 4
DEFAULT_DRONE_COUNT = 3
DEFAULT_OPERATION_MODE = "rescue_search"
SUPPORTED_OPERATION_MODES = (
    "rescue_search",
    "mapping_3d",
)

RESCUE_OBSTACLE_STATES = [
    "INITIAL_TAKEOFF",
    "INITIAL_HOVER",
    "READY",
    "SEARCHING",
    "COOP_SEARCH_PREPARING",
    "COOP_SEARCH_TRANSIT",
    "COOP_SEARCHING",
    "COOP_SEARCH_COMPLETE",
    "VICTIM_DETECTED",
    "RETURNING_NO_VICTIM",
    "COMPLETE",
    "COMPLETE_WITH_LANDING_ERROR",
    "MISSION_FAILED",
]

MAPPING_OBSTACLE_STATES = [
    "INITIAL_TAKEOFF",
    "INITIAL_HOVER",
    "MAPPING_READY",
    "MAPPING_PREPARING",
    "MAPPING",
    "MAPPING_PAUSED",
    "MAPPING_RETURNING",
    "MAPPING_NOT_IMPLEMENTED",
    "MAPPING_COMPLETE",
    "MAPPING_COMPLETE_WITH_ERROR",
    "MAPPING_FAILED",
]


def _parse_drone_count(raw_value):
    try:
        drone_count = int(str(raw_value).strip())
    except ValueError as error:
        raise RuntimeError(
            f"drone_count는 정수여야 합니다: {raw_value!r}"
        ) from error

    if not MIN_DRONE_COUNT <= drone_count <= MAX_DRONE_COUNT:
        raise RuntimeError(
            f"drone_count는 {MIN_DRONE_COUNT}~{MAX_DRONE_COUNT} "
            f"범위여야 합니다: {drone_count}"
        )
    return drone_count


def _parse_operation_mode(raw_value):
    operation_mode = str(raw_value).strip().lower()
    if operation_mode not in SUPPORTED_OPERATION_MODES:
        supported = ", ".join(SUPPORTED_OPERATION_MODES)
        raise RuntimeError(
            f"operation_mode는 다음 중 하나여야 합니다: {supported}. "
            f"입력값={raw_value!r}"
        )
    return operation_mode


def _manager_node(
    executable,
    name,
    config,
    use_sim_time,
    operation_mode,
    drone_ids,
):
    return Node(
        package="forest_rescue_system",
        executable=executable,
        name=name,
        output="screen",
        parameters=[
            config,
            {
                "use_sim_time": use_sim_time,
                "operation_mode": operation_mode,
                # YAML 기본값보다 실제 launch 함대 구성을 우선한다.
                "drone_ids": drone_ids,
            },
        ],
    )


def _launch_nodes(context):
    config = LaunchConfiguration("config")
    mavsdk_python = LaunchConfiguration("mavsdk_python")
    detector_python = LaunchConfiguration("detector_python")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    drone_count = _parse_drone_count(
        LaunchConfiguration("drone_count").perform(context)
    )
    operation_mode = _parse_operation_mode(
        LaunchConfiguration("operation_mode").perform(context)
    )
    drone_ids = [
        f"quadrotor_{index:02d}"
        for index in range(1, drone_count + 1)
    ]

    package_share = get_package_share_directory("forest_rescue_system")
    rviz_config = os.path.join(
        package_share,
        "config",
        f"forest_rescue_{drone_count}.rviz",
    )
    if not os.path.isfile(rviz_config):
        raise RuntimeError(
            f"드론 수에 맞는 RViz 설정을 찾지 못했습니다: {rviz_config}"
        )

    nodes = [
        Node(
            package="forest_rescue_system",
            executable="rviz_visualization",
            name="rviz_visualization_node",
            output="screen",
            parameters=[
                config,
                {
                    "use_sim_time": use_sim_time,
                    "operation_mode": operation_mode,
                    "drone_ids": drone_ids,
                },
            ],
        ),
    ]

    if operation_mode == "rescue_search":
        nodes.insert(
            0,
            _manager_node(
                executable="mission_manager",
                name="mission_manager_node",
                config=config,
                use_sim_time=use_sim_time,
                operation_mode=operation_mode,
                drone_ids=drone_ids,
            ),
        )
        obstacle_states = RESCUE_OBSTACLE_STATES
    else:
        nodes.insert(
            0,
            _manager_node(
                executable="mapping_manager",
                name="mapping_manager_node",
                config=config,
                use_sim_time=use_sim_time,
                operation_mode=operation_mode,
                drone_ids=drone_ids,
            ),
        )
        obstacle_states = MAPPING_OBSTACLE_STATES

    for index, drone_id in enumerate(drone_ids, start=1):
        suffix = f"{index:02d}"
        common = dict(
            package="forest_rescue_system",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        )

        # 두 모드에서 공통으로 필요한 센서 TF, 장애물 감시, PX4 제어다.
        nodes.extend(
            [
                Node(
                    **common,
                    executable="sensor_tf",
                    name=f"sensor_tf_{suffix}",
                ),
                Node(
                    package="forest_rescue_system",
                    executable="pointcloud_local_mapper",
                    name=f"pointcloud_local_mapper_{suffix}",
                    output="screen",
                    parameters=[
                        config,
                        {
                            "use_sim_time": use_sim_time,
                            "active_mission_states": obstacle_states,
                        },
                    ],
                ),
                Node(
                    package="forest_rescue_system",
                    executable="obstacle_monitor",
                    name=f"obstacle_monitor_{suffix}",
                    output="screen",
                    parameters=[
                        config,
                        {
                            "use_sim_time": use_sim_time,
                            "active_mission_states": obstacle_states,
                        },
                    ],
                ),
                Node(
                    **common,
                    executable="drone_controller",
                    name=f"drone_controller_{suffix}",
                    prefix=[mavsdk_python],
                ),
            ]
        )

        # 조난자 탐지와 위치추정은 구조 수색 모드에서만 실행한다.
        if operation_mode == "rescue_search":
            nodes.extend(
                [
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
                ]
            )

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="forest_rescue_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        )
    )

    print(
        "[LAUNCH] 동적 함대/모드 구성: "
        f"operation_mode={operation_mode}, "
        f"drone_count={drone_count}, drone_ids={drone_ids}, "
        f"rviz_config={os.path.basename(rviz_config)}"
    )
    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("forest_rescue_system")
    default_config = os.path.join(
        package_share,
        "config",
        "forest_rescue.yaml",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="PYTHONNOUSERSITE", value="1"),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="동적 다중 드론 통합 시스템 YAML 설정 파일",
            ),
            DeclareLaunchArgument(
                "drone_count",
                default_value=str(DEFAULT_DRONE_COUNT),
                description="실행할 드론 수(1~4, 기본값 3)",
            ),
            DeclareLaunchArgument(
                "operation_mode",
                default_value=DEFAULT_OPERATION_MODE,
                description=(
                    "운용 모드: rescue_search 또는 mapping_3d "
                    "(기본값 rescue_search)"
                ),
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
                description="통합 RViz 화면을 함께 실행",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Isaac Sim /clock을 모든 ROS 2 노드에서 사용",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
