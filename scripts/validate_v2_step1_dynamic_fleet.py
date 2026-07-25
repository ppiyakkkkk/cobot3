#!/usr/bin/env python3
"""V2 정적 검증: 두 패키지 구조와 통합/A/B launch 구성을 검사한다."""

from __future__ import annotations

import compileall
import importlib.util
from pathlib import Path
import subprocess
import sys
import types
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SIM_DIR = ROOT / "isaac_sim"
SYSTEM_PACKAGE = ROOT / "src/forest_rescue_system"
PYTHON_PACKAGE = SYSTEM_PACKAGE / "forest_rescue_system"
LAUNCH_DIR = SYSTEM_PACKAGE / "launch"
LAUNCH_COMMON = PYTHON_PACKAGE / "bringup/launch_common.py"


def check_sim_config(count):
    code = (
        "import sys; "
        f"sys.path.insert(0, {str(SIM_DIR)!r}); "
        "import sim_config as c; "
        f"c.configure_drone_count({count}); "
        "print(c.DRONE_COUNT); "
        "print(','.join(c.DRONE_IDS)); "
        "print(';'.join(str(item[1]) for item in c.DRONE_CONFIGS))"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        check=True,
        capture_output=True,
        text=True,
    )
    lines = result.stdout.strip().splitlines()
    assert int(lines[0]) == count
    assert lines[1].split(",") == [
        f"quadrotor_{index:02d}" for index in range(1, count + 1)
    ]
    assert [int(value) for value in lines[2].split(";")] == list(range(count))


def check_package_layout():
    actual_packages = {
        path.parent.name for path in (ROOT / "src").glob("*/package.xml")
    }
    expected_packages = {
        "forest_rescue_interfaces",
        "forest_rescue_system",
    }
    assert actual_packages == expected_packages

    expected_modules = {
        "bringup",
        "common",
        "detection",
        "drone",
        "mission",
        "rescuer",
        "visualization",
    }
    actual_modules = {
        path.name
        for path in PYTHON_PACKAGE.iterdir()
        if path.is_dir() and not path.name.startswith("__")
    }
    assert expected_modules <= actual_modules

    launch_files = {
        path.name for path in LAUNCH_DIR.glob("*.launch.py")
    }
    assert launch_files == {
        "forest_rescue_system.launch.py",
        "forest_rescue_pc_a.launch.py",
        "forest_rescue_pc_b.launch.py",
    }

    for package_xml in (ROOT / "src").glob("*/package.xml"):
        ET.parse(package_xml)


def install_launch_stubs():
    """ROS 2가 없는 PC에서도 실제 launch 생성 함수를 정적으로 실행한다."""

    class FakeLaunchDescription:
        def __init__(self, actions):
            self.actions = actions

    class FakeAction:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class FakeLaunchConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    class FakeNode:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    ament_index_python = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = (
        lambda _package_name: str(SYSTEM_PACKAGE)
    )

    launch = types.ModuleType("launch")
    launch.LaunchDescription = FakeLaunchDescription
    launch_actions = types.ModuleType("launch.actions")
    launch_actions.DeclareLaunchArgument = FakeAction
    launch_actions.OpaqueFunction = FakeAction
    launch_conditions = types.ModuleType("launch.conditions")
    launch_conditions.IfCondition = FakeAction
    launch_substitutions = types.ModuleType("launch.substitutions")
    launch_substitutions.LaunchConfiguration = FakeLaunchConfiguration

    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = FakeNode

    stubs = {
        "ament_index_python": ament_index_python,
        "ament_index_python.packages": ament_packages,
        "launch": launch,
        "launch.actions": launch_actions,
        "launch.conditions": launch_conditions,
        "launch.substitutions": launch_substitutions,
        "launch_ros": launch_ros,
        "launch_ros.actions": launch_ros_actions,
    }
    sys.modules.update(stubs)


def load_launch_common():
    install_launch_stubs()
    spec = importlib.util.spec_from_file_location(
        "forest_rescue_launch_common",
        LAUNCH_COMMON,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def node_keys(nodes):
    return {
        (
            node.kwargs.get("package"),
            node.kwargs.get("executable"),
            node.kwargs.get("name"),
        )
        for node in nodes
    }


def check_launch_roles():
    launch_common = load_launch_common()

    for operation_mode in launch_common.SUPPORTED_OPERATION_MODES:
        for drone_count in range(1, 5):
            context = {
                "config": str(
                    SYSTEM_PACKAGE / "config/forest_rescue.yaml"
                ),
                "mavsdk_python": "~/venvs/pegasus_control/bin/python",
                "detector_python": "~/venvs/pegasus_control/bin/python",
                "coverage_python": "~/venvs/pegasus_control/bin/python",
                "use_rviz": "false",
                "use_sim_time": "true",
                "drone_count": str(drone_count),
                "operation_mode": operation_mode,
            }
            integrated = node_keys(
                launch_common._launch_nodes(
                    context,
                    launch_common.ROLE_INTEGRATED,
                )
            )
            pc_a = node_keys(
                launch_common._launch_nodes(
                    context,
                    launch_common.ROLE_PC_A,
                )
            )
            pc_b = node_keys(
                launch_common._launch_nodes(
                    context,
                    launch_common.ROLE_PC_B,
                )
            )

            assert not (pc_a & pc_b)
            assert integrated == (pc_a | pc_b)

            if operation_mode == "rescue_search":
                assert len(pc_b) == drone_count * 2
            else:
                assert not pc_b


def check_sources():
    final_24 = (SIM_DIR / "final_24.py").read_text(encoding="utf-8")
    sim_utils = (SIM_DIR / "sim_utils.py").read_text(encoding="utf-8")
    mission = (
        PYTHON_PACKAGE / "mission/mission_manager_node.py"
    ).read_text(encoding="utf-8")
    yaml_text = (
        SYSTEM_PACKAGE / "config/forest_rescue.yaml"
    ).read_text(encoding="utf-8")
    setup_text = (SYSTEM_PACKAGE / "setup.py").read_text(encoding="utf-8")

    assert '"--drone_count"' in final_24
    assert "configure_drone_count(runtime_args.drone_count)" in final_24
    assert '"zone_bounds_xy"' in sim_utils
    assert '"format_version": 5' in sim_utils
    assert "def _load_search_plan_metadata" in mission
    assert "human_detector_04:" in yaml_text
    assert "mavsdk_server_port: 50054" in yaml_text
    assert "forest_rescue_system.detection.human_detector_node:main" in setup_text
    assert "forest_rescue_system.drone.drone_controller_node:main" in setup_text

    assert compileall.compile_dir(
        ROOT / "src",
        quiet=1,
        force=True,
    )
    assert compileall.compile_dir(
        ROOT / "scripts",
        quiet=1,
        force=True,
    )


def main():
    check_package_layout()
    print("[OK] src에는 인터페이스/기능 패키지 두 개만 존재")

    for count in range(1, 5):
        check_sim_config(count)
    print("[OK] 드론 1~4대 동적 설정")

    check_sources()
    print("[OK] Python 문법, 모듈 경로, YAML 핵심 설정")

    check_launch_roles()
    print("[OK] 모든 모드·드론 수에서 PC A + PC B = 통합 launch")
    print("[PASS] V2 두 패키지/두 PC 정적 검증 완료")


if __name__ == "__main__":
    main()
