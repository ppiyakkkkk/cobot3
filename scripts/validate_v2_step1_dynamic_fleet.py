#!/usr/bin/env python3
"""V2 Step 1 정적 검증: 1~4대 설정과 drone_count 인자를 검사한다."""

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
SIM_DIR = ROOT / "isaac_sim"


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
    ids = lines[1].split(",")
    assert ids == [f"quadrotor_{i:02d}" for i in range(1, count + 1)]
    vehicle_ids = [int(value) for value in lines[2].split(";")]
    assert vehicle_ids == list(range(count))
    print(f"[OK] sim_config drone_count={count}: {ids}")


def check_default_count():
    code = (
        "import sys; "
        f"sys.path.insert(0, {str(SIM_DIR)!r}); "
        "import sim_config as c; "
        "print(c.DEFAULT_DRONE_COUNT); "
        "print(c.DRONE_COUNT)"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        check=True,
        capture_output=True,
        text=True,
    )
    values = [int(value) for value in result.stdout.strip().splitlines()]
    assert values == [3, 3]
    print("[OK] 기본 drone_count=3")


def check_sources():
    final_24 = (SIM_DIR / "final_24.py").read_text(encoding="utf-8")
    launch = (
        ROOT
        / "src/forest_rescue_system/launch/forest_rescue_system.launch.py"
    ).read_text(encoding="utf-8")
    mission = (
        ROOT
        / "src/forest_rescue_system/forest_rescue_system/mission_manager_node.py"
    ).read_text(encoding="utf-8")
    sim_utils = (SIM_DIR / "sim_utils.py").read_text(encoding="utf-8")
    yaml_text = (
        ROOT / "src/forest_rescue_system/config/forest_rescue.yaml"
    ).read_text(encoding="utf-8")

    assert '"--drone_count"' in final_24
    assert "configure_drone_count(runtime_args.drone_count)" in final_24
    assert 'LaunchConfiguration("drone_count")' in launch
    assert 'default_value=str(DEFAULT_DRONE_COUNT)' in launch
    assert "FOREST_RESCUE_DRONE_COUNT" not in launch
    assert '"zone_bounds_xy"' in sim_utils
    assert '"format_version": 4' in sim_utils
    assert "def _load_search_plan_metadata" in mission
    assert "human_detector_04:" in yaml_text
    assert "mavsdk_server_port: 50054" in yaml_text
    print("[OK] Python/Launch/Plan/Mission/YAML 필수 구조 확인")


check_default_count()
for count in range(1, 5):
    check_sim_config(count)
check_sources()
print("[PASS] V2 Step 1 drone_count 정적 검증 완료")
