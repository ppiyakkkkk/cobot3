#!/usr/bin/env python3
"""record_mission_maps.py가 녹화한 bag 하나를 드론 3대 순서로 재생하며
LIO-SAM 지도를 만들고, 끝나면 merge_maps.py로 합친다.

혼자 실행할 수도 있다 (rate 생략 시 1.0배속):
    python3 scripts/process_mission_maps.py ~/lio_sam_maps/bags/mission_20260723_120000 [rate]
"""
import os

# 라이브 임무(Isaac Sim)가 계속 떠 있는 상태에서 이 스크립트가 bag을
# --clock으로 재생하면, 같은 ROS 도메인에 /clock 발행자가 두 개가 되어
# 서로 충돌하며 시간이 거꾸로 튄다("jump back in time", TF_OLD_DATA,
# rviz 응답 없음). bag에는 이미 필요한 데이터가 다 들어있어 라이브 토픽이
# 필요 없으므로, 별도 ROS_DOMAIN_ID로 완전히 격리해서 돈다.
# rclpy를 import하기 전에 설정해야 한다.
os.environ["ROS_DOMAIN_ID"] = "149"

import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

REPO_ROOT = Path(__file__).resolve().parent.parent
RING_FILLER = REPO_ROOT / "isaac_sim" / "ring_filler_node.py"
IMU_FILTER = REPO_ROOT / "isaac_sim" / "imu_filter_node.py"
LIO_SAM_PARAMS = (
    REPO_ROOT / "install" / "lio_sam" / "share" / "lio_sam"
    / "config" / "params_isaacsim.yaml"
)
MERGE_SCRIPT = REPO_ROOT / "src" / "lio_sam" / "config" / "doc" / "pcd2usd" / "merge_maps.py"
MAP_ROOT = Path.home() / "lio_sam_maps"

# sim_config.py DRONE_CONFIGS 스폰 좌표(world_enu, 회전 없음) + 이륙 고도.
# bag 녹화는 스폰 순간이 아니라 SEARCHING 진입 시점(=이륙+호버링 완료 후)부터
# 시작되므로, LIO-SAM map 원점은 스폰 위치가 아니라 그만큼 위(z)에 있다.
# 실제 임무 로그(~/.ros/log의 drone_controller_0N 로그)로 확인:
# - x,y는 SEARCHING 시작 시점에 N=0,E=0(스폰과 동일, 수평 드리프트 없음)
# - IMU 첫 샘플의 yaw도 세 드론 다 0도 근접(identity 회전 가정 그대로 유효)
# - z만 takeoff_altitude_m(기본 6.0) 만큼 스폰보다 높음
_TAKEOFF_ALTITUDE_M = 6.0
DRONES = {
    "quadrotor_01": (-34.0, 40.0, 31.0 + _TAKEOFF_ALTITUDE_M),
    "quadrotor_02": (-29.0, 40.0, 31.0 + _TAKEOFF_ALTITUDE_M),
    "quadrotor_03": (-39.0, 40.0, 31.0 + _TAKEOFF_ALTITUDE_M),
}


def check_environment():
    if os.environ.get("ROS_DISTRO") != "humble" or shutil.which("ros2") is None:
        sys.exit("[ERROR] 먼저 ros_setup을 실행하세요.")
    if subprocess.run(
        ["ros2", "pkg", "prefix", "lio_sam"],
        capture_output=True,
    ).returncode != 0:
        sys.exit("[ERROR] lio_sam 패키지가 안 보입니다. install/setup.bash를 source하세요.")


def wait_for_node(name, timeout_sec=20.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["ros2", "node", "list"], capture_output=True, text=True, timeout=10
        )
        if f"/{name}" in result.stdout.splitlines():
            return True
        time.sleep(1.0)
    return False


def call_save_map(destination, resolution=0.2):
    from lio_sam.srv import SaveMap

    rclpy.init(args=None)
    node = Node("process_mission_maps_client")
    client = node.create_client(SaveMap, "/lio_sam/save_map")
    if not client.wait_for_service(timeout_sec=20.0):
        node.get_logger().error("save_map 서비스가 안 뜸")
        node.destroy_node()
        rclpy.shutdown()
        return False

    req = SaveMap.Request()
    req.resolution = resolution
    req.destination = destination
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    success = bool(future.result() and future.result().success)
    node.destroy_node()
    rclpy.shutdown()
    return success


def process_drone(drone_name, offset, bag_path, rate):
    point_cloud_topic = f"/{drone_name}/point_cloud"
    imu_topic_raw = f"/{drone_name}/imu/data"
    imu_topic_filtered = f"/{drone_name}/imu/data_filtered"
    world_x, world_y, world_z = offset
    destination = f"/lio_sam_maps/{drone_name}"

    print(f"\n===== {drone_name} 처리 시작 =====", flush=True)

    ring_filler_proc = subprocess.Popen(
        [
            sys.executable, str(RING_FILLER), "--ros-args",
            "-p", f"input_topic:={point_cloud_topic}",
            "-p", "output_topic:=/points",
        ],
        start_new_session=True,
    )
    imu_filter_proc = subprocess.Popen(
        [
            sys.executable, str(IMU_FILTER), "--ros-args",
            "-p", f"input_topic:={imu_topic_raw}",
            "-p", f"output_topic:={imu_topic_filtered}",
        ],
        start_new_session=True,
    )
    lio_sam_proc = subprocess.Popen(
        [
            "ros2", "launch", "lio_sam", "run.launch.py",
            f"params_file:={LIO_SAM_PARAMS}",
            f"imu_topic:={imu_topic_filtered}",
            f"world_x:={world_x}", f"world_y:={world_y}", f"world_z:={world_z}",
            f"drone_name:={drone_name}",
            "publish_robot_urdf:=false",
            "use_sim_time:=true",
        ],
        start_new_session=True,
    )

    try:
        if not wait_for_node("lio_sam_mapOptimization"):
            print(f"[WARN] {drone_name}: lio_sam_mapOptimization이 안 떠서 건너뜁니다.", flush=True)
            return None

        print(f"{drone_name}: bag 재생 시작 ({bag_path})", flush=True)
        # --clock은 재생 진행률로 /clock을 자체 합성해서 발행하는 옵션이라,
        # --topics에 녹화된 원본 /clock까지 같이 넣으면 같은 토픽에 두 시간
        # 소스가 겹쳐써져서 시간이 거꾸로 튄다. --topics에는 /clock을 넣지 않는다.
        # 급기동 구간 스킵은 롤백함(세그먼트 전환마다 궤적이 순간이동하고
        # save_map이 오히려 더 자주 실패해서, imu_filter_node의 클램핑과
        # imuPreintegration.cpp의 TF frame_id 수정만으로 통째로 재생한다.
        subprocess.run(
            ["ros2", "bag", "play", str(bag_path), "--clock", "--rate", str(rate),
             "--topics", point_cloud_topic, imu_topic_raw],
            check=False,
        )
        print(f"{drone_name}: bag 재생 끝, save_map 호출", flush=True)

        success = call_save_map(destination)
        if success:
            print(f"{drone_name}: 저장 성공 -> ~{destination}", flush=True)
        else:
            print(f"[WARN] {drone_name}: save_map success=False (키프레임 없음 등)", flush=True)
        return (MAP_ROOT / drone_name / "GlobalMap.pcd") if success else None
    finally:
        # ros2 launch의 SIGINT가 가끔 static_transform_publisher/
        # robot_state_publisher 같은 자식 프로세스까지 확실히 못 죽여서,
        # 다음 드론 처리 때 이전 드론의 오프셋으로 world->map을 계속
        # 발행하는 좀비가 쌓이는 문제를 실제로 겪었다(맵이 "순간이동"하는
        # 것처럼 보였음). 세 프로세스 모두 자기 프로세스 그룹으로 띄웠으니
        # (start_new_session=True), 그룹째로 죽여서 자식까지 확실히 정리한다
        # — 이름으로 pkill하면 라이브 임무 쪽 동일 이름 프로세스까지 죽일
        # 위험이 있어서 쓰지 않는다.
        for proc in (lio_sam_proc, imu_filter_proc, ring_filler_proc):
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass
        for proc in (lio_sam_proc, imu_filter_proc, ring_filler_proc):
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=5)


def main():
    if len(sys.argv) not in (2, 3):
        sys.exit(f"Usage: {sys.argv[0]} <bag_path> [rate]")
    bag_path = Path(sys.argv[1]).expanduser()
    rate = float(sys.argv[2]) if len(sys.argv) == 3 else 1.0
    if not bag_path.exists():
        sys.exit(f"[ERROR] bag 경로가 없습니다: {bag_path}")

    check_environment()

    saved_maps = {}
    for drone_name, offset in DRONES.items():
        result = process_drone(drone_name, offset, bag_path, rate)
        if result is not None:
            saved_maps[drone_name] = result

    if not saved_maps:
        sys.exit("[ERROR] 저장된 지도가 하나도 없습니다.")

    print(f"\n===== 병합: {list(saved_maps.keys())} =====", flush=True)
    merge_args = [sys.executable, str(MERGE_SCRIPT), str(MAP_ROOT / "merged.pcd")]
    for drone_name, pcd_path in saved_maps.items():
        merge_args += [f"--{drone_name}", str(pcd_path)]
    subprocess.run(merge_args, check=True)


if __name__ == "__main__":
    main()
