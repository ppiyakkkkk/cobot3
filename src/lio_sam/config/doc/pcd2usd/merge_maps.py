#!/usr/bin/env python3
"""각 드론별로 /lio_sam/save_map으로 저장한 GlobalMap.pcd를 하나의 지도로 합친다.

LIO-SAM의 map 프레임 원점은 그 드론이 SLAM을 시작한 위치(=스폰 위치)라서,
드론별 GlobalMap.pcd는 서로 다른 좌표계에 있다. sim_config.py의
DRONE_CONFIGS 스폰 좌표(회전 없음, world_enu)만큼 평행이동해서 정렬한다
(run.launch.py의 world_x/y/z 정적 TF와 동일한 오프셋).

Usage:
    python3 merge_maps.py output.pcd \\
        --quadrotor_01 ~/lio_sam_maps/quadrotor_01/GlobalMap.pcd \\
        --quadrotor_02 ~/lio_sam_maps/quadrotor_02/GlobalMap.pcd \\
        --quadrotor_03 ~/lio_sam_maps/quadrotor_03/GlobalMap.pcd

Requires: numpy and the PCL CLI tool pcl_convert_pcd_ascii_binary
(둘 다 이 환경에 이미 있음).
"""
import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from pcd_to_usd import pcd_to_ascii, read_ascii_pcd  # noqa: E402

# sim_config.py DRONE_CONFIGS 스폰 좌표(world_enu, 회전 없음) + 이륙 고도.
# process_mission_maps.py의 DRONES와 반드시 동일한 값을 써야 한다
# (bag 녹화가 스폰이 아니라 SEARCHING 진입 시점부터 시작되는 이유는 그쪽 주석 참고).
_TAKEOFF_ALTITUDE_M = 6.0
DRONE_OFFSETS = {
    "quadrotor_01": np.array([-34.0, 40.0, 31.0 + _TAKEOFF_ALTITUDE_M], dtype=np.float32),
    "quadrotor_02": np.array([-29.0, 40.0, 31.0 + _TAKEOFF_ALTITUDE_M], dtype=np.float32),
    "quadrotor_03": np.array([-39.0, 40.0, 31.0 + _TAKEOFF_ALTITUDE_M], dtype=np.float32),
}


def write_ascii_pcd(path, points, intensity):
    n = len(points)
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        inten = intensity if intensity is not None else np.zeros(n, dtype=np.float32)
        for p, i in zip(points, inten):
            f.write(f"{p[0]} {p[1]} {p[2]} {i}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_pcd")
    for name in DRONE_OFFSETS:
        parser.add_argument(f"--{name}", help=f"{name}의 GlobalMap.pcd 경로")
    args = parser.parse_args()

    merged_points = []
    merged_intensity = []
    with tempfile.TemporaryDirectory() as tmp_dir:
        for name, offset in DRONE_OFFSETS.items():
            pcd_path = getattr(args, name)
            if not pcd_path:
                continue
            ascii_pcd = pcd_to_ascii(pcd_path, tmp_dir)
            points, intensity = read_ascii_pcd(ascii_pcd)
            points = points + offset
            merged_points.append(points)
            merged_intensity.append(
                intensity if intensity is not None else np.zeros(len(points), dtype=np.float32)
            )
            print(f"{name}: {len(points)} points, offset {offset.tolist()}")

    if not merged_points:
        sys.exit("합칠 드론 지도를 하나도 안 줬습니다 (--quadrotor_01 등).")

    all_points = np.concatenate(merged_points, axis=0)
    all_intensity = np.concatenate(merged_intensity, axis=0)
    write_ascii_pcd(args.output_pcd, all_points, all_intensity)
    print(f"합계 {len(all_points)} points -> {args.output_pcd}")


if __name__ == "__main__":
    main()
