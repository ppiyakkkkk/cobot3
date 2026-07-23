#!/usr/bin/env python3
"""/mission/state를 지켜보다가 SEARCHING 구간만 3드론 라이다/IMU를 녹화한다.

SEARCHING이 끝나면(RETURNING_NO_VICTIM/COMPLETE 등) 자동으로 녹화를 멈추고
process_mission_maps.py를 그 bag 경로로 이어서 실행한다.

Usage:
    python3 scripts/record_mission_maps.py
"""
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BAG_TOPICS = [
    "/quadrotor_01/point_cloud", "/quadrotor_01/imu/data",
    "/quadrotor_02/point_cloud", "/quadrotor_02/imu/data",
    "/quadrotor_03/point_cloud", "/quadrotor_03/imu/data",
    "/clock",
]
BAG_ROOT = Path.home() / "lio_sam_maps" / "bags"
SCRIPT_DIR = Path(__file__).resolve().parent


class MissionMapRecorder(Node):
    def __init__(self):
        super().__init__("mission_map_recorder")
        self.recording_proc = None
        self.bag_path = None
        self.finished_bag_path = None
        self.create_subscription(
            String, "/mission/state", self._on_state, 10
        )
        self.get_logger().info("Waiting for /mission/state == SEARCHING ...")

    def _on_state(self, msg):
        state = msg.data
        if state == "SEARCHING" and self.recording_proc is None:
            self._start_recording()
        elif state != "SEARCHING" and self.recording_proc is not None:
            self._stop_recording()

    def _start_recording(self):
        BAG_ROOT.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.bag_path = BAG_ROOT / f"mission_{stamp}"
        self.get_logger().info(f"SEARCHING 시작 -> 녹화 시작: {self.bag_path}")
        self.recording_proc = subprocess.Popen(
            ["ros2", "bag", "record", "-o", str(self.bag_path), *BAG_TOPICS]
        )

    def _stop_recording(self):
        self.get_logger().info("SEARCHING 종료 -> 녹화 중지")
        self.recording_proc.send_signal(signal.SIGINT)
        self.recording_proc.wait(timeout=15)
        self.finished_bag_path = self.bag_path
        self.recording_proc = None
        self.bag_path = None


def main():
    rclpy.init()
    node = MissionMapRecorder()
    try:
        while rclpy.ok() and node.finished_bag_path is None:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        if node.recording_proc is not None:
            node._stop_recording()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.finished_bag_path is None:
        print("녹화된 bag이 없습니다 (SEARCHING 상태를 못 봄). 종료합니다.")
        return

    print(f"녹화 완료: {node.finished_bag_path}")
    print("process_mission_maps.py로 이어서 처리합니다 ...")
    os.execv(
        sys.executable,
        [
            sys.executable,
            str(SCRIPT_DIR / "process_mission_maps.py"),
            str(node.finished_bag_path),
        ],
    )


if __name__ == "__main__":
    main()
