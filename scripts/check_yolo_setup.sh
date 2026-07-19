#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
EXPECTED_VENV="${HOME}/venvs/pegasus_control"
MODEL_PATH="${WORKSPACE}/models/yolo11s.pt"

if [[ "${ROS_DISTRO:-}" != "humble" ]] || ! command -v ros2 >/dev/null; then
    echo "[ERROR] 먼저 ros_setup을 실행하세요."
    exit 1
fi

if [[ "${VIRTUAL_ENV:-}" != "${EXPECTED_VENV}" ]]; then
    echo "[ERROR] 먼저 mavsdk_on을 실행하세요."
    exit 1
fi

if [[ ! -f "${MODEL_PATH}" ]]; then
    echo "[ERROR] YOLO 가중치가 없습니다: ${MODEL_PATH}"
    echo "bash scripts/setup_integration_env.sh --with-yolo 를 실행하세요."
    exit 1
fi

python - "${MODEL_PATH}" <<'PY'
from pathlib import Path
import sys

import cv2
import mavsdk
import rclpy
from ultralytics import YOLO

model_path = Path(sys.argv[1])
YOLO(str(model_path))

print("[OK] rclpy:", rclpy.__file__)
print("[OK] mavsdk:", mavsdk.__file__)
print("[OK] OpenCV:", cv2.__version__)
print("[OK] YOLO11:", model_path)
PY

echo "[OK] YOLO 실행 환경 확인 완료"
