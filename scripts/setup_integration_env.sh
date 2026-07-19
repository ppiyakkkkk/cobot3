#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 기존 ros_setup과 mavsdk_on 환경이 함께 동작하는지 검사한다.
if [[ "${ROS_DISTRO:-}" != "humble" ]] || ! command -v ros2 >/dev/null; then
    echo "[ERROR] 먼저 현재 터미널에서 ros_setup을 실행하세요."
    exit 1
fi

EXPECTED_VENV="${HOME}/venvs/pegasus_control"
if [[ "${VIRTUAL_ENV:-}" != "${EXPECTED_VENV}" ]]; then
    echo "[ERROR] 먼저 현재 터미널에서 mavsdk_on을 실행하세요."
    echo "현재 VIRTUAL_ENV=${VIRTUAL_ENV:-설정되지 않음}"
    exit 1
fi

SYSTEM_PYTHON="/usr/bin/python3"

echo "[INFO] ROS/apt Python 영상 환경 확인"
PYTHONNOUSERSITE=1 "${SYSTEM_PYTHON}" - <<'PY'
import cv2
import numpy
import rclpy
from cv_bridge import CvBridge

print("[OK] system NumPy:", numpy.__version__)
print("[OK] system OpenCV:", cv2.__version__)
print("[OK] system rclpy:", rclpy.__file__)
print("[OK] system CvBridge")
PY

echo "[INFO] pegasus_control 빌드 및 MAVSDK 의존성 확인"
python -m pip install \
    "empy==3.3.4" \
    catkin_pkg \
    lark-parser \
    "numpy==1.26.4"

# 실제 YOLO 모드는 별도의 NumPy 1.x와 OpenCV wheel을 사용한다.
# --with-yolo를 지정하면 패키지를 설치하고 YOLO11 가중치까지
# workspace/models에 준비한다.
if [[ "${1:-}" == "--with-yolo" ]]; then
    echo "[INFO] YOLO 실행 의존성 설치"
    python -m pip install \
        "numpy==1.26.4" \
        "opencv-python==4.11.0.86" \
        "ultralytics==8.4.101"

    YOLO_MODEL_NAME="${YOLO_MODEL_NAME:-yolo11s.pt}"
    YOLO_MODEL_DIR="${WORKSPACE}/models"
    mkdir -p "${YOLO_MODEL_DIR}"

    echo "[INFO] YOLO 가중치 준비: ${YOLO_MODEL_NAME}"
    python - "${YOLO_MODEL_DIR}" "${YOLO_MODEL_NAME}" <<'PY'
import os
from pathlib import Path
import sys

from ultralytics import YOLO

model_dir = Path(sys.argv[1]).resolve()
model_name = sys.argv[2]
model_path = model_dir / model_name
model_dir.mkdir(parents=True, exist_ok=True)

if model_path.is_file():
    print(f"[OK] 기존 YOLO 가중치 사용: {model_path}")
else:
    os.chdir(model_dir)
    YOLO(model_name)
    if not model_path.is_file():
        raise FileNotFoundError(
            f"YOLO 가중치 다운로드 결과를 찾을 수 없습니다: {model_path}"
        )
    print(f"[OK] YOLO 가중치 다운로드 완료: {model_path}")
PY
fi

python - <<'PY'
import catkin_pkg
import em
import lark
import mavsdk
import numpy
import rclpy
from geometry_msgs.msg import PointStamped

print("[OK] venv Empy:", em.__file__)
print("[OK] venv catkin_pkg:", catkin_pkg.__file__)
print("[OK] venv lark:", lark.__file__)
print("[OK] venv NumPy:", numpy.__version__)
print("[OK] venv mavsdk:", mavsdk.__file__)
print("[OK] venv rclpy:", rclpy.__file__)
print("[OK] venv ROS message import")
PY

if [[ "${1:-}" == "--with-yolo" ]]; then
    python - <<'PY'
from ultralytics import YOLO
import cv2

print("[OK] venv Ultralytics YOLO")
print("[OK] venv OpenCV:", cv2.__version__)
PY
fi

echo "[OK] ros_setup + mavsdk_on 통합 환경 확인 완료"
