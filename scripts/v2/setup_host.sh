#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

INSTALL_APT=false
WITH_YOLO=false
BUILD_PX4=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --install-apt) INSTALL_APT=true ;;
        --with-yolo) WITH_YOLO=true ;;
        --build-px4) BUILD_PX4=true ;;
        -h|--help)
            cat <<'EOF'
사용법:
  setup_host.sh [--install-apt] [--with-yolo] [--build-px4]
EOF
            exit 0
            ;;
        *) die "알 수 없는 인자: $1" ;;
    esac
    shift
done

assert_expected_branch
require_file "${FOREST_RESCUE_WS}/requirements.txt"

if [[ "${INSTALL_APT}" == true ]]; then
    require_command sudo
    info "Ubuntu/ROS 실행 의존성 설치"
    sudo apt-get update
    sudo apt-get install -y \
        git \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-pip \
        python3-venv \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-cv-bridge \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-sensor-msgs-py \
        ros-humble-rviz2
fi

source_ros

if [[ ! -d "${VENV_PATH}" ]]; then
    info "system ROS package를 볼 수 있는 venv 생성: ${VENV_PATH}"
    python3 -m venv --system-site-packages "${VENV_PATH}"
fi

activate_venv
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r "${FOREST_RESCUE_WS}/requirements.txt"
python -m pip install --force-reinstall "numpy==1.26.4"

if [[ "${WITH_YOLO}" == true ]]; then
    model_dir="${FOREST_RESCUE_WS}/models"
    model_path="${model_dir}/yolo11s.pt"
    mkdir -p "${model_dir}"
    if [[ ! -f "${model_path}" ]]; then
        info "YOLO11 가중치 준비: ${model_path}"
        python - "${model_dir}" <<'PY'
from pathlib import Path
import os
import sys
from ultralytics import YOLO

target = Path(sys.argv[1]).resolve()
target.mkdir(parents=True, exist_ok=True)
os.chdir(target)
YOLO("yolo11s.pt")
if not (target / "yolo11s.pt").is_file():
    raise FileNotFoundError(target / "yolo11s.pt")
PY
    fi
fi

info "Python/ROS import 확인"
python - <<'PY'
import cv2
import mavsdk
import numpy
import open3d
import rclpy
import scipy
from cv_bridge import CvBridge
print("[OK] NumPy", numpy.__version__)
print("[OK] OpenCV", cv2.__version__)
print("[OK] rclpy", rclpy.__file__)
print("[OK] cv_bridge", CvBridge)
print("[OK] MAVSDK", mavsdk.__file__)
print("[OK] Open3D", open3d.__version__)
print("[OK] SciPy", scipy.__version__)
try:
    from ultralytics import YOLO
    print("[OK] Ultralytics")
except ImportError:
    print("[INFO] Ultralytics 미설치 또는 --with-yolo 미사용")
PY

if [[ "${BUILD_PX4}" == true ]]; then
    require_dir "${PX4_AUTOPILOT_PATH}"
    info "PX4 SITL 빌드"
    make -C "${PX4_AUTOPILOT_PATH}" px4_sitl_default none
fi

echo "[OK] 호스트/venv 설정 완료"
