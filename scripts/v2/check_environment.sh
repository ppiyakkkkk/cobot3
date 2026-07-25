#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

PROFILE="single"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)
            shift
            [[ $# -gt 0 ]] || die "--profile 값이 필요합니다."
            PROFILE="$1"
            ;;
        -h|--help)
            echo "사용법: check_environment.sh --profile single|pc-a|pc-b"
            exit 0
            ;;
        *) die "알 수 없는 인자: $1" ;;
    esac
    shift
done

case "${PROFILE}" in
    single|pc-a|pc-b) ;;
    *) die "profile은 single, pc-a, pc-b 중 하나여야 합니다." ;;
esac

assert_expected_branch
print_runtime_summary
source_ros
activate_venv

info "Python 실행 파일: $(command -v python)"
python - <<'PY'
import sys
import cv2
import mavsdk
import numpy
import rclpy
import scipy
from cv_bridge import CvBridge
print("[OK] Python", sys.version)
print("[OK] NumPy", numpy.__version__)
print("[OK] OpenCV", cv2.__version__)
print("[OK] rclpy", rclpy.__file__)
print("[OK] cv_bridge")
print("[OK] MAVSDK", mavsdk.__file__)
print("[OK] SciPy", scipy.__version__)
try:
    import open3d
    print("[OK] Open3D", open3d.__version__)
except ImportError as error:
    print("[WARN] Open3D:", error)
try:
    from ultralytics import YOLO
    print("[OK] Ultralytics")
except ImportError as error:
    print("[WARN] Ultralytics:", error)
PY

require_file "${FOREST_RESCUE_WS}/isaac_sim/worlds/my_forest.usdc"
require_file "${FOREST_RESCUE_WS}/src/forest_rescue_system/config/forest_rescue.yaml"

if [[ -f "${FOREST_RESCUE_WS}/install/setup.bash" ]]; then
    source_workspace
    ros2 pkg prefix forest_rescue_system >/dev/null
    echo "[OK] ROS package install"
else
    warn "install/setup.bash가 없습니다. setup_workspace.sh를 실행하세요."
fi

if [[ "${PROFILE}" == "single" || "${PROFILE}" == "pc-a" ]]; then
    if isaac_python="$(resolve_isaac_python)"; then
        echo "[OK] Isaac Python: ${isaac_python}"
    else
        warn "ISAAC_PYTHON을 찾지 못했습니다. .forest_rescue.env를 수정하세요."
    fi

    px4_binary="${PX4_AUTOPILOT_PATH}/build/px4_sitl_default/bin/px4"
    if [[ -x "${px4_binary}" ]]; then
        echo "[OK] PX4 SITL: ${px4_binary}"
    else
        warn "PX4 SITL binary 없음: ${px4_binary}"
    fi
fi

if [[ "${PROFILE}" == "single" || "${PROFILE}" == "pc-b" ]]; then
    if [[ -f "${FOREST_RESCUE_WS}/models/yolo11s.pt" ]]; then
        echo "[OK] YOLO model"
    else
        warn "YOLO model 없음. setup_host.sh --with-yolo를 실행하세요."
    fi
fi

if [[ "${ROS_LOCALHOST_ONLY}" != "0" ]]; then
    warn "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; 두 PC 통신에는 0이 필요합니다."
fi

echo "[OK] 환경 점검 완료: profile=${PROFILE}"
