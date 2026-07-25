#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

DRONE_COUNT="${DEFAULT_DRONE_COUNT}"
MODE="${DEFAULT_OPERATION_MODE}"
USE_RVIZ=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --drone-count)
            shift
            [[ $# -gt 0 ]] || die "--drone-count 값이 필요합니다."
            DRONE_COUNT="$1"
            ;;
        --mode)
            shift
            [[ $# -gt 0 ]] || die "--mode 값이 필요합니다."
            MODE="$1"
            ;;
        --rviz) USE_RVIZ=true ;;
        -h|--help)
            echo "사용법: run_single_pc.sh [--drone-count 1~4] [--mode MODE] [--rviz]"
            exit 0
            ;;
        *) die "알 수 없는 인자: $1" ;;
    esac
    shift
done

validate_drone_count "${DRONE_COUNT}"
validate_mode "${MODE}"
assert_expected_branch
load_ros_workspace



print_runtime_summary
echo "[RUN] launch=forest_rescue_system.launch.py, drone_count=${DRONE_COUNT}, mode=${MODE}, rviz=${USE_RVIZ}"

exec ros2 launch forest_rescue_system forest_rescue_system.launch.py \
    drone_count:="${DRONE_COUNT}" \
    operation_mode:="${MODE}" \
    use_rviz:="${USE_RVIZ}" \
    use_sim_time:=true \
    mavsdk_python:="${VENV_PATH}/bin/python" \
    detector_python:="${VENV_PATH}/bin/python" \
    coverage_python:="${VENV_PATH}/bin/python"
