#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

DRONE_COUNT="${DEFAULT_DRONE_COUNT}"
MODE="${DEFAULT_OPERATION_MODE}"

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
        -h|--help)
            echo "사용법: run_isaac.sh [--drone-count 1~4] [--mode rescue_search|eval_coverage|mapping_3d]"
            exit 0
            ;;
        *) die "알 수 없는 인자: $1" ;;
    esac
    shift
done

validate_drone_count "${DRONE_COUNT}"
validate_mode "${MODE}"
assert_expected_branch

if [[ -n "${ISAAC_ROS_SETUP_SCRIPT:-}" ]]; then
    require_file "${ISAAC_ROS_SETUP_SCRIPT}"
    # shellcheck disable=SC1090
    source "${ISAAC_ROS_SETUP_SCRIPT}"
fi

isaac_python="$(resolve_isaac_python)" || \
    die "Isaac Python을 찾지 못했습니다. .forest_rescue.env의 ISAAC_PYTHON을 설정하세요."

px4_binary="${PX4_AUTOPILOT_PATH}/build/px4_sitl_default/bin/px4"
require_file "${px4_binary}"
require_file "${FOREST_RESCUE_WS}/isaac_sim/final_24.py"

export PX4_AUTOPILOT_PATH
print_runtime_summary
echo "[RUN] Isaac: drone_count=${DRONE_COUNT}, mode=${MODE}"

cd "${FOREST_RESCUE_WS}/isaac_sim"
exec "${isaac_python}" final_24.py \
    --drone_count "${DRONE_COUNT}" \
    --operation_mode "${MODE}"
