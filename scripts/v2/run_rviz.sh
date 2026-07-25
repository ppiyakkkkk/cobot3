#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

DRONE_COUNT="${DEFAULT_DRONE_COUNT}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --drone-count)
            shift
            [[ $# -gt 0 ]] || die "--drone-count 값이 필요합니다."
            DRONE_COUNT="$1"
            ;;
        -h|--help)
            echo "사용법: run_rviz.sh [--drone-count 1~4]"
            exit 0
            ;;
        *) die "알 수 없는 인자: $1" ;;
    esac
    shift
done

validate_drone_count "${DRONE_COUNT}"
assert_expected_branch
load_ros_workspace

config="$(ros2 pkg prefix forest_rescue_system)/share/forest_rescue_system/config/forest_rescue_${DRONE_COUNT}.rviz"
require_file "${config}"
exec rviz2 -d "${config}"
