#!/usr/bin/env bash
set -Eeuo pipefail

V2_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WS="$(cd "${V2_SCRIPT_DIR}/../.." && pwd)"
FOREST_RESCUE_WS="${FOREST_RESCUE_WS:-${DEFAULT_WS}}"
FOREST_RESCUE_ENV_FILE="${FOREST_RESCUE_ENV_FILE:-${FOREST_RESCUE_WS}/.forest_rescue.env}"

if [[ -f "${FOREST_RESCUE_ENV_FILE}" ]]; then
    # shellcheck disable=SC1090
    source "${FOREST_RESCUE_ENV_FILE}"
fi

FOREST_RESCUE_WS="${FOREST_RESCUE_WS:-${DEFAULT_WS}}"
VENV_PATH="${VENV_PATH:-${HOME}/venvs/pegasus_control}"
PX4_AUTOPILOT_PATH="${PX4_AUTOPILOT_PATH:-${HOME}/PX4-Autopilot}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-143}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
DEFAULT_DRONE_COUNT="${DEFAULT_DRONE_COUNT:-3}"
DEFAULT_OPERATION_MODE="${DEFAULT_OPERATION_MODE:-rescue_search}"

export FOREST_RESCUE_WS
export VENV_PATH
export PX4_AUTOPILOT_PATH
export ROS_DISTRO
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION
export ROS_LOCALHOST_ONLY

die() {
    echo "[ERROR] $*" >&2
    exit 1
}

warn() {
    echo "[WARN] $*" >&2
}

info() {
    echo "[INFO] $*"
}

require_file() {
    [[ -f "$1" ]] || die "파일을 찾지 못했습니다: $1"
}

require_dir() {
    [[ -d "$1" ]] || die "디렉터리를 찾지 못했습니다: $1"
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || die "명령을 찾지 못했습니다: $1"
}

assert_expected_branch() {
    require_command git
    if ! git -C "${FOREST_RESCUE_WS}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        die "Git 저장소가 아닙니다: ${FOREST_RESCUE_WS}"
    fi
    local branch
    branch="$(git -C "${FOREST_RESCUE_WS}" branch --show-current)"
    if [[ "${branch}" != "feat/jj/v2_updates" ]]; then
        die "현재 브랜치가 feat/jj/v2_updates가 아닙니다: ${branch}"
    fi
}

source_ros() {
    local ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
    require_file "${ros_setup}"
    # shellcheck disable=SC1090
    source "${ros_setup}"
}

activate_venv() {
    require_file "${VENV_PATH}/bin/activate"
    # shellcheck disable=SC1090
    source "${VENV_PATH}/bin/activate"
}

source_workspace() {
    require_file "${FOREST_RESCUE_WS}/install/setup.bash"
    # shellcheck disable=SC1090
    source "${FOREST_RESCUE_WS}/install/setup.bash"
}

load_ros_workspace() {
    source_ros
    activate_venv
    source_workspace
}

validate_drone_count() {
    local value="$1"
    [[ "${value}" =~ ^[1-4]$ ]] || die "drone-count는 1~4여야 합니다: ${value}"
}

validate_mode() {
    case "$1" in
        rescue_search|eval_coverage|mapping_3d) ;;
        *) die "지원하지 않는 mode입니다: $1" ;;
    esac
}

resolve_isaac_python() {
    if [[ -n "${ISAAC_PYTHON:-}" && -x "${ISAAC_PYTHON}" ]]; then
        printf '%s\n' "${ISAAC_PYTHON}"
        return 0
    fi

    local candidates=(
        "${HOME}/isaacsim/python.sh"
        "${HOME}/isaac-sim/python.sh"
        "${HOME}/.local/share/ov/pkg/isaac-sim-5.1.0/python.sh"
    )
    local candidate
    for candidate in "${candidates[@]}"; do
        if [[ -x "${candidate}" ]]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done

    return 1
}

print_runtime_summary() {
    echo "[ENV] workspace=${FOREST_RESCUE_WS}"
    echo "[ENV] branch=$(git -C "${FOREST_RESCUE_WS}" branch --show-current 2>/dev/null || echo UNKNOWN)"
    echo "[ENV] commit=$(git -C "${FOREST_RESCUE_WS}" rev-parse --short HEAD 2>/dev/null || echo UNKNOWN)"
    echo "[ENV] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "[ENV] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
    echo "[ENV] ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
    echo "[ENV] venv=${VENV_PATH}"
    echo "[ENV] PX4=${PX4_AUTOPILOT_PATH}"
}
