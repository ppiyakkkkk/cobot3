#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

assert_expected_branch
source_ros
activate_venv
cd "${FOREST_RESCUE_WS}"

if command -v rosdep >/dev/null 2>&1; then
    info "rosdep 실행"
    rosdep install --from-paths src --ignore-src -r -y || \
        warn "rosdep 일부 항목을 해결하지 못했습니다. apt/ROS 설치 상태를 확인하세요."
fi

info "정적 검증"
python3 scripts/validate_v2_step1_dynamic_fleet.py
python3 -m compileall -q src scripts

info "ROS 2 workspace 빌드"
colcon build --symlink-install

# shellcheck disable=SC1091
source "${FOREST_RESCUE_WS}/install/setup.bash"
ros2 pkg prefix forest_rescue_interfaces >/dev/null
ros2 pkg prefix forest_rescue_system >/dev/null

echo "[OK] workspace 빌드 및 package 확인 완료"
