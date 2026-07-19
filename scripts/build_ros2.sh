#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ "${ROS_DISTRO:-}" != "humble" ]] || ! command -v ros2 >/dev/null; then
    echo "[ERROR] 먼저 현재 터미널에서 ros_setup을 실행하세요."
    exit 1
fi

cd "${WORKSPACE}"

# ROS 환경은 사용자가 ros_setup으로 준비하고, 빌드만 수행한다.
colcon build --symlink-install

echo "[OK] ROS 2 workspace 빌드 완료"
echo "다음 명령을 실행하세요: source ${WORKSPACE}/install/setup.bash"
