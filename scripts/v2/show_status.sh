#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
assert_expected_branch
load_ros_workspace

echo "===== Environment ====="
print_runtime_summary

echo
echo "===== ROS nodes ====="
ros2 node list || true

echo
echo "===== Mission mode/state ====="
timeout 3 ros2 topic echo /mission/mode --once || true
timeout 3 ros2 topic echo /mission/state --once || true

echo
echo "===== Clock ====="
timeout 3 ros2 topic echo /clock --once || true

echo
echo "===== Key topics ====="
ros2 topic list | grep -E 'mission|victim|rescue|quadrotor_01/(Camera|point_cloud)' || true
