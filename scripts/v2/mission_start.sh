#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
assert_expected_branch
load_ros_workspace
exec ros2 service call /mission/start std_srvs/srv/Trigger "{}"
