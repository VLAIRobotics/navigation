#!/usr/bin/env bash
set -euo pipefail

# 手动校准 MID360 IMU 的 body -> base_link Roll/Pitch
# 用法:
#   ./tool/calibrate_body_to_base.sh
#   ./tool/calibrate_body_to_base.sh use_fast_lio:=true
#   ./tool/calibrate_body_to_base.sh sample_count:=300 gyro_threshold:=0.03

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "calibrate_body_to_base"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${SCRIPT_DIR}/../install/setup.bash}"

[[ -r "${ROS_SETUP}" ]] || {
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
}

source_setup() {
  local setup_file="$1"
  set +u
  source "${setup_file}"
  set -u
}

source_setup "${ROS_SETUP}"
if [[ -r "${WORKSPACE_SETUP}" ]]; then
  source_setup "${WORKSPACE_SETUP}"
else
  echo "Workspace setup not found: ${WORKSPACE_SETUP}" >&2
  echo "Build first: colcon build --symlink-install" >&2
  exit 1
fi

echo "Place the vehicle on level ground and keep it completely still."
echo "Calibration uses 200 IMU frames by default. Press Ctrl-C after checking the result."
ros2 launch bringup calibrate_body_to_base.launch.py "$@"
