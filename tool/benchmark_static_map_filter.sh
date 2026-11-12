#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  printf 'Usage: %s <rosbag>\n' "$0" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "benchmark_static_map_filter"

BAG_PATH="$1"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$(dirname -- "$0")/../install/setup.bash}"
DURATION="${DURATION:-60}"
OUTPUT_DIR="${OUTPUT_DIR:-/tmp/static-map-filter-benchmark-$(date +%Y%m%d-%H%M%S)}"

[[ -r "${BAG_PATH}" || -d "${BAG_PATH}" ]] || {
  printf 'ROS bag not found: %s\n' "${BAG_PATH}" >&2
  exit 1
}
[[ -r "${ROS_SETUP}" ]] || {
  printf 'ROS setup not found: %s\n' "${ROS_SETUP}" >&2
  exit 1
}
[[ -r "${WORKSPACE_SETUP}" ]] || {
  printf 'Workspace setup not found: %s\n' "${WORKSPACE_SETUP}" >&2
  exit 1
}
mkdir -p "${OUTPUT_DIR}"

set +u
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"
set -u

NODE_LOG="${OUTPUT_DIR}/node.log"
BAG_INFO="${OUTPUT_DIR}/bag_info.log"
METRICS="${OUTPUT_DIR}/node_metrics.csv"
STATIC_HZ="${OUTPUT_DIR}/static_cloud_hz.log"
REJECTED_HZ="${OUTPUT_DIR}/rejected_cloud_hz.log"
SAVE_LOG="${OUTPUT_DIR}/save.log"
FRAMES_LOG="${OUTPUT_DIR}/processed_frames.log"
PCD_PATH="${OUTPUT_DIR}/clean_map.pcd"

ros2 bag info "${BAG_PATH}" >"${BAG_INFO}"
has_topic() {
  awk -v wanted="$1" '$1 == "Topic:" && $2 == wanted {found = 1} END {exit !found}' "${BAG_INFO}"
}
if ! has_topic /cloud_registered || ! has_topic /Odometry; then
  printf 'ROS bag must contain /cloud_registered and /Odometry; see %s\n' "${BAG_INFO}" >&2
  exit 1
fi

ros2 run bringup static_map_filter_node --ros-args \
  -p map_file_path:="${PCD_PATH}" \
  >"${NODE_LOG}" 2>&1 &
NODE_PID=$!

cleanup() {
  local status=$?
  trap - INT TERM EXIT
  kill "${METRICS_PID:-}" 2>/dev/null || true
  kill "${STATIC_HZ_PID:-}" "${REJECTED_HZ_PID:-}" 2>/dev/null || true
  kill -INT "${NODE_PID}" 2>/dev/null || true
  wait "${NODE_PID}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup INT TERM EXIT

printf 'timestamp,cpu_percent,rss_kb\n' >"${METRICS}"
(
  while kill -0 "${NODE_PID}" 2>/dev/null; do
    read -r cpu rss < <(ps -p "${NODE_PID}" -o '%cpu=,rss=') || true
    if [[ -n "${cpu:-}" && -n "${rss:-}" ]]; then
      printf '%s,%s,%s\n' "$(date +%s)" "${cpu}" "${rss}" >>"${METRICS}"
    fi
    sleep 1
  done
) &
METRICS_PID=$!

ros2 topic hz /dynamic_filter/static_cloud >"${STATIC_HZ}" 2>&1 &
STATIC_HZ_PID=$!
ros2 topic hz /dynamic_filter/rejected_cloud >"${REJECTED_HZ}" 2>&1 &
REJECTED_HZ_PID=$!

set +e
timeout --signal=INT --kill-after=5s "${DURATION}s" \
  ros2 bag play "${BAG_PATH}" --clock
BAG_STATUS=$?
set -e

FINAL_STATUS=0
case "${BAG_STATUS}" in
  0|124|130|143) ;;
  *) FINAL_STATUS=1 ;;
esac

SERVICE_STATUS=0
if ! timeout 10s ros2 service call /static_map_save std_srvs/srv/Trigger "{}" >"${SAVE_LOG}" 2>&1; then
  SERVICE_STATUS=1
fi
SAVE_SUCCESS="$(awk '
  {
    line = tolower($0)
    if (line ~ /success[[:space:]]*:[[:space:]]*true/ ||
        line ~ /success[[:space:]]*=[[:space:]]*true/) {
      print "true"
      found = 1
      exit
    }
  }
  END {
    if (!found) print "false"
  }
' "${SAVE_LOG}")"
if [[ "${SERVICE_STATUS}" -ne 0 || "${SAVE_SUCCESS}" != "true" || ! -s "${PCD_PATH}" ]]; then
  FINAL_STATUS=1
fi

FRAME_STATUS=0
if ! timeout 10s ros2 topic echo --qos-durability transient_local --once \
  /dynamic_filter/processed_frames >"${FRAMES_LOG}" 2>&1; then
  FRAME_STATUS=1
fi
PROCESSED_FRAMES="$(awk '$1 == "data:" {print $2; exit}' "${FRAMES_LOG}")"
if [[ "${FRAME_STATUS}" -ne 0 || ! "${PROCESSED_FRAMES}" =~ ^[1-9][0-9]*$ ]]; then
  FINAL_STATUS=1
fi

kill "${METRICS_PID}" "${STATIC_HZ_PID}" "${REJECTED_HZ_PID}" 2>/dev/null || true
wait "${METRICS_PID}" 2>/dev/null || true
kill -INT "${NODE_PID}" 2>/dev/null || true
wait "${NODE_PID}" 2>/dev/null || true
trap - INT TERM EXIT

printf 'Benchmark output: %s\n' "${OUTPUT_DIR}"
printf 'Bag exit status: %s\n' "${BAG_STATUS}"
printf 'Processed frames: %s\n' "${PROCESSED_FRAMES:-unknown}"
printf 'PCD: %s\n' "${PCD_PATH}"
printf 'Review %s, %s, %s, %s, %s and %s for CPU/RSS, input throughput, rejected-point behavior, save status and bag topics.\n' \
  "${METRICS}" "${STATIC_HZ}" "${REJECTED_HZ}" "${NODE_LOG}" "${SAVE_LOG}" "${BAG_INFO}"
exit "${FINAL_STATUS}"
