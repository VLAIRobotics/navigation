#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "record_fastdem_terrain_baseline"
stamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${1:-/tmp/fastdem_terrain_baseline_${stamp}}"
mkdir -p "$(dirname "${output_dir}")"

topics=(
  /fastdem/terrain_baseline_phase
  /fastdem/postprocess/gridmap
  /fastdem/mapping/gridmap
  /Odometry
  /tf
  /tf_static
  /patchworkpp/nonground
  /local_costmap/costmap
)

echo "Recording to ${output_dir}"
echo "Phase 1/4: keep the robot stationary on flat ground for 30 s."
echo "Phase 2/4: drive straight at low speed for 30 s."
echo "Phase 3/4: turn at low speed for 30 s."
echo "Phase 4/4: cross one real threshold or small bump during this 30 s phase."

ros2 bag record -o "${output_dir}" "${topics[@]}" \
  >"${output_dir}.record.log" 2>&1 &
recorder_pid=$!
recorder_status=''

stop_recorder()
{
  local status=0
  if [[ -z "${recorder_pid}" ]]; then
    return 0
  fi
  if kill -0 "${recorder_pid}" 2>/dev/null; then
    kill -INT "${recorder_pid}" 2>/dev/null || true
  fi
  if wait "${recorder_pid}"; then
    status=0
  else
    status=$?
  fi
  recorder_pid=''
  return "${status}"
}

handle_exit()
{
  local script_status=$1
  trap - EXIT
  if stop_recorder; then
    :
  else
    recorder_status=$?
    if [[ "${script_status}" -eq 0 ]]; then
      script_status=${recorder_status}
    fi
    echo "ros2 bag record exited with status ${recorder_status}" >&2
  fi
  exit "${script_status}"
}
trap 'handle_exit $?' EXIT
trap 'exit 130' INT TERM

mark_phase()
{
  local phase=$1
  ros2 topic pub --qos-durability transient_local --qos-reliability reliable \
    --once /fastdem/terrain_baseline_phase std_msgs/msg/String "{data: ${phase}}" \
    >/dev/null 2>&1
}

sleep 1
if kill -0 "${recorder_pid}" 2>/dev/null; then
  :
else
  if wait "${recorder_pid}"; then
    recorder_status=0
  else
    recorder_status=$?
  fi
  recorder_pid=''
  if [[ "${recorder_status}" -eq 0 ]]; then
    recorder_status=1
  fi
  echo "ros2 bag record failed to start (status ${recorder_status}); see ${output_dir}.record.log" >&2
  exit "${recorder_status}"
fi

for phase in stationary straight turn threshold; do
  echo "Starting ${phase}; follow the phase instruction above."
  mark_phase "${phase}"
  for _ in {1..30}; do
    sleep 1
    if kill -0 "${recorder_pid}" 2>/dev/null; then
      :
    else
      if wait "${recorder_pid}"; then
        recorder_status=0
      else
        recorder_status=$?
      fi
      recorder_pid=''
      if [[ "${recorder_status}" -eq 0 ]]; then
        recorder_status=1
      fi
      echo "ros2 bag record exited during ${phase} (status ${recorder_status}); see ${output_dir}.record.log" >&2
      exit "${recorder_status}"
    fi
  done
done

if stop_recorder; then
  :
else
  recorder_status=$?
  if [[ "${recorder_status}" -eq 0 ]]; then
    recorder_status=1
  fi
  echo "ros2 bag record failed while stopping (status ${recorder_status}); see ${output_dir}.record.log" >&2
  exit "${recorder_status}"
fi
trap - EXIT INT TERM
echo "Baseline recording complete: ${output_dir}"
