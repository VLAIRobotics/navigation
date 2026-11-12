#!/usr/bin/env bash
set -euo pipefail

# 采集起步转向、导航稳定性和安全门控诊断数据；导航启动日志由启动脚本另存到 logs/。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "record_nav2_stability"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${SCRIPT_DIR}/../install/setup.bash}"
usage() {
  cat <<'EOF'
用法: record_nav2_stability.sh [输出目录] [--raw]

在导航运行时录制地图、感知、速度、定位状态和日志；按 Ctrl-C 结束并写出 rosbag 元数据。
默认记录全局与局部 raw costmap、局部障碍/地形层快照、障碍源点、published footprint、平滑路径和 smooth_path action 状态，用于定位碰撞检查失败。
默认包含 /cloud_registered_body（用于 obstacle layer 清障）、/cloud_registered_body_clearance（点与逐点射线原点配对）和 /fastdem/terrain_valid；--raw 额外录制 /livox/lidar、/livox/imu，适合短时排查传感器时间戳与点云处理延迟。
输出目录默认位于仓库 `PCD/nav2_stability_<时间戳>`，rosbag 保存在其中的 bag/ 目录。
同时以 /proc 每秒采样进程 CPU、RSS、线程数、系统内存、温度和频率，写入 resources.jsonl；不依赖 pidstat。
只做轻量资源采样时可单独运行: python3 tool/sample_nav2_resources.py /tmp/nav2_resources.jsonl
EOF
}

include_raw=false
output_dir=''
for argument in "$@"; do
  case "${argument}" in
    --raw) include_raw=true ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "未知选项: ${argument}" >&2; usage >&2; exit 2 ;;
    *)
      if [[ -n "${output_dir}" ]]; then
        echo "只能指定一个输出目录" >&2
        usage >&2
        exit 2
      fi
      output_dir="${argument}"
      ;;
  esac
done

stamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${output_dir:-${SCRIPT_DIR}/../PCD/nav2_stability_${stamp}}"

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
  echo "[!] 找不到工作区环境: ${WORKSPACE_SETUP}" >&2
  echo "[!] 请先构建工作区，或设置 WORKSPACE_SETUP 指向 install/setup.bash" >&2
fi

mkdir -p "${output_dir}"

topics=(
  /Odometry
  /fastlio/health
  /fastlio/recovery_state
  /small_gicp/pose
  /small_gicp/preview_map_odom
  /small_gicp/relocalization_status
  /navigation_ready
  /initialpose
  /plan
  /local_plan
  /plan_display
  /local_plan_display
  /compute_path_to_pose/_action/status
  /follow_path/_action/status
  /navigate_to_pose/_action/status
  /chassis/reverse_blocked
  /chassis/safety_status
  /cmd_vel_nav
  /cmd_vel
  /patchworkpp/nonground
  /patchworkpp/ground
  /obstacles/blocking
  /obstacles/global_blocking
  /obstacles/ground_veto
  /obstacles/overhead
  /cloud_registered_body
  /cloud_registered_body_clearance
  /fastdem/mapping/cloud
  /fastdem/mapping/gridmap
  /fastdem/scan/preprocessed
  /fastdem/postprocess/cloud
  /fastdem/postprocess/gridmap
  /fastdem/terrain_valid
  /terrain_map/static
  /map_prior
  /map
  /map_updates
  /navigation/occupancy_source
  /local_costmap/costmap
  /local_costmap/costmap_updates
  /local_costmap/costmap_raw
  /local_costmap/temporal_obstacle_layer/costmap_raw
  /local_costmap/temporal_obstacle_layer/source_points
  /local_costmap/terrain_layer/costmap_raw
  /local_costmap/published_footprint
  /global_costmap/costmap
  /global_costmap/costmap_updates
  /global_costmap/costmap_raw
  /global_costmap/published_footprint
  /tf
  /tf_static
  /diagnostics
  /rosout
)
if topic_list="$(ros2 topic list --no-daemon 2>/dev/null)"; then
  while IFS= read -r topic; do
    if [[ "${topic}" =~ ^/localization/(map_odom_handoff|handoff_ready)_[0-9]+_[0-9]+$ ]]; then
      topics+=("${topic}")
    fi
  done <<<"${topic_list}"
else
  echo "[!] Could not discover session-scoped localization handoff topics; TF is still recorded." >&2
fi
if [[ "${include_raw}" == true ]]; then
  topics+=(/livox/lidar /livox/imu)
fi

echo "Saving navigation stability data to ${output_dir}"
date --iso-8601=seconds >"${output_dir}/started_at.txt"
echo "Topics recorded: ${topics[*]}"
if [[ "${include_raw}" == true ]]; then
  echo "Raw Livox recording enabled; keep this run short to limit disk and CPU load."
fi
echo "For performance comparison, record the same route for at least 30 minutes with RViz on and off."
echo "Also capture stationary start, obstacle appearance/clearing, goal changes and save/reload."
echo "The ros2 bag and rosout are under ${output_dir}."

recorder_pid=''
resource_sampler_pid=''
stop_recorder() {
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

stop_resource_sampler() {
  if [[ -z "${resource_sampler_pid}" ]]; then
    return 0
  fi
  if kill -0 "${resource_sampler_pid}" 2>/dev/null; then
    kill -TERM "${resource_sampler_pid}" 2>/dev/null || true
  fi
  wait "${resource_sampler_pid}" 2>/dev/null || true
  resource_sampler_pid=''
}

handle_exit() {
  local script_status=$1
  trap - EXIT INT TERM
  if stop_recorder; then
    :
  else
    local recorder_status=$?
    if [[ "${script_status}" -eq 0 ]]; then
      script_status=${recorder_status}
    fi
    echo "ros2 bag record exited with status ${recorder_status}" >&2
  fi
  stop_resource_sampler
  date --iso-8601=seconds >"${output_dir}/stopped_at.txt"
  echo "Navigation stability recording saved: ${output_dir}"
  exit "${script_status}"
}
trap 'handle_exit $?' EXIT
trap 'exit 130' INT TERM

ros2 bag record --include-hidden-topics -o "${output_dir}/bag" "${topics[@]}" \
  >"${output_dir}/ros2_bag_record.log" 2>&1 &
recorder_pid=$!

if command -v python3 >/dev/null 2>&1; then
  python3 "${SCRIPT_DIR}/sample_nav2_resources.py" "${output_dir}/resources.jsonl" \
    >"${output_dir}/resource_sampler.log" 2>&1 &
  resource_sampler_pid=$!
  echo "Per-process and system resource samples: ${output_dir}/resources.jsonl"
else
  echo "python3 is unavailable; resource sampling could not start." >&2
fi

sleep 1
if kill -0 "${recorder_pid}" 2>/dev/null; then
  echo "Recording started: ${output_dir}/bag"
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
  echo "ros2 bag record failed to start (status ${recorder_status}); see ${output_dir}/ros2_bag_record.log" >&2
  exit "${recorder_status}"
fi

ros2 node list --no-daemon >"${output_dir}/node_list.txt" 2>&1 || true
ros2 topic list -t --no-daemon >"${output_dir}/topic_list.txt" 2>&1 || true

for topic in "${topics[@]}"; do
  topic_file="${topic#/}"
  topic_file="${topic_file//\//_}"
  ros2 topic info -v "${topic}" --no-daemon >"${output_dir}/topic_info_${topic_file}.txt" 2>&1 || true
done

for node in \
  /bt_navigator \
  /controller_server \
  /planner_server \
  /velocity_smoother \
  /global_costmap/global_costmap \
  /local_costmap/local_costmap \
  /chassis_driver; do
  node_file="${node#/}"
  node_file="${node_file//\//_}"
  ros2 param dump "${node}" --no-daemon >"${output_dir}/params_${node_file}.yaml" 2>&1 || true
done

if wait "${recorder_pid}"; then
  recorder_status=0
else
  recorder_status=$?
fi
recorder_pid=''
exit "${recorder_status}"
