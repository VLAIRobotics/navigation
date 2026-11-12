#!/usr/bin/env bash
set -euo pipefail

# 启动 MID360 + FAST-LIO + 动态障碍过滤 + RViz，构建包含地面的完整点云地图。
# 用法:
#   ./start_fastlio_mapping.sh
#   ./start_fastlio_mapping.sh map_file:=/absolute/path/clean_map.pcd config_file:=mid360.yaml

# 保存命令和实际输出路径会在启动时打印。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "start_fastlio_mapping"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${SCRIPT_DIR}/../install/setup.bash}"
MAP_FILE="${MAP_FILE:-${SCRIPT_DIR}/../PCD/clean_map.pcd}"
MAP_SAVE_FILE="${MAP_SAVE_FILE:-${SCRIPT_DIR}/../PCD/test.pcd}"
CONFIG_FILE="${CONFIG_FILE:-mid360.yaml}"
RVIZ="${RVIZ:-true}"
ENABLE_STATIC_MAP_FILTER="${ENABLE_STATIC_MAP_FILTER:-true}"
WINDOW_RADIUS="${WINDOW_RADIUS:-5.0}"
STATIC_MAP_CONFIRMED_MISS_LIMIT="${STATIC_MAP_CONFIRMED_MISS_LIMIT:-10}"
STATIC_MAP_ARCHIVE_MIN_DURATION="${STATIC_MAP_ARCHIVE_MIN_DURATION:-1.0}"
STATIC_MAP_ARCHIVE_MIN_ORIGIN_SEPARATION="${STATIC_MAP_ARCHIVE_MIN_ORIGIN_SEPARATION:-0.5}"
BODY_TO_BASE_X="${BODY_TO_BASE_X:--0.098346}"
BODY_TO_BASE_Y="${BODY_TO_BASE_Y:-0.004472}"
BODY_TO_BASE_ROLL="${BODY_TO_BASE_ROLL:-}"
BODY_TO_BASE_PITCH="${BODY_TO_BASE_PITCH:-}"
BODY_TO_BASE_YAW="${BODY_TO_BASE_YAW:-0.0}"

source_setup() {
  local setup_file="$1"
  set +u
  source "${setup_file}"
  set -u
}

usage() {
  cat <<'EOF'
用法: start_fastlio_mapping.sh [map_file:=PATH] [map_save_file:=PATH] [config_file:=FILE] [rviz:=true|false]

动态障碍过滤默认开启，RViz 默认使用 5 m 机器人中心显示窗口。

环境变量:
  MAP_FILE       清理后的输出 PCD，默认 <workspace>/PCD/clean_map.pcd
  MAP_SAVE_FILE  FAST-LIO /map_save 原始点云输出 PCD，默认 <workspace>/PCD/test.pcd
  CONFIG_FILE    FAST-LIO 配置文件，默认 mid360.yaml
  RVIZ           是否启动 RViz，默认 true；也可传 rviz:=false
  ENABLE_STATIC_MAP_FILTER 是否启用动态障碍过滤，默认 true
  WINDOW_RADIUS  RViz 机器人中心显示窗口半径，默认 5.0 m
  STATIC_MAP_CONFIRMED_MISS_LIMIT  动态清理 miss 滞回，默认 10 帧（约 1 秒@10Hz）
  STATIC_MAP_ARCHIVE_MIN_DURATION  历史静态体素最少观测时间，默认 1.0 秒
  STATIC_MAP_ARCHIVE_MIN_ORIGIN_SEPARATION  历史静态体素最少跨越位移，默认 0.5 m
  body_to_base_x/y/yaw 和可选 roll/pitch 参数；z 固定为 0，保持与雷达同高
  ROS_SETUP      ROS 2 setup 路径
  WORKSPACE_SETUP 工作区 install/setup.bash 路径
EOF
}

for argument in "$@"; do
  case "${argument}" in
    map_file:=*) MAP_FILE="${argument#map_file:=}" ;;
    map_save_file:=*) MAP_SAVE_FILE="${argument#map_save_file:=}" ;;
    config_file:=*) CONFIG_FILE="${argument#config_file:=}" ;;
    rviz:=*) RVIZ="${argument#rviz:=}" ;;
    enable_static_map_filter:=*) ENABLE_STATIC_MAP_FILTER="${argument#enable_static_map_filter:=}" ;;
    window_radius:=*) WINDOW_RADIUS="${argument#window_radius:=}" ;;
    static_map_confirmed_miss_limit:=*) STATIC_MAP_CONFIRMED_MISS_LIMIT="${argument#static_map_confirmed_miss_limit:=}" ;;
    static_map_archive_min_duration:=*) STATIC_MAP_ARCHIVE_MIN_DURATION="${argument#static_map_archive_min_duration:=}" ;;
    static_map_archive_min_origin_separation:=*) STATIC_MAP_ARCHIVE_MIN_ORIGIN_SEPARATION="${argument#static_map_archive_min_origin_separation:=}" ;;
    body_to_base_x:=*) BODY_TO_BASE_X="${argument#body_to_base_x:=}" ;;
    body_to_base_y:=*) BODY_TO_BASE_Y="${argument#body_to_base_y:=}" ;;
    body_to_base_roll:=*) BODY_TO_BASE_ROLL="${argument#body_to_base_roll:=}" ;;
    body_to_base_pitch:=*) BODY_TO_BASE_PITCH="${argument#body_to_base_pitch:=}" ;;
    body_to_base_yaw:=*) BODY_TO_BASE_YAW="${argument#body_to_base_yaw:=}" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: ${argument}" >&2; usage >&2; exit 2 ;;
  esac
done

check_residual_processes || exit 1

[[ -r "${ROS_SETUP}" ]] || { echo "ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
source_setup "${ROS_SETUP}"
if [[ -r "${WORKSPACE_SETUP}" ]]; then
  source_setup "${WORKSPACE_SETUP}"
else
  echo "[!] 找不到工作区环境: ${WORKSPACE_SETUP}" >&2
  echo "[!] 请先执行: colcon build --parallel-workers 1 --symlink-install" >&2
  exit 1
fi

if [[ "${CONFIG_FILE}" = /* ]]; then
  CONFIG_PATH="${CONFIG_FILE}"
else
  CONFIG_PATH="${SCRIPT_DIR}/../src/FAST_LIO_ROS2/config/${CONFIG_FILE}"
fi
[[ -r "${CONFIG_PATH}" ]] || { echo "FAST-LIO config not found: ${CONFIG_PATH}" >&2; exit 1; }
CONFIG_DIR="$(dirname -- "${CONFIG_PATH}")"
CONFIG_NAME="$(basename -- "${CONFIG_PATH}")"

MAP_FILE="$(realpath -m -- "$(dirname -- "${MAP_FILE}")")/$(basename -- "${MAP_FILE}")"
MAP_SAVE_FILE="$(realpath -m -- "${MAP_SAVE_FILE}")"
mkdir -p "$(dirname -- "${MAP_FILE}")" "$(dirname -- "${MAP_SAVE_FILE}")"
if [[ -e "${MAP_FILE}" ]]; then
  existing_map_info="$(stat -c '%s bytes, mtime %y' -- "${MAP_FILE}")"
  echo "[!] Existing clean map will be replaced if /static_map_save is called: ${MAP_FILE} (${existing_map_info})"
fi
echo "[*] FAST-LIO config: ${CONFIG_PATH}"
echo "[*] Clean PCD output: ${MAP_FILE}"
echo "[*] Raw FAST-LIO PCD output: ${MAP_SAVE_FILE}"
echo "[*] RViz: ${RVIZ}; dynamic obstacle filter: ${ENABLE_STATIC_MAP_FILTER}; window radius: ${WINDOW_RADIUS} m"
echo "[*] Stop with Ctrl-C to shut down. Save the cleaned map with:"
echo '    ros2 service call /static_map_save std_srvs/srv/Trigger "{}"'
echo "    output path: ${MAP_FILE}"
echo "[*] Save the raw FAST-LIO map with:"
echo '    ros2 service call /map_save std_srvs/srv/Trigger "{}"'
echo "    output path: ${MAP_SAVE_FILE}"

LAUNCH_ARGS=(
  config_path:="${CONFIG_DIR}"
  config_file:="${CONFIG_NAME}"
  map_file_path:="${MAP_SAVE_FILE}"
  visualize:="${RVIZ}"
  window_radius:="${WINDOW_RADIUS}"
  body_to_base_x:="${BODY_TO_BASE_X}"
  body_to_base_y:="${BODY_TO_BASE_Y}"
  body_to_base_yaw:="${BODY_TO_BASE_YAW}"
  enable_static_map_filter:="${ENABLE_STATIC_MAP_FILTER}"
  static_map_file:="${MAP_FILE}"
  static_map_voxel_size:=0.18
  static_map_min_observations:=4
  static_map_min_duration:=0.4
  static_map_candidate_timeout:=1.5
  static_map_confirmed_miss_limit:="${STATIC_MAP_CONFIRMED_MISS_LIMIT}"
  static_map_archive_min_duration:="${STATIC_MAP_ARCHIVE_MIN_DURATION}"
  static_map_archive_min_origin_separation:="${STATIC_MAP_ARCHIVE_MIN_ORIGIN_SEPARATION}"
  static_map_ray_angular_resolution:=2.0
  static_map_min_range:=0.5
  static_map_max_range:=40.0
  static_map_publish_period:=1.0
)
[[ -n "${BODY_TO_BASE_ROLL}" ]] && LAUNCH_ARGS+=("body_to_base_roll:=${BODY_TO_BASE_ROLL}")
[[ -n "${BODY_TO_BASE_PITCH}" ]] && LAUNCH_ARGS+=("body_to_base_pitch:=${BODY_TO_BASE_PITCH}")

setsid ros2 launch bringup livox_fastlio_bringup.launch.py "${LAUNCH_ARGS[@]}" &
MAPPING_PID=$!

stopping=0
cleanup() {
  local status=$?
  ((stopping)) && return
  stopping=1
  trap '' INT TERM
  trap - EXIT

  # setsid 为 launch 创建了独立会话；按会话清理，避免子节点残留。
  kill -INT -- "-${MAPPING_PID}" 2>/dev/null || true
  for _ in {1..50}; do
    pkill -0 -s "${MAPPING_PID}" 2>/dev/null || break
    sleep 0.2
  done
  if pkill -0 -s "${MAPPING_PID}" 2>/dev/null; then
    pkill -TERM -s "${MAPPING_PID}" 2>/dev/null || true
    for _ in {1..10}; do
      pkill -0 -s "${MAPPING_PID}" 2>/dev/null || break
      sleep 0.2
    done
    pkill -KILL -s "${MAPPING_PID}" 2>/dev/null || true
  fi
  wait "${MAPPING_PID}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup INT TERM EXIT

set +e
wait "${MAPPING_PID}"
CHILD_STATUS=$?
set -e
exit "${CHILD_STATUS}"
