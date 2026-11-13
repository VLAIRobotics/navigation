#!/usr/bin/env bash
set -euo pipefail

# 一键启动 PCD 地图导航：按需生成二维占据图、静态 2.5D 地形图和 BBS/GICP 共用定位图，
# 首次定位交接后关闭 GICP RViz；导航 RViz 保持显示，GICP 跟踪与 map -> odom 平滑修正持续运行。
# 用法：./start_map_navigation.sh chassis_model:=two_wheel pcd_file:=/absolute/path/map.pcd visualize:=true
#      ./start_map_navigation.sh chassis_model:=omni
#      publish_initial_pose:=true initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "start_map_navigation"
STARTUP_STARTED_NS="$(date +%s%N)"
STARTUP_LAST_PHASE_NS="${STARTUP_STARTED_NS}"

mark_startup_timing() {
  local phase="$1"
  local now_ns phase_ms total_ms
  now_ns="$(date +%s%N)"
  phase_ms="$(( (now_ns - STARTUP_LAST_PHASE_NS) / 1000000 ))"
  total_ms="$(( (now_ns - STARTUP_STARTED_NS) / 1000000 ))"
  echo "[timing] startup phase=${phase} phase_ms=${phase_ms} total_ms=${total_ms}"
  STARTUP_LAST_PHASE_NS="${now_ns}"
}

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${SCRIPT_DIR}/../install/setup.bash}"
PCD_FILE="${PCD_FILE:-${MAP_FILE:-${SCRIPT_DIR}/../PCD/clean_map.pcd}}"
MAP_CACHE_DIR="${MAP_CACHE_DIR:-}"
SAVED_MAP_ROOT="${SAVED_MAP_ROOT:-}"
MAP_SEMANTICS_VERSION="measured_ground_clearance_v2"
ROBOT_HEIGHT="${ROBOT_HEIGHT:-1.20}"
CLEARANCE_MARGIN="${CLEARANCE_MARGIN:-0.10}"
REBUILD_MAP="${REBUILD_MAP:-false}"
RESOLUTION="${RESOLUTION:-0.10}"
DISPLAY_VOXEL_SIZE="${DISPLAY_VOXEL_SIZE:-0.03}"
RELOCALIZATION_VOXEL_SIZE="${RELOCALIZATION_VOXEL_SIZE:-0.20}"
ANALYSIS_RADIUS="${ANALYSIS_RADIUS:-0.30}"
MIN_VALID_NEIGHBORS="${MIN_VALID_NEIGHBORS:-16}"
MIN_GROUND_NEIGHBORS="${MIN_GROUND_NEIGHBORS:-1}"
MIN_GROUND_COMPONENT_CELLS="${MIN_GROUND_COMPONENT_CELLS:-10}"
GROUND_COMPONENT_HEIGHT_BAND="${GROUND_COMPONENT_HEIGHT_BAND:-0.35}"
VISUALIZE="${VISUALIZE:-true}"
CHASSIS_MODEL="${CHASSIS_MODEL:-two_wheel}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-}"
GICP_VISUALIZE="${GICP_VISUALIZE:-false}"
GICP_RVIZ_CONFIG="${GICP_RVIZ_CONFIG:-}"
OFFLINE_SNAPSHOT_DIR="${OFFLINE_SNAPSHOT_DIR:-}"
OFFLINE_SNAPSHOT_MAX_COUNT="${OFFLINE_SNAPSHOT_MAX_COUNT:-8}"
CONFIG_FILE="${CONFIG_FILE:-mid360.yaml}"
CONFIG_PATH="${CONFIG_PATH:-}"
ENABLE_CHASSIS=true
CAN_INTERFACE=can3
USE_SIM_TIME="${USE_SIM_TIME:-false}"
BODY_TO_BASE_X="${BODY_TO_BASE_X:--0.098346}"
BODY_TO_BASE_Y="${BODY_TO_BASE_Y:-0.004472}"
BODY_TO_BASE_YAW="${BODY_TO_BASE_YAW:-0.0}"
BODY_TO_BASE_ROLL="${BODY_TO_BASE_ROLL:-}"
BODY_TO_BASE_PITCH="${BODY_TO_BASE_PITCH:-}"
HANDOFF_TOPIC="${HANDOFF_TOPIC:-/localization/map_odom_handoff}"
LOCALIZATION_READY_TOPIC="${LOCALIZATION_READY_TOPIC:-/localization/handoff_ready}"
NAVIGATION_PAUSED_TOPIC="${NAVIGATION_PAUSED_TOPIC:-/navigation_paused}"
NAVIGATION_PAUSE_REQUEST_TOPIC="${NAVIGATION_PAUSE_REQUEST_TOPIC:-/localization/navigation_pause_request}"
NAVIGATION_PAUSE_STATUS_TOPIC="${NAVIGATION_PAUSE_STATUS_TOPIC:-/localization/navigation_pause_status}"
HANDOFF_SMOOTHING_TIME_CONSTANT_SEC="${HANDOFF_SMOOTHING_TIME_CONSTANT_SEC:-4.0}"
HANDOFF_MAX_TRANSLATION_SPEED_MPS="${HANDOFF_MAX_TRANSLATION_SPEED_MPS:-0.02}"
HANDOFF_MAX_ROTATION_SPEED_DEG_S="${HANDOFF_MAX_ROTATION_SPEED_DEG_S:-1.0}"
HANDOFF_UPDATE_FREQUENCY_HZ="${HANDOFF_UPDATE_FREQUENCY_HZ:-100.0}"
HANDOFF_READY_TIMEOUT_SEC="${HANDOFF_READY_TIMEOUT_SEC:-20}"
CORE_RESTART_DELAY_SEC="${CORE_RESTART_DELAY_SEC:-2}"
DISPLAY_PCD="${DISPLAY_PCD:-true}"
CAPTURE_CORES="${CAPTURE_CORES:-false}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
MAP_NAV_PUBLISH_INITIAL_POSE=false
MAP_NAV_INITIAL_POSE_X=0.0
MAP_NAV_INITIAL_POSE_Y=0.0
MAP_NAV_INITIAL_POSE_YAW=0.0
MAP_NAV_INITIAL_POSE_TIMEOUT=30.0

source_setup() {
  local setup_file="$1"
  set +u
  source "${setup_file}"
  set -u
}

usage() {
  cat <<'EOF'
用法: start_map_navigation.sh [chassis_model:=two_wheel|omni] [pcd_file:=/absolute/path/map.pcd] [map_cache_dir:=/path/cache] [saved_map_root:=/path/saved_maps] [rebuild_map:=true] [map_navigation.launch.py 参数...]

启动顺序: 先启动地图导航和 NavigationGate，Nav2 lifecycle 保持未激活；首次定位完成并由 Gate 确认暂停、handoff 发布 map→odom 基准后，Gate 才允许 Nav2 激活。

环境变量:
  PCD_FILE          PCD 地图，默认 <workspace>/PCD/clean_map.pcd，也可用 MAP_FILE
  MAP_CACHE_DIR     转换缓存目录，默认 <pcd目录>/<文件名>_nav
  SAVED_MAP_ROOT    融合导航地图保存目录，默认 <pcd目录>/<文件名>_nav_saved，与转换缓存分开
  REBUILD_MAP       true 时强制重建
  RESOLUTION        2.5D 栅格分辨率，默认 0.10 m
  ROBOT_HEIGHT / CLEARANCE_MARGIN 机器人高度和净空余量，默认 1.20 / 0.10 m
  DISPLAY_VOXEL_SIZE 静态结构显示 PCD 的三维体素，默认 0.03 m；地面 RViz 单独按 0.12 m XY 采样
  RELOCALIZATION_VOXEL_SIZE BBS/GICP 共用定位 PCD 的体素，默认 0.20 m
  tracking_frequency_hz     定位验收后的 GICP 更新频率，默认 1 Hz；busy 时只保留最新输入
  tracking_input_voxel_size 单独运行 GICP 时的跟踪输入体素，默认 0.25 m；本入口跳过该降采样
  tracking_max_iterations   首次定位后的 GICP 迭代上限，默认 25
  tracking_max_processing_time_sec GICP 跟踪结果最大耗时，默认 0.8 s
  tracking_small_correction_* 小修正门限；超过时需连续帧一致性确认
  tracking_consistency_*     连续候选变换容差与确认帧数
  handoff_smoothing_time_constant_sec TF 修正平滑时间常数，默认 4 s
  handoff_max_translation_speed_mps map→odom 机器人局部平移上限，默认 0.02 m/s
  handoff_max_rotation_speed_deg_s  map→odom 机器人局部旋转上限，默认 1°/s
  navigation_paused_topic Nav2 lifecycle 确认全部非 active 后的状态话题
  navigation_pause_request_topic handoff 按定位身份请求 Nav2 暂停的话题
  navigation_pause_status_topic NavigationGate 对暂停请求发布新鲜会话确认的话题
  display_max_height_above_ground:=1.8  PCD 显示离地高度上限，默认 1.8 m；更高点不显示
  local_cloud_max_height_above_ground:=1.8 实时局部点云离地高度上限，默认 1.8 m
  ANALYSIS_RADIUS   FastDEM 特征分析半径，默认 0.30 m
  MIN_VALID_NEIGHBORS FastDEM 特征窗口最少有效邻居数，默认 16
  MIN_GROUND_NEIGHBORS       地面格最少相邻候选数，默认 1
  MIN_GROUND_COMPONENT_CELLS 保留地面分量的最小格数，默认 10
  GROUND_COMPONENT_HEIGHT_BAND 相对最低主地面的高度带，默认 0.35 m
  ROS_SETUP         ROS 2 setup 路径
  WORKSPACE_SETUP   工作区 install/setup.bash 路径
  CHASSIS_MODEL     底盘模型，two_wheel（默认）或 omni；omni 自动选择对应 Nav2 参数文件
  NAV2_PARAMS_FILE  可选的 Nav2 参数文件覆盖；命令行 params_file:=... 也可覆盖
  HANDOFF_TOPIC / LOCALIZATION_READY_TOPIC 话题前缀；启动时自动追加本次会话 ID
  NAVIGATION_PAUSED_TOPIC 自动追加本次会话 ID，仅用于生命周期诊断
  NAVIGATION_PAUSE_REQUEST_TOPIC / NAVIGATION_PAUSE_STATUS_TOPIC 自动追加本次会话 ID，并关联当前定位身份
  CORE_RESTART_DELAY_SEC 定位 core 退出后重启前的等待时间，默认 2 s
  HANDOFF_READY_TIMEOUT_SEC transient-local 交接兜底等待时间，默认 20 s
  DISPLAY_PCD       true 时在 RViz 显示缓存三维 PCD，默认 true
  GICP_VISUALIZE    定位阶段是否显示 GICP RViz，默认 false
  GICP_RVIZ_CONFIG GICP 定位 RViz 配置，默认 bringup/rviz/gicp_relocalization.rviz
  CAPTURE_CORES     true 时为启动子进程设置 core size=unlimited；默认 false
  VISUALIZE         是否显示导航 RViz，默认 true；由 map_navigation.launch.py 启动

其余参数原样传给 bringup/map_navigation.launch.py，例如 visualize:=false、enable_chassis:=false。
默认由 3D-BBS 自动搜索全局位姿，再由 GICP 质量门控决定是否交接，无需人工发布初始位姿。
需要人工指定初值时，可在 RViz 使用 “2D Pose Estimate”，或传
publish_initial_pose:=true 并用 initial_pose_x/y/yaw 设置地图坐标。质量门控未通过时，
导航与底盘仍保持禁用。
Ctrl+C 会先请求导航 launch 停车并关闭底盘，再通知本次启动的所有 launch 退出；
所有启动进程共用 4 秒退出期限，第二次 Ctrl+C 立即强制结束已登记进程组。
EOF
}

gicp_args=()
nav_args=()
params_file_arg_seen=0
for argument in "$@"; do
  case "${argument}" in
    chassis_model:=*) CHASSIS_MODEL="${argument#chassis_model:=}" ;;
    params_file:=*)
      NAV2_PARAMS_FILE="${argument#params_file:=}"
      params_file_arg_seen=1
      nav_args+=("${argument}")
      ;;
    pcd_file:=*) PCD_FILE="${argument#pcd_file:=}" ;;
    map_file:=*) PCD_FILE="${argument#map_file:=}" ;;
    map_cache_dir:=*) MAP_CACHE_DIR="${argument#map_cache_dir:=}" ;;
    saved_map_root:=*) SAVED_MAP_ROOT="${argument#saved_map_root:=}" ;;
    rebuild_map:=*) REBUILD_MAP="${argument#rebuild_map:=}" ;;
    robot_height:=*) ROBOT_HEIGHT="${argument#robot_height:=}" ;;
    clearance_margin:=*) CLEARANCE_MARGIN="${argument#clearance_margin:=}" ;;
    resolution:=*) RESOLUTION="${argument#resolution:=}" ;;
    display_voxel_size:=*) DISPLAY_VOXEL_SIZE="${argument#display_voxel_size:=}" ;;
    relocalization_voxel_size:=*) RELOCALIZATION_VOXEL_SIZE="${argument#relocalization_voxel_size:=}" ;;
    analysis_radius:=*) ANALYSIS_RADIUS="${argument#analysis_radius:=}" ;;
    min_valid_neighbors:=*) MIN_VALID_NEIGHBORS="${argument#min_valid_neighbors:=}" ;;
    min_ground_neighbors:=*) MIN_GROUND_NEIGHBORS="${argument#min_ground_neighbors:=}" ;;
    min_ground_component_cells:=*) MIN_GROUND_COMPONENT_CELLS="${argument#min_ground_component_cells:=}" ;;
    ground_component_height_band:=*) GROUND_COMPONENT_HEIGHT_BAND="${argument#ground_component_height_band:=}" ;;
    visualize:=*) VISUALIZE="${argument#visualize:=}" ;;
    gicp_visualize:=*) GICP_VISUALIZE="${argument#gicp_visualize:=}" ;;
    gicp_rviz_config:=*) GICP_RVIZ_CONFIG="${argument#gicp_rviz_config:=}" ;;
    offline_snapshot_dir:=*) OFFLINE_SNAPSHOT_DIR="${argument#offline_snapshot_dir:=}" ;;
    offline_snapshot_max_count:=*) OFFLINE_SNAPSHOT_MAX_COUNT="${argument#offline_snapshot_max_count:=}" ;;
    capture_cores:=*) CAPTURE_CORES="${argument#capture_cores:=}" ;;
    config_file:=*) CONFIG_FILE="${argument#config_file:=}" ;;
    config_path:=*) CONFIG_PATH="${argument#config_path:=}" ;;
    enable_chassis:=*) ENABLE_CHASSIS="${argument#enable_chassis:=}"; nav_args+=("${argument}") ;;
    can_interface:=*) CAN_INTERFACE="${argument#can_interface:=}"; nav_args+=("${argument}") ;;
    use_sim_time:=*) USE_SIM_TIME="${argument#use_sim_time:=}" ;;
    body_to_base_x:=*) BODY_TO_BASE_X="${argument#body_to_base_x:=}" ;;
    body_to_base_y:=*) BODY_TO_BASE_Y="${argument#body_to_base_y:=}" ;;
    body_to_base_yaw:=*) BODY_TO_BASE_YAW="${argument#body_to_base_yaw:=}" ;;
    body_to_base_roll:=*) BODY_TO_BASE_ROLL="${argument#body_to_base_roll:=}" ;;
    body_to_base_pitch:=*) BODY_TO_BASE_PITCH="${argument#body_to_base_pitch:=}" ;;
    handoff_topic:=*) HANDOFF_TOPIC="${argument#handoff_topic:=}" ;;
    localization_ready_topic:=*) LOCALIZATION_READY_TOPIC="${argument#localization_ready_topic:=}" ;;
    navigation_paused_topic:=*) NAVIGATION_PAUSED_TOPIC="${argument#navigation_paused_topic:=}" ;;
    navigation_pause_request_topic:=*) NAVIGATION_PAUSE_REQUEST_TOPIC="${argument#navigation_pause_request_topic:=}" ;;
    navigation_pause_status_topic:=*) NAVIGATION_PAUSE_STATUS_TOPIC="${argument#navigation_pause_status_topic:=}" ;;
    handoff_smoothing_time_constant_sec:=*) HANDOFF_SMOOTHING_TIME_CONSTANT_SEC="${argument#handoff_smoothing_time_constant_sec:=}" ;;
    handoff_max_translation_speed_mps:=*) HANDOFF_MAX_TRANSLATION_SPEED_MPS="${argument#handoff_max_translation_speed_mps:=}" ;;
    handoff_max_rotation_speed_deg_s:=*) HANDOFF_MAX_ROTATION_SPEED_DEG_S="${argument#handoff_max_rotation_speed_deg_s:=}" ;;
    handoff_update_frequency_hz:=*) HANDOFF_UPDATE_FREQUENCY_HZ="${argument#handoff_update_frequency_hz:=}" ;;
    display_pcd:=*) DISPLAY_PCD="${argument#display_pcd:=}" ;;
    map_yaml:=*|terrain_map_file:=*|display_pcd_file:=*|display_ground_cache_file:=*|display_structure_cache_file:=*)
      echo "请使用 map_cache_dir:=；地图与显示缓存文件由脚本自动生成" >&2
      exit 2
      ;;
    -h|--help) usage; exit 0 ;;
    publish_initial_pose:=*) MAP_NAV_PUBLISH_INITIAL_POSE="${argument#publish_initial_pose:=}" ;;
    initial_pose_x:=*) MAP_NAV_INITIAL_POSE_X="${argument#initial_pose_x:=}" ;;
    initial_pose_y:=*) MAP_NAV_INITIAL_POSE_Y="${argument#initial_pose_y:=}" ;;
    initial_pose_yaw:=*) MAP_NAV_INITIAL_POSE_YAW="${argument#initial_pose_yaw:=}" ;;
    initial_pose_timeout:=*) MAP_NAV_INITIAL_POSE_TIMEOUT="${argument#initial_pose_timeout:=}" ;;
    bbs_seed_retry_sec:=*|registration_voxel_size:=*|tracking_frequency_hz:=*|\
    tracking_input_voxel_size:=*|tracking_max_iterations:=*|\
    tracking_max_processing_time_sec:=*|tracking_small_correction_translation_m:=*|\
    tracking_small_correction_rotation_rad:=*|tracking_consistency_translation_tolerance_m:=*|\
    tracking_consistency_rotation_tolerance_rad:=*|tracking_consistency_frames_required:=*|\
    tracking_max_translation_correction:=*|tracking_max_rotation_correction:=*|\
    tracking_max_cloud_age_sec:=*|num_threads:=*|max_iterations:=*|\
    min_inlier_ratio:=*|max_geometry_rmse_m:=*|max_geometry_p95_m:=*|stable_frames_required:=*|\
    stable_duration:=*|max_stable_translation_spread:=*|max_stable_yaw_spread:=*|min_match_sectors:=*|\
    require_multistart_consistency:=*|max_multistart_translation_spread:=*|max_multistart_yaw_spread:=*|\
    validity_timeout:=*|max_consecutive_failures:=*|offline_snapshot_max_count:=*|\
    manual_seed_input_timeout_sec:=*|manual_seed_validation_timeout_sec:=*|\
    manual_seed_bbs_height_max_age_sec:=*|timing_log_period_ms:=*)
      gicp_args+=("${argument}") ;;
    rviz_config:=*) RVIZ_CONFIG="${argument#rviz_config:=}" ;;
    *) nav_args+=("${argument}") ;;
  esac
done

case "${CHASSIS_MODEL}" in
  two_wheel|diff)
    CHASSIS_MODEL="two_wheel"
    if [[ -z "${NAV2_PARAMS_FILE}" ]]; then
      NAV2_PARAMS_FILE="${SCRIPT_DIR}/../src/bringup/config/nav2_map_navigation.yaml"
    fi
    ;;
  omni)
    if [[ -z "${NAV2_PARAMS_FILE}" ]]; then
      NAV2_PARAMS_FILE="${SCRIPT_DIR}/../src/bringup/config/nav2_map_navigation.yaml"
    fi
    ;;
  *)
    echo "[!] Unsupported chassis_model '${CHASSIS_MODEL}'; use two_wheel or omni." >&2
    exit 2
    ;;
esac

if [[ -n "${NAV2_PARAMS_FILE}" && ! -r "${NAV2_PARAMS_FILE}" ]]; then
  echo "[!] Nav2 parameter file is not readable: ${NAV2_PARAMS_FILE}" >&2
  exit 2
fi

if [[ -n "${NAV2_PARAMS_FILE}" && "${params_file_arg_seen}" == 0 ]]; then
  nav_args+=("params_file:=${NAV2_PARAMS_FILE}")
fi
nav_args+=("chassis_model:=${CHASSIS_MODEL}")
echo "[*] Chassis model=${CHASSIS_MODEL}; Nav2 parameter source=${NAV2_PARAMS_FILE}"

required_clearance_m="$(python3 - "${ROBOT_HEIGHT}" "${CLEARANCE_MARGIN}" <<'PYCODE'
import math
import sys

try:
    height, margin = map(float, sys.argv[1:])
    clearance = height + margin
    if not all(map(math.isfinite, (height, margin, clearance))) or height <= 0 or margin < 0 or clearance > 3.4028234663852886e38:
        raise ValueError
except ValueError:
    sys.exit("robot_height must be positive and clearance_margin nonnegative; both must be finite")
print(format(clearance, ".17g"))
PYCODE
)"
clearance_id="$(printf '%s' "${required_clearance_m}" | sha256sum | awk '{print $1}')"

validate_chassis_can_preflight() {
  local enabled="${ENABLE_CHASSIS,,}"
  local details first_line flags can_state
  case "${enabled}" in
    true|1|yes|on) ;;
    false|0|no|off) return 0 ;;
    *) echo "enable_chassis must be true or false: ${ENABLE_CHASSIS}" >&2; return 2 ;;
  esac

  if ! details="$(ip -details link show dev "${CAN_INTERFACE}" 2>&1)"; then
    echo "[!] Chassis CAN preflight failed for ${CAN_INTERFACE}: ${details}" >&2
    return 1
  fi
  first_line="${details%%$'\n'*}"
  flags="${first_line#*<}"
  flags="${flags%%>*}"
  can_state="$(sed -nE 's/^[[:space:]]*can([[:space:]]+<[^>]+>)?[[:space:]]+state[[:space:]]+([A-Z-]+).*/\2/p' <<<"${details}" | head -n 1)"
  if [[ ",${flags}," != *,UP,* ]] ||
    ! grep -Eiq '^[[:space:]]*can([[:space:]]+<[^>]+>)?[[:space:]]+state[[:space:]]+ERROR-ACTIVE([[:space:]]|$)' <<<"${details}" ||
    ! grep -Eq '[[:space:]]bitrate[[:space:]]+[0-9]+' <<<"${details}"; then
    echo "[!] Chassis CAN preflight failed for ${CAN_INTERFACE} (state=${can_state:-UNKNOWN}); "\
      "requires interface UP, configured bitrate, and controller state ERROR-ACTIVE." >&2
    echo "[!] Resolve the CAN interface before localization starts; no interface settings were changed." >&2
    return 1
  fi
}

SESSION_ID="$(date +%s%N)_$$"
HANDOFF_TOPIC="${HANDOFF_TOPIC%/}_${SESSION_ID}"
LOCALIZATION_READY_TOPIC="${LOCALIZATION_READY_TOPIC%/}_${SESSION_ID}"
NAVIGATION_PAUSED_TOPIC="${NAVIGATION_PAUSED_TOPIC%/}_${SESSION_ID}"
NAVIGATION_PAUSE_REQUEST_TOPIC="${NAVIGATION_PAUSE_REQUEST_TOPIC%/}_${SESSION_ID}"
NAVIGATION_PAUSE_STATUS_TOPIC="${NAVIGATION_PAUSE_STATUS_TOPIC%/}_${SESSION_ID}"
nav_args+=("navigation_paused_topic:=${NAVIGATION_PAUSED_TOPIC}")
nav_args+=("navigation_pause_request_topic:=${NAVIGATION_PAUSE_REQUEST_TOPIC}")
nav_args+=("navigation_pause_status_topic:=${NAVIGATION_PAUSE_STATUS_TOPIC}")

validate_chassis_can_preflight || exit $?
check_residual_processes || exit 1

gicp_args+=(
  "publish_initial_pose:=${MAP_NAV_PUBLISH_INITIAL_POSE}"
  "initial_pose_x:=${MAP_NAV_INITIAL_POSE_X}"
  "initial_pose_y:=${MAP_NAV_INITIAL_POSE_Y}"
  "initial_pose_yaw:=${MAP_NAV_INITIAL_POSE_YAW}"
  "initial_pose_timeout:=${MAP_NAV_INITIAL_POSE_TIMEOUT}"
)

[[ -r "${ROS_SETUP}" ]] || { echo "ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
source_setup "${ROS_SETUP}"
if [[ -r "${WORKSPACE_SETUP}" ]]; then
  source_setup "${WORKSPACE_SETUP}"
else
  echo "[!] 找不到工作区环境: ${WORKSPACE_SETUP}" >&2
  echo "[!] 请先执行: colcon build --packages-select bringup --symlink-install" >&2
  exit 1
fi
mark_startup_timing "ros_environment_ready"

[[ -f "${PCD_FILE}" && -r "${PCD_FILE}" ]] || {
  echo "PCD map must be a readable regular file: ${PCD_FILE}" >&2
  exit 1
}
PCD_FILE="$(realpath "${PCD_FILE}")"

case "${REBUILD_MAP}" in
  true|True|TRUE|1|yes|YES) REBUILD_MAP=true ;;
  false|False|FALSE|0|no|NO) REBUILD_MAP=false ;;
  *) echo "rebuild_map must be true or false: ${REBUILD_MAP}" >&2; exit 2 ;;
esac

if [[ -z "${MAP_CACHE_DIR}" ]]; then
  pcd_name="$(basename "${PCD_FILE}")"
  pcd_stem="${pcd_name%.*}"
  MAP_CACHE_DIR="$(dirname "${PCD_FILE}")/${pcd_stem}_nav"
fi
MAP_CACHE_DIR="$(realpath -m "${MAP_CACHE_DIR}")"
mkdir -p "${MAP_CACHE_DIR}"
if [[ -z "${SAVED_MAP_ROOT}" ]]; then
  SAVED_MAP_ROOT="$(dirname "${PCD_FILE}")/$(basename "${PCD_FILE%.*}")_nav_saved"
fi
SAVED_MAP_ROOT="$(realpath -m "${SAVED_MAP_ROOT}")"
if [[ "${SAVED_MAP_ROOT}" == "${MAP_CACHE_DIR}" ||
  "${SAVED_MAP_ROOT}" == "${MAP_CACHE_DIR}/"* ]]; then
  echo "saved_map_root must be separate from the PCD conversion cache: ${MAP_CACHE_DIR}" >&2
  exit 2
fi

terrain_map_file="${MAP_CACHE_DIR}/terrain_map.npz"
display_pcd_file="${MAP_CACHE_DIR}/display_map.pcd"
display_ground_cache_file="${MAP_CACHE_DIR}/display_map_ground_cache.pcd"
display_structure_cache_file="${MAP_CACHE_DIR}/display_map_structure_index.bin"
relocalization_pcd_file="${MAP_CACHE_DIR}/relocalization_map.pcd"
map_yaml_file="${MAP_CACHE_DIR}/map.yaml"
map_pgm_file="${MAP_CACHE_DIR}/map.pgm"
stamp_file="${MAP_CACHE_DIR}/.source_stamp"
lock_file="${MAP_CACHE_DIR}/.convert.lock"
if [[ -z "${OFFLINE_SNAPSHOT_DIR}" ]]; then
  OFFLINE_SNAPSHOT_DIR="${MAP_CACHE_DIR}/relocalization_snapshots"
fi
converter_prefix="$(ros2 pkg prefix bringup 2>/dev/null || true)"
if [[ -z "${converter_prefix}" ]]; then
  echo "找不到 bringup 包；请先构建 bringup: colcon build --packages-select bringup --symlink-install" >&2
  exit 1
fi
converter="${converter_prefix}/lib/bringup/pcd_to_terrain_map"
[[ -x "${converter}" ]] || {
  echo "pcd_to_terrain_map not found: ${converter}" >&2
  echo "请先构建 bringup: colcon build --packages-select bringup --symlink-install" >&2
  exit 1
}

content_fingerprint() {
  local file="$1"
  local size
  size="$(stat -c '%s' "${file}")"
  {
    printf 'size=%s\n' "${size}"
    head -c 1048576 "${file}"
    if (( size > 1048576 )); then
      tail -c 1048576 "${file}"
    fi
  } | sha256sum | awk '{print $1}'
}

pcd_fingerprint() { content_fingerprint "${PCD_FILE}"; }
pcd_source_id() { sha256sum -- "${PCD_FILE}" | awk '{print $1}'; }
converter_fingerprint() { content_fingerprint "${converter}"; }
make_stamp() {
  printf -v map_stamp 'pcd_path=%q\npcd_fingerprint=%q\nconverter=%q\nconverter_fingerprint=%q\nmap_semantics=%q\nresolution=%q\ndisplay_voxel_size=%q\nrelocalization_voxel_size=%q\nanalysis_radius=%q\nmin_valid_neighbors=%q\nmin_ground_neighbors=%q\nmin_ground_component_cells=%q\nground_component_height_band=%q\nrequired_clearance_m=%q\n' \
    "${PCD_FILE}" "$(pcd_fingerprint)" "${converter}" "$(converter_fingerprint)" \
    "${MAP_SEMANTICS_VERSION}" "${RESOLUTION}" "${DISPLAY_VOXEL_SIZE}" \
    "${RELOCALIZATION_VOXEL_SIZE}" "${ANALYSIS_RADIUS}" "${MIN_VALID_NEIGHBORS}" "${MIN_GROUND_NEIGHBORS}" \
    "${MIN_GROUND_COMPONENT_CELLS}" "${GROUND_COMPONENT_HEIGHT_BAND}" "${required_clearance_m}"
  printf '%s' "${map_stamp}"
}

expected_stamp="$(make_stamp)"
mark_startup_timing "source_and_converter_fingerprints_ready"
map_source_id="$(pcd_source_id)"
cache_ready=false
cache_invalidation_reason=""
cache_validation_reason() {
  if [[ "${REBUILD_MAP}" == true ]]; then
    echo "forced_rebuild"
    return
  fi
  local missing=()
  local artifact
  for artifact in "${terrain_map_file}" "${display_pcd_file}" \
    "${display_ground_cache_file}" "${display_structure_cache_file}" \
    "${relocalization_pcd_file}" "${map_yaml_file}" "${map_pgm_file}"; do
    if [[ ! -s "${artifact}" ]]; then
      missing+=("$(basename "${artifact}")")
    fi
  done
  if ((${#missing[@]} > 0)); then
    echo "missing_outputs:${missing[*]}"
    return
  fi
  if [[ ! -s "${stamp_file}" ]]; then
    echo "cache_stamp_missing"
    return
  fi
  local changed=()
  local key expected_value actual_value
  while IFS='=' read -r key expected_value; do
    [[ -n "${key}" ]] || continue
    actual_value="$(awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "${stamp_file}")"
    if [[ "${actual_value}" != "${expected_value}" ]]; then
      changed+=("${key}")
    fi
  done <<<"${expected_stamp}"
  if ((${#changed[@]} > 0)); then
    echo "stamp_fields_changed:${changed[*]}"
  else
    echo "cache_reusable"
  fi
}
cache_invalidation_reason="$(cache_validation_reason)"
if [[ "${REBUILD_MAP}" == false && -s "${terrain_map_file}" && -s "${display_pcd_file}" && \
  -s "${display_ground_cache_file}" && -s "${display_structure_cache_file}" && \
  -s "${relocalization_pcd_file}" && -s "${map_yaml_file}" && \
  -s "${map_pgm_file}" && -s "${stamp_file}" && "$(<"${stamp_file}")" == "${expected_stamp}" ]]; then
  cache_ready=true
fi

if [[ "${cache_ready}" == true ]]; then
  echo "[*] Reusing cached 2.5D map: ${MAP_CACHE_DIR} (reason=${cache_invalidation_reason})"
else
  echo "[*] Rebuilding navigation map cache: reason=${cache_invalidation_reason}"
  exec {lock_fd}>"${lock_file}"
  if ! flock -w 0 "${lock_fd}"; then
    echo "[*] 等待另一个转换完成: ${MAP_CACHE_DIR}" >&2
    flock "${lock_fd}"
  fi
  expected_stamp="$(make_stamp)"
  cache_invalidation_reason="$(cache_validation_reason)"
  if [[ "${REBUILD_MAP}" == false && -s "${terrain_map_file}" && -s "${display_pcd_file}" && \
    -s "${display_ground_cache_file}" && -s "${display_structure_cache_file}" && \
    -s "${relocalization_pcd_file}" && -s "${map_yaml_file}" && \
    -s "${map_pgm_file}" && -s "${stamp_file}" && "$(<"${stamp_file}")" == "${expected_stamp}" ]]; then
    echo "[*] Reusing cached 2.5D map after lock: ${MAP_CACHE_DIR} (reason=${cache_invalidation_reason})"
  else
    temp_dir="$(mktemp -d "${MAP_CACHE_DIR}/.convert.XXXXXX")"
    cleanup_temp() { rm -rf -- "${temp_dir}"; }
    trap cleanup_temp EXIT
    conversion_pcd_fingerprint="$(pcd_fingerprint)"
    echo "[*] Generating 2D + static 2.5D map from ${PCD_FILE}"
    "${converter}" "${PCD_FILE}" "${temp_dir}/terrain_map.npz" "${temp_dir}/map.yaml" \
      "--resolution=${RESOLUTION}" "--analysis-radius=${ANALYSIS_RADIUS}" \
      "--required-clearance-m=${required_clearance_m}" \
      "--min-valid-neighbors=${MIN_VALID_NEIGHBORS}" \
      "--min-ground-neighbors=${MIN_GROUND_NEIGHBORS}" \
      "--min-ground-component-cells=${MIN_GROUND_COMPONENT_CELLS}" \
      "--ground-component-height-band=${GROUND_COMPONENT_HEIGHT_BAND}" \
      "--display-pcd=${temp_dir}/display_map.pcd" \
      "--display-voxel-size=${DISPLAY_VOXEL_SIZE}" \
      "--display-ground-cache=${temp_dir}/display_map_ground_cache.pcd" \
      "--display-structure-cache=${temp_dir}/display_map_structure_index.bin" \
      "--relocalization-pcd=${temp_dir}/relocalization_map.pcd" \
      "--relocalization-voxel-size=${RELOCALIZATION_VOXEL_SIZE}"
    [[ -s "${temp_dir}/terrain_map.npz" && -s "${temp_dir}/display_map.pcd" && \
      -s "${temp_dir}/display_map_ground_cache.pcd" && \
      -s "${temp_dir}/display_map_structure_index.bin" && \
      -s "${temp_dir}/relocalization_map.pcd" && -s "${temp_dir}/map.yaml" && \
      -s "${temp_dir}/map.pgm" ]] || {
      echo "terrain map converter did not produce all outputs" >&2
      exit 1
    }
    if [[ "$(pcd_fingerprint)" != "${conversion_pcd_fingerprint}" ]]; then
      echo "PCD changed while converting; refusing to start with a partial map" >&2
      exit 1
    fi
    mv -f -- "${temp_dir}/terrain_map.npz" "${terrain_map_file}"
    mv -f -- "${temp_dir}/display_map.pcd" "${display_pcd_file}"
    mv -f -- "${temp_dir}/display_map_ground_cache.pcd" "${display_ground_cache_file}"
    mv -f -- "${temp_dir}/display_map_structure_index.bin" "${display_structure_cache_file}"
    mv -f -- "${temp_dir}/relocalization_map.pcd" "${relocalization_pcd_file}"
    mv -f -- "${temp_dir}/map.yaml" "${map_yaml_file}"
    mv -f -- "${temp_dir}/map.pgm" "${map_pgm_file}"
    printf '%s' "${expected_stamp}" > "${stamp_file}"
    rm -rf -- "${temp_dir}"
    trap - EXIT
    echo "[*] Wrote cached 2.5D map: ${MAP_CACHE_DIR}"
  fi
  flock -u "${lock_fd}"
  exec {lock_fd}>&-
fi
mark_startup_timing "navigation_map_cache_ready"

if [[ "$(pcd_source_id)" != "${map_source_id}" ]]; then
  echo "PCD changed while selecting navigation maps; refusing to mix saved and converted versions" >&2
  exit 1
fi
saved_map_namespace="${SAVED_MAP_ROOT}/${MAP_SEMANTICS_VERSION}/${clearance_id}"
saved_map_version="${saved_map_namespace}/${map_source_id}/latest"
ground_source_file=""
occupancy_source_file=""
if [[ -s "${saved_map_version}/.source_id" &&
  "$(<"${saved_map_version}/.source_id")" == "${map_source_id}" &&
  -s "${saved_map_version}/map.yaml" && -s "${saved_map_version}/map.pgm" &&
  -s "${saved_map_version}/terrain_map.npz" &&
  -s "${saved_map_version}/ground_source.pgm" &&
  -s "${saved_map_version}/ground_source.yaml" &&
  -s "${saved_map_version}/occupancy_source.pgm" &&
  -s "${saved_map_version}/occupancy_source.yaml" ]]; then
  map_yaml_file="${saved_map_version}/map.yaml"
  terrain_map_file="${saved_map_version}/terrain_map.npz"
  ground_source_file="${saved_map_version}/ground_source.pgm"
  occupancy_source_file="${saved_map_version}/occupancy_source.pgm"
  echo "[*] Resuming saved navigation map: ${saved_map_version}"
else
  echo "[*] No saved navigation map matches PCD source ${map_source_id}; using conversion cache"
fi
mark_startup_timing "saved_map_selection_ready"

echo "[*] PCD map: ${PCD_FILE}"
echo "[*] Localization session: ${SESSION_ID}"
echo "[*] Handoff topic: ${HANDOFF_TOPIC}"
echo "[*] Localization ready topic: ${LOCALIZATION_READY_TOPIC}"
echo "[*] Relocalization PCD: ${relocalization_pcd_file} (${RELOCALIZATION_VOXEL_SIZE} m voxels)"
echo "[*] GICP input: /cloud_registered_body_downsampled (default tracking update: 1 Hz)"
echo "[*] Terrain map: ${terrain_map_file}"
echo "[*] Display PCD: ${DISPLAY_PCD}"
echo "[*] Relocalization RViz: ${GICP_VISUALIZE}"
echo "[*] Navigation RViz: ${VISUALIZE}"

core_args=(
  "config_file:=${CONFIG_FILE}"
  "use_sim_time:=${USE_SIM_TIME}"
  "body_to_base_x:=${BODY_TO_BASE_X}"
  "body_to_base_y:=${BODY_TO_BASE_Y}"
  "body_to_base_yaw:=${BODY_TO_BASE_YAW}"
  "handoff_topic:=${HANDOFF_TOPIC}"
  "handoff_ready_topic:=${LOCALIZATION_READY_TOPIC}"
  "navigation_pause_request_topic:=${NAVIGATION_PAUSE_REQUEST_TOPIC}"
  "navigation_pause_status_topic:=${NAVIGATION_PAUSE_STATUS_TOPIC}"
  "handoff_smoothing_time_constant_sec:=${HANDOFF_SMOOTHING_TIME_CONSTANT_SEC}"
  "handoff_max_translation_speed_mps:=${HANDOFF_MAX_TRANSLATION_SPEED_MPS}"
  "handoff_max_rotation_speed_deg_s:=${HANDOFF_MAX_ROTATION_SPEED_DEG_S}"
  "handoff_update_frequency_hz:=${HANDOFF_UPDATE_FREQUENCY_HZ}"
)
if [[ -n "${CONFIG_PATH}" ]]; then
  core_args+=("config_path:=${CONFIG_PATH}")
fi

gicp_args+=(
  "pcd_file:=${relocalization_pcd_file}"
  "navigation_map_file:=${terrain_map_file}"
  "offline_snapshot_dir:=${OFFLINE_SNAPSHOT_DIR}"
  "offline_snapshot_max_count:=${OFFLINE_SNAPSHOT_MAX_COUNT}"
  "odometry_topic:=/Odometry"
  "base_frame:=base_link"
  "startup_context:=chassis_model=${CHASSIS_MODEL},nav2_params=${NAV2_PARAMS_FILE},pcd=${PCD_FILE},terrain=${terrain_map_file},body_to_base_x=${BODY_TO_BASE_X},body_to_base_y=${BODY_TO_BASE_Y},body_to_base_yaw=${BODY_TO_BASE_YAW}"
  "config_file:=${CONFIG_FILE}"
  "use_sim_time:=${USE_SIM_TIME}"
  "body_to_base_x:=${BODY_TO_BASE_X}"
  "body_to_base_y:=${BODY_TO_BASE_Y}"
  "body_to_base_yaw:=${BODY_TO_BASE_YAW}"
  "handoff_topic:=${HANDOFF_TOPIC}"
  "enable_handoff:=false"
  "exit_after_localized:=false"
  "shutdown_on_exit:=true"
  "enable_sensors:=false"
  "publish_body_to_base_tf:=false"
  "visualize:=${GICP_VISUALIZE}"
  "gicp_visualize:=false"
)
if [[ -n "${CONFIG_PATH}" ]]; then
  gicp_args+=("config_path:=${CONFIG_PATH}")
fi

nav_args+=(
  "map_yaml:=${map_yaml_file}"
  "terrain_map_file:=${terrain_map_file}"
  "saved_map_root:=${saved_map_namespace}"
  "robot_height:=${ROBOT_HEIGHT}"
  "clearance_margin:=${CLEARANCE_MARGIN}"
  "map_source_id:=${map_source_id}"
  "localization_ready_topic:=${LOCALIZATION_READY_TOPIC}"
  "use_sim_time:=${USE_SIM_TIME}"
  "body_to_base_x:=${BODY_TO_BASE_X}"
  "body_to_base_y:=${BODY_TO_BASE_Y}"
  "body_to_base_yaw:=${BODY_TO_BASE_YAW}"
  "startup_epoch_ns:=${STARTUP_STARTED_NS}"
)
if [[ -n "${ground_source_file}" ]]; then
  nav_args+=("ground_source_file:=${ground_source_file}")
fi
if [[ -n "${occupancy_source_file}" ]]; then
  nav_args+=("occupancy_source_file:=${occupancy_source_file}")
fi
rviz_prefix=""
if [[ -z "${RVIZ_CONFIG}" || -z "${GICP_RVIZ_CONFIG}" ]]; then
  rviz_prefix="$(ros2 pkg prefix bringup)"
fi
if [[ -z "${RVIZ_CONFIG}" ]]; then
  if [[ "${DISPLAY_PCD}" == "true" ]]; then
    RVIZ_CONFIG="${rviz_prefix}/share/bringup/rviz/map_navigation.rviz"
  else
    RVIZ_CONFIG="${rviz_prefix}/share/bringup/rviz/map_navigation_terrain.rviz"
  fi
fi
if [[ -z "${GICP_RVIZ_CONFIG}" ]]; then
  GICP_RVIZ_CONFIG="${rviz_prefix}/share/bringup/rviz/gicp_relocalization.rviz"
fi
nav_args+=("visualize:=${VISUALIZE}" "rviz_config:=${RVIZ_CONFIG}")
if [[ "${DISPLAY_PCD}" == "true" ]]; then
  nav_args+=("display_pcd_file:=${display_pcd_file}")
  nav_args+=("display_ground_cache_file:=${display_ground_cache_file}")
  nav_args+=("display_structure_cache_file:=${display_structure_cache_file}")
fi

if [[ -n "${BODY_TO_BASE_ROLL}" ]]; then
  core_args+=("body_to_base_roll:=${BODY_TO_BASE_ROLL}")
  gicp_args+=("body_to_base_roll:=${BODY_TO_BASE_ROLL}")
  nav_args+=("body_to_base_roll:=${BODY_TO_BASE_ROLL}")
fi
if [[ -n "${BODY_TO_BASE_PITCH}" ]]; then
  core_args+=("body_to_base_pitch:=${BODY_TO_BASE_PITCH}")
  gicp_args+=("body_to_base_pitch:=${BODY_TO_BASE_PITCH}")
  nav_args+=("body_to_base_pitch:=${BODY_TO_BASE_PITCH}")
fi

core_pid=""
gicp_pid=""
gicp_rviz_pid=""
nav_pid=""
navigation_handoff_completed=false
watcher_pid=""
handoff_echo_pid=""
handoff_check_file=""
handoff_ready_pid=""
handoff_ready_file=""
nav_status=0
launch_group_ids=()
helper_pids=()
shutdown_started=false
force_shutdown_requested=false
background_start_in_progress=false
pending_shutdown_signal=""

background_job_is_running() {
  local target_pid="$1"
  local running_pid
  while IFS= read -r running_pid; do
    if [[ "${running_pid}" == "${target_pid}" ]]; then
      return 0
    fi
  done < <(jobs -p -r)
  return 1
}

register_helper_pid() {
  helper_pids+=("$1")
}

retire_helper_pid() {
  local retired_pid="$1"
  local candidate
  local -a retained_helper_pids=()
  for candidate in "${helper_pids[@]}"; do
    [[ "${candidate}" == "${retired_pid}" ]] || retained_helper_pids+=("${candidate}")
  done
  helper_pids=("${retained_helper_pids[@]}")
}

start_registered_launch() {
  # Explicitly restore default signal dispositions before exec; non-interactive
  # Bash launches background jobs with SIGINT ignored.
  background_start_in_progress=true
  setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); signal.signal(signal.SIGTERM, signal.SIG_DFL); signal.signal(signal.SIGQUIT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' "$@" &
  started_pid=$!
  launch_group_ids+=("${started_pid}")
  background_start_in_progress=false
  if [[ -n "${pending_shutdown_signal}" ]]; then
    local signal_name="${pending_shutdown_signal}"
    pending_shutdown_signal=""
    handle_shutdown_signal "${signal_name}"
  fi
}

start_registered_helper() {
  background_start_in_progress=true
  "$@" &
  started_helper_pid=$!
  register_helper_pid "${started_helper_pid}"
  background_start_in_progress=false
  if [[ -n "${pending_shutdown_signal}" ]]; then
    local signal_name="${pending_shutdown_signal}"
    pending_shutdown_signal=""
    handle_shutdown_signal "${signal_name}"
  fi
}

process_group_has_live_processes() {
  local group_id="$1"
  ps -eo pgid=,stat= 2>/dev/null |
    awk -v group_id="${group_id}" '$1 == group_id && $2 !~ /^Z/ { found = 1 } END { exit !found }'
}

retire_launch_group() {
  local retired_group_id="$1"
  local candidate
  local -a retained_group_ids=()
  for candidate in "${launch_group_ids[@]}"; do
    [[ "${candidate}" == "${retired_group_id}" ]] || retained_group_ids+=("${candidate}")
  done
  launch_group_ids=("${retained_group_ids[@]}")
}

registered_processes_are_running() {
  local group_id pid
  for group_id in "${launch_group_ids[@]}"; do
    if process_group_has_live_processes "${group_id}"; then
      return 0
    fi
  done
  for pid in "${helper_pids[@]}"; do
    if background_job_is_running "${pid}"; then
      return 0
    fi
  done
  return 1
}

force_kill_registered_processes() {
  local group_id pid
  for group_id in "${launch_group_ids[@]}"; do
    kill -KILL -- "-${group_id}" 2>/dev/null || true
  done
  for pid in "${helper_pids[@]}"; do
    kill -KILL "${pid}" 2>/dev/null || true
  done
}

shutdown_registered_processes() {
  local group_id pid deadline term_deadline term_sent=false

  echo "[*] Requesting chassis stop through the navigation launch"
  if [[ -n "${nav_pid}" ]]; then
    kill -INT "${nav_pid}" 2>/dev/null || true
  fi
  # Signal each remaining launch parent immediately after the chassis-bearing
  # navigation launch. ROS launch then owns its normal child shutdown sequence.
  for group_id in "${launch_group_ids[@]}"; do
    [[ "${group_id}" == "${nav_pid}" ]] && continue
    kill -INT "${group_id}" 2>/dev/null || true
  done
  for pid in "${helper_pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done

  term_deadline=$((SECONDS + 2))
  deadline=$((SECONDS + 4))
  while (( SECONDS < deadline )) && registered_processes_are_running; do
    if [[ "${force_shutdown_requested}" == true ]]; then
      break
    fi
    if [[ "${term_sent}" == false ]] && (( SECONDS >= term_deadline )); then
      for group_id in "${launch_group_ids[@]}"; do
        if process_group_has_live_processes "${group_id}"; then
          kill -TERM -- "-${group_id}" 2>/dev/null || true
        fi
      done
      term_sent=true
    fi
    sleep 0.05
  done

  if registered_processes_are_running || [[ "${force_shutdown_requested}" == true ]]; then
    force_kill_registered_processes
  fi
  for group_id in "${launch_group_ids[@]}"; do
    wait "${group_id}" 2>/dev/null || true
  done
  for pid in "${helper_pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}

cleanup() {
  local exit_code=$?
  trap - EXIT
  if [[ "${shutdown_started}" == false ]]; then
    shutdown_started=true
    shutdown_registered_processes
  elif [[ "${force_shutdown_requested}" == true ]]; then
    force_kill_registered_processes
  fi
  if [[ -n "${handoff_check_file}" ]]; then
    rm -f -- "${handoff_check_file}"
  fi
  if [[ -n "${handoff_ready_file}" ]]; then
    rm -f -- "${handoff_ready_file}"
  fi
  exit "${exit_code}"
}

handle_shutdown_signal() {
  local signal_name="$1"
  if [[ "${background_start_in_progress}" == true ]]; then
    if [[ -n "${pending_shutdown_signal}" ]]; then
      force_shutdown_requested=true
    else
      pending_shutdown_signal="${signal_name}"
    fi
    return 0
  fi
  if [[ "${shutdown_started}" == true ]]; then
    force_shutdown_requested=true
    echo "[!] Second ${signal_name}; immediately killing registered navigation process groups" >&2
    force_kill_registered_processes
    return 0
  fi
  shutdown_started=true
  echo "[*] ${signal_name} received; stopping recovery and shutting down registered processes"
  shutdown_registered_processes
  exit 130
}

trap cleanup EXIT
trap 'handle_shutdown_signal SIGINT' INT
trap 'handle_shutdown_signal SIGTERM' TERM

stop_process_group_members() {
  local group_id="$1"
  local deadline
  [[ -n "${group_id}" ]] || return 0
  if ! process_group_has_live_processes "${group_id}"; then
    retire_launch_group "${group_id}"
    return 0
  fi

  kill -INT -- "-${group_id}" 2>/dev/null || true
  deadline=$((SECONDS + 2))
  while ((SECONDS < deadline)) && process_group_has_live_processes "${group_id}"; do
    sleep 0.1
  done
  if process_group_has_live_processes "${group_id}"; then
    kill -TERM -- "-${group_id}" 2>/dev/null || true
    deadline=$((SECONDS + 1))
    while ((SECONDS < deadline)) && process_group_has_live_processes "${group_id}"; do
      sleep 0.1
    done
  fi
  if process_group_has_live_processes "${group_id}"; then
    kill -KILL -- "-${group_id}" 2>/dev/null || true
    sleep 0.1
  fi
  if process_group_has_live_processes "${group_id}"; then
    echo "[!] Processes remain in navigation launch process group ${group_id}" >&2
    return 1
  fi
  retire_launch_group "${group_id}"
}

stop_launch_job() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  if background_job_is_running "${pid}"; then
    # Let ros2 launch enter shutdown before signaling any child in its process group.
    kill -INT "${pid}" 2>/dev/null || true
  fi
  stop_process_group_members "${pid}" || true
  wait "${pid}" 2>/dev/null || true
}

stop_background_pid() {
  local pid="$1"
  local deadline
  [[ -n "${pid}" ]] || return 0
  if background_job_is_running "${pid}"; then
    kill -TERM "${pid}" 2>/dev/null || true
    deadline=$((SECONDS + 1))
    while ((SECONDS < deadline)) && background_job_is_running "${pid}"; do
      sleep 0.1
    done
    if background_job_is_running "${pid}"; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi
  wait "${pid}" 2>/dev/null || true
  retire_helper_pid "${pid}"
}

stop_handoff_ready_watcher() {
  stop_background_pid "${handoff_ready_pid}"
  handoff_ready_pid=""
  if [[ -n "${handoff_ready_file}" ]]; then
    rm -f -- "${handoff_ready_file}"
    handoff_ready_file=""
  fi
}

capture_navigation_exit() {
  [[ -n "${nav_pid}" ]] || return 1
  if background_job_is_running "${nav_pid}"; then
    return 1
  fi
  if wait "${nav_pid}" 2>/dev/null; then
    nav_status=0
  else
    nav_status=$?
  fi
  stop_process_group_members "${nav_pid}" || true
  nav_pid=""
  return 0
}

sleep_with_navigation_monitor() {
  local duration="$1"
  local completed_pid wait_status
  local sleep_pid

  [[ "${shutdown_started}" == false ]] || return 1
  if [[ -z "${nav_pid}" ]]; then
    sleep "${duration}"
    return 0
  fi
  if capture_navigation_exit; then
    return 1
  fi

  start_registered_helper sleep "${duration}"
  sleep_pid="${started_helper_pid}"
  watcher_pid="${sleep_pid}"
  if wait -n -p completed_pid "${nav_pid}" "${sleep_pid}"; then
    wait_status=0
  else
    wait_status=$?
  fi

  if [[ "${completed_pid:-}" == "${nav_pid}" ]]; then
    nav_status=${wait_status}
    stop_process_group_members "${nav_pid}" || true
    nav_pid=""
    stop_background_pid "${sleep_pid}"
    watcher_pid=""
    return 1
  fi
  wait "${sleep_pid}" 2>/dev/null || true
  retire_helper_pid "${sleep_pid}"
  watcher_pid=""
  return 0
}

wait_for_fresh_handoff() {
  local deadline=$((SECONDS + HANDOFF_READY_TIMEOUT_SEC))
  local handoff_output handoff_status echo_pid echo_file

  while (( SECONDS < deadline )); do
    [[ "${shutdown_started}" == false ]] || return 3
    if capture_navigation_exit; then
      return 3
    fi
    if [[ -n "${core_pid}" ]] && ! background_job_is_running "${core_pid}"; then
      wait "${core_pid}" 2>/dev/null || true
      stop_process_group_members "${core_pid}" || true
      core_pid=""
      return 2
    fi

    echo_file="$(mktemp)"
    handoff_check_file="${echo_file}"
    start_registered_helper ros2 topic echo --no-daemon --once \
      --spin-time 2 --qos-durability transient_local --qos-reliability reliable --qos-depth 1 \
      "${LOCALIZATION_READY_TOPIC}" std_msgs/msg/Bool >"${echo_file}"
    echo_pid="${started_helper_pid}"
    handoff_echo_pid="${echo_pid}"

    while background_job_is_running "${echo_pid}"; do
      if capture_navigation_exit; then
        stop_background_pid "${echo_pid}"
        handoff_echo_pid=""
        rm -f -- "${echo_file}"
        handoff_check_file=""
        return 3
      fi
      if [[ -n "${core_pid}" ]] && ! background_job_is_running "${core_pid}"; then
        wait "${core_pid}" 2>/dev/null || true
        stop_process_group_members "${core_pid}" || true
        core_pid=""
        stop_background_pid "${echo_pid}"
        handoff_echo_pid=""
        rm -f -- "${echo_file}"
        handoff_check_file=""
        return 2
      fi
      if (( SECONDS >= deadline )); then
        break
      fi
      sleep 0.1
    done

    if background_job_is_running "${echo_pid}"; then
      stop_background_pid "${echo_pid}"
      handoff_status=124
      handoff_output=""
    elif wait "${echo_pid}"; then
      handoff_status=0
      handoff_output="$(<"${echo_file}")"
    else
      handoff_status=$?
      handoff_output="$(<"${echo_file}")"
    fi
    retire_helper_pid "${echo_pid}"
    handoff_echo_pid=""
    rm -f -- "${echo_file}"
    handoff_check_file=""

    if [[ -n "${core_pid}" ]] && ! background_job_is_running "${core_pid}"; then
      wait "${core_pid}" 2>/dev/null || true
      stop_process_group_members "${core_pid}" || true
      core_pid=""
      return 2
    fi
    if [[ "${handoff_status}" -eq 124 ]]; then
      if (( SECONDS < deadline )); then
        continue
      fi
      echo "[!] Timed out after ${HANDOFF_READY_TIMEOUT_SEC} s waiting for a fresh ${LOCALIZATION_READY_TOPIC}=true" >&2
      return 124
    elif [[ "${handoff_status}" -ne 0 ]]; then
      echo "[!] ROS 2 CLI failed while checking ${LOCALIZATION_READY_TOPIC} (status ${handoff_status})" >&2
      return "${handoff_status}"
    elif [[ "${handoff_output}" =~ (^|[[:space:]])data:[[:space:]]*true([[:space:]]|$) ]]; then
      echo "[*] Fresh localization handoff confirmed: ${LOCALIZATION_READY_TOPIC}=true"
      return 0
    elif [[ "${handoff_output}" =~ (^|[[:space:]])data:[[:space:]]*false([[:space:]]|$) ]]; then
      if ! sleep_with_navigation_monitor 0.2; then
        return 3
      fi
    elif [[ -z "${handoff_output}" ]]; then
      if ! sleep_with_navigation_monitor 0.2; then
        return 3
      fi
    else
      echo "[!] ROS 2 CLI returned an unexpected Bool payload for ${LOCALIZATION_READY_TOPIC}: ${handoff_output}" >&2
      return 1
    fi
  done

  echo "[!] Timed out after ${HANDOFF_READY_TIMEOUT_SEC} s waiting for a fresh ${LOCALIZATION_READY_TOPIC}=true" >&2
  return 124
}
wait_for_handoff_ready_message() {
  exec python3 - "${LOCALIZATION_READY_TOPIC}" "$1" <<'PY'
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class HandoffWaiter(Node):
    def __init__(self, topic):
        super().__init__("map_navigation_handoff_waiter")
        self.ready = False
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(Bool, topic, self.on_ready, qos)

    def on_ready(self, message):
        self.ready = message.data
        if self.ready:
            Path(sys.argv[2]).write_text("ready\n", encoding="utf-8")


rclpy.init(args=[])
node = HandoffWaiter(sys.argv[1])
try:
    while rclpy.ok() and not node.ready:
        rclpy.spin_once(node, timeout_sec=0.2)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
sys.exit(0 if node.ready else 1)
PY
}

option_is_enabled() {
  case "${1,,}" in
    true|1|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

configure_core_dumps() {
  option_is_enabled "${CAPTURE_CORES}" || return 0
  if ! ulimit -c unlimited; then
    echo "[!] Could not enable core dumps for child processes" >&2
    return 1
  fi
  local core_pattern="unavailable"
  if [[ -r /proc/sys/kernel/core_pattern ]]; then
    IFS= read -r core_pattern < /proc/sys/kernel/core_pattern
  fi
  echo "[*] Core dumps enabled for child processes (limit=$(ulimit -c), pattern=${core_pattern})"
}

start_gicp_rviz() {
  option_is_enabled "${GICP_VISUALIZE}" || return 0
  [[ "${navigation_handoff_completed}" == false ]] || return 0
  if [[ -n "${gicp_rviz_pid}" ]] && background_job_is_running "${gicp_rviz_pid}"; then
    return 0
  fi
  if [[ -n "${gicp_rviz_pid}" ]]; then
    stop_process_group_members "${gicp_rviz_pid}" || true
  fi
  echo "[*] Starting GICP localization RViz"
  start_registered_launch ros2 run rviz2 rviz2 -d "${GICP_RVIZ_CONFIG}"
  gicp_rviz_pid="${started_pid}"
}

stop_gicp_rviz() {
  if [[ -n "${gicp_rviz_pid}" ]]; then
    stop_launch_job "${gicp_rviz_pid}"
    gicp_rviz_pid=""
  fi
}

complete_first_localization_handoff() {
  stop_gicp_rviz
  navigation_handoff_completed=true
}

start_map_navigation_stage() {
  local start_status
  [[ "${shutdown_started}" == false ]] || return 1
  if [[ -n "${nav_pid}" ]] && background_job_is_running "${nav_pid}"; then
    return 0
  fi
  if [[ -n "${nav_pid}" ]]; then
    stop_process_group_members "${nav_pid}" || true
  fi
  nav_pid=""
  echo "[*] Starting map navigation stage (NavigationGate keeps Nav2 inactive until localization is ready)"
  start_registered_launch ros2 launch bringup map_navigation.launch.py "${nav_args[@]}"
  nav_pid="${started_pid}"
  if ! kill -0 "${nav_pid}" 2>/dev/null; then
    set +e
    wait "${nav_pid}"
    start_status=$?
    set -e
    stop_process_group_members "${nav_pid}" || true
    nav_pid=""
    echo "[!] map navigation launch exited during startup (status ${start_status})" >&2
    [[ "${start_status}" -ne 0 ]] || start_status=1
    return "${start_status}"
  fi
  return 0
}

run_localization_cycle() {
  local mode="${1:-recovery}"
  local completed_pid wait_status handoff_status handoff_wait_status

  while true; do
    [[ "${shutdown_started}" == false ]] || return 0
    if [[ -z "${core_pid}" ]]; then
      echo "[*] Waiting ${CORE_RESTART_DELAY_SEC} s before restarting localization core"
      if ! sleep_with_navigation_monitor "${CORE_RESTART_DELAY_SEC}"; then
        return 0
      fi
      echo "[*] Restarting navigation core (Livox + FAST-LIO + handoff bridge)"
      start_registered_launch ros2 launch bringup navigation_core.launch.py "${core_args[@]}"
      core_pid="${started_pid}"
      echo "[*] navigation core launch PID=${core_pid}"
    fi

    if [[ "${mode}" == "initial" ]]; then
      echo "[*] Starting 3D PCD relocalization stage"
    else
      echo "[*] Rerunning 3D PCD relocalization against the restarted core"
    fi
    start_gicp_rviz
    start_registered_launch ros2 launch bringup livox_fastlio_gicp_relocalization.launch.py "${gicp_args[@]}"
    gicp_pid="${started_pid}"
    handoff_ready_file="$(mktemp)"
    start_registered_helper wait_for_handoff_ready_message "${handoff_ready_file}"
    handoff_ready_pid="${started_helper_pid}"
    while true; do
      if [[ -n "${nav_pid}" ]] && ! background_job_is_running "${nav_pid}"; then
        if wait "${nav_pid}"; then wait_status=0; else wait_status=$?; fi
        stop_process_group_members "${nav_pid}" || true
        completed_pid="${nav_pid}"
        break
      fi
      if ! background_job_is_running "${core_pid}"; then
        if wait "${core_pid}"; then wait_status=0; else wait_status=$?; fi
        stop_process_group_members "${core_pid}" || true
        completed_pid="${core_pid}"
        break
      fi
      if [[ -n "${handoff_ready_file}" && -s "${handoff_ready_file}" ]]; then
        echo "[*] Fresh localization handoff received; closing GICP RViz"
        mark_startup_timing "localization_handoff_ready"
        stop_handoff_ready_watcher
        complete_first_localization_handoff
        start_map_navigation_stage || return $?
        return 0
      fi
      if [[ -n "${handoff_ready_pid}" ]] && ! background_job_is_running "${handoff_ready_pid}"; then
        if wait "${handoff_ready_pid}"; then
          handoff_wait_status=0
        else
          handoff_wait_status=$?
        fi
        retire_helper_pid "${handoff_ready_pid}"
        handoff_ready_pid=""
        if [[ "${handoff_wait_status}" -ne 0 ]]; then
          echo "[!] Handoff watcher exited before localization became ready; using the post-localization check" >&2
        fi
      fi
      if ! background_job_is_running "${gicp_pid}"; then
        if wait "${gicp_pid}"; then wait_status=0; else wait_status=$?; fi
        stop_process_group_members "${gicp_pid}" || true
        completed_pid="${gicp_pid}"
        break
      fi
      sleep 0.1
    done

    if [[ -n "${nav_pid}" && "${completed_pid:-}" == "${nav_pid}" ]]; then
      nav_status=${wait_status}
      nav_pid=""
      stop_launch_job "${gicp_pid}"
      gicp_pid=""
      stop_handoff_ready_watcher
      if [[ "${mode}" == "initial" ]]; then
        echo "[!] Map navigation launch exited before first localization completed (status ${nav_status})" >&2
        [[ "${nav_status}" -ne 0 ]] || nav_status=1
        return "${nav_status}"
      fi
      return 0
    fi
    if [[ "${completed_pid:-}" == "${core_pid}" ]]; then
      echo "[!] Navigation core exited during relocalization (status ${wait_status})" >&2
      core_pid=""
      stop_launch_job "${gicp_pid}"
      gicp_pid=""
      stop_handoff_ready_watcher
      if [[ "${mode}" == "initial" ]]; then
        echo "[!] Navigation core failed during initial localization; map navigation was not started" >&2
        return 1
      fi
      if capture_navigation_exit; then
        return 0
      fi
      continue
    fi

    gicp_pid=""
    if [[ -z "${completed_pid:-}" ]]; then
      echo "[!] Could not identify the process that ended during relocalization" >&2
      return 1
    fi
    if ! background_job_is_running "${core_pid}"; then
      wait "${core_pid}" 2>/dev/null || true
      stop_process_group_members "${core_pid}" || true
      core_pid=""
      echo "[!] Navigation core exited as GICP completed; restarting the localization cycle" >&2
      if [[ "${mode}" == "initial" ]]; then
        echo "[!] Navigation core failed during initial localization; map navigation was not started" >&2
        return 1
      fi
      if capture_navigation_exit; then
        return 0
      fi
      continue
    fi
    if [[ "${wait_status}" -ne 0 ]]; then
      stop_handoff_ready_watcher
      if [[ "${mode}" == "initial" ]]; then
        echo "[!] GICP relocalization failed (status ${wait_status}); map navigation was not started" >&2
        return "${wait_status}"
      fi
      echo "[!] GICP relocalization failed (status ${wait_status}); retrying in ${CORE_RESTART_DELAY_SEC} s" >&2
      if ! sleep_with_navigation_monitor "${CORE_RESTART_DELAY_SEC}"; then
        return 0
      fi
      continue
    fi

    echo "[*] GICP exited successfully; waiting for a fresh localization handoff"
    stop_handoff_ready_watcher
    if wait_for_fresh_handoff; then
      handoff_status=0
    else
      handoff_status=$?
    fi
    if [[ "${handoff_status}" -eq 0 ]]; then
      mark_startup_timing "localization_handoff_ready"
      complete_first_localization_handoff
      start_map_navigation_stage || return $?
      return 0
    elif [[ "${handoff_status}" -eq 3 && -z "${nav_pid}" ]]; then
      return 0
    elif [[ "${handoff_status}" -eq 2 && -z "${core_pid}" ]]; then
      echo "[!] Navigation core exited before the new handoff; restarting the localization cycle" >&2
      if [[ "${mode}" == "initial" ]]; then
        echo "[!] Navigation core failed during initial localization; map navigation was not started" >&2
        return 1
      fi
      if capture_navigation_exit; then
        return 0
      fi
      continue
    fi
    if [[ "${mode}" == "initial" ]]; then
      echo "[!] Could not confirm ${LOCALIZATION_READY_TOPIC} (status ${handoff_status}); map navigation was not started" >&2
      return "${handoff_status}"
    fi
    echo "[!] Fresh localization handoff was not confirmed (status ${handoff_status}); retrying GICP" >&2
    if ! sleep_with_navigation_monitor "${CORE_RESTART_DELAY_SEC}"; then
      return 0
    fi
  done
}

configure_core_dumps || exit $?
echo "[*] Starting map navigation gate before initial localization"
start_map_navigation_stage || exit $?
echo "[*] Starting navigation core (FAST-LIO + handoff bridge)"
start_registered_launch ros2 launch bringup navigation_core.launch.py "${core_args[@]}"
core_pid="${started_pid}"
echo "[*] navigation core launch PID=${core_pid}"
set +e
run_localization_cycle initial
handoff_status=$?
set -e
if [[ "${handoff_status}" -ne 0 ]]; then
  exit "${handoff_status}"
fi
if [[ -z "${nav_pid}" ]]; then
  start_map_navigation_stage || exit $?
fi

echo "[*] Localization handoff is ready; NavigationGate may now activate Nav2"
echo "[*] Map navigation is starting; lifecycle logs will report Nav2 activation and readiness"
while [[ "${shutdown_started}" == false ]]; do
  if [[ -n "${nav_pid}" ]] && ! background_job_is_running "${nav_pid}"; then
    if wait "${nav_pid}"; then nav_or_core_status=0; else nav_or_core_status=$?; fi
    stop_process_group_members "${nav_pid}" || true
    completed_pid="${nav_pid}"
  elif ! background_job_is_running "${core_pid}"; then
    if wait "${core_pid}"; then nav_or_core_status=0; else nav_or_core_status=$?; fi
    stop_process_group_members "${core_pid}" || true
    completed_pid="${core_pid}"
  elif [[ -n "${gicp_pid}" ]] && ! background_job_is_running "${gicp_pid}"; then
    if wait "${gicp_pid}"; then nav_or_core_status=0; else nav_or_core_status=$?; fi
    stop_process_group_members "${gicp_pid}" || true
    completed_pid="${gicp_pid}"
  else
    sleep 0.1
    continue
  fi

  if [[ -n "${gicp_pid}" && "${completed_pid:-}" == "${gicp_pid}" ]]; then
    gicp_pid=""
    if [[ -n "${nav_pid}" ]] && background_job_is_running "${nav_pid}" &&
      [[ -n "${core_pid}" ]] && background_job_is_running "${core_pid}"; then
      echo "[!] Persistent GICP tracker exited (status ${nav_or_core_status}); restarting it while the navigation gate holds Nav2 on localization status"
      start_registered_launch ros2 launch bringup livox_fastlio_gicp_relocalization.launch.py "${gicp_args[@]}"
      gicp_pid="${started_pid}"
    elif [[ "${nav_or_core_status}" -ne 0 ]]; then
      echo "[!] Relocalization launch cleanup exited with status ${nav_or_core_status} after handoff" >&2
    fi
    continue
  elif [[ "${completed_pid:-}" == "${nav_pid}" ]]; then
    nav_status=${nav_or_core_status}
    nav_pid=""
    break
  elif [[ "${completed_pid:-}" == "${core_pid}" ]]; then
    core_pid=""
    echo "[!] Navigation core exited (status ${nav_or_core_status}); recovering localization before Nav2 resumes" >&2
    stop_launch_job "${gicp_pid}"
    gicp_pid=""
    run_localization_cycle recovery
    if [[ -z "${nav_pid}" ]]; then
      break
    fi
  else
    echo "[!] Could not identify the process that ended while monitoring navigation" >&2
    exit "${nav_or_core_status:-1}"
  fi
done
exit "${nav_status}"
