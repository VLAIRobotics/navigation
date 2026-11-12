#!/usr/bin/env bash
set -euo pipefail

# 启动 Livox + FAST-LIO + small_gicp 地图重定位。
# 用法:
#   ./start_gicp_relocalization.sh
#   # 默认读取 <workspace>/PCD/clean_map.pcd 并启动 RViz

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "start_gicp_relocalization"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${SCRIPT_DIR}/../install/setup.bash}"
PCD_FILE="${PCD_FILE:-${MAP_FILE:-${SCRIPT_DIR}/../PCD/clean_map.pcd}}"
CONFIG_FILE="${CONFIG_FILE:-mid360.yaml}"
VISUALIZE="${VISUALIZE:-true}"
REGISTRATION_VOXEL_SIZE="${REGISTRATION_VOXEL_SIZE:-0.20}"
DISPLAY_VOXEL_SIZE="${DISPLAY_VOXEL_SIZE:-0.10}"
NUM_THREADS="${NUM_THREADS:-2}"
MAX_ITERATIONS="${MAX_ITERATIONS:-20}"
OFFLINE_SNAPSHOT_DIR="${OFFLINE_SNAPSHOT_DIR:-}"

source_setup() {
  local setup_file="$1"
  set +u
  source "${setup_file}"
  set -u
}

usage() {
  cat <<'EOF'
用法: start_gicp_relocalization.sh [pcd_file:=/absolute/path/map.pcd] [config_file:=mid360.yaml] [visualize:=true] [registration_voxel_size:=0.20] [display_voxel_size:=0.10] [num_threads:=2] [max_iterations:=20] [offline_snapshot_dir:=/tmp/gicp_snapshot]

环境变量:
  PCD_FILE       地图 PCD 文件，或包含 clean_map.pcd 的目录；默认 <workspace>/PCD/clean_map.pcd，也可用 MAP_FILE
  CONFIG_FILE    FAST-LIO 配置文件，默认 mid360.yaml
  VISUALIZE      是否启动 RViz，默认 true
  REGISTRATION_VOXEL_SIZE GICP 注册体素尺寸，默认 0.20 m
  DISPLAY_VOXEL_SIZE     仅 RViz 地图显示体素尺寸，默认 0.10 m
  NUM_THREADS            GICP/OpenMP 线程数，默认 2；4 线程仅用于基线对照
  MAX_ITERATIONS         GICP 最大迭代数，默认 20
  OFFLINE_SNAPSHOT_DIR   非空时保存一帧离线基线快照
  ROS_SETUP      ROS 2 setup 路径
  WORKSPACE_SETUP 工作区 install/setup.bash 路径
EOF
}

for argument in "$@"; do
  case "${argument}" in
    pcd_file:=*) PCD_FILE="${argument#pcd_file:=}" ;;
    map_file:=*) PCD_FILE="${argument#map_file:=}" ;;
    config_file:=*) CONFIG_FILE="${argument#config_file:=}" ;;
    visualize:=*) VISUALIZE="${argument#visualize:=}" ;;
    registration_voxel_size:=*) REGISTRATION_VOXEL_SIZE="${argument#registration_voxel_size:=}" ;;
    display_voxel_size:=*) DISPLAY_VOXEL_SIZE="${argument#display_voxel_size:=}" ;;
    num_threads:=*) NUM_THREADS="${argument#num_threads:=}" ;;
    max_iterations:=*) MAX_ITERATIONS="${argument#max_iterations:=}" ;;
    offline_snapshot_dir:=*) OFFLINE_SNAPSHOT_DIR="${argument#offline_snapshot_dir:=}" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: ${argument}" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -r "${ROS_SETUP}" ]] || { echo "ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
source_setup "${ROS_SETUP}"
if [[ -r "${WORKSPACE_SETUP}" ]]; then
  source_setup "${WORKSPACE_SETUP}"
else
  echo "[!] 找不到工作区环境: ${WORKSPACE_SETUP}" >&2
  echo "[!] 请先执行: colcon build --parallel-workers 1 --symlink-install" >&2
  exit 1
fi

if [[ -z "${PCD_FILE}" ]]; then
  echo "必须提供地图: pcd_file:=/absolute/path/map.pcd" >&2
  usage >&2
  exit 2
fi
if [[ -d "${PCD_FILE}" ]]; then
  candidate_pcd="${PCD_FILE%/}/clean_map.pcd"
  if [[ -f "${candidate_pcd}" && -r "${candidate_pcd}" ]]; then
    echo "[!] pcd_file is a directory; using ${candidate_pcd}"
    PCD_FILE="${candidate_pcd}"
  else
    echo "PCD directory does not contain a readable clean_map.pcd: ${PCD_FILE}" >&2
    exit 1
  fi
fi
[[ -f "${PCD_FILE}" && -r "${PCD_FILE}" ]] || {
  echo "PCD map must be a readable regular file: ${PCD_FILE}" >&2
  exit 1
}

echo "[*] PCD map: ${PCD_FILE}"
echo "[*] FAST-LIO config: ${CONFIG_FILE}"
echo "[*] RViz: ${VISUALIZE}"
echo "[*] GICP registration voxel: ${REGISTRATION_VOXEL_SIZE} m"
echo "[*] RViz display voxel: ${DISPLAY_VOXEL_SIZE} m"
echo "[*] GICP threads: ${NUM_THREADS}; max iterations: ${MAX_ITERATIONS}"
launch_args=(
  "pcd_file:=${PCD_FILE}"
  "config_file:=${CONFIG_FILE}"
  "visualize:=${VISUALIZE}"
  "registration_voxel_size:=${REGISTRATION_VOXEL_SIZE}"
  "display_voxel_size:=${DISPLAY_VOXEL_SIZE}"
  "num_threads:=${NUM_THREADS}"
  "max_iterations:=${MAX_ITERATIONS}"
)
if [[ -n "${OFFLINE_SNAPSHOT_DIR}" ]]; then
  launch_args+=("offline_snapshot_dir:=${OFFLINE_SNAPSHOT_DIR}")
fi
exec ros2 launch bringup livox_fastlio_gicp_relocalization.launch.py "${launch_args[@]}"
