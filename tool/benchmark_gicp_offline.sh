#!/usr/bin/env bash
set -euo pipefail

# 用已保存的一帧扫描，离线扫描线程数 × 最大迭代数 × 初值扰动。
# 示例：
#   ./benchmark_gicp_offline.sh /tmp/gicp_snapshot /path/to/clean_map.pcd

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SNAPSHOT_DIR="${1:-}"
MAP_FILE="${2:-${MAP_FILE:-}}"
THREADS="${THREADS:-2,4}"
ITERATIONS="${ITERATIONS:-10,20,30}"
TRANSLATION_PERTURBATIONS="${TRANSLATION_PERTURBATIONS:-0,0.25,0.50}"
YAW_PERTURBATIONS="${YAW_PERTURBATIONS:-0,0.15,0.30}"
OUTPUT="${OUTPUT:-${SNAPSHOT_DIR:-.}/gicp_offline_benchmark.csv}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${SCRIPT_DIR}/../install/setup.bash}"

if [[ -z "${SNAPSHOT_DIR}" || ! -d "${SNAPSHOT_DIR}" ]]; then
  echo "用法: $0 <snapshot_dir> [map.pcd]" >&2
  exit 2
fi

source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"

benchmark_args=(
  --snapshot-dir "${SNAPSHOT_DIR}"
  --threads "${THREADS}"
  --iterations "${ITERATIONS}"
  --translation-perturbations "${TRANSLATION_PERTURBATIONS}"
  --yaw-perturbations "${YAW_PERTURBATIONS}"
  --output "${OUTPUT}"
)
if [[ -n "${MAP_FILE}" ]]; then
  benchmark_args+=(--map "${MAP_FILE}")
fi
exec ros2 run small_gicp_relocalization gicp_offline_benchmark "${benchmark_args[@]}"
