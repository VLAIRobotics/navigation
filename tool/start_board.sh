#!/usr/bin/env bash
set -euo pipefail

# 板端运行：Livox 驱动 + FAST-LIO + Patchwork++ + FastDEM；默认无 RViz。
# 用法: ./start_board.sh [visualize:=true] [config_file:=mid360.yaml]
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "start_board"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${SCRIPT_DIR}/../install/setup.bash}"
PTP_INTERFACE="${PTP_INTERFACE:-enP8p1s0}"
PTP_LIDAR_IP="${PTP_LIDAR_IP:-192.168.1.194}"
PTP_WAIT_SECONDS="${PTP_WAIT_SECONDS:-30}"
PTP_SETTLE_SECONDS="${PTP_SETTLE_SECONDS:-10}"
PTP_LOG="${PTP_LOG:-/tmp/ptp4l-${PTP_INTERFACE}.log}"

[[ -r "${ROS_SETUP}" ]] || { echo "ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }

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
  echo "[!] 请先在工作区根目录执行: colcon build --symlink-install" >&2
fi
# echo "[*] 启动 PTP: ${PTP_INTERFACE}"
# sudo -v
# sudo ptp4l -i "${PTP_INTERFACE}" -S -m -l 6 >"${PTP_LOG}" 2>&1 &
# PTP_PID=$!
# trap 'kill "${PTP_PID}" 2>/dev/null || true' EXIT INT TERM
# sleep 1
# if ! kill -0 "${PTP_PID}" 2>/dev/null; then
#     echo "ptp4l 启动失败，未启动 ROS。日志: ${PTP_LOG}" >&2
#     exit 1
# fi

# if ! command -v tcpdump >/dev/null 2>&1; then
#     echo "tcpdump not found; cannot verify PTP traffic" >&2
#     exit 1
# fi

# echo "[*] 等待 MID360 PTP 响应: ${PTP_LIDAR_IP}"
# if ! sudo timeout "${PTP_WAIT_SECONDS}" tcpdump -ni "${PTP_INTERFACE}" -c 3 \
#     "src host ${PTP_LIDAR_IP} and (udp port 319 or udp port 320)" >/dev/null 2>&1; then
#     echo "PTP 未收到 MID360 响应，未启动 ROS。日志: ${PTP_LOG}" >&2
#     exit 1
# fi

# echo "[*] 已收到 MID360 PTP 响应，等待时钟稳定 ${PTP_SETTLE_SECONDS}s"
# sleep "${PTP_SETTLE_SECONDS}"

VISUALIZE="false"
LAUNCH_ARGS=()
for argument in "$@"; do
  case "${argument}" in
    visualize:=*) VISUALIZE="${argument#visualize:=}" ;;
    *) LAUNCH_ARGS+=("${argument}") ;;
  esac
done

ros2 launch bringup livox_fastlio_patchwork.launch.py "visualize:=${VISUALIZE}" "${LAUNCH_ARGS[@]}"
