#!/usr/bin/env bash
set -euo pipefail

# 板端运行：MID360 + FAST-LIO + Patchwork++ + FastDEM + Nav2
# 全局规划器：Theta*；局部控制器：TEB
# 用法: ./start_nav2.sh chassis_model:=two_wheel sensor_height:=0.30 visualize:=true
#      ./start_nav2.sh chassis_model:=omni
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "start_nav2"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${SCRIPT_DIR}/../install/setup.bash}"

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

chassis_model="${CHASSIS_MODEL:-two_wheel}"
params_file="${NAV2_PARAMS_FILE:-}"
launch_args=()
params_file_arg_seen=0
for argument in "$@"; do
  case "${argument}" in
    chassis_model:=*) chassis_model="${argument#chassis_model:=}" ;;
    params_file:=*)
      params_file="${argument#params_file:=}"
      params_file_arg_seen=1
      launch_args+=("${argument}")
      ;;
    *) launch_args+=("${argument}") ;;
  esac
done

case "${chassis_model}" in
  two_wheel|diff)
    chassis_model="two_wheel"
    if [[ -z "${params_file}" ]]; then
      params_file="${SCRIPT_DIR}/../src/bringup/config/nav2_theta_teb.yaml"
    fi
    ;;
  omni)
    if [[ -z "${params_file}" ]]; then
      params_file="${SCRIPT_DIR}/../src/bringup/config/nav2_theta_teb.yaml"
    fi
    ;;
  *)
    echo "[!] Unsupported chassis_model '${chassis_model}'; use two_wheel or omni." >&2
    exit 2
    ;;
esac

if [[ -n "${params_file}" && ! -r "${params_file}" ]]; then
  echo "[!] Nav2 parameter file is not readable: ${params_file}" >&2
  exit 2
fi

if [[ -n "${params_file}" && "${params_file_arg_seen}" == 0 ]]; then
  launch_args+=("params_file:=${params_file}")
fi
launch_args+=("chassis_model:=${chassis_model}")
echo "[*] Chassis model=${chassis_model}; Nav2 parameter source=${params_file}"

if [[ "${NAV2_AUTO_RESTART:-1}" != "1" ]]; then
  exec ros2 launch bringup nav2_theta_teb.launch.py "${launch_args[@]}"
fi

max_restarts="${NAV2_MAX_RESTARTS:-3}"
restart_window="${NAV2_RESTART_WINDOW_SEC:-300}"
restart_count=0
window_started="$(date +%s)"
stopping=0
trap 'stopping=1' INT TERM

while (( stopping == 0 )); do
  set +e
  ros2 launch bringup nav2_theta_teb.launch.py "${launch_args[@]}"
  launch_status=$?
  set -e

  (( stopping == 1 )) && break
  now="$(date +%s)"
  if (( now - window_started >= restart_window )); then
    restart_count=0
    window_started="${now}"
  fi
  if (( restart_count >= max_restarts )); then
    echo "[!] Nav2 stopped ${restart_count} times within ${restart_window}s; refusing another automatic restart (last status ${launch_status})." >&2
    exit "${launch_status}"
  fi
  backoff=(2 5 10)
  delay="${backoff[$restart_count]:-10}"
  restart_count=$((restart_count + 1))
  echo "[!] Nav2 stack stopped (status ${launch_status}); restarting in ${delay}s (${restart_count}/${max_restarts})." >&2
  for ((remaining=delay; remaining>0 && stopping==0; remaining--)); do
    sleep 1
  done
done
