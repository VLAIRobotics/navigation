#!/usr/bin/env bash
set -euo pipefail

# 一键验证底盘角速度标定。脚本自启 Nav2，结束时关闭本次启动的 Nav2。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/logging.sh"
start_tool_logging "verify_chassis_angular"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-${SCRIPT_DIR}/../install/setup.bash}"
NAV2_PID=''

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
  echo "[!] 请先执行 colcon build --symlink-install" >&2
  exit 1
fi

if ros2 node list 2>/dev/null | grep -qx '/chassis_driver'; then
  echo "已有 /chassis_driver 在运行，请先停止现有导航，再运行本脚本。" >&2
  exit 2
fi

stop_robot() {
  set +e
  if ros2 node list 2>/dev/null | grep -qx '/chassis_driver'; then
    timeout 2s ros2 topic pub --once -w 1 /cmd_vel_nav geometry_msgs/msg/Twist \
      "linear: {x: 0.0}, angular: {z: 0.0}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${NAV2_PID}" ]] && kill -0 "${NAV2_PID}" 2>/dev/null; then
    kill -INT "${NAV2_PID}" 2>/dev/null || true
    wait "${NAV2_PID}" 2>/dev/null || true
  fi
}
trap stop_robot EXIT
trap 'exit 130' INT TERM

echo "启动 Nav2，临时使用 max_angular_speed:=1.8 ..."
mkdir -p "${SCRIPT_DIR}/../logs"
"${SCRIPT_DIR}/start_nav2.sh" visualize:=false max_angular_speed:=1.8 \
  >"${SCRIPT_DIR}/../logs/verify_chassis_angular_nav2.log" 2>&1 &
NAV2_PID=$!

for attempt in $(seq 1 60); do
  if ros2 node list 2>/dev/null | grep -qx '/chassis_driver'; then
    break
  fi
  if ! kill -0 "${NAV2_PID}" 2>/dev/null; then
    echo "Nav2 启动失败，查看 logs/verify_chassis_angular_nav2.log" >&2
    exit 1
  fi
  sleep 1
done

if ! ros2 node list 2>/dev/null | grep -qx '/chassis_driver'; then
  echo "等待 /chassis_driver 超时，查看 logs/verify_chassis_angular_nav2.log" >&2
  exit 1
fi

for attempt in $(seq 1 60); do
  if ros2 lifecycle get /velocity_smoother 2>/dev/null | grep -q 'active \[3\]'; then
    break
  fi
  if ! kill -0 "${NAV2_PID}" 2>/dev/null; then
    echo "Nav2 在 velocity_smoother 激活前退出，查看 logs/verify_chassis_angular_nav2.log" >&2
    exit 1
  fi
  sleep 1
done

if ! ros2 lifecycle get /velocity_smoother 2>/dev/null | grep -q 'active \[3\]'; then
  echo "等待 /velocity_smoother active 超时，查看 logs/verify_chassis_angular_nav2.log" >&2
  exit 1
fi

echo "开始发送 +/-0.30 rad/s 脉冲，请保持机器人周围无障碍。"
python3 "${SCRIPT_DIR}/verify_chassis_angular.py"
