#!/usr/bin/env bash

start_tool_logging() {
  local script_name="$1"
  local log_dir="${SCRIPT_DIR}/../logs"
  local log_pattern="${script_name}_[0-9]*.log"
  local retention_count=20
  local index
  local -a log_names=()
  mkdir -p "${log_dir}"
  LOG_FILE="${log_dir}/${script_name}_$(date '+%Y%m%d_%H%M%S').log"
  : > "${LOG_FILE}"
  mapfile -t log_names < <(
    find "${log_dir}" -maxdepth 1 -type f -name "${log_pattern}" -printf '%f\n' | sort -r
  )
  for ((index = retention_count; index < ${#log_names[@]}; index++)); do
    rm -f -- "${log_dir}/${log_names[index]}"
  done
  exec > >(trap '' INT TERM; tee "${LOG_FILE}") 2>&1
  echo "[${script_name}] logging to: ${LOG_FILE}"
}

NAVIGATION_PROCESS_NAMES=(
  navigation_core.launch.py map_navigation.launch.py
  livox_fastlio_gicp_relocalization.launch.py
  map_odom_handoff_node small_gicp_relocalization_node
  fastlio_mapping livox_ros_driver2 rviz2 map_server
)
NAVIGATION_PROCESS_PATTERNS=(
  '(^|[[:space:]/])navigation_core\.launch\.py([[:space:]]|$)'
  '(^|[[:space:]/])map_navigation\.launch\.py([[:space:]]|$)'
  '(^|[[:space:]/])livox_fastlio_gicp_relocalization\.launch\.py([[:space:]]|$)'
  '(^|[[:space:]/])map_odom_handoff_node([[:space:]]|$)'
  '(^|[[:space:]/])small_gicp_relocalization_node([[:space:]]|$)'
  '(^|[[:space:]/])fastlio_mapping([[:space:]]|$)'
  '(^|[[:space:]/])livox_ros_driver2(_node)?([[:space:]]|$)'
  '(^|[[:space:]/])rviz2([[:space:]]|$)'
  '(^|[[:space:]/])map_server([[:space:]]|$)'
)

navigation_process_pids() {
  local process_pattern
  for process_pattern in "${NAVIGATION_PROCESS_PATTERNS[@]}"; do
    pgrep -f "${process_pattern}" 2>/dev/null || true
  done | sort -nu
}

navigation_processes_running() {
  [[ -n "$(navigation_process_pids)" ]]
}

report_navigation_processes() {
  local process_index process_name process_pattern matches
  for ((process_index = 0; process_index < ${#NAVIGATION_PROCESS_NAMES[@]}; process_index++)); do
    process_name="${NAVIGATION_PROCESS_NAMES[process_index]}"
    process_pattern="${NAVIGATION_PROCESS_PATTERNS[process_index]}"
    if matches="$(pgrep -af "${process_pattern}" 2>/dev/null)"; then
      printf '[!] Existing %s process(es):\n%s\n' "${process_name}" "${matches}" >&2
    fi
  done
}

signal_navigation_processes() {
  local signal_name="$1"
  local process_pid current_pid
  local -a ancestor_pids=()
  current_pid="$$"
  while [[ "${current_pid}" =~ ^[0-9]+$ ]] && ((current_pid > 1)); do
    ancestor_pids+=("${current_pid}")
    current_pid="$(ps -o ppid= -p "${current_pid}" 2>/dev/null | tr -d '[:space:]')"
  done

  while IFS= read -r process_pid; do
    [[ "${process_pid}" =~ ^[0-9]+$ ]] || continue
    local is_ancestor=0
    for current_pid in "${ancestor_pids[@]}"; do
      if [[ "${process_pid}" == "${current_pid}" ]]; then
        is_ancestor=1
        break
      fi
    done
    ((is_ancestor)) && continue
    kill -s "${signal_name}" "${process_pid}" 2>/dev/null || true
  done < <(navigation_process_pids)
}

wait_for_navigation_processes_to_exit() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  while ((SECONDS < deadline)); do
    navigation_processes_running || return 0
    sleep 0.1
  done
  ! navigation_processes_running
}

check_residual_processes() {
  local mode="${1:-refuse}"
  local process_index process_name process_pattern matches
  local found=0
  case "${mode}" in
    refuse|--terminate) ;;
    *) echo "Unknown residual-process mode: ${mode}" >&2; return 2 ;;
  esac

  for ((process_index = 0; process_index < ${#NAVIGATION_PROCESS_NAMES[@]}; process_index++)); do
    process_name="${NAVIGATION_PROCESS_NAMES[process_index]}"
    process_pattern="${NAVIGATION_PROCESS_PATTERNS[process_index]}"
    if matches="$(pgrep -af "${process_pattern}" 2>/dev/null)"; then
      printf '[!] Existing %s process(es):\n%s\n' "${process_name}" "${matches}" >&2
      found=1
    fi
  done
  ((found)) || return 0

  if [[ "${mode}" != "--terminate" ]]; then
    echo "[!] Stop the listed processes manually before starting; none were killed." >&2
    return 1
  fi

  echo "[!] Stopping residual navigation processes before startup."
  signal_navigation_processes INT
  if ! wait_for_navigation_processes_to_exit 10; then
    echo "[!] Navigation processes remain after SIGINT; escalating to SIGTERM." >&2
    signal_navigation_processes TERM
    wait_for_navigation_processes_to_exit 2 || true
  fi
  if navigation_processes_running; then
    echo "[!] Navigation processes remain after SIGTERM; escalating to SIGKILL." >&2
    signal_navigation_processes KILL
    wait_for_navigation_processes_to_exit 2 || true
  fi
  if navigation_processes_running; then
    report_navigation_processes
    echo "[!] Could not stop every residual navigation process; refusing to start." >&2
    return 1
  fi

  echo "[*] Residual navigation processes stopped."
}
