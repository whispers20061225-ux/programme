#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
TOPIC_TIMEOUT_SEC="${2:-20}"
HZ_SAMPLE_SEC="${3:-12}"
MIN_COLOR_HZ="${4:-3.0}"
MIN_DEPTH_HZ="${5:-3.0}"
START_TACTILE_SENSOR="${6:-true}"
CONDA_ENV="${7:-dayiprogramme312}"
ARM_TIMEOUT_SEC="${8:-20}"
MIN_TACTILE_HZ="${9:-3.0}"
ARM_REQUIRED="${10:-true}"
VISION_PROFILE_RAW="${11:-minimal}"
KEEP_LAUNCH="${KEEP_LAUNCH:-false}"
VISION_UI_MAX_FPS="${VISION_UI_MAX_FPS:-20.0}"
VISION_UI_STALE_TIMEOUT_SEC="${VISION_UI_STALE_TIMEOUT_SEC:-3.0}"
VISION_QOS_MODE="${VISION_QOS_MODE:-auto}"
USE_LATEST_FRAME_RELAY_ENV="${USE_LATEST_FRAME_RELAY:-}"
START_REALSENSE_MONITOR_ENV="${START_REALSENSE_MONITOR:-}"
VM_ARM_SERIAL_PORT_ENV="${VM_ARM_SERIAL_PORT:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BASE_PARAM_FILE="${PROJECT_ROOT}/ros2_ws/src/tactile_bringup/config/split_vm_app.yaml"
LAUNCH_LOG_DIR="${PROJECT_ROOT}/ros2_ws/log"
LAUNCH_LOG_FILE="${LAUNCH_LOG_DIR}/split_vm_app_$(date +%Y%m%d_%H%M%S).log"

VISION_PROFILE="${VISION_PROFILE_RAW,,}"
USE_RELAY_TOPICS="false"
ENABLE_REALSENSE_MONITOR="false"
VISION_COLOR_TOPIC="/camera/camera/color/image_raw"
VISION_DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
VISION_CAMERA_INFO_TOPIC="/camera/camera/color/camera_info"
PARAM_FILE_PATH=""
VM_ARM_SERIAL_PORT=""

log_step() { echo "[STEP] $*"; }
log_ok() { echo "[OK] $*"; }
log_fail() { echo "[FAIL] $*" >&2; }
is_true() { [[ "${1,,}" == "true" || "${1}" == "1" || "${1,,}" == "yes" ]]; }

has_ros_package() {
  local package_name="$1"
  set +e
  ros2 pkg prefix "${package_name}" >/dev/null 2>&1
  local rc=$?
  set -e
  return ${rc}
}

case "${VISION_PROFILE}" in
  minimal|lite|raw)
    VISION_PROFILE="minimal"
    USE_RELAY_TOPICS="false"
    ENABLE_REALSENSE_MONITOR="false"
    ;;
  relay|full)
    VISION_PROFILE="relay"
    USE_RELAY_TOPICS="true"
    ENABLE_REALSENSE_MONITOR="true"
    ;;
  *)
    echo "[WARN] unknown vision profile '${VISION_PROFILE_RAW}', fallback to minimal"
    VISION_PROFILE="minimal"
    USE_RELAY_TOPICS="false"
    ENABLE_REALSENSE_MONITOR="false"
    ;;
esac

if [[ -n "${USE_LATEST_FRAME_RELAY_ENV}" ]]; then
  if is_true "${USE_LATEST_FRAME_RELAY_ENV}"; then
    USE_RELAY_TOPICS="true"
  else
    USE_RELAY_TOPICS="false"
  fi
fi

if [[ -n "${START_REALSENSE_MONITOR_ENV}" ]]; then
  if is_true "${START_REALSENSE_MONITOR_ENV}"; then
    ENABLE_REALSENSE_MONITOR="true"
  else
    ENABLE_REALSENSE_MONITOR="false"
  fi
fi

if [[ "${USE_RELAY_TOPICS}" == "true" ]]; then
  VISION_COLOR_TOPIC="/camera/relay/color/image_raw"
  VISION_DEPTH_TOPIC="/camera/relay/aligned_depth_to_color/image_raw"
  VISION_CAMERA_INFO_TOPIC="/camera/relay/color/camera_info"
fi


wait_for_topic() {
  local topic="$1"
  local topic_alt="${topic#/}"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local topics
    topics="$(ros2 topic list 2>/dev/null || true)"
    if echo "${topics}" | grep -Fxq "${topic}" \
      || echo "${topics}" | grep -Fxq "${topic_alt}" \
      || echo "${topics}" | grep -Fxq "/${topic_alt}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

wait_for_message() {
  local topic="$1"
  local topic_alt="${topic#/}"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local rc=1
    set +e
    timeout 2s ros2 topic echo "${topic}" --once >/dev/null 2>&1
    rc=$?
    set -e
    if [[ ${rc} -eq 0 ]]; then
      return 0
    fi
    if [[ "${topic_alt}" != "${topic}" ]]; then
      set +e
      timeout 2s ros2 topic echo "${topic_alt}" --once >/dev/null 2>&1
      rc=$?
      set -e
      if [[ ${rc} -eq 0 ]]; then
        return 0
      fi
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

wait_for_service() {
  local service="$1"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local services
    services="$(ros2 service list 2>/dev/null || true)"
    if echo "${services}" | grep -Fxq "${service}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

ensure_param_file_copy() {
  if [[ -n "${PARAM_FILE_PATH}" ]]; then
    return 0
  fi
  PARAM_FILE_PATH="$(mktemp)"
  cp "${BASE_PARAM_FILE}" "${PARAM_FILE_PATH}"
}

configure_vm_arm_serial_port() {
  local serial_port="$1"
  local escaped_port="${serial_port//&/\&}"
  ensure_param_file_copy
  sed -i -E "0,/arm_serial_port:[[:space:]]*.*$/s#arm_serial_port:[[:space:]]*.*#    arm_serial_port: \"${escaped_port}\"#" "${PARAM_FILE_PATH}"
}

print_vm_arm_serial_diagnostics() {
  local had_nullglob=0
  local by_id_candidates=()
  local tty_candidates=()

  if shopt -q nullglob; then
    had_nullglob=1
  fi
  shopt -s nullglob
  by_id_candidates=(/dev/serial/by-id/*)
  tty_candidates=(/dev/ttyACM* /dev/ttyUSB*)
  if (( had_nullglob == 0 )); then
    shopt -u nullglob
  fi

  echo "[DIAG] VM serial devices (/dev/serial/by-id):"
  if (( ${#by_id_candidates[@]} > 0 )); then
    ls -l "${by_id_candidates[@]}" 2>/dev/null || true
  else
    echo "(none)"
  fi

  echo "[DIAG] VM serial devices (/dev/ttyACM* /dev/ttyUSB*):"
  if (( ${#tty_candidates[@]} > 0 )); then
    ls -l "${tty_candidates[@]}" 2>/dev/null || true
  else
    echo "(none)"
  fi
}

resolve_vm_arm_serial_port() {
  local had_nullglob=0
  local candidates=()
  local override_port="${VM_ARM_SERIAL_PORT_ENV}"
  local path=""
  local lower=""

  if [[ -n "${override_port}" ]]; then
    if [[ -e "${override_port}" ]]; then
      printf '%s\n' "${override_port}"
      return 0
    fi
    echo "[WARN] VM_ARM_SERIAL_PORT=${override_port} does not exist; auto-detecting instead." >&2
  fi

  if shopt -q nullglob; then
    had_nullglob=1
  fi
  shopt -s nullglob

  for path in /dev/serial/by-id/*; do
    lower="${path,,}"
    if [[ "${lower}" == *stm32* || "${lower}" == *stmicro* || "${lower}" == *virtual*com* || "${lower}" == *0483* ]]; then
      candidates+=("${path}")
    fi
  done

  if (( ${#candidates[@]} == 0 )); then
    for path in /dev/ttyACM*; do
      candidates+=("${path}")
    done
  fi

  if (( ${#candidates[@]} == 0 )); then
    for path in /dev/ttyUSB*; do
      candidates+=("${path}")
    done
  fi

  if (( had_nullglob == 0 )); then
    shopt -u nullglob
  fi

  if (( ${#candidates[@]} == 0 )); then
    return 1
  fi
  if (( ${#candidates[@]} > 1 )); then
    echo "[WARN] multiple VM STM32 serial candidates found; using ${candidates[0]}" >&2
  fi
  printf '%s\n' "${candidates[0]}"
}

call_setbool_service() {
  local service_name="$1"
  local data_value="$2"
  local timeout_sec="$3"
  local output=""
  local rc=1

  set +e
  output="$(timeout "${timeout_sec}s" ros2 service call "${service_name}" std_srvs/srv/SetBool "{data: ${data_value}}" 2>&1)"
  rc=$?
  set -e

  echo "${output}"
  if [[ ${rc} -ne 0 ]]; then
    return 1
  fi
  if echo "${output}" | tr '[:upper:]' '[:lower:]' | grep -Eq 'success[[:space:]]*[:=][[:space:]]*true'; then
    return 0
  fi
  return 1
}

wait_for_arm_connected_state() {
  local timeout_sec="$1"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local state_output=""
    local rc=1
    set +e
    state_output="$(timeout 3s ros2 topic echo /arm/state --once 2>/dev/null)"
    rc=$?
    set -e
    if [[ ${rc} -eq 0 ]] && echo "${state_output}" | tr '[:upper:]' '[:lower:]' | grep -Eq 'connected[[:space:]]*:[[:space:]]*true|connected[[:space:]]*=[[:space:]]*true'; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

wait_for_node() {
  local node="$1"
  local node_alt="${node#/}"
  local launch_tag="${node#/}"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local nodes
    nodes="$(ros2 node list 2>/dev/null || true)"
    if echo "${nodes}" | grep -Fxq "${node}" || echo "${nodes}" | grep -Fxq "${node_alt}"; then
      return 0
    fi
    # Fallback: if launch already confirms process started and no crash reported,
    # treat it as discovered to avoid false negatives from graph refresh latency.
    if [[ -f "${LAUNCH_LOG_FILE}" ]] \
      && grep -Fq "[INFO] [${launch_tag}-" "${LAUNCH_LOG_FILE}" \
      && ! grep -Fq "[ERROR] [${launch_tag}-" "${LAUNCH_LOG_FILE}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

sample_hz() {
  local topic="$1"
  local sample_sec="$2"
  local out_file
  local err_file
  local hz_pid
  local output
  out_file="$(mktemp)"
  err_file="$(mktemp)"

  set +e
  stdbuf -oL -eL ros2 topic hz "${topic}" >"${out_file}" 2>"${err_file}" &
  hz_pid=$!
  set -e

  sleep "${sample_sec}"
  kill "${hz_pid}" >/dev/null 2>&1 || true
  wait "${hz_pid}" >/dev/null 2>&1 || true

  output="$(cat "${out_file}" "${err_file}" 2>/dev/null || true)"
  rm -f "${out_file}" "${err_file}" || true
  echo "${output}" | awk '/average rate/ {print $3}' | tail -n 1
}

check_rate_ge() {
  local actual="$1"
  local minimum="$2"
  awk -v r="${actual}" -v m="${minimum}" 'BEGIN { exit ((r + 0.0 >= m + 0.0) ? 0 : 1) }'
}

dump_runtime_diagnostics() {
  echo "[DIAG] ros2 node list:"
  ros2 node list 2>/dev/null || true
  echo "[DIAG] ros2 topic list (filtered):"
  ros2 topic list 2>/dev/null | grep -E 'tactile|arm|camera|health' || true
  echo "[DIAG] tail launch log:"
  tail -n 120 "${LAUNCH_LOG_FILE}" 2>/dev/null || true
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    if [[ "${KEEP_LAUNCH}" == "true" ]]; then
      echo "[INFO] KEEP_LAUNCH=true, keeping split_vm_app.launch.py running (pid=${LAUNCH_PID})."
    else
      echo "[INFO] stopping split_vm_app.launch.py (pid=${LAUNCH_PID})"
      kill "${LAUNCH_PID}" 2>/dev/null || true
    fi
  fi
  if [[ -n "${PARAM_FILE_PATH:-}" && -f "${PARAM_FILE_PATH}" && "${PARAM_FILE_PATH}" == /tmp/* ]]; then
    rm -f "${PARAM_FILE_PATH}" || true
  fi
}
trap cleanup EXIT INT TERM

cd "${PROJECT_ROOT}"

log_step "validating Windows -> VM RealSense link"
bash "${SCRIPT_DIR}/test_realsense_stream.sh" \
  "${DOMAIN_ID}" \
  "${TOPIC_TIMEOUT_SEC}" \
  "${HZ_SAMPLE_SEC}" \
  "${MIN_COLOR_HZ}" \
  "${MIN_DEPTH_HZ}"
log_ok "cross-machine camera link is healthy"

log_step "loading VM ROS2 environment"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"
# refresh daemon cache to reduce stale graph false-negative
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

if is_true "${ARM_REQUIRED}"; then
  if ! VM_ARM_SERIAL_PORT="$(resolve_vm_arm_serial_port)"; then
    log_fail "no STM32 serial device detected on VM"
    print_vm_arm_serial_diagnostics
    exit 1
  fi
  log_ok "VM STM32 serial detected: ${VM_ARM_SERIAL_PORT}"
  configure_vm_arm_serial_port "${VM_ARM_SERIAL_PORT}"
fi

if [[ "${USE_RELAY_TOPICS}" == "true" ]] && ! has_ros_package "tactile_vision_cpp"; then
  USE_RELAY_TOPICS="false"
  ENABLE_REALSENSE_MONITOR="false"
  VISION_COLOR_TOPIC="/camera/camera/color/image_raw"
  VISION_DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
  VISION_CAMERA_INFO_TOPIC="/camera/camera/color/camera_info"
  echo "[WARN] package tactile_vision_cpp not found on VM; disabling latest_frame_relay_node and monitor."
fi

if [[ "${USE_RELAY_TOPICS}" == "false" ]]; then
  VISION_COLOR_TOPIC="/camera/camera/color/image_raw"
  VISION_DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
  VISION_CAMERA_INFO_TOPIC="/camera/camera/color/camera_info"

  if [[ "${ENABLE_REALSENSE_MONITOR}" == "true" ]]; then
    ensure_param_file_copy
    sed -i \
      -e "s#/camera/relay/color/image_raw#${VISION_COLOR_TOPIC}#g" \
      -e "s#/camera/relay/aligned_depth_to_color/image_raw#${VISION_DEPTH_TOPIC}#g" \
      -e "s#/camera/relay/color/camera_info#${VISION_CAMERA_INFO_TOPIC}#g" \
      "${PARAM_FILE_PATH}"
  fi
fi

if [[ "${USE_RELAY_TOPICS}" == "true" ]]; then
  log_ok "latest_frame_relay_node enabled"
else
  echo "[WARN] latest_frame_relay_node disabled; UI uses raw camera topics."
fi

if [[ "${ENABLE_REALSENSE_MONITOR}" == "true" ]]; then
  log_ok "realsense_monitor_node enabled"
else
  echo "[WARN] realsense_monitor_node disabled to reduce ROS2 load."
fi

echo "[INFO] vision profile=${VISION_PROFILE} relay=${USE_RELAY_TOPICS} monitor=${ENABLE_REALSENSE_MONITOR}"
echo "[INFO] UI vision limits: max_fps=${VISION_UI_MAX_FPS} stale_timeout_sec=${VISION_UI_STALE_TIMEOUT_SEC} qos=${VISION_QOS_MODE}"

log_step "cleaning stale VM ROS2 processes"
pkill -f "split_vm_app.launch.py" >/dev/null 2>&1 || true
pkill -f "latest_frame_relay_node" >/dev/null 2>&1 || true
pkill -f "realsense_monitor_node" >/dev/null 2>&1 || true
pkill -f "arm_driver_node" >/dev/null 2>&1 || true
sleep 1

mkdir -p "${LAUNCH_LOG_DIR}"
log_step "starting split_vm_app.launch.py in background"
launch_args=("start_tactile_sensor:=${START_TACTILE_SENSOR}")
if is_true "${ARM_REQUIRED}"; then
  launch_args+=("start_arm_driver:=true")
else
  launch_args+=("start_arm_driver:=false")
fi
if [[ "${USE_RELAY_TOPICS}" == "true" ]]; then
  launch_args+=("start_latest_frame_relay:=true")
else
  launch_args+=("start_latest_frame_relay:=false")
fi
if [[ "${ENABLE_REALSENSE_MONITOR}" == "true" ]]; then
  launch_args+=("start_realsense_monitor:=true")
else
  launch_args+=("start_realsense_monitor:=false")
fi
if [[ -n "${PARAM_FILE_PATH}" ]]; then
  launch_args+=("param_file:=${PARAM_FILE_PATH}")
fi
ros2 launch tactile_bringup split_vm_app.launch.py "${launch_args[@]}" >"${LAUNCH_LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"
sleep 4
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  log_fail "split_vm_app.launch.py exited early. Check log: ${LAUNCH_LOG_FILE}"
  tail -n 80 "${LAUNCH_LOG_FILE}" >&2 || true
  exit 1
fi
log_ok "split_vm_app.launch.py running (pid=${LAUNCH_PID})"
echo "[INFO] launch log: ${LAUNCH_LOG_FILE}"

log_step "validating VM-side core nodes"
core_nodes=("/arm_control_node" "/demo_task_node" "/tactile_ui_subscriber")
if is_true "${ARM_REQUIRED}"; then
  core_nodes=("/arm_driver_node" "${core_nodes[@]}")
fi
if [[ "${USE_RELAY_TOPICS}" == "true" ]]; then
  core_nodes+=("/latest_frame_relay_node")
fi
if [[ "${ENABLE_REALSENSE_MONITOR}" == "true" ]]; then
  core_nodes+=("/realsense_monitor_node")
fi
for node in "${core_nodes[@]}"; do
  if ! wait_for_node "${node}" 15; then
    log_fail "required VM node missing: ${node}"
    log_fail "Check launch log: ${LAUNCH_LOG_FILE}"
    exit 1
  fi
done
if is_true "${START_TACTILE_SENSOR}"; then
  if ! wait_for_node "/tactile_sensor_node" 15; then
    log_fail "tactile_sensor_node was requested but not running."
    log_fail "Check launch log: ${LAUNCH_LOG_FILE}"
    exit 1
  fi
fi
log_ok "VM core nodes are online"

if [[ "${USE_RELAY_TOPICS}" == "true" ]]; then
  log_step "validating relay camera topics for UI path"
else
  log_step "validating raw camera topics for UI path (relay disabled)"
fi
for topic in "${VISION_COLOR_TOPIC}" "${VISION_DEPTH_TOPIC}" "${VISION_CAMERA_INFO_TOPIC}"; do
  if ! wait_for_topic "${topic}" 15; then
    log_fail "required vision topic missing: ${topic}"
    if [[ "${USE_RELAY_TOPICS}" == "true" ]]; then
      log_fail "Check latest_frame_relay_node and split_vm_app.yaml topic mapping."
    else
      log_fail "Check Windows camera publisher and DDS connectivity."
    fi
    dump_runtime_diagnostics
    exit 1
  fi
done
log_ok "vision topics for UI path are online"

if is_true "${START_TACTILE_SENSOR}"; then
  log_step "validating tactile simulation stream"
  if ! wait_for_topic "/tactile/raw" 20; then
    log_fail "topic /tactile/raw is missing."
    log_fail "Check tactile sensor config/runtime in split_vm_app.launch.py"
    dump_runtime_diagnostics
    exit 1
  fi
  tactile_hz="$(sample_hz "/tactile/raw" 8 || true)"
  if [[ -z "${tactile_hz}" ]]; then
    tactile_hz="$(sample_hz "tactile/raw" 8 || true)"
  fi
  if [[ -z "${tactile_hz}" ]]; then
    sleep 2
    tactile_hz="$(sample_hz "/tactile/raw" 8 || true)"
  fi
  if [[ -z "${tactile_hz}" ]]; then
    sleep 2
    tactile_hz="$(sample_hz "/tactile/raw" 8 || true)"
  fi
  if [[ -z "${tactile_hz}" ]]; then
    log_fail "unable to measure tactile hz on /tactile/raw"
    dump_runtime_diagnostics
    exit 1
  fi
  if ! check_rate_ge "${tactile_hz}" "${MIN_TACTILE_HZ}"; then
    log_fail "tactile hz ${tactile_hz} < min ${MIN_TACTILE_HZ}"
    dump_runtime_diagnostics
    exit 1
  fi
  log_ok "tactile stream is healthy: ${tactile_hz}Hz"
fi

if is_true "${ARM_REQUIRED}"; then
  log_step "validating VM STM32/arm chain"
  if ! wait_for_topic "/arm/state" "${ARM_TIMEOUT_SEC}"; then
    log_fail "topic /arm/state not available within ${ARM_TIMEOUT_SEC}s"
    log_fail "VM arm_driver_node did not publish arm state. Check STM32 passthrough to the VM."
    print_vm_arm_serial_diagnostics
    exit 1
  fi
  for svc in "/arm/enable" "/arm/home" "/arm/move_joint" "/arm/move_joints" "/control/arm/enable" "/control/arm/move_joint"; do
    if ! wait_for_service "${svc}" "${ARM_TIMEOUT_SEC}"; then
      log_fail "required arm service missing: ${svc}"
      log_fail "Check VM arm_driver_node and arm_control_node."
      print_vm_arm_serial_diagnostics
      exit 1
    fi
  done

  log_step "enabling STM32/arm bridge before launching UI"
  arm_enable_output="$(call_setbool_service "/control/arm/enable" "true" "${ARM_TIMEOUT_SEC}" || true)"
  if ! echo "${arm_enable_output}" | tr '[:upper:]' '[:lower:]' | grep -Eq 'success[[:space:]]*[:=][[:space:]]*true'; then
    log_fail "failed to enable STM32/arm bridge via /control/arm/enable"
    if [[ -n "${arm_enable_output}" ]]; then
      echo "${arm_enable_output}" >&2
    fi
    if [[ -n "${VM_ARM_SERIAL_PORT}" ]]; then
      log_fail "current VM arm_serial_port: ${VM_ARM_SERIAL_PORT}"
    fi
    print_vm_arm_serial_diagnostics
    exit 1
  fi

  if ! wait_for_arm_connected_state "${ARM_TIMEOUT_SEC}"; then
    log_fail "/arm/state is online but connected=true was not observed within ${ARM_TIMEOUT_SEC}s"
    if [[ -n "${VM_ARM_SERIAL_PORT}" ]]; then
      log_fail "current VM arm_serial_port: ${VM_ARM_SERIAL_PORT}"
    fi
    print_vm_arm_serial_diagnostics
    echo "[DIAG] latest /arm/state sample:" >&2
    timeout 3s ros2 topic echo /arm/state --once >&2 || true
    exit 1
  fi

  log_ok "VM STM32/arm bridge is enabled and connected"
else
  echo "[WARN] ARM_REQUIRED=false, skipping arm chain guard."
fi

log_step "starting UI in ROS2 mode"
if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV}"
  log_ok "conda env activated: ${CONDA_ENV}"
else
  echo "[WARN] conda init script not found, using current python environment."
fi

# Conda activation can overwrite ROS-related env; source again.
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"

echo "[READY] all guards passed, launching UI..."
python "${PROJECT_ROOT}/main_ros2.py" \
  --control-mode ros2 \
  --vision-color-topic "${VISION_COLOR_TOPIC}" \
  --vision-depth-topic "${VISION_DEPTH_TOPIC}" \
  --vision-camera-info-topic "${VISION_CAMERA_INFO_TOPIC}" \
  --vision-max-fps "${VISION_UI_MAX_FPS}" \
  --vision-stale-timeout-sec "${VISION_UI_STALE_TIMEOUT_SEC}" \
  --vision-qos-mode "${VISION_QOS_MODE}" \
  --log-level INFO
