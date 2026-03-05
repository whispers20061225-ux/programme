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
KEEP_LAUNCH="${KEEP_LAUNCH:-false}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LAUNCH_LOG_DIR="${PROJECT_ROOT}/ros2_ws/log"
LAUNCH_LOG_FILE="${LAUNCH_LOG_DIR}/split_vm_app_$(date +%Y%m%d_%H%M%S).log"

log_step() { echo "[STEP] $*"; }
log_ok() { echo "[OK] $*"; }
log_fail() { echo "[FAIL] $*" >&2; }
is_true() { [[ "${1,,}" == "true" || "${1}" == "1" || "${1,,}" == "yes" ]]; }

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    if ros2 topic list | grep -Fxq "${topic}"; then
      return 0
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
    if ros2 service list | grep -Fxq "${service}"; then
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
  local output
  local rc
  set +e
  output="$(timeout "${sample_sec}s" ros2 topic hz "${topic}" 2>&1)"
  rc=$?
  set -e
  if [[ ${rc} -ne 0 && ${rc} -ne 124 ]]; then
    echo ""
    return 1
  fi
  echo "${output}" | awk '/average rate/ {print $3}' | tail -n 1
}

check_rate_ge() {
  local actual="$1"
  local minimum="$2"
  awk -v r="${actual}" -v m="${minimum}" 'BEGIN { exit ((r + 0.0 >= m + 0.0) ? 0 : 1) }'
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

mkdir -p "${LAUNCH_LOG_DIR}"
log_step "starting split_vm_app.launch.py in background"
ros2 launch tactile_bringup split_vm_app.launch.py start_tactile_sensor:="${START_TACTILE_SENSOR}" >"${LAUNCH_LOG_FILE}" 2>&1 &
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
for node in "/arm_control_node" "/demo_task_node" "/tactile_ui_subscriber" "/realsense_monitor_node"; do
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

if is_true "${START_TACTILE_SENSOR}"; then
  log_step "validating tactile simulation stream"
  if ! wait_for_topic "/tactile/raw" 15; then
    log_fail "topic /tactile/raw is missing."
    log_fail "Check tactile sensor config in split_vm_app.yaml"
    exit 1
  fi
  tactile_hz="$(sample_hz "/tactile/raw" 8 || true)"
  if [[ -z "${tactile_hz}" ]]; then
    log_fail "unable to measure tactile hz on /tactile/raw"
    exit 1
  fi
  if ! check_rate_ge "${tactile_hz}" "${MIN_TACTILE_HZ}"; then
    log_fail "tactile hz ${tactile_hz} < min ${MIN_TACTILE_HZ}"
    exit 1
  fi
  log_ok "tactile stream is healthy: ${tactile_hz}Hz"
fi

if is_true "${ARM_REQUIRED}"; then
  log_step "validating arm hardware/control chain"
  if ! wait_for_topic "/arm/state" "${ARM_TIMEOUT_SEC}"; then
    log_fail "topic /arm/state not available within ${ARM_TIMEOUT_SEC}s"
    log_fail "Windows side likely started camera only. Start arm driver on Windows:"
    log_fail ".\\deploy\\windows\\start_hw_nodes.ps1 -RosSetup <ros_setup> -DomainId ${DOMAIN_ID} -StartArm:\$true -StartRealsense:\$true -Execute"
    exit 1
  fi
  for svc in "/arm/enable" "/arm/home" "/arm/move_joint" "/arm/move_joints" "/control/arm/enable" "/control/arm/move_joint"; do
    if ! wait_for_service "${svc}" "${ARM_TIMEOUT_SEC}"; then
      log_fail "required arm service missing: ${svc}"
      log_fail "Check Windows arm_driver_node and VM arm_control_node."
      exit 1
    fi
  done
  log_ok "arm chain is healthy"
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
python "${PROJECT_ROOT}/main_ros2.py" --control-mode ros2 --log-level INFO
