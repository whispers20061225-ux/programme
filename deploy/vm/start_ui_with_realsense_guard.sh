#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
TOPIC_TIMEOUT_SEC="${2:-20}"
HZ_SAMPLE_SEC="${3:-12}"
MIN_COLOR_HZ="${4:-3.0}"
MIN_DEPTH_HZ="${5:-3.0}"
START_TACTILE_SENSOR="${6:-true}"
CONDA_ENV="${7:-dayiprogramme312}"
KEEP_LAUNCH="${KEEP_LAUNCH:-false}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LAUNCH_LOG_DIR="${PROJECT_ROOT}/ros2_ws/log"
LAUNCH_LOG_FILE="${LAUNCH_LOG_DIR}/split_vm_app_$(date +%Y%m%d_%H%M%S).log"

log_step() { echo "[STEP] $*"; }
log_ok() { echo "[OK] $*"; }
log_fail() { echo "[FAIL] $*" >&2; }

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

python "${PROJECT_ROOT}/main_ros2.py" --control-mode ros2 --log-level INFO

