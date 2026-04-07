#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${REPO_ROOT}/ros2_ws"
LOG_DIR="${MODULAR_GRASP_LOG_DIR:-${HOME}/modular_grasp_logs}"

QWEN_HOST="${QWEN_HOST:-127.0.0.1}"
QWEN_PORT="${QWEN_PORT:-8000}"
CONTACT_GRASPNET_HOST="${CONTACT_GRASPNET_HOST:-127.0.0.1}"
CONTACT_GRASPNET_PORT="${CONTACT_GRASPNET_PORT:-5001}"
CONTACT_GRASPNET_READY_TIMEOUT_SEC="${CONTACT_GRASPNET_READY_TIMEOUT_SEC:-180}"
CONTACT_GRASPNET_READY_POLL_INTERVAL_SEC="${CONTACT_GRASPNET_READY_POLL_INTERVAL_SEC:-2}"
CONTACT_GRASPNET_LOG_PATH="${CONTACT_GRASPNET_LOG_PATH:-${LOG_DIR}/contact_graspnet_http_service.log}"
CONTACT_GRASPNET_REQUEST_LOG_DIR="${CONTACT_GRASPNET_REQUEST_LOG_DIR:-${LOG_DIR}/contact_graspnet_requests}"
MODULAR_GRASP_SHADOW_ONLY="${MODULAR_GRASP_SHADOW_ONLY:-0}"
MODULAR_GRASP_SHADOW_IMAGE_VIEW="${MODULAR_GRASP_SHADOW_IMAGE_VIEW:-0}"
MODULAR_GRASP_SKIP_QWEN_START="${MODULAR_GRASP_SKIP_QWEN_START:-0}"

mkdir -p "${LOG_DIR}"
mkdir -p "${CONTACT_GRASPNET_REQUEST_LOG_DIR}"

find_contact_graspnet_pid() {
  ps -ef | grep -F "contact_graspnet_http_service.py --host ${CONTACT_GRASPNET_HOST} --port ${CONTACT_GRASPNET_PORT}" | grep -v grep | awk 'NR==1 {print $2}'
}

recycle_stale_contact_graspnet() {
  local pid_list=""
  pid_list="$(
    ps -ef | grep -F "contact_graspnet_http_service.py --host ${CONTACT_GRASPNET_HOST} --port ${CONTACT_GRASPNET_PORT}" | grep -v grep | awk '{print $2}' | tr '\n' ' ' || true
  )"
  if [[ -n "${pid_list// }" ]]; then
    echo "[start] Recycling stale Contact-GraspNet sidecar: ${pid_list}"
    kill ${pid_list} 2>/dev/null || true
    sleep 2
  fi
}

contact_graspnet_ready() {
  local status_code
  status_code="$(
    curl -s -o /dev/null -w '%{http_code}' \
      "http://${CONTACT_GRASPNET_HOST}:${CONTACT_GRASPNET_PORT}/health" || true
  )"
  [[ "${status_code}" == "200" ]]
}

wait_for_contact_graspnet_ready() {
  local elapsed=0
  while (( elapsed < CONTACT_GRASPNET_READY_TIMEOUT_SEC )); do
    if contact_graspnet_ready; then
      return 0
    fi
    sleep "${CONTACT_GRASPNET_READY_POLL_INTERVAL_SEC}"
    elapsed=$((elapsed + CONTACT_GRASPNET_READY_POLL_INTERVAL_SEC))
  done
  return 1
}

ensure_contact_graspnet_service() {
  if contact_graspnet_ready; then
    local pid
    pid="$(find_contact_graspnet_pid || true)"
    echo "[start] Contact-GraspNet sidecar is already ready on http://${CONTACT_GRASPNET_HOST}:${CONTACT_GRASPNET_PORT}"
    if [[ -n "${pid}" ]]; then
      echo "[start] Contact-GraspNet PID: ${pid}"
    fi
    echo "[start] Contact-GraspNet log: ${CONTACT_GRASPNET_LOG_PATH}"
    echo "[start] Contact-GraspNet request logs: ${CONTACT_GRASPNET_REQUEST_LOG_DIR}"
    return 0
  fi

  if ss -ltn | grep -q ":${CONTACT_GRASPNET_PORT} "; then
    recycle_stale_contact_graspnet
  fi

  if ss -ltn | grep -q ":${CONTACT_GRASPNET_PORT} "; then
    echo "[start] Port ${CONTACT_GRASPNET_PORT} is already occupied and Contact-GraspNet is not ready." >&2
    return 1
  fi

  echo "[start] Starting Contact-GraspNet sidecar on http://${CONTACT_GRASPNET_HOST}:${CONTACT_GRASPNET_PORT}"
  setsid -f /usr/bin/env \
    PYTHONUNBUFFERED=1 \
    TF_CPP_MIN_LOG_LEVEL="${TF_CPP_MIN_LOG_LEVEL:-2}" \
    CONTACT_GRASPNET_REQUEST_LOG_DIR="${CONTACT_GRASPNET_REQUEST_LOG_DIR}" \
    stdbuf -oL -eL \
    "${REPO_ROOT}/scripts/start_contact_graspnet_http_service.sh" \
    > "${CONTACT_GRASPNET_LOG_PATH}" 2>&1

  if ! wait_for_contact_graspnet_ready; then
    echo "[start] Contact-GraspNet sidecar did not become ready within ${CONTACT_GRASPNET_READY_TIMEOUT_SEC}s." >&2
    echo "[start] Last Contact-GraspNet log lines:" >&2
    tail -n 60 "${CONTACT_GRASPNET_LOG_PATH}" >&2 || true
    return 1
  fi

  local pid
  pid="$(find_contact_graspnet_pid || true)"
  echo "[start] Contact-GraspNet sidecar ready on http://${CONTACT_GRASPNET_HOST}:${CONTACT_GRASPNET_PORT}"
  if [[ -n "${pid}" ]]; then
    echo "[start] Contact-GraspNet PID: ${pid}"
  fi
  echo "[start] Contact-GraspNet log: ${CONTACT_GRASPNET_LOG_PATH}"
  echo "[start] Contact-GraspNet request logs: ${CONTACT_GRASPNET_REQUEST_LOG_DIR}"
}

if [[ "${MODULAR_GRASP_SKIP_QWEN_START}" == "1" ]]; then
  echo "[start] Skipping local Qwen vLLM startup"
  echo "[start] Expectation: a compatible model endpoint is already available, or semantic tasks will be published manually"
else
  echo "[start] Ensuring Qwen semantic service is ready on http://${QWEN_HOST}:${QWEN_PORT}"
  "${REPO_ROOT}/scripts/start_qwen_vllm.sh"
fi

launch_args=("$@")
if [[ "${MODULAR_GRASP_SHADOW_ONLY}" == "1" ]]; then
  echo "[start] Shadow-only mode enabled: skipping Contact-GraspNet sidecar and formal pick execution"
  launch_args+=("shadow_only_mode:=true")
  if [[ "${MODULAR_GRASP_SHADOW_IMAGE_VIEW}" == "1" ]]; then
    launch_args+=("start_ggcnn_shadow_image_view:=true")
  else
    launch_args+=("start_ggcnn_shadow_image_view:=false")
  fi
else
  ensure_contact_graspnet_service
fi

echo "[start] Cleaning stale Gazebo/MoveIt/ROS processes"
bash "${ROS_WS}/scripts/cleanup_sim_stack.sh"

cd "${ROS_WS}"
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

echo "[start] Launching modular grasp demo"
echo "[start] Gazebo GUI and RViz are enabled by default"
exec ros2 launch tactile_bringup phase8_modular_grasp.launch.py "${launch_args[@]}"
