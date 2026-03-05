#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
TOPIC_TIMEOUT_SEC="${2:-20}"
HZ_SAMPLE_SEC="${3:-12}"
MIN_COLOR_HZ="${4:-3.0}"
MIN_DEPTH_HZ="${5:-3.0}"
HZ_RETRY_COUNT="${6:-3}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"

COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC="/camera/camera/color/camera_info"

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    if ros2 topic list | grep -Fxq "${topic}"; then
      echo "[OK] topic discovered: ${topic}"
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  echo "[FAIL] topic not discovered within ${timeout_sec}s: ${topic}" >&2
  return 1
}

wait_for_message() {
  local topic="$1"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    if timeout 2s ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

sample_hz_once() {
  local topic="$1"
  local sample_sec="$2"
  local output
  local rc
  set +e
  output="$(timeout "${sample_sec}s" ros2 topic hz "${topic}" 2>&1)"
  rc=$?
  set -e
  if [[ ${rc} -ne 0 && ${rc} -ne 124 ]]; then
    return 1
  fi
  echo "${output}" | awk '/average rate/ {print $3}' | tail -n 1
}

sample_hz_retry() {
  local topic="$1"
  local sample_sec="$2"
  local retry_count="$3"
  local attempt=1
  local rate=""
  while (( attempt <= retry_count )); do
    rate="$(sample_hz_once "${topic}" "${sample_sec}" || true)"
    if [[ -n "${rate}" ]]; then
      echo "${rate}"
      return 0
    fi
    echo "[WARN] attempt ${attempt}/${retry_count}: no average rate captured for ${topic}" >&2
    sleep 2
    attempt=$((attempt + 1))
  done
  return 1
}

check_rate() {
  local rate="$1"
  local minimum="$2"
  awk -v r="${rate}" -v m="${minimum}" 'BEGIN { exit ((r + 0.0 >= m + 0.0) ? 0 : 1) }'
}

echo "[INFO] checking RealSense topics from VM..."
wait_for_topic "${COLOR_TOPIC}" "${TOPIC_TIMEOUT_SEC}"
wait_for_topic "${DEPTH_TOPIC}" "${TOPIC_TIMEOUT_SEC}"
wait_for_topic "${INFO_TOPIC}" "${TOPIC_TIMEOUT_SEC}"

echo "[INFO] waiting for first camera messages..."
if ! wait_for_message "${COLOR_TOPIC}" "${TOPIC_TIMEOUT_SEC}"; then
  echo "[FAIL] color topic exists but no message arrived within ${TOPIC_TIMEOUT_SEC}s: ${COLOR_TOPIC}" >&2
  exit 1
fi
if ! wait_for_message "${DEPTH_TOPIC}" "${TOPIC_TIMEOUT_SEC}"; then
  echo "[FAIL] depth topic exists but no message arrived within ${TOPIC_TIMEOUT_SEC}s: ${DEPTH_TOPIC}" >&2
  exit 1
fi
if ! wait_for_message "${INFO_TOPIC}" "${TOPIC_TIMEOUT_SEC}"; then
  echo "[FAIL] camera_info topic exists but no message arrived within ${TOPIC_TIMEOUT_SEC}s: ${INFO_TOPIC}" >&2
  exit 1
fi
echo "[OK] first camera messages received."

echo "[INFO] sampling color topic hz (${HZ_SAMPLE_SEC}s)..."
color_rate="$(sample_hz_retry "${COLOR_TOPIC}" "${HZ_SAMPLE_SEC}" "${HZ_RETRY_COUNT}" || true)"
if [[ -z "${color_rate}" ]]; then
  echo "[FAIL] no average rate captured for ${COLOR_TOPIC} after ${HZ_RETRY_COUNT} attempts" >&2
  exit 1
fi
echo "[INFO] color average rate: ${color_rate} Hz"

echo "[INFO] sampling depth topic hz (${HZ_SAMPLE_SEC}s)..."
depth_rate="$(sample_hz_retry "${DEPTH_TOPIC}" "${HZ_SAMPLE_SEC}" "${HZ_RETRY_COUNT}" || true)"
if [[ -z "${depth_rate}" ]]; then
  echo "[FAIL] no average rate captured for ${DEPTH_TOPIC} after ${HZ_RETRY_COUNT} attempts" >&2
  exit 1
fi
echo "[INFO] depth average rate: ${depth_rate} Hz"

echo "[INFO] checking camera_info once..."
if ! timeout 8s ros2 topic echo "${INFO_TOPIC}" --once >/dev/null 2>&1; then
  echo "[FAIL] camera_info did not arrive within 8s" >&2
  exit 1
fi
echo "[OK] camera_info received."

if ! check_rate "${color_rate}" "${MIN_COLOR_HZ}"; then
  echo "[FAIL] color rate ${color_rate} < min ${MIN_COLOR_HZ}" >&2
  exit 1
fi
if ! check_rate "${depth_rate}" "${MIN_DEPTH_HZ}"; then
  echo "[FAIL] depth rate ${depth_rate} < min ${MIN_DEPTH_HZ}" >&2
  exit 1
fi

echo "[PASS] VM RealSense stream test passed."
echo "[PASS] color=${color_rate}Hz depth=${depth_rate}Hz (min color=${MIN_COLOR_HZ}, min depth=${MIN_DEPTH_HZ})"
