#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
TOPIC_TIMEOUT_SEC="${2:-20}"
HZ_SAMPLE_SEC="${3:-12}"
MIN_COLOR_HZ="${4:-3.0}"
MIN_DEPTH_HZ="${5:-3.0}"

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

sample_hz() {
  local topic="$1"
  local sample_sec="$2"
  local output
  local rc
  set +e
  output="$(timeout "${sample_sec}s" ros2 topic hz "${topic}" 2>&1)"
  rc=$?
  set -e
  # timeout exits with 124, which is acceptable for sampling
  if [[ ${rc} -ne 0 && ${rc} -ne 124 ]]; then
    echo "[FAIL] ros2 topic hz failed for ${topic}" >&2
    echo "${output}" >&2
    return 1
  fi
  echo "${output}"
  local rate
  rate="$(echo "${output}" | awk '/average rate/ {print $3}' | tail -n 1)"
  if [[ -z "${rate}" ]]; then
    echo "[FAIL] no average rate captured for ${topic}" >&2
    return 1
  fi
  echo "${rate}"
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

echo "[INFO] sampling color topic hz (${HZ_SAMPLE_SEC}s)..."
color_report="$(sample_hz "${COLOR_TOPIC}" "${HZ_SAMPLE_SEC}")"
color_rate="$(echo "${color_report}" | tail -n 1)"
echo "${color_report}" | sed '$d'
echo "[INFO] color average rate: ${color_rate} Hz"

echo "[INFO] sampling depth topic hz (${HZ_SAMPLE_SEC}s)..."
depth_report="$(sample_hz "${DEPTH_TOPIC}" "${HZ_SAMPLE_SEC}")"
depth_rate="$(echo "${depth_report}" | tail -n 1)"
echo "${depth_report}" | sed '$d'
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

