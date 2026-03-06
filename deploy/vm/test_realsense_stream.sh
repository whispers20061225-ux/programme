#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
TOPIC_TIMEOUT_SEC="${2:-20}"
HZ_SAMPLE_SEC="${3:-12}"
MIN_COLOR_HZ="${4:-3.0}"
MIN_DEPTH_HZ="${5:-3.0}"
HZ_RETRY_COUNT="${6:-3}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PROBE_SCRIPT="${SCRIPT_DIR}/realsense_stream_probe.py"
PYTHON_CMD=""
VM_ROS_PYTHON_ENV="${VM_ROS_PYTHON:-}"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"
# stop daemon before probe to free one DDS participant slot
ros2 daemon stop >/dev/null 2>&1 || true

COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC="/camera/camera/color/camera_info"

python_supports_ros_probe() {
  local candidate="$1"
  set +e
  "${candidate}" -c 'import json, yaml, rclpy' >/dev/null 2>&1
  local rc=$?
  set -e
  return ${rc}
}

get_python_cmd() {
  if [[ -n "${PYTHON_CMD}" ]]; then
    printf '%s\n' "${PYTHON_CMD}"
    return 0
  fi

  local candidates=()
  local candidate=""
  local resolved=""
  local diagnostics=()

  if [[ -n "${VM_ROS_PYTHON_ENV}" ]]; then
    candidates+=("${VM_ROS_PYTHON_ENV}")
  fi
  candidates+=("/usr/bin/python3" "python3" "python")

  for candidate in "${candidates[@]}"; do
    if [[ -z "${candidate}" ]]; then
      continue
    fi
    resolved="${candidate}"
    if [[ "${candidate}" != */* ]]; then
      resolved="$(command -v "${candidate}" 2>/dev/null || true)"
      if [[ -z "${resolved}" ]]; then
        diagnostics+=("${candidate}: not found")
        continue
      fi
    elif [[ ! -x "${candidate}" ]]; then
      diagnostics+=("${candidate}: not executable")
      continue
    fi

    if python_supports_ros_probe "${resolved}"; then
      PYTHON_CMD="${resolved}"
      printf '%s\n' "${PYTHON_CMD}"
      return 0
    fi
    diagnostics+=("${resolved}: missing python modules (need yaml + rclpy)")
  done

  echo "[FAIL] no VM python interpreter can import yaml and rclpy" >&2
  for candidate in "${diagnostics[@]}"; do
    echo "[DIAG][python] ${candidate}" >&2
  done
  return 1
}

json_probe_field() {
  local json_payload="$1"
  local field_path="$2"
  printf '%s' "${json_payload}" | "$(get_python_cmd)" -c '
import json
import sys

field_path = sys.argv[1].split(".")
data = json.load(sys.stdin)
value = data
for part in field_path:
    if isinstance(value, dict):
        value = value.get(part)
    else:
        value = None
        break
if value is None:
    raise SystemExit(1)
if isinstance(value, bool):
    print("true" if value else "false")
else:
    print(value)
' "${field_path}"
}

run_realsense_probe() {
  "$(get_python_cmd)" "${PROBE_SCRIPT}"     --color-topic "${COLOR_TOPIC}"     --depth-topic "${DEPTH_TOPIC}"     --info-topic "${INFO_TOPIC}"     --first-timeout-sec "$1"     --sample-sec "$2"
}

check_rate() {
  local rate="$1"
  local minimum="$2"
  awk -v r="${rate}" -v m="${minimum}" 'BEGIN { exit ((r + 0.0 >= m + 0.0) ? 0 : 1) }'
}

echo "[INFO] probing RealSense topics and sampling hz (${HZ_SAMPLE_SEC}s)..."
set +e
probe_output="$(run_realsense_probe "${TOPIC_TIMEOUT_SEC}" "${HZ_SAMPLE_SEC}" 2>&1)"
probe_rc=$?
set -e
if [[ ${probe_rc} -ne 0 ]]; then
  probe_reason="$(json_probe_field "${probe_output}" "reason" 2>/dev/null || echo "probe_failed")"
  echo "[FAIL] RealSense VM probe failed: ${probe_reason}" >&2
  echo "[DIAG][probe] ${probe_output}" >&2
  exit 1
fi

color_received="$(json_probe_field "${probe_output}" "streams.color.received" 2>/dev/null || echo "false")"
depth_received="$(json_probe_field "${probe_output}" "streams.depth.received" 2>/dev/null || echo "false")"
info_received="$(json_probe_field "${probe_output}" "streams.camera_info.received" 2>/dev/null || echo "false")"
if [[ "${color_received}" != "true" ]]; then
  echo "[FAIL] color topic exists but probe received no message: ${COLOR_TOPIC}" >&2
  echo "[DIAG][probe] ${probe_output}" >&2
  exit 1
fi
if [[ "${depth_received}" != "true" ]]; then
  echo "[FAIL] depth topic exists but probe received no message: ${DEPTH_TOPIC}" >&2
  echo "[DIAG][probe] ${probe_output}" >&2
  exit 1
fi
if [[ "${info_received}" != "true" ]]; then
  echo "[FAIL] camera_info topic exists but probe received no message: ${INFO_TOPIC}" >&2
  echo "[DIAG][probe] ${probe_output}" >&2
  exit 1
fi
echo "[OK] first camera messages received."

color_rate="$(json_probe_field "${probe_output}" "streams.color.sample_hz" 2>/dev/null || true)"
depth_rate="$(json_probe_field "${probe_output}" "streams.depth.sample_hz" 2>/dev/null || true)"
if [[ -z "${color_rate}" ]]; then
  echo "[FAIL] probe captured no color average rate for ${COLOR_TOPIC}" >&2
  echo "[DIAG][probe] ${probe_output}" >&2
  exit 1
fi
if [[ -z "${depth_rate}" ]]; then
  echo "[FAIL] probe captured no depth average rate for ${DEPTH_TOPIC}" >&2
  echo "[DIAG][probe] ${probe_output}" >&2
  exit 1
fi
echo "[INFO] color average rate: ${color_rate} Hz"
echo "[INFO] depth average rate: ${depth_rate} Hz"

echo "[INFO] checking camera_info once..."
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
