#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
SAMPLE_SEC="${2:-12}"
START_EPOCH_SEC="${3:-0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"

COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"

wait_until_epoch() {
  local target="$1"
  if [[ "${target}" -le 0 ]]; then
    return 0
  fi
  while (( "$(date +%s)" < target )); do
    sleep 0.2
  done
}

sample_hz() {
  local topic="$1"
  local window="$2"
  local output
  local rc
  set +e
  output="$(timeout "${window}s" ros2 topic hz "${topic}" 2>&1)"
  rc=$?
  set -e
  if [[ ${rc} -ne 0 && ${rc} -ne 124 ]]; then
    return 1
  fi
  echo "${output}" | awk '/average rate/ {print $3}' | tail -n 1
}

wait_until_epoch "${START_EPOCH_SEC}"
start_epoch="$(date +%s)"

color_hz="$(sample_hz "${COLOR_TOPIC}" "${SAMPLE_SEC}" || true)"
depth_hz="$(sample_hz "${DEPTH_TOPIC}" "${SAMPLE_SEC}" || true)"
if [[ -z "${color_hz}" || -z "${depth_hz}" ]]; then
  echo "[FAIL] failed to sample hz (color=${color_hz:-n/a}, depth=${depth_hz:-n/a})" >&2
  exit 1
fi

end_epoch="$(date +%s)"
echo "[RESULT][vm] start=${start_epoch} end=${end_epoch} sample=${SAMPLE_SEC}s color_hz=${color_hz} depth_hz=${depth_hz}"
