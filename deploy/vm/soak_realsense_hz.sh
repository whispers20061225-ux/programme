#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
DURATION_MIN="${2:-15}"
INTERVAL_SEC="${3:-60}"
SAMPLE_SEC="${4:-8}"
TOPIC_TIMEOUT_SEC="${5:-30}"
OUTPUT_CSV="${6:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"

COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

sample_hz_once() {
  local topic="$1"
  local window="$2"
  local output
  local rc
  set +e
  output="$(timeout "${window}s" ros2 topic hz "${topic}" 2>&1)"
  rc=$?
  set -e
  if [[ ${rc} -ne 0 && ${rc} -ne 124 ]]; then
    echo ""
    return 1
  fi
  echo "${output}" | awk '/average rate/ {print $3}' | tail -n 1
}

if ! wait_for_topic "${COLOR_TOPIC}" "${TOPIC_TIMEOUT_SEC}"; then
  echo "[FAIL] color topic not found: ${COLOR_TOPIC}" >&2
  exit 1
fi
if ! wait_for_topic "${DEPTH_TOPIC}" "${TOPIC_TIMEOUT_SEC}"; then
  echo "[FAIL] depth topic not found: ${DEPTH_TOPIC}" >&2
  exit 1
fi
echo "[OK] topics discovered"

LOG_DIR="${PROJECT_ROOT}/ros2_ws/log/soak"
mkdir -p "${LOG_DIR}"
if [[ -z "${OUTPUT_CSV}" ]]; then
  OUTPUT_CSV="${LOG_DIR}/vm_soak_$(date +%Y%m%d_%H%M%S).csv"
fi

echo "sample_index,timestamp_iso,timestamp_epoch,color_hz,depth_hz,status" > "${OUTPUT_CSV}"

duration_sec=$(( DURATION_MIN * 60 ))
end_epoch=$(( $(date +%s) + duration_sec ))
sample_index=0

echo "[STEP] starting soak test (duration=${DURATION_MIN}min, interval=${INTERVAL_SEC}s, sample=${SAMPLE_SEC}s)"
while (( $(date +%s) < end_epoch )); do
  sample_index=$((sample_index + 1))
  stamp_epoch="$(date +%s)"
  stamp_iso="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"

  color_hz="$(sample_hz_once "${COLOR_TOPIC}" "${SAMPLE_SEC}" || true)"
  depth_hz="$(sample_hz_once "${DEPTH_TOPIC}" "${SAMPLE_SEC}" || true)"

  status="ok"
  if [[ -z "${color_hz}" || -z "${depth_hz}" ]]; then
    status="no_rate"
    color_hz="${color_hz:-0.0}"
    depth_hz="${depth_hz:-0.0}"
    echo "[WARN] [SAMPLE][vm] idx=${sample_index} no average rate captured"
  else
    echo "[SAMPLE][vm] idx=${sample_index} color=${color_hz} depth=${depth_hz}"
  fi

  echo "${sample_index},${stamp_iso},${stamp_epoch},${color_hz},${depth_hz},${status}" >> "${OUTPUT_CSV}"

  if (( $(date +%s) >= end_epoch )); then
    break
  fi
  sleep "${INTERVAL_SEC}"
done

echo "[OK] csv exported: ${OUTPUT_CSV}"

awk -F',' '
  NR==1 {next}
  {
    c=$4+0; d=$5+0;
    if (count==0 || c<cmin) cmin=c;
    if (count==0 || c>cmax) cmax=c;
    if (count==0 || d<dmin) dmin=d;
    if (count==0 || d>dmax) dmax=d;
    csum+=c; dsum+=d; count++;
    if (c<1.0) czero++;
    if (d<1.0) dzero++;
  }
  END {
    if (count==0) {
      print "[FAIL] no samples captured";
      exit 1;
    }
    printf("[SUMMARY][vm] samples=%d color(avg/min/max)=%.3f/%.3f/%.3f depth(avg/min/max)=%.3f/%.3f/%.3f\n",
      count, csum/count, cmin, cmax, dsum/count, dmin, dmax);
    printf("[SUMMARY][vm] color<1Hz=%d/%d (%.1f%%) depth<1Hz=%d/%d (%.1f%%)\n",
      czero, count, (czero*100.0)/count, dzero, count, (dzero*100.0)/count);
  }
' "${OUTPUT_CSV}"
