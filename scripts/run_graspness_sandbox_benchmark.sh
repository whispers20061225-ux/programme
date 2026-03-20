#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${GRASPNESS_REPO_ROOT:-/home/whispers/graspness_unofficial}"
ENV_NAME="${GRASPNESS_ENV_NAME:-graspness_env}"
CHECKPOINT_PATH="${GRASPNESS_CHECKPOINT_PATH:-${REPO_ROOT}/checkpoints/model_realsense.tar}"
INPUT_PATH="${GRASPNESS_INPUT_PATH:-/home/whispers/contact_graspnet/test_data/0.npy}"
MASK_ID="${GRASPNESS_MASK_ID:-1}"
GPU_INDEX="${GRASPNESS_GPU_INDEX:-0}"
WARMUP_RUNS="${GRASPNESS_WARMUP_RUNS:-1}"
BENCHMARK_RUNS="${GRASPNESS_BENCHMARK_RUNS:-3}"

log() {
  printf '[graspness-bench] %s\n' "$*"
}

find_conda_sh() {
  local candidate
  for candidate in \
    "/home/whispers/miniforge3/etc/profile.d/conda.sh" \
    "/home/whispers/mambaforge/etc/profile.d/conda.sh" \
    "/home/whispers/miniconda3/etc/profile.d/conda.sh" \
    "/opt/conda/etc/profile.d/conda.sh"
  do
    if [[ -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

CONDA_SH="$(find_conda_sh || true)"
if [[ -z "${CONDA_SH}" ]]; then
  log "conda.sh not found"
  exit 1
fi

# shellcheck disable=SC1090
source "${CONDA_SH}"
conda activate "${ENV_NAME}"

if [[ ! -f "${CHECKPOINT_PATH}" ]]; then
  log "checkpoint not found: ${CHECKPOINT_PATH}"
  exit 1
fi

if [[ ! -f "${INPUT_PATH}" ]]; then
  log "input sample not found: ${INPUT_PATH}"
  exit 1
fi

python /home/whispers/programme/scripts/benchmark_graspness_sandbox.py \
  --repo-root "${REPO_ROOT}" \
  --checkpoint-path "${CHECKPOINT_PATH}" \
  --input-path "${INPUT_PATH}" \
  --mask-id "${MASK_ID}" \
  --gpu-index "${GPU_INDEX}" \
  --warmup-runs "${WARMUP_RUNS}" \
  --benchmark-runs "${BENCHMARK_RUNS}"
