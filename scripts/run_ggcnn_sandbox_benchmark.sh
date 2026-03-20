#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${GGCNN_REPO_ROOT:-/home/whispers/ggcnn}"
ENV_NAME="${GGCNN_ENV_NAME:-ggcnn_env}"
INPUT_PATH="${GGCNN_INPUT_PATH:-/home/whispers/contact_graspnet/test_data/0.npy}"
GPU_INDEX="${GGCNN_GPU_INDEX:-0}"
MODEL_VARIANT="${GGCNN_MODEL_VARIANT:-ggcnn2}"
WEIGHTS_DATASET="${GGCNN_WEIGHTS_DATASET:-cornell}"
WEIGHTS_ROOT="${GGCNN_WEIGHTS_ROOT:-/home/whispers/ggcnn_weights}"
WARMUP_RUNS="${GGCNN_WARMUP_RUNS:-1}"
BENCHMARK_RUNS="${GGCNN_BENCHMARK_RUNS:-3}"
MASK_ID="${GGCNN_MASK_ID:-1}"
INPUT_SIZE="${GGCNN_INPUT_SIZE:-300}"
TOP_K="${GGCNN_TOP_K:-10}"

log() {
  printf '[ggcnn-bench] %s\n' "$*"
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

checkpoint_dir="${WEIGHTS_ROOT}/${MODEL_VARIANT}_weights_${WEIGHTS_DATASET}"
checkpoint_path="${GGCNN_CHECKPOINT_PATH:-}"

if [[ -z "${checkpoint_path}" ]]; then
  checkpoint_statedict="$(find "${checkpoint_dir}" -maxdepth 1 -type f -name '*statedict.pt' | sort | head -n 1 || true)"
  checkpoint_model="$(find "${checkpoint_dir}" -maxdepth 1 -type f ! -name '*.zip' ! -name '*statedict.pt' | sort | head -n 1 || true)"
  if [[ -n "${checkpoint_statedict}" && -f "${checkpoint_statedict}" ]]; then
    checkpoint_path="${checkpoint_statedict}"
  elif [[ -n "${checkpoint_model}" && -f "${checkpoint_model}" ]]; then
    checkpoint_path="${checkpoint_model}"
  else
    log "checkpoint not found under ${checkpoint_dir}"
    exit 1
  fi
fi

if [[ ! -f "${INPUT_PATH}" ]]; then
  log "input sample not found: ${INPUT_PATH}"
  exit 1
fi

python /home/whispers/programme/scripts/benchmark_ggcnn_sandbox.py \
  --repo-root "${REPO_ROOT}" \
  --checkpoint-path "${checkpoint_path}" \
  --input-path "${INPUT_PATH}" \
  --model "${MODEL_VARIANT}" \
  --mask-id "${MASK_ID}" \
  --input-size "${INPUT_SIZE}" \
  --top-k "${TOP_K}" \
  --gpu-index "${GPU_INDEX}" \
  --warmup-runs "${WARMUP_RUNS}" \
  --benchmark-runs "${BENCHMARK_RUNS}"
