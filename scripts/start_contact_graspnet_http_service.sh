#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTACT_GRASPNET_ROOT="${CONTACT_GRASPNET_ROOT:-${HOME}/contact_graspnet}"
CONTACT_GRASPNET_CKPT_DIR="${CONTACT_GRASPNET_CKPT_DIR:-${CONTACT_GRASPNET_ROOT}/checkpoints/scene_test_2048_bs3_hor_sigma_001}"
CONTACT_GRASPNET_PYTHON="${CONTACT_GRASPNET_PYTHON:-}"
CONTACT_GRASPNET_CONDA_BIN="${CONTACT_GRASPNET_CONDA_BIN:-${HOME}/miniforge3/bin/conda}"
CONTACT_GRASPNET_CONDA_ENV="${CONTACT_GRASPNET_CONDA_ENV:-contact_graspnet_env}"
CONTACT_GRASPNET_HOST="${CONTACT_GRASPNET_HOST:-127.0.0.1}"
CONTACT_GRASPNET_PORT="${CONTACT_GRASPNET_PORT:-5001}"
CONTACT_GRASPNET_DEFAULT_VISUALIZE="${CONTACT_GRASPNET_DEFAULT_VISUALIZE:-0}"
CONTACT_GRASPNET_INFERENCE_TIMEOUT_SEC="${CONTACT_GRASPNET_INFERENCE_TIMEOUT_SEC:-10}"
CONTACT_GRASPNET_REQUEST_LOG_DIR="${CONTACT_GRASPNET_REQUEST_LOG_DIR:-${HOME}/modular_grasp_logs/contact_graspnet_requests}"

extra_args=()
if [[ "${CONTACT_GRASPNET_DEFAULT_VISUALIZE}" != "0" ]]; then
  extra_args+=(--default-visualize)
fi

python_cmd=()
if [[ -n "${CONTACT_GRASPNET_PYTHON}" ]]; then
  python_cmd=("${CONTACT_GRASPNET_PYTHON}")
elif [[ -x "${CONTACT_GRASPNET_CONDA_BIN}" ]]; then
  python_cmd=("${CONTACT_GRASPNET_CONDA_BIN}" run --no-capture-output -n "${CONTACT_GRASPNET_CONDA_ENV}" python)
else
  python_cmd=(python)
fi

exec "${python_cmd[@]}" "${REPO_ROOT}/scripts/contact_graspnet_http_service.py" \
  --host "${CONTACT_GRASPNET_HOST}" \
  --port "${CONTACT_GRASPNET_PORT}" \
  --contact-graspnet-root "${CONTACT_GRASPNET_ROOT}" \
  --ckpt-dir "${CONTACT_GRASPNET_CKPT_DIR}" \
  --inference-timeout-sec "${CONTACT_GRASPNET_INFERENCE_TIMEOUT_SEC}" \
  --log-dir "${CONTACT_GRASPNET_REQUEST_LOG_DIR}" \
  "${extra_args[@]}"
