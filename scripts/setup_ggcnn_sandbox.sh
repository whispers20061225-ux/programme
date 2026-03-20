#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${GGCNN_REPO_URL:-https://github.com/dougsm/ggcnn.git}"
REPO_ROOT="${GGCNN_REPO_ROOT:-/home/whispers/ggcnn}"
ENV_NAME="${GGCNN_ENV_NAME:-ggcnn_env}"
PYTHON_VERSION="${GGCNN_PYTHON_VERSION:-3.10}"
TORCH_SPEC="${GGCNN_TORCH_SPEC:-torch==2.10.0}"
TORCHVISION_SPEC="${GGCNN_TORCHVISION_SPEC:-torchvision==0.25.0}"
TORCHAUDIO_SPEC="${GGCNN_TORCHAUDIO_SPEC:-}"
MODEL_VARIANT="${GGCNN_MODEL_VARIANT:-ggcnn2}"
WEIGHTS_DATASET="${GGCNN_WEIGHTS_DATASET:-cornell}"
PIP_INDEX_URL="${GGCNN_PIP_INDEX_URL:-}"
EXTRA_INDEX_URL="${GGCNN_EXTRA_INDEX_URL:-}"
WEIGHTS_ROOT="${GGCNN_WEIGHTS_ROOT:-/home/whispers/ggcnn_weights}"

log() {
  printf '[ggcnn-setup] %s\n' "$*"
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

if [[ ! -d "${REPO_ROOT}/.git" ]]; then
  log "cloning ggcnn repo to ${REPO_ROOT}"
  git clone "${REPO_URL}" "${REPO_ROOT}"
else
  log "updating ggcnn repo in ${REPO_ROOT}"
  git -C "${REPO_ROOT}" fetch --all --tags
  git -C "${REPO_ROOT}" pull --ff-only
fi

CONDA_SH="$(find_conda_sh || true)"
if [[ -z "${CONDA_SH}" ]]; then
  log "conda.sh not found. Install miniforge/miniconda first."
  exit 1
fi

# shellcheck disable=SC1090
source "${CONDA_SH}"

if ! conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  log "creating conda env ${ENV_NAME} with python ${PYTHON_VERSION}"
  conda create -y -n "${ENV_NAME}" "python=${PYTHON_VERSION}"
else
  log "using existing conda env ${ENV_NAME}"
fi

conda activate "${ENV_NAME}"

log "upgrading base python tooling"
python -m pip install --upgrade pip setuptools wheel

log "installing torch stack"
pip_args=()
if [[ -n "${PIP_INDEX_URL}" ]]; then
  pip_args+=(--index-url "${PIP_INDEX_URL}")
fi
if [[ -n "${EXTRA_INDEX_URL}" ]]; then
  pip_args+=(--extra-index-url "${EXTRA_INDEX_URL}")
fi
torch_pkgs=("${TORCH_SPEC}" "${TORCHVISION_SPEC}")
if [[ -n "${TORCHAUDIO_SPEC}" ]]; then
  torch_pkgs+=("${TORCHAUDIO_SPEC}")
fi
python -m pip install "${torch_pkgs[@]}" "${pip_args[@]}"

log "installing ggcnn requirements"
python -m pip install -r "${REPO_ROOT}/requirements.txt"

mkdir -p "${WEIGHTS_ROOT}"
weights_zip="${WEIGHTS_ROOT}/${MODEL_VARIANT}_weights_${WEIGHTS_DATASET}.zip"
weights_dir="${WEIGHTS_ROOT}/${MODEL_VARIANT}_weights_${WEIGHTS_DATASET}"
weights_url="https://github.com/dougsm/ggcnn/releases/download/v0.1/${MODEL_VARIANT}_weights_${WEIGHTS_DATASET}.zip"

if [[ ! -f "${weights_zip}" ]]; then
  log "downloading weights: ${weights_url}"
  wget -O "${weights_zip}" "${weights_url}"
else
  log "weights archive already exists: ${weights_zip}"
fi

if [[ ! -d "${weights_dir}" ]]; then
  log "unpacking weights to ${weights_dir}"
  unzip -o "${weights_zip}" -d "${WEIGHTS_ROOT}"
else
  log "weights directory already exists: ${weights_dir}"
fi

cat <<EOF
[ggcnn-setup] done
[ggcnn-setup] repo: ${REPO_ROOT}
[ggcnn-setup] env: ${ENV_NAME}
[ggcnn-setup] weights: ${weights_dir}
[ggcnn-setup] next:
  1. run scripts/run_ggcnn_sandbox_benchmark.sh
EOF
