#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${GRASPNESS_REPO_URL:-https://github.com/graspnet/graspness_unofficial.git}"
REPO_ROOT="${GRASPNESS_REPO_ROOT:-/home/whispers/graspness_unofficial}"
GRASPNET_API_ROOT="${GRASPNET_API_ROOT:-/home/whispers/graspnetAPI}"
ENV_NAME="${GRASPNESS_ENV_NAME:-graspness_env}"
PYTHON_VERSION="${GRASPNESS_PYTHON_VERSION:-3.10}"
TORCH_SPEC="${GRASPNESS_TORCH_SPEC:-}"
TORCHVISION_SPEC="${GRASPNESS_TORCHVISION_SPEC:-}"
TORCHAUDIO_SPEC="${GRASPNESS_TORCHAUDIO_SPEC:-}"
EXTRA_INDEX_URL="${GRASPNESS_EXTRA_INDEX_URL:-}"
PIP_INDEX_URL="${GRASPNESS_PIP_INDEX_URL:-}"
MINKOWSKI_SPEC="${GRASPNESS_MINKOWSKI_SPEC:-MinkowskiEngine==0.5.4}"
SKIP_MINKOWSKI="${GRASPNESS_SKIP_MINKOWSKI:-0}"
CUDA_VERSION_SPEC="${GRASPNESS_CUDA_VERSION_SPEC:-12.1}"

log() {
  printf '[graspness-setup] %s\n' "$*"
}

warn() {
  printf '[graspness-setup][warn] %s\n' "$*"
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
  log "cloning graspness repo to ${REPO_ROOT}"
  git clone "${REPO_URL}" "${REPO_ROOT}"
else
  log "updating graspness repo in ${REPO_ROOT}"
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
export NVCC_PREPEND_FLAGS="${NVCC_PREPEND_FLAGS:-}"
export NVCC_APPEND_FLAGS="${NVCC_APPEND_FLAGS:-}"

if ! conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  log "creating conda env ${ENV_NAME} with python ${PYTHON_VERSION}"
  conda create -y -n "${ENV_NAME}" "python=${PYTHON_VERSION}"
else
  log "using existing conda env ${ENV_NAME}"
fi

conda activate "${ENV_NAME}"

log "upgrading base python tooling"
python -m pip install --upgrade pip setuptools wheel

if [[ -n "${TORCH_SPEC}" ]]; then
  log "installing explicit torch stack"
  pip_args=()
  if [[ -n "${PIP_INDEX_URL}" ]]; then
    pip_args+=(--index-url "${PIP_INDEX_URL}")
  fi
  if [[ -n "${EXTRA_INDEX_URL}" ]]; then
    pip_args+=(--extra-index-url "${EXTRA_INDEX_URL}")
  fi
  if [[ -n "${TORCH_SPEC}" ]]; then
    pip_args+=("${TORCH_SPEC}")
  fi
  if [[ -n "${TORCHVISION_SPEC}" ]]; then
    pip_args+=("${TORCHVISION_SPEC}")
  fi
  if [[ -n "${TORCHAUDIO_SPEC}" ]]; then
    pip_args+=("${TORCHAUDIO_SPEC}")
  fi
  python -m pip install "${pip_args[@]}"
else
  warn "torch stack not specified; keeping current env torch packages"
fi

log "installing env-local CUDA compiler and dev libraries"
conda install -y -c conda-forge -c nvidia \
  "cuda-version=${CUDA_VERSION_SPEC}" \
  "cuda-nvcc=${CUDA_VERSION_SPEC}" \
  "cuda-cudart-dev=${CUDA_VERSION_SPEC}" \
  "cuda-libraries-dev=${CUDA_VERSION_SPEC}" \
  ninja

export CUDA_HOME="${CONDA_PREFIX}"
export PATH="${CUDA_HOME}/bin:${PATH}"
export LD_LIBRARY_PATH="${CUDA_HOME}/lib:${CUDA_HOME}/targets/x86_64-linux/lib:${LD_LIBRARY_PATH:-}"
export TORCH_CUDA_ARCH_LIST="${GRASPNESS_TORCH_CUDA_ARCH_LIST:-8.9+PTX}"

log "installing repo requirements except torch and MinkowskiEngine"
python - <<EOF
from pathlib import Path
req_src = Path("${REPO_ROOT}") / "requirements.txt"
req_dst = Path("${REPO_ROOT}") / ".requirements_sandbox.filtered.txt"
lines = []
for raw in req_src.read_text().splitlines():
    entry = raw.strip()
    if not entry:
        continue
    lowered = entry.lower()
    if lowered.startswith("torch") or "minkowskiengine" in lowered:
        continue
    lines.append(raw)
req_dst.write_text("\\n".join(lines) + "\\n")
print(req_dst)
EOF
python -m pip install -r "${REPO_ROOT}/.requirements_sandbox.filtered.txt"

if [[ ! -d "${GRASPNET_API_ROOT}/.git" ]]; then
  log "cloning graspnetAPI to ${GRASPNET_API_ROOT}"
  git clone https://github.com/graspnet/graspnetAPI.git "${GRASPNET_API_ROOT}"
fi

log "installing graspnetAPI"
SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL=True python -m pip install "${GRASPNET_API_ROOT}"

if [[ "${SKIP_MINKOWSKI}" == "1" ]]; then
  warn "skipping MinkowskiEngine installation by request"
else
  log "installing ${MINKOWSKI_SPEC}"
  python -m pip install "${MINKOWSKI_SPEC}"
fi

log "building pointnet2"
(
  cd "${REPO_ROOT}/pointnet2"
  python setup.py install
)

log "building knn"
(
  cd "${REPO_ROOT}/knn"
  python setup.py install
)

cat <<EOF
[graspness-setup] done
[graspness-setup] repo: ${REPO_ROOT}
[graspness-setup] env:  ${ENV_NAME}
[graspness-setup] next:
  1. download a checkpoint into ${REPO_ROOT}/checkpoints
  2. run scripts/run_graspness_sandbox_benchmark.sh
EOF
