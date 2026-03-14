#!/usr/bin/env bash
set -euo pipefail

VENV_DIR="${QWEN_VENV_DIR:-$HOME/.venvs/qwen_vl}"
MODEL_NAME="${QWEN_MODEL_NAME:-Qwen/Qwen2.5-VL-3B-Instruct-AWQ}"
HOST="${QWEN_HOST:-127.0.0.1}"
PORT="${QWEN_PORT:-8000}"
MAX_MODEL_LEN="${QWEN_MAX_MODEL_LEN:-4096}"
GPU_MEMORY_UTILIZATION="${QWEN_GPU_MEMORY_UTILIZATION:-0.85}"
ALLOWED_MEDIA_PATH="${QWEN_ALLOWED_MEDIA_PATH:-$HOME}"
LOG_PATH="${QWEN_LOG_PATH:-$HOME/qwen_vllm_server.log}"
PID_FILE="${QWEN_PID_FILE:-$HOME/qwen_vllm_server.pid}"
VLLM_BIN="$VENV_DIR/bin/vllm"
READY_TIMEOUT_SEC="${QWEN_READY_TIMEOUT_SEC:-180}"
READY_POLL_INTERVAL_SEC="${QWEN_READY_POLL_INTERVAL_SEC:-2}"

if [[ ! -x "$VLLM_BIN" ]]; then
  echo "vLLM binary not found at $VLLM_BIN" >&2
  exit 1
fi

find_existing_pid() {
  ps -ef | grep -F "vllm serve $MODEL_NAME --host $HOST --port $PORT" | grep -v grep | awk 'NR==1 {print $2}'
}

wait_for_ready() {
  local timeout_sec="$1"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    if curl -sf "http://$HOST:$PORT/v1/models" >/dev/null 2>&1; then
      return 0
    fi
    sleep "$READY_POLL_INTERVAL_SEC"
    elapsed=$((elapsed + READY_POLL_INTERVAL_SEC))
  done
  return 1
}

EXISTING_PID="$(find_existing_pid || true)"
if [[ -n "${EXISTING_PID:-}" ]]; then
  if wait_for_ready "$READY_TIMEOUT_SEC"; then
    echo "$EXISTING_PID" > "$PID_FILE"
    echo "Qwen vLLM server is already running on http://$HOST:$PORT"
    echo "PID: $EXISTING_PID"
    echo "Log: $LOG_PATH"
    exit 0
  fi

  echo "Found existing Qwen vLLM process PID $EXISTING_PID, but it did not become ready within ${READY_TIMEOUT_SEC}s." >&2
  echo "Last server log lines:" >&2
  tail -n 40 "$LOG_PATH" >&2 || true
  exit 1
fi

if ss -ltn | grep -q ":$PORT "; then
  if curl -sf "http://$HOST:$PORT/v1/models" >/dev/null 2>&1; then
    PID="$(find_existing_pid || true)"
    if [[ -n "${PID:-}" ]]; then
      echo "$PID" > "$PID_FILE"
    fi
    echo "Qwen vLLM server is already running on http://$HOST:$PORT"
    if [[ -n "${PID:-}" ]]; then
      echo "PID: $PID"
    fi
    echo "Log: $LOG_PATH"
    exit 0
  fi

  echo "Port $PORT is already in use by another process. Refusing to start another server." >&2
  exit 1
fi

mkdir -p "$(dirname "$LOG_PATH")"

setsid -f /usr/bin/env PYTHONUNBUFFERED=1 stdbuf -oL -eL \
  "$VLLM_BIN" serve "$MODEL_NAME" \
  --host "$HOST" \
  --port "$PORT" \
  --gpu-memory-utilization "$GPU_MEMORY_UTILIZATION" \
  --max-model-len "$MAX_MODEL_LEN" \
  --allowed-local-media-path "$ALLOWED_MEDIA_PATH" \
  --enforce-eager \
  --mm-encoder-attn-backend TORCH_SDPA \
  --quantization awq \
  --dtype float16 \
  --attention-backend TRITON_ATTN \
  > "$LOG_PATH" 2>&1

sleep 1
PID="$(find_existing_pid || true)"
if [[ -n "${PID:-}" ]]; then
  echo "$PID" > "$PID_FILE"
  echo "Started Qwen vLLM process, waiting for readiness on http://$HOST:$PORT"
  echo "PID: $PID"
  echo "Log: $LOG_PATH"
  if wait_for_ready "$READY_TIMEOUT_SEC"; then
    echo "Qwen vLLM server is ready on http://$HOST:$PORT"
  else
    echo "Qwen vLLM process started, but the API was not ready within ${READY_TIMEOUT_SEC}s." >&2
    echo "Last server log lines:" >&2
    tail -n 60 "$LOG_PATH" >&2 || true
    exit 1
  fi
else
  echo "Server launch command returned, but no matching vLLM process was found." >&2
  exit 1
fi
