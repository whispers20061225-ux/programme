#!/usr/bin/env bash
set -euo pipefail

PORT="${QWEN_PORT:-8000}"
PID_FILE="${QWEN_PID_FILE:-$HOME/qwen_vllm_server.pid}"

wait_for_port_release() {
  local attempts=0
  while (( attempts < 20 )); do
    if ! ss -ltn | grep -q ":$PORT "; then
      return 0
    fi
    sleep 1
    attempts=$((attempts + 1))
  done
  return 1
}

force_kill_pid() {
  local pid="$1"
  if [[ -z "${pid:-}" ]]; then
    return 0
  fi
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    sleep 2
    kill -9 "$pid" 2>/dev/null || true
  fi
}

if [[ -f "$PID_FILE" ]]; then
  PID="$(cat "$PID_FILE")"
  if kill -0 "$PID" 2>/dev/null; then
    force_kill_pid "$PID"
    rm -f "$PID_FILE"
    if wait_for_port_release; then
      echo "Stopped Qwen vLLM server PID $PID"
      exit 0
    fi
  fi
  rm -f "$PID_FILE"
fi

PIDS="$(lsof -ti TCP:"$PORT" -sTCP:LISTEN 2>/dev/null || true)"
if [[ -n "$PIDS" ]]; then
  echo "$PIDS" | xargs -r -n1 bash -lc 'kill "$0" 2>/dev/null || true; sleep 1; kill -9 "$0" 2>/dev/null || true'
  if wait_for_port_release; then
    echo "Stopped server(s) listening on port $PORT"
    exit 0
  fi
fi

if ss -ltn | grep -q ":$PORT "; then
  echo "Port $PORT is still in use after stop attempts" >&2
  exit 1
fi

echo "No Qwen vLLM server found on port $PORT"
