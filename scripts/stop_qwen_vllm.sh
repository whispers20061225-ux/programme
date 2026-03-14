#!/usr/bin/env bash
set -euo pipefail

PORT="${QWEN_PORT:-8000}"
PID_FILE="${QWEN_PID_FILE:-$HOME/qwen_vllm_server.pid}"

if [[ -f "$PID_FILE" ]]; then
  PID="$(cat "$PID_FILE")"
  if kill -0 "$PID" 2>/dev/null; then
    kill "$PID"
    echo "Stopped Qwen vLLM server PID $PID"
    rm -f "$PID_FILE"
    exit 0
  fi
  rm -f "$PID_FILE"
fi

PIDS="$(lsof -ti TCP:"$PORT" -sTCP:LISTEN 2>/dev/null || true)"
if [[ -n "$PIDS" ]]; then
  echo "$PIDS" | xargs -r kill
  echo "Stopped server(s) listening on port $PORT"
  exit 0
fi

echo "No Qwen vLLM server found on port $PORT"
