#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PROGRAMME_ROOT="$(cd -- "$WS_ROOT/.." && pwd)"
LOG_DIR="$WS_ROOT/.runtime/programme-ui"
ROS_PID_FILE="$LOG_DIR/ros.pid"
FRONTEND_PID_FILE="$LOG_DIR/frontend.pid"
QWEN_PID_FILE="$LOG_DIR/qwen_vllm_server.pid"
QWEN_OWNED_MARKER="$LOG_DIR/qwen_vllm_owned"
ROS_SESSION="programme-ui-ros"
FRONTEND_SESSION="programme-ui-frontend"
ROS_UNIT="programme-ui-ros.service"
FRONTEND_UNIT="programme-ui-frontend.service"
QWEN_STOP_SCRIPT="$PROGRAMME_ROOT/scripts/stop_qwen_vllm.sh"

kill_pid_file() {
  local pid_file="$1"
  if [[ ! -f "$pid_file" ]]; then
    return
  fi

  local pid
  pid="$(<"$pid_file")"
  if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
  fi
  rm -f "$pid_file"
}

kill_tmux_session() {
  local session="$1"
  if ! command -v tmux >/dev/null 2>&1; then
    return 0
  fi
  if ! tmux has-session -t "$session" 2>/dev/null; then
    return 0
  fi
  tmux kill-session -t "$session" 2>/dev/null || true
}

stop_systemd_unit() {
  local unit="$1"
  if ! command -v systemctl >/dev/null 2>&1; then
    return 0
  fi
  systemctl --user stop "$unit" >/dev/null 2>&1 || true
  systemctl --user reset-failed "$unit" >/dev/null 2>&1 || true
}

echo "[stop] stopping Programme Web UI processes"
kill_pid_file "$ROS_PID_FILE"
kill_pid_file "$FRONTEND_PID_FILE"
stop_systemd_unit "$ROS_UNIT"
stop_systemd_unit "$FRONTEND_UNIT"
kill_tmux_session "$ROS_SESSION"
kill_tmux_session "$FRONTEND_SESSION"
"$SCRIPT_DIR/cleanup_sim_stack.sh" >/dev/null 2>&1 || true
pkill -f "programme_web_ui_component.sh ros" 2>/dev/null || true
pkill -f "programme_web_ui_component.sh frontend" 2>/dev/null || true
pkill -f "ros2 launch tactile_bringup phase8_web_ui.launch.py" 2>/dev/null || true
pkill -f "tactile_web_gateway" 2>/dev/null || true
pkill -f "npm run dev -- --host 127.0.0.1 --port 5173" 2>/dev/null || true
pkill -f "vite --host 127.0.0.1 --port 5173" 2>/dev/null || true
if [[ -f "$QWEN_OWNED_MARKER" && -x "$QWEN_STOP_SCRIPT" ]]; then
  echo "[stop] stopping managed Qwen vLLM service"
  QWEN_PID_FILE="$QWEN_PID_FILE" "$QWEN_STOP_SCRIPT" || true
fi
rm -f "$QWEN_OWNED_MARKER" "$QWEN_PID_FILE"
echo "[stop] done"
