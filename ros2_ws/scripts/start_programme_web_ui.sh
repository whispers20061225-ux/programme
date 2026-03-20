#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PROGRAMME_ROOT="$(cd -- "$WS_ROOT/.." && pwd)"
LOG_DIR="$WS_ROOT/.runtime/programme-ui"
ROS_LOG="$LOG_DIR/ros.log"
FRONTEND_LOG="$LOG_DIR/frontend.log"
QWEN_LOG="$LOG_DIR/qwen_vllm_server.log"
ROS_PID_FILE="$LOG_DIR/ros.pid"
FRONTEND_PID_FILE="$LOG_DIR/frontend.pid"
QWEN_PID_FILE="$LOG_DIR/qwen_vllm_server.pid"
QWEN_OWNED_MARKER="$LOG_DIR/qwen_vllm_owned"
ROS_SESSION="programme-ui-ros"
FRONTEND_SESSION="programme-ui-frontend"
ROS_UNIT="programme-ui-ros.service"
FRONTEND_UNIT="programme-ui-frontend.service"
QWEN_START_SCRIPT="$PROGRAMME_ROOT/scripts/start_qwen_vllm.sh"
QWEN_STOP_SCRIPT="$PROGRAMME_ROOT/scripts/stop_qwen_vllm.sh"
QWEN_MODELS_URL="${QWEN_MODELS_URL:-http://127.0.0.1:8000/v1/models}"
QWEN_PORT="${QWEN_PORT:-8000}"
PROGRAMME_WEB_UI_START_QWEN_VLLM="${PROGRAMME_WEB_UI_START_QWEN_VLLM:-1}"

OPEN_BROWSER=1
if [[ "${1:-}" == "--no-browser" ]]; then
  OPEN_BROWSER=0
fi

mkdir -p "$LOG_DIR"

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

ensure_user_linger() {
  if ! command -v loginctl >/dev/null 2>&1; then
    return 0
  fi
  local linger_state
  linger_state="$(loginctl show-user "$USER" -p Linger --value 2>/dev/null || true)"
  if [[ "$linger_state" == "yes" ]]; then
    return 0
  fi
  if command -v sudo >/dev/null 2>&1; then
    sudo -n loginctl enable-linger "$USER" >/dev/null 2>&1 || true
  fi
}

has_systemd_user() {
  command -v systemd-run >/dev/null 2>&1 && systemctl --user show-environment >/dev/null 2>&1
}

stop_systemd_unit() {
  local unit="$1"
  if ! has_systemd_user; then
    return 0
  fi
  systemctl --user stop "$unit" >/dev/null 2>&1 || true
  systemctl --user reset-failed "$unit" >/dev/null 2>&1 || true
}

start_component() {
  local session="$1"
  local unit="$2"
  local pid_file="$3"
  local log_file="$4"
  local component="$5"

  : >"$log_file"

  if has_systemd_user; then
    systemd-run \
      --user \
      --unit "$unit" \
      --property=Type=simple \
      --property=WorkingDirectory="$WS_ROOT" \
      --property=Restart=always \
      --property=RestartSec=3s \
      "$SCRIPT_DIR/programme_web_ui_component.sh" "$component" "$log_file" >/dev/null
    sleep 1
    if ! systemctl --user is-active --quiet "$unit"; then
      echo "[start] $component systemd unit exited immediately"
      systemctl --user status "$unit" --no-pager -l || true
      tail -n 120 "$log_file" || true
      exit 1
    fi
    systemctl --user show "$unit" -p MainPID --value | head -n 1 >"$pid_file" || true
    return
  fi

  if command -v tmux >/dev/null 2>&1; then
    tmux new-session -d -s "$session" "$SCRIPT_DIR/programme_web_ui_component.sh" "$component" "$log_file"
    sleep 1
    if ! tmux has-session -t "$session" 2>/dev/null; then
      echo "[start] $component session exited immediately"
      tail -n 120 "$log_file" || true
      exit 1
    fi
    tmux list-panes -t "$session" -F '#{pane_pid}' | head -n 1 >"$pid_file" || true
    return
  fi

  echo "[start] tmux not found; falling back to setsid for $component"
  case "$component" in
    ros)
      setsid "$SCRIPT_DIR/programme_web_ui_component.sh" ros "$log_file" < /dev/null &
      ;;
    frontend)
      setsid "$SCRIPT_DIR/programme_web_ui_component.sh" frontend "$log_file" < /dev/null &
      ;;
    *)
      echo "[start] unsupported component: $component"
      exit 1
      ;;
  esac
  local pid=$!
  echo "$pid" >"$pid_file"
  sleep 1
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[start] $component process exited immediately"
    tail -n 120 "$log_file" || true
    exit 1
  fi
}

wait_for_http() {
  local url="$1"
  local label="$2"
  local timeout_sec="$3"

  for ((attempt = 1; attempt <= timeout_sec; attempt++)); do
    if curl -fsS "$url" >/dev/null 2>&1; then
      echo "[start] $label ready: $url"
      return 0
    fi
    sleep 1
  done

  echo "[start] $label did not become ready: $url"
  return 1
}

ensure_python_deps() {
  if python3 - <<'PY' >/dev/null 2>&1
import fastapi  # noqa: F401
import uvicorn  # noqa: F401
import websockets  # noqa: F401
PY
  then
    return
  fi

  echo "[start] installing missing Python web gateway dependencies"
  python3 -m pip install --user --break-system-packages fastapi uvicorn websockets
}

ensure_qwen_service() {
  if curl -fsS "$QWEN_MODELS_URL" >/dev/null 2>&1; then
    if [[ -f "$QWEN_OWNED_MARKER" ]]; then
      echo "[start] managed Qwen vLLM already ready: $QWEN_MODELS_URL"
    else
      echo "[start] external Qwen vLLM already ready: $QWEN_MODELS_URL"
    fi
    return
  fi

  if [[ "$PROGRAMME_WEB_UI_START_QWEN_VLLM" != "1" ]]; then
    echo "[start] dialog model endpoint is offline and PROGRAMME_WEB_UI_START_QWEN_VLLM=0"
    exit 1
  fi

  if [[ ! -x "$QWEN_START_SCRIPT" ]]; then
    echo "[start] Qwen vLLM start script not found: $QWEN_START_SCRIPT"
    exit 1
  fi

  if ss -ltn | grep -q ":$QWEN_PORT "; then
    echo "[start] found stale listener on port $QWEN_PORT; clearing it before Qwen startup"
    if [[ -x "$QWEN_STOP_SCRIPT" ]]; then
      QWEN_PORT="$QWEN_PORT" QWEN_PID_FILE="$QWEN_PID_FILE" "$QWEN_STOP_SCRIPT" || true
    fi
    sleep 2
  fi

  echo "[start] launching Qwen vLLM service"
  QWEN_LOG_PATH="$QWEN_LOG" QWEN_PID_FILE="$QWEN_PID_FILE" "$QWEN_START_SCRIPT"
  if ! curl -fsS "$QWEN_MODELS_URL" >/dev/null 2>&1; then
    echo "[start] Qwen vLLM did not become ready: $QWEN_MODELS_URL"
    tail -n 80 "$QWEN_LOG" || true
    exit 1
  fi
  touch "$QWEN_OWNED_MARKER"
}

echo "[start] stopping stale Programme Web UI processes"
kill_pid_file "$ROS_PID_FILE"
kill_pid_file "$FRONTEND_PID_FILE"
ensure_user_linger
stop_systemd_unit "$ROS_UNIT"
stop_systemd_unit "$FRONTEND_UNIT"
kill_tmux_session "$ROS_SESSION"
kill_tmux_session "$FRONTEND_SESSION"
"$SCRIPT_DIR/cleanup_sim_stack.sh" >/dev/null 2>&1 || true
pkill -f "tactile_web_gateway" 2>/dev/null || true
pkill -f "npm run dev -- --host 127.0.0.1 --port 5173" 2>/dev/null || true
pkill -f "vite --host 127.0.0.1 --port 5173" 2>/dev/null || true

ensure_python_deps
ensure_qwen_service

echo "[start] launching ROS gateway stack"
start_component "$ROS_SESSION" "$ROS_UNIT" "$ROS_PID_FILE" "$ROS_LOG" ros
if ! wait_for_http "http://127.0.0.1:8765/api/bootstrap" "gateway" 60; then
  echo "[start] gateway failed to start; recent log:"
  tail -n 120 "$ROS_LOG" || true
  exit 1
fi

echo "[start] launching frontend dev server"
start_component "$FRONTEND_SESSION" "$FRONTEND_UNIT" "$FRONTEND_PID_FILE" "$FRONTEND_LOG" frontend
if ! wait_for_http "http://127.0.0.1:5173/control" "frontend" 30; then
  echo "[start] frontend failed to start; recent log:"
  tail -n 120 "$FRONTEND_LOG" || true
  exit 1
fi

if [[ "$OPEN_BROWSER" -eq 1 ]] && command -v powershell.exe >/dev/null 2>&1; then
  powershell.exe -NoProfile -Command "Start-Process 'http://127.0.0.1:5173/control'" >/dev/null 2>&1 || true
fi

echo "[start] Programme Web UI is ready"
echo "[start] control page: http://127.0.0.1:5173/control"
echo "[start] gateway root: http://127.0.0.1:8765/"
echo "[start] ros log: $ROS_LOG"
echo "[start] frontend log: $FRONTEND_LOG"
echo "[start] qwen log: $QWEN_LOG"
echo "[start] ros pid: $(<"$ROS_PID_FILE")"
echo "[start] frontend pid: $(<"$FRONTEND_PID_FILE")"
echo "[start] ros session: $ROS_SESSION"
echo "[start] frontend session: $FRONTEND_SESSION"
echo "[start] ros unit: $ROS_UNIT"
echo "[start] frontend unit: $FRONTEND_UNIT"
