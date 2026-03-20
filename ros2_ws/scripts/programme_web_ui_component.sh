#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

COMPONENT="${1:-}"
LOG_FILE="${2:-}"

if [[ -z "$COMPONENT" || -z "$LOG_FILE" ]]; then
  echo "usage: $0 <ros|frontend> <log-file>" >&2
  exit 2
fi

mkdir -p "$(dirname -- "$LOG_FILE")"
exec >>"$LOG_FILE" 2>&1

echo "[runner] component=$COMPONENT started at $(date --iso-8601=seconds)"

case "$COMPONENT" in
  ros)
    cd "$WS_ROOT"
    set +u
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    set -u
    PROGRAMME_WEB_UI_CPU_ONLY="${PROGRAMME_WEB_UI_CPU_ONLY:-1}"
    if [[ "$PROGRAMME_WEB_UI_CPU_ONLY" == "1" ]]; then
      unset CUDA_VISIBLE_DEVICES
      unset HIP_VISIBLE_DEVICES
      unset ROCR_VISIBLE_DEVICES
      unset MESA_D3D12_DEFAULT_ADAPTER_NAME
      export ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS="1"
      echo "[runner] CPU-only ML mode enabled for Programme Web UI"
    fi
    restart_count=0
    while true; do
      started_at="$(date +%s)"
      echo "[runner] starting ros2 launch tactile_bringup phase8_web_ui.launch.py at $(date --iso-8601=seconds)"
      set +e
      ros2 launch tactile_bringup phase8_web_ui.launch.py
      exit_code=$?
      set -e
      ended_at="$(date +%s)"
      runtime_sec=$((ended_at - started_at))
      echo "[runner] ros stack exited with code=${exit_code} after ${runtime_sec}s"
      if (( runtime_sec < 20 )); then
        restart_count=$((restart_count + 1))
      else
        restart_count=0
      fi
      if (( restart_count >= 3 )); then
        echo "[runner] ros stack exited too quickly three times; giving up"
        exit "$exit_code"
      fi
      echo "[runner] restarting ros stack in 3s"
      sleep 3
    done
    ;;
  frontend)
    export PATH="$HOME/.local/bin:$PATH"
    cd "$WS_ROOT/src/tactile_web_bridge/frontend"
    exec npm run dev -- --host 127.0.0.1 --port 5173
    ;;
  *)
    echo "[runner] unsupported component: $COMPONENT" >&2
    exit 2
    ;;
esac
