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
    echo "[runner] backend ROS stack is owned by compatibility unit programme-phase8-full.service; use systemctl --user restart programme-phase8-full.service" >&2
    exit 2
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
