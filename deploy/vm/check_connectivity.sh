#!/usr/bin/env bash
set -euo pipefail

HOST_IP="${1:-192.168.56.1}"
MODE="${2:-none}"  # none|talker|listener

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh"

echo "Pinging Windows host-only IP: ${HOST_IP}"
if ! ping -c 2 "${HOST_IP}" >/dev/null; then
  echo "Cannot reach host IP ${HOST_IP}. Check NAT+Host-only configuration." >&2
  exit 1
fi

echo "Ping OK."

case "${MODE}" in
  talker)
    echo "Starting ROS2 talker on VM..."
    exec ros2 run demo_nodes_cpp talker
    ;;
  listener)
    echo "Starting ROS2 listener on VM..."
    exec ros2 run demo_nodes_cpp listener
    ;;
  *)
    echo "Connectivity check complete. Optional mode: talker or listener."
    ;;
esac
