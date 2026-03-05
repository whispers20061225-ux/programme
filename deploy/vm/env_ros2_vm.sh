#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${PROJECT_ROOT}/ros2_ws/install/setup.bash"

source_setup_compat() {
  local setup_path="$1"
  local had_nounset=0
  case "$-" in
    *u*) had_nounset=1 ;;
  esac

  # Some ROS setup scripts read vars that may be unset under `set -u`.
  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
  set +u
  # shellcheck disable=SC1090
  source "${setup_path}"
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

if [[ -f "${ROS_SETUP}" ]]; then
  source_setup_compat "${ROS_SETUP}"
else
  echo "ROS2 setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ -f "${WS_SETUP}" ]]; then
  source_setup_compat "${WS_SETUP}"
fi

require_cyclonedds_rmw() {
  if [[ -f "/opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so" ]]; then
    return 0
  fi
  if ldconfig -p 2>/dev/null | grep -q "librmw_cyclonedds_cpp.so"; then
    return 0
  fi

  echo "[ERROR] RMW implementation 'rmw_cyclonedds_cpp' is not installed on this VM." >&2
  echo "[ERROR] Install it with:" >&2
  echo "        sudo apt update && sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp" >&2
  return 1
}

require_cyclonedds_rmw

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
unset ROS_LOCALHOST_ONLY || true
export CYCLONEDDS_URI="file://${PROJECT_ROOT}/config/dds/cyclonedds_vm.xml"

echo "ROS2 VM environment ready."
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
