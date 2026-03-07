#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${PROJECT_ROOT}/ros2_ws/install/setup.bash"
DDS_TEMPLATE_FILE="${PROJECT_ROOT}/config/dds/cyclonedds_vm.xml"
DDS_RUNTIME_FILE="${TMPDIR:-/tmp}/programme_cyclonedds_vm_${DOMAIN_ID}.xml"

# Keep ROS2 tools on the system interpreter path even when the caller shell
# already has Conda or custom Python site-packages active.
unset PYTHONHOME || true
unset PYTHONPATH || true
export PYTHONNOUSERSITE=1

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

resolve_vm_dds_ip() {
  local requested_ip="${VM_HOST_ONLY_IP:-${DDS_INTERFACE_IP:-}}"
  local default_iface=""
  local candidate=""

  if [[ -n "${requested_ip}" ]]; then
    if ip -o -4 addr show up 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | grep -Fxq "${requested_ip}"; then
      printf '%s\n' "${requested_ip}"
      return 0
    fi
    echo "[WARN] requested DDS interface IP ${requested_ip} is not present on this VM; auto-detecting instead." >&2
  fi

  default_iface="$(ip route show default 2>/dev/null | awk 'NR==1 {print $5}')"
  candidate="$(
    ip -o -4 addr show scope global up 2>/dev/null | awk -v default_iface="${default_iface}" '
      {
        iface = $2
        split($4, addr_parts, "/")
        ip = addr_parts[1]
        if (iface ~ /^(lo|docker|br-|veth|virbr|zt|tailscale|tun|tap)/) {
          next
        }
        if (default_iface != "" && iface != default_iface) {
          print ip
          exit
        }
      }
    '
  )"
  if [[ -n "${candidate}" ]]; then
    printf '%s\n' "${candidate}"
    return 0
  fi

  candidate="$(
    ip -o -4 addr show scope global up 2>/dev/null | awk '
      {
        iface = $2
        split($4, addr_parts, "/")
        ip = addr_parts[1]
        if (iface ~ /^(lo|docker|br-|veth|virbr|zt|tailscale|tun|tap)/) {
          next
        }
        print ip
        exit
      }
    '
  )"
  if [[ -n "${candidate}" ]]; then
    printf '%s\n' "${candidate}"
    return 0
  fi

  return 1
}

resolve_windows_dds_peer_ip() {
  local local_ip="$1"
  local requested_ip="${WINDOWS_HOST_ONLY_IP:-${DDS_PEER_IP:-}}"

  if [[ -n "${requested_ip}" ]]; then
    printf '%s\n' "${requested_ip}"
    return 0
  fi

  if [[ "${local_ip}" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)\.[0-9]+$ ]]; then
    printf '%s.%s.%s.1\n' "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}" "${BASH_REMATCH[3]}"
    return 0
  fi

  printf '%s\n' "192.168.147.1"
}

write_cyclonedds_runtime_file() {
  local local_ip="$1"
  local peer_ip="$2"

  if [[ ! -f "${DDS_TEMPLATE_FILE}" ]]; then
    echo "[ERROR] CycloneDDS template not found: ${DDS_TEMPLATE_FILE}" >&2
    return 1
  fi

  cat >"${DDS_RUNTIME_FILE}" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="${local_ip}" />
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer Address="127.0.0.1" />
        <Peer Address="${local_ip}" />
        <Peer Address="${peer_ip}" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
}

if ! VM_DDS_IP="$(resolve_vm_dds_ip)"; then
  echo "[ERROR] failed to resolve a usable VM IPv4 for CycloneDDS." >&2
  echo "[ERROR] Set VM_HOST_ONLY_IP=<ipv4> explicitly and retry." >&2
  exit 1
fi

WINDOWS_DDS_PEER_IP="$(resolve_windows_dds_peer_ip "${VM_DDS_IP}")"
write_cyclonedds_runtime_file "${VM_DDS_IP}" "${WINDOWS_DDS_PEER_IP}"

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
unset ROS_LOCALHOST_ONLY || true
export PROGRAMME_VM_HOST_ONLY_IP="${VM_DDS_IP}"
export PROGRAMME_WINDOWS_HOST_ONLY_IP="${WINDOWS_DDS_PEER_IP}"
export CYCLONEDDS_URI="file://${DDS_RUNTIME_FILE}"

echo "ROS2 VM environment ready."
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "PROGRAMME_VM_HOST_ONLY_IP=${PROGRAMME_VM_HOST_ONLY_IP}"
echo "PROGRAMME_WINDOWS_HOST_ONLY_IP=${PROGRAMME_WINDOWS_HOST_ONLY_IP}"
