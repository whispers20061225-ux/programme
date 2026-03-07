#!/usr/bin/env bash
set -euo pipefail

DOMAIN_ID="${1:-0}"
LAUNCH_TIMEOUT_SEC="${2:-25}"
ARM_TIMEOUT_SEC="${3:-20}"
MIN_TACTILE_HZ="${4:-10.0}"
CONDA_ENV="${5:-dayiprogramme312}"
START_GUI="${6:-false}"
BRIDGE_CLOCK="${7:-true}"
KEEP_LAUNCH="${KEEP_LAUNCH:-false}"
TACTILE_HZ_SAMPLE_SEC="${TACTILE_HZ_SAMPLE_SEC:-8}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
SHOW_VISION_UI="${SHOW_VISION_UI:-false}"
SHOW_SIMULATION_UI="${SHOW_SIMULATION_UI:-false}"
GAZEBO_WORLD_NAME="${GAZEBO_WORLD_NAME:-phase6_tabletop_world}"
GAZEBO_SPAWN_X="${GAZEBO_SPAWN_X:-0.24}"
GAZEBO_SPAWN_Y="${GAZEBO_SPAWN_Y:-0.0}"
GAZEBO_SPAWN_Z="${GAZEBO_SPAWN_Z:-0.405}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LAUNCH_LOG_DIR="${PROJECT_ROOT}/ros2_ws/log"
LAUNCH_LOG_FILE="${LAUNCH_LOG_DIR}/phase6_sim_gazebo_$(date +%Y%m%d_%H%M%S).log"

log_step() { echo "[STEP] $*"; }
log_ok() { echo "[OK] $*"; }
log_fail() { echo "[FAIL] $*" >&2; }
is_true() { [[ "${1,,}" == "true" || "${1}" == "1" || "${1,,}" == "yes" ]]; }

has_ros_package() {
  local package_name="$1"
  set +e
  ros2 pkg prefix "${package_name}" >/dev/null 2>&1
  local rc=$?
  set -e
  return ${rc}
}

package_prefix() {
  local package_name="$1"
  set +e
  ros2 pkg prefix "${package_name}" 2>/dev/null
  local rc=$?
  set -e
  return ${rc}
}

check_installed_asset() {
  local package_name="$1"
  local relative_path="$2"
  local source_path="${3:-}"
  local prefix=""
  local installed_path=""

  prefix="$(package_prefix "${package_name}" || true)"
  if [[ -z "${prefix}" ]]; then
    return 1
  fi

  installed_path="${prefix}/${relative_path}"
  if [[ ! -e "${installed_path}" ]]; then
    echo "[WARN] missing installed asset: ${installed_path}"
    return 1
  fi

  if [[ -n "${source_path}" && -e "${source_path}" && "${source_path}" -nt "${installed_path}" ]]; then
    echo "[WARN] stale installed asset: ${installed_path}"
    return 1
  fi

  return 0
}

gazebo_workspace_assets_ready() {
  check_installed_asset \
    "tactile_sim" \
    "share/tactile_sim/urdf/dofbot_gazebo.urdf.xacro" \
    "${PROJECT_ROOT}/ros2_ws/src/tactile_sim/urdf/dofbot_gazebo.urdf.xacro" || return 1
  check_installed_asset \
    "tactile_sim" \
    "share/tactile_sim/launch/gazebo_arm.launch.py" \
    "${PROJECT_ROOT}/ros2_ws/src/tactile_sim/launch/gazebo_arm.launch.py" || return 1
  check_installed_asset \
    "tactile_sim" \
    "share/tactile_sim/config/ros2_controllers.yaml" \
    "${PROJECT_ROOT}/ros2_ws/src/tactile_sim/config/ros2_controllers.yaml" || return 1
  check_installed_asset \
    "tactile_sim" \
    "share/tactile_sim/worlds/phase6_tabletop.world" \
    "${PROJECT_ROOT}/ros2_ws/src/tactile_sim/worlds/phase6_tabletop.world" || return 1
  check_installed_asset \
    "tactile_sim" \
    "share/tactile_sim/meshes/base_link.STL" \
    "${PROJECT_ROOT}/models/meshes/base_link.STL" || return 1
  check_installed_asset \
    "tactile_bringup" \
    "share/tactile_bringup/launch/phase6_sim_gazebo.launch.py" \
    "${PROJECT_ROOT}/ros2_ws/src/tactile_bringup/launch/phase6_sim_gazebo.launch.py" || return 1
  check_installed_asset \
    "tactile_bringup" \
    "share/tactile_bringup/config/phase6_sim_gazebo.yaml" \
    "${PROJECT_ROOT}/ros2_ws/src/tactile_bringup/config/phase6_sim_gazebo.yaml" || return 1
  return 0
}

wait_for_node() {
  local node="$1"
  local node_alt="${node#/}"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local nodes
    nodes="$(ros2 node list 2>/dev/null || true)"
    if echo "${nodes}" | grep -Fxq "${node}" || echo "${nodes}" | grep -Fxq "${node_alt}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

wait_for_topic() {
  local topic="$1"
  local topic_alt="${topic#/}"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local topics
    topics="$(ros2 topic list 2>/dev/null || true)"
    if echo "${topics}" | grep -Fxq "${topic}" \
      || echo "${topics}" | grep -Fxq "${topic_alt}" \
      || echo "${topics}" | grep -Fxq "/${topic_alt}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

wait_for_service() {
  local service="$1"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local services
    services="$(ros2 service list 2>/dev/null || true)"
    if echo "${services}" | grep -Fxq "${service}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

wait_for_action() {
  local action_name="$1"
  local timeout_sec="$2"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local actions
    actions="$(ros2 action list 2>/dev/null || true)"
    if echo "${actions}" | grep -Fxq "${action_name}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

call_setbool_service() {
  local service_name="$1"
  local data_value="$2"
  local timeout_sec="$3"
  local output=""
  local rc=1

  set +e
  output="$(timeout "${timeout_sec}s" ros2 service call "${service_name}" std_srvs/srv/SetBool "{data: ${data_value}}" 2>&1)"
  rc=$?
  set -e

  echo "${output}"
  if [[ ${rc} -ne 0 ]]; then
    return 1
  fi
  if echo "${output}" | tr '[:upper:]' '[:lower:]' | grep -Eq 'success[[:space:]]*[:=][[:space:]]*true'; then
    return 0
  fi
  return 1
}

wait_for_arm_connected_state() {
  local timeout_sec="$1"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    local state_output=""
    local rc=1
    set +e
    state_output="$(timeout 3s ros2 topic echo /arm/state --once 2>/dev/null)"
    rc=$?
    set -e
    if [[ ${rc} -eq 0 ]] && echo "${state_output}" | tr '[:upper:]' '[:lower:]' | grep -Eq 'connected[[:space:]]*:[[:space:]]*true|connected[[:space:]]*=[[:space:]]*true'; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

sample_hz() {
  local topic="$1"
  local sample_sec="$2"
  local out_file
  local err_file
  local hz_pid
  local output
  out_file="$(mktemp)"
  err_file="$(mktemp)"

  set +e
  stdbuf -oL -eL ros2 topic hz "${topic}" >"${out_file}" 2>"${err_file}" &
  hz_pid=$!
  set -e

  sleep "${sample_sec}"
  kill "${hz_pid}" >/dev/null 2>&1 || true
  wait "${hz_pid}" >/dev/null 2>&1 || true

  output="$(cat "${out_file}" "${err_file}" 2>/dev/null || true)"
  rm -f "${out_file}" "${err_file}" || true
  echo "${output}" | awk '/average rate/ {print $3}' | tail -n 1
}

check_rate_ge() {
  local actual="$1"
  local minimum="$2"
  awk -v r="${actual}" -v m="${minimum}" 'BEGIN { exit ((r + 0.0 >= m + 0.0) ? 0 : 1) }'
}

dump_runtime_diagnostics() {
  echo "[DIAG] ros2 node list:"
  ros2 node list 2>/dev/null || true
  echo "[DIAG] ros2 topic list:"
  ros2 topic list 2>/dev/null || true
  echo "[DIAG] ros2 service list:"
  ros2 service list 2>/dev/null || true
  echo "[DIAG] ros2 action list:"
  ros2 action list 2>/dev/null || true
  echo "[DIAG] tail launch log:"
  tail -n 120 "${LAUNCH_LOG_FILE}" 2>/dev/null || true
}

launch_log_has_dds_interface_mismatch() {
  [[ -f "${LAUNCH_LOG_FILE}" ]] && grep -Eq \
    'does not match an available interface|rmw_create_node: failed to create domain' \
    "${LAUNCH_LOG_FILE}"
}

report_dds_interface_mismatch() {
  log_fail "CycloneDDS failed to bind a usable interface for this VM."
  echo "[HINT] resolved VM DDS IP: ${PROGRAMME_VM_HOST_ONLY_IP:-unknown}" >&2
  echo "[HINT] resolved Windows DDS peer IP: ${PROGRAMME_WINDOWS_HOST_ONLY_IP:-unknown}" >&2
  echo "[HINT] override explicitly if needed:" >&2
  echo "[HINT]   WINDOWS_HOST_ONLY_IP=<windows_ip> VM_HOST_ONLY_IP=<vm_ip> bash deploy/vm/start_ui_with_gazebo_guard.sh ..." >&2
}

cleanup_stale_gazebo_processes() {
  pkill -f "phase6_sim_gazebo.launch.py" >/dev/null 2>&1 || true
  pkill -f "gazebo_arm.launch.py" >/dev/null 2>&1 || true
  pkill -f "arm_sim_driver_node" >/dev/null 2>&1 || true
  pkill -f "tactile_sim_node" >/dev/null 2>&1 || true
  pkill -f "arm_control_node" >/dev/null 2>&1 || true
  pkill -f "demo_task_node" >/dev/null 2>&1 || true
  pkill -f "tactile_ui_subscriber" >/dev/null 2>&1 || true
  pkill -f "robot_state_publisher" >/dev/null 2>&1 || true
  pkill -f "ros_gz_bridge" >/dev/null 2>&1 || true
  pkill -f "parameter_bridge" >/dev/null 2>&1 || true
  pkill -f "gz sim" >/dev/null 2>&1 || true
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
}

auto_build_gazebo_packages() {
  local ws_dir="${PROJECT_ROOT}/ros2_ws"
  local build_log="${LAUNCH_LOG_DIR}/phase6_sim_build_$(date +%Y%m%d_%H%M%S).log"
  local rc=1

  if ! /usr/bin/python3 -c "import catkin_pkg" >/dev/null 2>&1; then
    log_fail "system python is missing catkin_pkg; install VM build dependency first"
    echo "[HINT] sudo apt update && sudo apt install -y python3-catkin-pkg-modules"
    return 1
  fi

  mkdir -p "${LAUNCH_LOG_DIR}"
  log_step "building missing Gazebo workspace packages"
  echo "[INFO] build log: ${build_log}"

  pushd "${ws_dir}" >/dev/null
  set +e
  (
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_EXE CONDA_PYTHON_EXE PYTHONHOME PYTHONPATH _CE_CONDA _CE_M
    export PATH="/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"
    export AMENT_PYTHON_EXECUTABLE="/usr/bin/python3"
    export COLCON_PYTHON_EXECUTABLE="/usr/bin/python3"
    export Python3_EXECUTABLE="/usr/bin/python3"
    export PYTHON_EXECUTABLE="/usr/bin/python3"
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"
    colcon build --symlink-install \
      --packages-select tactile_interfaces tactile_control tactile_task tactile_ui_bridge tactile_sim tactile_bringup \
      --allow-overriding tactile_interfaces tactile_control tactile_task tactile_ui_bridge tactile_sim tactile_bringup \
      --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
  ) >"${build_log}" 2>&1
  rc=$?
  set -e
  popd >/dev/null

  if [[ ${rc} -ne 0 ]]; then
    log_fail "failed to build Gazebo workspace packages. Check log: ${build_log}"
    tail -n 60 "${build_log}" || true
    return 1
  fi

  log_ok "Gazebo workspace packages built"
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
  return 0
}

require_gazebo_runtime() {
  local missing_packages=()
  local required_packages=(
    "ros_gz_sim"
    "ros_gz_bridge"
    "gz_ros2_control"
    "controller_manager"
    "joint_state_broadcaster"
    "joint_trajectory_controller"
    "xacro"
    "robot_state_publisher"
  )

  for package_name in "${required_packages[@]}"; do
    if ! has_ros_package "${package_name}"; then
      missing_packages+=("${package_name}")
    fi
  done

  if ! command -v gz >/dev/null 2>&1; then
    missing_packages+=("gz")
  fi

  if (( ${#missing_packages[@]} > 0 )); then
    log_fail "Gazebo runtime dependencies are missing: ${missing_packages[*]}"
    echo "[HINT] Install them with:"
    echo "       sudo apt update && sudo apt install -y \\"
    echo "         ros-jazzy-ros-gz-sim \\"
    echo "         ros-jazzy-ros-gz \\"
    echo "         ros-jazzy-ros-gz-bridge \\"
    echo "         ros-jazzy-gz-ros2-control \\"
    echo "         ros-jazzy-ros2-control \\"
    echo "         ros-jazzy-ros2-controllers \\"
    echo "         ros-jazzy-joint-state-broadcaster \\"
    echo "         ros-jazzy-joint-trajectory-controller \\"
    echo "         ros-jazzy-xacro"
    return 1
  fi

  return 0
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    if [[ "${KEEP_LAUNCH}" == "true" ]]; then
      echo "[INFO] KEEP_LAUNCH=true, keeping phase6_sim_gazebo.launch.py running (pid=${LAUNCH_PID})."
    else
      echo "[INFO] stopping phase6_sim_gazebo.launch.py (pid=${LAUNCH_PID})"
      kill "${LAUNCH_PID}" 2>/dev/null || true
    fi
  fi
}
trap cleanup EXIT INT TERM

cd "${PROJECT_ROOT}"

log_step "loading VM ROS2 environment"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

log_step "checking Gazebo runtime dependencies"
if ! require_gazebo_runtime; then
  exit 1
fi
log_ok "Gazebo runtime dependencies are available"

missing_workspace_packages=()
for package_name in tactile_interfaces tactile_control tactile_task tactile_ui_bridge tactile_sim tactile_bringup; do
  if ! has_ros_package "${package_name}"; then
    missing_workspace_packages+=("${package_name}")
  fi
done

if (( ${#missing_workspace_packages[@]} > 0 )); then
  echo "[WARN] missing workspace packages: ${missing_workspace_packages[*]}"
  if ! auto_build_gazebo_packages; then
    exit 1
  fi
fi

if ! gazebo_workspace_assets_ready; then
  echo "[WARN] Gazebo workspace assets are missing or stale; rebuilding workspace packages."
  if ! auto_build_gazebo_packages; then
    exit 1
  fi
fi

log_step "cleaning stale Gazebo ROS2 processes"
cleanup_stale_gazebo_processes

log_step "reloading VM ROS2 environment after cleanup"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"
ros2 daemon start >/dev/null 2>&1 || true

mkdir -p "${LAUNCH_LOG_DIR}"
log_step "starting phase6_sim_gazebo.launch.py in background"
launch_args=(
  "start_gui:=${START_GUI}"
  "bridge_clock:=${BRIDGE_CLOCK}"
  "use_sim_time:=${USE_SIM_TIME}"
  "world_name:=${GAZEBO_WORLD_NAME}"
  "spawn_x:=${GAZEBO_SPAWN_X}"
  "spawn_y:=${GAZEBO_SPAWN_Y}"
  "spawn_z:=${GAZEBO_SPAWN_Z}"
)
ros2 launch tactile_bringup phase6_sim_gazebo.launch.py "${launch_args[@]}" >"${LAUNCH_LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"
sleep 6
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  log_fail "phase6_sim_gazebo.launch.py exited early. Check log: ${LAUNCH_LOG_FILE}"
  tail -n 80 "${LAUNCH_LOG_FILE}" >&2 || true
  exit 1
fi
log_ok "phase6_sim_gazebo.launch.py running (pid=${LAUNCH_PID})"
echo "[INFO] launch log: ${LAUNCH_LOG_FILE}"

if launch_log_has_dds_interface_mismatch; then
  report_dds_interface_mismatch
  dump_runtime_diagnostics
  exit 1
fi

log_step "validating Gazebo-side core nodes"
for node in \
  "/robot_state_publisher" \
  "/tactile_sim_node" \
  "/arm_sim_driver_node" \
  "/arm_control_node" \
  "/demo_task_node" \
  "/tactile_ui_subscriber"; do
  if ! wait_for_node "${node}" "${LAUNCH_TIMEOUT_SEC}"; then
    if launch_log_has_dds_interface_mismatch; then
      report_dds_interface_mismatch
    fi
    log_fail "required Gazebo node missing: ${node}"
    dump_runtime_diagnostics
    exit 1
  fi
done
log_ok "Gazebo core nodes are online"

log_step "validating Gazebo topics and services"
for topic in "/joint_states" "/arm/state" "/tactile/raw" "/system/health"; do
  if ! wait_for_topic "${topic}" "${LAUNCH_TIMEOUT_SEC}"; then
    log_fail "required topic missing: ${topic}"
    dump_runtime_diagnostics
    exit 1
  fi
done
if is_true "${BRIDGE_CLOCK}"; then
  if ! wait_for_topic "/clock" "${LAUNCH_TIMEOUT_SEC}"; then
    log_fail "clock bridge requested but /clock was not detected"
    dump_runtime_diagnostics
    exit 1
  fi
fi
for svc in \
  "/controller_manager/list_controllers" \
  "/arm/enable" \
  "/arm/home" \
  "/arm/move_joint" \
  "/arm/move_joints" \
  "/control/arm/enable" \
  "/control/arm/home" \
  "/control/arm/move_joint" \
  "/control/arm/move_joints"; do
  if ! wait_for_service "${svc}" "${LAUNCH_TIMEOUT_SEC}"; then
    log_fail "required service missing: ${svc}"
    dump_runtime_diagnostics
    exit 1
  fi
done
if ! wait_for_action "/joint_trajectory_controller/follow_joint_trajectory" "${LAUNCH_TIMEOUT_SEC}"; then
  log_fail "joint trajectory action is not available"
  dump_runtime_diagnostics
  exit 1
fi
log_ok "Gazebo topics, services, and controllers are online"

log_step "validating tactile simulation stream"
tactile_hz="$(sample_hz "/tactile/raw" "${TACTILE_HZ_SAMPLE_SEC}")"
if [[ -z "${tactile_hz}" ]]; then
  log_fail "failed to sample /tactile/raw rate"
  dump_runtime_diagnostics
  exit 1
fi
echo "[INFO] tactile average rate: ${tactile_hz} Hz"
if ! check_rate_ge "${tactile_hz}" "${MIN_TACTILE_HZ}"; then
  log_fail "tactile rate ${tactile_hz}Hz is below minimum ${MIN_TACTILE_HZ}Hz"
  dump_runtime_diagnostics
  exit 1
fi
log_ok "tactile simulation stream is healthy"

log_step "enabling simulated arm backend before launching UI"
arm_enable_output="$(call_setbool_service "/control/arm/enable" "true" "${ARM_TIMEOUT_SEC}" || true)"
if ! echo "${arm_enable_output}" | tr '[:upper:]' '[:lower:]' | grep -Eq 'success[[:space:]]*[:=][[:space:]]*true'; then
  log_fail "failed to enable simulated arm backend via /control/arm/enable"
  if [[ -n "${arm_enable_output}" ]]; then
    echo "${arm_enable_output}" >&2
  fi
  dump_runtime_diagnostics
  exit 1
fi
if ! wait_for_arm_connected_state "${ARM_TIMEOUT_SEC}"; then
  log_fail "/arm/state did not report connected=true within ${ARM_TIMEOUT_SEC}s"
  echo "[DIAG] latest /arm/state sample:" >&2
  timeout 3s ros2 topic echo /arm/state --once >&2 || true
  dump_runtime_diagnostics
  exit 1
fi
log_ok "simulated arm backend is enabled and connected"

log_step "starting UI in ROS2 Gazebo mode"
if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV}"
  log_ok "conda env activated: ${CONDA_ENV}"
else
  echo "[WARN] conda init script not found, using current python environment."
fi

# Conda activation can overwrite ROS-related env; source again.
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env_ros2_vm.sh" "${DOMAIN_ID}"

echo "[INFO] Gazebo UI profile: start_gui=${START_GUI} bridge_clock=${BRIDGE_CLOCK} use_sim_time=${USE_SIM_TIME} spawn=(${GAZEBO_SPAWN_X},${GAZEBO_SPAWN_Y},${GAZEBO_SPAWN_Z})"
echo "[READY] all Gazebo guards passed, launching UI..."
python "${PROJECT_ROOT}/main_ros2.py" \
  --control-mode ros2 \
  --vision-enabled false \
  --show-vision-ui "${SHOW_VISION_UI}" \
  --show-simulation-ui "${SHOW_SIMULATION_UI}" \
  --log-level INFO
