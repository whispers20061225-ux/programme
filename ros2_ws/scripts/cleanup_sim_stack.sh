#!/usr/bin/env bash
set -euo pipefail

patterns=(
  "gz sim -r"
  "gz sim -g"
  "gz sim -s"
  "gz sim server"
  "gz sim gui"
  "ruby /opt/ros/.*/gz sim"
  "parameter_bridge"
  "move_group"
  "rviz2"
  "robot_state_publisher"
  "controller_manager/spawner"
  "sim_realsense_adapter_node"
  "sim_target_pose_node"
  "sim_search_sweep_node"
  "sim_pick_task_node"
  "arm_sim_driver_node"
  "arm_control_node"
  "tactile_sim_node"
  "demo_task_node"
  "tactile_ui_subscriber"
  "spawn_dofbot"
)

echo "[cleanup] stopping stale simulation stack processes"

for pattern in "${patterns[@]}"; do
  pkill -f "$pattern" 2>/dev/null || true
done

sleep 1

for pattern in "${patterns[@]}"; do
  pkill -9 -f "$pattern" 2>/dev/null || true
done

echo "[cleanup] remaining matching processes:"
ps -eo pid,ppid,cmd | egrep 'gz sim|parameter_bridge|move_group|rviz2|robot_state_publisher|spawner|sim_realsense_adapter_node|sim_target_pose_node|sim_search_sweep_node|sim_pick_task_node|arm_sim_driver_node|arm_control_node|tactile_sim_node|demo_task_node|tactile_ui_subscriber|spawn_dofbot' | egrep -v 'egrep' || true
