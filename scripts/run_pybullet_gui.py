#!/usr/bin/env python3
"""
独立启动 PyBullet GUI（不依赖主界面仿真控件）。

用法：
  python3 scripts/run_pybullet_gui.py

说明：
- 会读取 config/simulation_config.py 的默认配置。
- 自动强制 ENGINE.mode = 'gui'。
- 关闭 PyBullet 窗口或 Ctrl+C 即可退出。
- 键盘视角控制：←/→ 旋转，↑/↓ 俯仰，WASD 平移，R/F 上下，Q/E 缩放。
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

# 确保项目根目录在 Python 路径中，避免找不到本地包
project_root = Path(__file__).resolve().parent.parent
src_root = project_root / "src"
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

import pybullet as p
from src.simulation.simulator import Simulator
from config.simulation_config import SimulationConfig


def _apply_keyboard_camera(sim: Simulator) -> None:
    """使用键盘控制视角（箭头旋转，WASD 平移，R/F 上下，Q/E 缩放）"""
    if sim.client_id is None:
        return

    events = p.getKeyboardEvents(physicsClientId=sim.client_id)
    if not events:
        return

    cam = p.getDebugVisualizerCamera(physicsClientId=sim.client_id)
    cam_up = cam[4]
    cam_forward = cam[5]
    yaw = float(cam[8])
    pitch = float(cam[9])
    dist = float(cam[10])
    target = [float(v) for v in cam[11]]

    changed = False
    rot_step = 2.0
    zoom_step = max(0.05, dist * 0.05)
    pan_step = max(0.01, dist * 0.02)

    def key_down(key_code: int) -> bool:
        return (events.get(key_code, 0) & p.KEY_IS_DOWN) != 0

    if key_down(p.B3G_LEFT_ARROW):
        yaw -= rot_step
        changed = True
    if key_down(p.B3G_RIGHT_ARROW):
        yaw += rot_step
        changed = True
    if key_down(p.B3G_UP_ARROW):
        pitch = max(-89.0, min(89.0, pitch + rot_step))
        changed = True
    if key_down(p.B3G_DOWN_ARROW):
        pitch = max(-89.0, min(89.0, pitch - rot_step))
        changed = True

    # 使用 PyBullet 提供的 forward/up 方向，保证 WASD 与视角一致
    forward = [float(v) for v in cam_forward]
    up = [float(v) for v in cam_up]
    right = [
        forward[1] * up[2] - forward[2] * up[1],
        forward[2] * up[0] - forward[0] * up[2],
        forward[0] * up[1] - forward[1] * up[0],
    ]
    right_len = math.sqrt(right[0] ** 2 + right[1] ** 2 + right[2] ** 2) or 1.0
    right = [v / right_len for v in right]
    f_len = math.sqrt(forward[0] ** 2 + forward[1] ** 2 + forward[2] ** 2) or 1.0
    forward = [v / f_len for v in forward]

    if key_down(ord("w")) or key_down(ord("W")):
        target[0] += forward[0] * pan_step
        target[1] += forward[1] * pan_step
        target[2] += forward[2] * pan_step
        changed = True
    if key_down(ord("s")) or key_down(ord("S")):
        target[0] -= forward[0] * pan_step
        target[1] -= forward[1] * pan_step
        target[2] -= forward[2] * pan_step
        changed = True
    if key_down(ord("a")) or key_down(ord("A")):
        target[0] -= right[0] * pan_step
        target[1] -= right[1] * pan_step
        target[2] -= right[2] * pan_step
        changed = True
    if key_down(ord("d")) or key_down(ord("D")):
        target[0] += right[0] * pan_step
        target[1] += right[1] * pan_step
        target[2] += right[2] * pan_step
        changed = True
    if key_down(ord("r")) or key_down(ord("R")):
        target[2] += pan_step
        changed = True
    if key_down(ord("f")) or key_down(ord("F")):
        target[2] -= pan_step
        changed = True

    if key_down(ord("q")) or key_down(ord("Q")):
        dist = max(0.2, dist - zoom_step)
        changed = True
    if key_down(ord("e")) or key_down(ord("E")):
        dist = min(10.0, dist + zoom_step)
        changed = True

    if changed:
        p.resetDebugVisualizerCamera(
            cameraDistance=dist,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=target,
            physicsClientId=sim.client_id,
        )


def main() -> None:
    config = SimulationConfig()
    config.ENGINE["mode"] = "gui"
    # 由脚本主动 step，避免实时模式下看不到自定义关节驱动
    config.ENGINE["real_time_simulation"] = False
    sim = Simulator(config)
    sim.start()
    # 禁用 PyBullet 默认快捷键，避免 W 切换视图等行为
    try:
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0, physicsClientId=sim.client_id)
    except Exception:
        pass
    try:
        while sim.is_connected():
            sim.step()
            _apply_keyboard_camera(sim)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()


if __name__ == "__main__":
    main()
