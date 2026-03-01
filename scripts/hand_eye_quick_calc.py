#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼简化 + 已知坐标抓取工具（关节5为相机/夹爪相对旋转）

功能：
1) 根据测量的相机->夹取点平移向量 + 关节5角度，计算变换矩阵
2) 给定相机坐标系下的目标点，换算到机械臂基座坐标系并执行抓取

适用前提：
- 相机固定在机械臂末端附近
- 相机与夹爪之间只有关节5引入相对旋转
- 平移向量是固定的（测量得到），旋转由关节5角度决定
"""

import argparse
import math
import os
import sys
import time
from typing import List, Optional, Tuple

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    from config.demo_config import DemoConfig
    from arm_control.learm_interface import LearmInterface
    from arm_control.joint_controller import JointController
    from arm_control.cartesian_controller import CartesianController, Pose
except Exception as exc:
    raise RuntimeError(f"无法导入项目模块，请确认在项目根目录运行：{exc}")


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _normalize(v: List[float]) -> List[float]:
    norm = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if norm <= 1e-9:
        raise ValueError("旋转轴向量长度为0")
    return [v[0] / norm, v[1] / norm, v[2] / norm]


def _rot_axis(axis: List[float], angle_deg: float) -> List[List[float]]:
    """轴角 -> 旋转矩阵（罗德里格斯公式）。"""
    ax = _normalize(axis)
    x, y, z = ax
    theta = _deg2rad(angle_deg)
    c = math.cos(theta)
    s = math.sin(theta)
    t = 1.0 - c

    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def _rot_zyx(roll_deg: float, pitch_deg: float, yaw_deg: float) -> List[List[float]]:
    """ZYX欧拉角（先yaw再pitch再roll）转旋转矩阵。"""
    cr = math.cos(_deg2rad(roll_deg))
    sr = math.sin(_deg2rad(roll_deg))
    cp = math.cos(_deg2rad(pitch_deg))
    sp = math.sin(_deg2rad(pitch_deg))
    cy = math.cos(_deg2rad(yaw_deg))
    sy = math.sin(_deg2rad(yaw_deg))

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    """3x3 矩阵乘法。"""
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
    return out


def _mat_vec_mul(a: List[List[float]], v: List[float]) -> List[float]:
    """3x3 矩阵乘向量。"""
    return [
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ]


def _transpose(a: List[List[float]]) -> List[List[float]]:
    return [
        [a[0][0], a[1][0], a[2][0]],
        [a[0][1], a[1][1], a[2][1]],
        [a[0][2], a[1][2], a[2][2]],
    ]


def _build_mat4(r: List[List[float]], t: List[float]) -> List[List[float]]:
    return [
        [r[0][0], r[0][1], r[0][2], t[0]],
        [r[1][0], r[1][1], r[1][2], t[1]],
        [r[2][0], r[2][1], r[2][2], t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat4_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = (
                a[i][0] * b[0][j]
                + a[i][1] * b[1][j]
                + a[i][2] * b[2][j]
                + a[i][3] * b[3][j]
            )
    return out


def _mat4_vec(a: List[List[float]], v: List[float]) -> List[float]:
    return [
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2] + a[0][3] * v[3],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2] + a[1][3] * v[3],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2] + a[2][3] * v[3],
        a[3][0] * v[0] + a[3][1] * v[1] + a[3][2] * v[2] + a[3][3] * v[3],
    ]


def _pose_to_mat4(pose: Pose) -> List[List[float]]:
    """将末端位姿（mm + deg）转为 4x4 变换矩阵。"""
    roll = _deg2rad(pose.roll)
    pitch = _deg2rad(pose.pitch)
    yaw = _deg2rad(pose.yaw)

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    r = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    return _build_mat4(r, [pose.x, pose.y, pose.z])


def _fmt_mat4(m: List[List[float]]) -> str:
    lines = []
    for row in m:
        lines.append("[" + ", ".join(f"{v: .6f}" for v in row) + "]")
    return "\n".join(lines)


def _parse_triplet(values: Optional[List[float]], name: str) -> Optional[Tuple[float, float, float]]:
    if values is None:
        return None
    if len(values) != 3:
        raise ValueError(f"{name} 需要3个数值")
    return float(values[0]), float(values[1]), float(values[2])


def _scale_to_mm(value: float, unit: str) -> float:
    unit = unit.lower()
    if unit == "mm":
        return value
    if unit == "cm":
        return value * 10.0
    if unit == "m":
        return value * 1000.0
    raise ValueError("单位仅支持 mm/cm/m")


def _connect_arm(port: str, baud: int, timeout: float) -> Tuple[LearmInterface, JointController, CartesianController]:
    config = DemoConfig()
    try:
        config.enable_arm_integration()
    except Exception:
        pass

    # 写入连接参数
    if isinstance(getattr(config, "learm_arm", None), dict):
        conn = config.learm_arm.get("CONNECTION", {})
        conn["serial_port"] = port
        conn["baud_rate"] = baud
        conn["timeout"] = timeout
        config.learm_arm["CONNECTION"] = conn

    arm = LearmInterface(config)
    if not arm.connect(port=port):
        raise RuntimeError("机械臂连接失败，请确认串口占用/权限")

    joint = JointController(arm)
    cart = CartesianController(joint)
    return arm, joint, cart


def main() -> None:
    parser = argparse.ArgumentParser(description="手眼简化变换 + 已知坐标抓取")
    # 你提供的测量值（默认值）
    default_t = [0.0, 4.8, 12.0]  # cm
    parser.add_argument(
        "--t",
        type=float,
        nargs=3,
        default=default_t,
        help="相机->夹取点平移向量 (tx ty tz)",
    )
    parser.add_argument("--t-unit", type=str, default="cm", help="平移向量单位: mm/cm/m")
    parser.add_argument("--joint5", type=float, help="关节5角度（度）")
    parser.add_argument("--joint5-index", type=int, default=4, help="关节5索引(0-based), 默认4")
    parser.add_argument("--axis", type=float, nargs=3, default=[0.0, 0.0, 1.0],
                        help="关节5旋转轴（相机坐标系），默认 0 0 1")
    parser.add_argument("--rpy", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help="初始旋转偏置 roll pitch yaw（度）")
    parser.add_argument(
        "--rotate-translation",
        action="store_true",
        help="让平移向量随关节旋转（用于平移定义在旋转坐标系的情况）",
    )
    parser.add_argument(
        "--use-joint-rotation",
        action="store_true",
        help="启用关节5旋转叠加于相机-夹爪姿态",
    )
    parser.add_argument(
        "--offset-is-gripper",
        action="store_true",
        help="--t 是夹爪->相机的偏移（夹爪坐标系）",
    )
    parser.add_argument("--read-joint5", action="store_true",
                        help="从机械臂读取关节5角度（优先使用该值）")
    parser.add_argument("--port", type=str, default="COM4", help="机械臂串口")
    parser.add_argument("--baud", type=int, default=115200, help="机械臂波特率")
    parser.add_argument("--timeout", type=float, default=1.0, help="串口超时")

    # 抓取相关
    parser.add_argument("--object", type=float, nargs=3,
                        help="目标物体坐标（相机坐标系）x y z")
    parser.add_argument("--object-unit", type=str, default="cm", help="目标坐标单位: mm/cm/m")
    parser.add_argument("--approach", type=float, default=50.0, help="预抓取高度偏置(mm)")
    parser.add_argument("--speed", type=float, default=0.3, help="机械臂速度系数(0~1)")
    parser.add_argument("--do-grasp", action="store_true", help="执行抓取动作")

    args = parser.parse_args()

    t = _parse_triplet(args.t, "t")
    t_mm = [_scale_to_mm(v, args.t_unit) for v in t]

    arm = None
    joint = None
    cart = None
    try:
        if args.read_joint5 or args.do_grasp:
            arm, joint, cart = _connect_arm(args.port, args.baud, args.timeout)

        if args.read_joint5:
            angles = joint.get_current_angles(update=True)
            idx = int(args.joint5_index)
            if idx < 0 or idx >= len(angles):
                raise ValueError("关节5索引越界")
            args.joint5 = float(angles[idx])

        if args.joint5 is None:
            raise ValueError("请提供 --joint5 或使用 --read-joint5")

        # 初始旋转偏置（joint5=0 时的相机->夹爪姿态）
        r_offset = _rot_zyx(args.rpy[0], args.rpy[1], args.rpy[2])
        r_joint = _rot_axis(args.axis, float(args.joint5)) if args.use_joint_rotation else [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        # ??->????
        r_cam_gripper = _mat_mul(r_joint, r_offset) if args.use_joint_rotation else r_offset

        if args.offset_is_gripper:
            # ???????->????????????
            t_gripper_cam = list(t_mm)
            if args.rotate_translation and args.use_joint_rotation:
                t_gripper_cam = _mat_vec_mul(r_joint, t_gripper_cam)
        else:
            # ???????->????????????
            # ?????????t_GC = -R_GC * t_CG
            t_gripper_cam = _mat_vec_mul(
                r_cam_gripper,
                [-t_mm[0], -t_mm[1], -t_mm[2]],
            )

        t_gripper_cam = _build_mat4(r_cam_gripper, t_gripper_cam)
        r_inv = _transpose(r_cam_gripper)
        t_cam_gripper = _mat_vec_mul(
            r_inv,
            [-t_gripper_cam[0][3], -t_gripper_cam[1][3], -t_gripper_cam[2][3]],
        )
        t_cam_gripper = _build_mat4(r_inv, t_cam_gripper)

        print("\n相机 -> 夹爪 变换矩阵 (mm):")
        print(_fmt_mat4(t_cam_gripper))
        print("\n夹爪 -> 相机 变换矩阵 (mm):")
        print(_fmt_mat4(t_gripper_cam))

        if args.object:
            obj = _parse_triplet(args.object, "object")
            obj_mm = [_scale_to_mm(v, args.object_unit) for v in obj]

            if cart is None:
                raise RuntimeError("需要连接机械臂以获取当前姿态")

            current_pose = cart.get_current_pose(update=True)
            t_base_gripper = _pose_to_mat4(current_pose)
            t_base_cam = _mat4_mul(t_base_gripper, t_gripper_cam)

            obj_base = _mat4_vec(t_base_cam, [obj_mm[0], obj_mm[1], obj_mm[2], 1.0])
            target_x = float(obj_base[0])
            target_y = float(obj_base[1])
            target_z = float(obj_base[2])

            print(f"\n目标点(基座坐标, mm): x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}")

            if args.do_grasp:
                # 预抓取位（抬高）
                pre_pose = Pose(
                    target_x,
                    target_y,
                    target_z + float(args.approach),
                    current_pose.roll,
                    current_pose.pitch,
                    current_pose.yaw,
                )
                ok = cart.move_to_pose(pre_pose, speed=args.speed, wait=True)
                if not ok:
                    raise RuntimeError("预抓取位移动失败")

                # 下探到抓取点
                grasp_pose = Pose(
                    target_x,
                    target_y,
                    target_z,
                    current_pose.roll,
                    current_pose.pitch,
                    current_pose.yaw,
                )
                ok = cart.move_to_pose(grasp_pose, speed=args.speed, wait=True)
                if not ok:
                    raise RuntimeError("抓取位移动失败")

                # 简单等待（此处可接入夹爪闭合指令）
                time.sleep(0.2)

                # 抬升
                lift_pose = Pose(
                    target_x,
                    target_y,
                    target_z + float(args.approach),
                    current_pose.roll,
                    current_pose.pitch,
                    current_pose.yaw,
                )
                cart.move_to_pose(lift_pose, speed=args.speed, wait=True)

                print("抓取动作完成（夹爪闭合请在此处接入控制逻辑）")

    finally:
        if arm is not None:
            arm.disconnect()


if __name__ == "__main__":
    main()
