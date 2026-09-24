#!/usr/bin/env python3
"""把仿真机械臂冻结在 home（ARM_INIT）姿态，作为真机标定的参考姿态。

不跑控制环、不推进物理步进，手臂完全静止，方便对着它摆真机。

Usage:
    # 打开 viewer，冻结在 ARM_INIT
    python scripts/view_r1arm_home.py

    # 只看角度、不开窗口（打参数用）
    python scripts/view_r1arm_home.py --no-viewer

    # 看任意姿态
    python scripts/view_r1arm_home.py --q 0,1.2,0,-1.2,0,0,0
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.mujoco_sim import MujocoArmSim
from robot.r1arm_config import ARM_INIT, ARM_JOINTS, DEX1_JOINTS, R1ArmConfig
from robot.r1arm_sdk import MODEL_TO_SDK_SIGN


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--q", type=str, help="逗号分隔的 7 个臂关节角（仿真坐标），覆盖 home")
    parser.add_argument("--mjcf", type=str, help="MuJoCo MJCF/scene 路径")
    parser.add_argument("--no-viewer", action="store_true", help="不开窗口，只打印角度")
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="自动退出时间；0 表示一直显示到 Ctrl+C",
    )
    args = parser.parse_args()

    config = R1ArmConfig()
    if args.mjcf:
        config.mjcf_path = args.mjcf

    arm_q = ARM_INIT.copy()
    if args.q:
        values = [float(v) for v in args.q.split(",")]
        if len(values) != len(ARM_JOINTS):
            parser.error(f"--q 需要 {len(ARM_JOINTS)} 个数，收到 {len(values)} 个")
        arm_q = np.asarray(values, dtype=float)

    dex1_q = np.full(len(DEX1_JOINTS), float(config.dex1_open), dtype=float)
    q = np.hstack([arm_q, dex1_q])
    q_sdk = arm_q * MODEL_TO_SDK_SIGN + 0.0  # +0.0 把 -0.0 归一成 0.0

    print("仿真姿态（不推进物理，手臂静止）")
    print("  MJCF:", config.mjcf_path)
    print()
    print(f"  {'joint':<6}{'仿真坐标':>12}{'':>4}{'SDK 坐标':>12}")
    for i, name in enumerate(ARM_JOINTS):
        print(
            f"  {name:<6}{arm_q[i]:>+12.4f}{'':>4}{q_sdk[i]:>+12.4f}"
            f"    ({np.degrees(arm_q[i]):+7.2f}°)"
        )
    print(f"  {'dex1':<6}{float(config.dex1_open):>+12.4f}{'':>4}{'--':>12}    (张开)")
    print()
    print("标定回读时，test_arm_lowlevel 的 q 应等于上面 SDK 坐标那一列：")
    print("  " + " ".join(f"{v:+.4f}" for v in q_sdk))
    print()
    print("对应 calibrate_joints 的参数：")
    print("  --reference-q " + ",".join(f"{v:g}" for v in q_sdk))
    print()

    if args.no_viewer:
        return

    sim = MujocoArmSim(
        config.mjcf_path,
        q_init=q,
        show_viewer=True,
        joint_names=ARM_JOINTS + DEX1_JOINTS,
    )
    print("窗口已打开，姿态已冻结。对着它把真机摆成一样，然后 Ctrl+C 退出。")

    deadline = time.monotonic() + args.hold_seconds if args.hold_seconds > 0 else None
    try:
        while sim.viewer is not None:
            if deadline is not None and time.monotonic() >= deadline:
                break
            sim.sync()
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        sim.close()

    # MuJoCo/GLFW 在解释器 teardown 阶段销毁 GL 上下文时会段错误（close() 本身
    # 已经正常返回）。这里主动 _exit 跳过 teardown，避免退出时刷一个 core dump。
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
