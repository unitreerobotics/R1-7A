#!/usr/bin/env python3
"""R1 单臂 / 双臂 MuJoCo 仿真遥操入口。

复用 A7 的 PICO 相对位姿跟踪 + Placo/DLS IK，输出到 MuJoCo。

Usage:
    # mock XR（不需要 PICO）
    python scripts/teleop_r1arm_sim.py --mock-xr

    # 连接 PICO，右手柄 grip 激活跟踪
    python scripts/teleop_r1arm_sim.py

    # 双臂场景：左右手柄分别控制左右臂（无需再传 --side）
    python scripts/teleop_r1arm_sim.py --use-dual-arm

    # 无 viewer（仅跑物理步进，适合无显示环境）
    python scripts/teleop_r1arm_sim.py --mock-xr --no-viewer
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.r1arm_config import DEFAULT_DUAL_MJCF_PATH, R1ArmConfig
from teleop.r1arm_mujoco_controller import R1ArmMujocoTeleopController


def main():
    parser = argparse.ArgumentParser(description="R1 Arm MuJoCo Simulation Teleoperation")
    parser.add_argument("--urdf", type=str, help="URDF path (IK / Pinocchio)")
    parser.add_argument("--mjcf", type=str, help="MuJoCo MJCF/scene path")
    parser.add_argument(
        "--use-dual-arm",
        action="store_true",
        help="双臂场景 + 左右手柄分别控制（忽略 --side）",
    )
    parser.add_argument(
        "--side",
        choices=["left", "right"],
        default="right",
        help="单臂模式：哪只手柄控制（--use-dual-arm 时忽略）",
    )
    parser.add_argument("--scale-factor", type=float, default=1.0, help="手柄位移缩放")
    parser.add_argument("--mock-xr", action="store_true", help="无 PICO，用 mock 手柄")
    parser.add_argument("--debug", action="store_true", help="调试打印")
    parser.add_argument("--rate", type=float, default=50.0, help="控制环频率 Hz")
    parser.add_argument("--viewer", action="store_true", default=True, help="打开 MuJoCo viewer")
    parser.add_argument("--no-viewer", action="store_true", help="关闭 viewer")
    parser.add_argument("--sim-substeps", type=int, default=10, help="每控制步推进的仿真子步数")
    parser.add_argument("--reset-blend-frames", type=int, default=100, help="A 键回 home 平滑处理")
    args = parser.parse_args()

    config = R1ArmConfig(
        scale_factor=args.scale_factor,
        dual_arm=args.use_dual_arm,
        control_side=args.side,
        reset_blend_frames=args.reset_blend_frames,
    )
    if args.urdf:
        config.urdf_path = args.urdf
    if args.mjcf:
        config.mjcf_path = args.mjcf
    elif args.use_dual_arm:
        config.mjcf_path = str(DEFAULT_DUAL_MJCF_PATH)

    if args.use_dual_arm and args.side != "right":
        print("Note: --use-dual-arm enables both handlers; --side is ignored.")

    controller = R1ArmMujocoTeleopController(
        config=config,
        mock_xr=args.mock_xr,
        debug=args.debug,
        scale_factor=args.scale_factor,
        show_viewer=not args.no_viewer,
        sim_substeps=args.sim_substeps,
    )
    controller.run(rate=args.rate)


if __name__ == "__main__":
    main()
