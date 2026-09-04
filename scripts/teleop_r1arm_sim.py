#!/usr/bin/env python3
"""R1 单臂 MuJoCo 仿真遥操入口。

复用 A7 的 PICO 相对位姿跟踪 + Placo/DLS IK，输出到 MuJoCo。

Usage:
    # mock XR（不需要 PICO）
    python scripts/teleop_r1arm_sim.py --mock-xr

    # 连接 PICO，右手柄 grip 激活跟踪
    python scripts/teleop_r1arm_sim.py

    # 无 viewer（仅跑物理步进，适合无显示环境）
    python scripts/teleop_r1arm_sim.py --mock-xr --no-viewer
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.r1arm_config import R1ArmConfig
from teleop.r1arm_mujoco_controller import R1ArmMujocoTeleopController


def main():
    parser = argparse.ArgumentParser(description="R1 Arm MuJoCo Simulation Teleoperation")
    parser.add_argument("--urdf", type=str, help="URDF path (IK / Pinocchio)")
    parser.add_argument("--mjcf", type=str, help="MuJoCo MJCF/scene path")
    parser.add_argument("--side", choices=["left", "right"], default="right", help="哪只手柄控制")
    parser.add_argument("--scale-factor", type=float, default=1.0, help="手柄位移缩放")
    parser.add_argument("--mock-xr", action="store_true", help="无 PICO，用 mock 手柄")
    parser.add_argument("--debug", action="store_true", help="调试打印")
    parser.add_argument("--rate", type=float, default=50.0, help="控制环频率 Hz")
    parser.add_argument("--viewer", action="store_true", default=True, help="打开 MuJoCo viewer")
    parser.add_argument("--no-viewer", action="store_true", help="关闭 viewer")
    parser.add_argument("--sim-substeps", type=int, default=10, help="每控制步推进的仿真子步数")
    parser.add_argument(
        "--no-align-frame",
        action="store_true",
        help="关闭 A7→r1arm 基座坐标对齐（调试用）",
    )
    args = parser.parse_args()

    config = R1ArmConfig(
        scale_factor=args.scale_factor,
        control_side=args.side,
        align_teleop_frame=not args.no_align_frame,
    )
    if args.urdf:
        config.urdf_path = args.urdf
    if args.mjcf:
        config.mjcf_path = args.mjcf

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
