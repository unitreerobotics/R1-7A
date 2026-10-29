"""Shared real-hardware CLI options for teleoperation and replay."""
import argparse
import os
from pathlib import Path
from robot.r1arm_sdk import DEFAULT_BRIDGE_LIBRARY, DEFAULT_SDK_ROOT, parse_gain

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-mode", action="store_true", help="Actually open and command the arm")
    parser.add_argument(
        "--port",
        default=os.environ.get("PORT", ""),
        help="R1Arm RS485 serial device",
    )
    parser.add_argument("--baud", type=int, default=6_000_000)
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=DEFAULT_SDK_ROOT,
        help="Used to derive the default calibration path",
    )
    parser.add_argument("--calib", type=Path, help="joint calibration file")
    parser.add_argument("--bridge-lib", type=Path, default=DEFAULT_BRIDGE_LIBRARY)
    parser.add_argument("--urdf", type=Path, help="Simulation/IK URDF (same model as the working sim)")
    parser.add_argument("--side", choices=["left", "right"], default="right")
    parser.add_argument("--scale-factor", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=90.0,
                        help="控制环 Hz。默认 90；XR 独立接收，图像采样频率单独配置")
    parser.add_argument("--trajectory-position-gain", type=float, default=64.0,
                        help="Baseline q_purpose position-error response (s^-2)")
    parser.add_argument("--trajectory-velocity-gain", type=float, default=16.0,
                        help="Baseline q_purpose velocity damping (s^-1)")
    parser.add_argument(
        "--ik-dt", type=float, default=0.0,
        help="Placo velocity-limit timestep s; 0 = 1/--rate. With current Placo "
             "joint-velocity limits disabled, changing this does not scale raw pose-IK steps",
    )
    parser.add_argument(
        "--gravity-ff", dest="gravity_ff", action="store_true", default=True,
        help="下发重力前馈力矩（指令姿态处计算，clip 后经 v2 tau 通道）；"
             "默认开（P0 三重实测：J4 悬垂误差 −70%%）。--no-gravity-ff 关闭",
    )
    parser.add_argument(
        "--no-gravity-ff", dest="gravity_ff", action="store_false",
        help="关闭重力前馈（回到 kp 硬扛悬垂的旧行为）",
    )
    parser.add_argument(
        "--tau-clip", type=float, default=8.0,
        help="重力前馈单关节力矩限幅 Nm（默认 8）",
    )
    parser.add_argument("--kp", type=parse_gain, default=60.0,
                        help="位置环增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--kd", type=parse_gain, default=10.0,
                        help="阻尼增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--max-joint-speed", type=float, default=1.0,
                        help="SDK command speed limit (rad/s; default 1.0)")
    parser.add_argument(
        "--trajectory-max-speed", type=float, default=None,
        help="q_purpose shaper speed limit (rad/s; defaults to --max-joint-speed)",
    )
    parser.add_argument(
        "--max-command-lead", type=float, default=0.15,
        help="命令相对实测的目标最大超前量 rad（默认 0.15）；与位置步长冲突时优先步长，不能作为总力矩上界",
    )
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--foc-timeout", type=float, default=1.0)
    parser.add_argument("--enable-dex1", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dex1-net", default=os.environ.get("DEX1_NETWORK", ""), help="DDS interface for the one real Dex1")
    parser.add_argument("--dex1-side", choices=("left", "right"), default=os.environ.get("DEX1_SIDE", "right"),
                        help="DDS side topic for the physical Dex1; independent of the arm side")
    parser.add_argument("--dex1-domain", type=int, default=0)
    parser.add_argument("--grip-torque-limit", type=float, default=-1.0)
    parser.add_argument(
        "--enable-a-reset", action=argparse.BooleanOptionalAction, default=True,
        help="Allow A to move toward ARM_INIT",
    )
    parser.add_argument(
        "--home-on-start",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="After startup FOC hold, soft-blend to ARM_INIT before enabling tracking",
    )
    parser.add_argument(
        "--reset-blend-frames",
        type=int,
        default=60,
        help="A/home reset 时长（控制帧数；blend 按 wall-clock frames/50Hz 计秒，"
             "默认 60 ≈ 1.2s；想要旧 2s 体验传 100）",
    )
    parser.add_argument("--mock-xr", action="store_true", help="Dry-run only; rejected with --motion-mode")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--gripper-log", type=Path, help="Write per-cycle trigger/DDS diagnostics to a new CSV")
    parser.add_argument("--no-confirm", action="store_true")
    return parser
