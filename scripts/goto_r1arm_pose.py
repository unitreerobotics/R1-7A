#!/usr/bin/env python3
"""把真机送到指定姿态（SDK 坐标），带运动前预检。

执行阶段与 scripts/teleop_r1arm_real.py 用同一套位置控制：mode=1 FOC 位置环，
每周期对目标依次做硬限位、速度限制、实测位置超前量限制，然后下发 position 命令。

分三步：
  1. 只读预检 —— 零力矩探测，读当前姿态，算出每个关节要动多少
  2. 安全检查 —— 幅度过大或超出限位就拒绝执行
  3. 加 --move 才真的运动（还要在终端手输 MOVE）

默认目标是仿真 ARM_INIT 对应的 SDK 姿态 [0,1.2,0,1.2,0,0,0]。

⚠️ 真机是零力矩/低增益状态，全程托住机械臂。
⚠️ 运动到位后（--hold-ms 到点）会失去保持并下坠。

Usage:
    # 只预检，不动（安全，先跑这个）
    python scripts/goto_r1arm_pose.py

    # 预检通过后真的运动
    python scripts/goto_r1arm_pose.py --move

    # 到位后保持 60 秒方便观察
    python scripts/goto_r1arm_pose.py --move --hold-ms 60000
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

from robot.r1arm_config import ARM_JOINTS, ARM_LIMITS
from robot.r1arm_sdk import (
    DEFAULT_BRIDGE_LIBRARY,
    DEFAULT_SDK_ROOT,
    R1ArmSdkBus,
    R1ArmSdkState,
    model_to_sdk_q,
)

ARM_INIT_SDK = "0,1.2,0,1.2,0,0,0"

# SDK 坐标系下的关节限位（由仿真 URDF 限位经 J4 反号得到）
_LOWER_MODEL = np.array([ARM_LIMITS[n][0] for n in ARM_JOINTS], dtype=float)
_UPPER_MODEL = np.array([ARM_LIMITS[n][1] for n in ARM_JOINTS], dtype=float)
_SDK_A = model_to_sdk_q(_LOWER_MODEL)
_SDK_B = model_to_sdk_q(_UPPER_MODEL)
# +0.0 把 -0.0 归一成 0.0，否则 J4 下限会打印成 "-0.0000"
SDK_LOWER = np.minimum(_SDK_A, _SDK_B) + 0.0
SDK_UPPER = np.maximum(_SDK_A, _SDK_B) + 0.0


def parse_q(text: str) -> np.ndarray:
    values = [float(v) for v in text.split(",")]
    if len(values) != len(ARM_JOINTS):
        raise argparse.ArgumentTypeError(
            f"需要 {len(ARM_JOINTS)} 个逗号分隔的值，收到 {len(values)} 个"
        )
    return np.asarray(values, dtype=float)


def limit_command(
    target: np.ndarray,
    previous: np.ndarray,
    measured: np.ndarray,
    *,
    max_delta: float,
    max_lead: float,
) -> np.ndarray:
    """硬限位 + 每周期速度限制 + 实测位置超前量限制。

    与 teleop/r1arm_real_controller.py 的 limit_sdk_position_command 逐行一致；
    复制一份是为了不把 Placo IK 等重依赖拉进这个一次性验证脚本。
    """
    target = np.asarray(target, dtype=float).reshape(7)
    previous = np.asarray(previous, dtype=float).reshape(7)
    measured = np.asarray(measured, dtype=float).reshape(7)
    if not all(np.all(np.isfinite(v)) for v in (target, previous, measured)):
        raise RuntimeError("Non-finite R1Arm command/feedback")
    bounded = np.clip(target, SDK_LOWER, SDK_UPPER)
    slewed = previous + np.clip(bounded - previous, -abs(max_delta), abs(max_delta))
    return measured + np.clip(slewed - measured, -abs(max_lead), abs(max_lead))


def probe_feedback(bus: R1ArmSdkBus, rate: float, timeout: float) -> R1ArmSdkState:
    """零力矩探测，拿到完整反馈（零增益 FOC，机械臂不保持姿态）。"""
    print("零力矩探测（机械臂不保持姿态）...")
    deadline = time.monotonic() + timeout
    state: R1ArmSdkState | None = None
    while time.monotonic() < deadline:
        bus.probe_once()
        time.sleep(1.0 / rate)
        state = bus.read_state()
        if np.all(state.ready):
            faults = np.flatnonzero(state.error_code != 0)
            if faults.size:
                raise RuntimeError(
                    "电机故障: "
                    + ", ".join(f"J{i + 1}=0x{int(state.error_code[i]):x}" for i in faults)
                )
            print("7 轴反馈就绪。")
            return state
    missing = [] if state is None else np.flatnonzero(~state.ready).tolist()
    raise TimeoutError(f"{timeout:.1f}s 内未收到完整反馈；缺失关节 {missing}")


def _check_feedback(state: R1ArmSdkState, require_foc: bool) -> None:
    if not np.all(state.ready):
        missing = np.flatnonzero(~state.ready).tolist()
        raise RuntimeError(f"反馈不完整，缺失关节 {missing}")
    faults = np.flatnonzero(state.error_code != 0)
    if faults.size:
        raise RuntimeError(
            "电机故障: "
            + ", ".join(f"J{i + 1}=0x{int(state.error_code[i]):x}" for i in faults)
        )
    if require_foc:
        stopped = np.flatnonzero(state.mode != 1)
        if stopped.size:
            raise RuntimeError("掉出 FOC: " + ", ".join(f"J{i + 1}" for i in stopped))


def _print_progress(state: R1ArmSdkState, target: np.ndarray) -> None:
    print(f"  {'joint':<6}{'当前 q':>10}{'目标 q':>10}{'误差':>9}{'tau':>9}")
    for i, name in enumerate(ARM_JOINTS):
        err = target[i] - state.q[i]
        print(
            f"  {name:<6}{state.q[i]:>+10.4f}{target[i]:>+10.4f}"
            f"{err:>+9.4f}{state.tau_est[i]:>+9.3f}"
        )
    print()


def run_position_goto(
    bus: R1ArmSdkBus,
    target: np.ndarray,
    *,
    kp: float,
    kd: float,
    rate: float,
    max_joint_speed: float,
    max_lead: float,
    hold_ms: int,
    done_rad: float,
    timeout_ms: float,
) -> None:
    """FOC 原位保持 → 位置控制爬行到 target → 保持。失败/中断由调用方停机。"""
    max_delta = max_joint_speed / rate
    period = 1.0 / rate

    # Phase 1: 零增益 FOC 已在探测时建立，这里把增益提上来做原位保持（目标=实测，
    # 误差≈0，无启动跳变），确认 7 轴都在 FOC。
    print("Phase 1: FOC 使能，原位保持...")
    state = probe_feedback(bus, rate, 3.0)
    _check_feedback(state, require_foc=False)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        bus.command_position(state.q, kp, kd)
        time.sleep(period)
        state = bus.read_state()
        if np.all(state.mode == 1):
            break
    else:
        raise TimeoutError("FOC 使能超时")
    _check_feedback(state, require_foc=True)
    print("  7 轴已进入 FOC，开始爬行。")

    # Phase 2: 位置控制爬行。力矩上限 = kp * max_lead，速度上限 = max_joint_speed，
    # 两个旋钮独立，不会出现"力矩够了但冲太快"的问题。
    command = state.q.copy()
    deadline = time.monotonic() + timeout_ms / 1000.0
    last_print = 0.0
    while time.monotonic() < deadline:
        state = bus.read_state()
        _check_feedback(state, require_foc=True)
        command = limit_command(
            target, command, state.q, max_delta=max_delta, max_lead=max_lead
        )
        bus.command_position(command, kp, kd)
        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            _print_progress(state, target)
        if np.all(np.abs(target - state.q) < done_rad):
            break
        time.sleep(period)
    else:
        raise TimeoutError(f"爬行超时（{timeout_ms / 1000:.0f}s），可能增益不足或标定不符")
    print("  到达目标。")

    # Phase 3: 保持（观察用）
    if hold_ms > 0:
        print(f"Phase 3: 保持 {hold_ms / 1000:.1f}s（Ctrl+C 提前停）...")
        deadline = time.monotonic() + hold_ms / 1000.0
        last_print = 0.0
        while time.monotonic() < deadline:
            bus.command_position(target, kp, kd)
            state = bus.read_state()
            _check_feedback(state, require_foc=True)
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                _print_progress(state, target)
            time.sleep(period)


def _safe_stop(bus: R1ArmSdkBus, rate: float) -> None:
    try:
        if bus.is_open:
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                bus.stop()
                time.sleep(1.0 / rate)
    except Exception as exc:
        print(f"警告：停机命令发送失败（{exc}），关串口后电机会自行超时。", file=sys.stderr)
    finally:
        bus.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--q", type=parse_q, default=parse_q(ARM_INIT_SDK),
                        help=f"目标姿态，SDK 坐标（默认 {ARM_INIT_SDK}）")
    parser.add_argument("--port", default=os.environ.get(
        "PORT", "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTANPO49-if02-port0"))
    parser.add_argument("--baud", type=int, default=6_000_000)
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--calib", type=Path)
    parser.add_argument("--bridge-lib", type=Path, default=DEFAULT_BRIDGE_LIBRARY)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--max-delta", type=float, default=1.5,
                        help="单个关节允许的最大移动量 rad（默认 1.5 ≈ 86°）")
    parser.add_argument("--allow-large", action="store_true",
                        help="跳过 --max-delta 检查（危险）")
    # 默认对齐 scripts/teleop_r1arm_real.py 的增益，让初始摆位跟后面的摇操一致：
    #   kp=40, max-command-lead=0.15  →  力矩上限 kp*lead = 6 Nm
    # 这个 SDK 里没有独立的"力矩模式"：mode 只有 0(stop)/1(FOC 位置环)，力矩是
    # 位置环的输出 tau = kp * clamp(q_target - q_meas, ±lead)，离目标越近越小。
    # 所以力矩的唯一旋钮就是 kp * --max-command-lead 这个乘积，别只调 kp 不看 lead。
    parser.add_argument("--kp", type=float, default=40.0,
                        help="位置环增益 Nm/rad（默认 40，同摇操）")
    parser.add_argument("--kd", type=float, default=0.5,
                        help="阻尼增益 Nm/(rad/s)（默认 0.5，同摇操）")
    parser.add_argument("--max-joint-speed", type=float, default=0.5,
                        help="目标爬行速度上限 rad/s（默认 0.5，同摇操）")
    parser.add_argument("--max-command-lead", type=float, default=0.15,
                        help="命令相对实测的最大超前量 rad；力矩上限 = kp × 此值（默认 6 Nm）")
    parser.add_argument("--done-rad", type=float, default=0.07,
                        help="判定到位的半径 rad")
    parser.add_argument("--goto-timeout", type=float, default=30.0,
                        help="爬行超时 s")
    parser.add_argument("--hold-ms", type=int, default=0,
                        help="到位后保持毫秒数；0 = 立即松开（会下坠）")
    parser.add_argument("--move", action="store_true", help="真的执行运动")
    args = parser.parse_args()

    calib = args.calib or args.sdk_root / "config/joint_calib_candidate.txt"
    target = args.q

    for label, path in (("标定文件", calib), ("桥接库", args.bridge_lib)):
        if not Path(path).exists():
            print(f"错误：{label}不存在: {path}", file=sys.stderr)
            return 1

    # ---- 1. 只读预检 ------------------------------------------------------
    bus = R1ArmSdkBus(args.bridge_lib)
    try:
        bus.open(args.port, args.baud, calib)
        current = probe_feedback(bus, args.rate, args.feedback_timeout).q.astype(float)
    except Exception as exc:
        print(f"预检失败：{exc}", file=sys.stderr)
        return 1
    finally:
        _safe_stop(bus, args.rate)

    delta = target - current

    # ---- 2. 打印计划 ------------------------------------------------------
    print()
    print(f"标定文件: {calib}")
    print(f"串口    : {args.port} @ {args.baud}")
    print()
    print(f"  {'joint':<6}{'当前 q_sdk':>12}{'目标 q_sdk':>12}{'Δ(rad)':>11}{'Δ(°)':>10}")
    for i, name in enumerate(ARM_JOINTS):
        flag = "  <<<" if abs(delta[i]) > args.max_delta else ""
        print(f"  {name:<6}{current[i]:>+12.4f}{target[i]:>+12.4f}"
              f"{delta[i]:>+11.4f}{np.degrees(delta[i]):>+10.2f}{flag}")
    print()
    print(f"  最大单关节移动: {np.abs(delta).max():.4f} rad "
          f"({np.degrees(np.abs(delta).max()):.1f}°)")
    print(f"  位置控制      : kp={args.kp} kd={args.kd} "
          f"速度≤{args.max_joint_speed} rad/s 超前≤{args.max_command_lead} rad "
          f"→ 力矩上限 {args.kp * args.max_command_lead:.1f} Nm")

    # ---- 3. 安全检查 ------------------------------------------------------
    problems = []

    over = np.flatnonzero(np.abs(delta) > args.max_delta)
    if over.size and not args.allow_large:
        detail = ", ".join(
            f"{ARM_JOINTS[i]}({np.degrees(abs(delta[i])):.0f}°)" for i in over
        )
        problems.append(
            f"移动量超过 --max-delta={args.max_delta} rad：{detail}\n"
            f"    这通常说明标定文件跟真机不匹配（当前姿态读数不对）。\n"
            f"    确认无误再加 --allow-large，或换一份标定。"
        )

    oob_lo = np.flatnonzero(target < SDK_LOWER - 1e-6)
    oob_hi = np.flatnonzero(target > SDK_UPPER + 1e-6)
    if oob_lo.size or oob_hi.size:
        idx = sorted(set(oob_lo.tolist()) | set(oob_hi.tolist()))
        detail = ", ".join(
            f"{ARM_JOINTS[i]}(目标{target[i]:+.3f} ∉ [{SDK_LOWER[i]:+.3f},{SDK_UPPER[i]:+.3f}])"
            for i in idx
        )
        problems.append(f"目标超出 SDK 关节限位：{detail}")

    if problems:
        print()
        for p in problems:
            print(f"  ✗ {p}")
        print("\n拒绝执行。")
        return 1

    print("\n  ✓ 安全检查通过")

    if not args.move:
        print("\n这是预检模式（没有 --move），未执行任何运动。")
        print("确认上面的数字合理后，执行：")
        print()
        print("  python scripts/goto_r1arm_pose.py --move")
        if args.hold_ms:
            print(f"    （到位后保持 {args.hold_ms} ms）")
        print()
        return 0

    # ---- 4. 执行 ----------------------------------------------------------
    print()
    print(
        f"即将用位置控制爬行到目标（kp={args.kp} kd={args.kd} "
        f"速度≤{args.max_joint_speed} rad/s 超前≤{args.max_command_lead} rad，"
        f"力矩上限 {args.kp * args.max_command_lead:.1f} Nm）"
    )
    print("  ⚠️ 全程托住机械臂；急停放在手边。")
    print("  ⚠️ 到位后（--hold-ms 到点）机械臂会失去保持并下坠。")
    if input("  输入 MOVE 继续：").strip() != "MOVE":
        print("已取消，未发送任何运动命令。")
        return 1

    print()
    bus = R1ArmSdkBus(args.bridge_lib)
    try:
        bus.open(args.port, args.baud, calib)
        run_position_goto(
            bus,
            target,
            kp=args.kp,
            kd=args.kd,
            rate=args.rate,
            max_joint_speed=args.max_joint_speed,
            max_lead=args.max_command_lead,
            hold_ms=args.hold_ms,
            done_rad=args.done_rad,
            timeout_ms=args.goto_timeout * 1000.0,
        )
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停机。")
    except Exception as exc:
        print(f"\n运动中止：{exc}", file=sys.stderr)
        return 1
    finally:
        _safe_stop(bus, args.rate)

    print("完成。机械臂已停机（mode 0），请托住。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
