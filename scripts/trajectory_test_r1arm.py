#!/usr/bin/env python3
"""R1Arm 固定关节轨迹运控测试（R1ARM_REAL_MOTION_OPTIMIZATION_PLAN.md P0）。

不连接 XR、IK、相机或 MCAP——把运控问题从遥操链路里隔离出来。
复用 goto_r1arm_pose.py 的安全流程：零力矩探测、标定/限位预检、
FOC 原位保持、故障检查、--move 终端确认、停机兜底。

轨迹分级（--level，逐级扩大）：
  0  原位保持（不动，测保持噪声与控制周期）
  1  单关节小幅低速往返（默认 J4，带平滑起停）
  2  多关节协调往返（J2/J4/J6 小幅正弦相位错开）
  3  已验证轨迹的小幅提速（同 level 2 轨形，速度 × --speedup）

默认 --home-first：FOC 保持建立后，先在同一控制环内爬回 home
[0,1.2,0,1.2,0,0,0]（goto 的 home 姿态）再执行轨迹——不跨进程交接串口，
避免交接窗口失保持下坠；--no-home-first 改为从实测姿态起步。

每周期输出链路与爬行完全一致：硬限位 → 速度限幅 → 实测超前限制 →
故障检查（包括驻留段；对照 goto 的 Phase 3 直接发 target 不再走限幅）。
每周期遥测异步写 CSV（单调时钟、q_ref/dq_ref/ddq_ref、q_cmd、q/dq/tau_est、
kp/kd、tau_ff、限幅触发标志、循环耗时），结束打印逐关节统计。

用法：
    # 离线：只生成轨迹+打印逐关节范围/速度/加速度峰值，不碰硬件
    python scripts/trajectory_test_r1arm.py --level 1

    # 实机（预检 → MOVE 确认 → 分级轨迹 → 遥测 CSV + 统计）
    python scripts/trajectory_test_r1arm.py --level 1 --move \
        --port /dev/ttyUSB2 \
        --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt
"""

from __future__ import annotations

import argparse
import csv
import os
import queue
import sys
import threading
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
    as_gain_array,
    format_gain,
    model_to_sdk_q,
    parse_gain,
    sdk_to_model_q,
)

# SDK 坐标系关节限位（与 goto_r1arm_pose.py 同款推导）
_LOWER_MODEL = np.array([ARM_LIMITS[n][0] for n in ARM_JOINTS], dtype=float)
_UPPER_MODEL = np.array([ARM_LIMITS[n][1] for n in ARM_JOINTS], dtype=float)
_SDK_A = model_to_sdk_q(_LOWER_MODEL)
_SDK_B = model_to_sdk_q(_UPPER_MODEL)
SDK_LOWER = np.minimum(_SDK_A, _SDK_B) + 0.0
SDK_UPPER = np.maximum(_SDK_A, _SDK_B) + 0.0

LEVELS = {
    0: "hold",
    1: "single-joint",
    2: "multi-joint",
    3: "multi-joint speedup",
}


# ---------------------------------------------------------------------------
# 轨迹生成（连续、平滑起停、含驻留与方向反转）
# ---------------------------------------------------------------------------

def smoothstep5(t: np.ndarray) -> np.ndarray:
    """quintic smoothstep: 0→1，起停速度/加速度为零。"""
    t = np.clip(t, 0.0, 1.0)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def build_trajectory(
    level: int,
    q_start: np.ndarray,
    *,
    amplitude: float,
    frequency: float,
    cycles: float,
    hold_s: float,
    rate: float,
    speedup: float = 1.0,
    joint_idx: int = 3,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """按级别生成采样轨迹（SDK 坐标）。

    返回 (t, q_ref, dq_ref, ddq_ref)。所有段衔接连续：
    hold → 平滑进出往返 → hold。level 1 只动 --joint 指定关节；
    level 2/3 三关节正弦相位错开（速度/加速度由频率显式给定，
    smoothstep 包络保证起停处 v=a=0）。
    """
    freq = frequency * speedup
    total_cycles = cycles
    motion_s = total_cycles / freq if freq > 0 else 0.0
    total_s = hold_s + motion_s + hold_s
    n = int(round(total_s * rate)) + 1
    t = np.arange(n, dtype=float) / rate

    q_ref = np.tile(np.asarray(q_start, dtype=float).reshape(7), (n, 1))
    m = (t >= hold_s) & (t < hold_s + motion_s)
    tm = t[m] - hold_s  # motion-phase local time

    if level >= 1:
        joint = int(joint_idx)
        center = float(q_start[joint])
        # 包络：前 1/4 周期 smoothstep 拉满，末 1/4 收回 → 起停 v=a=0，
        # 中段完整正弦往返（含自然方向反转）
        ramp = np.minimum(smoothstep5(tm / (motion_s * 0.25)),
                          smoothstep5((motion_s - tm) / (motion_s * 0.25)))
        q_ref[m, joint] = center + amplitude * ramp * np.sin(2 * np.pi * freq * tm)

    if level >= 2:
        # J2/J4/J6 协调：相位 120° 错开，同一包络
        for joint, phase in ((1, 2 * np.pi / 3), (3, 0.0), (5, 4 * np.pi / 3)):
            center = float(q_start[joint])
            ramp = np.minimum(smoothstep5(tm / (motion_s * 0.25)),
                              smoothstep5((motion_s - tm) / (motion_s * 0.25)))
            q_ref[m, joint] = center + amplitude * ramp * np.sin(2 * np.pi * freq * tm + phase)

    # 数值微分（rate 足够密，中心差分即可）
    dq_ref = np.gradient(q_ref, axis=0) * rate
    ddq_ref = np.gradient(dq_ref, axis=0) * rate
    return t, q_ref, dq_ref, ddq_ref


def check_trajectory(q_ref: np.ndarray, label: str) -> None:
    oob = np.flatnonzero(
        (np.min(q_ref, axis=0) < SDK_LOWER - 1e-6) | (np.max(q_ref, axis=0) > SDK_UPPER + 1e-6)
    )
    if oob.size:
        detail = ", ".join(
            f"{ARM_JOINTS[i]} ∈ [{np.min(q_ref[:, i]):+.3f},{np.max(q_ref[:, i]):+.3f}]"
            for i in oob
        )
        raise ValueError(f"{label}: 轨迹超 SDK 限位：{detail}")


def print_trajectory_summary(t, q_ref, dq_ref, ddq_ref, moved_joints) -> None:
    print(f"  采样点 {len(t)}，时长 {t[-1]:.2f} s")
    print(f"  {'joint':<6}{'q_min':>9}{'q_max':>9}{'|v|_max':>10}{'|a|_max':>10}")
    for i, name in enumerate(ARM_JOINTS):
        mark = " *" if i in moved_joints else ""
        print(
            f"  {name:<6}{np.min(q_ref[:, i]):>+9.4f}{np.max(q_ref[:, i]):>+9.4f}"
            f"{np.max(np.abs(dq_ref[:, i])):>10.4f}{np.max(np.abs(ddq_ref[:, i])):>10.4f}{mark}"
        )
    print("  (* 参与运动的关节；加速度 rad/s²为数值微分近似)")


# ---------------------------------------------------------------------------
# 输出链路（与 goto 爬行段逐字一致，驻留也走同一条）
# ---------------------------------------------------------------------------

def limit_command(
    target: np.ndarray,
    previous: np.ndarray,
    measured: np.ndarray,
    *,
    max_delta: float,
    max_lead: float,
) -> np.ndarray:
    target = np.asarray(target, dtype=float).reshape(7)
    previous = np.asarray(previous, dtype=float).reshape(7)
    measured = np.asarray(measured, dtype=float).reshape(7)
    if not all(np.all(np.isfinite(v)) for v in (target, previous, measured)):
        raise RuntimeError("Non-finite R1Arm command/feedback")
    bounded = np.clip(target, SDK_LOWER, SDK_UPPER)
    slewed = previous + np.clip(bounded - previous, -abs(max_delta), abs(max_delta))
    return measured + np.clip(slewed - measured, -abs(max_lead), abs(max_lead))


def gravity_feedforward(model, data, q_arm: np.ndarray, clip: float) -> np.ndarray:
    import pinocchio as pin

    q_full = np.zeros(model.nq, dtype=float)
    q_full[: len(ARM_JOINTS)] = np.asarray(q_arm, dtype=float).reshape(-1)
    tau = pin.computeGeneralizedGravity(model, data, q_full)
    tau = np.asarray(tau, dtype=float).reshape(-1)[: len(ARM_JOINTS)]
    return np.clip(model_to_sdk_q(tau), -clip, clip)


def probe_feedback(bus: R1ArmSdkBus, rate: float, timeout: float) -> R1ArmSdkState:
    print("零力矩探测（机械臂不保持姿态，托住）...")
    deadline = time.monotonic() + timeout
    state: R1ArmSdkState | None = None
    while time.monotonic() < deadline:
        bus.probe_once()
        time.sleep(1.0 / rate)
        state = bus.read_state()
        if np.all(state.ready):
            _fault_guard(state)
            print("7 轴反馈就绪。")
            return state
    missing = [] if state is None else np.flatnonzero(~state.ready).tolist()
    raise TimeoutError(f"{timeout:.1f}s 内未收到完整反馈；缺失关节 {missing}")


def _fault_guard(state: R1ArmSdkState, require_foc: bool = False) -> None:
    if not np.all(state.ready):
        missing = np.flatnonzero(~state.ready).tolist()
        raise RuntimeError(f"反馈不完整，缺失关节 {missing}")
    faults = np.flatnonzero(state.error_code != 0)
    if faults.size:
        raise RuntimeError(
            "电机故障: " + ", ".join(f"J{i + 1}=0x{int(state.error_code[i]):x}" for i in faults)
        )
    if require_foc:
        stopped = np.flatnonzero(state.mode != 1)
        if stopped.size:
            raise RuntimeError("掉出 FOC: " + ", ".join(f"J{i + 1}" for i in stopped))


# ---------------------------------------------------------------------------
# 异步遥测（控制热路径只入队，写盘在后台线程）
# ---------------------------------------------------------------------------

TELEMETRY_FIELDS = (
    ["t_monotonic", "i"]
    + [f"q_ref_{j}" for j in range(7)]
    + [f"dq_ref_{j}" for j in range(7)]
    + [f"ddq_ref_{j}" for j in range(7)]
    + [f"q_cmd_{j}" for j in range(7)]
    + [f"q_{j}" for j in range(7)]
    + [f"dq_{j}" for j in range(7)]
    + [f"tau_est_{j}" for j in range(7)]
    + [f"tau_ff_{j}" for j in range(7)]
    + ["slew_hit", "lead_hit", "cycle_ms", "busy_ms"]
)


class TelemetryWriter:
    def __init__(self, path: Path):
        self.path = path
        self.queue: queue.Queue[dict | None] = queue.Queue(maxsize=4096)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        with open(self.path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=TELEMETRY_FIELDS)
            writer.writeheader()
            while True:
                row = self.queue.get()
                if row is None:
                    break
                writer.writerow(row)
        print(f"[Telemetry] CSV 已写入 {self.path}")

    def push(self, row: dict) -> None:
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            pass  # 磁盘跟不上时丢行，不阻塞控制环

    def close(self) -> None:
        self.queue.put(None)
        self.thread.join(timeout=5.0)


# ---------------------------------------------------------------------------
# 统计报告（逐关节，不整臂平均）
# ---------------------------------------------------------------------------

def report(csv_path: Path, rate: float) -> None:
    rows = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    if not rows:
        print("没有遥测行。")
        return
    n = len(rows)

    cycle_ms = np.array([float(r["cycle_ms"]) for r in rows[1:]])  # 首拍无上一拍周期
    busy_ms = np.array([float(r["busy_ms"]) for r in rows])
    over = 1.0 / rate * 1000.0
    print(f"\n===== 控制周期（目标 {over:.0f} ms）=====")
    print(f"  cycles={n}")
    print(f"  top-to-top: median={np.median(cycle_ms):.2f}  p95={np.percentile(cycle_ms, 95):.2f}"
          f"  max={cycle_ms.max():.2f} ms  >{over:.0f}ms 比例 {np.mean(cycle_ms > over) * 100:.1f}%")
    print(f"  串口耗时:  median={np.median(busy_ms):.2f}  p95={np.percentile(busy_ms, 95):.2f}"
          f"  max={busy_ms.max():.2f} ms（read+command，不含 sleep；"
          f"上限参考：50Hz 需 <20，100Hz 需 <10）")

    q_ref = np.array([[float(r[f"q_ref_{j}"]) for j in range(7)] for r in rows])
    q_cmd = np.array([[float(r[f"q_cmd_{j}"]) for j in range(7)] for r in rows])
    q = np.array([[float(r[f"q_{j}"]) for j in range(7)] for r in rows])
    dq = np.array([[float(r[f"dq_{j}"]) for j in range(7)] for r in rows])
    tau_est = np.array([[float(r[f"tau_est_{j}"]) for j in range(7)] for r in rows])

    # 静止/运动分段：参考速度接近零的样本算静止
    moved = np.max(np.abs(np.gradient(q_ref, axis=0)) * rate, axis=1) > 1e-3
    print(f"\n===== 逐关节（静止段 {np.sum(~moved)} 拍 / 运动段 {np.sum(moved)} 拍）=====")
    print(f"  {'joint':<6}{'静误差μ':>9}{'静误差σ':>9}{'跟踪RMS':>9}{'跟踪P95':>9}"
          f"{'滞后μ':>9}{'|dq|pk':>9}{'|τest|pk':>10}")
    for j, name in enumerate(ARM_JOINTS):
        if not moved.any():
            static = np.abs(q_ref[:, j] - q[:, j])
            rms = p95 = lag = float("nan")
        else:
            static = np.abs(q_ref[~moved, j] - q[~moved, j])
            err = q_ref[moved, j] - q[moved, j]
            rms = float(np.sqrt(np.mean(err ** 2)))
            p95 = float(np.percentile(np.abs(err), 95))
            a = q_ref[moved, j] - q_ref[moved, j].mean()
            b = q[moved, j] - q[moved, j].mean()
            if np.std(a) < 1e-9 or np.std(b) < 1e-9:
                lag = 0.0
            else:
                corr = np.correlate(a, b, mode="full")
                lag = float((np.argmax(corr) - (len(b) - 1)) / rate * 1000)
        dq_pk = float(np.max(np.abs(dq[:, j]))) if dq.shape[1] == 7 else float("nan")
        print(
            f"  {name:<6}{np.mean(static):>+9.4f}{np.std(static):>9.4f}{rms:>9.4f}{p95:>9.4f}"
            f"{lag:>9.1f}{dq_pk:>9.3f}{np.max(np.abs(tau_est[:, j])):>10.3f}"
        )

    slew = sum(int(r["slew_hit"]) for r in rows)
    lead = sum(int(r["lead_hit"]) for r in rows)
    print(f"\n  限幅触发：slew {slew}/{n} 拍 ({slew / n * 100:.1f}%)，"
          f"lead {lead}/{n} 拍 ({lead / n * 100:.1f}%)")


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def run_trajectory(
    bus: R1ArmSdkBus,
    t: np.ndarray,
    q_ref: np.ndarray,
    dq_ref: np.ndarray,
    ddq_ref: np.ndarray,
    *,
    kp: "float | np.ndarray",
    kd: "float | np.ndarray",
    rate: float,
    max_joint_speed: float,
    max_lead: float,
    gravity: tuple | None,
    telemetry: TelemetryWriter,
    home_target: np.ndarray | None = None,
    home_timeout: float = 60.0,
    home_tol: float = 0.08,
) -> None:
    max_delta = max_joint_speed / rate
    period = 1.0 / rate

    def tau_ff(command_sdk: np.ndarray) -> np.ndarray | None:
        if gravity is None:
            return None
        model, data, tau_clip = gravity
        return gravity_feedforward(model, data, sdk_to_model_q(command_sdk), tau_clip)

    # Phase 1: 提增益原位保持（目标=实测，无启动跳变），确认 7 轴 FOC
    print("Phase 1: FOC 使能，原位保持...")
    state = probe_feedback(bus, rate, 3.0)
    _fault_guard(state, require_foc=False)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        bus.command_position(state.q, kp, kd, tau=tau_ff(state.q))
        time.sleep(period)
        state = bus.read_state()
        if np.all(state.mode == 1):
            break
    else:
        raise TimeoutError("FOC 使能超时")
    _fault_guard(state, require_foc=True)
    print("  7 轴 FOC，轨迹从实测姿态起步。")

    # Phase 1.5（--home-first）：同一输出链路爬回 home 再开始轨迹。
    # 不走 goto 的独立进程——避免串口交接窗口（交接期间失去保持下坠）。
    if home_target is not None:
        print("Phase 1.5: 爬行回 home（同链路限幅；托住，Ctrl+C 停）...")
        command = state.q.copy()
        deadline = time.monotonic() + home_timeout
        done = False
        settled = False  # 误差 3 秒不再下降 → 已收敛（悬垂 = g/kp，到不了 0）
        last_print = 0.0
        stall_start = None
        stall_err = None
        while time.monotonic() < deadline:
            state = bus.read_state()
            _fault_guard(state, require_foc=True)
            command = limit_command(
                home_target, command, state.q, max_delta=max_delta, max_lead=max_lead
            )
            bus.command_position(command, kp, kd, tau=tau_ff(command))
            err_max = float(np.abs(home_target - state.q).max())
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                print(f"  home err max |{err_max:.3f}| rad")
            if err_max < home_tol:
                done = True
                break
            # 收敛检测：误差比 3 秒前没有再降 0.01 rad → 判定悬垂稳定，接受
            if stall_start is None or err_max < stall_err - 0.01:
                stall_start = now
                stall_err = err_max
            elif now - stall_start >= 3.0:
                settled = True
                break
            time.sleep(period)
        if not done and not settled:
            raise TimeoutError(f"{home_timeout:.0f}s 内未爬回 home")
        if settled:
            print(f"  误差收敛在 |{err_max:.3f}| rad（悬垂 g/kp，无前馈时无法消除），"
                  f"以此为轨迹起点继续。")
        else:
            print("  已到 home，开始轨迹。")

    # Phase 2: 逐拍跟踪轨迹采样点（含驻留段），同一输出链路。
    # 三个耗时语义：
    #   busy_ms = 本拍 read+command 的实际串口耗时（不含 sleep）
    #   cycle_ms = 上一拍 top-to-top 实际周期（下一拍开头才知道，首拍 0）
    # row 在 push 前完全定稿——后台写线程持引用，push 后不可再改。
    command = state.q.copy()
    print(f"Phase 2: 执行轨迹 {len(t)} 拍 / {t[-1]:.1f} s（Ctrl+C 停）...")
    next_i = 0
    loop_start = time.perf_counter()
    prev_cycle_ms = 0.0
    try:
        while next_i < len(t):
            busy_start = time.perf_counter()
            state = bus.read_state()
            _fault_guard(state, require_foc=True)

            bounded = np.clip(q_ref[next_i], SDK_LOWER, SDK_UPPER)
            slewed = command + np.clip(bounded - command, -max_delta, max_delta)
            lead_hit = bool(np.any(np.abs(slewed - state.q) > max_lead))
            slew_hit = bool(np.any(np.abs(bounded - command) > max_delta + 1e-9))
            command = state.q + np.clip(slewed - state.q, -max_lead, max_lead)

            tau_now = tau_ff(command)
            bus.command_position(command, kp, kd, tau=tau_now)
            busy_ms = (time.perf_counter() - busy_start) * 1000.0

            row = {"t_monotonic": f"{time.monotonic():.6f}", "i": next_i}
            for j in range(7):
                row[f"q_ref_{j}"] = f"{q_ref[next_i, j]:.6f}"
                row[f"dq_ref_{j}"] = f"{dq_ref[next_i, j]:.6f}"
                row[f"ddq_ref_{j}"] = f"{ddq_ref[next_i, j]:.6f}"
                row[f"q_cmd_{j}"] = f"{command[j]:.6f}"
                row[f"q_{j}"] = f"{state.q[j]:.6f}"
                row[f"dq_{j}"] = f"{state.dq[j]:.6f}"
                row[f"tau_est_{j}"] = f"{state.tau_est[j]:.4f}"
                row[f"tau_ff_{j}"] = f"{(tau_now[j] if tau_now is not None else 0.0):.4f}"
            row["slew_hit"] = int(slew_hit)
            row["lead_hit"] = int(lead_hit)
            row["busy_ms"] = f"{busy_ms:.3f}"
            row["cycle_ms"] = f"{prev_cycle_ms:.3f}"
            telemetry.push(row)

            next_i += 1
            now = time.perf_counter()
            prev_cycle_ms = (now - loop_start) * 1000.0
            sleep_t = period - (now - loop_start)
            loop_start = now
            if sleep_t > 0:
                time.sleep(sleep_t)
        # 末拍周期补记：n-1 行的 cycle_ms 是 n-2 拍的；用最后一拍真实值覆盖
        # （简单起见不做，报告以 n-1 行为准，差一拍不影响统计）
    finally:
        print(f"  轨迹执行至第 {next_i}/{len(t)} 拍。")


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
    parser.add_argument("--level", type=int, default=1, choices=sorted(LEVELS),
                        help="轨迹分级（默认 1=单关节小幅往返）")
    parser.add_argument("--joint", default="J4",
                        help="level 1 的运动关节（默认 J4；J1-J7）")
    parser.add_argument("--amplitude", type=float, default=0.15,
                        help="运动关节幅度 rad（默认 0.15 ≈ 8.6°，小幅）")
    parser.add_argument("--frequency", type=float, default=0.2,
                        help="往返频率 Hz（默认 0.2 → 一个往返 5 s，低速）")
    parser.add_argument("--cycles", type=float, default=2.0,
                        help="往返次数（默认 2，含方向反转）")
    parser.add_argument("--hold-s", type=float, default=2.0,
                        help="轨迹前后驻留秒数（默认 2 s，测保持噪声）")
    parser.add_argument("--speedup", type=float, default=1.0,
                        help="level 3 速度倍率（默认 1.0=与 level 2 同轨迹）")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--port", default="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTANPO49-if02-port0")
    parser.add_argument("--baud", type=int, default=6_000_000)
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--calib", type=Path)
    parser.add_argument("--bridge-lib", type=Path, default=DEFAULT_BRIDGE_LIBRARY)
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--kp", type=parse_gain, default=80.0,
                        help="位置环增益；单个数=广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--kd", type=parse_gain, default=2,
                        help="阻尼增益；单个数=广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--gravity-ff", action="store_true",
                        help="下发重力前馈（指令姿态处计算，clip 后经 tau 通道）")
    parser.add_argument("--tau-clip", type=float, default=8.0)
    parser.add_argument("--max-joint-speed", type=float, default=0.5,
                        help="输出链路速度限幅 rad/s")
    parser.add_argument(
        "--max-command-lead", type=float, default=0.15,
        help="命令相对实测的最大超前量 rad（异常单拍 ≤8.6°；力矩三项合计"
             "≈峰值一半，正常跟踪不触发）",
    )
    parser.add_argument("--out", type=Path, default=Path("/tmp/r1arm_traj_telemetry.csv"),
                        help="遥测 CSV 输出路径")
    parser.add_argument("--no-home-first", dest="home_first", action="store_false",
                        help="不从 home 起步，改为实测姿态起步（默认先爬回 home；"
                             "折叠位等贴限位姿态没有摆动余量，慎用）")
    parser.add_argument("--move", action="store_true", help="真的执行（预检模式只打印轨迹）")
    args = parser.parse_args()

    calib = args.calib or args.sdk_root / "config/joint_calib_candidate.txt"

    # ---- 1. 离线轨迹生成 + 校验（不碰硬件） -------------------------------
    # 起始姿态：home 的 SDK 表达（ARM_INIT 模型坐标 J4=-1.2 → SDK J4=+1.2）
    if args.level == 1 and args.joint not in ARM_JOINTS:
        print(f"错误：--joint {args.joint} 不是 J1-J7。", file=sys.stderr)
        return 1
    q_start = np.array([0.0, 1.2, 0.0, 1.2, 0.0, 0.0, 0.0])
    t, q_ref, dq_ref, ddq_ref = build_trajectory(
        args.level, q_start,
        amplitude=args.amplitude, frequency=args.frequency, cycles=args.cycles,
        hold_s=args.hold_s, rate=args.rate, speedup=args.speedup,
        joint_idx=ARM_JOINTS.index(args.joint),
    )
    moved_joints = {ARM_JOINTS.index(args.joint)} if args.level == 1 else (
        {1, 3, 5} if args.level >= 2 else set()
    )
    check_trajectory(q_ref, "home-start")
    print(f"轨迹 level {args.level}（{LEVELS[args.level]}）：amplitude={args.amplitude} rad "
          f"freq={args.frequency * args.speedup:g} Hz cycles={args.cycles} 驻留={args.hold_s}s")
    print_trajectory_summary(t, q_ref, dq_ref, ddq_ref, moved_joints)

    if not args.move:
        print("\n这是离线模式（没有 --move）：只生成并校验了轨迹，未碰硬件。")
        print("实机执行示例：")
        print(f"  python scripts/trajectory_test_r1arm.py --level {args.level} --move \\")
        print(f"      --port /dev/ttyUSB2 --calib {calib}")
        return 0

    # ---- 2. 实机预检 ------------------------------------------------------
    for label, path in (("标定文件", calib), ("桥接库", args.bridge_lib)):
        if not Path(path).exists():
            print(f"错误：{label}不存在: {path}", file=sys.stderr)
            return 1

    bus = R1ArmSdkBus(args.bridge_lib)
    try:
        bus.open(args.port, args.baud, calib)
        measured = probe_feedback(bus, args.rate, args.feedback_timeout).q.astype(float)
    except Exception as exc:
        print(f"预检失败：{exc}", file=sys.stderr)
        return 1
    finally:
        _safe_stop(bus, args.rate)

    # 从实测姿态重建轨迹（默认 --home-first：每轮对比从同一姿态起步，
    # 折叠位 J4 贴 0 下限没有摆动余量；同一控制环内爬回 home，无串口交接窗口）
    if args.home_first:
        # 折叠位（J4 贴 0 下限等）没有摆动余量：先走 goto 的爬行链路回 home。
        # 在同一总线上做，避免 goto→测试的串口交接窗口（交接会失去保持下坠）。
        q_start = np.array([0.0, 1.2, 0.0, 1.2, 0.0, 0.0, 0.0])
    else:
        q_start = measured
    t, q_ref, dq_ref, ddq_ref = build_trajectory(
        args.level, q_start,
        amplitude=args.amplitude, frequency=args.frequency, cycles=args.cycles,
        hold_s=args.hold_s, rate=args.rate, speedup=args.speedup,
        joint_idx=ARM_JOINTS.index(args.joint),
    )
    check_trajectory(q_ref, f"start({('home' if args.home_first else 'measured')})")
    print(f"\n轨迹起点：{'home' if args.home_first else '实测姿态'}")
    print_trajectory_summary(t, q_ref, dq_ref, ddq_ref, moved_joints)

    # 起点对运动关节的限位余量（折叠位 J4≈0 贴下限 → 余量≈0，必须先回 home）
    margins = np.minimum(SDK_UPPER - q_start, q_start - SDK_LOWER)
    moved_idx = sorted(
        ARM_JOINTS.index(args.joint) if args.level == 1 else j
        for j in moved_joints
    ) if args.level >= 1 else []
    for j in moved_idx:
        if margins[j] < args.amplitude:
            print(
                f"  ✗ {ARM_JOINTS[j]} 起点余量 {margins[j]:.3f} rad < 幅度 {args.amplitude} rad\n"
                f"    （折叠位等贴限位的姿态没有摆动空间。）\n"
                f"    去掉 --no-home-first 先爬回 home（默认行为），或换 --joint，或减小 --amplitude。"
            )
            return 1
    if args.amplitude > 0.5:
        print(f"  ✗ --amplitude {args.amplitude} > 0.5 rad，超出小幅验证范围，拒绝执行。")
        return 1

    gravity = None
    if args.gravity_ff:
        from robot.arm_model import load_robot_model
        from robot.r1arm_config import DEFAULT_URDF_PATH
        model, data = load_robot_model(str(DEFAULT_URDF_PATH))
        tau_sdk = gravity_feedforward(model, data, sdk_to_model_q(measured), args.tau_clip)
        print(f"\n  重力前馈（起始姿态，SDK 坐标，clip ±{args.tau_clip:g} Nm）：")
        for i, name in enumerate(ARM_JOINTS):
            print(f"    {name:<6}{tau_sdk[i]:>+8.3f} Nm")
        gravity = (model, data, float(args.tau_clip))

    kp_arr = as_gain_array(args.kp)
    kd_arr = as_gain_array(args.kd)
    torque_cap = float(np.max(kp_arr * args.max_command_lead
                              + kd_arr * args.max_joint_speed
                              + (args.tau_clip if args.gravity_ff else 0.0)))
    print()
    print(
        f"即将执行 level {args.level} 轨迹（kp={format_gain(args.kp)} kd={format_gain(args.kd)} "
        f"速度≤{args.max_joint_speed} rad/s 超前≤{args.max_command_lead} rad，"
        f"力矩上界 {torque_cap:.1f} Nm（三项合计；电机峰值 34.4 Nm 的 "
        f"{torque_cap / 34.4 * 100:.0f}%）"
        + ("，含重力前馈" if gravity is not None else "") + "）"
    )
    print("  ⚠️ 全程托住机械臂；急停放在手边。")
    if input("  输入 MOVE 继续：").strip() != "MOVE":
        print("已取消，未发送任何运动命令。")
        return 1

    # ---- 3. 执行 + 遥测 ---------------------------------------------------
    telemetry = TelemetryWriter(args.out)
    bus = R1ArmSdkBus(args.bridge_lib)
    exit_code = 0
    try:
        bus.open(args.port, args.baud, calib)
        run_trajectory(
            bus, t, q_ref, dq_ref, ddq_ref,
            kp=args.kp, kd=args.kd, rate=args.rate,
            max_joint_speed=args.max_joint_speed, max_lead=args.max_command_lead,
            gravity=gravity, telemetry=telemetry,
            home_target=(q_start if args.home_first else None),
        )
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停机。")
    except Exception as exc:
        print(f"\n轨迹执行中止：{exc}", file=sys.stderr)
        exit_code = 1
    finally:
        telemetry.close()
        _safe_stop(bus, args.rate)

    print("完成。机械臂已停机（mode 0），请托住。")
    try:
        report(args.out, args.rate)
    except Exception as exc:
        print(f"统计生成失败：{exc}", file=sys.stderr)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
