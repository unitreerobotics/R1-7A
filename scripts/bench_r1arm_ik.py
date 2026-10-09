#!/usr/bin/env python3
"""测量摇操控制循环各环节的耗时上限（IK 解算频率能到多少 Hz）。

不打开 SDK、不发电机命令：复用 teleop 控制器的完整 step() 路径
（XR 读 → IK 解算 → Dex1 → clamp），输出命令走安全分支（不下发）。

两种控制器后端：
  默认          纯计算链路（无窗口）
  --mujoco     MuJoCo 子类：每拍把 IK 结果写进位置伺服并推仿真，
                弹出 viewer 可看到臂跟手 —— 同时测量含仿真步的频率。
                ⚠ 此模式的耗时含 MuJoCo 步进与 viewer 同步，测的是
                "带可视化的完整链路"，IK 单项耗时仍单列。

两种模式：
  --duration N   按 --rate 跑 N 秒，打印每拍耗时分解 + 实际达成频率
  --sweep        从 50 Hz 起倍频（50→100→200→…），每档跑 3 秒，
                 直到"实际频率 < 目标频率 × 95%"，报告 IK 能稳住的上限

XR 不可用时自动落 mock 模式（IK/控制链路照测，只少了真实 XR 读取时间）。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.r1arm_config import R1ArmConfig
from teleop.r1arm_controller_base import R1ArmTeleopControllerBase


class TimingController(R1ArmTeleopControllerBase):
    """R1ArmTeleopControllerBase with per-stage timing instrumentation.

    _output_command is the real controller's only hardware touchpoint; the
    base-class implementation is already a no-op (sim subclasses override it),
    so this class never issues motor commands.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.timing = {}
        self._t0 = 0.0

    def _mark(self, key: str) -> None:
        now = time.perf_counter()
        if self._t0 == 0.0:
            self._t0 = now
            return
        self.timing[key] = self.timing.get(key, 0.0) + (now - self._t0)
        self._t0 = now

    def read_controller_poses(self):
        self._mark("start")
        result = super().read_controller_poses()
        self._mark("xr_read")
        return result

    def _solve(self, side: str, target_pos: np.ndarray, target_quat) -> None:
        self._mark("pre_solve")
        super()._solve(side, target_pos, target_quat)
        self._mark("ik_solve")

    def _update_dex1_side(self, side: str, dt: float) -> None:
        self._mark("pre_dex1")
        super()._update_dex1_side(side, dt)
        self._mark("dex1")

    def _send_dex1_all(self) -> None:
        self._mark("pre_send_dex1")
        super()._send_dex1_all()
        self._mark("send_dex1")

    def step(self, dt: float):
        self._mark("pre_step")
        self._t0 = time.perf_counter()
        result = super().step(dt)
        self._mark("total")
        return result


def make_timing_controller(mock_xr: bool, mujoco: bool) -> "TimingController":
    """Timing wrapper around the base or the MuJoCo sim controller.

    The MuJoCo variant instantiates the real sim subclass (viewer included)
    and layers the same per-stage timing on top of its step; its
    _output_command writes to the MuJoCo position actuators, never hardware.
    """
    if not mujoco:
        return TimingController(
            config=R1ArmConfig(control_side="right"), mock_xr=mock_xr, debug=False
        )

    from teleop.r1arm_mujoco_controller import R1ArmMujocoTeleopController

    class TimingMujocoController(TimingController, R1ArmMujocoTeleopController):
        pass

    return TimingMujocoController(
        config=R1ArmConfig(control_side="right"), mock_xr=mock_xr, debug=False,
        show_viewer=True,
    )


STAGES = ("xr_read", "pre_solve", "ik_solve", "pre_dex1", "dex1", "send_dex1", "total")


def run_at_rate(controller: TimingController, rate: float, duration: float) -> dict:
    controller.timing = {k: 0.0 for k in STAGES}
    solve_calls = {"n": 0}
    orig_solve = controller._solve

    def counting_solve(side, target_pos, target_quat):
        solve_calls["n"] += 1
        return orig_solve(side, target_pos, target_quat)

    controller._solve = counting_solve
    dt = 1.0 / rate
    cycles = 0
    wall = 0.0
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        loop_start = time.perf_counter()
        controller.step(dt)
        elapsed = time.perf_counter() - loop_start
        wall += elapsed
        cycles += 1
        sleep_t = dt - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)
    controller._solve = orig_solve
    timing = {k: v / max(cycles, 1) for k, v in controller.timing.items()}
    return {
        "cycles": cycles,
        "wall_per_cycle": wall / max(cycles, 1),
        "timing": timing,
        "solve_calls": solve_calls["n"],
    }


def report(rate: float, result: dict) -> float:
    achieved = 1.0 / max(result["wall_per_cycle"], 1e-9)
    busy = result["timing"]["total"]
    timing = result["timing"]
    solved = result["solve_calls"]
    ratio = solved / max(result["cycles"], 1)
    warn = "" if ratio > 0.9 else (
        f"  ⚠ IK 只解算了 {ratio * 100:.0f}% 的拍（握住 PICO grip 才激活跟踪）"
    )
    print(f"rate={rate:>6.0f} Hz | 实际 {achieved:7.1f} Hz | 每拍忙时 {busy * 1000:6.2f} ms "
          f"| XR读 {timing['xr_read'] * 1000:5.2f} | IK {timing['ik_solve'] * 1000:5.2f} "
          f"| Dex1 {timing['dex1'] * 1000:5.2f} | 其他 "
          f"{(busy - timing['xr_read'] - timing['ik_solve'] - timing['dex1']) * 1000:5.2f} ms"
          f" | IK解算 {solved}/{result['cycles']}{warn}")
    return achieved


def log_stream(controller, rate: float, duration: float, csv_path: Path) -> None:
    """遥操录波：逐拍记录 IK 目标流（action 语义）+ 仿真实测 + solver 原始意图。

    分层验证用：只看 IK 层输出的平滑性（v/a 的 p95），不碰真机。
    列：t, arm_q(7)（整形后=录进 action 的值）, solver_q(7)（整形前）,
    sim_q(7)（MuJoCo 实测；纯计算后端时无此列）。
    """
    import csv as csv_mod

    dt = 1.0 / rate
    has_sim = hasattr(controller, "sim")
    # 拦截整形前的 solver 输出：_solve 里 set_q 之前抓 sol
    raw = {"q": None}

    def _wrap_solve(side, target_pos, target_quat):
        solver = controller._placo_for_side(side)
        if controller.use_placo and solver is not None:
            sol = solver.solve(controller._q_for_side(side), target_pos, target_quat)
            if sol is not None:
                raw["q"] = np.asarray(sol, dtype=float).copy()
                q_next = controller._q_for_side(side).copy()
                for idx in controller.arm_q_indices:
                    q_next[idx] = sol[idx]
                controller._set_q_for_side(
                    side, controller._clamp_ik_step(side, controller._q_for_side(side), q_next))
                return
        raw["q"] = None
        controller._set_q_for_side(
            side, controller._clamp_ik_step(
                side, controller._q_for_side(side),
                controller._solve_dls(controller._q_for_side(side), target_pos, target_quat)))

    orig_solve = controller._solve
    controller._solve = _wrap_solve

    fields = (["t", "i"]
              + [f"arm_q_{j}" for j in range(7)]
              + [f"solver_q_{j}" for j in range(7)]
              + ([f"sim_q_{j}" for j in range(7)] if has_sim else []))
    rows: list[list] = []
    cycles = 0
    deadline = time.monotonic() + duration
    try:
        while time.monotonic() < deadline:
            loop_start = time.perf_counter()
            controller.step(dt)
            arm = controller.arm_q()
            solver_q = raw["q"][:7] if raw["q"] is not None else arm
            row = [controller.t, cycles] + [float(v) for v in arm] + \
                  [float(v) for v in solver_q]
            if has_sim:
                row += [float(v) for v in controller.sim.get_q()[:7]]
            rows.append(row)
            cycles += 1
            sleep_t = dt - (time.perf_counter() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)
    finally:
        controller._solve = orig_solve

    with open(csv_path, "w", newline="") as f:
        w = csv_mod.writer(f)
        w.writerow(fields)
        w.writerows(rows)

    # 平滑度统计：action 流（arm_q）与 solver 原始意图对照
    arr = np.asarray(rows, dtype=float)
    t, arm = arr[:, 0], arr[:, 2:9]
    solver = arr[:, 9:16]
    real_dt = np.median(np.diff(t))
    eff_hz = 1.0 / max(real_dt, 1e-9)
    names = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]

    def v_a(x):
        v = np.abs(np.diff(x, axis=0)) * eff_hz
        a = np.abs(np.diff(v, axis=0)) * eff_hz
        return v, a

    v_arm, a_arm = v_a(arm)
    v_sol, a_sol = v_a(solver)
    print(f"\n===== IK 目标流平滑度（{cycles} 拍 @ 实际 {eff_hz:.0f} Hz，CSV: {csv_path}）=====")
    print(f"  {'joint':<5}{'v_p95整形':>9}{'v_p95原始':>9}{'a_p50':>7}{'a_p95':>8}"
          f"{'a_max':>8}{'a_p95原始':>9}{'a_max原始':>9}")
    for j in range(7):
        print(f"  {names[j]:<5}{np.percentile(v_arm[:,j],95):>9.3f}"
              f"{np.percentile(v_sol[:,j],95):>9.3f}"
              f"{np.median(a_arm[:,j]):>7.2f}{np.percentile(a_arm[:,j],95):>8.2f}"
              f"{a_arm[:,j].max():>8.2f}{np.percentile(a_sol[:,j],95):>9.2f}"
              f"{a_sol[:,j].max():>9.2f}")
    worst = float(np.percentile(a_arm, 95))
    shaper_cap = 8.0
    verdict = "PASS（整形后 a_p95 ≤ 上限）" if worst <= shaper_cap + 0.5 else \
        f"FAIL（a_p95 {worst:.1f} > 整形上限 {shaper_cap}）"
    print(f"\n  action 流 a_p95 = {worst:.2f} rad/s² —— {verdict}")
    raw_gain = float(np.percentile(a_sol, 95) / max(worst, 1e-9))
    print(f"  整形收益：solver 原始意图 a_p95 是整形后的 {raw_gain:.1f}×（毛刺被斜坡化）")
    if has_sim:
        sim = arr[:, 16:23]
        v_sim, a_sim = v_a(sim)
        print(f"\n  仿真实测（对照真机 state 语义）：a_p95 = {np.percentile(a_sim, 95):.1f} rad/s²"
              f"（执行层动态，允许高于 action 流）")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rate", type=float, default=50.0, help="固定频率测量（默认 50）")
    parser.add_argument("--duration", type=float, default=5.0, help="每档测量秒数（默认 5）")
    parser.add_argument("--sweep", action="store_true",
                        help="从 50 Hz 倍频扫描到 IK 极限（50/100/200/…）")
    parser.add_argument("--mock-xr", action="store_true", help="强制 mock XR（不连 PICO）")
    parser.add_argument("--mujoco", action="store_true",
                        help="用 MuJoCo 仿真后端（弹 viewer 看臂跟手）；耗时含仿真步与渲染同步")
    parser.add_argument(
        "--log", type=float, default=0.0, metavar="SECONDS",
        help="遥操录波 N 秒：逐拍记录 IK action 流 + solver 原始意图 + 仿真实测到 CSV，"
             "结束打印平滑度统计（v/a p95，对照 P2 整形上限 8 rad/s²）。"
             "与 --rate/--mujoco 组合；--sweep 下忽略",
    )
    parser.add_argument("--out", type=Path, default=Path("/tmp/r1arm_ik_stream.csv"),
                        help="--log 的 CSV 输出路径")
    args = parser.parse_args()

    controller = make_timing_controller(mock_xr=args.mock_xr, mujoco=args.mujoco)
    print(f"IK solver: {'Placo QP' if controller.use_placo else '自写 DLS'}"
          f"  XR: {'mock' if controller.mock_xr else 'PICO live'}")
    print(f"（不打开 SDK、不发电机命令；每拍跑完整 step()：XR 读 → IK → Dex1 → clamp）")
    if not controller.mock_xr:
        print("⚠ 测量期间请握住 PICO grip 并持续移动手柄——grip 未激活时 IK 不解算，测出来是空转频率\n")
    else:
        print("⚠ mock 模式：XR 读耗时为 0，测的是纯计算链路\n")

    try:
        if args.log > 0:
            if args.mock_xr:
                print("⚠ mock XR 的位姿是脚本内正弦，曲线平滑度不代表真实手柄——"
                      "验证 IK 层请连真 PICO 握 grip 操作\n")
            print(f"录波 {args.log:.0f}s @ {args.rate:.0f} Hz"
                  f"（后端：{'MuJoCo' if args.mujoco else '纯计算'}）……"
                  "握住 grip 正常遥操，结束时自动出统计\n")
            log_stream(controller, args.rate, args.log, args.out)
        elif args.sweep:
            rate = 50.0
            last_ok = 0.0
            result = None
            while rate <= 3200:
                result = run_at_rate(controller, rate, args.duration)
                achieved = report(rate, result)
                if achieved < rate * 0.95:
                    print(f"\n→ {rate:.0f} Hz 档失守（实际 {achieved:.0f} Hz < 95% 目标）")
                    break
                last_ok = rate
                rate *= 2
            if result is not None:
                busy = result["timing"]["total"]
                print(f"\n结论：控制循环忙时上限 ≈ {1.0 / busy:.0f} Hz"
                      f"（IK 实测每拍 {result['timing']['ik_solve'] * 1000:.2f} ms）")
                if last_ok:
                    print(f"可稳定运行的最高扫描档：{last_ok:.0f} Hz")
                if result["solve_calls"] < result["cycles"] * 0.9:
                    print("⚠ 本次测量 IK 未全程激活（grip 没握住），数值只反映空转链路，不可作为结论！")
        else:
            result = run_at_rate(controller, args.rate, args.duration)
            report(args.rate, result)
            ik = result["timing"]["ik_solve"]
            print(f"\n分解：IK 每拍 {ik * 1000:.2f} ms → 纯 IK 频率上限 ≈ {1.0 / max(ik, 1e-9):.0f} Hz")
            if result["solve_calls"] < result["cycles"] * 0.9:
                print("⚠ 本次测量 IK 未全程激活（grip 没握住），数值只反映空转链路，不可作为结论！")
    finally:
        sim = getattr(controller, "sim", None)
        if sim is not None:
            try:
                sim.close()
            except Exception:
                pass
        xr = getattr(controller, "xr_client", None)
        if xr is not None:
            try:
                xr.close()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
