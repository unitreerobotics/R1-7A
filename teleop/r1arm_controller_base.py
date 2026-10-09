"""R1 单臂 / 双臂 PICO 遥操核心（相对位姿跟踪 + IK + Dex1），输出无关。

复用 A7 同款手柄相对跟踪语义：
- 按住 grip 激活跟踪
- 手柄 Δpos/Δrot → EE 目标（dex1_base_link）
- Placo QP（失败则 DLS）求 7 关节角
- trigger 驱动 Dex1（松开=开，按下=关），与 A7 GripperController 一致
- A 键复位

单臂：``control_side`` 指定左/右手柄。
双臂（``dual_arm=True``）：左右手柄分别控左右臂，忽略 ``control_side``。

子类通过 ``_output_command`` 接到 MuJoCo / 真机；真机可覆写 ``_send_dex1``。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from robot.arm_model import clamp_configuration, joint_indices_by_name, link_pose, load_robot_model
from robot.gripper import Dex1SimGripper
from robot.r1arm_config import (
    ARM_INIT,
    ARM_JOINTS,
    DEX1_JOINTS,
    R_TELEOP_TO_BASE,
    R1ArmConfig,
    sim_to_closed_ratio,
)
from robot.xr_interface import get_key_value, make_xr_client
from teleop.ik_solver import solve_upper_arm_ik
from teleop.r1arm_placo_solver import PlacoArmIKSolver
from teleop.utils.geometry import R_HEADSET_TO_WORLD, quaternion_to_euler, quaternion_to_world
from teleop.utils.mock import make_mock_controller_poses, make_mock_grip_values, xr_pose_is_valid


@dataclass
class _SideTrackState:
    """单侧手柄相对跟踪状态（左右各一份）。"""

    ref_ee_pos: np.ndarray | None = None
    ref_ee_quat: np.ndarray | None = None
    ref_controller_pos: np.ndarray | None = None
    ref_controller_rot: object | None = None
    ref_heading_yaw: float | None = None
    active: bool = False

    def clear(self) -> None:
        self.ref_ee_pos = None
        self.ref_ee_quat = None
        self.ref_controller_pos = None
        self.ref_controller_rot = None
        self.ref_heading_yaw = None
        self.active = False


class R1ArmTeleopControllerBase:
    """单臂或双臂 tracking + IK + Dex1。"""

    def __init__(
        self,
        config: R1ArmConfig,
        mock_xr: bool = False,
        debug: bool = False,
        scale_factor: float | None = None,
        allow_xr_fallback: bool = True,
    ):
        self.config = config
        self.mock_xr = mock_xr
        self.debug = debug
        self.allow_xr_fallback = bool(allow_xr_fallback)
        self.scale_factor = float(config.scale_factor if scale_factor is None else scale_factor)
        self.dual_arm = bool(config.dual_arm)
        self.control_side = str(config.control_side).lower()
        if self.dual_arm:
            self.control_sides = ("left", "right")
        else:
            if self.control_side not in {"left", "right"}:
                raise ValueError(f"control_side must be left|right, got {config.control_side!r}")
            self.control_sides = (self.control_side,)
        self.R_teleop_to_base = R_TELEOP_TO_BASE.copy()

        self.urdf_path = Path(config.urdf_path)
        self.model, self.data = load_robot_model(str(self.urdf_path))
        self.joint_indices = joint_indices_by_name(self.model)
        self.arm_q_indices = [self.joint_indices[name] for name in ARM_JOINTS]
        self.dex1_q_indices = [self.joint_indices[name] for name in DEX1_JOINTS if name in self.joint_indices]
        if len(self.dex1_q_indices) != len(DEX1_JOINTS):
            missing = [n for n in DEX1_JOINTS if n not in self.joint_indices]
            raise RuntimeError(f"URDF missing Dex1 joints: {missing}")
        self.ee_frame = config.ee_frame

        # 右臂（及单臂）用 self.q；双臂时左臂另有一份同构 URDF 状态
        self.q = self._create_initial_q()
        self.q_left = self._create_initial_q() if self.dual_arm else None

        self.use_placo = False
        self.placo_solver = None
        self.placo_solver_left = None
        try:
            self.placo_solver = PlacoArmIKSolver(
                self.urdf_path,
                self.q,
                self.ee_frame,
                self.joint_indices,
                arm_joints=ARM_JOINTS,
                hold_joints=DEX1_JOINTS,
                arm_task_weight=config.arm_task_weight,
                joint_velocity_limits=config.ik_joint_velocity_limits,
                default_velocity_limit=(
                    config.ik_qp_velocity_limit
                    if config.ik_qp_velocity_limit is not None else 0.0
                ),
            )
            if self.dual_arm:
                self.placo_solver_left = PlacoArmIKSolver(
                    self.urdf_path,
                    self.q_left,
                    self.ee_frame,
                    self.joint_indices,
                    arm_joints=ARM_JOINTS,
                    hold_joints=DEX1_JOINTS,
                    arm_task_weight=config.arm_task_weight,
                    joint_velocity_limits=config.ik_joint_velocity_limits,
                    default_velocity_limit=(
                        config.ik_qp_velocity_limit
                        if config.ik_qp_velocity_limit is not None else 0.0
                    ),
                )
            self.use_placo = True
            mode = "dual-arm" if self.dual_arm else "single-arm"
            qp_vlim = config.ik_qp_velocity_limit
            vlim_note = f", QP v≤{qp_vlim:g} rad/s" if qp_vlim else ""
            print(f"Using Placo QP IK solver ({mode}{vlim_note})")
        except Exception as exc:
            if self.debug:
                import traceback

                traceback.print_exc()
            print(f"Warning: Placo setup failed ({exc}); using DLS IK fallback.")

        print("Teleop frame aligned to r1arm base (A7 +X前/+Z上 → URDF -Y前/+Z上)")

        self.hand_ctrl = None
        self.last_gripper_update = time.monotonic()
        self.grippers = {
            side: Dex1SimGripper(
                open_value=config.dex1_open,
                close_value=config.dex1_close,
                speed=config.dex1_speed,
            )
            for side in self.control_sides
        }
        # 兼容旧字段：单臂时指向唯一夹爪
        self.gripper = self.grippers[self.control_sides[-1]]
        print(f"Dex1 sim gripper enabled (EE={self.ee_frame}, sides={list(self.control_sides)})")

        if mock_xr:
            self.xr_client = None
            print("Using mock XR input")
        else:
            try:
                self.xr_client = make_xr_client()
                print("Connected to XRoboToolkit")
            except Exception as e:
                if not self.allow_xr_fallback:
                    raise RuntimeError(
                        "Could not connect to XRoboToolkit and XR fallback is disabled"
                    ) from e
                print(f"Warning: Could not connect to XRoboToolkit: {e}")
                print("Falling back to mock XR input")
                self.xr_client = None
                self.mock_xr = True

        self.track: dict[str, _SideTrackState] = {side: _SideTrackState() for side in self.control_sides}
        # P2 整形器状态：每侧 7 个臂关节的上一拍输出速度（rad/s）。
        # grip 松开/跳变清锚时速度归零，重锚后从静止斜坡起步。
        self._ik_velocities: dict[str, np.ndarray] = {}
        # XR 帧时钟感知缓存（P2）：服务器帧时间戳 + 上一次的有效位姿快照
        self._xr_last_stamp: int | None = None
        self._xr_pose_cache: dict | None = None
        self._ik_shape_last_print: dict[str, float] = {}
        # 兼容单臂调试字段
        self.active = False

        self.reset_flag = True
        self.is_reset_flag = True
        self.enable_a_button_reset = True
        self.reset_blend_frames = max(1, int(config.reset_blend_frames))
        self._reset_blending = False
        self._reset_blend_deadline = 0.0
        self._q_blend_start: np.ndarray | None = None
        self._q_blend_goal: np.ndarray | None = None
        self._q_left_blend_start: np.ndarray | None = None
        self._q_left_blend_goal: np.ndarray | None = None
        self.running = True
        self.t = 0.0

    def _create_initial_q(self) -> np.ndarray:
        q = np.zeros(self.model.nq, dtype=float)
        for i, name in enumerate(ARM_JOINTS):
            if name in self.joint_indices:
                q[self.joint_indices[name]] = ARM_INIT[i]
        open_q = float(self.config.dex1_open)
        for name in DEX1_JOINTS:
            q[self.joint_indices[name]] = open_q
        return q

    def _q_for_side(self, side: str) -> np.ndarray:
        if side == "left" and self.dual_arm:
            return self.q_left
        return self.q

    def _set_q_for_side(self, side: str, q: np.ndarray) -> None:
        if side == "left" and self.dual_arm:
            self.q_left = q
        else:
            self.q = q

    def _placo_for_side(self, side: str):
        if side == "left" and self.dual_arm:
            return self.placo_solver_left
        return self.placo_solver

    def _link_pose(self, frame_name: str, q: np.ndarray | None = None) -> tuple:
        try:
            return link_pose(self.model, self.data, self.q if q is None else q, frame_name)
        except Exception:
            return None, None

    def _solve_dls(self, q: np.ndarray, target_pos: np.ndarray, target_quat: np.ndarray | None) -> np.ndarray:
        target_rot = None
        if target_quat is not None:
            target_rot = R.from_quat(target_quat).as_matrix()
        q_next, _ = solve_upper_arm_ik(
            self.model,
            self.data,
            q,
            ee_frame_name=self.ee_frame,
            ee_target_pos=target_pos,
            ee_target_rot=target_rot,
            elbow_frame_name=None,
            elbow_target_pos=None,
            q_indices=self.arm_q_indices,
            q_reference=q.copy(),
            control_mode="pose" if target_rot is not None else "position",
            debug=self.debug,
        )
        return q_next

    def _clamp_ik_step(self, side: str, q_prev: np.ndarray, q_next: np.ndarray) -> np.ndarray:
        """Shape a solver output's arm joints into a velocity/acceleration-feasible step.

        IK outputs are recorded as actions verbatim. Without this shaper a
        solver discontinuity (stale reference, target re-acquisition, DLS
        fallback) lands in the recording as a single-frame jump the hardware
        never executed; the recorded trajectory must stay physically
        executable at teleop speeds. Only arm joints are shaped; gripper
        joints are owned by the Dex1 update path.

        P2 velocity/acceleration shaper (replaces the hard velocity clamp):
        the old clip(v, ±v_max) produced acceleration steps at every clamp
        entry/exit (measured jerk p95 ~7000 rad/s³ on real teleop) — both the
        audible clicks and what a VLA would have to reproduce token-by-token.
        The shaper instead limits per-cycle velocity *and* the change of
        velocity, so clamp boundaries become 2-3 cycle ramps (a_max·dt):
          v_cmd = clip( v_prev ± a_max·dt, v_target, ±v_max )
          q_out = q_prev + v_cmd·dt
        Both caps keep their *speed/accel* meaning across control rates
        (calibrated at 50 Hz, rescaled with rate_hz like the SDK slew cap).
        """
        limit = self.config.ik_max_step_per_cycle
        if limit is None or limit <= 0:
            return q_next
        # Layer 1 of 2 (P1 统一目标变化约束):
        #   IK 层（本函数）：q_prev → q_next 的速度/加速度整形。
        #   SDK 层（limit_sdk_position_command）：command → 实测的 slew/lead 限制。
        # 两层有效速度按 rate 保持一致：默认 0.02 rad/拍 @50 Hz = 1 rad/s，
        # 与 SDK 层 max_delta = max_joint_speed / rate 同步换算。
        rate_hz = getattr(self, "rate_hz", 50.0) or 50.0
        dt = 1.0 / rate_hz
        v_max = limit * 50.0 / rate_hz / dt  # rad/s, rate-invariant
        a_max = self.config.ik_max_acc_per_s2 * 50.0 / rate_hz  # rad/s², rate-invariant
        q_out = q_next.copy()
        v_prev = self._ik_velocities.setdefault(side, np.zeros(len(self.arm_q_indices)))
        v_shaped = v_prev.copy()
        overshoot = []
        for k, (name, idx) in enumerate(zip(ARM_JOINTS, self.arm_q_indices)):
            v_target = float(q_out[idx] - q_prev[idx]) / dt  # solver's wanted velocity
            # QP 速度限是软约束，解可微超上限（浮点容差级）；只有显著超出
            # 才算整形事件——边界噪声不触发打印/处理。
            if abs(v_target) > v_max * (1.0 + 0.05) + 1e-6:
                overshoot.append(f"{name}={v_target * dt:+.3f}")
                v_target = float(np.clip(v_target, -v_max, v_max))
            v_shaped[k] = float(
                np.clip(v_target, v_prev[k] - a_max * dt, v_prev[k] + a_max * dt)
            )
            q_out[idx] = q_prev[idx] + v_shaped[k] * dt
        if overshoot:
            # 限速顶格在快速遥操中是常态（手速 > 臂速上限），打印限频：
            # 每秒最多一行，避免每拍刷屏淹掉真正的告警。
            now = time.monotonic()
            if now - self._ik_shape_last_print.get(side, 0.0) >= 1.0:
                self._ik_shape_last_print[side] = now
                print(
                    f"[IK] shaped solver speed ({side}): {', '.join(overshoot)} "
                    f"(v≤{v_max:.2f} rad/s, a≤{a_max:.0f} rad/s²; SDK 层另有兜底)"
                )
            q_clamped = clamp_configuration(self.model, q_out)
            # 限位投影可能削减步长（贴限位时）：速度状态按实际输出回写，
            # 避免 v 状态与 q 序列脱节（否则下一拍会按虚高 v_prev 限加速）。
            for k, idx in enumerate(self.arm_q_indices):
                v_shaped[k] = float(q_clamped[idx] - q_prev[idx]) / dt
            q_out = q_clamped
        self._ik_velocities[side] = v_shaped
        return q_out

    def _solve(self, side: str, target_pos: np.ndarray, target_quat: np.ndarray | None) -> None:
        q = self._q_for_side(side)
        solver = self._placo_for_side(side)
        if self.use_placo and solver is not None:
            sol = solver.solve(q, target_pos, target_quat)
            if sol is not None:
                q_next = q.copy()
                for idx in self.arm_q_indices:
                    q_next[idx] = sol[idx]
                self._set_q_for_side(side, self._clamp_ik_step(side, q, q_next))
                return
        # Placo 失败（不可达/奇异/异常）→ DLS 回退。DLS 在远离目标时可能
        # 返回一个大幅解——q 不动的回退语义应保持"上一个可行目标"：
        # _solve_dls 输出同样过 _clamp_ik_step（0.02 rad/拍），所以最坏
        # 情况是 1 rad/s 的朝 DLS 解爬行，而非瞬时跳变。
        self._set_q_for_side(
            side, self._clamp_ik_step(side, q, self._solve_dls(q, target_pos, target_quat))
        )

    def reset(self):
        """Start a soft home recovery (linear joint cross-fade), not an instant snap."""
        self._q_blend_start = self.q.copy()
        self._q_blend_goal = self._create_initial_q()
        if self.dual_arm:
            self._q_left_blend_start = self.q_left.copy()
            self._q_left_blend_goal = self._create_initial_q()
        # P1: 按单调时钟计时长（帧计数在 rate≠50 或周期抖动下不再等于标称秒）。
        # reset_blend_frames 仍是名义参数：blend 时长 = frames / 50 Hz 秒（50Hz
        # 历史语义）；30Hz 定居后保持 2s 体验 = 传 --reset-blend-frames 60。
        self._reset_blend_deadline = time.monotonic() + self.reset_blend_frames / 50.0
        self._reset_blending = True

        for st in self.track.values():
            st.clear()
        self._ik_velocities.clear()
        self.active = False
        self.reset_flag = True
        self.is_reset_flag = True
        if self.debug:
            print(
                f"[reset] soft blend to home over {self.reset_blend_frames / 50.0:.2f}s "
                f"(wall-clock; was frame-count at 50Hz)"
            )

    def _step_reset_blend(self) -> bool:
        if not self._reset_blending or self._q_blend_start is None or self._q_blend_goal is None:
            return False

        # P1: wall-clock progress. Cycle overruns no longer stretch the blend
        # (each frame used to count 1/frames regardless of real duration);
        # the smoothstep shape is preserved.
        w_linear = float(
            np.clip(
                (time.monotonic() - (self._reset_blend_deadline - self.reset_blend_frames / 50.0))
                / (self.reset_blend_frames / 50.0),
                0.0,
                1.0,
            )
        )
        w_new = w_linear * w_linear * (3.0 - 2.0 * w_linear)
        w_old = 1.0 - w_new
        self.q = clamp_configuration(self.model, w_old * self._q_blend_start + w_new * self._q_blend_goal)
        if self.dual_arm and self._q_left_blend_start is not None and self._q_left_blend_goal is not None:
            self.q_left = clamp_configuration(
                self.model, w_old * self._q_left_blend_start + w_new * self._q_left_blend_goal
            )

        if time.monotonic() >= self._reset_blend_deadline:
            self.q = clamp_configuration(self.model, self._q_blend_goal.copy())
            if self.dual_arm and self._q_left_blend_goal is not None:
                self.q_left = clamp_configuration(self.model, self._q_left_blend_goal.copy())
            for side, gripper in self.grippers.items():
                gripper.reset()
                q = self._q_for_side(side)
                for idx in self.dex1_q_indices:
                    q[idx] = gripper.current
                self._set_q_for_side(side, q)
            self._send_dex1_all()
            self._reset_blending = False
            self._q_blend_start = None
            self._q_blend_goal = None
            self._q_left_blend_start = None
            self._q_left_blend_goal = None
            if self.debug:
                print("[reset] soft blend done")
            return False
        return True

    def read_controller_poses(self) -> tuple:
        if self.mock_xr or self.xr_client is None:
            return make_mock_controller_poses(self.t), True
        # P2 修复：帧时钟感知。XR 服务端按自己的帧率（~30Hz）推送位姿，
        # 而控制环 50Hz 每拍都读——同一个位姿被重复消费 1~2 次且交替不
        # 均匀（33ms 与 20ms 不成整数比），IK 目标流成为"30Hz 阶梯 ×
        # 50Hz 采样"的拍频调制信号。只在服务器帧时间戳更新时读新位姿，
        # 其余拍沿用上一次的快照——目标更新节拍与 XR 服务端一致，拍频
        # 调制从源头消失（读取代价：一次 int 调用 vs 三次 pose 拷贝）。
        stamp = self.xr_client.get_frame_stamp_ns()
        if stamp is not None and stamp == self._xr_last_stamp:
            if self._xr_pose_cache is not None:
                return self._xr_pose_cache, True
        try:
            controller_poses = {}
            for name in ["headset", "left_controller", "right_controller"]:
                pose = self.xr_client.get_pose_by_name(name)
                if xr_pose_is_valid(pose):
                    controller_poses[name] = np.asarray(pose, dtype=float)[:7]
                else:
                    if not self.allow_xr_fallback:
                        if self.debug:
                            print(f"XR pose invalid: {name}; holding current command")
                        return {}, False
                    return make_mock_controller_poses(self.t), True
            out = self._reject_xr_jumps(controller_poses)
            self._xr_pose_cache = out
            self._xr_last_stamp = stamp
            return out, True
        except Exception as e:
            if self.debug:
                print(f"XR read error: {e}")
            if not self.allow_xr_fallback:
                return {}, False
            return make_mock_controller_poses(self.t), True

    # P1: XR 位姿跳变检测。跟踪丢失/重捕获的瞬间，XR 会上报一个跟上一帧
    # 完全无关的位姿（几厘米甚至几米的单帧跳变）。这种跳变直接进 IK 会产
    # 生一帧大步目标——被 clamp 后录进 action 的轨迹仍然是"从未执行过的
    # 意图跳变"。逐帧位置/姿态差超过阈值时判定为跳变：保留上一帧位姿，
    # 并清空跟踪参考（下一拍 grip 仍按住时会重新锚定相对位姿，而不是把
    # 跳变后的 delta 累加进去）。
    #
    # 毛刺 vs 搬位：单帧尖峰（回到原位）hold 是对的；但手柄被真实放到新
    # 位置（放下/交接/换手）时，新位姿会持续稳定存在——连续 N 帧都在同一
    # 个"新位置"附近，说明这不是噪声而是真实搬位。持续拒绝会让参考永久
    # 冻结、目标卡死、日志每帧刷屏（真机日志实测：连刷 50+ 行 0.4m 拒绝）。
    # 连续拒绝 ≥ N 帧后接受新位置为现实，清锚重捕获。
    _XR_JUMP_POS_M = 0.30  # 单帧 30cm 位移 = 跳变（人手 50Hz 下限 ~0.5m/s）
    _XR_JUMP_ROT_RAD = 1.0  # 单帧 57° 转动 = 跳变
    _XR_JUMP_MAX_REJECTS = 10  # 连续拒绝上限：超过 = 真实搬位，接受新位置

    def _reject_xr_jumps(self, poses: dict) -> dict:
        prev = getattr(self, "_xr_prev_poses", None)
        if prev is None:
            self._xr_prev_poses = {k: v.copy() for k, v in poses.items()}
            self._xr_jump_rejects = 0
            return poses
        rejects = getattr(self, "_xr_jump_rejects", 0)
        jumped = []
        out = {}
        for name, pose in poses.items():
            ref = prev.get(name)
            if ref is None:
                out[name] = pose
                continue
            dpos = float(np.linalg.norm(pose[:3] - ref[:3]))
            drot = R.from_quat(pose[3:7]).inv() * R.from_quat(ref[3:7])
            drot = float(np.linalg.norm(drot.as_rotvec()))
            if dpos > self._XR_JUMP_POS_M or drot > self._XR_JUMP_ROT_RAD:
                if rejects + 1 >= self._XR_JUMP_MAX_REJECTS:
                    # 持续偏离 = 真实搬位（手柄放下/挪动），不是单帧毛刺：
                    # 接受新位置，参考随后重建。
                    out[name] = pose
                    continue
                jumped.append(f"{name}({dpos:.2f}m/{np.degrees(drot):.0f}deg)")
                out[name] = ref  # hold last valid pose
            else:
                out[name] = pose
        self._xr_prev_poses = {k: v.copy() for k, v in out.items()}
        if jumped:
            self._xr_jump_rejects = rejects + 1
            print(f"[XR] rejected jump: {', '.join(jumped)}; holding previous pose")
            # 跳变后旧参考已不可信：清锚，grip 重锚逻辑（_track_side 的
            # ref_ee_pos is None 分支）会在下一拍用当前位姿重建参考。
            for st in self.track.values():
                st.clear()
            self._ik_velocities.clear()
        elif rejects > 0 or any(
            np.linalg.norm(out[n][:3] - prev.get(n, out[n])[:3]) > self._XR_JUMP_POS_M
            for n in out
        ):
            # 没有拒绝但上一轮处于拒绝状态 / 帧刚被接受为搬位：计数清零、
            # 清锚让 grip 用新位置重建参考（不打印——正常搬位不是事件）。
            if rejects > 0:
                self._xr_jump_rejects = 0
                for st in self.track.values():
                    st.clear()
                self._ik_velocities.clear()
        else:
            self._xr_jump_rejects = 0
        return out

    def get_grip_state(self, side: str | None = None) -> float:
        side = side or self.control_side
        if self.mock_xr or self.xr_client is None:
            left_grip, right_grip, _, _ = make_mock_grip_values(self.t)
            return left_grip if side == "left" else right_grip
        return get_key_value(self.xr_client, f"{side}_grip")

    def get_trigger_state(self, side: str | None = None) -> float:
        side = side or self.control_side
        if self.mock_xr or self.xr_client is None:
            _, _, left_trigger, right_trigger = make_mock_grip_values(self.t)
            return left_trigger if side == "left" else right_trigger
        return get_key_value(self.xr_client, f"{side}_trigger")

    def get_button_state(self, name: str) -> bool:
        if self.mock_xr or self.xr_client is None:
            return False
        try:
            return bool(self.xr_client.get_button_state_by_name(name))
        except Exception:
            return False

    def get_reset_button(self) -> bool:
        if not self.enable_a_button_reset:
            return False
        return self.get_button_state("A")

    def _update_dex1_side(self, side: str, dt: float) -> None:
        trigger = self.get_trigger_state(side)
        if trigger < self.config.trigger_deadband:
            trigger = 0.0
        value = self.grippers[side].update(trigger, dt)
        q = self._q_for_side(side).copy()
        for idx in self.dex1_q_indices:
            q[idx] = value
        self._set_q_for_side(side, q)

    def _send_dex1_all(self) -> None:
        if self.hand_ctrl is None:
            return
        if self.dual_arm:
            left_v = self.grippers["left"].current
            right_v = self.grippers["right"].current
            left_r = sim_to_closed_ratio(left_v, self.config.dex1_open, self.config.dex1_close)
            right_r = sim_to_closed_ratio(right_v, self.config.dex1_open, self.config.dex1_close)
            self.hand_ctrl.set_gripper_ratios(left_r, right_r)
            return
        side = self.control_side
        ratio = sim_to_closed_ratio(
            self.grippers[side].current, self.config.dex1_open, self.config.dex1_close
        )
        if side == "left":
            self.hand_ctrl.set_gripper_ratios(ratio, 0.0)
        else:
            self.hand_ctrl.set_gripper_ratios(0.0, ratio)

    def _track_side(self, side: str, controller_poses: dict) -> None:
        st = self.track[side]
        st.active = self.get_grip_state(side) > 0.9
        controller_pose = controller_poses.get(f"{side}_controller")
        q = self._q_for_side(side)

        if controller_pose is not None and st.active:
            ee_pos, ee_quat = self._link_pose(self.ee_frame, q)
            if ee_pos is not None:
                ctrl_pos = R_HEADSET_TO_WORLD @ controller_pose[:3]
                ctrl_rot_world = quaternion_to_world(controller_pose[3:7])

                if st.ref_ee_pos is None:
                    st.ref_ee_pos = ee_pos.copy()
                    st.ref_ee_quat = ee_quat.copy() if ee_quat is not None else None
                    st.ref_controller_pos = ctrl_pos.copy()
                    st.ref_controller_rot = ctrl_rot_world
                    euler = quaternion_to_euler(*ctrl_rot_world.as_quat())
                    st.ref_heading_yaw = float(euler[2])
                    # Re-anchor the posture reference at the pose the arm is
                    # actually in when tracking starts. A stale reference (e.g.
                    # the dangling startup pose) otherwise pulls the solver one
                    # integrator step toward it on the very first solve,
                    # producing a single-frame action jump that never ran on
                    # the hardware.
                    solver = self._placo_for_side(side)
                    if self.use_placo and solver is not None:
                        solver.set_regularization_reference(q)
                else:
                    delta_pos = ctrl_pos - st.ref_controller_pos
                    yaw_ref = st.ref_heading_yaw or 0.0
                    c = math.cos(-yaw_ref)
                    s = math.sin(-yaw_ref)
                    delta_pos_corrected = np.array(
                        [
                            c * delta_pos[0] - s * delta_pos[1],
                            s * delta_pos[0] + c * delta_pos[1],
                            delta_pos[2],
                        ]
                    )
                    delta_pos_base = self.R_teleop_to_base @ delta_pos_corrected
                    target_pos = st.ref_ee_pos + delta_pos_base * self.scale_factor

                    target_quat = None
                    if st.ref_controller_rot is not None and st.ref_ee_quat is not None:
                        rot_delta = ctrl_rot_world * st.ref_controller_rot.inv()
                        delta_rot = rot_delta.as_rotvec()
                        delta_rot = np.array(
                            [
                                c * delta_rot[0] - s * delta_rot[1],
                                s * delta_rot[0] + c * delta_rot[1],
                                delta_rot[2],
                            ]
                        )
                        delta_rot = self.R_teleop_to_base @ delta_rot
                        rot_target = R.from_rotvec(delta_rot) * R.from_quat(st.ref_ee_quat)
                        target_quat = rot_target.as_quat()

                    self._solve(side, target_pos, target_quat)
        else:
            st.clear()
            # 松开 grip / XR 位姿缺失：跟踪停摆，整形器速度状态清零——
            # 重锚后从静止斜坡起步，不继承上一次跟踪的残余速度。
            self._ik_velocities.pop(side, None)

    def step(self, dt: float) -> np.ndarray:
        self.t += dt

        if self._reset_blending:
            self._step_reset_blend()
            self._output_command()
            return self.q

        controller_poses, valid = self.read_controller_poses()
        if not valid:
            self._output_command()
            return self.q

        if self.get_reset_button() and not self.is_reset_flag:
            self.reset()
            self._step_reset_blend()
            self._output_command()
            return self.q
        if not self.get_reset_button():
            self.is_reset_flag = False

        for side in self.control_sides:
            self._track_side(side, controller_poses)
        self.active = any(self.track[s].active for s in self.control_sides)

        now = time.monotonic()
        gripper_dt = max(0.0, now - self.last_gripper_update)
        self.last_gripper_update = now
        for side in self.control_sides:
            self._update_dex1_side(side, gripper_dt)
        self._send_dex1_all()

        self.q = clamp_configuration(self.model, self.q)
        if self.dual_arm:
            self.q_left = clamp_configuration(self.model, self.q_left)

        self._output_command()

        if self.debug and int(self.t * 5) % 5 == 0:
            parts = []
            for side in self.control_sides:
                q = self._q_for_side(side)
                arm_q = [q[idx] for idx in self.arm_q_indices]
                parts.append(
                    f"{side}={np.round(arm_q, 3)} act={self.track[side].active}"
                )
            print(f"t={self.t:.2f}s | " + " | ".join(parts))

        return self.q

    def _output_command(self):
        """子类实现：把 self.q（及 dual 的 q_left）下发到仿真 / 真机。"""

    def _on_start(self):
        pass

    def _on_stop(self):
        if self.hand_ctrl is not None:
            self.hand_ctrl.stop()
        # The native XR service owns background threads. Closing it explicitly
        # before interpreter teardown avoids occasional shutdown segfaults.
        xr_client = self.xr_client
        self.xr_client = None
        if xr_client is not None:
            try:
                xr_client.close()
            except Exception as exc:
                if self.debug:
                    print(f"XR close warning: {exc}")

    def get_joint_states(self) -> dict:
        return {name: float(self.q[idx]) for name, idx in self.joint_indices.items()}

    def arm_q(self, side: str = "right") -> np.ndarray:
        q = self._q_for_side(side)
        return np.array([q[idx] for idx in self.arm_q_indices], dtype=float)

    def dex1_q(self, side: str = "right") -> np.ndarray:
        q = self._q_for_side(side)
        return np.array([q[idx] for idx in self.dex1_q_indices], dtype=float)

    def sim_q(self) -> np.ndarray:
        """MuJoCo 控制向量。

        单臂：右臂 7+2。
        双臂：右臂 7+2 再左臂 7+2（与 scene_dual_arm actuator 序一致）。
        """
        right = np.hstack([self.arm_q("right"), self.dex1_q("right")])
        if not self.dual_arm:
            return right
        left = np.hstack([self.arm_q("left"), self.dex1_q("left")])
        return np.hstack([right, left])

    def run(self, rate: float = 50.0):
        if self.dual_arm:
            print(f"Starting R1Arm dual-arm teleoperation (rate={rate} Hz)")
            print("Hold left/right grip to activate corresponding arm")
            print("Hold left/right trigger to close corresponding Dex1")
        else:
            print(f"Starting R1Arm teleoperation (rate={rate} Hz, side={self.control_side})")
            print(f"Hold {self.control_side} grip to activate arm tracking")
            print(f"Hold {self.control_side} trigger to close Dex1 gripper")
        if self.enable_a_button_reset:
            print(
                f"Press A button to soft-reset to home "
                f"({self.reset_blend_frames} frames cross-fade)"
            )
        else:
            print("A-button home reset is disabled")
        print("Press Ctrl+C to stop")

        dt = 1.0 / rate
        try:
            self._on_start()
            while self.running:
                loop_start = time.time()
                self.step(dt)
                elapsed = time.time() - loop_start
                sleep_t = dt - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.running = False
            self._on_stop()
