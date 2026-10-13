"""Standalone R1Arm real-robot output for the proven simulation teleop core."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Callable

import numpy as np

from robot.r1arm_config import ARM_JOINTS, ARM_LIMITS, R1ArmConfig
from robot.arm_model import clamp_configuration
from robot.r1arm_sdk import R1ArmSdkBus, R1ArmSdkState, model_to_sdk_q, sdk_to_model_q
from teleop.r1arm_controller_base import R1ArmTeleopControllerBase
from teleop.r1arm_placo_solver import joint_delta_bounds


MODEL_LOWER = np.array([ARM_LIMITS[name][0] for name in ARM_JOINTS], dtype=float)
MODEL_UPPER = np.array([ARM_LIMITS[name][1] for name in ARM_JOINTS], dtype=float)
SDK_LIMIT_A = model_to_sdk_q(MODEL_LOWER)
SDK_LIMIT_B = model_to_sdk_q(MODEL_UPPER)
SDK_LOWER = np.minimum(SDK_LIMIT_A, SDK_LIMIT_B)
SDK_UPPER = np.maximum(SDK_LIMIT_A, SDK_LIMIT_B)


def limit_sdk_position_command(
    target: np.ndarray,
    previous: np.ndarray,
    measured: np.ndarray,
    *,
    max_delta: float,
    max_lead: float,
) -> np.ndarray:
    """Apply hard limits, per-cycle slew limiting and measured-position lead limiting."""
    target = np.asarray(target, dtype=float).reshape(7)
    previous = np.asarray(previous, dtype=float).reshape(7)
    measured = np.asarray(measured, dtype=float).reshape(7)
    if not all(np.all(np.isfinite(values)) for values in (target, previous, measured)):
        raise RuntimeError("Non-finite R1Arm command/feedback")
    delta = abs(float(max_delta))
    lead = abs(float(max_lead))
    slew_lower = np.maximum(SDK_LOWER, previous - delta)
    slew_upper = np.minimum(SDK_UPPER, previous + delta)
    lower = np.maximum(slew_lower, measured - lead)
    upper = np.minimum(slew_upper, measured + lead)
    feasible = lower <= upper
    command = np.clip(target, slew_lower, slew_upper)
    command[feasible] = np.clip(target[feasible], lower[feasible], upper[feasible])
    # If measured position is already farther from the previous command than
    # lead + slew permits, preserve the hard per-cycle slew and catch up over
    # subsequent cycles; both constraints cannot hold on this one cycle.
    return np.clip(command, SDK_LOWER, SDK_UPPER)


class R1ArmRealTeleopController(R1ArmTeleopControllerBase):
    """PICO + R1Arm IK core with output sent through ``ArmMotorBus``."""

    def __init__(
        self,
        config: R1ArmConfig,
        *,
        port: str,
        baudrate: int,
        calibration_path: str | Path,
        bridge_library: str | Path | None = None,
        motion_mode: bool = False,
        enable_dex1: bool = False,
        dex1_interface: str = "",
        dex1_domain: int = 0,
        grip_torque_limit: float = -1.0,
        kp: "float | np.ndarray" = 40.0,
        kd: "float | np.ndarray" = 0.5,
        rate: float = 50.0,
        max_joint_speed: float = 0.5,
        max_command_lead: float = 0.15,
        feedback_timeout: float = 3.0,
        foc_timeout: float = 1.0,
        home_on_start: bool = False,
        mock_xr: bool = False,
        debug: bool = False,
        scale_factor: float | None = None,
        ik_dt: float = 0.0,
        gravity_ff: bool = False,
        tau_clip: float = 8.0,
        bus_factory: Callable[..., R1ArmSdkBus] = R1ArmSdkBus,
    ) -> None:
        if config.dual_arm:
            raise ValueError("R1Arm real controller currently supports one standalone arm only")
        if motion_mode and mock_xr:
            raise ValueError("Mock XR is disabled in motion mode")
        if rate <= 0 or max_joint_speed <= 0 or max_command_lead <= 0:
            raise ValueError("rate, max_joint_speed and max_command_lead must be positive")

        super().__init__(
            config=config,
            mock_xr=mock_xr,
            debug=debug,
            scale_factor=scale_factor,
            allow_xr_fallback=not motion_mode,
        )
        self.motion_mode = bool(motion_mode)
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.calibration_path = Path(calibration_path).expanduser().resolve()
        self.bridge_library = bridge_library
        self.enable_dex1 = bool(enable_dex1)
        self.dex1_interface = str(dex1_interface)
        self.dex1_domain = int(dex1_domain)
        self.grip_torque_limit = float(grip_torque_limit)
        # Scalar gains broadcast to all joints via the legacy bridge ABI; a
        # 7-vector takes the per-joint v2 ABI (shoulders/elbows vs wrists).
        self.kp = kp if isinstance(kp, np.ndarray) else float(kp)
        self.kd = kd if isinstance(kd, np.ndarray) else float(kd)
        self.rate_hz = float(rate)
        self.max_delta = float(max_joint_speed) / self.rate_hz
        self.max_joint_speed = float(max_joint_speed)
        # Set Placo's velocity-limit timestep to the control period (or an
        # explicit override). In the current unconstrained pose-IK mode this
        # value does not scale the raw IK step.
        for solver in (self.placo_solver, self.placo_solver_left):
            if solver is not None:
                solver_dt = float(ik_dt) if ik_dt and ik_dt > 0 else 1.0 / self.rate_hz
                solver.control_rate_hz = 1.0 / solver_dt
                solver.solver_dt = solver_dt
                solver.solver.dt = solver_dt
        self.gravity_ff = bool(gravity_ff)
        self.tau_clip = float(tau_clip)
        self.max_command_lead = float(max_command_lead)
        self.feedback_timeout = float(feedback_timeout)
        self.foc_timeout = float(foc_timeout)
        self.home_on_start = bool(home_on_start)
        self.bus_factory = bus_factory
        self.arm_bus: R1ArmSdkBus | None = None
        self.command_sdk: np.ndarray | None = None
        self._last_command_time_ns: int | None = None
        self._last_command_monotonic_s: float | None = None
        self._last_command_dt = 1.0 / self.rate_hz
        self.last_state: R1ArmSdkState | None = None
        self._feedback_last_counts: np.ndarray | None = None
        self._feedback_last_seen_monotonic_ns: np.ndarray | None = None
        self.last_report_time = 0.0

    def _check_feedback_range(self, state: R1ArmSdkState) -> np.ndarray:
        if not np.all(state.ready):
            missing = np.flatnonzero(~state.ready).tolist()
            raise RuntimeError(f"R1Arm feedback incomplete; missing joints {missing}")
        if not np.all(np.isfinite(state.q)):
            raise RuntimeError("R1Arm feedback contains non-finite joint positions")
        model_q = sdk_to_model_q(state.q)
        tolerance = 0.10
        invalid = np.flatnonzero(
            (model_q < MODEL_LOWER - tolerance) | (model_q > MODEL_UPPER + tolerance)
        )
        if invalid.size:
            detail = ", ".join(
                f"{ARM_JOINTS[i]}={model_q[i]:+.3f}" for i in invalid
            )
            raise RuntimeError(
                "Feedback is inconsistent with the simulation URDF limits "
                f"({detail}). Check calibration and the J4 convention before enabling motion."
            )
        return model_q

    def _reset_feedback_state(self, side: str) -> tuple[np.ndarray, np.ndarray] | None:
        if side != self.control_side or self.last_state is None:
            return None
        if not np.all(self.last_state.ready):
            return None
        return (
            sdk_to_model_q(self.last_state.q),
            sdk_to_model_q(self.last_state.dq),
        )

    @staticmethod
    def _check_faults(state: R1ArmSdkState, require_foc: bool) -> None:
        faults = np.flatnonzero(state.error_code != 0)
        if faults.size:
            detail = ", ".join(
                f"J{i + 1}=0x{int(state.error_code[i]):x}" for i in faults
            )
            raise RuntimeError(f"R1Arm motor fault: {detail}")
        if require_foc:
            stopped = np.flatnonzero(state.mode != 1)
            if stopped.size:
                joints = ", ".join(f"J{i + 1}" for i in stopped)
                raise RuntimeError(f"R1Arm left FOC mode: {joints}")

    def _wait_for_feedback(self) -> R1ArmSdkState:
        assert self.arm_bus is not None
        deadline = time.monotonic() + self.feedback_timeout
        while time.monotonic() < deadline:
            self.arm_bus.probe_once()
            time.sleep(1.0 / self.rate_hz)
            state = self.arm_bus.read_state()
            if np.all(state.ready):
                self._check_feedback_range(state)
                self._check_faults(state, require_foc=False)
                return state
        state = self.arm_bus.read_state()
        missing = np.flatnonzero(~state.ready).tolist()
        raise TimeoutError(
            f"No complete R1Arm feedback within {self.feedback_timeout:.1f}s; "
            f"missing joints {missing}"
        )

    def _seed_model_from_feedback(self, state: R1ArmSdkState) -> None:
        model_q = self._check_feedback_range(state)
        q = self.q.copy()
        for index, value in zip(self.arm_q_indices, model_q):
            q[index] = value
        self.q = q
        self._solver_seeds[self.control_side] = q.copy()
        if self.placo_solver is not None:
            self.placo_solver.set_regularization_reference(self.q)
        for track in self.track.values():
            track.clear()
        self.active = False
        self.command_sdk = state.q.copy()
        self.last_state = state
        self._reset_trajectory_state(self.control_side, q)
        self._last_command_monotonic_s = time.monotonic()

    def _enable_foc_hold(self) -> R1ArmSdkState:
        assert self.arm_bus is not None and self.command_sdk is not None
        deadline = time.monotonic() + self.foc_timeout
        while time.monotonic() < deadline:
            self.arm_bus.command_position(
                self.command_sdk, self.kp, self.kd, tau=self._tau_ff(self.command_sdk)
            )
            time.sleep(1.0 / self.rate_hz)
            state = self.arm_bus.read_state()
            self._check_feedback_range(state)
            self._check_faults(state, require_foc=False)
            if np.all(state.mode == 1):
                return state
        state = self.arm_bus.read_state()
        stopped = np.flatnonzero(state.mode != 1).tolist()
        raise TimeoutError(f"Joints did not enter FOC within {self.foc_timeout:.1f}s: {stopped}")

    def _on_start(self) -> None:
        if not self.motion_mode:
            print("R1Arm REAL DRY RUN: SDK is not opened and no motor command is sent")
            return

        port_path = Path(self.port)
        if not port_path.exists():
            raise FileNotFoundError(
                f"R1Arm serial port does not exist: {self.port}. "
                "Check /dev/serial/by-id and /dev/ttyUSB*."
            )

        try:
            self.arm_bus = self.bus_factory(self.bridge_library)
            self.arm_bus.open(self.port, self.baudrate, self.calibration_path)
            print("Probing all 7 joints with zero-gain FOC; keep the arm supported...")
            state = self._wait_for_feedback()
            self._seed_model_from_feedback(state)
            print("Seeding IK and command target from measured pose:")
            print("  model q =", np.array2string(self.arm_q(), precision=3))
            state = self._enable_foc_hold()
            self.last_state = state
            print("All 7 joints are in FOC and holding the measured startup pose")

            if self.home_on_start:
                # 从实测姿态软 blend 回 ARM_INIT（不是瞬时跳），到位后停在 home，
                # 等 grip 激活才开始跟踪。reset() 只设 _reset_blending 标志，
                # 真正的插值在主循环 step() 里逐帧推进，仍受速度/lead 限制。
                print(
                    f"Moving to ARM_INIT home pose (soft blend over "
                    f"{self.reset_blend_frames} frames); keep the arm clear..."
                )
                self.reset()

            if self.enable_dex1:
                from robot.eef.dex1_single import Dex1Single

                self.hand_ctrl = Dex1Single(
                    side=self.control_side,
                    interface=self.dex1_interface,
                    domain=self.dex1_domain,
                    grip_torque_limit=self.grip_torque_limit,
                )
        except Exception:
            self._safe_stop_arm()
            raise

    def _tau_ff(self, command_sdk: np.ndarray) -> np.ndarray | None:
        """Gravity feedforward at the *commanded* pose, SDK frame, clipped.

        与 goto_r1arm_pose 同一套：pinocchio 在指令姿态处计算（非实测——
        无反馈环），经 tau 通道随位置命令下发。消除稳态悬垂 g/kp，使位置环
        不再需要高增益维持精度，噪声/振铃随 kp 一起降。
        """
        if not self.gravity_ff:
            return None
        import pinocchio as pin

        q_full = np.zeros(self.model.nq, dtype=float)
        q_full[: len(ARM_JOINTS)] = sdk_to_model_q(np.asarray(command_sdk, dtype=float))
        tau = pin.computeGeneralizedGravity(self.model, self.data, q_full)
        tau = np.asarray(tau, dtype=float).reshape(-1)[: len(ARM_JOINTS)]
        return np.clip(model_to_sdk_q(tau), -self.tau_clip, self.tau_clip)

    def _reconcile_ik_seed(self, target_sdk: np.ndarray, command_sdk: np.ndarray) -> None:
        """Use the issued motor target as next cycle's IK solver seed.

        The shaped IK trajectory in ``self.q`` stays independent and
        continuous; only the solver posture seed follows the motor command.
        """
        gap = np.abs(command_sdk - target_sdk)
        drifted = np.flatnonzero(gap > self._IK_SEED_GAP_RAD)
        seed = self.q.copy()
        cmd_model = sdk_to_model_q(command_sdk)
        for i, idx in enumerate(self.arm_q_indices):
            seed[idx] = cmd_model[i]
        self._solver_seeds[self.control_side] = seed
        if self.debug and drifted.size:
            names = ", ".join(ARM_JOINTS[i] for i in drifted)
            print(f"[seed] updated IK solver seed from issued command ({names})")

    _IK_SEED_GAP_RAD = 0.05  # clip gap that triggers a seed re-anchor

    def _control_diagnostics(self) -> dict:
        trigger, valid, sampled_at = self._trigger_samples.get(
            self.control_side, (0.0, False, 0.0)
        )
        closed_ratio = float(np.clip(
            (self.config.dex1_open - self.grippers[self.control_side].current)
            / max(self.config.dex1_open - self.config.dex1_close, 1e-9),
            0.0, 1.0,
        ))
        diagnostics = {
            "trigger_value": float(trigger),
            "trigger_valid": bool(valid),
            "trigger_age_s": max(0.0, time.monotonic() - sampled_at) if sampled_at else None,
            "dex1_command_open_ratio": 1.0 - closed_ratio,
            "dex1_state_open_ratio": None,
            "dex1_tau_est": None,
            "dex1_feedback_valid": False,
            "dex1_feedback_age_s": None,
            "dex1_feedback_time_ns": None,
            "dex1_command_age_s": None,
            "dex1_command_time_ns": None,
            "dex1_command_count": 0,
            "dex1_publish_ok": None,
            "dex1_publish_fail_count": 0,
            "dex1_state_valid": False,
            "dex1_gripper_sample_time_ns": None,
        }
        if self.hand_ctrl is not None:
            try:
                data = self.hand_ctrl.get_record_data()
                diagnostics["dex1_command_open_ratio"] = float(data["hand_cmd"][0])
                diagnostics["dex1_state_open_ratio"] = float(data["hand_state"][0])
                diagnostics["dex1_tau_est"] = float(data["hand_torque"][0])
                diagnostics["dex1_feedback_age_s"] = data.get("feedback_age_s")
                diagnostics["dex1_feedback_time_ns"] = data.get("feedback_time_ns")
                diagnostics["dex1_feedback_valid"] = bool(data.get("feedback_valid", False))
                diagnostics["dex1_state_valid"] = bool(data.get("feedback_valid", False))
                diagnostics["dex1_gripper_sample_time_ns"] = data.get("feedback_time_ns")
                diagnostics["dex1_command_age_s"] = data.get("command_age_s")
                diagnostics["dex1_command_time_ns"] = data.get("command_time_ns")
                diagnostics["dex1_command_count"] = int(data.get("command_count", 0))
                diagnostics["dex1_publish_ok"] = data.get("command_publish_ok")
                diagnostics["dex1_publish_fail_count"] = int(
                    data.get("command_publish_fail_count", 0)
                )
            except Exception:
                pass
        return diagnostics

    def _feedback_stream_diagnostics(self, state: R1ArmSdkState, observed_ns: int) -> dict:
        counts = state.feedback_count
        if counts is None:
            return {
                "feedback_seq": None,
                "feedback_updated_since_read": None,
                "feedback_age_since_counter_change_s": None,
                "feedback_missed_response_count": None,
                "feedback_last_step_fresh": None,
                "feedback_freshness_available": False,
            }
        counts = np.asarray(counts, dtype=np.uint64).copy()
        first_observation = self._feedback_last_counts is None
        updated = None if first_observation else counts != self._feedback_last_counts
        if first_observation:
            self._feedback_last_seen_monotonic_ns = np.full(len(counts), observed_ns, dtype=np.int64)
        else:
            self._feedback_last_seen_monotonic_ns[updated] = observed_ns
        ages = None if first_observation else np.maximum(
            0, observed_ns - self._feedback_last_seen_monotonic_ns
        ).astype(float) / 1e9
        self._feedback_last_counts = counts
        return {
            "feedback_seq": counts,
            "feedback_updated_since_read": updated,
            "feedback_age_since_counter_change_s": ages,
            "feedback_missed_response_count": state.missed_response_count,
            "feedback_last_step_fresh": state.last_step_feedback,
            "feedback_freshness_available": True,
        }

    def _output_command(self) -> None:
        purpose_model = self.arm_q().copy()
        _, purpose_velocity, purpose_acceleration = self._action_trajectory_state(
            self.control_side, self.q
        )
        xr_raw = self._xr_pose_raw or {}
        xr_filtered = self._xr_pose_cache or {}
        xr_raw_pose = xr_raw.get(f"{self.control_side}_controller")
        xr_filtered_pose = xr_filtered.get(f"{self.control_side}_controller")
        diagnostics = self._control_diagnostics()
        if not self.motion_mode:
            self._control_seq += 1
            ee_intent = self._last_ee_intent.get(self.control_side, {})
            command_time_ns = time.time_ns()
            self.publish_control_sample({
                "control_seq": self._control_seq,
                "control_time_ns": command_time_ns,
                "command_time_ns": command_time_ns,
                "q_purpose": purpose_model,
                "dq_purpose": purpose_velocity,
                "ddq_purpose": purpose_acceleration,
                "q_ik_raw": self._last_ik_raw.get(self.control_side, purpose_model).copy(),
                "q_motor_target": purpose_model.copy(),
                "dq_target": purpose_velocity.copy(),
                "q_measured": purpose_model.copy(),
                "dq_measured": np.zeros(7),
                "tau_est": np.zeros(7),
                "feedback_time_ns": command_time_ns,
                "feedback_read_start_ns": command_time_ns,
                "feedback_read_end_ns": command_time_ns,
                "feedback_read_start_monotonic_ns": time.monotonic_ns(),
                "feedback_read_end_monotonic_ns": time.monotonic_ns(),
                "feedback_source": "dry_run_model",
                "feedback_seq": None,
                "feedback_updated_since_read": None,
                "feedback_age_since_counter_change_s": None,
                "feedback_missed_response_count": None,
                "feedback_last_step_fresh": None,
                "feedback_freshness_available": False,
                "feedback_ready": np.zeros(7, dtype=bool),
                "feedback_timeout": np.ones(7, dtype=np.uint8),
                "motor_mode": np.zeros(7, dtype=np.uint8),
                "motor_error_code": np.zeros(7, dtype=np.uint32),
                "ee_intent": ee_intent,
                "ik_solver": self._last_ik_solver.get(self.control_side, "hold"),
                "xr_raw": None if xr_raw_pose is None else xr_raw_pose.copy(),
                "xr_filtered": None if xr_filtered_pose is None else xr_filtered_pose.copy(),
                "xr_frame_stamp_ns": int(getattr(self, "_xr_last_stamp", 0) or 0),
                "xr_received_monotonic_ns": int(getattr(self, "_xr_received_monotonic_ns", 0)),
                "xr_valid": bool(getattr(self, "_xr_valid_last", False)),
                "xr_seq": int(getattr(self, "_xr_frame_seq", 0)),
                "control_mode": (
                    "home_reset" if self._reset_blending else
                    "tracking" if self.active else "hold"
                ),
                "grip_active": bool(self.track[self.control_side].active),
                **diagnostics,
            })
            if self.debug and time.monotonic() - self.last_report_time >= 1.0:
                self.last_report_time = time.monotonic()
                print("[dry-run] model q =", np.array2string(self.arm_q(), precision=3))
            return
        if self.arm_bus is None or self.command_sdk is None:
            raise RuntimeError("R1Arm SDK bus has not been started")
        if not Path(self.port).exists():
            raise RuntimeError(f"R1Arm serial device disappeared: {self.port}")

        feedback_read_start_ns = time.time_ns()
        feedback_read_start_monotonic_ns = time.monotonic_ns()
        state = self.arm_bus.read_state()
        feedback_read_end_ns = time.time_ns()
        feedback_read_end_monotonic_ns = time.monotonic_ns()
        feedback_time_ns = feedback_read_end_ns
        feedback_stream = self._feedback_stream_diagnostics(
            state, feedback_read_end_monotonic_ns
        )
        self._check_feedback_range(state)
        self._check_faults(state, require_foc=True)
        target_sdk = model_to_sdk_q(self.arm_q())
        now_mono_ns = time.monotonic_ns()
        now_mono_s = time.monotonic()
        command_dt = (
            now_mono_s - self._last_command_monotonic_s
            if self._last_command_monotonic_s is not None else 1.0 / self.rate_hz
        )
        command_dt = float(np.clip(command_dt, 1e-4, 0.1))
        command = limit_sdk_position_command(
            target_sdk,
            self.command_sdk,
            state.q,
            max_delta=self._sdk_max_delta(command_dt),
            max_lead=self.max_command_lead,
        )
        dq_target_sdk = (command - self.command_sdk) / command_dt
        command_send_start_ns = time.time_ns()
        command_send_start_monotonic_ns = time.monotonic_ns()
        self.arm_bus.command_position(
            command, self.kp, self.kd, tau=self._tau_ff(command), dq=dq_target_sdk
        )
        command_time_ns = time.time_ns()
        command_send_end_monotonic_ns = time.monotonic_ns()
        self._last_command_monotonic_s = now_mono_s
        self._last_command_dt = command_dt
        self.command_sdk = command
        self.last_state = state
        self._reconcile_ik_seed(target_sdk, command)
        purpose_model = self.arm_q().copy()
        _, purpose_velocity, purpose_acceleration = self._action_trajectory_state(
            self.control_side, self.q
        )
        self._control_seq += 1
        ee_intent = self._last_ee_intent.get(self.control_side, {})
        self.publish_control_sample({
            "control_seq": self._control_seq,
            "control_time_ns": command_time_ns,
            "command_time_ns": command_time_ns,
            "control_time_monotonic_ns": command_send_end_monotonic_ns,
            "command_send_start_ns": command_send_start_ns,
            "command_send_start_monotonic_ns": command_send_start_monotonic_ns,
            "command_send_end_monotonic_ns": command_send_end_monotonic_ns,
            "feedback_time_ns": feedback_time_ns,
            "feedback_read_start_ns": feedback_read_start_ns,
            "feedback_read_end_ns": feedback_read_end_ns,
            "feedback_read_start_monotonic_ns": feedback_read_start_monotonic_ns,
            "feedback_read_end_monotonic_ns": feedback_read_end_monotonic_ns,
            "feedback_source": "sdk_latest_cache",
            "feedback_ready": state.ready.copy(),
            "feedback_timeout": state.timeout.copy(),
            "motor_mode": state.mode.copy(),
            "motor_error_code": state.error_code.copy(),
            **feedback_stream,
            "q_purpose": purpose_model,
            "dq_purpose": purpose_velocity,
            "ddq_purpose": purpose_acceleration,
            "q_ik_raw": self._last_ik_raw.get(self.control_side, purpose_model).copy(),
            "q_motor_target": sdk_to_model_q(command),
            "dq_target": sdk_to_model_q(dq_target_sdk),
            "q_measured": sdk_to_model_q(state.q.copy()),
            "dq_measured": sdk_to_model_q(state.dq.copy()),
            "tau_est": sdk_to_model_q(state.tau_est.copy()),
            "ee_intent": ee_intent,
            "ik_solver": self._last_ik_solver.get(self.control_side, "hold"),
            "xr_raw": None if xr_raw_pose is None else xr_raw_pose.copy(),
            "xr_filtered": None if xr_filtered_pose is None else xr_filtered_pose.copy(),
            "xr_frame_stamp_ns": int(getattr(self, "_xr_last_stamp", 0) or 0),
            "xr_received_monotonic_ns": int(getattr(self, "_xr_received_monotonic_ns", 0)),
            "xr_valid": bool(getattr(self, "_xr_valid_last", False)),
            "xr_seq": int(getattr(self, "_xr_frame_seq", 0)),
            "control_mode": (
                "home_reset" if self._reset_blending else
                "tracking" if self.active else "hold"
            ),
            "grip_active": bool(self.track[self.control_side].active),
            **diagnostics,
        })
        if self.debug and time.monotonic() - self.last_report_time >= 1.0:
            self.last_report_time = time.monotonic()
            error = command - state.q
            print(
                "[real] measured_sdk=",
                np.array2string(state.q, precision=3),
                " command_sdk=",
                np.array2string(command, precision=3),
                " error=",
                np.array2string(error, precision=3),
            )

    def _sdk_max_delta(self, dt: float) -> float:
        return self.max_delta

    def _safe_stop_arm(self) -> None:
        bus = self.arm_bus
        if bus is None:
            return
        try:
            if bus.is_open:
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    bus.stop()
                    time.sleep(1.0 / self.rate_hz)
        except Exception as exc:
            print(f"Warning: R1Arm stop command failed: {exc}")
        finally:
            try:
                bus.close()
            except Exception as exc:
                print(f"Warning: R1Arm SDK close failed: {exc}")
            self.arm_bus = None

    def _on_stop(self) -> None:
        self._safe_stop_arm()
        super()._on_stop()


class R1ArmUnshapedIKTeleopController(R1ArmRealTeleopController):
    """Direct IK tracking; home and grip-release transitions stay shaped."""

    def __init__(self, config: R1ArmConfig, *, max_joint_speed: float = 0.5, **kwargs) -> None:
        # Move the speed constraint into the IK solvers instead of post-IK shaping.
        config.ik_qp_velocity_limit = float(max_joint_speed)
        super().__init__(config, max_joint_speed=max_joint_speed, **kwargs)
        self._dls_max_step = self.max_delta

    def _solve(self, side: str, target_pos: np.ndarray, target_quat: np.ndarray | None) -> None:
        q = self._q_for_side(side)
        solver_seed = self._solver_seeds.get(side, q)
        solver = self._placo_for_side(side)
        q_next = None
        solver_name = "dls"
        if self.use_placo and solver is not None:
            sol = solver.solve(solver_seed, target_pos, target_quat)
            if sol is not None:
                q_next = q.copy()
                for idx in self.arm_q_indices:
                    q_next[idx] = sol[idx]
                solver_name = "placo"
        if q_next is None:
            q_next = self._solve_dls(solver_seed, target_pos, target_quat)
        q_next = clamp_configuration(self.model, q_next)
        dt = min(max(float(getattr(self, "_step_dt", 1.0 / self.rate_hz)), 1e-4), 0.1)
        self._ensure_trajectory_state(side, q)
        q_cmd = np.asarray([q_next[idx] for idx in self.arm_q_indices], dtype=float)
        v_cmd = (q_cmd - self._trajectory_q_cmd[side]) / dt
        a_cmd = np.clip(
            (v_cmd - self._trajectory_v_cmd[side]) / dt,
            -self.config.ik_max_acc_per_s2,
            self.config.ik_max_acc_per_s2,
        )
        # Keep transition state current for a smooth release, without shaping
        # the active IK result itself.
        self._trajectory_q_cmd[side] = q_cmd
        self._trajectory_v_cmd[side] = v_cmd
        self._trajectory_a_cmd[side] = a_cmd
        self._set_q_for_side(side, q_next)
        self._solver_seeds[side] = q_next.copy()
        self._last_ik_raw[side] = q_cmd.copy()
        self._last_ik_solver[side] = solver_name


class R1ArmQPConstrainedIKTeleopController(R1ArmRealTeleopController):
    """Active IK with optional velocity/acceleration/jerk QP bounds."""

    def __init__(
        self, config: R1ArmConfig, *, max_joint_speed: float = 1.0,
        qp_jerk_limit: float | None = None, **kwargs,
    ) -> None:
        config.ik_qp_velocity_limit = float(max_joint_speed)
        config.ik_qp_acceleration_limit = float(config.ik_max_acc_per_s2)
        config.ik_qp_jerk_limit = (
            float(qp_jerk_limit) if qp_jerk_limit is not None and qp_jerk_limit > 0.0 else None
        )
        super().__init__(config, max_joint_speed=max_joint_speed, **kwargs)
        per_joint = config.ik_joint_velocity_limits or {}
        self._qp_velocity_limits = np.array([
            float(per_joint.get(name, max_joint_speed)) for name in ARM_JOINTS
        ])
        self._qp_velocity_limits = np.where(
            self._qp_velocity_limits > 0.0,
            self._qp_velocity_limits,
            float(max_joint_speed),
        )
        self._qp_acceleration_limit = float(config.ik_max_acc_per_s2)
        self._qp_jerk_limit = config.ik_qp_jerk_limit
        self._qp_action_state: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self._qp_last_motor_q: np.ndarray | None = None
        self._qp_last_motor_v = np.zeros(7)

    def _seed_model_from_feedback(self, state: R1ArmSdkState) -> None:
        super()._seed_model_from_feedback(state)
        self._qp_last_motor_q = sdk_to_model_q(state.q.copy())
        self._qp_last_motor_v = np.zeros(7)

    def _action_trajectory_state(
        self, side: str, q: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if self._last_ik_solver.get(side) in {
            "placo_va_qp", "dls_va_projected", "placo_vaj_qp", "dls_vaj_projected"
        }:
            state = self._qp_action_state.get(side)
            if state is not None:
                return tuple(value.copy() for value in state)
        return super()._action_trajectory_state(side, q)

    def _solve(self, side: str, target_pos: np.ndarray, target_quat: np.ndarray | None) -> None:
        q = self._q_for_side(side)
        solver_seed = self._solver_seeds.get(side, q).copy()
        self._ensure_trajectory_state(side, q)
        dt = float(np.clip(getattr(self, "_step_dt", 1.0 / self.rate_hz), 1e-4, 0.1))
        previous_velocity = self._trajectory_v_cmd[side].copy()
        solver = self._placo_for_side(side)
        q_next = None
        q_raw = None
        suffix = "vaj" if self._qp_jerk_limit is not None else "va"
        solver_name = f"dls_{suffix}_projected"
        previous_acceleration = self._trajectory_a_cmd[side].copy()

        if self.use_placo and solver is not None:
            sol = solver.solve(
                solver_seed, target_pos, target_quat,
                dt=dt, previous_velocity=previous_velocity,
                previous_acceleration=previous_acceleration,
            )
            if sol is not None:
                q_next = solver_seed.copy()
                for idx in self.arm_q_indices:
                    q_next[idx] = sol[idx]
                q_raw = q_next.copy()
                solver_name = f"placo_{suffix}_qp"

        if q_next is None:
            lower, upper = joint_delta_bounds(
                previous_velocity, self._qp_velocity_limits,
                self._qp_acceleration_limit, dt,
                previous_acceleration=previous_acceleration,
                jerk_limit=self._qp_jerk_limit,
            )
            q_raw = self._solve_dls(solver_seed, target_pos, target_quat)
            q_next = solver_seed.copy()
            for i, idx in enumerate(self.arm_q_indices):
                delta = float(q_raw[idx] - solver_seed[idx])
                q_next[idx] = solver_seed[idx] + float(np.clip(delta, lower[i], upper[i]))

        q_next = clamp_configuration(self.model, q_next)
        q_cmd = np.asarray([q_next[idx] for idx in self.arm_q_indices], dtype=float)
        q_prev = np.asarray([solver_seed[idx] for idx in self.arm_q_indices], dtype=float)
        v_next = (q_cmd - q_prev) / dt
        a_next = (v_next - previous_velocity) / dt
        self._trajectory_q_cmd[side] = q_cmd.copy()
        self._trajectory_v_cmd[side] = v_next.copy()
        self._trajectory_a_cmd[side] = a_next.copy()
        self._qp_action_state[side] = (q_cmd.copy(), v_next.copy(), a_next.copy())
        self._set_q_for_side(side, q_next)
        self._solver_seeds[side] = q_next.copy()
        self._last_ik_raw[side] = np.asarray(
            [q_raw[idx] for idx in self.arm_q_indices], dtype=float
        )
        self._last_ik_solver[side] = solver_name

    def _reconcile_ik_seed(self, target_sdk: np.ndarray, command_sdk: np.ndarray) -> None:
        command_q = sdk_to_model_q(command_sdk)
        dt = float(np.clip(self._last_command_dt, 1e-4, 0.1))
        if self._qp_last_motor_q is None:
            command_v = np.zeros(7)
        else:
            command_v = (command_q - self._qp_last_motor_q) / dt
        command_a = (command_v - self._qp_last_motor_v) / dt
        self._qp_last_motor_q = command_q.copy()
        self._qp_last_motor_v = command_v.copy()

        seed = self.q.copy()
        for idx, value in zip(self.arm_q_indices, command_q):
            seed[idx] = value
        self._solver_seeds[self.control_side] = seed
        self._trajectory_q_cmd[self.control_side] = command_q.copy()
        self._trajectory_v_cmd[self.control_side] = command_v.copy()
        self._trajectory_a_cmd[self.control_side] = command_a.copy()

    def _sdk_max_delta(self, dt: float) -> float:
        return self.max_joint_speed * dt
