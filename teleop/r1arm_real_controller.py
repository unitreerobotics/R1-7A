"""Standalone R1Arm real-robot output for the proven simulation teleop core."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Callable

import numpy as np

from robot.r1arm_config import ARM_JOINTS, ARM_LIMITS, R1ArmConfig
from robot.r1arm_sdk import R1ArmSdkBus, R1ArmSdkState, model_to_sdk_q, sdk_to_model_q
from teleop.r1arm_controller_base import R1ArmTeleopControllerBase


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
    bounded = np.clip(target, SDK_LOWER, SDK_UPPER)
    slewed = previous + np.clip(bounded - previous, -abs(max_delta), abs(max_delta))
    return measured + np.clip(slewed - measured, -abs(max_lead), abs(max_lead))


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
        kp: float = 40.0,
        kd: float = 0.5,
        rate: float = 50.0,
        max_joint_speed: float = 0.5,
        max_command_lead: float = 0.15,
        feedback_timeout: float = 3.0,
        foc_timeout: float = 1.0,
        home_on_start: bool = False,
        mock_xr: bool = False,
        debug: bool = False,
        scale_factor: float | None = None,
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
        self.kp = float(kp)
        self.kd = float(kd)
        self.rate_hz = float(rate)
        self.max_delta = float(max_joint_speed) / self.rate_hz
        self.max_command_lead = float(max_command_lead)
        self.feedback_timeout = float(feedback_timeout)
        self.foc_timeout = float(foc_timeout)
        self.home_on_start = bool(home_on_start)
        self.bus_factory = bus_factory
        self.arm_bus: R1ArmSdkBus | None = None
        self.command_sdk: np.ndarray | None = None
        self.last_state: R1ArmSdkState | None = None
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
        if self.placo_solver is not None:
            self.placo_solver.set_regularization_reference(self.q)
        for track in self.track.values():
            track.clear()
        self.active = False
        self.command_sdk = state.q.copy()
        self.last_state = state

    def _enable_foc_hold(self) -> R1ArmSdkState:
        assert self.arm_bus is not None and self.command_sdk is not None
        deadline = time.monotonic() + self.foc_timeout
        while time.monotonic() < deadline:
            self.arm_bus.command_position(self.command_sdk, self.kp, self.kd)
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

    def _output_command(self) -> None:
        if not self.motion_mode:
            if self.debug and time.monotonic() - self.last_report_time >= 1.0:
                self.last_report_time = time.monotonic()
                print("[dry-run] model q =", np.array2string(self.arm_q(), precision=3))
            return
        if self.arm_bus is None or self.command_sdk is None:
            raise RuntimeError("R1Arm SDK bus has not been started")
        if not Path(self.port).exists():
            raise RuntimeError(f"R1Arm serial device disappeared: {self.port}")

        state = self.arm_bus.read_state()
        self._check_feedback_range(state)
        self._check_faults(state, require_foc=True)
        target_sdk = model_to_sdk_q(self.arm_q())
        command = limit_sdk_position_command(
            target_sdk,
            self.command_sdk,
            state.q,
            max_delta=self.max_delta,
            max_lead=self.max_command_lead,
        )
        self.arm_bus.command_position(command, self.kp, self.kd)
        self.command_sdk = command
        self.last_state = state

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
