"""ctypes wrapper for the standalone R1Arm x86 C++ SDK bridge."""

from __future__ import annotations

import argparse
import ctypes
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np


JOINT_COUNT = 7
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SDK_ROOT = Path(
    os.environ.get("R1ARM_SDK_ROOT", "/home/ycb410/nt/R1_arm_SDK_x86/r1_arm")
)
DEFAULT_BRIDGE_LIBRARY = PROJECT_ROOT / "build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so"
MODEL_TO_SDK_SIGN = np.array([1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0])


def model_to_sdk_q(q: np.ndarray) -> np.ndarray:
    """Convert the simulation URDF convention to the SDK URDF convention."""
    values = np.asarray(q, dtype=float).reshape(-1)
    if values.size != JOINT_COUNT:
        raise ValueError(f"Expected {JOINT_COUNT} arm joints, got {values.size}")
    return values * MODEL_TO_SDK_SIGN


def sdk_to_model_q(q: np.ndarray) -> np.ndarray:
    """Convert SDK feedback to the simulation/IK URDF convention."""
    return model_to_sdk_q(q)  # The sign transform is its own inverse.


def validate_calibration_file(path: str | Path) -> Path:
    """Validate before calling the SDK, whose current loader exits on bad input."""
    calibration = Path(path).expanduser().resolve()
    if not calibration.is_file():
        raise FileNotFoundError(f"R1Arm calibration file not found: {calibration}")
    seen: set[int] = set()
    for line_no, raw in enumerate(calibration.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) not in (3, 4):
            raise ValueError(f"Bad calibration line {line_no}: {raw!r}")
        joint = int(fields[0])
        direction = float(fields[1])
        scale = float(fields[2]) if len(fields) == 4 else 1.0
        if joint not in range(JOINT_COUNT) or joint in seen:
            raise ValueError(f"Bad/duplicate joint index on calibration line {line_no}: {joint}")
        if abs(direction) < 1e-6 or abs(scale) < 1e-6:
            raise ValueError(f"Zero direction/scale on calibration line {line_no}")
        float(fields[-1])
        seen.add(joint)
    if seen != set(range(JOINT_COUNT)):
        raise ValueError(f"Calibration must contain joints 0..6; found {sorted(seen)}")
    return calibration


def parse_gain(text: str, name: str = "gain") -> "float | np.ndarray":
    """argparse type: scalar ("100") or 7 per-joint values ("100,100,100,100,50,50,50").

    Returns a float for scalar input, an ndarray of 7 for per-joint input. The
    scalar form keeps the legacy broadcast ABI; the per-joint form routes
    through ``command_position``'s v2 ABI.
    """
    try:
        values = [float(v) for v in text.split(",")]
    except ValueError:
        raise argparse.ArgumentTypeError(f"{name} must be numbers separated by ','; got {text!r}")
    if not values or any(not np.isfinite(v) for v in values):
        raise argparse.ArgumentTypeError(f"{name} contains non-finite values: {text!r}")
    if len(values) == 1:
        return values[0]
    if len(values) != JOINT_COUNT:
        raise argparse.ArgumentTypeError(
            f"{name} must be 1 scalar or {JOINT_COUNT} comma-separated values; got {len(values)}"
        )
    return np.asarray(values, dtype=float)


def format_gain(gain: "float | np.ndarray") -> str:
    """One-line human-readable form of a scalar or per-joint gain."""
    if np.isscalar(gain):
        return f"{float(gain):g}"
    values = np.asarray(gain, dtype=float).reshape(-1)
    if values.size == 1:
        return f"{float(values[0]):g}"
    return "per-joint " + ",".join(f"{v:g}" for v in values)


def as_gain_array(gain: "float | np.ndarray") -> np.ndarray:
    """Broadcast a scalar-or-per-joint gain to a 7-vector (for metadata math)."""
    values = np.asarray(gain, dtype=float).reshape(-1)
    if values.size == 1:
        return np.full(JOINT_COUNT, float(values[0]))
    if values.size != JOINT_COUNT:
        raise ValueError(f"gain must be 1 scalar or {JOINT_COUNT} values; got {values.size}")
    return values

@dataclass(frozen=True)
class R1ArmSdkState:
    q: np.ndarray
    dq: np.ndarray
    tau_est: np.ndarray
    mode: np.ndarray
    timeout: np.ndarray
    error_code: np.ndarray
    ready: np.ndarray
    feedback_count: np.ndarray | None = None
    missed_response_count: np.ndarray | None = None
    last_step_feedback: np.ndarray | None = None


class R1ArmSdkBus:
    """Python owner for one ``ArmMotorBus`` instance."""

    _ERROR_CAPACITY = 512

    def __init__(self, library_path: str | Path | None = None) -> None:
        configured = library_path or os.environ.get("R1ARM_BRIDGE_LIB") or DEFAULT_BRIDGE_LIBRARY
        self.library_path = Path(configured).expanduser().resolve()
        if not self.library_path.is_file():
            raise FileNotFoundError(
                f"R1Arm bridge library not found: {self.library_path}\n"
                "Build it first with scripts/build_r1arm_sdk_bridge.py"
            )
        self._lib = ctypes.CDLL(str(self.library_path))
        self._configure_abi()
        self._handle = self._lib.r1arm_bridge_create()
        if not self._handle:
            raise RuntimeError("Could not allocate R1Arm SDK bridge")
        self._closed = False

    def _configure_abi(self) -> None:
        void_p = ctypes.c_void_p
        char_p = ctypes.c_char_p
        error_args = [ctypes.POINTER(ctypes.c_char), ctypes.c_int]
        float_p = ctypes.POINTER(ctypes.c_float)
        u8_p = ctypes.POINTER(ctypes.c_uint8)
        u32_p = ctypes.POINTER(ctypes.c_uint32)

        self._lib.r1arm_bridge_version.argtypes = []
        self._lib.r1arm_bridge_version.restype = char_p
        self._lib.r1arm_bridge_create.argtypes = []
        self._lib.r1arm_bridge_create.restype = void_p
        self._lib.r1arm_bridge_destroy.argtypes = [void_p]
        self._lib.r1arm_bridge_destroy.restype = None
        self._lib.r1arm_bridge_open.argtypes = [
            void_p,
            char_p,
            ctypes.c_int,
            char_p,
            *error_args,
        ]
        self._lib.r1arm_bridge_open.restype = ctypes.c_int
        self._lib.r1arm_bridge_is_open.argtypes = [void_p]
        self._lib.r1arm_bridge_is_open.restype = ctypes.c_int
        self._lib.r1arm_bridge_probe.argtypes = [void_p, *error_args]
        self._lib.r1arm_bridge_probe.restype = ctypes.c_int
        self._lib.r1arm_bridge_command_position.argtypes = [
            void_p,
            float_p,
            ctypes.c_float,
            ctypes.c_float,
            *error_args,
        ]
        self._lib.r1arm_bridge_command_position.restype = ctypes.c_int
        self._lib.r1arm_bridge_command_position_v2.argtypes = [
            void_p,
            float_p,
            float_p,
            float_p,
            float_p,
            *error_args,
        ]
        self._lib.r1arm_bridge_command_position_v2.restype = ctypes.c_int
        self._lib.r1arm_bridge_command_position_v3.argtypes = [
            void_p, float_p, float_p, float_p, float_p, float_p, *error_args,
        ]
        self._lib.r1arm_bridge_command_position_v3.restype = ctypes.c_int
        self._lib.r1arm_bridge_read_state.argtypes = [
            void_p,
            float_p,
            float_p,
            float_p,
            u8_p,
            u8_p,
            u32_p,
            u8_p,
            *error_args,
        ]
        self._lib.r1arm_bridge_read_state.restype = ctypes.c_int
        self._read_feedback_diagnostics = getattr(
            self._lib, "r1arm_bridge_read_feedback_diagnostics", None
        )
        if self._read_feedback_diagnostics is not None:
            self._read_feedback_diagnostics.argtypes = [
                void_p, ctypes.POINTER(ctypes.c_uint64), ctypes.POINTER(ctypes.c_uint64),
                u8_p, u8_p, *error_args,
            ]
            self._read_feedback_diagnostics.restype = ctypes.c_int
        self._lib.r1arm_bridge_stop.argtypes = [void_p, *error_args]
        self._lib.r1arm_bridge_stop.restype = ctypes.c_int
        self._lib.r1arm_bridge_close.argtypes = [void_p, *error_args]
        self._lib.r1arm_bridge_close.restype = ctypes.c_int

    def _call(self, function, *args) -> None:
        error = ctypes.create_string_buffer(self._ERROR_CAPACITY)
        result = function(*args, error, len(error))
        if result != 0:
            message = error.value.decode("utf-8", errors="replace") or "unknown bridge error"
            raise RuntimeError(message)

    @property
    def version(self) -> str:
        return self._lib.r1arm_bridge_version().decode("ascii")

    @property
    def is_open(self) -> bool:
        return bool(self._handle and self._lib.r1arm_bridge_is_open(self._handle))

    def open(self, port: str, baudrate: int, calibration_path: str | Path) -> None:
        calibration = validate_calibration_file(calibration_path)
        self._call(
            self._lib.r1arm_bridge_open,
            self._handle,
            os.fsencode(port),
            int(baudrate),
            os.fsencode(calibration),
        )

    def probe_once(self) -> None:
        self._call(self._lib.r1arm_bridge_probe, self._handle)

    def command_position(
        self,
        q: np.ndarray,
        kp: "float | np.ndarray",
        kd: "float | np.ndarray",
        tau: "np.ndarray | None" = None,
        dq: "np.ndarray | None" = None,
    ) -> None:
        """Position command with scalar or per-joint gains (and optional tau ff).

        Scalar kp/kd take the legacy broadcast ABI; arrays/tau use v2. Supplying
        desired velocity selects v3 and forwards q/dq/kp/kd/tau together.
        """
        values = np.ascontiguousarray(q, dtype=np.float32).reshape(-1)
        if values.size != JOINT_COUNT or not np.all(np.isfinite(values)):
            raise ValueError(f"Position command must be {JOINT_COUNT} finite values")

        def joint_array(gain, name: str) -> np.ndarray:
            if np.isscalar(gain):
                gain = np.full(JOINT_COUNT, float(gain))
            array = np.ascontiguousarray(gain, dtype=np.float32).reshape(-1)
            if array.size != JOINT_COUNT or not np.all(np.isfinite(array)):
                raise ValueError(f"{name} must be a scalar or {JOINT_COUNT} finite values")
            return array

        if np.isscalar(kp) and np.isscalar(kd) and tau is None and dq is None:
            self._call(
                self._lib.r1arm_bridge_command_position,
                self._handle,
                values.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                float(kp),
                float(kd),
            )
            return

        kp_array = joint_array(kp, "kp")
        kd_array = joint_array(kd, "kd")
        tau_array = None
        if tau is not None:
            tau_array = joint_array(tau, "tau")
        if dq is not None:
            dq_array = joint_array(dq, "dq")
            self._call(
                self._lib.r1arm_bridge_command_position_v3,
                self._handle,
                values.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                dq_array.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                kp_array.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                kd_array.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                tau_array.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
                if tau_array is not None else None,
            )
            return
        self._call(
            self._lib.r1arm_bridge_command_position_v2,
            self._handle,
            values.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            kp_array.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            kd_array.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            tau_array.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            if tau_array is not None
            else None,
        )

    def read_state(self) -> R1ArmSdkState:
        q = np.empty(JOINT_COUNT, dtype=np.float32)
        dq = np.empty(JOINT_COUNT, dtype=np.float32)
        tau = np.empty(JOINT_COUNT, dtype=np.float32)
        mode = np.empty(JOINT_COUNT, dtype=np.uint8)
        timeout = np.empty(JOINT_COUNT, dtype=np.uint8)
        error_code = np.empty(JOINT_COUNT, dtype=np.uint32)
        ready = np.empty(JOINT_COUNT, dtype=np.uint8)
        self._call(
            self._lib.r1arm_bridge_read_state,
            self._handle,
            q.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            dq.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            tau.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            mode.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            timeout.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            error_code.ctypes.data_as(ctypes.POINTER(ctypes.c_uint32)),
            ready.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        )
        feedback_count = missed_count = last_step_feedback = None
        if self._read_feedback_diagnostics is not None:
            feedback_count = np.empty(JOINT_COUNT, dtype=np.uint64)
            missed_count = np.empty(JOINT_COUNT, dtype=np.uint64)
            last_step_feedback = np.empty(JOINT_COUNT, dtype=np.uint8)
            supported = np.zeros(1, dtype=np.uint8)
            self._call(
                self._read_feedback_diagnostics,
                self._handle,
                feedback_count.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64)),
                missed_count.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64)),
                last_step_feedback.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                supported.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            )
            if not supported[0]:
                feedback_count = missed_count = last_step_feedback = None
        return R1ArmSdkState(
            q=q.astype(float),
            dq=dq.astype(float),
            tau_est=tau.astype(float),
            mode=mode,
            timeout=timeout,
            error_code=error_code,
            ready=ready.astype(bool),
            feedback_count=feedback_count,
            missed_response_count=missed_count,
            last_step_feedback=(
                last_step_feedback.astype(bool) if last_step_feedback is not None else None
            ),
        )

    def stop(self) -> None:
        if self._handle and self.is_open:
            self._call(self._lib.r1arm_bridge_stop, self._handle)

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._handle and self.is_open:
                self._call(self._lib.r1arm_bridge_close, self._handle)
        finally:
            if self._handle:
                self._lib.r1arm_bridge_destroy(self._handle)
                self._handle = None
            self._closed = True

    def __enter__(self) -> "R1ArmSdkBus":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
