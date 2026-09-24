"""ctypes wrapper for the standalone R1Arm x86 C++ SDK bridge."""

from __future__ import annotations

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


@dataclass(frozen=True)
class R1ArmSdkState:
    q: np.ndarray
    dq: np.ndarray
    tau_est: np.ndarray
    mode: np.ndarray
    timeout: np.ndarray
    error_code: np.ndarray
    ready: np.ndarray


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

    def command_position(self, q: np.ndarray, kp: float, kd: float) -> None:
        values = np.ascontiguousarray(q, dtype=np.float32).reshape(-1)
        if values.size != JOINT_COUNT or not np.all(np.isfinite(values)):
            raise ValueError(f"Position command must be {JOINT_COUNT} finite values")
        self._call(
            self._lib.r1arm_bridge_command_position,
            self._handle,
            values.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            float(kp),
            float(kd),
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
        return R1ArmSdkState(
            q=q.astype(float),
            dq=dq.astype(float),
            tau_est=tau.astype(float),
            mode=mode,
            timeout=timeout,
            error_code=error_code,
            ready=ready.astype(bool),
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
