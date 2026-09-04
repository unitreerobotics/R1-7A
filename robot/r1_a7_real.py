"""Low-level DDS bridge for R1-A7 real-robot teleoperation.

Migrated from ``teleop/robot_control_r1/r1_a7_real.py`` in the original
``xr_teleoperate_r1a7`` repo.  Publishes ``rt/lowcmd`` with CRC and subscribes
``rt/lowstate``.  The real-robot version commands the R1-A7 upper-body motor
slots visible in lowstate: 7 DoF per arm plus head pitch/yaw (no waist).
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Event, Lock, Thread

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SDK_PATH = ROOT / "third_party" / "unitree_sdk2_python"
if str(SDK_PATH) not in sys.path:
    sys.path.insert(0, str(SDK_PATH))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_, LowCmd_, LowState_  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402


HG_CMD_TOPIC = "rt/lowcmd"
HG_STATE_TOPIC = "rt/lowstate"
HG_IMU_TORSO_TOPIC = "rt/secondary_imu"

# Verified R1-A7 low-level upper-body subset. These are IDL motor_state indices.
REAL_JOINTS: tuple[tuple[str, int, float, float], ...] = (
    ("left_shoulder_pitch_joint", 15, 100.0, 2.0),
    ("left_shoulder_roll_joint", 16, 100.0, 2.0),
    ("left_shoulder_yaw_joint", 17, 100.0, 2.0),
    ("left_elbow_joint", 18, 100.0, 2.0),
    ("left_wrist_roll_joint", 19, 50.0, 2.0),
    ("left_wrist_pitch_joint", 20, 50.0, 2.0),
    ("left_wrist_yaw_joint", 21, 50.0, 2.0),
    ("right_shoulder_pitch_joint", 22, 100.0, 2.0),
    ("right_shoulder_roll_joint", 23, 100.0, 2.0),
    ("right_shoulder_yaw_joint", 24, 100.0, 2.0),
    ("right_elbow_joint", 25, 100.0, 2.0),
    ("right_wrist_roll_joint", 26, 50.0, 2.0),
    ("right_wrist_pitch_joint", 27, 50.0, 2.0),
    ("right_wrist_yaw_joint", 28, 50.0, 2.0),
    ("head_pitch_joint", 29, 50.0, 2.0),
    ("head_yaw_joint", 30, 10.0, 0.1),
)


@dataclass
class R1A7LowCmdConfig:
    interface: str = "eth0"
    domain: int = 0
    control_dt: float = 0.02
    report_period: float = 1.0
    max_joint_delta: float = 0.04
    enable_motion: bool = True
    no_confirm: bool = True
    skip_motion_switcher: bool = False


class R1A7LowCmdTeleop:
    """Threaded low-level writer used by the real teleop controller."""

    def __init__(self, config: R1A7LowCmdConfig, joint_indices: dict[str, int]) -> None:
        self.config = config
        self.joint_indices = joint_indices
        self.crc = CRC()
        self.stop_event = Event()
        self.lock = Lock()
        self.low_state: LowState_ | None = None
        self.mode_machine = 0
        self.command_q = np.zeros(len(REAL_JOINTS), dtype=float)
        self.target_q = np.zeros(len(REAL_JOINTS), dtype=float)
        self.last_lowstate_time: float | None = None
        self.last_publish_time: float | None = None
        self.last_report_time = 0.0
        self.lowcmd_publisher = None
        self.writer_thread: Thread | None = None
        self.pin_q_indices = [joint_indices.get(name) for name, _, _, _ in REAL_JOINTS]

    def init_dds(self) -> None:
        print(f"Initializing real DDS domain={self.config.domain}, interface={self.config.interface}")
        ChannelFactoryInitialize(self.config.domain, self.config.interface)
        if self.config.enable_motion and not self.config.skip_motion_switcher:
            self.release_motion_mode()
        self.lowstate_subscriber = ChannelSubscriber(HG_STATE_TOPIC, LowState_)
        self.lowstate_subscriber.Init(self.lowstate_handler, 1)
        self.imutorso_subscriber = ChannelSubscriber(HG_IMU_TORSO_TOPIC, IMUState_)
        self.imutorso_subscriber.Init(lambda msg: None, 1)
        if self.config.enable_motion:
            self.lowcmd_publisher = ChannelPublisher(HG_CMD_TOPIC, LowCmd_)
            self.lowcmd_publisher.Init()
            print(f"Publishing enabled on {HG_CMD_TOPIC}")
        else:
            print("REAL DRY RUN: not publishing lowcmd.")

    def release_motion_mode(self) -> None:
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

            msc = MotionSwitcherClient()
            msc.SetTimeout(5.0)
            msc.Init()
            for _ in range(3):
                _, result = msc.CheckMode()
                name = result.get("name", "") if result else ""
                if not name:
                    return
                print(f"Current motion mode: {name}; requesting ReleaseMode()")
                msc.ReleaseMode()
                time.sleep(1.0)
        except Exception as exc:
            print(f"Motion switcher release skipped/failed: {exc}")

    def lowstate_handler(self, msg: LowState_) -> None:
        try:
            if msg.crc != self.crc.Crc(msg):
                print("[real-dds] lowstate CRC mismatch; sample ignored")
                return
        except Exception:
            pass
        with self.lock:
            self.low_state = msg
            self.mode_machine = int(msg.mode_machine)
            self.last_lowstate_time = time.monotonic()

    def wait_for_lowstate(self) -> None:
        print("Waiting for real robot lowstate...")
        last_wait = time.monotonic()
        while True:
            with self.lock:
                ready = self.low_state is not None
            if ready:
                self.seed_from_lowstate()
                return
            if time.monotonic() - last_wait > 5.0:
                print("Still waiting for real robot lowstate. Check interface/DDS/power.")
                last_wait = time.monotonic()
            time.sleep(0.05)

    def seed_from_lowstate(self) -> None:
        with self.lock:
            if self.low_state is None:
                return
            values = [float(self.low_state.motor_state[idl_i].q) for _, idl_i, _, _ in REAL_JOINTS]
            self.command_q[:] = values
            self.target_q[:] = values

    def apply_lowstate_to_pin_q(self, q: np.ndarray) -> None:
        with self.lock:
            msg = self.low_state
            if msg is None:
                return
            for local_i, q_idx in enumerate(self.pin_q_indices):
                if q_idx is None:
                    continue
                _, idl_i, _, _ = REAL_JOINTS[local_i]
                q[q_idx] = float(msg.motor_state[idl_i].q)

    def update_target_from_pin_q(self, q: np.ndarray) -> None:
        target = self.target_q.copy()
        for local_i, q_idx in enumerate(self.pin_q_indices):
            if q_idx is None:
                continue
            target[local_i] = float(q[q_idx])
        with self.lock:
            self.target_q[:] = target

    def start(self) -> None:
        self.writer_thread = Thread(target=self._writer_loop, daemon=True)
        self.writer_thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.writer_thread is not None:
            self.writer_thread.join(timeout=1.0)

    def _writer_loop(self) -> None:
        next_t = time.monotonic()
        while not self.stop_event.is_set():
            self.write_once()
            next_t += self.config.control_dt
            time.sleep(max(0.0, next_t - time.monotonic()))

    def write_once(self) -> None:
        with self.lock:
            if self.low_state is None:
                return
            delta = np.clip(
                self.target_q - self.command_q,
                -abs(self.config.max_joint_delta),
                abs(self.config.max_joint_delta),
            )
            self.command_q += delta
            command_q = self.command_q.copy()
            mode_machine = self.mode_machine

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = int(mode_machine)
        for local_i, (_, idl_i, kp, kd) in enumerate(REAL_JOINTS):
            motor_cmd = cmd.motor_cmd[idl_i]
            motor_cmd.mode = 1
            motor_cmd.tau = 0.0
            motor_cmd.q = float(command_q[local_i])
            motor_cmd.dq = 0.0
            motor_cmd.kp = float(kp)
            motor_cmd.kd = float(kd)
        cmd.crc = self.crc.Crc(cmd)
        if self.config.enable_motion and self.lowcmd_publisher is not None:
            self.lowcmd_publisher.Write(cmd)
            self.last_publish_time = time.monotonic()
        self.maybe_report(command_q)

    def maybe_report(self, command_q: np.ndarray) -> None:
        now = time.monotonic()
        if now - self.last_report_time < self.config.report_period:
            return
        self.last_report_time = now
        with self.lock:
            msg = self.low_state
        if msg is None:
            return
        actual = [float(msg.motor_state[idl_i].q) for _, idl_i, _, _ in REAL_JOINTS]
        print(
            "\nreal lowcmd: "
            f"L_elbow cmd/act={command_q[3]:+.3f}/{actual[3]:+.3f} "
            f"R_elbow cmd/act={command_q[10]:+.3f}/{actual[10]:+.3f} "
            f"L_wrist r/p/y={command_q[4]:+.3f}/{command_q[5]:+.3f}/{command_q[6]:+.3f} "
            f"R_wrist r/p/y={command_q[11]:+.3f}/{command_q[12]:+.3f}/{command_q[13]:+.3f}"
        )


def confirm_real_motion(config: R1A7LowCmdConfig) -> None:
    if not config.enable_motion or config.no_confirm:
        return
    print("WARNING: --mode real publishes rt/lowcmd and will move the real R1-A7 upper body.")
    print("Make sure the robot is supported, the workspace is clear, and E-stop is reachable.")
    if input("Type 'move' to continue: ").strip() != "move":
        raise SystemExit("Aborted.")
