"""Right- or left-only Dex1 DDS controller for a standalone R1Arm."""

from __future__ import annotations

import threading
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_


DEX1_MAX_RANGE = 5.0


class Dex1Single:
    """Publish only one Dex1 side instead of the R1-A7 dual-gripper pair."""

    def __init__(
        self,
        side: str = "right",
        interface: str = "",
        domain: int = 0,
        grip_torque_limit: float = -1.0,
        torque_filter_alpha: float = 0.85,
        feedback_timeout: float = 2.0,
    ) -> None:
        self.side = str(side).lower()
        if self.side not in {"left", "right"}:
            raise ValueError(f"Dex1 side must be left or right, got {side!r}")

        if interface:
            ChannelFactoryInitialize(int(domain), interface)
        else:
            ChannelFactoryInitialize(int(domain))

        self.command_topic = f"rt/dex1/{self.side}/cmd"
        self.state_topic = f"rt/dex1/{self.side}/state"
        self.publisher = ChannelPublisher(self.command_topic, MotorCmds_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber(self.state_topic, MotorStates_)
        self.subscriber.Init()

        self.message = MotorCmds_()
        self.message.cmds = [unitree_go_msg_dds__MotorCmd_()]
        self.message.cmds[0].dq = 0.0
        self.message.cmds[0].tau = 0.0
        self.message.cmds[0].kp = 5.0
        self.message.cmds[0].kd = 0.05

        self._lock = threading.Lock()
        self._running = True
        self._state_received = False
        self._q = DEX1_MAX_RANGE
        self._tau = 0.0
        self._tau_zero_samples: list[float] = []
        self._tau_zero = 0.0
        self._tau_filtered = 0.0
        self._grip_torque_limit = float(grip_torque_limit)
        self._alpha = float(np.clip(torque_filter_alpha, 0.0, 0.999))
        self._latched_q: float | None = None
        self.last_command = DEX1_MAX_RANGE

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        deadline = time.monotonic() + float(feedback_timeout)
        while not self.state_received and time.monotonic() < deadline:
            time.sleep(0.01)
        if not self.state_received:
            self.stop()
            raise TimeoutError(
                f"No Dex1 feedback on {self.state_topic} within {feedback_timeout:.1f}s"
            )
        print(
            f"Dex1 single-side DDS enabled: side={self.side}, "
            f"cmd={self.command_topic}, state={self.state_topic}"
        )

    def _read_loop(self) -> None:
        while self._running:
            message = self.subscriber.Read()
            if message is not None and getattr(message, "states", None):
                state = message.states[0]
                with self._lock:
                    self._q = float(state.q)
                    self._tau = float(state.tau_est)
                    self._state_received = True
                    if len(self._tau_zero_samples) < 100:
                        self._tau_zero_samples.append(self._tau)
                        self._tau_zero = float(np.mean(self._tau_zero_samples))
            time.sleep(0.002)

    @property
    def state_received(self) -> bool:
        with self._lock:
            return self._state_received

    def _force_limited_target(self, target: float) -> float:
        with self._lock:
            q_now = self._q
            load = self._tau - self._tau_zero
        self._tau_filtered = self._alpha * self._tau_filtered + (1.0 - self._alpha) * load

        if self._latched_q is not None:
            if target < self._latched_q:
                return self._latched_q
            self._latched_q = None
        elif (
            self.state_received
            and target < q_now
            and self._tau_filtered < self._grip_torque_limit
        ):
            self._latched_q = q_now
            return q_now
        return target

    def set_closed_ratio(self, ratio: float) -> None:
        ratio = float(np.clip(ratio, 0.0, 1.0))
        target = (1.0 - ratio) * DEX1_MAX_RANGE
        target = float(np.clip(self._force_limited_target(target), 0.0, DEX1_MAX_RANGE))
        self.message.cmds[0].q = target
        self.publisher.Write(self.message)
        self.last_command = target

    def set_gripper_ratios(self, left_ratio: float, right_ratio: float) -> None:
        ratio = left_ratio if self.side == "left" else right_ratio
        self.set_closed_ratio(ratio)

    def get_record_data(self) -> dict:
        with self._lock:
            q = self._q
            tau = self._tau
        return {
            "hand_state": [q / DEX1_MAX_RANGE],
            "hand_cmd": [self.last_command / DEX1_MAX_RANGE],
            "hand_torque": [tau],
        }

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
