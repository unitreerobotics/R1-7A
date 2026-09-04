"""Dex1 real gripper controller (migrated from xr_teleoperate_r1a7).

Wraps ``robot.eef.dex1_controller.Dex1_1_Gripper_Controller`` with a
force-limit latch and a 0..1 ``set_gripper_ratios`` API (0.0=open, 1.0=closed).
"""

from robot.eef.dex1_controller import Dex1_1_Gripper_Controller
from multiprocessing import Array, Lock, Value
import numpy as np
import time

DEX1_MAX_RANGE = 5.0  # safe operating open position in radians


class Dex1:
    def __init__(self, grip_torque_limit=-1.0, alpha=0.85, latch_max_pos=5.0):
        self.left_hand_pos_array = Value('d', 1, lock=True)
        self.right_hand_pos_array = Value('d', 1, lock=True)
        self.dual_hand_data_lock = Lock()
        # state: [l_q, r_q, l_tau, r_tau]
        self.dual_hand_state_array_out = Array('d', 4, lock=False)
        # action: [l_cmd, r_cmd]
        self.dual_hand_action_array_out = Array('d', 2, lock=False)

        self.gripper_ctrl = Dex1_1_Gripper_Controller(
            self.left_hand_pos_array,
            self.right_hand_pos_array,
            self.dual_hand_data_lock,
            self.dual_hand_state_array_out,
            self.dual_hand_action_array_out,
            simulation_mode=False,
        )
        self.l_cmd = 0.0
        self.r_cmd = 0.0

        # Grip force limit
        self._grip_torque_limit = grip_torque_limit  # Nm (negative = grip direction)
        self._alpha = alpha                           # torque low-pass filter
        self._latch_max_pos = latch_max_pos                 # max pos (rad) where force limit engages
        self._l_tau_f = 0.0
        self._r_tau_f = 0.0
        self._tau0_l = 0.0
        self._tau0_r = 0.0
        self._l_latch_pos = None                      # latched max-close position (rad)
        self._r_latch_pos = None

        # Auto-calibrate and enable
        self.set_gripper_ratios(0.0, 0.0)  # open first
        self.calibrate_tau_zero(timeout=1.5)

    # ---- internal helpers ----

    @staticmethod
    def _physical_to_normalized(physical):
        """physical rad -> 0-1 (0=closed, 1=open)."""
        return float(np.clip(float(physical) / DEX1_MAX_RANGE, 0.0, 1.0))

    def _grip_force_limit(self, l_des, r_des, l_q_cur, r_q_cur, l_tau, r_tau):
        """Grip force limit with latch: once torque hits threshold, lock the
        close position until the user commands a release (opening).
        """
        l_load = l_tau - self._tau0_l
        r_load = r_tau - self._tau0_r
        self._l_tau_f = self._alpha * self._l_tau_f + (1.0 - self._alpha) * l_load
        self._r_tau_f = self._alpha * self._r_tau_f + (1.0 - self._alpha) * r_load

        l_q = l_des
        r_q = r_des
        l_held, r_held = False, False

        # Left
        if self._l_latch_pos is not None:
            if l_des < self._l_latch_pos:   # still trying to close past latch
                l_q = self._l_latch_pos
                l_held = True
            else:                            # command relaxed (opening) — release latch
                self._l_latch_pos = None
        elif (l_q_cur < self._latch_max_pos and l_des < l_q_cur
              and self._l_tau_f < self._grip_torque_limit):
            self._l_latch_pos = l_q_cur
            l_q = l_q_cur
            l_held = True

        # Right
        if self._r_latch_pos is not None:
            if r_des < self._r_latch_pos:
                r_q = self._r_latch_pos
                r_held = True
            else:
                self._r_latch_pos = None
        elif (r_q_cur < self._latch_max_pos and r_des < r_q_cur
              and self._r_tau_f < self._grip_torque_limit):
            self._r_latch_pos = r_q_cur
            r_q = r_q_cur
            r_held = True

        self._dbg_cnt = getattr(self, '_dbg_cnt', 0) + 1
        # if self._dbg_cnt % 10 == 0:
        #     l_latch = f" LATCH={self._l_latch_pos:.3f}" if self._l_latch_pos is not None else ""
        #     r_latch = f" LATCH={self._r_latch_pos:.3f}" if self._r_latch_pos is not None else ""
        #     print(f"[Dex1] L: raw={l_tau:+.3f} filt={self._l_tau_f:+.3f} "
        #           f"load={l_load:+.3f} des={l_des:.3f} cur={l_q_cur:.3f} out={l_q:.3f}"
        #           f"{' HOLD' if l_held else ''}{l_latch} | "
        #           f"R: raw={r_tau:+.3f} filt={self._r_tau_f:+.3f} "
        #           f"load={r_load:+.3f} des={r_des:.3f} cur={r_q_cur:.3f} out={r_q:.3f}"
        #           f"{' HOLD' if r_held else ''}{r_latch}")

        return float(np.clip(l_q, 0.0, DEX1_MAX_RANGE)), \
               float(np.clip(r_q, 0.0, DEX1_MAX_RANGE))

    # ---- public API ----

    def set_gripper_ratios(self, left_ratio, right_ratio):
        """0.0=fully open, 1.0=fully closed (uniform across all hand types)."""
        left_ratio = float(np.clip(left_ratio, 0.0, 1.0))
        right_ratio = float(np.clip(right_ratio, 0.0, 1.0))
        l_target = (1.0 - left_ratio) * DEX1_MAX_RANGE
        r_target = (1.0 - right_ratio) * DEX1_MAX_RANGE

        with self.dual_hand_data_lock:
            l_q_cur = self.dual_hand_state_array_out[0]
            r_q_cur = self.dual_hand_state_array_out[1]
            l_tau = self.dual_hand_state_array_out[2]
            r_tau = self.dual_hand_state_array_out[3]
        l_target, r_target = self._grip_force_limit(l_target, r_target, l_q_cur, r_q_cur, l_tau, r_tau)

        with self.left_hand_pos_array.get_lock():
            self.left_hand_pos_array.value = l_target
        with self.right_hand_pos_array.get_lock():
            self.right_hand_pos_array.value = r_target
        self.l_cmd = l_target / DEX1_MAX_RANGE
        self.r_cmd = r_target / DEX1_MAX_RANGE

    def set_hand_targets(self, targets):
        """Direct motor targets in 0-1 (0=closed, 1=open, same space as get_hand_states)."""
        l_target = float(np.clip(targets[0], 0.0, 1.0)) * DEX1_MAX_RANGE
        r_target = float(np.clip(targets[1], 0.0, 1.0)) * DEX1_MAX_RANGE

        with self.dual_hand_data_lock:
            l_q_cur = self.dual_hand_state_array_out[0]
            r_q_cur = self.dual_hand_state_array_out[1]
            l_tau = self.dual_hand_state_array_out[2]
            r_tau = self.dual_hand_state_array_out[3]
        l_target, r_target = self._grip_force_limit(l_target, r_target, l_q_cur, r_q_cur, l_tau, r_tau)

        with self.left_hand_pos_array.get_lock():
            self.left_hand_pos_array.value = l_target
        with self.right_hand_pos_array.get_lock():
            self.right_hand_pos_array.value = r_target
        self.l_cmd = l_target / DEX1_MAX_RANGE
        self.r_cmd = r_target / DEX1_MAX_RANGE

    def get_hand_states(self):
        with self.dual_hand_data_lock:
            left_state = [self._physical_to_normalized(self.dual_hand_state_array_out[0])]
            right_state = [self._physical_to_normalized(self.dual_hand_state_array_out[1])]
        return left_state, right_state

    def get_hand_torques(self):
        """Return (l_tau, r_tau) in Nm."""
        with self.dual_hand_data_lock:
            return (self.dual_hand_state_array_out[2],
                    self.dual_hand_state_array_out[3])

    def get_record_data(self):
        left_state, right_state = self.get_hand_states()
        l_tau, r_tau = self.get_hand_torques()
        return {
            "hand_state": list(left_state) + list(right_state),
            "hand_cmd": [self.l_cmd, self.r_cmd],
            "hand_torque": [l_tau, r_tau],
        }

    def stop(self):
        """Stop the command loop during TeleopRunner shutdown."""
        controller = self.gripper_ctrl
        controller.running = False
        thread = getattr(controller, "gripper_control_thread", None)
        if thread is not None:
            thread.join(timeout=1.0)

    def calibrate_tau_zero(self, timeout=1.5):
        """Calibrate tau zero offset. Call once, with grippers open and unloaded."""
        samples_l, samples_r = [], []
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.dual_hand_data_lock:
                l_tau = self.dual_hand_state_array_out[2]
                r_tau = self.dual_hand_state_array_out[3]
            samples_l.append(l_tau)
            samples_r.append(r_tau)
            time.sleep(0.01)
        self._tau0_l = float(np.mean(samples_l))
        self._tau0_r = float(np.mean(samples_r))
        self._l_load_f = 0.0
        self._r_load_f = 0.0
        return self._tau0_l, self._tau0_r

    def change_open_pose(self, fsm_mode):
        pass
