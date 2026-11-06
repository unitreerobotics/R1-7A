"""Independent models; prepare both arms before a parallel serial-output barrier."""
from concurrent.futures import ThreadPoolExecutor, wait
from dataclasses import replace
from copy import deepcopy
import time
import threading

import numpy as np

from dual.config import LEFT_BUS_SIGN, validate_pair, validate_arm_port
from robot.r1arm_sdk import R1ArmSdkBus, sdk_to_model_q, model_to_sdk_q
from teleop.r1arm_real_controller import R1ArmRealTeleopController, MODEL_LOWER, MODEL_UPPER
from teleop.xr_snapshot import sample_xr


class LeftArmBus:
    def __init__(self, library=None, *, bus_factory=R1ArmSdkBus, sign=LEFT_BUS_SIGN):
        self.bus = bus_factory(library)
        self.sign = np.asarray(sign, dtype=float).copy()
        if self.sign.shape != (7,) or not np.isin(self.sign, [-1., 1.]).all():
            raise ValueError('Left bus sign must contain seven +/-1 entries')

    def __getattr__(self, name):
        return getattr(self.bus, name)

    def read_state(self):
        state = self.bus.read_state()
        return replace(state, q=state.q * self.sign, dq=state.dq * self.sign,
                       tau_est=state.tau_est * self.sign)

    def command_position(self, q, kp, kd, tau=None, dq=None):
        return self.bus.command_position(np.asarray(q) * self.sign, kp, kd,
            tau=None if tau is None else np.asarray(tau) * self.sign,
            dq=None if dq is None else np.asarray(dq) * self.sign)


class ArmController(R1ArmRealTeleopController):
    tracking_paused = False

    def __init__(self, *args, recovery_margin=.25, **kwargs):
        self.recovery_margin = float(recovery_margin)
        if not 0 < self.recovery_margin <= .25:
            raise ValueError("Recovery margin must be (0,.25]")
        self._startup_phase = True
        self._recovery_active = False
        self._recovery_origin = None
        self._trajectory_position_bounds = None
        self._command_lower = self._command_upper = None
        self._cycle_feedback = None
        super().__init__(*args, **kwargs)
        self._recovery_started = time.monotonic()

    def get_grip_state(self, side=None):
        return 0. if self.tracking_paused or self._recovery_active else super().get_grip_state(side)

    def _check_feedback_range(self, state):
        if not np.all(state.ready) or not np.isfinite(state.q).all():
            return super()._check_feedback_range(state)
        q = sdk_to_model_q(state.q)
        if self._startup_phase and self.home_on_start:
            excess = max(float(np.max(MODEL_LOWER-q)), float(np.max(q-MODEL_UPPER)), 0.)
            if excess > self.recovery_margin:
                raise RuntimeError(f'{self.control_side}: startup exceeds home recovery margin {self.recovery_margin:g} rad')
            if self._recovery_origin is None and excess > 0:
                self._recovery_origin = q.copy()
                self._recovery_active = True
                self._trajectory_position_bounds = (np.minimum(MODEL_LOWER, q), np.maximum(MODEL_UPPER, q))
                a, b = [model_to_sdk_q(bound) for bound in self._trajectory_position_bounds]
                self._command_lower, self._command_upper = np.minimum(a, b), np.maximum(a, b)
            if self._recovery_active:
                self._check_corridor(q)
                return q
        if self._recovery_active:
            self._check_corridor(q)
            return q
        return super()._check_feedback_range(state)

    def _check_corridor(self, q):
        lo, hi = self._trajectory_position_bounds
        if np.any(q < lo-.02) or np.any(q > hi+.02):
            raise RuntimeError(f'{self.control_side}: feedback left bounded startup recovery corridor')

    def _check_faults(self,state,require_foc):
        try:
            super()._check_faults(state,require_foc)
        except RuntimeError as exc:
            raise RuntimeError(f"{self.control_side}: {exc}") from exc

    def _on_start(self):
        super()._on_start()
        self._startup_phase = False
        self._recovery_started = time.monotonic()

    def preflight(self):
        self._cycle_feedback = None
        if not self.motion_mode:
            return
        read_wall = time.time_ns()
        read_mono = time.monotonic_ns()
        state = self.arm_bus.read_state()
        self._cycle_feedback_read = (read_wall,read_mono,time.time_ns(),time.monotonic_ns())
        self._check_feedback_range(state)
        self._check_faults(state, True)
        self._cycle_feedback = state

    def prepare_step(self, dt):
        if self._recovery_active:
            if time.monotonic()-self._recovery_started > 30:
                raise TimeoutError(f'{self.control_side}: startup recovery timed out')
            # The original shaper runs inside an initial-position-to-URDF corridor.
            # No IK or grip activation is allowed until command AND feedback re-enter limits.
            self._step_dt = dt
            self._step_grippers()
            goal = self._create_initial_q()
            self.q = self._trajectory_step(self.control_side, self.q, goal)
            feedback = self._cycle_feedback if self._cycle_feedback is not None else self.last_state
            measured = sdk_to_model_q(feedback.q)
            if all(np.all(q >= MODEL_LOWER) and np.all(q <= MODEL_UPPER)
                   for q in (measured, self.arm_q(), sdk_to_model_q(self.command_sdk))):
                self._recovery_active = False
                self._trajectory_position_bounds = None
                self._command_lower = self._command_upper = None
                self.reset()
            return self.q
        return super().prepare_step(dt)

    def step(self, dt):
        self.preflight()
        self.prepare_step(dt)
        self._output_command()
        return self.q

    def _output_command(self):
        try:
            return super()._output_command()
        except Exception as exc:
            raise RuntimeError(f'{self.control_side} arm output failed: {exc}') from exc


PausableArmController = ArmController


class R1ArmDualRealController:
    def __init__(self, config, *, right_port, left_port, right_calibration,
                 left_calibration, left_sign=LEFT_BUS_SIGN, right_home=None, left_home=None,
                 recovery_margin=.25, sides=('right', 'left'), **kwargs):
        if sides not in (('right',), ('left',), ('right', 'left')):
            raise ValueError('Select right, left or both arms')
        if len(sides) == 2:
            validate_pair(right_port, left_port, right_calibration, left_calibration)
        else:
            validate_arm_port(right_port if sides[0] == 'right' else left_port, sides[0])
        self.arms = {}
        self.pool = ThreadPoolExecutor(max_workers=len(sides), thread_name_prefix='arm-serial')
        self.paused = False
        self.native_xr = None
        self.snapshot = None
        self.closed = False
        self.last_timing = {}
        self._last_pair = None
        try:
            base = dict(kwargs, recovery_margin=recovery_margin)
            for index, side in enumerate(sides):
                options = dict(base) if index == 0 else dict(base, motion_mode=False, mock_xr=True)
                if side == 'left':
                    options['bus_factory'] = lambda lib: LeftArmBus(lib, sign=left_sign)
                self.arms[side] = ArmController(
                    replace(config, dual_arm=False, control_side=side,
                            home_q=right_home if side == 'right' else left_home),
                    port=right_port if side == 'right' else left_port,
                    calibration_path=right_calibration if side == 'right' else left_calibration,
                    dex1_side=side, **options)
                first = self.arms[sides[0]]
                if index == 0:
                    self.native_xr = first.xr_client
                self.arms[side].motion_mode = first.motion_mode
                self.arms[side].mock_xr = first.mock_xr
                self.arms[side].allow_xr_fallback = first.allow_xr_fallback
                self.arms[side].xr_client = self.native_xr
            for arm in self.arms.values():
                arm.enable_a_button_reset = False
        except BaseException:
            self.stop()
            raise

    def _parallel(self, method, *, operation=None):
        futures = {side: (self.pool.submit(operation, arm) if operation is not None
                          else self.pool.submit(getattr(arm, method)))
                   for side, arm in self.arms.items()}
        try:
            wait(futures.values())
        except BaseException:
            # Ctrl+C during the barrier must not close a bus under a native call.
            for future in futures.values():
                try:
                    future.result()
                except BaseException:
                    pass
            raise
        errors = []
        for side, future in futures.items():
            try:
                future.result()
            except BaseException as exc:
                errors.append((side, exc))
        if errors:
            side, error = errors[0]
            raise RuntimeError(f'{side} {method} failed: {error}') from error

    def start(self):
        startup_done = threading.Event()
        ready_lock = threading.Lock()
        ready = set()

        def start_and_hold(arm):
            try:
                arm._on_start()
                with ready_lock:
                    ready.add(arm.control_side)
                    if len(ready) == len(self.arms):
                        startup_done.set()
                # Keep the measured pose alive while the other bus is probing.
                while not startup_done.wait(1.0 / arm.rate_hz):
                    arm._output_command()
            except BaseException:
                startup_done.set()
                raise

        try:
            # Finish discovery for BOTH hands before either motor bus is enabled.
            for arm in self.arms.values():
                print(f"Initializing {arm.control_side} Dex1 DDS...", flush=True)
                arm._initialize_gripper()
            print('Starting both arm buses...', flush=True)
            self._parallel('_on_start', operation=start_and_hold)
        except BaseException:
            self.stop()
            raise

    def reset(self):
        for arm in self.arms.values():
            arm.reset()

    def set_paused(self, paused):
        self.paused = bool(paused)
        for arm in self.arms.values():
            arm.tracking_paused = self.paused
            for track in arm.track.values():
                track.clear()

    def sample_inputs(self):
        if self.native_xr is not None:
            self.snapshot = sample_xr(self.native_xr, self.snapshot)
            for arm in self.arms.values():
                arm.xr_client = self.snapshot
                arm.xr_snapshot = (self.snapshot.poses, self.snapshot.stamp)
                arm.xr_input_snapshot = self.snapshot

    def button(self, name):
        return self.snapshot.get_button_state_by_name(name) if self.snapshot else False

    def step(self, dt, *, sample_inputs=True):
        start = time.monotonic()
        if sample_inputs:
            self.sample_inputs()
        for arm in self.arms.values():
            arm.preflight()
        for arm in self.arms.values():
            arm.prepare_step(dt)
        prepared = time.monotonic()
        self._parallel('_output_command')
        samples = {side: arm.control_sample() for side, arm in self.arms.items()}
        stamps = [s['command_time_ns'] for s in samples.values()]
        skew = max(stamps)-min(stamps)
        for side, sample in samples.items():
            sample['pair_skew_ns'] = skew
            sample['gripper_enabled'] = bool(self.arms[side].motion_mode and self.arms[side].enable_dex1)
        self._last_pair = deepcopy(samples)
        for sample in samples.values():
            for value in sample.values():
                if isinstance(value,np.ndarray): value.setflags(write=False)
        self.last_timing = {'prepare': prepared-start, 'output': time.monotonic()-prepared,
                            'cycle': time.monotonic()-start,
                            **{side+'_io': (sample.get('command_send_end_monotonic_ns',0)-sample.get('command_send_start_monotonic_ns',0))*1e-9
                               for side,sample in samples.items()}}
        return samples

    def latest_pair(self):
        return deepcopy(self._last_pair)

    def stop(self):
        if self.closed:
            return
        self.closed = True
        for arm in self.arms.values():
            arm.xr_client = None
        try:
            self._parallel('_on_stop')
        finally:
            self.pool.shutdown(wait=True)
            if self.native_xr is not None:
                self.native_xr.close()
                self.native_xr = None
