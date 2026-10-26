"""Latest-state XR processing. No kinematics, motor access, filtering or prediction."""
from dataclasses import dataclass
from collections import Counter
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from teleop.utils.mock import xr_pose_is_valid
from teleop.xr_snapshot import XrSnapshot, sample_xr


@dataclass
class DeviceInput:
    phase: str = 'waiting'
    token: object = None
    stamp: int = 0
    received_ns: int = 0
    pose: object = None
    candidate: object = None
    candidate_since_ns: int = 0
    candidate_count: int = 0
    generation: int = 0
    rejects: int = 0
    grip: bool = False


class XrInputProcessor:
    def __init__(self, config, sides=('left', 'right')):
        self.config = config
        self.devices = {side: DeviceInput() for side in sides}
        self.timeout_ns = int(config.xr_stale_timeout_s*1e9)
        self.recovery_ns = int(config.xr_recovery_time_s*1e9)
        if (not np.isfinite([config.xr_stale_timeout_s, config.xr_recovery_time_s,
                config.xr_jump_linear_speed, config.xr_jump_angular_speed,
                config.xr_grip_on, config.xr_grip_off]).all()
                or self.timeout_ns <= 0 or self.recovery_ns < 0
                or config.xr_jump_linear_speed <= 0 or config.xr_jump_angular_speed <= 0
                or not 0 <= config.xr_grip_off < config.xr_grip_on <= 1):
            raise ValueError('Invalid XR input limits')

    def plausible(self, before, after, dt):
        distance = np.linalg.norm(after[:3]-before[:3])
        angle = (Rotation.from_quat(before[3:7]).inv()*Rotation.from_quat(after[3:7])).magnitude()
        return (distance <= min(.30, .05+self.config.xr_jump_linear_speed*max(dt, 0.))
                and angle <= min(1., .20+self.config.xr_jump_angular_speed*max(dt, 0.)))

    def process(self, frame, now_ns):
        frame = frame or XrSnapshot({}, None, {}, {})
        output, keys, statuses = {}, dict(frame.keys), {}
        raw = {}
        for name, value in frame.poses.items():
            try:
                if xr_pose_is_valid(value):
                    raw[name] = np.asarray(value, dtype=float).reshape(-1)[:7].copy()
            except (TypeError, ValueError):
                continue
        metadata = frame.metadata
        fields = metadata.get('pose_fields', {})
        for side, state in self.devices.items():
            name = side+'_controller'
            field = fields.get(name)
            stamp = int((field or {}).get('source_stamp_ns', frame.stamp) or 0)
            received = int((field or {}).get('received_monotonic_ns',
                metadata.get('received_monotonic_ns', 0)) or 0)
            # Legacy clients have no arrival metadata. A source-stamp change is
            # the only evidence of freshness; repeated polls never refresh it.
            token = stamp
            fresh = token != state.token
            if fresh and received == 0:
                received = now_ns
            age_ns = max(0, now_ns-(received if fresh else state.received_ns))
            pose = raw.get(name)
            field_valid = field is None or field.get('valid', False)
            rollback = fresh and state.stamp > 0 and stamp <= state.stamp
            gap = fresh and state.received_ns > 0 and received-state.received_ns > self.timeout_ns
            try:
                grip = float(frame.keys.get(side+'_grip', 0.))
            except (TypeError, ValueError):
                grip = float('nan')
            state.grip = (bool(np.isfinite(grip)) and
                          grip >= (self.config.xr_grip_off if state.grip else self.config.xr_grip_on))
            if pose is None or not field_valid or stamp <= 0:
                if fresh:
                    state.token, state.stamp, state.received_ns = token, stamp, received
                state.phase, state.candidate, state.candidate_count = 'invalid', None, 0
            elif fresh:
                old_stamp = state.stamp
                state.token, state.stamp, state.received_ns = token, stamp, received
                if age_ns > self.timeout_ns:
                    state.phase, state.candidate, state.candidate_count = 'stale', None, 0
                else:
                    dt = (stamp-old_stamp)*1e-9
                    live = state.phase in ('live', 'repeat')
                    normal = (not rollback and not gap and live and state.pose is not None
                              and self.plausible(state.pose, pose, dt))
                    if state.phase == 'waiting' or normal:
                        state.pose, state.phase = pose.copy(), 'live'
                        state.candidate, state.candidate_count = None, 0
                    else:
                        if live and not rollback and not gap:
                            state.rejects += 1
                        # Recover only after distinct, temporally fresh and
                        # mutually plausible frames. A single spike cannot reanchor.
                        if (state.candidate is None or rollback or gap or
                                not self.plausible(state.candidate, pose, dt)):
                            state.candidate_since_ns, state.candidate_count = received, 1
                        else:
                            state.candidate_count += 1
                        state.candidate = pose.copy()
                        state.phase = 'recovering'
                        if (state.candidate_count >= 3 and
                                received-state.candidate_since_ns >= self.recovery_ns):
                            state.pose, state.phase = pose.copy(), 'live'
                            state.generation += 1
                            state.candidate, state.candidate_count = None, 0
            elif age_ns > self.timeout_ns:
                state.phase, state.candidate, state.candidate_count = 'stale', None, 0
            elif state.phase == 'live':
                state.phase = 'repeat'
            ready = state.phase in ('live', 'repeat') and state.pose is not None
            if ready:
                output[name] = state.pose.copy()
            keys[side+'_grip'] = 1. if ready and state.grip else 0.
            statuses[side] = dict(phase=state.phase, ready=ready, age_s=age_ns*1e-9,
                generation=state.generation, rejects=state.rejects, received_ns=state.received_ns, raw_grip=grip if np.isfinite(grip) else None,
                field_freshness='per_device' if field is not None else 'legacy_source_stamp')
        # Headset is optional for the existing controller-relative mapping.
        if 'headset' in raw:
            output['headset'] = raw['headset'].copy()
        result = XrSnapshot(output, frame.stamp, keys, dict(frame.buttons),
                            dict(metadata, xr_input=statuses))
        result.raw_poses = raw
        return result


class XrInputWorker:
    """One owner polls the native cache; controllers read a nonblocking latest slot."""
    def __init__(self, client, config):
        self.client = client
        try:
            self.processor = XrInputProcessor(config)
        except BaseException:
            client.close()
            raise
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.closed = False
        self.latest = None
        self.pending_edges = Counter()
        self.release_next = set()
        self.previous_counts = {}
        self.previous_buttons = {}
        self.error = None
        self.thread = threading.Thread(target=self._run, name='xr-input', daemon=True)
        try:
            self.thread.start()
        except BaseException:
            client.close()
            raise

    def _run(self):
        previous = None
        while not self.stop_event.is_set():
            try:
                frame = sample_xr(self.client, previous)
                previous = frame
                snapshot = self.processor.process(frame, time.monotonic_ns())
                for pose in list(snapshot.poses.values())+list(snapshot.raw_poses.values()):
                    pose.setflags(write=False)
                with self.lock:
                    counts = frame.metadata.get('button_press_counts', {})
                    for key, value in frame.buttons.items():
                        if key in counts:
                            delta = max(0, int(counts[key])-self.previous_counts.get(key, 0))
                        else:
                            delta = int(bool(value) and not self.previous_buttons.get(key, False))
                        self.pending_edges[key] += delta
                        if self.pending_edges[key] > 4096:
                            raise RuntimeError('XR button edge buffer overflow')
                    self.previous_counts = dict(counts)
                    self.previous_buttons = dict(frame.buttons)
                    self.latest = snapshot
                    self.error = None
            except Exception as exc:
                with self.lock:
                    self.error = repr(exc)
                    self.latest = None
            self.stop_event.wait(1/180)

    def get_input_snapshot(self):
        with self.lock:
            snapshot = self.latest
            if snapshot is None:
                return XrSnapshot({}, None, {}, {}, {'xr_input_error': self.error})
            result = XrSnapshot(snapshot.poses, snapshot.stamp, dict(snapshot.keys),
                dict(snapshot.buttons), dict(snapshot.metadata))
            result.raw_poses = snapshot.raw_poses
            # Toggle buttons are delivered as ordered press pulses, separated
            # by a release cycle, even if several presses occurred between reads.
            for key in set(result.buttons) | set(self.pending_edges):
                result.buttons[key] = False
                if key in self.release_next:
                    self.release_next.remove(key)
                elif self.pending_edges[key]:
                    result.buttons[key] = True
                    self.pending_edges[key] -= 1
                    self.release_next.add(key)
        # Re-evaluate age without touching the worker's mutable processor state.
        now = time.monotonic_ns()
        result.metadata['xr_input'] = dict(result.metadata.get('xr_input', {}))
        for side, status in result.metadata['xr_input'].items():
            age_ns = max(0, now-status['received_ns'])
            updated = dict(status, age_s=age_ns*1e-9)
            if age_ns > self.processor.timeout_ns:
                result.poses = dict(result.poses)
                result.poses.pop(side+'_controller', None)
                result.keys[side+'_grip'] = 0.
                updated.update(ready=False, phase='stale')
            result.metadata['xr_input'][side] = updated
        return result

    def get_frame_stamp_ns(self):
        with self.lock:
            return self.latest.stamp if self.latest else None
    def get_pose_by_name(self, name):
        with self.lock:
            return self.latest.get_pose_by_name(name) if self.latest else None
    def get_key_value_by_name(self, name):
        with self.lock:
            return self.latest.get_key_value_by_name(name) if self.latest else 0.
    def get_button_state_by_name(self, name):
        with self.lock:
            return bool(self.pending_edges[name] > 0)

    def __getattr__(self, name):
        return getattr(self.client, name)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop_event.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('XR worker did not stop; native client remains open to avoid a close/read race')
        self.client.close()
