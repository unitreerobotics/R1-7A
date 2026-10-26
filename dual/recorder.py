"""Bounded 90 Hz control capture and independently sampled 30 Hz observations."""
import queue
from dataclasses import asdict
import threading
import time
from pathlib import Path

import numpy as np

from dual.config import fingerprint
from robot.r1arm_config import DEX1_SIM_OPEN, DEX1_SIM_CLOSE, DEFAULT_URDF_PATH
from teleop.mcap_writer import FourCameraMcapRecorder
from teleop.r1arm_dataset import R1ArmDatasetAdapter


def paired_vectors(samples, *, measured):
    result = []
    for side in ('right', 'left'):
        sample = samples[side]
        ratio = sample.get('dex1_state_open_ratio' if measured else 'dex1_command_open_ratio')
        if measured and sample.get('gripper_enabled') and (ratio is None or not sample.get('dex1_feedback_valid')):
            raise RuntimeError(f'{side} gripper feedback missing/stale; recording stopped')
        if ratio is None:
            ratio = sample['dex1_command_open_ratio']
        if not np.isfinite(ratio):
            raise ValueError('Non-finite gripper feedback/command')
        grip = DEX1_SIM_CLOSE + float(np.clip(ratio, 0., 1.)) * (DEX1_SIM_OPEN-DEX1_SIM_CLOSE)
        result.extend(sample['q_measured' if measured else 'q_purpose'])
        result.extend((grip, grip))
    values = np.asarray(result)
    if not np.isfinite(values).all():
        raise ValueError('Non-finite paired joint snapshot')
    return values


class DualRecorder:
    def __init__(self, args, camera=None):
        self.args, self.camera = args, camera
        self.adapter = R1ArmDatasetAdapter(dual_arm=True)
        self.writer = FourCameraMcapRecorder(args.data_root, args.task_goal, args.frequency,
            format_name='r1arm_dual_real_mcap_json_v2', camera_order=() if camera is None else camera.active_keys,
            control_frequency=args.rate)
        self.queue = queue.Queue(maxsize=max(16, int(args.rate*10)))
        self.error = None
        self.active = self.paused = False
        self.segment = self.generation = 0
        self.diagnostics = {}
        self._previous_camera_ids = {}
        self.thread = threading.Thread(target=self._run, daemon=True, name='dual-mcap')
        self.thread.start()

    def _put(self, item):
        if self.error:
            raise RuntimeError('MCAP writer failed') from self.error
        try:
            self.queue.put_nowait(item)
        except queue.Full as exc:
            self.error = RuntimeError('Control recording queue overflow; no silent sample loss')
            raise self.error from exc

    def toggle(self):
        self.generation += 1
        if self.active:
            self._put(('close', None, self.segment))
            self.active = False
        else:
            self.segment = 0
            self._put(('open', None, 0))
            self.active = True
        self.paused = False

    def pause(self, paused):
        self.paused = bool(paused)
        if not paused:
            self.segment += 1
        self._put(('event', {'event': 'pause' if paused else 'resume', 'time_ns': time.time_ns()}, self.segment))

    def publish(self, samples):
        if self.active and not self.paused:
            self._put(('sample', samples, self.segment))

    def _metadata(self):
        args = self.args
        urdf = Path(args.urdf or DEFAULT_URDF_PATH)
        metadata = {'dual_arm': True, 'state_dim':16, 'action_dim':16,
            'joint_names':self.adapter.joint_names, 'urdf':str(urdf), 'urdf_sha256':fingerprint(urdf),
            'vector_order':'right J1..J7, grip, left J1..J7, grip', 'dry_run':not args.motion_mode,
            'camera_devices':None if self.camera is None else self.camera.describe(),
            'camera_specs':[] if self.camera is None else [asdict(spec) for spec in self.camera.specs],
            'control_parameters':{k:str(v) if isinstance(v,Path) else v.tolist() if isinstance(v,np.ndarray) else v
                                  for k,v in vars(args).items()}}
        for side in ('right','left'):
            path = Path(getattr(args,side+'_calib'))
            metadata[side+'_calibration'] = str(path)
            metadata[side+'_calibration_sha256'] = fingerprint(path) if path.is_file() else None
            metadata[side+'_port'] = str(getattr(args,side+'_port'))
        return metadata

    def _run(self):
        next_observation = 0
        try:
            while True:
                kind, data, segment = self.queue.get()
                if kind == 'stop':
                    return
                if kind == 'open':
                    path = self.writer.start_episode(self._metadata())
                    next_observation = 0
                    self.diagnostics = dict(controls=0,observations=0,schedule_missed=0,
                        camera_missing=0,camera_stale=0,camera_repeats={},camera_age_max_ms={})
                    self._previous_camera_ids = {}
                    print(f'[Record] started {path}', flush=True)
                elif kind == 'close':
                    self.writer.add_event(dict(event='record_summary',diagnostics=self.diagnostics))
                    self.writer.stop_episode()
                    print(f'[Record] diagnostics: {self.diagnostics}',flush=True)
                    print('[Record] saved' , flush=True)
                elif kind == 'event':
                    self.writer.add_event(dict(data, record_segment=segment))
                    next_observation = 0
                elif kind == 'sample':
                    measured, commanded = paired_vectors(data, measured=True), paired_vectors(data, measured=False)
                    states, actions = self.adapter.frame_data(measured, commanded)
                    stamp = max(s['control_time_ns'] for s in data.values())
                    seq = max(s['control_seq'] for s in data.values())
                    control = {'control_time_ns':stamp, 'control_seq':seq,
                        'q_purpose': actions['action'], 'q_measured':states['joint_state'],
                        'right':data['right'], 'left':data['left'],
                        'pair_skew_ns':data['right'].get('pair_skew_ns')}
                    self.writer.add_control_sample(control, record_segment=segment)
                    self.diagnostics['controls'] += 1
                    if stamp < next_observation:
                        continue
                    period = int(1e9/self.args.frequency)
                    if next_observation == 0: next_observation = stamp
                    due = max(1,(stamp-next_observation)//period+1)
                    self.diagnostics['schedule_missed'] += due-1
                    next_observation += due*period
                    target_mono = max(s.get('control_time_monotonic_ns',0) for s in data.values())
                    packet = None if self.camera is None else self.camera.get_atomic_snapshot(at_monotonic_ns=target_mono)
                    if packet is not None:
                        if packet['frames'] is None:
                            self.diagnostics['camera_missing'] += 1
                            continue
                        ages = {key:(target_mono-value)*1e-6 for key,value in packet['received_monotonic_ns'].items()}
                        if any(age < 0 or age > 500 for age in ages.values()):
                            self.diagnostics['camera_stale'] += 1
                            continue
                        for key, age in ages.items():
                            self.diagnostics['camera_age_max_ms'][key] = max(age,self.diagnostics['camera_age_max_ms'].get(key,0))
                            frame_id = packet['frame_ids'][key]
                            if self._previous_camera_ids.get(key) == frame_id:
                                repeats = self.diagnostics['camera_repeats']
                                repeats[key] = repeats.get(key,0)+1
                            self._previous_camera_ids[key] = frame_id
                    self.diagnostics['observations'] += 1
                    self.writer.add_frame(() if packet is None else packet['frames'],states,actions,seq,
                        sample_time_ns=stamp,control_seq=seq,record_segment=segment,
                        camera_frame_ids=None if packet is None else packet['frame_ids'],
                        camera_frame_times_ns=None if packet is None else packet['received_at_ns'],
                        camera_frame_times_monotonic_ns=None if packet is None else packet['received_monotonic_ns'],
                        camera_device_timestamps_ms=None if packet is None else packet['device_timestamp_ms'])
        except Exception as exc:
            self.error = exc
        finally:
            try:
                if self.writer.writer is not None:
                    self.writer.add_event(dict(event='record_summary',diagnostics=self.diagnostics))
                self.writer.stop_episode()
            except Exception as exc:
                self.error = self.error or exc

    def close(self):
        self.active = False
        while self.thread.is_alive():
            try:
                self.queue.put(('stop',None,0),timeout=.1)
                break
            except queue.Full:
                continue
        self.thread.join()
        if self.error:
            raise RuntimeError('Recording failed; inspect incomplete episode') from self.error
