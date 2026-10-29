#!/usr/bin/env python3
"""Replay exactly 16 values: right arm/grip, then left arm/grip. Default: dry run."""
import argparse
import csv
import json
import queue
import threading
from concurrent.futures import ThreadPoolExecutor, wait
import math
from pathlib import Path
import signal
import sys
import time

from mcap.reader import make_reader
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.cli import build_parser as teleop_parser
from scripts.replay.replay_r1arm_real import (
    observation_mosaic, resolve_episode, load_episode_states, recorded_sample_times, plan_frame_intervals,
    interp_trajectory, recorded_peak_speed, wait_for_feedback, enable_foc_hold,
    check_feedback_range, check_faults, gravity_feedforward, safe_stop,
)
from robot.arm_model import load_robot_model
from dual.config import CALIBRATIONS, PORTS, validate_pair
from robot.r1arm_config import DEFAULT_URDF_PATH, DEX1_SIM_CLOSE, DEX1_SIM_OPEN, sim_to_closed_ratio
from robot.r1arm_sdk import DEFAULT_BRIDGE_LIBRARY, R1ArmSdkBus, model_to_sdk_q, sdk_to_model_q, validate_calibration_file
from dual.controller import LeftArmBus
from teleop.r1arm_real_controller import (
    MODEL_LOWER, MODEL_UPPER, limit_sdk_position_command,
)
from teleop.replay_preview import ReplayPreview

ROOT = Path(__file__).resolve().parents[1]


class StartupLimitRecovery:
    """Allow a bounded, inward-only replay start from a small model-limit offset."""
    def __init__(self, side, margin=.25):
        self.side = side
        self.margin = float(margin)
        self.origin = None
        self.lower = self.upper = None
        self.command_lower = self.command_upper = None

    @property
    def active(self):
        return self.origin is not None

    def check(self, state, *, startup=False):
        if not np.all(state.ready) or not np.isfinite(state.q).all():
            return check_feedback_range(state)
        q = sdk_to_model_q(state.q)
        if self.active:
            if np.any(q < self.lower - .02) or np.any(q > self.upper + .02):
                raise RuntimeError(f'{self.side}: feedback left bounded replay startup recovery corridor')
            return q
        if startup:
            excess = max(float(np.max(MODEL_LOWER-q)), float(np.max(q-MODEL_UPPER)), 0.)
            if excess > self.margin:
                raise RuntimeError(f'{self.side}: startup exceeds replay recovery margin {self.margin:g} rad')
            if excess > 0:
                self.origin = q.copy()
                self.lower, self.upper = np.minimum(MODEL_LOWER, q), np.maximum(MODEL_UPPER, q)
                a, b = model_to_sdk_q(self.lower), model_to_sdk_q(self.upper)
                self.command_lower, self.command_upper = np.minimum(a, b), np.maximum(a, b)
                print(f'[Replay] {self.side} starts {excess:.3f} rad outside model limits; '
                      f'slewing inward within the {self.margin:g} rad startup bound')
                return q
        return check_feedback_range(state)

    def target(self, model_target):
        return np.clip(model_target, MODEL_LOWER, MODEL_UPPER) if self.active else model_target

    def finish_if_inside(self, measured_model, command_sdk):
        if self.active and (np.all(measured_model >= MODEL_LOWER) and
                            np.all(measured_model <= MODEL_UPPER)):
            command_model = sdk_to_model_q(command_sdk)
            if np.all(command_model >= MODEL_LOWER) and np.all(command_model <= MODEL_UPPER):
                self.origin = self.lower = self.upper = None
                self.command_lower = self.command_upper = None


class ReplayTelemetry:
    """Bounded background CSV writer; overflow and disk errors stop replay."""
    def __init__(self,path,header):
        self.stream = Path(path).open('x',newline='')
        self.queue = queue.Queue(maxsize=1800)
        self.error = None
        self.thread = threading.Thread(target=self._run,args=(header,),daemon=True)
        self.thread.start()

    def _run(self,header):
        try:
            writer = csv.writer(self.stream)
            writer.writerow(header)
            while True:
                row = self.queue.get()
                if row is None:
                    break
                writer.writerow(row)
        except Exception as exc:
            self.error = exc
        finally:
            try:
                self.stream.close()
            except Exception as exc:
                self.error = self.error or exc

    def put(self,row):
        if self.error:
            raise RuntimeError('Replay telemetry failed') from self.error
        try:
            self.queue.put_nowait(row)
        except queue.Full as exc:
            raise RuntimeError('Replay telemetry queue overflow') from exc

    def close(self):
        while self.thread.is_alive():
            try:
                self.queue.put(None,timeout=.1)
                break
            except queue.Full:
                pass
        self.thread.join()
        if self.error:
            raise RuntimeError('Replay telemetry failed') from self.error


def load_dual_episode(path, section='action', data_source='auto'):
    frames, controls, info, config = [], [], {}, {}
    with Path(path).open('rb') as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            if channel.topic not in ('/episode/meta','/whole_body/frame','/r1arm/control_sample'):
                continue
            row = json.loads(message.data)
            if channel.topic == '/episode/meta':
                info, config = row.get('info',{}), row.get('config',{})
            elif channel.topic == '/whole_body/frame':
                frames.append(row)
            else:
                controls.append(row)
    use_control = data_source == 'control' or (data_source == 'auto' and bool(controls))
    if use_control:
        if not controls:
            raise ValueError('No high-rate dual control samples found')
        frames = [dict(idx=row['control_seq'],sample_time_ns=row['control_time_ns'],
            record_segment=row.get('record_segment',0),
            actions={'q_purpose':row['q_purpose']},states={'joint_state':row['q_measured']})
            for row in sorted(controls,key=lambda row:row['control_seq'])]
        info = dict(info,frequency_hz=info.get('control_frequency_hz',90))
    else:
        frames.sort(key=lambda row:row.get('idx',0))
    info = dict(info,replay_data_source='control' if use_control else 'frame')
    key, field = ('actions','q_purpose') if section == 'action' else ('states','joint_state')
    vectors = [np.asarray(row[key].get(field,row[key].get('action' if section == 'action' else 'robot_q_current',[])),float) for row in frames]
    if not vectors:
        raise ValueError('No replay trajectory found')
    for vector, frame in zip(vectors, frames):
        if vector.size != 16 or not np.isfinite(vector).all():
            raise ValueError(f"Frame {frame.get('idx')} must contain exactly 16 finite values")
        for offset in (0, 8):
            if np.any(vector[offset:offset + 7] < MODEL_LOWER - 1e-6) or np.any(
                vector[offset:offset + 7] > MODEL_UPPER + 1e-6
            ):
                raise ValueError(f"Frame {frame.get('idx')}: arm target outside URDF limits")
            if not DEX1_SIM_CLOSE - 1e-6 <= vector[offset + 7] <= DEX1_SIM_OPEN + 1e-6:
                raise ValueError(f"Frame {frame.get('idx')}: gripper outside recorded simulation range")
    return np.asarray(vectors), frames, info, config


def parse_args(argv=None):
    # Use the same gains, gravity and serial defaults as live teleoperation.
    parser = teleop_parser()
    parser.description = __doc__
    parser.add_argument('--episode', required=True, type=Path)
    parser.add_argument('--right-port', default=str(PORTS['right']))
    parser.add_argument('--left-port', default=str(PORTS['left']))
    parser.add_argument('--right-calib', type=Path, default=CALIBRATIONS['right'])
    parser.add_argument('--left-calib', type=Path, default=CALIBRATIONS['left'])
    parser.add_argument('--data-source', choices=('auto','control','frame'), default='auto')
    parser.add_argument('--show-observations', action=argparse.BooleanOptionalAction, default=True,
                        help='show recorded camera views by timestamp (default; use --no-show-observations to disable)')
    parser.add_argument('--preview-only', action='store_true',
                        help='Play recorded images and targets without opening serial/DDS devices')
    parser.add_argument('--preview-backend', choices=('browser', 'window'), default='browser',
                        help='browser: VS Code Simple Browser; window: OpenCV')
    parser.add_argument('--preview-scale', type=float, default=.75)
    parser.add_argument('--print-actions', action=argparse.BooleanOptionalAction, default=True,
                        help='Print targets during preview and commands during real replay')
    parser.add_argument('--section', choices=('action', 'state'), default='action')
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--max-gap', type=float, default=0.5)
    parser.add_argument('--approach-speed', type=float, default=0.3)
    parser.add_argument('--approach-timeout', type=float, default=30.0)
    parser.add_argument('--hold-time', type=float, default=2.0)
    parser.add_argument('--telemetry-csv', type=Path)
    parser.set_defaults(port='', dex1_side='right',
        bridge_lib=DEFAULT_BRIDGE_LIBRARY)
    args = parser.parse_args(argv)
    if args.preview_only and args.motion_mode:
        parser.error('--preview-only cannot be combined with --motion-mode')
    if not math.isfinite(args.preview_scale) or args.preview_scale <= 0:
        parser.error('--preview-scale must be finite and positive')
    try:
        validate_pair(args.right_port,args.left_port,args.right_calib,args.left_calib)
    except ValueError as exc:
        parser.error(str(exc))
    for name in ('rate', 'speed', 'max_gap', 'approach_speed', 'approach_timeout',
                 'max_joint_speed', 'max_command_lead', 'feedback_timeout', 'foc_timeout', 'tau_clip'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if not math.isfinite(args.hold_time) or args.hold_time < 0:
        parser.error('--hold-time must be finite and nonnegative')
    if args.port or args.calib or args.side != 'right' or args.dex1_side != 'right':
        parser.error('Use the two explicit arm ports/calibrations; sides are fixed')
    if args.mock_xr or False or False or False:
        parser.error('Replay takes recorded joint targets; XR and experimental IK modes do not apply')
    if Path(args.right_port).resolve() == Path(args.left_port).resolve():
        parser.error('Arm ports must be different')
    if args.motion_mode:
        if not args.bridge_lib.is_file() or not all(Path(p).exists() for p in (args.right_port, args.left_port)):
            parser.error('Verify bridge library and USB1/USB3 arm ports')
        if args.enable_dex1 and not args.dex1_net:
            parser.error('Set --dex1-net to the same interface as both gripper services')
        try:
            args.right_calib = validate_calibration_file(args.right_calib)
            args.left_calib = validate_calibration_file(args.left_calib)
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
    return args


def print_action(vector, *, mode, phase, grippers_enabled=False):
    print('[Action] ' + json.dumps(dict(mode=mode, phase=phase, grippers_enabled=grippers_enabled,
        right=dict(q_rad=np.asarray(vector[:7]).tolist(), gripper_slide=float(vector[7])),
        left=dict(q_rad=np.asarray(vector[8:15]).tolist(), gripper_slide=float(vector[15])))), flush=True)


def preview_only(args, path, vectors, frames, info, intervals, observations):
    worker = None
    def stop_signal(*_):
        raise KeyboardInterrupt
    previous_handler = signal.signal(signal.SIGTERM, stop_signal)
    try:
        if args.show_observations and observations:
            worker = ReplayPreview(path, tuple(info.get('camera_order') or observations[0]['colors']),
                scale=args.preview_scale, backend=args.preview_backend, window='Dual recorded observations')
        print('Open the VS Code preview URL above, then type MOVE here to start slow playback.')
        if input('Type MOVE to play recorded images and actions (Enter cancels): ').strip() != 'MOVE':
            raise SystemExit('Preview cancelled; no serial/DDS devices were opened')
        print(f'PREVIEW ONLY at {args.speed:g}x: recorded targets; no serial/DDS devices opened. Ctrl+C stops preview.')
        deadline = time.monotonic()
        for index, (vector, frame) in enumerate(zip(vectors, frames)):
            if index:
                deadline += float(intervals[index]) / args.speed
                time.sleep(max(0., deadline - time.monotonic()))
            if worker is not None and worker.update(int(frame['sample_time_ns'])):
                break
            if args.print_actions:
                print_action(vector, mode='preview', phase=f'frame_{index}')
    except KeyboardInterrupt:
        print('Preview stopped')
    finally:
        try:
            if worker is not None:
                worker.close()
        finally:
            signal.signal(signal.SIGTERM, previous_handler)


def run(args):
    path = resolve_episode(args.episode)
    vectors, frames, info, _ = load_dual_episode(path, args.section, args.data_source)
    observations = []
    if args.show_observations:
        with path.open('rb') as stream:
            observations = [json.loads(m.data) for _,c,m in make_reader(stream).iter_messages()
                            if c.topic == '/whole_body/frame']
        observations.sort(key=lambda row:row['sample_time_ns'])
        observations = [row for row in observations if row.get('colors')]
        if not observations:
            print('[Replay] No recorded images; preview disabled')
    observation_times = np.array([row['sample_time_ns'] for row in observations],dtype=np.int64)
    preview_worker = None
    preview_index = -1
    def preview(stamp):
        nonlocal preview_index
        if preview_worker is None or not observations:
            return
        index = int(np.searchsorted(observation_times,stamp,side='right'))-1
        if index >= 0 and index != preview_index:
            if preview_worker.update(observations[index]['sample_time_ns']):
                raise KeyboardInterrupt
            preview_index = index
    hz = float(info.get('frequency_hz', 30.0))
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError('Episode frequency must be finite and positive')
    intervals = plan_frame_intervals(recorded_sample_times(vectors, frames), hz, args.max_gap)
    segments = np.asarray([int(frame.get('record_segment', 0)) for frame in frames])
    # Pause/resume gaps are approached separately, never interpolated as motion.
    boundaries = np.r_[0, np.flatnonzero(np.diff(segments) != 0) + 1, len(vectors)]
    peak = max(recorded_peak_speed(vectors[a:b, [0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14]],
                                   intervals[a:b]) for a, b in zip(boundaries[:-1], boundaries[1:])) * args.speed
    print(f'{path}: {len(vectors)} frames, {len(boundaries)-1} segments, {hz:g} Hz, section={args.section}')
    print(f'Right={args.right_port}, left={args.left_port}; peak recorded joint speed={peak:.3f} rad/s')
    if peak > args.max_joint_speed + 1e-6 and not args.preview_only:
        raise ValueError(f'Recorded speed exceeds {args.max_joint_speed:g} rad/s; reduce --speed')
    if args.preview_only:
        preview_only(args, path, vectors, frames, info, intervals, observations)
        return
    if not args.motion_mode:
        print('DRY RUN: validated both arms and grippers; no serial/DDS devices opened')
        return
    if not args.no_confirm:
        print(f'Both arms will approach the first {args.section} frame at ≤{args.approach_speed:g} rad/s.')
        print(f'Calibrations: right={args.right_calib}; left={args.left_calib}')
        print('Right J4=-URDF; left J4=URDF. Support both arms; keep emergency stop reachable.')
        if input('Type MOVE to replay on BOTH arms: ').strip() != 'MOVE':
            raise SystemExit('Cancelled')
    buses, commands, hands = {}, {}, {}
    recovery = {side: StartupLimitRecovery(side) for side in ('right', 'left')}
    last_command_time = {}
    model, _ = load_robot_model(str(args.urdf or DEFAULT_URDF_PATH))
    gravity_data = {side: model.createData() for side in ('right', 'left')}
    tick = 1.0 / args.rate
    pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="replay-serial")
    running = True
    telemetry = None

    def stop_signal(*_):
        nonlocal running
        running = False

    previous_handler = signal.signal(signal.SIGTERM, stop_signal)

    def barrier(functions):
        futures = [pool.submit(function) for function in functions]
        wait(futures)
        results, errors = [], []
        for future in futures:
            try:
                results.append(future.result())
            except BaseException as exc:
                errors.append(exc)
        if errors:
            raise errors[0]
        return results

    def cycle(vector, speed_limit, phase='playback'):
        if not running:
            raise KeyboardInterrupt
        feedback = {side: bus.read_state() for side,bus in buses.items()}
        for side, state in feedback.items():
            recovery[side].check(state)
            check_faults(state, require_foc=True)
        def output(side, offset):
            state = feedback[side]
            model_q = sdk_to_model_q(state.q)
            target = model_to_sdk_q(recovery[side].target(vector[offset:offset+7]))
            now = time.monotonic()
            dt = now-last_command_time[side]
            if not 0 < dt <= .1:
                raise RuntimeError(f'{side} replay command interval invalid: {dt:.6f}s')
            bounds = {}
            if recovery[side].active:
                bounds = dict(lower_limit=recovery[side].command_lower,
                              upper_limit=recovery[side].command_upper)
            command = limit_sdk_position_command(target,commands[side],state.q,
                max_delta=speed_limit*dt,max_lead=args.max_command_lead,**bounds)
            dq=(command-commands[side])/dt
            tau = gravity_feedforward(model,gravity_data[side],sdk_to_model_q(command),args.tau_clip) if args.gravity_ff else None
            try:
                buses[side].command_position(command,args.kp,args.kd,tau=tau,dq=dq)
            except Exception as exc:
                raise RuntimeError(f'{side} replay output failed: {exc}') from exc
            commands[side]=command
            last_command_time[side]=now
            recovery[side].finish_if_inside(model_q, command)
            if side in hands:
                hands[side].set_closed_ratio(sim_to_closed_ratio(vector[offset+7]))
            return model_q, dict(side=side,time_ns=time.time_ns(),command_dt_s=dt,
                q_requested=vector[offset:offset+7].tolist(),
                q_sent=sdk_to_model_q(command).tolist(),dq_sent=sdk_to_model_q(dq).tolist(),
                q_measured=model_q.tolist(),
                dq_measured=sdk_to_model_q(state.dq).tolist(),
                q_limit_delta_model=sdk_to_model_q(command-target).tolist(),
                feedback_timeout=np.asarray(state.timeout).tolist(),
                motor_error_code=np.asarray(state.error_code).tolist())
        outputs = barrier([lambda: output('right',0),lambda: output('left',8)])
        if args.print_actions:
            sent = np.r_[outputs[0][1]['q_sent'], vector[7], outputs[1][1]['q_sent'], vector[15]]
            print_action(sent, mode='command', phase=phase, grippers_enabled=args.enable_dex1)
        measured = np.concatenate([item[0] for item in outputs])
        telemetry.put([time.time_ns(),*vector,*measured,json.dumps([item[1] for item in outputs])])
        return measured

    def approach(vector):
        deadline = time.monotonic() + args.approach_timeout
        desired = np.r_[vector[:7], vector[8:15]]
        settled = 0
        previous_q = None
        while time.monotonic() < deadline:
            start = time.monotonic()
            measured = cycle(vector, min(args.approach_speed, args.max_joint_speed), phase='approach')
            cmd = np.r_[sdk_to_model_q(commands['right']), sdk_to_model_q(commands['left'])]
            slow = previous_q is not None and np.max(np.abs(measured - previous_q)) / tick < .05
            settled = settled + 1 if np.max(np.abs(measured - desired)) < .05 and np.max(np.abs(cmd - desired)) < .001 and slow else 0
            previous_q = measured
            if settled >= max(1, math.ceil(.1 * args.rate)):
                return
            time.sleep(max(0., tick - (time.monotonic() - start)))
        raise TimeoutError('Both arms did not settle at segment start; replay cancelled')

    def hold(vector, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            start = time.monotonic()
            cycle(vector, args.max_joint_speed, phase='hold')
            time.sleep(max(0., tick - (time.monotonic() - start)))

    try:
        if args.show_observations and observations:
            preview_worker = ReplayPreview(
                path, tuple(info.get('camera_order') or observations[0]['colors']),
                window='Dual recorded observations', scale=args.preview_scale, backend=args.preview_backend,
            )
            preview_worker.update(observations[0]['sample_time_ns'])
        log_path = args.telemetry_csv or path.with_name(path.stem + f'_dual_replay_{time.time_ns()}.csv')
        telemetry = ReplayTelemetry(log_path,['time_ns', *[f'target_{i}' for i in range(16)],
            *[f'{s}_measured_J{i}' for s in ('right','left') for i in range(1,8)],'diagnostics_json'])
        def startup(side, port, calib, factory):
            bus = buses[side] = factory(args.bridge_lib)
            bus.open(port,args.baud,calib)
            validate = lambda sample: recovery[side].check(sample, startup=(side == 'left'))
            state=wait_for_feedback(bus,args.rate,args.feedback_timeout,check_state=validate)
            commands[side]=state.q.copy()
            tau_fn=lambda q: gravity_feedforward(model,gravity_data[side],sdk_to_model_q(q),args.tau_clip) if args.gravity_ff else None
            enable_foc_hold(bus,commands[side],args.kp,args.kd,args.rate,args.foc_timeout,
                            tau_fn=tau_fn,check_state=validate)
            last_command_time[side]=time.monotonic()
        barrier([lambda: startup('right',args.right_port,args.right_calib,R1ArmSdkBus),
                 lambda: startup('left',args.left_port,args.left_calib,LeftArmBus)])
        if args.enable_dex1:
            from robot.eef.dex1_single import Dex1Single
            for side in ('right', 'left'):
                hands[side] = Dex1Single(side=side, interface=args.dex1_net,
                    domain=args.dex1_domain, grip_torque_limit=args.grip_torque_limit)
        last_command_time.update({side:time.monotonic()-tick for side in buses})
        for a, b in zip(boundaries[:-1], boundaries[1:]):
            if a:
                hold(vectors[a-1], intervals[a] / args.speed)
            approach(vectors[a])
            targets, _, gaps = interp_trajectory(vectors[a:b], np.zeros(b-a), intervals[a:b], args.rate, args.speed)
            preview(frames[a]['sample_time_ns'])
            cycle(targets[0], args.max_joint_speed)
            timeline_stamp = frames[a]['sample_time_ns']
            deadline = time.monotonic()
            for index in range(1, len(targets)):
                deadline = max(deadline + gaps[index] / args.speed, time.monotonic())
                time.sleep(max(0., deadline - time.monotonic()))
                timeline_stamp += int(gaps[index]*1e9)
                preview(timeline_stamp)
                cycle(targets[index], args.max_joint_speed)
        approach(vectors[-1])
        hold(vectors[-1], args.hold_time)
        print(f'Replay finished; telemetry: {log_path}')
    except KeyboardInterrupt:
        print('Replay stopped')
    finally:
        pool.shutdown(wait=True)
        for bus in reversed(list(buses.values())):
            safe_stop(bus, args.rate)
        for hand in hands.values():
            hand.stop()
        try:
            if preview_worker is not None:
                preview_worker.close()
            if telemetry is not None:
                telemetry.close()
        finally:
            signal.signal(signal.SIGTERM, previous_handler)


def main():
    run(parse_args())


if __name__ == '__main__':
    main()
