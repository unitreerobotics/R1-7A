#!/usr/bin/env python3
"""R1-7A teleoperation: --right / --left / --dual; add --record to save MCAP."""
import math
from pathlib import Path
import signal
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dual.config import ARM_SIDES, PORTS, parse_home, validate_arm_port, validate_pair
from dual.controller import R1ArmDualRealController
from dual.recorder import DualRecorder
from robot.r1arm_config import R1ArmConfig, DEFAULT_DUAL_MJCF_PATH
from robot.r1arm_sdk import format_gain, validate_calibration_file
from teleop.cli import build_parser
from teleop.control_timing import PeriodicDeadline, TimingReporter, set_affinity
from teleop.local_camera_client import LocalCameraSubscriber, parse_camera_specs, DEFAULT_CAMERAS, DUAL_DEFAULT_CAMERAS


def parse_args(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    sim = '--sim' in argv
    if sim:
        from teleop.sim_recorder import build_parser as simulation_parser
        parser = simulation_parser()
    else:
        parser = build_parser()
        parser.set_defaults(port='', side=None, dex1_side=None)
        parser.add_argument('--right-port', default=str(PORTS['right']))
        parser.add_argument('--left-port', default=str(PORTS['left']))
        parser.add_argument('--right-calib', type=Path)
        parser.add_argument('--left-calib', type=Path)
        parser.add_argument('--output-dir', '--data-root', dest='data_root', type=Path)
        parser.add_argument('--task-goal', default='')
        parser.add_argument('--frequency', type=float, default=30.)
        parser.add_argument('--camera', action='append', default=[])
        parser.add_argument('--no-cameras', action='store_true')
        parser.add_argument('--duration', type=float, default=0., help='Exit after N seconds; 0 runs until Ctrl+C')
        parser.add_argument('--auto-start', action='store_true', help='Start an episode immediately; requires --record')
        for joint, default in ((2, 'mirrored'), (4, 'model'), (6, 'mirrored')):
            parser.add_argument(f'--left-j{joint}-convention', choices=('model', 'mirrored', 'sdk'), default=default)
        parser.add_argument('--right-home-q', default='0,1.2,0,-1.2,0,0,0')
        parser.add_argument('--left-home-q', default='0,1.2,0,-1.2,0,0,0')
        parser.add_argument('--home-recovery-margin', type=float, default=.25)
        parser.add_argument('--max-joint-acceleration', type=float, default=6.)
        parser.add_argument('--timing-report', action='store_true')
        parser.add_argument('--cpu-affinity', default='')
    parser.description = __doc__
    modes = parser.add_mutually_exclusive_group(required=True)
    for mode in ARM_SIDES:
        modes.add_argument('--' + mode, action='store_const', const=mode, dest='arm')
    parser.add_argument('--record', action='store_true', help='Enable X/B recording controls; otherwise teleoperation only')
    parser.add_argument('--sim', action='store_true', help='Use MuJoCo; --sim --help lists simulation options')
    args = parser.parse_args(argv)
    if args.auto_start and not args.record:
        parser.error('--auto-start requires --record')
    for name in ('rate', 'frequency', 'max_joint_speed'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if args.record and args.frequency > args.rate:
        parser.error('Observation frequency cannot exceed control rate')
    if not math.isfinite(args.ik_dt) or args.ik_dt < 0:
        parser.error('--ik-dt must be finite and nonnegative')
    if sim:
        args.side = 'left' if args.arm == 'left' else 'right'
        args.use_dual_arm = args.arm == 'dual'
        if '--camera-views' not in argv:
            args.camera_views = ['head_left'] + [f'wrist_{side}' for side in ARM_SIDES[args.arm]]
        if args.ik_dt < 0 or not math.isfinite(args.ik_dt) or args.sim_substeps < 1:
            parser.error('Simulation timestep and substeps are invalid')
        return args
    sides = ARM_SIDES[args.arm]
    if args.side is not None:
        parser.error('Select an arm with --right, --left or --dual, not --side')
    if args.arm == 'dual' and (args.port or args.calib or args.dex1_side is not None):
        parser.error('Use --right-port/--left-port and --right-calib/--left-calib for dual mode')
    if args.dex1_side is not None and args.dex1_side != args.arm:
        parser.error('The gripper side must match the selected arm')
    if args.motion_mode and args.mock_xr:
        parser.error('--mock-xr cannot be combined with --motion-mode')
    if args.no_cameras and args.camera:
        parser.error('--no-cameras cannot be combined with --camera')
    if args.camera and not args.record:
        parser.error('--camera requires --record')
    if args.gripper_log:
        parser.error('Use recorded control samples for gripper diagnostics with --record')
    for name in ('max_command_lead', 'trajectory_position_gain', 'trajectory_velocity_gain',
                 'feedback_timeout', 'foc_timeout', 'home_recovery_margin', 'max_joint_acceleration', 'tau_clip'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if args.home_recovery_margin > .25:
        parser.error('--home-recovery-margin cannot exceed 0.25 rad')
    if args.trajectory_max_speed is not None and (not math.isfinite(args.trajectory_max_speed) or args.trajectory_max_speed <= 0):
        parser.error('--trajectory-max-speed must be finite and positive')
    for side, label in (('right', '右'), ('left', '左')):
        if getattr(args, side + '_calib') is None:
            setattr(args, side + '_calib', args.sdk_root / f'config/joint_calib_zero_{label}臂.txt')
    if len(sides) == 1:
        if args.port:
            setattr(args, args.arm + '_port', args.port)
        if args.calib:
            setattr(args, args.arm + '_calib', args.calib)
    try:
        if len(sides) == 2:
            validate_pair(args.right_port, args.left_port, args.right_calib, args.left_calib)
        for side in sides:
            validate_arm_port(getattr(args, side + '_port'), side)
            parse_home(getattr(args, side + '_home_q'))
        if args.motion_mode:
            if not args.bridge_lib.is_file():
                parser.error('Build scripts/build_r1arm_sdk_bridge.py first')
            if args.enable_dex1 and not args.dex1_net:
                parser.error('Set DEX1_NETWORK or --dex1-net')
            for side in sides:
                port = Path(getattr(args, side + '_port'))
                if not port.exists():
                    parser.error(f'{side} arm serial port does not exist: {port}')
                setattr(args, side + '_calib', validate_calibration_file(getattr(args, side + '_calib')))
        if args.record and not args.no_cameras:
            defaults = DUAL_DEFAULT_CAMERAS if args.arm == 'dual' else DEFAULT_CAMERAS if args.arm == 'right' else tuple(s for s in DUAL_DEFAULT_CAMERAS if 'color_3_wrist_right' not in s)
            args.camera_specs = parse_camera_specs(args.camera or list(defaults))
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    args.data_root = args.data_root or ROOT / 'data' / f'{args.arm}_real'
    return args


def run_real(args):
    sides = ARM_SIDES[args.arm]
    if args.motion_mode and not args.no_confirm:
        for side in sides:
            print(f'{side.upper()}: {getattr(args, side+"_port")}; calibration={getattr(args, side+"_calib")}')
        print(f'kp={format_gain(args.kp)}, kd={format_gain(args.kd)}, max speed={args.max_joint_speed} rad/s, '
              f'gravity_ff={args.gravity_ff}, tau_clip={args.tau_clip} Nm')
        print('Support the selected arm(s), clear the home path and keep emergency stop reachable.')
        if input(f'Type MOVE to enable {args.arm.upper()}: ').strip() != 'MOVE':
            raise SystemExit('Cancelled; no motor command sent')
    set_affinity(args.cpu_affinity)
    reporter = TimingReporter(args.timing_report or args.debug)
    config = R1ArmConfig(ik_max_acc_per_s2=args.max_joint_acceleration,
        reset_blend_frames=args.reset_blend_frames, scale_factor=args.scale_factor,
        trajectory_position_gain=args.trajectory_position_gain,
        trajectory_velocity_gain=args.trajectory_velocity_gain,
        trajectory_max_speed_rad_s=args.trajectory_max_speed or args.max_joint_speed)
    if args.urdf:
        config.urdf_path = str(args.urdf.resolve())
    controller = camera = recorder = None
    running = True
    def stop_signal(*_):
        nonlocal running
        running = False
    previous_term = signal.signal(signal.SIGTERM, stop_signal)
    try:
        if args.record and not args.no_cameras:
            camera = LocalCameraSubscriber(args.camera_specs)
            camera.start()
            camera.resolve_active(5.)
            if camera.inactive or not camera.active_keys:
                raise RuntimeError(f'Requested cameras unavailable: {camera.inactive}; verify cameras or use --no-cameras')
        controller = R1ArmDualRealController(config, sides=sides,
            right_port=args.right_port, left_port=args.left_port,
            right_calibration=args.right_calib, left_calibration=args.left_calib,
            left_sign=np.array([1., -1. if args.left_j2_convention == 'mirrored' else 1., 1.,
                -1. if args.left_j4_convention == 'model' else 1., 1.,
                -1. if args.left_j6_convention == 'mirrored' else 1., 1.]),
            right_home=tuple(parse_home(args.right_home_q)), left_home=tuple(parse_home(args.left_home_q)),
            recovery_margin=args.home_recovery_margin, baudrate=args.baud,
            bridge_library=args.bridge_lib, motion_mode=args.motion_mode,
            enable_dex1=args.enable_dex1, dex1_interface=args.dex1_net,
            dex1_domain=args.dex1_domain, grip_torque_limit=args.grip_torque_limit,
            kp=args.kp, kd=args.kd, rate=args.rate, max_joint_speed=args.max_joint_speed,
            max_command_lead=args.max_command_lead, feedback_timeout=args.feedback_timeout,
            foc_timeout=args.foc_timeout, home_on_start=args.home_on_start,
            mock_xr=args.mock_xr, debug=args.debug, ik_dt=args.ik_dt,
            gravity_ff=args.gravity_ff, tau_clip=args.tau_clip)
        controller.start()
        if args.record:
            recorder = DualRecorder(args, camera)
            if args.auto_start:
                recorder.toggle()
        print(f'Mode: {args.arm}; recording {"enabled (X start/save, B pause/resume)" if args.record else "disabled"}; A home')
        previous = dict.fromkeys(('A', 'X', 'B'), False)
        clock = PeriodicDeadline(args.rate)
        start = time.monotonic()
        last = start - 1 / args.rate
        while running and (args.duration <= 0 or time.monotonic() - start < args.duration):
            now = time.monotonic()
            controller.sample_inputs()
            buttons = {name: controller.button(name) for name in previous}
            edges = {name: buttons[name] and not previous[name] for name in previous}
            previous = buttons
            if edges['A'] and args.enable_a_reset:
                controller.reset()
            if recorder is not None:
                if edges['X']:
                    recorder.toggle()
                    controller.set_paused(False)
                if edges['B'] and recorder.active:
                    controller.set_paused(not controller.paused)
                    recorder.pause(controller.paused)
            samples = controller.step(now-last, sample_inputs=False)
            if recorder is not None:
                recorder.publish(samples)
                if recorder.error is not None:
                    raise RuntimeError('Recording failed') from recorder.error
            reporter.add(dict(controller.last_timing, cycle=now-last))
            last = now
            clock.wait()
    except KeyboardInterrupt:
        pass
    finally:
        reporter.close()
        try:
            if controller is not None:
                controller.stop()
        finally:
            try:
                if recorder is not None:
                    recorder.close()
            finally:
                if camera is not None:
                    camera.close()
                signal.signal(signal.SIGTERM, previous_term)


def run_sim(args):
    if args.record:
        from teleop.sim_recorder import R1ArmSimMcapRecorder
        recorder = R1ArmSimMcapRecorder(args)
        previous = signal.signal(signal.SIGTERM, lambda *_: setattr(recorder, 'running', False))
        try:
            recorder.run()
        finally:
            signal.signal(signal.SIGTERM, previous)
        return
    from teleop.r1arm_mujoco_controller import R1ArmMujocoTeleopController
    config = R1ArmConfig(dual_arm=args.use_dual_arm, control_side=args.side,
        scale_factor=args.scale_factor, reset_blend_frames=args.reset_blend_frames,
        trajectory_max_speed_rad_s=args.max_joint_speed)
    if args.urdf:
        config.urdf_path = args.urdf
    if args.mjcf:
        config.mjcf_path = args.mjcf
    elif args.use_dual_arm:
        config.mjcf_path = str(DEFAULT_DUAL_MJCF_PATH)
    controller = R1ArmMujocoTeleopController(config, mock_xr=args.mock_xr,
        debug=args.debug, show_viewer=not args.no_viewer, sim_substeps=args.sim_substeps,
        ik_dt=args.ik_dt if args.ik_dt > 0 else 1/args.rate)
    previous = signal.signal(signal.SIGTERM, lambda *_: setattr(controller, 'running', False))
    try:
        controller._on_start()
        clock = PeriodicDeadline(args.rate)
        start = last = time.monotonic()
        while controller.running and (args.duration <= 0 or time.monotonic()-start < args.duration):
            now = time.monotonic()
            controller.step(now-last if now > last else 1/args.rate)
            last = now
            clock.wait()
    except KeyboardInterrupt:
        pass
    finally:
        controller._on_stop()
        signal.signal(signal.SIGTERM, previous)


def main(argv=None):
    args = parse_args(argv)
    (run_sim if args.sim else run_real)(args)


if __name__ == '__main__':
    main()
