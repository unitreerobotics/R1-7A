#!/usr/bin/env python3
"""Dual real R1Arm PICO teleop: A home, X record/save, B pause/resume."""
from pathlib import Path
import math
import signal
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.teleop_r1arm_real import build_parser
from robot.r1arm_config import R1ArmConfig
from dual.config import CALIBRATIONS, PORTS, LEFT_BUS_SIGN, parse_home, validate_pair
from teleop.control_timing import PeriodicDeadline, TimingReporter, set_affinity
from robot.r1arm_sdk import DEFAULT_BRIDGE_LIBRARY, validate_calibration_file
from dual.controller import R1ArmDualRealController
from dual.recorder import DualRecorder
from teleop.local_camera_client import LocalCameraSubscriber, parse_camera_specs, DUAL_DEFAULT_CAMERAS


def parse_args(argv=None):
    parser = build_parser()
    parser.description = __doc__
    parser.set_defaults(port="", dex1_side="right", bridge_lib=
        DEFAULT_BRIDGE_LIBRARY)
    parser.add_argument('--right-port', default=str(PORTS['right']))
    parser.add_argument('--left-port', default=str(PORTS['left']))
    parser.add_argument('--right-calib', type=Path, default=CALIBRATIONS['right'])
    parser.add_argument('--left-calib', type=Path, default=CALIBRATIONS['left'])
    parser.add_argument('--data-root', default=str(Path(__file__).resolve().parents[1] / 'data/dual_real'))
    parser.add_argument('--task-goal', default='')
    parser.add_argument('--frequency', type=float, default=30.0)
    parser.add_argument('--camera', action='append', default=[])
    parser.add_argument('--no-cameras', action='store_true')
    parser.add_argument('--duration', type=float, default=0.0, help='Exit after N seconds; 0 runs until Ctrl+C')
    for joint, default in ((2,'mirrored'),(4,'model'),(6,'mirrored')):
        parser.add_argument(f'--left-j{joint}-convention', choices=('model','mirrored','sdk'), default=default)
    parser.add_argument('--right-home-q', default='0,1.2,0,-1.2,0,0,0')
    parser.add_argument('--left-home-q', default='0,1.2,0,-1.2,0,0,0')
    parser.add_argument('--home-recovery-margin', type=float, default=.25)
    parser.add_argument('--max-joint-acceleration', type=float, default=6.)
    parser.add_argument('--timing-report', action='store_true')
    parser.add_argument('--cpu-affinity', default='')
    args = parser.parse_args(argv)
    try:
        validate_pair(args.right_port,args.left_port,args.right_calib,args.left_calib)
        parse_home(args.right_home_q); parse_home(args.left_home_q)
    except ValueError as exc:
        parser.error(str(exc))
    for name in ('rate', 'frequency', 'max_joint_speed', 'max_command_lead',
                 'trajectory_position_gain', 'trajectory_velocity_gain', 'feedback_timeout', 'foc_timeout',
                 'home_recovery_margin','max_joint_acceleration'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if args.frequency > args.rate:
        parser.error("Observation frequency cannot exceed the configured control rate")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and nonnegative")
    if args.trajectory_max_speed is not None and (not math.isfinite(args.trajectory_max_speed) or args.trajectory_max_speed <= 0):
        parser.error("--trajectory-max-speed must be finite and positive")
    if args.port or args.calib or args.side != 'right' or args.dex1_side != 'right':
        parser.error('Use --right-port/--left-port and --right-calib/--left-calib; controller sides are fixed')
    if args.motion_mode and args.mock_xr:
        parser.error('--mock-xr cannot be combined with --motion-mode')
    ports = [Path(args.right_port).resolve(), Path(args.left_port).resolve()]
    if len(set(ports)) != 2:
        parser.error('Left and right arm ports must be different devices')
    if args.motion_mode:
        if not args.bridge_lib.is_file():
            parser.error('Build scripts/build_r1arm_sdk_bridge.py first')
        if not all(p.exists() for p in ports):
            parser.error('Arm serial ports do not exist; verify USB1 and USB3')
        if args.enable_dex1 and not args.dex1_net:
            parser.error('Set --dex1-net to the DDS network interface')
        try:
            args.right_calib = validate_calibration_file(args.right_calib)
            args.left_calib = validate_calibration_file(args.left_calib)
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
    return args


def main():
    args = parse_args()
    if args.motion_mode and not args.no_confirm:
        print(f'RIGHT: {args.right_port}, calibration={args.right_calib}, J4 SDK=-URDF')
        print(f'LEFT: {args.left_port}, calibration={args.left_calib}; J2={args.left_j2_convention}, J4={args.left_j4_convention}, J6={args.left_j6_convention}')
        print(f'kp={args.kp}, kd={args.kd}, max speed={args.max_joint_speed} rad/s, '
              f'gravity_ff={args.gravity_ff}, tau_clip={args.tau_clip} Nm')
        print('Both arms will move to home on startup unless --no-home-on-start. Keep emergency stop reachable.')
        if input('Type MOVE to enable BOTH arms: ').strip() != 'MOVE':
            raise SystemExit('Cancelled')
    set_affinity(args.cpu_affinity)
    reporter = TimingReporter(args.timing_report or args.debug)
    config = R1ArmConfig(
        ik_max_acc_per_s2=args.max_joint_acceleration,
        reset_blend_frames=args.reset_blend_frames, scale_factor=args.scale_factor,
        trajectory_position_gain=args.trajectory_position_gain,
        trajectory_velocity_gain=args.trajectory_velocity_gain,
        trajectory_max_speed_rad_s=args.trajectory_max_speed or args.max_joint_speed,
    )
    if args.urdf:
        config.urdf_path = str(args.urdf.resolve())
    controller = camera = recorder = None
    running = True

    def stop_signal(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop_signal)
    try:
        if not args.no_cameras:
            camera = LocalCameraSubscriber(parse_camera_specs(args.camera or list(DUAL_DEFAULT_CAMERAS)))
            camera.start()
            camera.resolve_active(5.0)
            if not camera.active_keys:
                raise RuntimeError('No cameras available; select --camera or use --no-cameras')
            if camera.inactive:
                raise RuntimeError(f'Requested cameras unavailable: {camera.inactive}')
        controller = R1ArmDualRealController(
            config, right_port=args.right_port, left_port=args.left_port,
            right_calibration=args.right_calib, left_calibration=args.left_calib,
            left_sign=np.array([1., -1. if args.left_j2_convention == 'mirrored' else 1., 1.,
                -1. if args.left_j4_convention == 'model' else 1., 1.,
                -1. if args.left_j6_convention == 'mirrored' else 1., 1.]),
            right_home=tuple(parse_home(args.right_home_q)), left_home=tuple(parse_home(args.left_home_q)),
            recovery_margin=args.home_recovery_margin,
            baudrate=args.baud, bridge_library=args.bridge_lib, motion_mode=args.motion_mode,
            enable_dex1=args.enable_dex1, dex1_interface=args.dex1_net,
            dex1_domain=args.dex1_domain, grip_torque_limit=args.grip_torque_limit,
            kp=args.kp, kd=args.kd, rate=args.rate, max_joint_speed=args.max_joint_speed,
            max_command_lead=args.max_command_lead, feedback_timeout=args.feedback_timeout,
            foc_timeout=args.foc_timeout, home_on_start=args.home_on_start,
            mock_xr=args.mock_xr, debug=False, ik_dt=args.ik_dt,
            gravity_ff=args.gravity_ff, tau_clip=args.tau_clip,
        )
        recorder = DualRecorder(args, camera)
        print('A: BOTH home | X: start/save episode | B: pause/resume recording and tracking')
        print('Left/right grip: corresponding arm tracking; trigger: corresponding gripper')
        previous = dict.fromkeys(('A', 'X', 'B'), False)
        controller.start()
        clock = PeriodicDeadline(args.rate)
        start = time.monotonic()
        last = start-1.0/args.rate
        while running and (args.duration <= 0 or time.monotonic() - start < args.duration):
            now = time.monotonic()
            cycle_dt = now-last
            controller.sample_inputs()
            buttons = {name: controller.button(name) for name in previous}
            edges = {name: buttons[name] and not previous[name] for name in previous}
            previous = buttons
            if edges['A'] and args.enable_a_reset:
                controller.reset()
            if edges['X']:
                recorder.toggle()
                controller.set_paused(False)
            if edges['B'] and recorder.active:
                controller.set_paused(not controller.paused)
                recorder.pause(controller.paused)
            samples = controller.step(cycle_dt, sample_inputs=False)
            for sample in samples.values():
                sample['gripper_enabled'] = bool(args.motion_mode and args.enable_dex1)
            recorder.publish(samples)
            last = now
            if recorder.error is not None:
                raise RuntimeError('Dual recording failed') from recorder.error
            reporter.add(dict(controller.last_timing, cycle=cycle_dt))
            clock.wait()
    except KeyboardInterrupt:
        pass
    finally:
        reporter.close()
        # Stop motors before waiting for image encoding / disk cleanup.
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


if __name__ == '__main__':
    main()
