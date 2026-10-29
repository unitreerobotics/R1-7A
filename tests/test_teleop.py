"""Unified CLI and compact single/dual recording; never commands hardware."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from mcap.reader import make_reader

from scripts.teleop import parse_args, run_real
from dual.config import ARM_SIDES, PORTS
from dual.controller import R1ArmDualRealController
from robot.r1arm_config import R1ArmConfig
from scripts.replay.replay_r1arm_real import load_episode_states

ROOT = Path(__file__).resolve().parents[1]


class UnifiedTeleop(unittest.TestCase):
    def test_selection_and_motion_boundaries(self):
        for mode in ARM_SIDES:
            args = parse_args(['--'+mode, '--mock-xr', '--no-cameras'])
            self.assertEqual(args.arm, mode)
            self.assertFalse(args.record)
            self.assertEqual(args.left_port, str(PORTS['left']))
            self.assertEqual(args.left_calib.name, 'joint_calib_zero_左臂.txt')
        for argv in ([], ['--left', '--right'], ['--left', '--motion-mode', '--mock-xr'],
                     ['--left', '--auto-start'], ['--dual', '--home-recovery-margin', '.3'],
                     ['--right', '--record', '--frequency', '100'], ['--right', '--rate', 'nan'], ['--left', '--ik-dt', 'nan'],
                     ['--left', '--dex1-side', 'right'], ['--left', '--port', str(PORTS['grip_left'])]):
            with self.subTest(argv=argv), self.assertRaises(SystemExit):
                parse_args(argv)

    def test_single_side_does_not_construct_or_require_other_arm(self):
        for mode in ('right', 'left'):
            args = parse_args(['--'+mode, '--mock-xr', '--no-cameras'])
            with patch('robot.r1arm_sdk.R1ArmSdkBus.open') as opened:
                controller = R1ArmDualRealController(R1ArmConfig(), sides=ARM_SIDES[mode],
                    right_port=args.right_port, left_port=args.left_port,
                    right_calibration=args.right_calib, left_calibration=args.left_calib,
                    baudrate=6000000, motion_mode=False, mock_xr=True, enable_dex1=False)
                try:
                    self.assertEqual(tuple(controller.arms), (mode,))
                    arm = controller.arms[mode]
                    self.assertEqual(arm.control_side, mode)
                    self.assertEqual(arm.dex1_side, mode)
                    controller.start()
                    samples = controller.step(1/90)
                    self.assertEqual(tuple(samples), (mode,))
                finally:
                    controller.stop()
                opened.assert_not_called()

    def test_plain_teleop_creates_neither_recorder_nor_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            args = parse_args(['--right', '--mock-xr', '--duration', '.03', '--output-dir', directory+'/out'])
            with patch('scripts.teleop.DualRecorder') as recorder, patch('scripts.teleop.LocalCameraSubscriber') as camera:
                run_real(args)
                recorder.assert_not_called()
                camera.assert_not_called()
            self.assertFalse(Path(directory, 'out').exists())

    def test_all_recording_modes_keep_side_dimensions_and_control_samples(self):
        for mode in ARM_SIDES:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                result = subprocess.run([sys.executable, 'scripts/teleop.py', '--'+mode,
                    '--record', '--auto-start', '--mock-xr', '--no-cameras',
                    '--duration', '.25', '--output-dir', directory], cwd=ROOT,
                    capture_output=True, text=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                path = next(Path(directory).glob('*.mcap'))
                with path.open('rb') as stream:
                    rows = [(c.topic,json.loads(m.data)) for _,c,m in make_reader(stream).iter_messages()]
                config = next(row['config'] for topic,row in rows if topic=='/episode/meta')
                controls = [row for topic,row in rows if topic=='/r1arm/control_sample']
                frames = [row for topic,row in rows if topic=='/whole_body/frame']
                width = 16 if mode=='dual' else 8
                self.assertEqual(config['state_dim'], width)
                self.assertEqual(config['control_side'], ARM_SIDES[mode][0])
                self.assertGreater(len(controls), len(frames))
                self.assertTrue(all(len(row['q_purpose'])==width for row in controls))
                self.assertTrue(all(len(row['states']['joint_state'])==width for row in frames))
                self.assertEqual([row['control_seq'] for row in controls], sorted(row['control_seq'] for row in controls))
                if mode!='dual':
                    states, _, _, metadata, _ = load_episode_states(path, 'action')
                    self.assertEqual(metadata['control_side'], mode)
                    np.testing.assert_allclose(states[0], controls[0]['q_purpose'])
                    self.assertIn('q_motor_target', controls[0])

    def test_cancelled_motion_never_constructs_controller(self):
        args = parse_args(['--right', '--mock-xr'])
        args.motion_mode = True
        with patch('builtins.input', return_value='cancel'), patch('scripts.teleop.R1ArmDualRealController') as factory:
            with self.assertRaises(SystemExit):
                run_real(args)
            factory.assert_not_called()

    def test_motor_shutdown_precedes_recorder_and_camera_cleanup(self):
        args = parse_args(['--dual', '--record', '--mock-xr', '--no-cameras', '--duration', '.01'])
        events = []
        with patch('scripts.teleop.R1ArmDualRealController') as factory, patch('scripts.teleop.DualRecorder') as recording:
            controller = factory.return_value
            controller.last_timing = {}
            controller.step.return_value = {}
            controller.button.return_value = False
            controller.stop.side_effect = lambda: events.append('stop')
            recording.return_value.error = None
            recording.return_value.close.side_effect = lambda: events.append('save')
            run_real(args)
        self.assertEqual(events, ['stop', 'save'])


if __name__ == '__main__':
    unittest.main()
