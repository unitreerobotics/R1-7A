import csv
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from dual.replay import (ReplayTelemetry, StartupLimitRecovery, load_dual_episode,
                         pause_boundary_indices, recorded_settle_target, wait_for_move_while_holding)
from robot.r1arm_sdk import R1ArmSdkState, model_to_sdk_q
from teleop.r1arm_real_controller import MODEL_LOWER
from teleop.local_camera_client import CameraSpec, _CameraWorker
from teleop.mcap_writer import FourCameraMcapRecorder


class Alignment(unittest.TestCase):
    def test_pause_boundaries_follow_recorded_time_and_reject_bad_times(self):
        frames = [{'sample_time_ns': 10**18 + i * 10**9} for i in range(4)]
        self.assertEqual(pause_boundary_indices(frames, '1.5,2.5'), {2, 3})
        for spec in ('0', '3', '2,1', '1.1,1.2', 'nan', '1,'):
            with self.subTest(spec=spec), self.assertRaises(ValueError):
                pause_boundary_indices(frames, spec)

    def test_pause_keeps_issuing_hold_commands_until_move(self):
        read_fd, write_fd = os.pipe()
        try:
            with os.fdopen(read_fd) as stream:
                count = 0
                def hold():
                    nonlocal count
                    count += 1
                    if count == 3:
                        os.write(write_fd, b'MOVE\n')
                self.assertTrue(wait_for_move_while_holding(hold, 1000, .2, stream))
                self.assertEqual(count, 3)
        finally:
            os.close(write_fd)

        read_fd, write_fd = os.pipe()
        try:
            with os.fdopen(read_fd) as stream:
                os.write(write_fd, b'\n')
                self.assertFalse(wait_for_move_while_holding(lambda: None, 1000, .2, stream))
        finally:
            os.close(write_fd)

        read_fd, write_fd = os.pipe()
        try:
            with os.fdopen(read_fd) as stream:
                with self.assertRaisesRegex(TimeoutError, 'No MOVE'):
                    wait_for_move_while_holding(lambda: None, 1000, .003, stream)
        finally:
            os.close(write_fd)

    def test_startup_limit_error_identifies_joint_without_widening_margin(self):
        q = np.array([0., 1.2, 0., -1.2, 0., 0., 0.])
        q[5] = MODEL_LOWER[5] - .3
        state = R1ArmSdkState(q=model_to_sdk_q(q), dq=np.zeros(7), tau_est=np.zeros(7),
            mode=np.zeros(7), timeout=np.zeros(7), error_code=np.zeros(7), ready=np.ones(7, dtype=bool))
        with self.assertRaisesRegex(RuntimeError, r'left: J6 feedback .* by 0.300 rad; startup recovery limit 0.25 rad'):
            StartupLimitRecovery('left').check(state, startup=True)

    def test_recorded_settle_target_preserves_action_and_bounds_feedback_gap(self):
        action = np.array([0., 1.2, 0., -1.2, 0., 0., 0., .0245] * 2)
        measured = action.copy()
        measured[9] -= .076
        frame = {'idx': 1, 'states': {'joint_state': measured.tolist()}}
        target = recorded_settle_target(frame, action, .1)
        self.assertAlmostEqual(target[8], action[9] - .076)
        self.assertAlmostEqual(action[9], 1.2)
        with self.assertRaisesRegex(ValueError, 'recorded action/feedback gap'):
            recorded_settle_target(frame, action, .05)

    def test_control_source_preserves_16d_grippers_and_segments(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = FourCameraMcapRecorder(directory,'test',30,camera_order=(),control_frequency=90)
            path = writer.start_episode({'state_dim':16})
            home = [0,1.2,0,-1.2,0,0,0,.0245]*2
            for i in range(4):
                q = list(home); q[7] = -.02+i*.01; q[15] = .0245-i*.01
                writer.add_control_sample(dict(control_seq=i,control_time_ns=10**18+i*11111111,
                    q_purpose=q,q_measured=home),record_segment=int(i>=2))
            writer.add_frame((),{'joint_state':home},{'action':home},0,sample_time_ns=10**18)
            writer.stop_episode()
            vectors, frames, info, _ = load_dual_episode(path)
            self.assertEqual(vectors.shape,(4,16))
            self.assertEqual(info['replay_data_source'],'control')
            self.assertEqual(info['frequency_hz'],90)
            self.assertEqual(frames[-1]['record_segment'],1)
            self.assertAlmostEqual(vectors[-1,15],-.0055)
            self.assertEqual(load_dual_episode(path,data_source='frame')[0].shape,(1,16))
            self.assertTrue(np.allclose(load_dual_episode(path,section='state')[0],[home]*4))

    def test_history_never_pairs_future_frame(self):
        worker = _CameraWorker(CameraSpec(key='color_0_head_left'))
        with patch('teleop.local_camera_client.time.monotonic_ns',side_effect=[100,200]):
            worker.publish(np.zeros((2,2,3),np.uint8))
            worker.publish(np.ones((2,2,3),np.uint8))
        self.assertIsNone(worker.snapshot(50)[0])
        frame, count, _, mono, _ = worker.snapshot(150)
        self.assertEqual((count,mono),(1,100))
        self.assertEqual(int(frame.max()),0)
        frame[:] = 255
        self.assertEqual(int(worker.snapshot(150)[0].max()),0)

    def test_background_telemetry_drains_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'replay.csv'
            writer=ReplayTelemetry(path,['a','b'])
            for i in range(10): writer.put([i,i+1])
            writer.close()
            with path.open() as stream: rows=list(csv.reader(stream))
            self.assertEqual(len(rows),11)
            self.assertEqual(rows[-1],['9','10'])
            with self.assertRaises(FileExistsError): ReplayTelemetry(path,['x'])


if __name__=='__main__': unittest.main()
