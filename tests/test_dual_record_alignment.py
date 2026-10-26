import csv
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from dual.replay import ReplayTelemetry, load_dual_episode
from teleop.local_camera_client import CameraSpec, _CameraWorker
from teleop.mcap_writer import FourCameraMcapRecorder


class Alignment(unittest.TestCase):
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
