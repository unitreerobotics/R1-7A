"""Offline tests for baseline recording; no physical device is opened."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import threading
import numpy as np
from scripts.record_r1arm_real import R1ArmRealMcapRecord
from teleop.mcap_writer import FourCameraMcapRecorder
from mcap.reader import make_reader

class RecordBaseline(unittest.TestCase):

    def test_background_flush_preserves_every_command_and_feedback(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = FourCameraMcapRecorder(task_dir=tmp, task_goal="test", frequency=30, camera_order=(), control_frequency=90)
            path = writer.start_episode({})
            samples = [dict(control_seq=i, control_time_ns=1000000000+i*11111111,
                q_purpose=np.full(7, i), q_measured=np.full(7, i-.01),
                q_ik_raw=np.full(7, i+.1), q_motor_target=np.full(7, i),
                dq_purpose=np.zeros(7), ddq_purpose=np.zeros(7),
                dq_target=np.zeros(7), dq_measured=np.zeros(7), tau_est=np.zeros(7))
                for i in range(1, 4)]
            recorder = R1ArmRealMcapRecord.__new__(R1ArmRealMcapRecord)
            recorder.controller = SimpleNamespace(
                drain_control_sample_capture=lambda **kw: (samples, samples[-1]),
                control_sample_capture_high_watermark=lambda: 3)
            recorder.writer = writer
            recorder.record_start_control_seq = 0
            recorder.record_segment = 0
            recorder.high_rate_diagnostics = dict(written=0, seq_gaps=0, last_seq=None, period_ms=[])
            errors = []
            def flush():
                try: recorder._flush_control_samples_locked()
                except BaseException as exc: errors.append(exc)
            worker = threading.Thread(target=flush)
            worker.start(); worker.join()
            self.assertEqual(errors, [])
            writer.stop_episode()
            with open(path, 'rb') as stream:
                rows = [json.loads(msg.data) for _, channel, msg in make_reader(stream).iter_messages()
                        if channel.topic == '/r1arm/control_sample']
            self.assertEqual([r['control_seq'] for r in rows], [1,2,3])
            for source, row in zip(samples, rows):
                np.testing.assert_array_equal(row['q_motor_target'], source['q_motor_target'])
                np.testing.assert_array_equal(row['q_measured'], source['q_measured'])
            self.assertEqual(recorder.high_rate_diagnostics['seq_gaps'], 0)


if __name__ == '__main__': unittest.main()
