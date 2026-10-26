"""Offline tests for baseline recording; no physical device is opened."""
import json
import queue
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock
from types import SimpleNamespace
import threading
import numpy as np
from scripts.record_r1arm_real import R1ArmRealMcapRecord
from teleop.mcap_writer import FourCameraMcapRecorder
from mcap.reader import make_reader

class RecordBaseline(unittest.TestCase):

    def test_slow_episode_save_runs_off_control_thread(self):
        recorder = R1ArmRealMcapRecord.__new__(R1ArmRealMcapRecord)
        recorder.args = SimpleNamespace(record_button='X', pause_button='B', frequency=30., duration=0.)
        recorder.record_commands = queue.Queue(maxsize=16)
        recorder.resume_tracking = threading.Event()
        recorder.record_stop_event = threading.Event()
        recorder.record_error = None
        recorder.running = recorder.is_recording = True
        recorder.record_paused = False
        recorder.controller = SimpleNamespace(track={'right': Mock()})
        recorder._button_edges = Mock(return_value={'X': True, 'B': False})
        entered = threading.Event()
        release = threading.Event()
        worker_ids = []

        def slow_save():
            worker_ids.append(threading.get_ident())
            entered.set()
            release.wait(timeout=2)
            recorder.is_recording = False

        recorder._stop_episode = slow_save
        recorder._handle_buttons()
        self.assertFalse(entered.is_set())
        worker = threading.Thread(target=recorder._record_loop)
        worker.start()
        try:
            self.assertTrue(entered.wait(timeout=2))
            recorder._button_edges.return_value = {'X': False, 'B': False}
            recorder.resume_tracking.set()
            recorder._handle_buttons()
            recorder.controller.track['right'].clear.assert_called_once()
            self.assertNotEqual(worker_ids[0], threading.get_ident())
            self.assertTrue(worker.is_alive())
        finally:
            recorder.record_stop_event.set()
            release.set()
            worker.join(timeout=3)
        self.assertFalse(worker.is_alive())
        self.assertIsNone(recorder.record_error)

    def test_queued_record_toggles_keep_order(self):
        recorder = R1ArmRealMcapRecord.__new__(R1ArmRealMcapRecord)
        recorder.record_commands = queue.Queue(maxsize=16)
        recorder.is_recording = False
        operations = []
        def start():
            operations.append('start')
            recorder.is_recording = True
        def stop():
            operations.append('stop')
            recorder.is_recording = False
        recorder._start_episode, recorder._stop_episode = start, stop
        for _ in range(3):
            recorder.record_commands.put_nowait('toggle')
        recorder._process_record_commands()
        self.assertEqual(operations, ['start', 'stop', 'start'])

    def test_shutdown_stops_hardware_before_writer_join_and_finalize(self):
        recorder = R1ArmRealMcapRecord.__new__(R1ArmRealMcapRecord)
        operations = []
        recorder.record_stop_event = threading.Event()
        recorder.record_error = None
        recorder.controller = SimpleNamespace(_on_stop=lambda: operations.append('motor_stop'))
        recorder.record_thread = SimpleNamespace(join=lambda: operations.append('writer_join'))
        recorder.preview_thread = None
        recorder.preview_created = False
        recorder.camera = SimpleNamespace(close=lambda: operations.append('camera_close'))
        recorder._stop_episode = lambda **kwargs: operations.append('finalize')
        recorder.shutdown()
        self.assertEqual(operations, ['motor_stop', 'writer_join', 'finalize', 'camera_close'])

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
