"""Offline checks for preview composition and latest-only IPC behavior."""
import base64
import queue
import unittest

import cv2
import numpy as np

from teleop.replay_preview import ReplayPreview, _mosaic, _BrowserPreview
from scripts.replay.replay_r1arm_real import build_parser as real_parser
from scripts.replay.replay_r1arm_sim import build_parser as sim_parser
from dual.replay import parse_args as dual_parse_args


class ReplayPreviewTest(unittest.TestCase):
    def test_replay_entries_show_images_by_default_and_allow_disabling(self):
        for parser in (real_parser(), sim_parser()):
            self.assertTrue(parser.parse_args(['--episode', 'unused.mcap']).show_observations)
            self.assertFalse(parser.parse_args(['--episode', 'unused.mcap', '--no-show-observations']).show_observations)
        self.assertTrue(dual_parse_args(['--episode', 'unused.mcap']).show_observations)
        self.assertFalse(dual_parse_args(['--episode', 'unused.mcap', '--no-show-observations']).show_observations)

    def test_dual_preview_never_constructs_hardware_and_preserves_actions(self):
        import contextlib
        import io
        import json
        import tempfile
        from unittest.mock import patch
        from dual import replay
        from dual.config import HOME
        from robot.r1arm_config import DEX1_SIM_OPEN
        from teleop.mcap_writer import FourCameraMcapRecorder
        keys = ('color_0_head_left', 'color_2_wrist_left', 'color_3_wrist_right')
        vectors = [np.r_[HOME, DEX1_SIM_OPEN, HOME, DEX1_SIM_OPEN] for _ in range(2)]
        vectors[1][0], vectors[1][8] = .1, -.1
        with tempfile.TemporaryDirectory() as directory:
            writer = FourCameraMcapRecorder(directory, 'preview', 30, camera_order=keys)
            path = writer.start_episode({'state_dim': 16})
            for i, vector in enumerate(vectors):
                writer.add_frame(tuple(np.zeros((24, 32, 3), np.uint8) for _ in keys),
                    {'joint_state': vector.tolist()}, {'q_purpose': vector.tolist()}, i,
                    sample_time_ns=10**18+i*33333333)
            writer.stop_episode()
            args = replay.parse_args(['--episode', path, '--preview-only'])
            self.assertEqual(args.preview_backend, 'browser')
            output = io.StringIO()
            with patch.object(replay, 'R1ArmSdkBus') as right, patch.object(replay, 'LeftArmBus') as left, \
                 patch.object(replay, 'load_robot_model') as model, patch('robot.eef.dex1_single.Dex1Single') as hand, \
                 patch.object(replay, 'ReplayPreview') as preview, patch('builtins.input', return_value='MOVE'), \
                 contextlib.redirect_stdout(output):
                preview.return_value.update.return_value = False
                replay.run(args)
                for hardware in (right, left, model, hand):
                    hardware.assert_not_called()
                self.assertEqual(preview.call_args.args[1], keys)
                preview.return_value.close.assert_called_once()
                self.assertEqual(preview.return_value.update.call_count, 2)
            rows = [json.loads(line[len('[Action] '):]) for line in output.getvalue().splitlines()
                    if line.startswith('[Action] ')]
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[-1]['mode'], 'preview')
            self.assertFalse(rows[-1]['grippers_enabled'])
            self.assertEqual(rows[-1]['right']['q_rad'], vectors[-1][:7].tolist())
            self.assertEqual(rows[-1]['left']['q_rad'], vectors[-1][8:15].tolist())

    def test_dual_preview_rejects_motion_mode(self):
        with self.assertRaises(SystemExit):
            dual_parse_args(['--episode', 'unused.mcap', '--preview-only', '--motion-mode'])
        for value in ('0', 'nan'):
            with self.assertRaises(SystemExit):
                dual_parse_args(['--episode', 'unused.mcap', '--preview-scale', value])

    def test_browser_serves_latest_jpeg_and_closes(self):
        from urllib.request import urlopen
        preview = _BrowserPreview()
        try:
            preview.show(np.zeros((24, 32, 3), dtype=np.uint8))
            root = f'http://127.0.0.1:{preview.server.server_port}'
            with urlopen(root) as response:
                self.assertIn(b'/frame.jpg', response.read())
            with urlopen(root+'/frame.jpg?t=1') as response:
                decoded = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(decoded.shape, (24, 32, 3))
            args = real_parser().parse_args(['--episode', 'unused.mcap'])
            self.assertEqual(args.preview_backend, 'browser')
            self.assertTrue(args.print_actions)
        finally:
            preview.close()
        self.assertFalse(preview.thread.is_alive())

    def test_browser_process_reads_three_camera_mcap_and_exits(self):
        import json
        from pathlib import Path
        import tempfile
        import time
        from mcap.writer import Writer
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'three_views.mcap'
            ok, jpeg = cv2.imencode('.jpg', np.zeros((48, 64, 3), np.uint8))
            self.assertTrue(ok)
            keys = ('color_0_head_left', 'color_2_wrist_left', 'color_3_wrist_right')
            colors = {key: {'encoding': 'jpeg', 'data': base64.b64encode(jpeg).decode()} for key in keys}
            with path.open('wb') as stream:
                writer = Writer(stream)
                writer.start()
                channel = writer.register_channel('/whole_body/frame', 'json', 0)
                writer.add_message(channel, 100, json.dumps({'sample_time_ns': 100, 'colors': colors}).encode(), 100)
                writer.finish()
            preview = ReplayPreview(path, keys, backend='browser')
            try:
                self.assertFalse(preview.disabled)
                preview.update(100)
                time.sleep(.2)
                preview._report_error()
                self.assertFalse(preview.disabled)
                self.assertTrue(preview.process.is_alive())
            finally:
                preview.close()
            self.assertFalse(preview.process.is_alive())

    def test_three_camera_mosaic_keeps_a_two_column_layout(self):
        ok, encoded = cv2.imencode('.jpg', np.zeros((48, 64, 3), dtype=np.uint8))
        self.assertTrue(ok)
        record = {'encoding': 'jpeg', 'data': base64.b64encode(encoded).decode('ascii')}
        image = _mosaic({key: record for key in ('head', 'left', 'right')}, 1, 4,
                        ('head', 'left', 'right'), .5)
        self.assertEqual(image.shape, (48, 64, 3))

    def test_preview_updates_replace_pending_timestamp_without_waiting(self):
        class LatestQueue:
            def __init__(self):
                self.items = []

            def put_nowait(self, value):
                if self.items:
                    raise queue.Full
                self.items.append(value)

            def get_nowait(self):
                if not self.items:
                    raise queue.Empty
                return self.items.pop(0)

        preview = ReplayPreview.__new__(ReplayPreview)
        preview.requests = LatestQueue()
        preview.quit_requested = type('Event', (), {'is_set': lambda self: False})()
        preview.last_request = None
        preview.reported_error = True
        preview.disabled = False
        preview.update(100)
        preview.update(200)
        preview.update(200)
        self.assertEqual(preview.requests.items, [200])


if __name__ == '__main__':
    unittest.main()
