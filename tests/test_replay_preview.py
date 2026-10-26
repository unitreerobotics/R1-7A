"""Offline checks for preview composition and latest-only IPC behavior."""
import base64
import queue
import unittest

import cv2
import numpy as np

from teleop.replay_preview import ReplayPreview, _mosaic
from scripts.replay_r1arm_real import build_parser as real_parser
from scripts.replay_r1arm_sim import build_parser as sim_parser
from dual.replay import parse_args as dual_parse_args


class ReplayPreviewTest(unittest.TestCase):
    def test_replay_entries_show_images_by_default_and_allow_disabling(self):
        for parser in (real_parser(), sim_parser()):
            self.assertTrue(parser.parse_args(['--episode', 'unused.mcap']).show_observations)
            self.assertFalse(parser.parse_args(['--episode', 'unused.mcap', '--no-show-observations']).show_observations)
        self.assertTrue(dual_parse_args(['--episode', 'unused.mcap']).show_observations)
        self.assertFalse(dual_parse_args(['--episode', 'unused.mcap', '--no-show-observations']).show_observations)

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
