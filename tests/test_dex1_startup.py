"""Exercise concurrent DDS startup on loopback without publishing motor commands."""
from concurrent.futures import ThreadPoolExecutor
import threading
import time
import unittest
from unittest.mock import patch

from robot.eef import dex1_single


class Dex1Startup(unittest.TestCase):
    def test_parallel_hands_serialize_shared_dds_type_initialization(self):
        barrier = threading.Barrier(2)
        counter_lock = threading.Lock()
        active = 0
        peak = 0
        publisher = dex1_single.ChannelPublisher

        def create_publisher(*args):
            nonlocal active, peak
            with counter_lock:
                active += 1
                peak = max(peak, active)
            try:
                # Release the GIL while shared types are still being initialized.
                time.sleep(.02)
                return publisher(*args)
            finally:
                with counter_lock:
                    active -= 1

        def start(side):
            barrier.wait(timeout=5)
            return dex1_single.Dex1Single(side, interface='lo', domain=97)

        with patch.object(dex1_single, 'ChannelPublisher', side_effect=create_publisher), \
                patch.object(dex1_single.Dex1Single, 'state_received', property(lambda self: True)):
            with ThreadPoolExecutor(max_workers=2) as pool:
                futures = [pool.submit(start, side) for side in ('right', 'left')]
                hands = []
                try:
                    for future in futures:
                        hands.append(future.result(timeout=10))
                    self.assertEqual(peak, 1)
                    self.assertEqual({hand.side for hand in hands}, {'right', 'left'})
                finally:
                    for future in futures:
                        if future.done() and future.exception() is None:
                            future.result().stop()


if __name__ == '__main__':
    unittest.main()
