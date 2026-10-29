"""Exercise concurrent DDS startup on loopback without publishing motor commands."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import os
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from robot.eef import dex1_single


class Dex1LauncherEnvironment(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='dex1 launch ')
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        server = self.root / 'fake_server'
        server.write_text('#!/bin/bash\nprintf "PREFIX=%s\\nLIBS=%s\\nARGS=%s\\n" "$CONDA_PREFIX" "$LD_LIBRARY_PATH" "$*"\n')
        server.chmod(0o700)
        self.env = dict(PATH='/usr/bin:/bin', DEX1_SERVER=str(server))
        self.script = Path(__file__).resolve().parents[1] / 'scripts/start_dex1_local.sh'

    def launch(self):
        return subprocess.run(['/bin/bash', str(self.script), '--help'], env=self.env,
                              capture_output=True, text=True, timeout=10)

    def test_active_environment_does_not_require_conda_in_child_path(self):
        prefix = str(self.root / 'active environment')
        self.env.update(CONDA_PREFIX=prefix, LD_LIBRARY_PATH='/existing/lib')
        result = self.launch()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('PREFIX='+prefix, result.stdout)
        self.assertIn('LIBS='+prefix+'/lib:', result.stdout)
        self.assertIn('/existing/lib', result.stdout)
        self.assertIn('ARGS=--help', result.stdout)

    def test_conda_executable_fallback_and_explicit_environment_override(self):
        base = self.root / 'conda base'
        profile = base / 'etc/profile.d/conda.sh'
        profile.parent.mkdir(parents=True)
        profile.write_text('conda() { export CONDA_PREFIX="$TEST_CONDA_BASE/envs/$2"; }\n')
        executable = self.root / 'conda'
        executable.write_text('#!/bin/bash\nprintf "%s\\n" "$TEST_CONDA_BASE"\n')
        executable.chmod(0o700)
        self.env.update(CONDA_EXE=str(executable), TEST_CONDA_BASE=str(base))
        for override in (None, 'chosen'):
            if override:
                self.env.update(CONDA_PREFIX='/previous', DEX1_CONDA_ENV=override)
            result = self.launch()
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(f'PREFIX={base}/envs/{override or "r1_teleop"}', result.stdout)

    def test_missing_environment_fails_before_server(self):
        result = self.launch()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('No active Conda environment', result.stderr)
        self.assertNotIn('ARGS=', result.stdout)


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
