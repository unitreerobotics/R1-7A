"""Local calibration overrides must take effect without changing shared defaults."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot.eef.dex1_config import load_dex1_endpoints


class Dex1Endpoints(unittest.TestCase):
    def test_local_override_and_explicit_path_precedence(self):
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory) / 'local.json'
            explicit = Path(directory) / 'explicit.json'
            local.write_text(json.dumps({'right': {'closed': 1.8, 'open': 7.1}}))
            explicit.write_text(json.dumps({'right': {'closed': 2., 'open': 6.}}))
            with patch.dict(os.environ, {'DEX1_ENDPOINTS_FILE': str(local)}):
                self.assertEqual(load_dex1_endpoints('right'), (1.8, 7.1))
                self.assertEqual(load_dex1_endpoints('right', explicit), (2., 6.))
                local.write_text(json.dumps({'right': {'closed': 7.1, 'open': 1.8}}))
                with self.assertRaises(ValueError):
                    load_dex1_endpoints('right')
                local.unlink()
                with self.assertRaises(FileNotFoundError):
                    load_dex1_endpoints('right')
