"""Shell and Python must select the same adapter and keep port-role guards."""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest


class SerialConfig(unittest.TestCase):
    def test_replacement_adapter_is_shared_with_shell_and_preserves_guards(self):
        root = Path(__file__).resolve().parents[1]
        env = dict(os.environ, R1ARM_SERIAL_PREFIX='/dev/serial/by-id/usb-FTDI_TEST_TESTSERIAL-')
        for name in ('PORT', 'PORT_RIGHT', 'PORT_LEFT', 'RIGHT_DEX1_PORT', 'LEFT_DEX1_PORT'):
            env.pop(name, None)
        code = '''
import json, os
from dual.config import PORTS, validate_pair, validate_arm_port
mapping = {'right':'PORT_RIGHT', 'left':'PORT_LEFT', 'grip_right':'RIGHT_DEX1_PORT', 'grip_left':'LEFT_DEX1_PORT'}
assert all(str(PORTS[side]) == os.environ[name] for side,name in mapping.items())
assert os.environ['PORT'] == str(PORTS['right'])
validate_pair(PORTS['right'], PORTS['left'], 'right-calib', 'left-calib')
for port, side in ((PORTS['grip_right'],'right'), (PORTS['grip_left'],'left'), (PORTS['left'],'right'), (PORTS['right'],'left')):
    try:
        validate_arm_port(port, side)
    except ValueError:
        pass
    else:
        raise AssertionError((port, side))
print(json.dumps({side:str(port) for side,port in PORTS.items()}))
'''
        result = subprocess.run(['bash', '-c', 'source scripts/real_env.sh >/dev/null; exec "$1" -c "$2"',
                                 'serial-check', sys.executable, code], cwd=root, env=env,
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        ports = json.loads(result.stdout)
        self.assertEqual(ports['right'], env['R1ARM_SERIAL_PREFIX'] + 'if01-port0')
        self.assertEqual(ports['left'], env['R1ARM_SERIAL_PREFIX'] + 'if03-port0')
