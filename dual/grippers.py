"""Start one or both official Dex1 services with independent ID7 buses/topics."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import time

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dual.config import ROOT, PORTS


def port_users(port):
    """Read fd link text without stat-ing unrelated, possibly stalled mounts."""
    target = str(Path(port).resolve())
    users = set()
    deadline = time.monotonic() + 3.0
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        if time.monotonic() > deadline:
            raise RuntimeError(f'Port ownership inspection timed out: {port}')
        try:
            for fd in (process / 'fd').iterdir():
                try:
                    if os.readlink(fd) == target:
                        users.add(int(process.name))
                except (FileNotFoundError, PermissionError, ProcessLookupError):
                    continue
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return sorted(users)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('right','left','both'), default='both')
    parser.add_argument('--network', default=os.environ.get('DEX1_NETWORK', 'lo'))
    parser.add_argument('--right-port', type=Path, default=PORTS['grip_right'])
    parser.add_argument('--left-port', type=Path, default=PORTS['grip_left'])
    parser.add_argument('--right-id', type=int, default=7)
    parser.add_argument('--left-id', type=int, default=7)
    args = parser.parse_args(argv)
    if args.side == 'both' and args.right_port.resolve() == args.left_port.resolve():
        parser.error('Gripper ports refer to the same device')
    if any(not 0 <= mid <= 14 for mid in (args.right_id, args.left_id)):
        parser.error('Motor IDs must be 0..14 (broadcast 15 is forbidden)')
    if subprocess.run(['ip', 'link', 'show', args.network], capture_output=True).returncode:
        parser.error(f'DDS interface does not exist: {args.network}')
    entries = [(side,port,mid) for side,port,mid in (('right',args.right_port,args.right_id),('left',args.left_port,args.left_id)) if args.side in (side,'both')]
    processes, locks = [], []
    lock_dir = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / f'r1arm-grippers-{os.getuid()}'
    lock_dir.mkdir(mode=0o700, exist_ok=True)
    if lock_dir.stat().st_uid != os.getuid() or lock_dir.is_symlink():
        parser.error('Unsafe gripper lock directory')
    def interrupted(*_):
        raise KeyboardInterrupt
    previous = {s: signal.signal(s, interrupted) for s in (signal.SIGTERM, signal.SIGINT)}
    try:
        for _, port, _ in entries:
            if not port.exists() or not os.access(port, os.R_OK | os.W_OK):
                raise RuntimeError(f'Gripper port missing or inaccessible: {port}')
            lock = (lock_dir / (port.resolve().name + '.lock')).open('a')
            locks.append(lock)
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            print(f'Checking Dex1 port: {port}', flush=True)
            users = port_users(port)
            if users:
                raise RuntimeError(f'Port is already used: {port}; PIDs={users}; existing service was not killed')
        for side, port, mid in entries:
            env = dict(os.environ, DEX1_PORT=str(port), DEX1_NETWORK=args.network)
            processes.append(subprocess.Popen(['bash', str(ROOT/'scripts/start_dex1_local.sh'),
                '--motor-id', str(mid), '--side', side], env=env, start_new_session=True))
        print(f'Official Dex1: {entries}; DDS={args.network}', flush=True)
        while all(p.poll() is None for p in processes):
            time.sleep(.1)
        failed = next(p for p in processes if p.poll() is not None)
        raise RuntimeError(f'Gripper service exited ({failed.returncode}); stopping managed services')
    except KeyboardInterrupt:
        pass
    finally:
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for process in processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for lock in locks:
            lock.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
