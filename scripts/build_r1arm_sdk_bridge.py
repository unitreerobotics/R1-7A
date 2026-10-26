#!/usr/bin/env python3
"""Configure and build the C ABI wrapper around the standalone R1Arm SDK."""

from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIR = PROJECT_ROOT / "native/r1arm_sdk_bridge"
DEFAULT_BUILD_DIR = PROJECT_ROOT / "build/r1arm_sdk_bridge"
DEFAULT_SDK_ROOT = Path(
    os.environ.get("R1ARM_SDK_ROOT", str(PROJECT_ROOT / "third_party" / "r1_arm_sdk"))
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--jobs", type=int, default=0, help="Parallel jobs (0 = CMake default)")
    args = parser.parse_args()

    sdk_root = args.sdk_root.expanduser().resolve()
    build_dir = args.build_dir.expanduser().resolve()
    subprocess.run(
        [
            "cmake",
            "-S",
            str(SOURCE_DIR),
            "-B",
            str(build_dir),
            f"-DR1ARM_SDK_ROOT={sdk_root}",
            "-DCMAKE_BUILD_TYPE=Release",
        ],
        check=True,
    )
    command = ["cmake", "--build", str(build_dir)]
    if args.jobs > 0:
        command += ["-j", str(args.jobs)]
    subprocess.run(command, check=True)
    output = build_dir / "libr1arm_sdk_bridge.so"
    if not output.is_file():
        raise SystemExit(f"Build completed but bridge was not found: {output}")
    print(f"R1Arm SDK bridge ready: {output}")


if __name__ == "__main__":
    main()
