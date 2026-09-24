#!/usr/bin/env bash
set -euo pipefail

# Start the locally connected Unitree Dex1 serial-to-DDS service.
# This script only starts Dex1; it does not open or command the R1Arm.

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DEX1_SERVICE_ROOT="${DEX1_SERVICE_ROOT:-/home/ycb410/nt/dex1/dex1_1_service}"
DEX1_SERVER="${DEX1_SERVER:-${DEX1_SERVICE_ROOT}/bin/dex1_1_gripper_server}"
DEX1_NETWORK="${DEX1_NETWORK:-enp4s0}"
DEX1_PORT="${DEX1_PORT:-/dev/ttyUSB0}"
DEX1_SDK_LIB="${DEX1_SDK_LIB:-/home/ycb410/nt/dex1/unitree_sdk2/thirdparty/lib/x86_64}"
DEX1_SERVICE_LIB="${DEX1_SERVICE_LIB:-${DEX1_SERVICE_ROOT}/lib}"

if [[ ! -x "$DEX1_SERVER" ]]; then
    echo "Dex1 server not found or not executable: $DEX1_SERVER" >&2
    echo "Build it first in: ${DEX1_SERVICE_ROOT}/build" >&2
    exit 1
fi

if [[ ! -e "$DEX1_PORT" ]]; then
    echo "Dex1 serial port does not exist: $DEX1_PORT" >&2
    echo "Available ports:" >&2
    ls -l /dev/ttyUSB* 2>/dev/null || true
    exit 1
fi

if ! ip link show "$DEX1_NETWORK" >/dev/null 2>&1; then
    echo "DDS network interface does not exist: $DEX1_NETWORK" >&2
    echo "Use 'ip -br addr' to check the interface name." >&2
    exit 1
fi

if ! command -v conda >/dev/null 2>&1; then
    echo "conda was not found in PATH." >&2
    exit 1
fi

CONDA_BASE="$(conda info --base)"
# Load conda's shell functions, then use the environment containing the newer
# libstdc++ required by libUnitreeMotorSDK_Linux64.so.
# shellcheck disable=SC1091
set +u
source "${CONDA_BASE}/etc/profile.d/conda.sh"
conda activate "${DEX1_CONDA_ENV:-r1_teleop}"
set -u

export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${DEX1_SDK_LIB}:${DEX1_SERVICE_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

echo "Starting local Dex1 service"
echo "  server  : $DEX1_SERVER"
echo "  network : $DEX1_NETWORK"
echo "  port    : $DEX1_PORT"
echo "  project : $PROJECT_ROOT"
echo "Press Ctrl+C to stop Dex1 service."

exec sudo env "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}" \
    "$DEX1_SERVER" \
    --network "$DEX1_NETWORK" \
    --port "$DEX1_PORT" \
    "$@"
