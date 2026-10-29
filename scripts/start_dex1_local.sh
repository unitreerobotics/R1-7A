#!/usr/bin/env bash
set -euo pipefail

# Start only the Dex1 serial-to-DDS service, never the R1Arm bus.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEX1_SERVICE_ROOT="${DEX1_SERVICE_ROOT:-${PROJECT_ROOT}/third_party/dex1_1_service}"
DEX1_SERVER="${DEX1_SERVER:-${DEX1_SERVICE_ROOT}/bin/dex1_1_gripper_server}"
SDK2_ROOT="${UNITREE_SDK2_ROOT:-${DEX1_DEP_ROOT:-${PROJECT_ROOT}/third_party}/unitree_sdk2}"
DEX1_SDK_LIB="${DEX1_SDK_LIB:-${SDK2_ROOT}/thirdparty/lib/$(uname -m)}"
DEX1_SERVICE_LIB="${DEX1_SERVICE_LIB:-${DEX1_SERVICE_ROOT}/lib}"

if [[ ! -x "$DEX1_SERVER" ]]; then
    echo "Dex1 server not built: $DEX1_SERVER" >&2
    echo "Build third_party/dex1_1_service as described in README.md." >&2
    exit 1
fi
# Activated environments are inherited by child shells, but the conda function is not.
if [[ -z "${CONDA_PREFIX:-}" || -n "${DEX1_CONDA_ENV:-}" ]]; then
    if [[ -n "${CONDA_EXE:-}" && -x "$CONDA_EXE" ]]; then
        CONDA_BASE="$("$CONDA_EXE" info --base)"
    elif command -v conda >/dev/null 2>&1; then
        CONDA_BASE="$(conda info --base)"
    else
        echo "No active Conda environment; activate r1_teleop or set CONDA_EXE to the conda executable." >&2
        exit 1
    fi
    set +u
    source "${CONDA_BASE}/etc/profile.d/conda.sh"
    conda activate "${DEX1_CONDA_ENV:-r1_teleop}"
    set -u
fi
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${DEX1_SERVICE_LIB}:${DEX1_SDK_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# Help must not require a connected gripper or initialize DDS.
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    exec "$DEX1_SERVER" --help
fi
: "${DEX1_NETWORK:?Set DEX1_NETWORK to the DDS interface shown by ip -br addr}"
: "${DEX1_PORT:?Set DEX1_PORT to the gripper serial device shown in /dev/serial/by-id}"
if [[ ! -e "$DEX1_PORT" || ! -r "$DEX1_PORT" || ! -w "$DEX1_PORT" ]]; then
    echo "Dex1 serial port is missing or inaccessible: $DEX1_PORT" >&2
    echo 'Check the device and dialout membership; log in again after adding the group.' >&2
    exit 1
fi
if ! ip link show "$DEX1_NETWORK" >/dev/null 2>&1; then
    echo "DDS interface does not exist: $DEX1_NETWORK; check ip -br addr." >&2
    exit 1
fi
DEX1_PORT="$(readlink -f "$DEX1_PORT")"
echo "Starting Dex1: server=$DEX1_SERVER network=$DEX1_NETWORK port=$DEX1_PORT"
echo "Press Ctrl+C to stop."
exec "$DEX1_SERVER" --network "$DEX1_NETWORK" --port "$DEX1_PORT" "$@"
