#!/usr/bin/env bash
# Source only: configure paths and inspect files/permissions, never open hardware.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ARM="${R1ARM_SDK_ROOT:-$PROJECT_ROOT/../R1_arm_SDK_x86/r1_arm}"
export R1ARM_SDK_ROOT="$ARM"
export R1ARM_SERIAL_PREFIX="${R1ARM_SERIAL_PREFIX:-/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBPG771-}"
export PORT_RIGHT="${PORT_RIGHT:-${R1ARM_SERIAL_PREFIX}if01-port0}"
export PORT_LEFT="${PORT_LEFT:-${R1ARM_SERIAL_PREFIX}if03-port0}"
export RIGHT_DEX1_PORT="${RIGHT_DEX1_PORT:-${R1ARM_SERIAL_PREFIX}if00-port0}"
export LEFT_DEX1_PORT="${LEFT_DEX1_PORT:-${R1ARM_SERIAL_PREFIX}if02-port0}"
# Deliberately reset stale right-calibration shell aliases; CLI overrides still win.
export CALIB_RIGHT="$ARM/config/joint_calib_zero_右臂.txt"
export CALIB_LEFT="${CALIB_LEFT:-$ARM/config/joint_calib_zero_左臂.txt}"
export CALIB="$CALIB_RIGHT"
export CANDIDATE="$CALIB_RIGHT"
export PORT="${PORT:-$PORT_RIGHT}"
export BIN="$ARM/bin"
# Prefer the verified local bridge; fresh installs use the standard build output.
if [[ -z "${R1ARM_BRIDGE_LIB:-}" ]]; then
    if [[ -f "$PROJECT_ROOT/build/dual_sdk_bridge/libr1arm_sdk_bridge.so" ]]; then
        export R1ARM_BRIDGE_LIB="$PROJECT_ROOT/build/dual_sdk_bridge/libr1arm_sdk_bridge.so"
    else
        export R1ARM_BRIDGE_LIB="$PROJECT_ROOT/build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so"
    fi
fi
export DEX1_NETWORK="${DEX1_NETWORK:-lo}"
export LD_LIBRARY_PATH="${CONDA_PREFIX:+$CONDA_PREFIX/lib:}$ARM/thirdparty/motor_controller/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
printf 'ARM=%s\nRIGHT=%s\nLEFT=%s\nCALIB_RIGHT=%s\nCALIB_LEFT=%s\nDDS=%s\n' "$ARM" "$PORT_RIGHT" "$PORT_LEFT" "$CALIB_RIGHT" "$CALIB_LEFT" "$DEX1_NETWORK"
for path in "$PORT_RIGHT" "$PORT_LEFT" "$RIGHT_DEX1_PORT" "$LEFT_DEX1_PORT"; do
    if [[ -e "$path" && -r "$path" && -w "$path" ]]; then
        printf 'Accessible: %s\n' "$path"
    else
        printf 'Missing/inaccessible: %s\n' "$path"
    fi
done
for path in "$CALIB_RIGHT" "$CALIB_LEFT"; do
    [[ -r "$path" ]] || printf 'Calibration not readable: %s\n' "$path"
done
if [[ " $(id -nG) " != *" dialout "* ]]; then
    echo 'Current session lacks dialout; after group setup log in again, reactivate Conda and source this file.'
fi
