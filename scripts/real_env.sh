#!/usr/bin/env bash
# 真机调试环境变量。用法：
#   source scripts/real_env.sh
#
# 只设变量 + 自检，不打开串口、不碰硬件。
# 所有真机命令都假设这些变量已经设好。

export ARM=/home/ycb410/nt/R1_arm_SDK_x86/r1_arm
export PORT="${PORT:-/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTANPO49-if02-port0}"
export CALIB="$ARM/config/joint_calib.txt"
export CANDIDATE="$ARM/config/joint_calib_candidate.txt"
export BIN="$ARM/bin"
export LD_LIBRARY_PATH="$ARM/thirdparty/motor_controller/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "ARM       = $ARM"
echo "PORT      = $PORT"
echo "CALIB     = $CALIB"
echo "CANDIDATE = $CANDIDATE"

if id -nG | grep -qw dialout; then
  echo "dialout   = OK"
else
  echo "dialout   = 缺失!  先执行:  newgrp dialout   (或注销重登)"
fi

if [ ! -e "$PORT" ]; then
  echo "串口      = 路径不存在（检查 FTDI 序列号和 PORT）"
elif [ -r "$PORT" ] && [ -w "$PORT" ]; then
  echo "串口      = 可读写"
else
  echo "串口      = 无权限（确认当前 shell 已加入 dialout）"
fi

if [ -x "$BIN/test_arm_lowlevel" ]; then
  echo "工具      = $BIN/{test_arm_lowlevel,calibrate_joints} 就绪"
else
  echo "工具      = 缺失!  检查 $BIN"
fi
