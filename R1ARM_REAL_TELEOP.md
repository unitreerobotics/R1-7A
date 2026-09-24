# R1Arm 单臂真机遥操作

该入口复用已经在 MuJoCo 中验证的 `R1ArmTeleopControllerBase`：PICO 手柄、相对位姿、
Placo/DLS IK、grip 激活与 trigger 夹爪逻辑不变，只把 MuJoCo position actuator 输出替换成
独立 R1Arm SDK 的 `ArmMotorBus` 串口输出。

> 真机入口是 `scripts/teleop_r1arm_real.py`。不要用 R1-A7 的
> `scripts/teleop_real.py` 控制桌面式 R1Arm。

## 1. 前置检查

确认机械臂固定可靠、急停可触及，并先找出真实串口：

```bash
ls -l /dev/serial/by-id/ 2>/dev/null
ls -l /dev/ttyUSB* 2>/dev/null
```

SDK 默认示例使用 `/dev/ttyUSB3`，但实际编号必须以本机枚举结果为准。当前用户应属于
`dialout` 组。机械臂本体经 RS485 串口控制，`--dex1-net` 仅供 Dex1 DDS 使用。

## 2. 编译 Python/C++ 桥接库

桥接库只封装 SDK 的 `ArmMotorBus`，不会在编译时连接或驱动机械臂：

```bash
conda activate r1_teleop
cd /home/ycb410/nt/r1arm_teleop

python scripts/build_r1arm_sdk_bridge.py \
  --sdk-root /home/ycb410/nt/R1_arm_SDK_x86/r1_arm
```

默认输出：

```text
build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so
```

## 3. 不连接真机的 dry-run

以下命令不会打开 SDK，也不会发送电机命令：

```bash
python scripts/teleop_r1arm_real.py --mock-xr --debug
```

也可连接 PICO 只检查手柄与 IK，但仍不发送真机命令：

```bash
python scripts/teleop_r1arm_real.py --debug
```

## 4. 真机启动

首次建议先不启用 Dex1：

```bash
python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port "$PORT"
```

程序显示全部参数后，必须在终端输入大写 `MOVE` 才会打开串口。启动流程为：

1. 零增益 FOC 探测全部 7 个关节反馈；此时请托住机械臂。
2. 检查标定后的关节角是否落在仿真 URDF 限位内。
3. 用真机实测关节角初始化 IK，避免从仿真 home 突跳。
4. 在实测启动姿态进入 FOC 位置保持。
5. 按住对应手柄 grip 后才建立末端相对跟踪。

默认右手柄。正常启动后：

- 按住右 grip：控制机械臂末端相对移动。
- 松开 grip：保持最后目标位置。
- `Ctrl+C`：持续发送 0.5 秒 mode 0，然后关闭串口；机械臂会失去位置保持，需托住机械臂。
- A 键回 home 默认关闭，避免真机意外大范围运动。

若确实需要 A 键缓慢回 `ARM_INIT`：

```bash
python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port "$PORT" \
  --enable-a-reset \
  --reset-blend-frames 250
```

## 5. 启用单侧 Dex1

R1Arm 电机仍走串口，Dex1 单独走 DDS。以本机当前有线网卡 `enp4s0` 为例：

```bash
python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port "$PORT" \
  --enable-dex1 \
  --dex1-net enp4s0
```

程序只发布所选侧的 topic；默认右侧为：

```text
rt/dex1/right/cmd
rt/dex1/right/state
```

如果 2 秒内收不到所选侧的 state，程序会拒绝继续并停止机械臂。

右 trigger：松开为开，按下为关。

## 6. 坐标与安全限制

仿真/IK 使用项目中的 URDF。SDK 附带 URDF 的 J4 采用相反的轴和角度符号，因此桥接层只对
J4 做显式符号转换：

```text
q_sdk[J4] = -q_sim[J4]
```

其余 J1～J3、J5～J7 一一对应。输出端还会依次执行：

- URDF 关节硬限位；
- 默认 `0.5 rad/s` 的目标变化限制；
- 相对实测位置默认 `0.15 rad` 的最大指令超前量；
- 电机错误码与 FOC 模式检查；
- SDK 电机通信超时保护；
- XR 断流保持当前位置，禁止自动回退到 mock XR。

可用 `--help` 查看增益、速度与超时参数。不要在没有官方确认时随意提高 `--kp`、
`--max-joint-speed` 或 `--max-command-lead`。

当前 A/home reset 默认使用 `100` 帧（50 Hz 下约 2 秒），并采用 smoothstep 缓入缓出：
起步和接近 home 时减速，中段较快；真机输出仍会经过关节速度和命令超前限制。
