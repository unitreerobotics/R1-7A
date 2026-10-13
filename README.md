# R1Arm XR 遥操作、录制与回放

本项目面向独立 R1Arm 七轴机械臂，支持 PICO XR 遥操作、MuJoCo 仿真、本机相机采集、Dex1 夹爪和 MCAP 回放。真机动作有风险：启动前支撑机械臂、确认急停可触及；只有确认串口、标定文件和活动范围正确后，才输入 `MOVE`。

## 1. 环境与硬件准备

进入项目并加载真机环境（不会打开串口或驱动机械臂）：

```bash
cd /home/ycb410/nt/r1arm_teleop
conda activate r1_teleop
source scripts/real_env.sh
```

编译本项目调用的 R1Arm SDK 原生桥接库（首次使用或 SDK/桥接源码更新后执行）：

```bash
python scripts/build_r1arm_sdk_bridge.py \
  --sdk-root /home/ycb410/nt/R1_arm_SDK_x86/r1_arm
```

成功时会生成 `build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so`；脚本也可不带参数运行，默认使用上述 SDK 路径。若 CMake 或编译器报错，先检查 SDK 的 `include/`、`lib/` 文件以及本机 CMake/C++ 构建工具是否齐全。

### R1Arm SDK 准备、编译与标定

`r1arm_teleop` 和 `R1_arm_SDK_x86/r1_arm` ：前者包含遥操作程序，后者包含电机总线 SDK、低层诊断工具和关节标定文件。确认 SDK 路径后，可从项目 shell 编译 SDK 工具及其 vendored 电机库，再编译上面的 Python/C++ bridge：

```bash
export ARM=/home/ycb410/nt/R1_arm_SDK_x86/r1_arm
cmake -S "$ARM" -B "$ARM/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ARM/build" -j"$(nproc)"
```

构建产物应包括 `$ARM/bin/test_arm_lowlevel`、`$ARM/bin/calibrate_joints` 和 `$ARM/thirdparty/motor_controller/lib/libmotor_controller.so`。`source scripts/real_env.sh` 会设置 `ARM`、`PORT`、`CALIB`、`CANDIDATE`、`BIN` 及运行时库路径，并检查串口权限/SDK 工具是否存在；它不会替你编译，也不会打开串口。

当前遥操作、录制和回放的程序默认使用 `$ARM/config/joint_calib_candidate.txt`；环境脚本中的 `$CALIB` 则指向 `$ARM/config/joint_calib.txt`，它们不是同一文件。实验时显式指定本次确认过的文件，避免不同终端或工具误用另一份：

```bash
printf 'ARM=%s\nPORT=%s\nCALIB=%s\nCANDIDATE=%s\n' "$ARM" "$PORT" "$CALIB" "$CANDIDATE"
nl -ba "$CANDIDATE"                 # 应有 joint 0..6 的 dir / scale / offset
"$BIN/calibrate_joints" --help      # 只查看选项，不连接硬件
```

标定文件描述 `q_urdf = dir × scale × q_motor + offset`。它与具体机械臂、编码器零位和装配方向相关；不要把另一台机器的文件、`joint_calib.txt` 或 factory backup 当作可互换配置，也不要仅为“检查”而重新 capture/覆盖标定。只有确认参考姿态、七轴方向和零位都正确且确需重标定时，才使用 `calibrate_joints`；该工具会连接电机并以零力矩 FOC 采样，操作时必须托住机械臂，并将结果先写入候选路径、复核后再决定是否替换。启动真机前仍应检查 7 轴反馈与模型姿态一致；`goto_r1arm_pose.py` 的启动探测会短暂发送零力矩命令，同样需要支撑机械臂。

串口、SDK、bridge 和标定路径应指向同一套本机安装：当前已确认 R1Arm 是 FTDI `if02`（此机器 `/dev/ttyUSB2`），Dex1 是 `if00`（此机器 `/dev/ttyUSB0`）。端口号重排后优先用 `/dev/serial/by-id/`，并将实际设备传给 `--port`；`--calib` 则传给上述明确的候选标定文件。SDK 独立构建和标定说明见 [`R1_arm_SDK_x86/r1_arm/README.md`](../R1_arm_SDK_x86/r1_arm/README.md)。

确认 R1Arm 和 Dex1 串口，避免两者选到同一个设备：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB*
```
现在的接线方式是夹爪/dev/ttyUSB0，机械臂/dev/ttyUSB2
```bash
ls -l /dev/serial/by-id/
udevadm info --query=property --name=/dev/ttyUSB0
udevadm info --query=property --name=/dev/ttyUSB2
```

`/dev/serial/by-id/` 的稳定链接优先于易变化的 `/dev/ttyUSB*`。当前确认的端口是 R1Arm FTDI `if02`、Dex1 FTDI `if00`；服务脚本使用 Dex1 的稳定链接，避免 ttyUSB 编号变化。接线或设备变化后应重新核对。标定默认读取 `$ARM/config/joint_calib_candidate.txt`；不要在未确认电机方向/零位时换用其他文件。

检查相机设备和取帧：

```bash
python scripts/probe_r1arm_cameras.py
```

查看实际图像（按 `q` 退出）：

```bash
python scripts/view_r1arm_cameras.py
```

确认预期的头部场景相机与腕部相机都显示有效、持续更新的画面后再录制。设备号可能随重插变化；探测器会打印稳定 by-id 路径以及可复制到 `--camera` 的规格。若只有一路可用，先修复相机/USB 链路；确需单路采集时，用重复的 `--camera SPEC` 显式指定并检查录制启动信息。

确认机械臂串口反馈完整（该脚本的默认检查会短暂零力矩探测，不会移动到目标；请托住机械臂）：

```bash
python scripts/goto_r1arm_pose.py
```

检查输出中 7 轴反馈就绪、故障码为零，且测得姿态与现场一致。此命令不加 `--move`；它不会保持机械臂，探测期间必须支撑机械臂。

如需夹爪，在另一个终端启动本地串口到 DDS 服务并保持运行：

```bash
cd /home/ycb410/nt/r1arm_teleop
source scripts/real_env.sh
./scripts/start_dex1_local.sh
```

确认脚本打印的网络接口和 Dex1 串口正确。网卡默认 `enp4s0`，Dex1 默认使用 FTDI `if00` 稳定链接（当前为 `/dev/ttyUSB0`）。R1Arm 遥操作/录制/回放程序默认启用右侧 Dex1；若不使用夹爪，传 `--no-enable-dex1`。

### 默认参数的最简启动命令

以下命令使用程序默认参数（控制 90 Hz、录制 30 Hz、滤波 `8/1`、轨迹跟随器 `64/16`、电机增益 `60/10`、速度 `1 rad/s`）。真机程序都会要求在终端输入 `MOVE` 才开始运动：

```bash
# 摇操
python scripts/teleop_r1arm_real.py --motion-mode

# 录制；PICO 按 X 开始/停止，数据写入默认数据目录
python scripts/record_r1arm_real.py --motion-mode

# 回放；将 episode 路径替换为已录制的 MCAP
python scripts/replay_r1arm_real.py \
  --episode /path/to/episode_0000.mcap \
  --motion-mode
```

注意：最简命令使用默认 R1Arm 串口 `/dev/ttyUSB2`；如果实际串口不同，添加 `--port <设备路径>`。回放的 `--episode` 是必填项。

## 2. 真机遥操作

当前推荐基线：控制/IK 90 Hz；图像观测采样 30 Hz；XR One-Euro cutoff/beta `8/1`；`q_purpose` 跟随增益 `64/16`；SDK `Kp/Kd=60/10`；速度 `1 rad/s`，命令超前限制 `0.15 rad`；轨迹加速度/jerk 限制 `6 rad/s²` / `80 rad/s³`。

```bash
python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --rate 90 \
  --xr-filter-min-cutoff 8 --xr-filter-beta 1 \
  --trajectory-position-gain 64 --trajectory-velocity-gain 16 \
  --max-joint-speed 1.0 --max-command-lead 0.15 \
  --kp 60 --kd 10 \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start --enable-a-reset
```

程序要求输入 `MOVE` 才打开真机运动。右手 `grip` 按住时跟踪手柄，`trigger` 控制夹爪，`A` 触发软回 home，`Ctrl+C` 停止。`--home-on-start` 默认开启，启动后会先软移动到 `ARM_INIT`；测试前确保该路径无碰撞。

## 3. 真机录制

控制环按 90 Hz 运行，观测/图像帧按 30 Hz 采样；MCAP 同时包含 30 Hz `/whole_body/frame` 和控制周期级 `/r1arm/control_sample`。后者保留完整高频 action/state 快照，便于后续按图像时间戳构造 action chunk。它们是异步时间序列，不表示每个电机反馈都以 90 Hz 同步刷新；训练前仍需做对齐、分段和模型格式转换。

```bash
python scripts/record_r1arm_real.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --rate 90 --frequency 30 \
  --xr-filter-min-cutoff 8 --xr-filter-beta 1 \
  --trajectory-position-gain 64 --trajectory-velocity-gain 16 \
  --max-joint-speed 1.0 --max-command-lead 0.15 \
  --kp 60 --kd 10 \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start --enable-a-reset \
  --no-image-show \
  --task-description "pick and place demo" \
  --task-dir /home/ycb410/nt/r1arm_teleop/dataset/my_run
```

PICO 操作：`X` 開始/停止並保存 episode；`B` 暫停/恢復記錄；`A` 軟回 home。暫停期間不記錄，恢復後在同一 episode 中以新 `record_segment` 繼續；時間戳仍保留暫停造成的間隙，訓練切 chunk 時不可跨段。启动后查看 `Cameras active`，确认所有需要的相机都成功启动；保存时查看有效 FPS、控制周期、缺样、相机新鲜度和夹爪诊断摘要。

默认相机是 `color_0_head_left`（匹配 `SN0002` 的 UVC）和 `color_3_wrist_right`（RealSense）。可用 `--camera 'key=...,backend=...,device=...,size=640x480,fps=30'` 重复指定相机；以探测脚本输出为准。传 `--require-all-cameras` 可在所需相机缺失时中止，而不是继续录制部分视角。

## 4. 真机回放

默认从 MCAP 控制流读取 `q_purpose`，按记录的控制时间戳回放，并关闭额外轨迹整形；SDK 安全限幅依旧有效。先确认 episode、起始姿态和播放速度，短时测试并保持急停可触及。

```bash
python scripts/replay_r1arm_real.py \
  --episode /home/ycb410/nt/r1arm_teleop/dataset/my_run/episode_0000.mcap \
  --section action --data-source control \
  --start-frame 0 --end-frame -1 \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --rate 90 --speed 1.0 \
  --max-joint-speed 1.0 --max-command-lead 0.15 \
  --kp 60 --kd 10 \
  --enable-dex1 --dex1-net enp4s0
```

每次回放默认在 episode 旁生成唯一 telemetry CSV，记录目标/安全命令和实测关节反馈。`--section action` 播放期望 action；`--section state` 播放记录的实测关节位置（不是原始遥操作意图）。不要通过加速回放来做正常跟踪评估；这会改变目标时间历程，也会提高动态负载。

回放 30 Hz 观测流，指定 --data-source frame
```bash
python scripts/replay_r1arm_real.py \
  --episode /home/ycb410/nt/r1arm_teleop/dataset/my_run/episode_0000.mcap \
  --data-source frame --section action \
  --start-frame 0 --end-frame -1 \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --rate 30 --speed 1.0 \
  --max-joint-speed 1.0 --max-command-lead 0.15 \
  --kp 60 --kd 10 \
  --enable-dex1 --dex1-net enp4s0
```

## 5. 常用可选参数

| 参数 | 用途 |
|---|---|
| `--motion-mode` | 实际使能/驱动真机；不传时不进入真机运动模式。真机必须先确认安全再输入 `MOVE`。 |
| `--port`、`--calib` | R1Arm 串口和电机标定文件。 |
| `--rate` | 控制/IK 主循环频率；真机基线 90 Hz。 |
| `--frequency` | 录制图像和 `/whole_body/frame` 的频率；基线 30 Hz，不改变控制环频率。 |
| `--xr-filter-min-cutoff`、`--xr-filter-beta` | One-Euro 手柄位姿滤波：cutoff 越高平滑越少/响应越快；beta 越高快速运动时 cutoff 增长越多。 |
| `--trajectory-position-gain`、`--trajectory-velocity-gain` | `q_purpose` 跟随器位置响应和速度阻尼，基线 `64/16`。 |
| `--max-joint-speed`、`--max-command-lead` | SDK 目标速度兜底和相对实测位置的超前安全限制；提高前应先确认力矩/声音/发热。 |
| `--kp`、`--kd` | 电机底层位置/速度反馈增益。支持一个数广播到七轴，或逗号分隔七个逐轴值。 |
| `--enable-dex1`、`--no-enable-dex1`、`--dex1-net` | 启用/关闭 Dex1 以及 DDS 网卡。需另行运行 `start_dex1_local.sh`。 |
| `--home-on-start`、`--no-home-on-start` | 启动时是否软移动到 `ARM_INIT`。 |
| `--enable-a-reset`、`--no-enable-a-reset` | 是否允许 PICO A 键软回 home。 |
| `--reset-blend-frames` | Home/reset 轨迹的标称帧数，默认 60。 |
| `--camera SPEC` | 录制相机规格，可重复传入；语法参照 `probe_r1arm_cameras.py` 输出。 |
| `--require-all-cameras`、`--no-cameras` | 缺相机时中止；或录制时关闭图像采集。两者不要同时使用。 |
| `--no-image-show` | 关闭录制时的相机预览窗口，减少图形开销。 |
| `--task-description`、`--task-goal`、`--task-dir` | 记录实验描述、任务文本以及 episode 输出目录；`task-description` 是实验备注，不等同于语义任务标签。 |
| `--record-button`、`--pause-button` | 默认 `X` 控制 episode 开始/结束，`B` 暂停/恢复；可用 `none` 禁用暂停键。 |
| Replay `--episode` | 一个 MCAP 文件，或包含 `episode_*.mcap` 的目录（自动选最大编号）。 |
| Replay `--data-source auto\|control\|frame` | 选择高频控制流或观测帧流；默认 auto 优先 control。旧数据没有控制流时选 frame。 |
| Replay `--speed`、`--start-frame`、`--end-frame`、`--loop` | 回放倍速、范围与循环。诊断对照时固定相同范围并使用 `--speed 1`。 |
| Replay `--rate`、`--control-hz` | `--rate` 控制接近/末端保持刷新率；播放默认依照 MCAP 时间戳，`--control-hz` 才显式指定播放循环频率（非零时会插值）。 |
| Replay `--telemetry-csv` | 指定回放遥测 CSV 路径；默认自动生成旁路文件。 |

脚本完整参数以 `python scripts/<script>.py --help` 为准。QP 约束和 unshaped 等入口是实验对照，不属于上述真机基线；日常采集请使用 `teleop_r1arm_real.py` / `record_r1arm_real.py`。

## 6. 仿真和维护工具

```bash
python scripts/teleop_r1arm_sim.py --mock-xr
python scripts/record_r1arm_sim.py --help
python scripts/replay_r1arm_sim.py --help
python scripts/goto_r1arm_pose.py --help
python scripts/trajectory_test_r1arm.py --help
python scripts/build_r1arm_sdk_bridge.py
```

`--mock-xr` 仅用于仿真/干跑，不要与 `--motion-mode` 一起使用。真机前先用仿真确认模型和限位；桥接库缺失时按需构建。旧 R1-A7 双臂/DDS 遥操作入口不再维护，本仓库当前只支持 R1Arm。
