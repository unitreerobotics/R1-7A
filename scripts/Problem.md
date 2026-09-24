结论：PICO、仿真遥操和 IK 目前基本正常。距离“PICO 实机遥操真机”还差两个核心问题：七个电机的总线反馈稳定性，以及真机关节零位标定。

```text
PICO 手柄
   ↓ XRoboToolkit
仿真中已验证的遥操/IK
   ↓ q_sim
J4 符号转换
   ↓ q_sdk
限位、速度、跟随误差保护
   ↓
ArmMotorBus → USB-RS485 → R1Arm 七个电机
```

## 已经完成的部分

- PICO 可以连接 PC，XRoboToolkit 能识别设备。
- `teleop_r1arm_sim.py` 在 MuJoCo 中可以正常遥操。
- 仿真 URDF 与真机七轴结构能够对应。
- 单臂实机遥操入口已经建立：
  [teleop_r1arm_real.py](/home/ycb410/nt/r1arm_teleop/scripts/teleop_r1arm_real.py)
- 仿真 IK、手柄控制逻辑、速度限制和安全检查已经复用到实机控制器。
- x86 SDK 已能编译运行，原来的动态库架构问题不再是当前主要阻塞。
- `/dev/ttyUSB0 @ 6000000` 可以成功打开。
- 已加入安全的单电机通信诊断参数 `--probe-id`。

## 当前问题一：七电机总线反馈不稳定

执行：

```bash
./test_arm_lowlevel \
  --zero-torque \
  --port /dev/ttyUSB0 \
  --baud 6000000 \
  --rate 50
```

当前结果是：

```text
Feedback incomplete: j1 j2 j3 j4 j5 j6
```

即这一次只有 ID 0 返回。

但之前运行实机遥操时曾经收到过七个关节的反馈，随后才因为角度超限而退出。因此更像是多电机轮询时序不稳定，而不是后六个电机全部断线。

目前代码连续给七个 ID 发包，只间隔约 `115 μs`，没有严格执行：

```text
发送 ID 0 → 等待 ID 0 回复
发送 ID 1 → 等待 ID 1 回复
...
```

这可能导致半双工 RS485 应答冲突。

### 下一步诊断

依次运行：

```bash
cd /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/bin

./test_arm_lowlevel --probe-id 0 --port /dev/ttyUSB0
./test_arm_lowlevel --probe-id 1 --port /dev/ttyUSB0
./test_arm_lowlevel --probe-id 2 --port /dev/ttyUSB0
./test_arm_lowlevel --probe-id 3 --port /dev/ttyUSB0
./test_arm_lowlevel --probe-id 4 --port /dev/ttyUSB0
./test_arm_lowlevel --probe-id 5 --port /dev/ttyUSB0
./test_arm_lowlevel --probe-id 6 --port /dev/ttyUSB0
```

每次看到稳定输出后按 `Ctrl+C`，再测试下一个。

判断：

- 七个 ID 都成功：硬件通信基本正常，需要把多电机通信改成“一问一答”。
- 某个 ID 始终失败：检查对应电机的 ID、供电和通信线。
- 只有 ID 0 成功：检查实际电机 ID 配置以及串联通信链路。

## 当前问题二：关节标定明显不正确

之前实机反馈转换到仿真关节后是：

```text
J1=-4.080
J3=+5.474
J4=+1.945
J5=-5.722
J6=+5.585
J7=-6.721
```

这些值严重超过机械臂关节范围，所以实机控制器在真正发送位置控制前主动退出了。这个安全检查是正确的，避免了机械臂突然跳动。

当前标定文件：

```text
config/joint_calib.txt
```

中的零偏与这台真机当前编码器零位不匹配。

标定关系是：

```text
q_sdk = dir × scale × q_motor_raw + offset
```

仿真初始姿态是：

```text
q_sim_init = [0, 1.2, 0, -1.2, 0, 0, 0]
```

由于 SDK URDF 的 J4 与仿真 URDF 方向相反，对应的 SDK 参考值应为：

```text
q_sdk_ref = [0, 1.2, 0, 1.2, 0, 0, 0]
```

需要把真机人工摆到与仿真初始姿态相同的物理姿态，然后根据原始电机角度计算：

```text
offset = q_sdk_ref - dir × scale × q_motor_raw
```

现有 `calibrate_joints` 只会把当前姿态标成全零，不能直接标成上述非零参考姿态，因此还需要增加 `--reference-q` 功能。

## 标定后还要验证的内容

单个姿态一致只能验证零偏，不能完全验证方向和比例。标定后还需要：

1. 静止时检查：

```text
q_sdk ≈ [0, 1.2, 0, 1.2, 0, 0, 0]
```

建议误差小于 `0.02～0.05 rad`。

2. 零力矩下逐个小幅移动关节，检查：

- 读取的是正确关节；
- 正负方向正确；
- 角度变化量一致；
- J4 经过符号转换后与仿真方向一致；
- 所有关节始终处于机械限位内。

## 最终实施顺序

1. 使用 `--probe-id 0～6` 确认每个电机独立通信。
2. 将七电机通信改成严格的一问一答，确认 50 Hz 下持续稳定。
3. 给标定工具增加非零 `q_sdk_ref` 标定。
4. 验证零偏、方向、比例和关节限位。
5. 先运行实机遥操的只读/dry-run模式，确认目标关节角正确。
6. 以低增益、低速度、低跟随范围启动真机位置保持。
7. 确认真机启动不跳变后，再用 PICO grip 激活遥操。
8. 最后接入 Dex1 夹爪和数据采集。

最终可用命令预计是：

```bash
cd /home/ycb410/nt/r1arm_teleop

python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port /dev/ttyUSB0 \
  --scale-factor 0.3 \
  --debug
```

但在“七电机稳定反馈”和“标定正确”完成前，不能安全执行实机运动。目前真正的阻塞点不是 PICO，也不是仿真 IK，而是真机底层通信和编码器到 URDF 的关节标定。

---

## 2026-09-23 已落地的修复

底层 SDK 已做以下修改并完成离线编译：

1. `ArmMotorBus::step()` 已从“固定等待 115 μs”改为严格的逐轴事务：

   ```text
   发 ID N → 等待 ID N 的反馈序号递增（最多 5 ms）→ 再发 ID N+1
   ```

2. 新增逐轴累计 `replies` 与 `timeouts` 统计；`--zero-torque` 和
   `--probe-id` 每秒显示统计。
3. Python 真机桥接在位置控制阶段要求本周期七轴都有新反馈；任一轴超时会抛错，
   上层进入 `finally` 并发送停机命令，避免使用历史反馈继续控制。
4. `calibrate_joints` 新增 `--reference-q`，标定公式改为：

   ```text
   offset = reference_q - dir × scale × motor_q
   ```

5. `test_arm_lowlevel` 新增 `--calib PATH`，因此可以验证候选标定文件而不覆盖原文件。

## 建议验收流程（每一关通过后再进入下一关）

### Gate 0：硬件与急停

- 机械臂固定可靠，两人配合；一人托住机械臂，一人操作终端和急停。
- 首次测试不安装夹爪负载，或至少确认负载不会在零力矩时坠落。
- `/dev/ttyUSB0` 当前已枚举为 FTDI 设备，当前用户属于 `dialout` 组。
- 尽量使用 `/dev/serial/by-id/...` 的稳定路径，避免重启后 ttyUSB 编号变化。

以下所有 `--zero-torque` / 标定操作都会进入零增益 FOC，机械臂不会保持姿态，必须托住。

### Gate 1：逐个 ID 验证

```bash
cd /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/bin

./test_arm_lowlevel --probe-id 0 --port /dev/ttyUSB0 --baud 6000000
./test_arm_lowlevel --probe-id 1 --port /dev/ttyUSB0 --baud 6000000
./test_arm_lowlevel --probe-id 2 --port /dev/ttyUSB0 --baud 6000000
./test_arm_lowlevel --probe-id 3 --port /dev/ttyUSB0 --baud 6000000
./test_arm_lowlevel --probe-id 4 --port /dev/ttyUSB0 --baud 6000000
./test_arm_lowlevel --probe-id 5 --port /dev/ttyUSB0 --baud 6000000
./test_arm_lowlevel --probe-id 6 --port /dev/ttyUSB0 --baud 6000000
```

每个 ID 至少运行 20～30 秒。通过标准：对应 `replies` 持续增长，`timeouts=0`，
角度不是固定零或乱跳，错误码为 0。若某轴单独也失败，优先查该轴 ID、供电、接插件和
串联链路；这时不要继续做标定。

### Gate 2：七轴总线压力测试

```bash
./test_arm_lowlevel \
  --zero-torque \
  --port /dev/ttyUSB0 \
  --baud 6000000 \
  --rate 50
```

先运行 2 分钟，稳定后运行 10 分钟。通过标准：

- 七个 `replies` 以近似相同速度持续增长；
- 七个 `timeouts` 始终为 0；
- 没有 CRC 错误、错误码或串口消失；
- 10 分钟约应收到每轴 30000 个回复（允许启停时有少量计数差，但不允许 timeout）。

如果单轴都通过但多轴仍超时，再检查 USB-RS485 转换器的自动收发切换、公共地、线束拓扑、
供电压降和终端电阻（终端电阻按硬件手册配置，不要盲目重复添加）。

### Gate 3：先验证方向/比例，再捕获零偏

零偏不影响角度变化量。保持零力矩并每次只人工移动一个关节约 `0.10～0.20 rad`，观察：

- 只有对应的 `jN q` 明显变化；
- SDK 坐标中的变化方向与 SDK URDF 一致；
- 变化量与实体转角基本一致，比例误差建议小于 5%；
- J4 特别处理：SDK J4 为正弯曲，仿真 J4 为负弯曲，两者由 Python 层取反。

当前 `dir/scale` 可作为候选值，但不能仅凭单个参考姿态证明方向和比例正确。

### Gate 4：生成候选标定，不覆盖原文件

先备份：

```bash
cd /home/ycb410/nt/R1_arm_SDK_x86/r1_arm
cp -a config/joint_calib.txt config/joint_calib.before_real_teleop.txt
```

把真机人工摆到仿真 `ARM_INIT=[0,1.2,0,-1.2,0,0,0]` 对应的实体姿态。
SDK 坐标的参考值是 `[0,1.2,0,1.2,0,0,0]`。托稳机械臂后执行：

```bash
cd /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/bin

./calibrate_joints \
  --port /dev/ttyUSB0 \
  --baud 6000000 \
  --rate 50 \
  --samples 100 \
  --load ../config/joint_calib.txt \
  --reference-q 0,1.2,0,1.2,0,0,0 \
  --out ../config/joint_calib_candidate.txt
```

不要在姿态未核对或总线还有 timeout 时执行这一步。

### Gate 5：只读验证候选标定

```bash
./test_arm_lowlevel \
  --zero-torque \
  --port /dev/ttyUSB0 \
  --baud 6000000 \
  --rate 50 \
  --calib ../config/joint_calib_candidate.txt
```

参考姿态下应满足：

```text
q_sdk ≈ [0, 1.2, 0, 1.2, 0, 0, 0]
```

每轴建议误差小于 `0.03 rad`。然后再次逐轴小幅人工移动，确认 ID、方向、变化量和限位；
把反馈换算到仿真坐标后，J4 应取反，所有关节必须落在仿真 URDF 限位内。

### Gate 6：低增益原位保持

仍由人托住机械臂，急停就位：

```bash
./test_arm_lowlevel \
  --foc \
  --port /dev/ttyUSB0 \
  --baud 6000000 \
  --rate 50 \
  --kp 10 \
  --kd 0.3 \
  --calib ../config/joint_calib_candidate.txt
```

该命令以实测姿态作为目标，正常情况不应出现启动跳变。若有明显跳动、方向相反、持续振荡、
任一 timeout 或错误码，立即急停并回到 Gate 3～5。

### Gate 7：PICO dry-run

```bash
cd /home/ycb410/nt/r1arm_teleop
conda activate r1_teleop

python scripts/teleop_r1arm_real.py \
  --port /dev/ttyUSB0 \
  --scale-factor 0.2 \
  --debug
```

不带 `--motion-mode` 时不会打开串口或发送电机命令。检查 grip 激活前目标不变；激活后手柄小幅
移动时，IK 目标连续、无 NaN、无突然跳到关节限位。

### Gate 8：首次实机遥操

首次不接 Dex1、不启用 A 键复位，并采用低增益、低速度、小超前量：

```bash
python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port /dev/ttyUSB0 \
  --baud 6000000 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --scale-factor 0.2 \
  --kp 10 \
  --kd 0.3 \
  --max-joint-speed 0.10 \
  --max-command-lead 0.03 \
  --debug
```

启动后先不按 grip，观察原位保持 30 秒；再短按 grip，只做 1～2 cm 的单方向手柄移动。
依次验证基座 X/Y/Z 平移和三个转动方向。全部正确后再逐级提高 `scale-factor`、速度和增益，
每次只改一个参数。Dex1 和数据采集放到机械臂遥操连续稳定之后再接入。
