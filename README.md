# R1Arm 遥操、录制与回放

当前只维护 R1Arm 基线：右臂单臂、双臂、MuJoCo 仿真，以及 MCAP 录制和回放。
真机启动前支撑机械臂、确认急停和运动空间；输入 `MOVE` 后才使能运动。夹爪服务和 XR PC Service 分别运行。

## 1. 从夹爪服务开始

本机每个新终端执行一次：

```bash
source /home/unitree/miniconda3/etc/profile.d/conda.sh
conda activate r1_teleop
cd /home/unitree/nt/R1ARM/r1arm_teleop
source scripts/real_env.sh
```

`real_env.sh` 配置串口、左右标定、bridge 和本机 DDS 网卡 `lo`，只检查路径／权限，不打开电机。
以下选择一个夹爪启动命令，保持这个终端运行：

```bash
# 单臂：只启动右夹爪，ID7/right
bash scripts/start_gripper.sh

# 双臂：启动左右夹爪，两侧均为 ID7，DDS side 分开
bash dual/start_grippers.sh
```

不要同时运行上述两个命令。切换模式时先 Ctrl+C 停止之前的服务。
单臂需看到 right 的 `Detected motors` 和一次 `Dex1-1 Gripper Server started.`；双臂需看到 left/right 各一次。
只出现 `Checking Dex1 port` 或 `Official Dex1` 不能认为电机已识别。
另一终端再次执行本节开头的环境命令，然后进行遥操／录制／回放。
录制目录由命令参数直接指定，不需要额外创建 `STAMP`、`OUT`、`LOG` 或使用 `tee`。

## 2. XR 服务与网络

真机遥操／录制前，启动 PC Service，保持 PICO 遥操应用在前台、头显唤醒，并移动手柄确认数据更新。
本机头显安装包为 `XRoboToolkit-PICO-1.1.1.apk`；PC Service 是桌面服务，Python SDK 客户端连接本机 `127.0.0.1:60061`。
机械臂串口和夹爪 DDS 不依赖头显 Wi-Fi；回放使用记录中的关节数据，不需要实时 XR。

- **头显 → PC Service**：头显和电脑应网络互通，优先电脑有线接路由器、头显接稳定的 5 GHz Wi-Fi。避免访客网络／AP 客户端隔离；必要时检查路由、防火墙和 VPN 是否影响局域网连接。可用 `ip -br addr` 查看地址、`ip route` 查看路由；不要仅因安装了代理就认定它是间断原因。
- **Python → PC Service**：`Connected to XRoboToolkit` 表示本机 SDK 连接成功，不能单独证明手柄数据持续更新。头显休眠、应用退到后台或发送暂停，仍可能看到客户端已连接。
- **Python → 夹爪服务**：同机 DDS 默认 `DEX1_NETWORK=lo`、domain 0。`lo is not multicast-capable: disabling multicast` 在本机配置中是已知提示，不能据此认定夹爪故障，也不应把这里改成头显 Wi-Fi 网卡。
- 运行一个 PC Service，退出旧 XR 采样器／控制器后再启动新遥操；避免多个版本进程混用。替换 binding／SDK 后需退出旧 Python 进程重启；替换服务库后需重启 PC Service。
- 单臂右侧运动只依赖右手柄；默认 **X 录制在左手柄**，所以单臂录制仍需左手柄配对、唤醒。头显应用若启用了 A 键暂停发送功能，会与机械臂 A 回 home 冲突，需要关闭该发送快捷键或重新配置。

本机换网后 XR 间断曾明显减少；这是实测现象，不能据此断言所有间断都来自 Wi-Fi。
当前 XR 工作线程只取最新快照，不增加滤波／插值预测；短重复维持目标，超过 250 ms 停更后停止／保持，恢复稳定后重新锚定，不追补断流期间手柄位移。

## 3. 右臂：遥操 → 录制 → 回放

按第 1 节启动右夹爪，确认 XR 正常；控制终端选择遥操或录制之一：

```bash
# 仅遥操
python scripts/teleop_r1arm_real.py --motion-mode

# 遥操并录制，数据写入指定目录
python scripts/record_r1arm_real.py --motion-mode \
  --output-dir data/right_real --require-all-cameras
```

默认启动后回 home；右 grip 跟踪机械臂，右 trigger 控制夹爪，A 回 home。
命令不覆盖录制按键。单／双臂录制都沿用默认按键：**X 开始／停止并保存，B 暂停／恢复记录和跟踪，A 回 home**。
X 在左手柄，B/A 在右手柄；“单臂”指只驱动右臂，不是只需要一个手柄完成默认录制操作。
暂停不记录，恢复后以新的 `record_segment` 继续，训练或回放时不可跨段直接拼接动作。
保存后等待完成提示，再用 Ctrl+C 退出。目录中自动生成 `episode_XXXX.mcap`，不需要每轮手动创建时间戳目录。
单臂录制的开始、保存和暂停事件由现有录制线程处理，按键后以 `START`／`SAVED` 提示确认；恢复跟踪时由控制线程清理旧锚点。
可选 `--image-show` 显示录制预览；`--no-cameras` 只录状态／动作，此时去掉 `--require-all-cameras`。

先只读检查回放：

```bash
python scripts/replay_r1arm_real.py --episode data/right_real/episode_0000.mcap
```

确认后真机回放：

```bash
python scripts/replay_r1arm_real.py \
  --episode data/right_real/episode_0000.mcap --motion-mode
```

回放默认优先使用高频控制流的整形目标 `q_purpose`，不重复整形。
`--section state` 播放实测位置；`--speed 0.5` 慢放；`--data-source frame` 显式选择观测流。
图像默认在独立预览进程中按 MCAP 时间同步显示，不占用电机控制线程；无图像的 episode 会跳过预览。加 `--no-show-observations` 可关闭窗口，预览按 `q`／Esc 会停止回放。
不加 `--motion-mode` 的单臂／双臂真机 replay 只做只读数据检查，不播放时间轴；要看图且不驱动真机，使用下面的仿真 replay。

## 4. 双臂：遥操 → 录制 → 回放

按第 1 节启动双夹爪，两侧均成功后，在控制终端选择遥操或录制之一：

```bash
# 仅遥操；不按 X 时不保存 episode
python dual/teleop.py --motion-mode

# 遥操并录制，三路图像默认启用
python dual/record.py --motion-mode --data-root data/dual_real
```

输入 MOVE 后两臂回 home。左右 grip/trigger 分别控制各自机械臂／夹爪。
启动时先确认双侧夹爪反馈，再使能机械臂；先就绪的机械臂会持续保持实测姿态，等待另一侧就绪后才开始归位控制。
X 开始／保存 episode，B 暂停／恢复记录和跟踪，A 两臂回 home；暂停后以新 `record_segment` 继续。
`--no-cameras` 只录状态／动作。双臂默认要求所有指定相机都可用。

先只读检查：

```bash
python dual/replay.py --episode data/dual_real/episode_0000.mcap
```

确认后真机回放：

```bash
python dual/replay.py \
  --episode data/dual_real/episode_0000.mcap --motion-mode
```

默认 `--data-source auto` 优先高频控制流，旧数据没有控制流时回退观测流。
支持 `--section action/state`、`--speed`；跨暂停分段先保持、再分别接近新段首帧，不跨段直接插值。
回放前检查峰值速度；若要求降低速度，使用 `--speed 0.5`，不要放宽速度限值。
双臂并行执行两个串口事务；`pair_skew_ns` 表示完成时间差，不代表硬件同步。
图像默认在独立预览进程中按 MCAP 时间同步显示；加 `--no-show-observations` 可关闭窗口。预览按 `q`／Esc 会停止回放。

## 5. 图像配置

| 图像键 | 相机 | 启用模式 | 请求配置 |
|---|---|---|---|
| `color_0_head_left` | D405 / RealSense | 单臂、双臂 | 640×480，30 fps，手动曝光 8000 µs，gain 16 |
| `color_2_wrist_left` | JR0001 / V4L2 | 双臂 | 640×480，30 fps，MJPG |
| `color_3_wrist_right` | JR0002 / V4L2 | 单臂、双臂 | 640×480，30 fps，MJPG |

默认值统一在 `teleop/local_camera_client.py`。腕部使用两个 V4L2 缓冲，已在本机三路并行验证约 30 Hz。
头部曝光按当前现场照明设置，环境变化后需要重新查看画面。相机编号不用于串口设备编号。

```bash
# 不启用机器人，分别验证单臂两路／双臂三路
python scripts/verify_r1arm_right_cameras.py
python scripts/verify_r1arm_dual_cameras.py

# 检查录制图像，并只播放图像，不发送机械臂命令
python scripts/check_r1arm_images.py data/dual_real/episode_0000.mcap --preview
```

预览按 q 退出；相机验证程序和录制程序依次运行，避免争用设备。
自定义相机可重复传 `--camera 'key=...,backend=...,match=...,size=640x480,fps=30'`，覆盖整套默认配置。
支持 `auto_exposure=true/false`、`exposure`、`gain`；RealSense 和 V4L2 曝光单位不同，不能互相照抄。

## 6. 仿真

仿真不启用真实电机。省略 `--mock-xr` 时使用真实 PICO 输入：

```bash
python scripts/teleop_r1arm_sim.py --mock-xr
python scripts/record_r1arm_sim.py --mock-xr --data-root data --task-dir-name sim_right
python scripts/replay_r1arm_sim.py --episode data/sim_right/episode_0000.mcap
```

仿真双臂在遥操／录制命令中增加 `--use-dual-arm`。仿真入口默认控制 50 Hz、每拍 10 个仿真子步、reset-blend 100 帧；仿真录制默认观测 30 Hz、渲染 320×240，需 640×480 时加 `--image-width 640 --image-height 480`，不能把真机相机设置误认为仿真渲染默认。
用 `replay_r1arm_sim.py` 查看单臂或双臂 MCAP 时，图像默认按录制时间显示；加 `--no-show-observations` 关闭图像窗口。`--no-viewer` 只关闭 MuJoCo 机器人窗口，不关闭图像预览。
各入口完整参数使用 `--help` 查看；仿真输出目录与真机录制格式兼容，但设备反馈含义不同。

## 7. 基线参数及数据含义

以下数值来自当前入口和 `R1ArmConfig`；表中为**真机遥操／录制**的实际默认值，不将配置类或仿真入口的其他默认值混进来。只需改动时才在命令中显式传参。

| 参数／设置 | 默认值 | 作用 |
|---|---|---|
| `--rate` | 90 Hz | 控制／IK 主循环目标频率；实际频率由串口和运算决定 |
| `--frequency` | 30 Hz | MCAP 图像／观测采样频率，不降低高频控制记录 |
| `--scale-factor` | 1.0 | 手柄平移到末端目标平移的比例 |
| IK | Placo 位姿 QP，失败回退 DLS | 求末端目标对应的原始关节目标；QP 未启用实验动态限幅 |
| `--ik-dt` | 0，使用 `1/rate` | 求解器时间步；不是额外的输入滤波参数 |
| `--trajectory-position-gain` | 64 s⁻² | 整形器对位置误差的响应强度 |
| `--trajectory-velocity-gain` | 16 s⁻¹ | 整形器速度阻尼 |
| `--trajectory-max-speed` | 未指定，跟随 `--max-joint-speed`，即 1 rad/s | 内部整形轨迹速度上限 |
| 整形加速度／jerk 设置 | 6 rad/s²／80 rad/s³ | 抑制内部轨迹速度和加速度突变；双臂加速度可用 `--max-joint-acceleration` 修改 |
| `--max-joint-speed` | 1 rad/s | 最终 SDK 指令位置步长限速 |
| `--max-command-lead` | 0.15 rad | 最终指令相对实测位置的目标超前限制；与步长约束冲突时优先步长，不是总力矩上限 |
| `--kp`／`--kd` | 60／10 | 电机位置反馈增益／速度阻尼，与整形器 64/16 不同；支持七轴分别指定 |
| `--gravity-ff` | 开 | 在指令姿态计算重力前馈 |
| `--tau-clip` | 8 Nm | 逐关节重力前馈限幅，不限制电机全部输出力矩 |
| `--baud` | 6000000 | 机械臂串口波特率；夹爪服务也使用 6000000 |
| `--feedback-timeout` | 3 s | 启动时等待七轴完整反馈的最长时间 |
| `--foc-timeout` | 1 s | 启动时等待电机进入 FOC 的最长时间 |
| `--home-on-start`／`--enable-a-reset` | 均开 | 启动回 home／允许 A 回 home |
| home | `[0,1.2,0,-1.2,0,0,0]` rad，共同模型坐标 | 归位目标，不要求启动前人工处于零位 |
| `--reset-blend-frames` | 60 | 名义归位最短时间按 frames/50 Hz 计算，约 1.2 s；实际还需满足限速和反馈稳定 |
| 双臂 `--home-recovery-margin` | 0.25 rad | 启动 home 时允许的有限初始超限恢复，不是允许错误标定 |
| `--enable-dex1` | 开 | 启用真实夹爪 DDS 客户端 |
| `--dex1-net`／`--dex1-domain` | source 环境后 `lo`／0 | 同机 DDS 通信；与头显网络分开 |
| 夹爪服务 ID | 左右各为 7 | 两条独立串口上的相同 ID；DDS topic 根据 side 区分 |
| `--grip-torque-limit` | -1 | 关闭额外的夹持力矩锁定功能；不是给电机下发负力矩目标 |
| 夹爪客户端 kp／kd | 5／0.05 | 真实夹爪位置控制增益；与机械臂增益独立 |
| 夹爪端点 | `robot/eef/dex1_endpoints.json` | 使用逐侧保存的电机端点，不共用 0～5 rad |
| 图像 | 640×480、30 fps；腕部 V4L2 双缓冲 | 两路／三路按模式选择；头部曝光 8000 µs、gain 16 |
| 单臂 `--camera-wait`／`--max-image-age` | 8 s／0.5 s | 启动等图时间／允许图像帧龄；双臂启动等图 5 s，最大帧龄 0.5 s |
| 图像窗口 | 默认关闭 | 单臂录制 `--image-show` 开预览，预览默认 12 Hz，不改变图像采集／保存的 30 Hz；双臂录制没有此窗口选项 |
| `--record-button`／`--pause-button` | X／B | 单臂录制默认开始保存／暂停恢复；双臂固定 X/B |
| 默认输出目录 | 单臂 `dataset/teleop_kp64_kd16_control90_baseline`；双臂 `data/dual_real` | 单臂用 `--output-dir`、双臂用 `--data-root` 指定目录，示例明确改到 `data/right_real`／`data/dual_real` |
| `--timing-report` | 双臂默认关 | 开启后打印后台控制／准备／通信耗时统计，不改变控制目标频率 |
| XR 工作线程轮询 | 目标 180 Hz | 读取原子最新快照，不代表头显发送 180 Hz |
| XR 停更／恢复 | 250 ms／至少 3 个不同新帧且跨度 ≥80 ms | 过期停止跟踪；稳定恢复后一次性重建锚点 |
| grip 迟滞 | 按下 ≥0.90，释放 ≤0.80 | 避免阈值附近反复启停 |
| XR 跳变速度参考 | 平移 3 m/s，旋转 12 rad/s | 与帧间隔共同形成逐设备跳变阈值；位置阈值上限 0.30 m，旋转上限 1 rad |
| XR 滤波／预测 | 均关闭 | 保留有效性、时效和跳变保护，平滑由原控制整形器承担 |

内部整形使用 `a_desired = 64 × 位置误差 − 16 × 速度`，再做速度／加速度／jerk 处理。
速度边界冲突时可能放宽 jerk，硬限位处理和最终指令限幅也会改变导数，因此 **80 rad/s³ 不是最终电机轨迹的严格 jerk 保证**。
运行期间检查显式反馈 timeout、非有限值、故障码；反馈计数器／缓存年龄保留为诊断，不因计数停更单独触发停机。

回放参数与遥操有意区分：

| 参数 | 单臂回放默认 | 双臂回放默认 | 作用 |
|---|---|---|---|
| `--data-source`／`--section` | auto／action | auto／action | 优先高频控制流，播放整形目标；state 改为实测轨迹 |
| `--speed` | 1.0 | 1.0 | 回放时间倍率 |
| 刷新率 | `--rate 30` 用于接近／保持；播放默认跟随记录时间，`--control-hz 0` 不额外重采样 | `--rate 90` | 不是所有回放阶段都由同一个 rate 参数控制 |
| `--approach-speed` | 1 rad/s | 0.3 rad/s | 接近段首帧／结束姿态的速度上限 |
| 接近超时 | 10 s | 30 s | 未稳定到位则退出 |
| 记录长间隔上限 | `--max-frame-gap 2` s | `--max-gap 0.5` s | 回放长暂停时的压缩上限；分段首帧仍单独接近 |
| `--hold-time` | 2 s | 2 s | 结束后保持时间 |
| `--show-observations` | 开 | 开 | 独立进程按记录时间显示图像；`--no-show-observations` 关闭预览 |

- 单臂向量为右臂七轴＋夹爪，共 8 维；双臂为右臂七轴＋右夹爪＋左臂七轴＋左夹爪，共 16 维。
- `/r1arm/control_sample` 保存完整控制快照，`/whole_body/frame` 保存图像和观测；state 是反馈，action 是整形目标，最终电机指令另存。
- 相机时间戳是接收时间，不保证等于曝光时刻；双臂录制选择控制时间之前最近的历史图像，缺图／过期会统计，控制流仍保留。
- 回放先接近首帧、最后保持；检查反馈 timeout／故障和限位。CSV 诊断自动生成，双臂日志后台有界写入。

## 8. 串口、启动排查与标定

FTDI 稳定路径前缀：`/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBPG771-`。

| 设备 | 后缀 | 标定／ID |
|---|---|---|
| 右臂 | if01-port0 | `joint_calib_zero_右臂.txt` |
| 左臂 | if03-port0 | `joint_calib_zero_左臂.txt` |
| 右夹爪 | if00-port0 | ID7，DDS right |
| 左夹爪 | if02-port0 | ID7，DDS left |

`real_env.sh` 提供 `PORT_RIGHT`、`PORT_LEFT`、`CALIB_RIGHT`、`CALIB_LEFT`。
需要覆盖设备时使用 CLI：单臂 `--port/--calib`，双臂 `--right-port/--left-port/--right-calib/--left-calib`。
夹爪同机 DDS 默认 `lo`，可通过 `DEX1_NETWORK` 或 `--network` 修改。
左右 arm 模型方向不同，不能交换标定文件或在零点表中重复加入模型反号；详见 [dual/README.md](dual/README.md)。

连接问题先按下面顺序排查，避免把通信问题当作标定问题：

| 现象 | 检查和处理 |
|---|---|
| `Accessible` | 只表示设备路径存在且当前用户可读写，不代表电机通电或能回复 |
| 夹爪 `motor id=7 does not reply` | 核对对应侧供电、线缆、稳定路径和 ID7；先退出控制器，再单侧重启服务排查。网卡配置不会造成串口 ID 探测失败 |
| 夹爪只有一侧启动后又都退出 | 双侧启动器发现任一服务失败会清理自己启动的另一侧；先看最先失败的那一侧 |
| `Port is already used` | 退出原服务，避免两个进程争用同一串口；启动器保留检查，不自动杀用户进程 |
| 启动器曾卡在 fuser | 现已用 `/proc` 文件描述符检查替代；不再用可能被异常挂载拖住的 fuser 做预检 |
| 七轴反馈不全／`missing joints` | 检查机械臂电源、线缆、if01/if03 映射和端口占用；可靠支撑机械臂。不要盲目改 ID、放宽反馈检查或重新标定 |
| `No Dex1 feedback ...state` | 保持对应服务运行，核对 left/right topic、同一个 DDS domain 和网卡；串口识别成功不代表 DDS 客户端收到反馈 |
| 反馈稳定、按 trigger 无动作 | 检查 trigger 值和夹爪命令／反馈是否变化，确认 home 后开爪流程完成；不能因服务 started 就认为动作链路验证通过 |
| 重插 USB 后编号变化 | 使用 FTDI by-id 路径，避免依赖 `/dev/ttyUSB0` 等易变编号；不要把夹爪口用于机械臂 |
| 权限不足 | 将用户加入 dialout 后重新登录；不要靠启动多个 sudo 控制进程绕过权限 |
| `pinocchio`／其他模块缺失 | 检查当前是 r1_teleop 环境，不能使用 base 运行真机入口 |
| `libmotor_controller.so`／bridge 缺失 | source `real_env.sh`，核对 SDK 和 bridge 路径；迁移机器后重新构建，不复用 CMake 缓存 |
| Qt wayland 插件错误 | 当前 OpenCV 提供 xcb；在桌面会话使用 xcb。字体目录告警与相机曝光是两件事 |
| 相机忙／无帧 | 先退出相机验证程序，再启动录制；三路尺寸／采集率用 verify 脚本确认 |

串口占用检查只能看到当前进程有权限读取的描述符；双侧启动器还使用设备锁防止自身重复启动。
普通断电不自动要求重新标定。只有零位对应关系确实改变时，才重新标定；错误标定会使反馈角度与模型限位不符。

重新标定需要可靠支撑并人工摆到零位，普通断电不意味着必须标定：

```bash
python dual/calibrate.py --side right --capture --install
python dual/calibrate.py --side left --capture --install
```

夹爪端点在 `robot/eef/dex1_endpoints.json`。只有需要重新采集端点时，保留夹爪服务、退出所有命令发布器后运行：

```bash
python scripts/capture_dex1_endpoints.py --network lo
```

## 9. Submodule、官方来源与安装

当前 `.gitmodules` **实际指定了 6 个 submodule**，当前固定提交如下：

| 本地路径 | 当前仓库来源 | 固定提交（缩写） | 用途 |
|---|---|---|---|
| `third_party/r1_arm_sdk` | [unitreerobotics/r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk) | `2c89fe4` | 官方机械臂 SDK，叠加本地总线／串口基线补丁 |
| `third_party/unitree_sdk2` | [unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | `f29ee9f` | 夹爪服务的 C++ DDS 依赖 |
| `third_party/unitree_sdk2_python` | [unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | `9c51902` | Python 夹爪 DDS 客户端 |
| `third_party/XRoboToolkit-PC-Service-Pybind` | [YanjieZe/XRoboToolkit-PC-Service-Pybind](https://github.com/YanjieZe/XRoboToolkit-PC-Service-Pybind) | `75cb113` | 当前绑定来源，叠加本地原子快照／计时扩展；不是官方新版绑定 |
| `third_party/XRoboToolkit-PC-Service` | [XR-Robotics/XRoboToolkit-PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) | `85bac4d` | XR 原生 SDK／服务源码，叠加本地最新帧合并修改 |
| `third_party/dex1_1_service` | [unitreerobotics/dex1_1_service](https://github.com/unitreerobotics/dex1_1_service) | `ced3216` | 官方夹爪服务，叠加串口选择、ID7 与 side 解耦等基线补丁 |

### 机械臂 SDK 官方上游已确认

官方仓库是 [unitreerobotics/r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk)，不是“没有官方上游”。
2026-10-26 核对官方 main 为 `2c89fe4f60f0c98f1bc06795cb2477cc73d0c92c`，有 `include/arm_motor_bus.h`、`src/arm_motor_bus.cpp`、标定／低层控制示例和电机库源码。
官方原版缺少当前基线的 `stepJoint`、`feedbackCounts`、`missedResponseCounts`、`lastStepFeedback` 等扩展。
现在机械臂 submodule 已切换为官方 URL 和官方提交；Dex1 service 也已从普通目录转换为官方 submodule。
转换保留了原有源码和运行库内容，本地修改作为工作树差异继续存在，没有用官方原版覆盖已验证的设备适配。

对应源码补丁在 `native/dependency_patches/r1_arm_sdk.patch` 和 `native/dependency_patches/dex1_1_service.patch`。
`setup.sh` 初始化这两个 submodule 后、编译前会幂等应用补丁；如果补丁已应用就跳过，如果与用户改动冲突则停止，不强制覆盖。
补丁已在官方提交的干净副本中检查可应用，也检查了当前本地源码符合补丁后的状态。
补丁不携带本机标定文件或预编译运行库；本机实际运行的外部 SDK／bridge 路径仍见下节。

### 固定来源与本机实际运行路径

本机 `source scripts/real_env.sh` 后，默认使用相邻 `R1_arm_SDK_x86/r1_arm` 的 SDK 和左右标定，而不是自动改用 `third_party/r1_arm_sdk`。
`R1ARM_SDK_ROOT` 可指定外部 SDK；`R1ARM_BRIDGE_LIB`／`--bridge-lib` 指定实际 bridge；环境脚本优先选择本机已验证的 `build/dual_sdk_bridge`，否则使用标准 `build/r1arm_sdk_bridge`。
机械臂、Dex1、XR 绑定和 PC Service 的本地源码适配均保存在 `native/dependency_patches/`，安装时在各依赖编译前应用。XR 对应 `xr_pybind.patch`（原子快照／解析计时）和 `xr_pc_service.patch`（最新帧合并）；只拉取子模块的上游提交、不应用补丁，不等于当前基线。设备标定和已生成运行库需另行保管。

```bash
# 查看子模块固定版本与工作树状态，只读
git submodule status
git submodule foreach 'git status --short'
```

### 新机器安装

带 submodule 初始化仓库，然后安装：

```bash
git submodule update --init --recursive
bash setup.sh
```

`bash setup.sh --skip-xrt` 可先安装仿真所需部分。setup 安装依赖并编译 SDK／bridge／夹爪服务，不会启用机器人，也不会部署完整 PC Service；头显应用与 PC Service 按上游说明单独安装。
setup 默认编译仓库内 `third_party/r1_arm_sdk`；本机 source 环境后的外部 SDK 覆盖需要与标定路径、bridge 构建来源保持一致。
以下仅在 SDK 源码或机器改变、需要重建时执行：

```bash
python scripts/build_r1arm_sdk_bridge.py --sdk-root "$ARM"
```

SDK 单独的事件驱动串口补丁保留在 `native/r1arm_sdk_bridge/serial_event_poll.patch`；安装使用的完整 SDK 基线补丁已包含该适配，不要重复应用两份补丁。更换上游固定版本后要重新核对补丁。
第三方源码及许可证保留在 `third_party/`。不要在现有有本地修改的依赖上执行强制 checkout／reset 来“更新版本”。

## 10. 目录

`robot/` 设备和模型；`teleop/` 共享控制、XR、相机和 MCAP；`scripts/` 单臂及仿真入口；`dual/` 双臂入口；`tests/` 和 `dual/tests/` 基线离线回归；`assets/` 模型；`native/` bridge；`third_party/` 依赖。
`data/`、`dataset/` 为已有录制，`build/` 含本机运行库。清理代码不删除数据和设备标定。

```bash
python -m unittest discover -s tests
python -m unittest dual.tests.test_dual
```
