python scripts/replay_r1arm_real.py \
  --episode /home/ycb410/nt/r1arm_teleop/dataset/rate_diag_fixed/episode_0000.mcap \
  --section action \
  --start-frame 0 --end-frame -1 \
  --reshape-raw-ik \
  --trajectory-position-gain 28 \
  --trajectory-velocity-gain 10.6 \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --rate 30 --speed 1.0 \
  --max-frame-gap 0.08 \
  --max-joint-speed 1.0 --max-command-lead 0.15 \
  --kp 150 --kd 3 \
  --telemetry-csv /home/ycb410/nt/r1arm_teleop/dataset/rate_diag_fixed/episode_0000_replay_shape_kp28_kd10_6_sdk_kp150_kd3.csv




# R1 系列机械臂遥操作与数据采集框架

基于 Unitree R1 机械臂的 XR 遥操作框架：用 PICO VR 手柄遥操机械臂，并把
「图像 + 关节状态 + 控制命令」录成 MCAP 数据集，用于后续训练与回放。

本仓库覆盖两个硬件平台：

| 平台 | 说明 | 仿真后端 | 真机通道 | 主线状态 |
|------|------|----------|----------|----------|
| **R1Arm** | 独立单臂，7-DoF + 可选 Dex1 夹爪 | MuJoCo | RS485 串口（Unitree SDK） | 当前主线 |
| **R1-A7** | 双臂人形机器人（+ 腰/头） | Meshcat | DDS（`rt/lowcmd` + Dex1） | 兼容保留 |

> 两者共用同一套 PICO 相对位姿跟踪 + IK 逻辑，只有「输出后端」不同。
> 下文默认先讲 R1Arm（主线），R1-A7 在对应小节单独说明。

---

## 0. 真机三步走：遥操 / 录制 / 回放（复制即用）

> **前置**：终端已 `conda activate r1_teleop`；机械臂上电、串口 `/dev/ttyUSB2` 在线；



> Dex1 服务已启动（另开终端：`source scripts/real_env.sh && ./scripts/start_dex1_local.sh`，需 sudo）。
> 每条命令都会在 `Type MOVE to continue:` 处等你输入 `MOVE` 才真的运动。

cd /home/ycb410/nt/r1arm_teleop
conda activate r1_teleop


终端 A（Dex1 夹爪服务，需要 sudo，保持运行）：

    source /home/ycb410/nt/r1arm_teleop/scripts/real_env.sh
   ./scripts/start_dex1_local.sh

### 0.1 遥操（手感基线）

#### 当前手感测试基线

当前恢复到本轮 XR 滤波调参前使用的 cutoff=2.0、beta=0.15；轨迹器 position/velocity gain 保持 16/8。此次只恢复 XR 滤波基线，其他速度、整形器及电机参数不变：

```bash
# 50 Hz
python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --rate 50 \
  --xr-filter-min-cutoff 2.0 --xr-filter-beta 0.15 \
  --trajectory-position-gain 16 --trajectory-velocity-gain 8 \
  --kp 60 --kd 10 \
  --max-joint-speed 1.0 --max-command-lead 0.15 \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start --enable-a-reset
```

启动真机前托住机械臂并确认急停可触及。若想判断增益效果，先固定 XR 参数，只改 position/velocity gain；若想判断滤波效果，则固定整形器和电机参数，只改 cutoff/beta。

```bash
python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start \
  --enable-a-reset
```
teleop_r1arm_real_qp_jerk.py
| 参数 | 作用 |
|------|------|
| `--enable-dex1 --dex1-net enp4s0` | 夹爪 DDS 通道（不传则不控夹爪） |
| `--home-on-start` | 启动 FOC 后软 blend 回 `ARM_INIT`（约 1.2 s） |
| `--enable-a-reset` | 允许 **A 键**随时软回 home |

**PICO 操作**：握 **grip** 开始跟踪 · 扣 **trigger** 闭合夹爪 · **A** 软回 home · `Ctrl+C` 停止。

实验对照版（只绕过 active IK 后的 q/v/a 整形；Placo/DLS 内按 1 rad/s 限速，SDK 限幅仍保留。home/reset 与松 grip 刹停仍走安全轨迹；不做加速度/jerk 限制）：

```bash
python scripts/teleop_r1arm_real_unshaped.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --rate 30 \
  --xr-filter-min-cutoff 2.0 --xr-filter-beta 0.15 \
  --kp 80 --kd 2 --max-joint-speed 1.0 --max-command-lead 0.15 \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start --enable-a-reset
```

这是用于 A/B 手感验证的实验入口，不保证与整形版同样平滑；先托住机械臂并短时低幅度测试。

QP 约束对照版（主动跟踪时在 Placo QP 内约束速度 ≤1 rad/s、加速度 ≤6 rad/s²；不加 jerk，作为对照）：

```bash
python scripts/teleop_r1arm_real_qp.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --rate 30 \
  --xr-filter-min-cutoff 2.0 --xr-filter-beta 0.15 \
  --kp 80 --kd 2 --max-joint-speed 1.0 --max-command-lead 0.15 \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start --enable-a-reset
```

录制同一实验版本时，用 `scripts/record_r1arm_real_qp.py` 替换 `record_r1arm_real.py`；episode metadata 会标记 `ik_control_mode`，并继续记录 `q_purpose`、`q_motor_target`、速度/加速度及实测 state。

同一组参数下启用 jerk 约束（初始值 80 rad/s³），XR 滤波和其它参数保持不变。与上面的无 jerk 入口做 A/B；reset/home 和松 grip 刹停仍沿用基线轨迹。录制时用对应的 `record_r1arm_real_qp_jerk.py`，metadata 会标记 jerk 上限：

```bash
python scripts/teleop_r1arm_real_qp_jerk.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --rate 30 \
  --xr-filter-min-cutoff 2.0 --xr-filter-beta 0.15 \
  --kp 80 --kd 2 --max-joint-speed 1.0 --max-command-lead 0.15 \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start --enable-a-reset
```

### 0.2 录制（产出 MCAP 数据集）

```bash
python scripts/record_r1arm_real.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start \
  --enable-a-reset \
  --task-description "pick and place demo" \
  --task-dir /home/ycb410/nt/r1arm_teleop/dataset/try
```

**PICO 操作**：**X** 开始/停止并保存 episode · **B** 暂停/恢复（录制与 XR 跟踪一起停）· **A** 回 home（归位动作也录进 episode）。

保存后终端打印**质检表**：action 平滑度（逐关节 a_p95）、夹爪通道是否有变化、相机是否丢失/冻结、实际 fps、暂停断点位置、图像对齐偏移。

### 0.3 回放（同一条 episode 重演到真机）

```bash
python scripts/replay_r1arm_real.py \
  --episode /home/ycb410/nt/r1arm_teleop/dataset/2026_10_09_robot_r1arm_right_dex1/episode_0011.mcap \
  --section action \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0
```

| 参数 | 作用 |
|------|------|
| `--section action` | 默认重放已整形的 `actions.q_purpose`，不再经过回放轨迹整形器；SDK 安全限幅仍生效。`--section state` 重放实测位姿 |
| `--episode` 给目录 | 自动取该目录下最新的 episode |

重力前馈和 action 回放的 SDK 限速上限从 episode 元数据继承；回放不再对 `q_purpose` 做第二轮 v/a 整形。流程：接近段（当前姿态 → 第一帧，限速 1 rad/s）→ 按录制节奏播放 → 收敛到末帧 → 保持 2 s → 停机。每轮真机回放会在 episode 旁生成唯一 CSV，记录 q_purpose、最终安全命令、dq_target、实测 q/dq/力矩和跟踪误差；可用 `--telemetry-csv` 指定路径。

### 已生效的默认值（无需再传）

| 默认 | 值 | 说明 |
|------|-----|------|
| `--rate` | 100 Hz（真机默认） | 控制环 / IK / 电机命令；XR 输入刷新仍约 30 Hz |
| `--frequency` | 30 Hz | 图像与 observation 帧采样率；真机另存完整控制频率流 |
| `--kp` `--kd` | 80 / 2 | 可逐关节：`--kp 80,80,80,60,80,80,80` |
| `--gravity-ff` | **开** | 重力前馈（`--no-gravity-ff` 关闭） |
| `--max-joint-speed` | 1.0 rad/s | 命令限速 |
| `--max-command-lead` | 0.15 rad | 力矩三项合上界 22 Nm = 电机峰值 64 % |
| `--trajectory-position-gain` / `--trajectory-velocity-gain` | 16 / 8 | `q_purpose` 轨迹器响应与阻尼 |
| `--xr-filter-min-cutoff` / `--xr-filter-beta` | 2 / 0.15 | XR One-Euro 滤波；高速动作会自适应提高 cutoff |
| `--reset-blend-frames` | 60 | A/home 归位时长 ≈ 1.2 s |
| `--preview-hz` | 12 | 预览渲染（独立线程，不占控制环） |
| 相机 | 头 SN0002 + 腕 D405 | 各 640×480@30；`--no-image-show` 关预览 |

---

## 1. 一图看懂：我要做什么 → 跑哪个脚本

| 想做什么 | 平台 | 入口 |
|----------|------|------|
| 仿真遥操（无 PICO，自动轨迹） | R1Arm | `scripts/teleop_r1arm_sim.py --mock-xr` |
| 仿真遥操（PICO 手柄） | R1Arm | `scripts/teleop_r1arm_sim.py` |
| 仿真遥操（双臂场景） | R1Arm | `scripts/teleop_r1arm_sim.py --use-dual-arm` |
| 仿真遥操 | R1-A7 | `scripts/teleop_sim.py --mock-xr` |
| 真机遥操 | R1Arm | `scripts/teleop_r1arm_real.py --motion-mode --port $PORT` |
| 真机遥操 | R1-A7 | `scripts/teleop_real.py --motion-mode --enable-dex1 --net <网卡>` |
| 仿真数据采集（MCAP） | R1Arm | `scripts/record_r1arm_sim.py --output-dir <目录>` |
| 真机数据采集（两路本机相机 MCAP） | R1Arm | `scripts/record_r1arm_real.py --motion-mode ...` |
| 真机数据采集（四相机 MCAP） | R1-A7 | `scripts/record_real.py --motion-mode ...` |
| 查本机相机节点 / 测哪路能出图 | R1Arm | `scripts/probe_r1arm_cameras.py` |
| 仿真回放 | R1Arm | `scripts/replay_r1arm_sim.py --episode <...>.mcap` |
| 真机回放 | R1Arm | `scripts/replay_r1arm_real.py --episode <...>.mcap --section action --motion-mode ...` |
| MCAP 回放 / 真机回放 | R1-A7 | `scripts/replay_mcap.py --episode <...>.mcap` |
| 编译 SDK 桥接库 | R1Arm | `scripts/build_r1arm_sdk_bridge.py` |
| 送真机到指定姿态（带预检） | R1Arm | `scripts/goto_r1arm_pose.py --move` |
| 冻结仿真参考姿态（标定用） | R1Arm | `scripts/view_r1arm_home.py` |
| 启动本机 Dex1 串口→DDS 服务 | R1Arm | `scripts/start_dex1_local.sh` |
| 真机环境变量自检 | R1Arm | `source scripts/real_env.sh` |

---

## 2. 目录结构

```txt
r1arm_teleop/
├── robot/                        # 机器人专用代码
│   ├── config.py                 # R1-A7 配置 / URDF / 关节限位 / 初始位姿
│   ├── r1arm_config.py           # R1Arm 配置 / 关节限位 / ARM_INIT / 坐标映射
│   ├── r1arm_sdk.py              # R1Arm SDK ctypes 封装（R1ArmSdkBus）
│   ├── arm_model.py              # Pinocchio 模型加载与工具函数
│   ├── arm_control.py            # 手臂控制器（仿真/真机，R1-A7）
│   ├── r1_a7_real.py             # R1-A7 真机 DDS 桥（rt/lowcmd + lowstate）
│   ├── mujoco_sim.py             # MuJoCo 仿真封装（R1Arm）
│   ├── urdf_utils.py             # URDF 预处理（肘关节下限 override）
│   ├── visualization.py          # Meshcat 可视化
│   ├── xr_interface.py           # XR 设备接口
│   └── eef/                      # 末端执行器驱动（Dex1 等）
├── teleop/                       # 遥操作框架
│   ├── r1arm_controller_base.py  # R1Arm 共享 tracking + IK 基类
│   ├── r1arm_mujoco_controller.py# R1Arm MuJoCo 输出
│   ├── r1arm_real_controller.py  # R1Arm 真机 SDK 串口输出
│   ├── r1arm_placo_solver.py     # R1Arm Placo IK
│   ├── r1arm_dataset.py          # R1Arm 数据集适配（8/16 维）
│   ├── ik_solver.py              # 阻尼最小二乘 IK
│   ├── placo_solver.py           # Placo QP IK 封装
│   ├── retargeting.py            # 人体姿态映射 / mock XR 位姿
│   ├── xr_client.py              # XRoboToolkit 客户端封装
│   ├── r1a7_controller_base.py   # R1-A7 共享 tracking + IK（旧栈）
│   ├── sim_controller.py         # R1-A7 仿真控制器
│   ├── real_controller.py        # R1-A7 真机控制器
│   ├── image_client.py           # PC2 四相机订阅（R1-A7 图像服务）
│   ├── local_camera_client.py    # 本机相机采集（V4L2 + RealSense）
│   ├── mcap_writer.py            # MCAP 写入
│   └── utils/                    # 几何 / mock 工具
├── scripts/                      # 可执行入口（见 §1 表格）
│   ├── teleop_r1arm_sim.py       # R1Arm MuJoCo 仿真遥操
│   ├── teleop_r1arm_real.py  python scripts/replay_r1arm_real.py \
  --episode /home/ycb410/nt/r1arm_teleop/dataset/2026_10_09_robot_r1arm_right_dex1/episode_0011.mcap \
  --section action \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0    # R1Arm 真机遥操
│   ├── record_r1arm_sim.py       # R1Arm 仿真 MCAP 采集
│   ├── record_r1arm_real.py      # R1Arm 真机 MCAP 采集（本机双相机）
│   ├── record_r1arm_real_qp.py        # Placo QP 速度/加速度对照录制
│   ├── teleop_r1arm_real_qp.py        # Placo QP 速度/加速度对照遥操
│   ├── record_r1arm_real_qp_jerk.py  # Placo QP 加 jerk 约束录制
│   ├── teleop_r1arm_real_qp_jerk.py  # Placo QP 加 jerk 约束遥操
│   ├── replay_r1arm_sim.py       # R1Arm MCAP 回放（仿真/真机通用）
│   ├── probe_r1arm_cameras.py    # 本机相机节点探测
│   ├── build_r1arm_sdk_bridge.py # 编译 R1Arm SDK C++ 桥接库
│   ├── goto_r1arm_pose.py        # 真机送姿态（预检 + 位置控制）
│   ├── view_r1arm_home.py        # 冻结仿真参考姿态（标定用）
│   ├── real_env.sh               # 真机环境变量自检
│   ├── start_dex1_local.sh       # 启动本机 Dex1 服务
│   ├── teleop_sim.py             # R1-A7 Meshcat 仿真遥操
│   ├── teleop_real.py            # R1-A7 DDS 真机遥操
│   ├── record_real.py            # R1-A7 四相机 MCAP 采集
│   └── replay_mcap.py            # R1-A7 MCAP 回放 / 真机回放
├── assets/
│   ├── r1arm/                    # R1Arm URDF + MuJoCo 场景（单/双臂）
│   └── r1_a7/                    # R1-A7 URDF + 场景
├── native/r1arm_sdk_bridge/      # R1Arm SDK C ABI 桥接源码
├── third_party/                  # Unitree SDK / XRoboToolkit（setup.sh 拉取）
└── setup.sh                      # 环境安装脚本
```

---

## 3. 环境准备（一次性）

```bash
cd /home/ycb410/nt/r1arm_teleop
bash setup.sh

conda activate r1_teleop
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
```

- `bash setup.sh --skip-apt`：系统依赖已装好时跳过 apt。
- `bash setup.sh --skip-xrt`：暂不装 XRoboToolkit（无 PICO 也能先跑仿真）。
- 装好后**不要**再手动 `pip install -r requirements.txt`，会与脚本里已验证的
  Placo/Pinocchio 版本组合冲突。

> GitHub 拉取（Unitree SDK / XRoboToolkit）需走本机代理；国内镜像直连。详细代理
> 配置与常见环境问题见 [start_env.md](start_env.md)。

R1Arm 真机还额外需要：

- 独立的 R1 Arm x86 SDK（默认路径 `/home/ycb410/nt/R1_arm_SDK_x86/r1_arm`，
  可用环境变量 `R1ARM_SDK_ROOT` 覆盖）。
- 当前用户加入 `dialout` 组（否则无串口读写权限）。

```bash
source scripts/real_env.sh   # 打印串口 / 标定 / 工具链自检结果
```

---

## 4. 硬件与坐标约定

**R1Arm（7-DoF + Dex1）**：关节序 `J1..J7 + dex1`。IK 使用本项目
`assets/r1arm/R1_Robotic_Arm.urdf`；SDK 自带的 URDF 里 **J4 采用相反的轴和符号**，
因此桥接层只对 J4 做显式反号：

```text
q_sdk[J4] = -q_sim[J4]        # 其余 J1..J3、J5..J7 一一对应
```

仿真坐标系到 R1Arm 基座的映射（home 附近臂沿 `-Y` 伸出、`+Z` 向上）：
遥操 `+X`(前) → 基座 `-Y`，遥操 `+Y`(左) → 基座 `+X`，遥操 `+Z`(上) → 基座 `+Z`。

**R1Arm 真机本机相机**：场景视角用 SN0002 1080P USB Camera（V4L2），腕部视角用
Intel RealSense D405（librealsense）。两者都插在本机 USB 上，不走 PC2 图像服务；
D405 需要 USB 3.x 才能稳定出图，详见 §7.3。

**R1-A7（双臂 + 腰 + 头）**：关节向量 `body(4) + arms(14) + hand(4)`，其中
`body = [waist, vx, vy, vz]`。本项目不控制底盘，`vx/vy/vz` 只是摇杆原始读数，
只入档、不下发。

---

## 5. 仿真遥操作

### 5.1 R1Arm（MuJoCo）

```bash
# 无 PICO，用 mock 手柄自动轨迹（先跑这个验证链路）
python scripts/teleop_r1arm_sim.py --mock-xr

# 连接 PICO；右手柄 grip 激活跟踪，trigger 控制 Dex1，A 键软复位
python scripts/teleop_r1arm_sim.py

# 双臂场景：左右手柄分别控左右臂
python scripts/teleop_r1arm_sim.py --use-dual-arm

# 无显示环境（SSH / 无桌面）
MUJOCO_GL=egl python scripts/teleop_r1arm_sim.py --mock-xr --no-viewer --debug
```

关键参数：`--side {left,right}`（单臂默认右）、`--use-dual-arm`、
`--scale-factor`（手柄位移缩放，默认 1.0）、`--rate`（控制环频率，默认 50 Hz）、
`--sim-substeps`（每控制步仿真子步数，默认 10）、`--reset-blend-frames`（A 键
回 home 的平滑帧数，默认 100）。

## 6. 真机遥操作

> 真机涉及真实运动，先读 [§12 安全须知](#12-安全须知)。R1Arm 桌面式单臂**不要**
> 用 R1-A7 的 `teleop_real.py` 控制，必须用 `teleop_r1arm_real.py`。

### 正常启动流程：
```


cd /home/ycb410/nt/r1arm_teleop
conda activate r1_teleop


终端 A（Dex1 夹爪服务，需要 sudo，保持运行）：

    source /home/ycb410/nt/r1arm_teleop/scripts/real_env.sh
   ./scripts/start_dex1_local.sh



终端 B（遥操）：
python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0 \
   --enable-a-reset --reset-blend-frames 100 \
  --home-on-start

```

python scripts/teleop_r1arm_real.py \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --motion-mode \
  --enable-dex1 --dex1-net enp4s0 \
  --enable-a-reset --reset-blend-frames 100 \
  --home-on-start
  --rate 30 \
  --kp 80 --kd 2

python scripts/teleop_r1arm_real.py \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --motion-mode \
  --enable-dex1 --dex1-net enp4s0 \
  --enable-a-reset --reset-blend-frames 100 \
  --home-on-start
  --rate 100 \
  --kp 80 --kd 2

python scripts/teleop_r1arm_real.py \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --motion-mode --rate 50 \
  --enable-a-reset --reset-blend-frames 100 \
  --home-on-start
  --kp 80 --kd 2 \
  --gravity-ff



数据采集（B 录制 / A 归位，归位轨迹也录进 episode；不自动退出；默认两路相机 640x480）：
python scripts/record_r1arm_real.py \
  --motion-mode --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start \
  --enable-a-reset \
  --task-description "pick and place" \
  --output-dir /home/ycb410/nt/dataset/try

  python scripts/record_r1arm_real.py \
  --motion-mode --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start \
  --enable-a-reset \
  --output-dir /home/ycb410/nt/dataset/try

  

 真机回放（--motion-mode 才动臂，会要你输 MOVE）
python scripts/replay_r1arm_real.py \
  --episode /home/ycb410/nt/dataset/p2_validation/2026_10_08_robot_r1arm_right_dex1/episode_0000.mcap \
  --section action \
  --motion-mode --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0


python scripts/replay_r1arm_real.py   --episode /home/ycb410/nt/dataset/try/episode_0000.mcap   --section action   --motion-mode --port /dev/ttyUSB2   --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt   --enable-dex1 --dex1-net enp4s0




### 6.1 前置：串口 / 桥接库 / 标定

1. 确认串口与权限：

   ```bash
   ls -l /dev/serial/by-id/ 2>/dev/null
   ls -l /dev/ttyUSB* 2>/dev/null
   source scripts/real_env.sh   # 自检 dialout / 串口 / 标定 / 工具链
   source /home/ycb410/nt/r1arm_teleop/scripts/real_env.sh

   ```

2. 编译 SDK 桥接库（只封装 SDK 的 `ArmMotorBus`，编译时不连接/驱动机械臂）：

   ```bash
   python scripts/build_r1arm_sdk_bridge.py \
     --sdk-root /home/ycb410/nt/R1_arm_SDK_x86/r1_arm
   # 产物: build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so
   ```

3. 标定文件默认 `$SDK_ROOT/config/joint_calib.txt`（候选 `joint_calib_candidate.txt`），
   可用 `--calib` 覆盖。标定流程见 [§8 标定与辅助工具](#8-标定与辅助工具)。

4. 终端 A（Dex1 服务，需要 sudo，保持运行）：


  cd /home/ycb410/nt/r1arm_teleop
  ./scripts/start_dex1_local.sh


### 6.2 R1Arm 真机（RS485 串口）

**先做 dry-run（不开串口、不发电机命令）：**

```bash
python scripts/teleop_r1arm_real.py --mock-xr --debug     # 完全无硬件
python scripts/teleop_r1arm_real.py --debug               # 连 PICO 检查手柄/IK，仍不下发
```

**真机启动（首次建议不启用 Dex1）：**

```bash
python scripts/teleop_r1arm_real.py --motion-mode --port "$PORT"
```

程序打印全部参数后，需在终端输入大写 `MOVE` 才会打开串口。启动流程：

1. 零增益 FOC 探测全部 7 关节反馈（此阶段请托住机械臂）。
2. 检查标定后的关节角是否落在仿真 URDF 限位内。
3. 用真机实测关节角初始化 IK，避免从仿真 home 突跳。
4. 在实测启动姿态进入 FOC 位置保持。
5. 按住对应手柄 grip 后才建立末端相对跟踪。

IK 连续性处理（影响录制 action 的质量）：

- **锚定时刷新姿态正则化参考**：Placo 的关节正则化任务参考若停留在
  启动悬垂位姿，grip 激活的第一帧求解会被它拖出一个单帧大跳（12+ rad/s
  的 IK 瞬态，遥操限速会滤掉、但会原样落进录制的 action）。现在每次锚定
  （握下 grip）时把参考刷成当前 q。
- **独立 q/v/a 轨迹状态机**：`trajectory_max_speed_rad_s` 速度上限和
  `ik_max_acc_per_s2` 加速度上限统一作用于 Placo/DLS 的原始解；release、
  regrip、home/reset 都复用当前状态，不重置速度。SDK lead/slew 限幅只是
  最后的安全兜底，录制同时区分 `q_purpose` 与 `q_motor_target`。

正常运行时：**按住 grip** 控制末端相对移动；松开 grip 后轨迹状态机按当前速度连续减速至零，再保持停止姿态；`Ctrl+C`
持续发 0.5 s mode 0 后关串口（机械臂失去位置保持，需托住）。A 键回 home 默认关闭。

**启用单侧 Dex1**（机械臂仍走串口，Dex1 走 DDS）：

```bash
python scripts/teleop_r1arm_real.py \
  --motion-mode \
  --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start
```

只发布所选侧（默认右）的 `rt/dex1/right/{cmd,state}`；2 秒内收不到 state 会拒绝
继续并停机。右 trigger：松开=开、按下=关。

**关键安全参数**（默认值即推荐，勿无确认调大）：

| 参数 | 默认 | 含义 |
|------|------|------|
| `--kp` / `--kd` | 80.0 / 2 | FOC 位置环增益；单个数全关节广播，或 7 个逗号分隔值逐关节（如 `--kp 100,100,100,100,50,50,50`，肩肘 J1–J4 / 腕 J5–J7 分组，对齐 R1-A7 官方增益表） |
| `--max-joint-speed` | 1 rad/s | 目标变化速率上限 |
| `--max-command-lead` | 1 rad | 相对实测位置的指令超前上限 |
| `--feedback-timeout` | 3.0 s | 电机反馈超时 |
| `--foc-timeout` | 1.0 s | FOC 使能超时 |

> 输出端依次执行：URDF 硬限位 → 速度限制 → 指令超前限制 → 电机错误码/FOC 检查 →
> 通信超时保护；XR 断流时保持当前位置，不会自动回退 mock XR。

如需 A 键缓慢回 `ARM_INIT`：

```bash
python scripts/teleop_r1arm_real.py \
  --motion-mode --port "$PORT" \
  --enable-a-reset --reset-blend-frames 250
```

### 6.3 R1-A7 真机（DDS）

```bash
python scripts/teleop_real.py --motion-mode --enable-dex1 --net enp5s0
```

关键参数：`--net`/`--interface`（DDS 网卡）、`--motion-mode`（下发开关，off 为
dry-run）、`--enable-dex1`、`--elbow-lower-override`（默认 -1.30）、
`--head-pose-file`、`--no-confirm`（跳过确认）。

---

## 7. 数据采集与回放

### 7.1 R1Arm 仿真采集

```bash
python scripts/record_r1arm_sim.py \
  --output-dir /home/unitree/datasets/r1arm/pick_demo \
  --camera-views head_left wrist_right \
  --task-description "pick an object" \
  --robot-id r1arm_sim
```

- 采集脚本**不连接真机、不依赖 SDK**，只用 MuJoCo + XRoboToolkit + 本项目 IK。
- 遥操启动即激活；PICO 按键：**Y** 开始/停止并保存、**B** 暂停/恢复、**A** 软复位
  home、**grip** 跟踪、**trigger** 夹爪。
- `--output-dir` 直接指定最终目录，`episode_XXXX.mcap` 自动续号不覆盖；不传时才用
  旧的 `data/<task>/<日期>_robot_<id>_dex1/` 规则。
- `--camera-views` 可选 `head_left / head_right / wrist_left / wrist_right`，数量
  即相机数，写入元数据 `camera_views` / `camera_count`。
- 左臂 `--side left`；双臂 `--use-dual-arm`。
- 无 PICO 自动采集测试（mock grip 第 8 秒才激活，建议至少录 12 秒）：

  ```bash
  MUJOCO_GL=egl python scripts/record_r1arm_sim.py \
    --mock-xr --no-viewer --auto-start --duration 12 --exit-after-duration \
    --output-dir /home/unitree/datasets/r1arm/mock_headless
  ```

### 7.2 R1-A7 真机采集（四相机）

先在机器人端启动图像服务与夹爪驱动：

```bash
ssh unitree@192.168.123.164
conda activate teleop
python3 ~/set_realsense_d405_exposure.py --exposure 9000 --gain 16
./pre_start.sh
```

然后在本机采集（确保 PC2 图像服务已启动）：

```bash
python scripts/record_real.py \
  --motion-mode --enable-dex1 \
  --image-server-address 192.168.123.164 \
  --net enp5s0
```

PICO 按键：**X** 进入/退出遥操、**Y** 开始/停止并保存、**B** 暂停/恢复写盘、
**A** 删除最近一条（空闲时一次）。相机顺序固定为 head_left / head_right /
wrist_left / wrist_right。

### 7.3 R1Arm 真机采集（两路本机相机）

R1Arm 真机把相机直接插在本机，不走 R1-A7 的 ZMQ 图像服务，因此用
`scripts/record_r1arm_real.py`。它复用真机遥操的全部链路（PICO → IK → RS485），
只额外把观测与动作写成 MCAP。

本机当前两路相机：

| 相机 | 角色 | 后端 | 默认分辨率 |
|------|------|------|------------|
| SN0002 1080P USB Camera（海康） | 外部/场景视角 | V4L2（OpenCV） | 640x480@30 MJPG（`/dev/v4l/by-id/...SN0002-video-index0`） |
| Intel RealSense D405 | 腕部视角 | librealsense | 640x480@30 bgr8（按序列号自动匹配） |

```bash
cd /home/ycb410/nt/r1arm_teleop
source scripts/real_env.sh                      # 仅取 PORT 变量
# 另开终端：sudo bash scripts/start_dex1_local.sh   （要用夹爪时）

python scripts/record_r1arm_real.py \
  --motion-mode \
  --port "$PORT" \
  --calib /home/ycb410/nt/calib/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0 \
  --home-on-start \
  --record-button B --pause-button none --enable-a-reset \
  --task-description "pick an object" \
  --output-dir /home/ycb410/nt/datasets/r1arm/real_pick_demo
```

按键：**B** 开始/停止并保存、**A** 软复位回 `ARM_INIT`（归位轨迹也录进 episode）、
暂停已关闭（`--pause-button none`）。这些键都在**右手柄**，单臂遥操无需左手柄。

增益参数（`--kp --kd --max-joint-speed --max-command-lead --scale-factor`）默认值与
`teleop_r1arm_real.py` **完全一致**（`kp=80 / kd=2 / speed=1 / lead=1 / scale=1.0`），
直接录制即可复现遥操的手感与力矩。如需更保守的启动值可显式传
`--kp 10 --kd 0.3 --max-joint-speed 0.10 --max-command-lead 0.03`。
逐关节增益（肩肘 100 / 腕 50，对齐 R1-A7 官方增益表）：
`--kp 100,100,100,100,50,50,50 --kd 2,2,2,2,1.5,1.5,1.5`，会记入 episode 元数据
（`teleop_kp_list`/`teleop_kd_list`）。

先不接真机验证采集链路（相机 + 写盘 + 回放）是否通：

```bash
python scripts/record_r1arm_real.py \
  --auto-start --duration 5 --exit-after-duration \
  --output-dir /tmp/r1arm_real_dryrun
```

不加 `--motion-mode` 时**不会打开串口、不会发任何电机命令**，只连 PICO 与相机。

#### 相机参数

`--camera` 可重复，**顺序即 MCAP 的 `camera_order`**；不传时用上表两台。
每个 `SPEC` 是逗号分隔的 `key=value`：

| 字段 | 说明 |
|------|------|
| `key` | `color_0_head_left` / `color_1_head_right` / `color_2_wrist_left` / `color_3_wrist_right` |
| `backend` | `v4l2`（普通 UVC）或 `realsense`（D405） |
| `device` | V4L2 路径、RealSense 序列号，或 `auto` |
| `match` | 在 `/dev/v4l/by-id` 里按子串找设备（避免 videoN 变号） |
| `size` / `fps` / `fourcc` | 分辨率（`WxH`）、帧率、V4L2 像素格式（如 `MJPG`） |

```bash
# 只录场景相机
python scripts/record_r1arm_real.py --motion-mode --port "$PORT" \
  --camera key=color_0_head_left,backend=v4l2,match=SN0002,size=640x480,fps=30,fourcc=MJPG

# 腕部相机改成 V4L2 设备（例如换成别的 UVC 相机）
python scripts/record_r1arm_real.py --motion-mode --port "$PORT" \
  --camera key=color_0_head_left,backend=v4l2,match=SN0002,size=640x480,fps=30,fourcc=MJPG \
  --camera key=color_3_wrist_right,backend=v4l2,device=/dev/video4,size=640x480,fps=30
```

默认两路均为 **640x480@30**（与仿真观测一致）。注意 SN0002 场景相机只支持
640x480 / 1280x720（1080 档）——请求 800x600 会被 UVC 驱动**静默回退**到 640x480，
不要在 `size=` 里写它不支持的档位。

**相机掉线时的行为**：启动时最多等 `--camera-wait`（默认 8 s）；没有出图、
或只出 1 帧就卡死的相机会被**剔除**，写进 `config.camera_inactive`，采集继续用
剩下的相机（至少要有 1 路）。加 `--require-all-cameras` 可改成直接报错退出。

⚠️ **D405 必须插在 USB 3.x 口上。** 本机实测：D405 挂在 USB 2.0（`usb=2.1`）时
librealsense 只能抓到 1 帧就停，`/dev/video4` 用纯 V4L2 更是完全不出图；
此时采集能跑，但只有场景相机。把 D405 换到蓝色 USB 3 口并重新插拔后即可恢复。

相机节点号每次插拔都会变，用它查：

```bash
conda run -n r1_teleop python scripts/probe_r1arm_cameras.py
# 打印每个 /dev/videoN 是否出图，并给出可直接粘贴的 --camera 参数
```

`realsense` 后端需要 `pyrealsense2`（已装进 `r1_teleop` 环境）：
`conda run -n r1_teleop pip install pyrealsense2`。

#### PICO 按键

> **手柄布局**：X / Y 在**左手柄**，A / B 在**右手柄**。单臂遥操只用右手柄
> （右 grip / trigger 做跟踪），所以若左手柄未配对 / 未唤醒，按 X 不会有反应。

| 按键 | 手柄 | 功能 |
|------|------|------|
| X（默认） | 左 | 开始 / 停止并保存一条 episode（可用 `--record-button` 改） |
| B（默认） | 右 | 暂停 / 恢复录制（同时暂停 XR 跟踪，电机保持位置，`--pause-button` 可改，`none` 关闭） |
| A | 右 | 软复位到 `ARM_INIT`（需 `--enable-a-reset`，**归位轨迹会一并录进当前 episode**） |
| Y | 左 | 未使用 |
| 右 grip / trigger | 右 | 手臂跟踪 / Dex1 夹爪 |

> **录制中按 A 归位**：只要开着 `--enable-a-reset`，录制过程中按 A 会软复位回
> `ARM_INIT`，这段归位运动照常写进 state/action，不需要（也不要）把 A 绑到暂停。
> 暂停键想留就用别的键，想关就 `--pause-button none`。

只接右手柄时，B 录制 / A 归位 / 无暂停：

```bash
python scripts/record_r1arm_real.py ... \
  --record-button B --pause-button none --enable-a-reset \
  --no-cameras
```
python scripts/replay_r1arm_sim.py --episode /home/ycb410/unitree_sh_disk/tools/nt/2026_10_08_robot_r1arm_right_dex1/episode_0000.mcap   --mode action --show-observations
（A 被 `--enable-a-reset` 保留为复位，不能同时作 `--record-button` / `--pause-button`，
脚本会直接报错。）

### 7.4 回放

R1Arm 采集的数据（仿真 §7.1 与真机 §7.3 同格式）都用同一个脚本回放：

```bash
# 动作回放（默认，复现控制命令）
python scripts/replay_r1arm_sim.py --episode <path>/episode_0000.mcap

# 状态回放（逐帧设置关节观测，轨迹最接近采集）
python scripts/replay_r1arm_sim.py --episode <path>/episode_0000.mcap --mode state
python scripts/replay_r1arm_sim.py --episode /home/ycb410/nt/dataset/p2_validation/2026_10_08_robot_r1arm_right_dex1/episode_0000.mcap --mode action
# 同时显示已存观测图像
python scripts/replay_r1arm_sim.py --episode /home/ycb410/nt/dataset/p2_validation/2026_10_08_robot_r1arm_right_dex1/episode_0000.mcap \
  --mode action --show-observations
python scripts/replay_r1arm_sim.py --episode /home/ycb410/nt/r1arm_teleop/dataset/2026_10_09_robot_r1arm_right_dex1/episode_0011.mcap \
  --mode action --show-observations
# 无窗口快速校验格式与动力学回放
MUJOCO_GL=egl python scripts/replay_r1arm_sim.py \
  --episode <path>/episode_0000.mcap --no-viewer --speed 20
```

回放脚本会从元数据自动识别单/双臂、控制侧与 MJCF 路径，并按
`info.camera_order` 校验每帧的 `colors` 键是否齐全，所以真机录的
两条相机数据也能直接回放观测图像。

真机回放（`replay_r1arm_real.py`，把录制到的 state 轨迹在真臂上重跑一遍验证数据）：

```bash
# 先 dry-run 看轨迹（不开串口、不发命令）
python scripts/replay_r1arm_real.py --episode /home/ycb410/nt/dataset/try

# 指定某一条 episode 并真正驱动真机（回放 state 的 7 个关节）
python scripts/replay_r1arm_real.py \
  --episode /home/ycb410/nt/dataset/try/episode_0005.mcap \
  --motion-mode --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt

# 连同夹爪一起回放：加 --enable-dex1，把 state 第 8 维（sim 空间的夹爪位置）
# 转回 closed ratio 后经 DDS 下发到 Dex1（需指定网卡，如 enp4s0）
python scripts/replay_r1arm_real.py \
  --episode /home/ycb410/unitree_sh_disk/tools/nt/2026_10_08_robot_r1arm_right_dex1/episode_0000.mcap \
  --motion-mode --port /dev/ttyUSB2 \
  --calib /home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt \
  --enable-dex1 --dex1-net enp4s0
```

脚本会先软逼近第一帧状态，再按**录制时间戳**逐帧回放（暂停段真实复现，
`--max-frame-gap` 封顶，默认 2 s）。命令 slew 上限分两种情况：**`--section
state`** 默认按录制峰值速度 ×1.2 自适应并封顶在 `--auto-speed-cap`（默认
5 rad/s）；**`--section action`** 默认直接采用**录制时遥操底层同款限速**
（episode meta 里的 `teleop_max_joint_speed`）——因为录制的 action 是 IK
整形后的 `q_purpose`，因此不应再串第二个轨迹整形器。SDK slew、lead 和关节
硬限位仍作为最终安全兜底；若触发，CSV 会显示 `q_purpose` 与最终电机目标的
差异。旧 episode 没有速度元数据时回退到峰值封顶逻辑。`--max-joint-speed`
均可手动覆盖，低于峰值会警告。
**逼近段（悬垂→起始、末端收敛、循环重置）固定用保守的
`--approach-speed`（默认 1 rad/s）**，不继承回放 slew 上限，避免多关节同
时起动的电流冲击。播完先向**最后一条录制状态收敛**再保持（末端位姿与录
制一致）。**`--control-hz 100`（默认 0=按录制帧率）**：控制环升频 + 帧间
线性插值——每周期步长按比例缩小（30 Hz 录制 → 100 Hz 时约 1/3.3），电机
换相纹波与"喀哒"声随之降低；轨迹、时序、录制数据均不变（段时长与隐含
关节速度严格保持，录制帧位姿仍逐帧下发，段尾留部分 tick）。可用 `--speed`
放慢、`--loop` 循环、`--start-frame`/`--end-frame`
截取片段，全程沿用遥操同一套 `limit_sdk_position_command` 限速/限超前保护。
`--episode` 传目录时自动选最新一条。夹爪默认不回放；加上 `--enable-dex1`
后按录制的 `joint_state` 第 8 维回放闭合程度。`--section action` 回放录制
的 action（遥操下发指令，无重力下垂分量）而不是 state——录制时 kp=80 顶
不住重力、实测会比指令低约 0.05 rad，直接回 state 会把下垂量二次叠加
（末端碰桌就是这么来的），回 action 则复现指令高度。action 默认选取
`q_purpose`；每轮真机回放自动保存遥测 CSV，包含原始 action、SDK 安全层最终
命令、期望速度和实测关节位置/速度/力矩，方便多组增益对比。

R1-A7 MCAP 回放：

```bash
python scripts/replay_mcap.py --episode <path>/episode_0000.mcap            # 可视化回放
python scripts/replay_mcap.py --episode <path>/episode_0000.mcap --no-video  # 只校验格式
```

真机回放（先把机器人固定好）：

```bash
python scripts/replay_mcap.py --episode ... --motion-mode --confirm REPLAY_R1
```

---

## 8. 标定与辅助工具

### 8.1 标定参考姿态

`scripts/view_r1arm_home.py` 把仿真机械臂冻结在 `ARM_INIT`，作为真机标定的参考：

```bash
python scripts/view_r1arm_home.py              # 打开 viewer 冻结在 home
python scripts/view_r1arm_home.py --no-viewer  # 只打印角度（SDK 坐标 + 标定参数）
python scripts/view_r1arm_home.py --q 0,1.2,0,-1.2,0,0,0  # 看任意姿态
```

输出里会给出 SDK 坐标系下各关节角与 `calibrate_joints` 的 `--reference-q` 参数。

标定使用 SDK 自带工具：

```bash
source scripts/real_env.sh
"$BIN/test_arm_lowlevel"   # 回读当前姿态
"$BIN/calibrate_joints"    # 生成 joint_calib（候选写入 joint_calib_candidate.txt）
```

### 8.2 送真机到指定姿态（带预检）

`scripts/goto_r1arm_pose.py` 与遥操共用同一套位置控制（mode=1 FOC 位置环 + 硬限位 +
速度限制 + 超前限制），默认目标为仿真 `ARM_INIT` 对应的 SDK 姿态 `[0,1.2,0,1.2,0,0,0]`：

```bash
python scripts/goto_r1arm_pose.py                 # 只读预检，不动（先跑这个）
python scripts/goto_r1arm_pose.py --move          # 预检通过后真的运动（还需手输 MOVE）
python scripts/goto_r1arm_pose.py --move --hold-ms 60000  # 到位后保持 60 s 观察
# 重力前馈：消除稳态悬垂（实测 kp=80 时 J2 垂 0.031 / J4 垂 0.065 rad）
python scripts/goto_r1arm_pose.py --move --hold-ms 30000 --gravity-ff --kp 80 --kd 2 --max-command-lead 1
```

> ⚠️ 真机处于零力矩/低增益状态，全程托住；`--hold-ms` 到点后会失去保持并下坠。

关键参数：`--q`（SDK 坐标）、`--kp/--kd/--max-joint-speed/--max-command-lead`
（`--kp/--kd` 支持单个数或 7 个逗号分隔值逐关节，如
`--kp 100,100,100,100,50,50,50`）、`--gravity-ff`/`--tau-clip`（重力前馈：
URDF 模型在指令姿态处算重力力矩、clip 后经 v2 ABI 的 tau 通道下发；悬垂 =
重力/kp，前馈直接抵掉模型部分——注意腕上挂相机的臂实测负载比裸 URDF 大
~35–40%，剩余由位置环承担）、`--max-delta`（单关节最大移动量，默认 1.5 rad）、
`--allow-large`（跳过移动量检查，危险）、`--done-rad`、`--goto-timeout`、`--move`。

### 8.3 本机 Dex1 服务

单侧 Dex1 走串口→DDS 时，用 `scripts/start_dex1_local.sh` 启动服务（不碰机械臂）：

```bash
scripts/start_dex1_local.sh   # 可用 DEX1_NETWORK / DEX1_PORT 环境变量覆盖默认值
```

---

## 9. PICO 手柄按键

### 9.1 R1Arm（仿真采集 / 遥操）

| 输入 | 功能 |
|------|------|
| grip（侧握） | 激活对应一侧机械臂的末端跟踪 |
| trigger（扳机） | 控制 Dex1：松开=开、按下=关 |
| A（右手柄） | 机械臂软复位到 home |
| Y（采集时） | 开始采集；再按停止并保存当前 episode |
| B（采集时） | 暂停/恢复采集（同时暂停遥操输出） |

> `record_r1arm_sim.py` 用 **Y** 开始/停止，`record_r1arm_real.py`（§7.3）用 **X**，
> 因为它把 Y 留空、且 A 键默认作为软复位而不是删除。

### 9.2 R1-A7 真机采集

| 输入 | 功能 |
|------|------|
| X | 进入 / 退出遥操作模式（退出时双臂平滑回初始位） |
| Y | 开始 / 停止并保存一条 episode（需先进入遥操） |
| B | 暂停 / 恢复当前 episode 写盘（同时暂停手臂输出） |
| A | 删除最近保存的一条 episode（仅空闲时，一次） |
| grip | 激活对应手臂末端跟踪 |
| trigger | 控制 Dex1 夹爪 |

> R1-A7 录制模式下 A 键被占用为「删除」，基类的 A 键复位自动关闭。

---

## 10. 数据格式

仿真和旧数据每条 episode 至少含两个 topic。新版真机录制会额外写入高频控制流：

| Topic | 内容 |
|-------|------|
| `/episode/meta` | 整条一条 JSON：`{info, text, config}` |
| `/whole_body/frame` | 每个 30 Hz 观测帧一条 JSON（图像 + 最近控制快照） |
| `/r1arm/control_sample` | 每个真机控制周期一条 JSON（默认 100 Hz，无图像） |

帧字段统一为：`idx`、`sample_time_ns`、`image_source_frame_id`、`colors`（base64
JPEG）、`states`（实测）、`actions`（下发）。

**R1-A7**（`info.format = r1_a7_four_camera_mcap_json_v1`，四相机固定）：

- 相机键：`color_0_head_left` / `color_1_head_right` / `color_2_wrist_left` /
  `color_3_wrist_right`。
- `robot_q_* = body(4) + arms(14) + hand(4)`，`body = [waist, vx, vy, vz]`。
- `states.robot_q_current` / `actions.robot_q_desired`；拆分字段 `body`、`arms`、
  `hand_state` / `hand_cmd`。

**R1Arm 仿真**（`info.format = r1arm_sim_multicamera_mcap_json_v1`，视角可选）：

- 关节向量严格用仿真实际动作空间，**不含 body**：单臂 8 维
  `J1..J7 + dex1_gripper`；双臂 16 维
  `right_arm(7)+right_gripper(1)+left_arm(7)+left_gripper(1)`。
- 每个 Dex1 只有 1 个逻辑夹爪自由度（状态取两指均值，回放复制给两指）。
- 字段：`states.joint_state`（别名 `states.robot_q_current`）、
  `actions.action`（别名 `actions.robot_q_desired`），拆分字段 `arms`、
  `hand_state` / `hand_cmd`。
- 具体关节顺序写入 `/episode/meta.config.joint_names`，由回放脚本校验。

**R1Arm 真机**（`info.format = r1arm_real_multicamera_mcap_json_v1`，本机相机）：

`/whole_body/frame` 的帧字段、8 维关节向量与上面 R1Arm 仿真**完全同构**，只是相机来自本机设备、
`config` 多出真机信息。新增 `/r1arm/control_sample` 不改变旧 topic；`scripts/replay_r1arm_sim.py` 仍从兼容的 30 Hz `actions.q_purpose` 回放。

`/episode/meta` 的 `config`：

| 字段 | 内容 |
|------|------|
| `robot` / `eef` | `"R1Arm"` / `"dex1"` |
| `simulator` | `null`（真机） |
| `motion_mode` | 是否真的打开串口下发电机指令 |
| `teleop_stack` | `"R1ArmRealTeleopController"` |
| `dual_arm` / `control_side` | `false` / `"right"` |
| `urdf_path` | IK 使用的 URDF 路径 |
| `serial_port` | 真机串口（dry-run 时为空串） |
| `camera_keys` | 实际录进去的相机键，顺序 = `--camera` 顺序 |
| `camera_count` | 实际相机数 |
| `camera_source_streams` | 每路相机的后端 / 设备 / 分辨率 / 累计帧数 |
| `camera_inactive` | 启动时被剔除的相机键（没出图或只出 1 帧就卡死） |
| `camera_devices` | 上面两个字段合并的可读串 |
| `mcap_color_keys` | 同 `camera_keys` |
| `joint_names` | `["J1".."J7", "dex1_gripper_joint"]` |
| `state_dim` / `action_dim` | `8`（与仿真单臂一致） |
| `gripper_dof_per_arm` | `1` |
| `state_layout` | `right_arm(7)+right_gripper(1)` |
| `sim_q_layout` | `right_arm(7)+right_hand_fingers(2)` |

`/episode/meta` 的 `info` 与仿真相同：`format`、`created_at`、`frequency_hz`
（= `--frequency`）、`camera_order`、`camera_positions`（去掉 `color_N_` 前缀，
如 `head_left`）；`text.goal` 为 `--task-description`。

`/whole_body/frame` 每帧字段：

| 字段 | 内容 |
|------|------|
| `idx` | 帧序号，从 0 递增 |
| `sample_time_ns` | 写盘时刻（`time.time_ns()`） |
| `image_source_frame_id` | 该快照中**最早**一路相机的累计帧号 |
| `colors` | `{相机键: {encoding:"jpeg", height, width, channels, data:<base64>}}` |
| `states` | 实测状态 |
| `actions` | 下发目标 |

`states` / `actions` 字段（与仿真单臂逐项一致）：

| 字段 | 维度 | 含义 |
|------|------|------|
| `states.joint_state` / `actions.action` | 8 | 主字段 |
| `states.robot_q_current` / `actions.robot_q_desired` | 8 | 别名，值相同 |
| `states.arms` / `actions.arms` | 7 | `J1..J7` |
| `states.hand_state` / `actions.hand_cmd` | 1 | 逻辑夹爪 |

- **状态** 8 维 = `J1..J7 + dex1_gripper`。`--motion-mode` 下 `arms` 取电机实测反馈
  （经 `sdk_to_model_q` 把 SDK 的 J4 符号翻回仿真约定）；dry-run 下等于指令值。
- **动作** 8 维 = 同一时刻下发的 IK 目标，回放时按 `--mode action` 驱动仿真执行器。
- **夹爪** 1 维 = Dex1 两指开合比例均值，线性映射到仿真区间
  `[DEX1_SIM_CLOSE, DEX1_SIM_OPEN]`；回放时复制给两个手指关节。
- 真机没有 body / 腰自由度，因此**不含 body**，关节向量与仿真单臂同构。
- 出帧按 `--frequency` 的固定时基推进；某帧凑不齐相机时跳过该帧但不改变时基，
  结束时打印 `avg_fps`，低于请求值 90% 会告警。
- 新版真机还记录 `info.control_frequency_hz` 和 `/r1arm/control_sample`；每条含完整 `control_seq`、`control_time_ns`、整形 action、IK 原始解、电机目标和实测 state。摄像头掉帧不会丢弃该控制流，暂停/恢复时通过 `record_segment` 切开。
- `--rate 100 --frequency 30` 可用于 action chunk：每张图像之后，从高频流按时间戳取连续控制样本组成未来 chunk。30 Hz `/whole_body/frame` 本身仍不是高频 chunk；100 Hz 控制也不意味着每个样本都有新的 XR 手部输入（XR 约 30 Hz）。相机到达时间不是曝光时间，训练前仍需校准图像传输延迟。
- 至少要有 1 路相机：`colors` 必须与 `camera_order` 一一对应，否则回放会拒绝。

---

## 11. 参数速查表

### 11.1 R1Arm 真机 `teleop_r1arm_real.py`

| 参数 | 默认 | 说明 |
|------|------|------|
| `--motion-mode` | off | 真正打开串口并下发（off = dry-run） |
| `--port` | `$PORT` / FTDI by-id | RS485 串口设备 |
| `--baud` | 6000000 | 串口波特率 |
| `--sdk-root` | `$R1ARM_SDK_ROOT` / `/home/ycb410/nt/R1_arm_SDK_x86/r1_arm` | SDK 根目录 |
| `--calib` | `<sdk-root>/config/joint_calib.txt` | 标定文件 |
| `--bridge-lib` | `build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so` | 桥接库 |
| `--side` | right | 单臂控制侧 |
| `--scale-factor` | 1.0 | 手柄位移缩放 |
| `--rate` | 100.0 | 控制环频率 Hz（控制序列写入 `/r1arm/control_sample`） |
| `--kp` / `--kd` | 80.0 / 2 | FOC 位置环增益 |
| `--max-joint-speed` | 1 | 目标变化速率上限 rad/s |
| `--max-command-lead` | 1 | 指令超前上限 rad |
| `--feedback-timeout` | 3.0 | 反馈超时 s |
| `--foc-timeout` | 1.0 | FOC 使能超时 s |
| `--enable-dex1` | off | 启用单侧 Dex1 |
| `--dex1-net` | "" | Dex1 的 DDS 网卡 |
| `--enable-a-reset` | off | 允许 A 键回 home |
| `--reset-blend-frames` | 100 | A/home 复位帧数（≈2 s @50 Hz） |
| `--home-on-start` | off | 启动后先软 blend 到 home 再开跟踪 |
| `--mock-xr` / `--debug` / `--no-confirm` | off | 调试开关 |

### 11.2 R1Arm 仿真 `teleop_r1arm_sim.py` / `record_r1arm_sim.py`

| 参数 | 默认 | 说明 |
|------|------|------|
| `--use-dual-arm` | off | 双臂场景 |
| `--side` | right | 单臂控制侧 |
| `--scale-factor` | 1.0 | 手柄位移缩放 |
| `--rate` | 50.0 | 控制环频率 Hz |
| `--sim-substeps` | 10 | 每控制步仿真子步数 |
| `--reset-blend-frames` | 100 | A 键回 home 平滑帧数 |
| `--no-viewer` | off | 无显示环境 |
| `--frequency`（采集） | 30.0 | MCAP 帧率 Hz |
| `--output-dir`（采集） | "" | episode 输出目录 |
| `--camera-views`（采集） | head_left wrist_right | 观测视角 |
| `--auto-start` / `--duration` / `--exit-after-duration`（采集） | off / 0 / off | 自动采集 |

### 11.3 R1Arm 真机采集 `record_r1arm_real.py`

遥操作参数（增益/缩放默认值与 `teleop_r1arm_real.py` **完全一致**，可直接录制）：

| 参数 | 默认 | 说明 |
|------|------|------|
| `--motion-mode` | off | 打开串口并下发（off = dry-run，不碰电机） |
| `--port` | `/dev/ttyUSB0` | RS485 串口设备 |
| `--baud` | 6000000 | 串口波特率 |
| `--sdk-root` | `/home/ycb410/nt/R1_arm_SDK_x86/r1_arm` | 用于推导默认标定路径 |
| `--calib` | `<sdk-root>/config/joint_calib.txt` | 标定文件 |
| `--bridge-lib` | `build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so` | 桥接库 |
| `--urdf` | "" | IK 用 URDF（默认用配置里的） |
| `--scale-factor` | 1.0 | 手柄位移缩放 |
| `--rate` | 50.0 | 控制环频率 Hz |
| `--kp` / `--kd` | 80.0 / 2 | FOC 位置环增益 |
| `--max-joint-speed` | 1 | 目标变化速率上限 rad/s |
| `--max-command-lead` | 1 | 指令超前上限 rad |
| `--feedback-timeout` / `--foc-timeout` | 3.0 / 1.0 | 反馈 / FOC 超时 s |
| `--enable-dex1` | off | 启用单侧 Dex1 |
| `--dex1-net` / `--dex1-domain` | "" / 0 | Dex1 DDS 网卡 / domain |
| `--grip-torque-limit` | -1.0 | 夹爪力矩上限（-1 = 用默认） |
| `--enable-a-reset` | off | 允许 A 键软复位到 `ARM_INIT`（录制中按 A，归位轨迹一并录进当前 episode） |
| `--reset-blend-frames` | 100 | A/home 复位帧数（≈2 s @50 Hz） |
| `--home-on-start` | off | 启动 FOC 保持后先软 blend 到 home 再开跟踪 |
| `--mock-xr` / `--debug` / `--no-confirm` | off | 调试开关 |

相机参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `--no-cameras` | off | 只录 state/action，跳过所有相机（`colors` 为空） |
| `--camera` | UVC 场景相机 + D405 腕部相机 | 可重复，**顺序即 `camera_order`** |
| `--camera-wait` | 8.0 | 启动时等待每路相机出图的秒数 |
| `--require-all-cameras` | off | 有相机没出图就直接退出，而不是只录剩下的 |
| `--max-image-age` | 0.5 | 相机快照超过该秒数视为过期，跳过该帧 |
| `--image-show` | off | 实时预览窗口（两路横向拼接） |
| `--image-preview-scale` | 0.75 | 预览缩放 |

采集参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `--record-button` | `X` | 开始/停止录制的按键（X/Y = 左手柄，A/B = 右手柄） |
| `--pause-button` | `B` | 暂停/恢复录制的按键（`none` = 关闭暂停） |
| `--frequency` | 30.0 | MCAP 图像 observation 出帧频率 Hz（与控制频率独立） |
| `--output-dir` | "" | episode 输出目录（直接指定则忽略下面的规则） |
| `--data-root` | `data/` | 未指定 `--output-dir` 时的根目录 |
| `--task-dir-name` | `r1arm_real_task` | 任务子目录名 |
| `--task-description` | `R1Arm right-arm teleoperation` | 写入 `text.goal` |
| `--robot-id` | `r1arm_right` | 目录名里的机器人标识 |
| `--auto-start` | off | 启动即开始录制（不用按录制键） |
| `--duration` | 0.0 | 单条时长秒数，0 = 手动停止 |
| `--exit-after-duration` | off | 到时长后自动退出程序 |

### 11.4 R1-A7 `teleop_real.py` / `record_real.py`

| 参数 | 默认 | 说明 |
|------|------|------|
| `--net` / `--interface` | eth0 / enp5s0 | DDS 网卡 |
| `--motion-mode` | off | 下发开关 |
| `--enable-dex1` | off | 启用真实 Dex1 |
| `--elbow-lower-override` | -1.30 | 肘关节下限覆盖 rad |
| `--head-pose-file` | "" | 头部姿态 JSON |
| `--image-server-address` | 192.168.123.164 | PC2 图像服务地址 |
| `--image-server-port` | 5555 | 图像服务端口 |
| `--frequency`（采集） | 30.0 | MCAP 帧率 Hz |
| `--waist-joystick-gain` / `--waist-joystick-deadband`（采集） | 0.8 / 0.08 | 腰摇杆 |

---

## 12. 安全须知

- **真机启动前**：确认机械臂固定可靠、急停触手可及、关节初始姿态正常，并先跑
  dry-run 与 `goto_r1arm_pose.py` 预检。
- **启动探测阶段**（零增益 FOC）机械臂不保持姿态，务必托住。
- **`Ctrl+C` 停机后**机械臂失去位置保持，需托住，避免下坠。
- **不要**无官方确认地调大 `--kp`、`--max-joint-speed` 或 `--max-command-lead`。
- **R1Arm 桌面式单臂**只走 `teleop_r1arm_real.py`，不要用 R1-A7 的 DDS 入口控制。
- 真机回放（`replay_mcap.py --motion-mode`）需显式 `--confirm REPLAY_R1`，且先把
  机器人固定好。

---

## 13. 常见问题

- **手柄动、机械臂不动**：需持续握住对应手柄的 grip；只连 PICO 或终端输入 `x/y`
  不会更新目标。
- **`--mock-xr` 启动后右臂暂时不动**：mock 周期前 3 秒不激活，3–8 秒只激活左臂，
  8–13 秒才激活右臂。
- **没有桌面 / OpenGL 报错**：加 `MUJOCO_GL=egl` 与 `--no-viewer`。
- **保存时等待较久**：所选视角后台渲染，停止后要把队列写完；出现 `SAVED` 前不要
  再按 Y 开始下一条。若更重视即时保存可 `--frequency 10`。
- **终端报 neutral self collision warning**：IK 自碰撞提示，不是采集失败，出现
  `ready` 即可继续。
- **`meshcat` / `sqlite3` 报 `CXXABI_1.3.15` not found**：`LD_LIBRARY_PATH` 被系统
  `libstdc++` 抢占，激活环境后执行
  `export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"`。
- **XRoboToolkit 报 CMakeCache 路径不匹配**：`build/` 从别处拷来带旧绝对路径，
  清掉重编（详见 [start_env.md](start_env.md)）。
- **串口路径不存在**：以 `ls /dev/serial/by-id/`、`ls /dev/ttyUSB*` 实际枚举为准，
  并确认当前用户已加入 `dialout`。
- **想快速校验 MCAP 与动作维度**：
  `MUJOCO_GL=egl python scripts/replay_r1arm_sim.py --episode <path>/episode_0000.mcap --no-viewer --speed 20`。
- **真机采集提示 `[Camera] UNAVAILABLE color_3_wrist_right`**：D405 没出图。先确认它
  插在 **USB 3.x** 口（`lsusb -t` 里 D405 应挂在 `20000M` 那条总线上，不是 `480M`），
  换口后重新插拔，再跑 `scripts/probe_r1arm_cameras.py` 复核。
- **真机采集 `avg_fps` 远低于 `--frequency`**：某路相机出图太慢，录制在等相干快照。
  多半是相机被剔除前的抖动或 USB 带宽不足；看
  `config.camera_source_streams` 里各路的 `frames=` 计数。
- **`--image-show` 打不开窗口**：需要图形界面（非 SSH 无头环境）。
- **`pyrealsense2` 缺失**：`conda run -n r1_teleop pip install pyrealsense2`。
