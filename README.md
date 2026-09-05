# R1-A7 遥操作与数据采集框架

基于 **Unitree R1-A7 人形机器人** 的 XR 遥操作框架, 支持 PICO VR 半身遥操数据采集。

## 项目结构

```txt
r1a7_teleop/
├── robot/                       # R1 机器人专用代码
│   ├── config.py                # 配置、URDF 路径、关节定义、初始位姿常量
│   ├── arm_model.py             # Pinocchio 模型加载与工具函数
│   ├── arm_control.py           # 手臂控制器（仿真/真机）
│   ├── r1_a7_real.py            # 真机 DDS 桥（rt/lowcmd + lowstate）
│   ├── eef/                     # 末端执行器驱动
│   │   ├── dex1.py              # Dex1 夹爪（set_gripper_ratios + 力限位 latch）
│   │   └── dex1_controller.py   # Dex1 夹爪控制（Dex1Controller）
│   ├── gripper.py               # Dex1 夹爪仿真控制（GripperController）
│   ├── urdf_utils.py            # URDF 预处理（肘关节下限 override）
│   ├── visualization.py         # meshcat 可视化
│   └── xr_interface.py          # XR 设备接口
├── teleop/                      # 通用遥操作框架
│   ├── placo_solver.py          # PlacoIKSolver（Placo QP IK 封装）
│   ├── ik_solver.py             # 阻尼最小二乘 IK 求解器（迭代 + 正则）
│   ├── retargeting.py           # 人体姿态映射 / mock XR 位姿
│   ├── xr_client.py             # XRoboToolkit 客户端封装
│   ├── r1a7_controller_base.py  # 共享 tracking+IK
│   ├── sim_controller.py        # SimTeleopController
│   ├── real_controller.py       # RealTeleopController
│   └── utils/
│       └── geometry.py          # 头显/手柄坐标系变换与四元数工具
├── scripts/                     # 可执行脚本
│   ├── teleop_sim.py            # 仿真遥操作入口
│   └── teleop_real.py           # 真机遥操作入口
├── assets/
│   ├── r1_a7/
│   │   └── A7.urdf              # R1-A7 URDF 模型
│   └── r1arm/
│       ├── R1_Robotic_Arm.urdf  # R1 单臂 7-DoF URDF
│       └── scene.xml            # MuJoCo 仿真场景
├── third_party/                 # Unitree SDK / XRoboToolkit（setup.sh 拉取）
├── setup.sh                     # 环境安装脚本
└── start_env.md                 # 代理说明、仿真测试、常见问题
```


## 快速开始

当前有两套控制栈，**入口脚本默认走新栈**；旧栈代码仍保留，可直接 import 使用。

| | 新栈（推荐） | 旧栈（兼容保留） |
|--|-------------|-----------------|
| tracking + IK | `teleop/tracking_controller.py` | `teleop/r1a7_controller_base.py` |
| 仿真 / 真机 | `teleop/teleop_runner.py`（`motion_mode` 切换） | `sim_controller` / `real_controller` |
| 录制 | `scripts/record_real.py` 驱动 `TeleopRunner` | `record_controller.py` |
| 可视化开关 | `--is-vis` / `--no-is-vis`（默认开） | 构造参数 `visualize=` |

### 1. 环境准备

```bash
bash setup.sh
conda activate r1_teleop
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
```

### 2. 仿真遥操作

**R1-A7（双臂，Meshcat）：**

```bash
# mock XR（不需要 PICO）；Meshcat 默认开启
python scripts/teleop_sim.py --mock-xr

# 带调试输出
python scripts/teleop_sim.py --mock-xr --debug

# 关掉可视化
python scripts/teleop_sim.py --mock-xr --no-visualize

# 连接 PICO，在 VR 中遥操作，可以不加 debug
python scripts/teleop_sim.py --debug
```

**R1 单臂 7-DoF（MuJoCo）：**

```bash
# mock XR；默认右手柄，MuJoCo viewer 开启
python scripts/teleop_r1arm_sim.py --mock-xr

# 连接 PICO，握右手柄 grip 激活跟踪，A 复位
python scripts/teleop_r1arm_sim.py

# trigger 控制 Dex1 开合（与 A7 相同：松开=开，按下=关）
python scripts/teleop_r1arm_sim.py --mock-xr

# 无显示环境
python scripts/teleop_r1arm_sim.py --mock-xr --no-viewer --debug
```
（需已安装 `mujoco`；重新跑 `bash setup.sh` 或 `pip install mujoco`。）


### 3. 真机遥操作

```bash
python scripts/teleop_real.py --motion-mode --enable-dex1 --net enp5s0
```

### 4. 数据采集 / 回放

```bash
# 开启机器人端图像服务端和夹爪驱动
ssh unitree@192.168.123.164
conda activate teleop
python3 ~/set_realsense_d405_exposure.py \
  --exposure 9000 \
  --gain 16

./pre_start.sh

# 采集（确保 PC2 image server 已启动）
python scripts/record_real.py \
  --motion-mode \
  --enable-dex1 \
  --image-server-address 192.168.123.164 \
  --net enp5s0

# 回放校验（纯本地）
python scripts/replay_mcap.py --episode /home/karthus_chen/unitree_sh_disk/tools/ycb/datasets/r1/logistics/mcap/episode_0000.mcap

# 只校验格式，不开窗口
python scripts/replay_mcap.py --episode <path>/episode_0000.mcap --no-video
```

**Pico 手柄按键**

| 按键 | 功能 |
|------|------|
| X | 进入 / 退出遥操作模式（退出时双臂平滑回初始位） |
| Y | 开始 / 停止并保存一条 episode（需先进入遥操作模式） |
| B | 暂停 / 恢复当前 episode 的写盘（同时暂停手臂输出） |
| A | 删除最近保存的一条 episode（仅空闲时可用，只能删一次） |
| 侧握 grip | 激活对应手臂的末端跟踪 |
| 扳机 trigger | 控制 Dex1 夹爪开合 |

> 录制模式下 A 键被占用为「删除」，因此基类的 A 键复位功能会自动关闭。

**数据格式**

每条 episode 为一个 `episode_XXXX.mcap` 文件，包含两个 topic：

| Topic | 内容 |
|-------|------|
| `/episode/meta` | 整条 episode 一条 JSON：`{info, text, config}` |
| `/whole_body/frame` | 每控制帧一条 JSON（见下） |

`info.format` 固定为 `r1_a7_four_camera_mcap_json_v1`。

**帧字段（`/whole_body/frame`）**

| 字段 | 说明 |
|------|------|
| `idx` | 帧序号 |
| `sample_time_ns` | 采样时间戳（ns） |
| `image_source_frame_id` | 图像源帧 id |
| `colors` | 四路相机 JPEG（base64） |
| `states` | 硬件实测 |
| `actions` | 控制下发 |

**相机顺序（`colors`）**

| Key | 相机 |
|-----|------|
| `color_0_head_left` | 头部左 |
| `color_1_head_right` | 头部右 |
| `color_2_wrist_left` | 腕部左 |
| `color_3_wrist_right` | 腕部右 |

**`states` / `actions`**

| | `states`（实测） | `actions`（下发） |
|--|------------------|-------------------|
| 全身拼接 | `robot_q_current` | `robot_q_desired` |
| 腰+摇杆 | `body` | `body` |
| 双臂 | `arms` | `arms` |
| 夹爪 | `hand_state` | `hand_cmd` |

维度：`robot_q_*` = `body(4) + arms(14) + hand(4)`，其中 `body = [waist, vx, vy, vz]`。

> 本项目不控制底盘：`vx/vy/vz` 只是摇杆原始读数，只入档、不下发，为与既有数据集布局兼容而保留。


## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--scale-factor` | 1.0 | IK 缩放因子 |
| `--elbow-lower-override` | -1.30 | 肘关节下限覆盖 |
| `--frequency` | 30.0 | 数据采集帧率（Hz） |
| `--image-server-address` | 192.168.123.164 | PC2 图像服务地址 |