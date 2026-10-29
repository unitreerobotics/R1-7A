# R1-7A 详细安装与使用说明

[返回快速上手](README.md)

本文记录首次安装、设备配置、标定、录制回放及故障排查。快速安装只需克隆仓库后执行 `bash setup.sh`，遇到具体配置问题时查阅相应章节。

硬件名称统一为 **R1-7A**。代码中的 `r1arm` 脚本路径、`R1ARM_*` 环境变量和 `r1_teleop` Conda 环境名保持与现有实现一致。

## 📥 环境安装

### 1. 克隆仓库

本文按全新机器配置编写。使用 Ubuntu 22.04 x86_64，先安装 Git、Conda 和系统 Python 3，再执行：

```bash
git clone --recurse-submodules https://github.com/unitreerobotics/R1-7A.git
cd R1-7A
```

已有代码但未初始化依赖：

```bash
git submodule update --init --recursive
```

### 2. 安装依赖与编译

安装项目所需依赖并完成编译：

```bash
bash setup.sh
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate r1_teleop
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

[setup.sh](setup.sh) 会创建或复用 `r1_teleop`，安装固定版本的 Placo / Pinocchio / NumPy 组合、Python DDS 客户端和 XR 绑定，并编译机械臂 SDK、bridge 与 Dex1 服务。**安装不连接或使能电机，也不生成实体设备标定。**

| 选项 | 用途 |
|---|---|
| `bash setup.sh --skip-apt` | 系统依赖已装好时跳过 apt；仍会执行后续安装和编译 |
| `bash setup.sh --skip-xrt` | 跳过 XR 依赖安装，先检查 mock 仿真；真机 PICO 遥操仍需 XR |
| `ENV_NAME=r1_teleop bash setup.sh` | 指定环境名；若改名，夹爪启动时同步设置 `DEX1_CONDA_ENV` |

核心版本为 `placo==0.9.23`、`pin==3.8.0`、`coal==3.0.2`、`numpy==2.2.6`，完整组合以脚本为准。脚本默认使用 Conda-forge 和阿里云 PyPI 镜像；可通过 `CONDA_FORGE`、`PIP_INDEX` 覆盖。

第三方依赖由 submodule 固定版本，并叠加 [native/dependency_patches](native/dependency_patches/README.md) 中的适配。只克隆上游源码、未应用补丁，不等于本仓库使用的基线。补丁冲突时安装会停止，请检查工作树，不要强制覆盖本地修改。

## 🥽 XRoboToolkit 配置

Python 绑定、电脑桌面服务和头显应用是三个独立部分：

| 部分 | 准备方式 |
|---|---|
| Python SDK / binding | `setup.sh` 安装，遥操进程使用 |
| 电脑 PC Service | 按 [XRoboToolkit-PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) 安装说明单独安装、启动；setup 不部署完整桌面服务 |
| PICO 客户端 | 按 [XRoboToolkit-Unity-Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client) 安装说明安装到头显；本机使用过 `XRoboToolkit-PICO-1.1.1.apk` |

1. 电脑与 PICO 网络互通；避免访客网络或 AP 客户端隔离。
2. 电脑打开一个 PC Service，PICO 应用输入电脑的局域网 IP 并连接。
3. 头显应用启用控制器数据发送，保持应用在前台、头显唤醒，移动左右手柄检查数据持续更新。
4. 先用仿真确认左右手柄输入，再启动真机。若客户端设置了 A 键暂停发送快捷键，需要处理它与本项目 A 回 home 的冲突。

Python 客户端连接本机 `127.0.0.1:60061`。`Connected to XRoboToolkit` 只表示 SDK 连接成功；时间戳为 0、持续不变或输入为 stale 时，仍不能进行正常跟踪。

同机夹爪 DDS 默认走 `lo`，与头显 Wi-Fi 分开配置。替换 Python binding 后重启 Python 进程；替换 PC Service 运行库后重启桌面服务。不要同时运行多个旧版 XR 采样器或控制器。

## ⚙️ 设备配置与标定

### 1. 确认串口与权限

```bash
ls -l /dev/serial/by-id/
id -nG
```

推荐使用稳定的 `by-id` 路径。USB 重插后 `/dev/ttyUSB0` 等编号可能变化；**更换转接器后，by-id 中的序列号也会变化**。

本项目四通道 FTDI 的参考接线如下，必须与实物核对：

| 设备 | 接口后缀 | 默认电机 ID / 用途 |
|---|---|---|
| 右夹爪 | `if00-port0` | ID7，DDS `right` |
| 右机械臂 | `if01-port0` | ID0～6，右臂七轴 |
| 左夹爪 | `if02-port0` | ID7，DDS `left` |
| 左机械臂 | `if03-port0` | ID0～6，左臂七轴 |

当前本机转接器序列号为 `FTANPO49`；代码兼容的旧默认值为 `FTBPG771`。两者都不是其他机器应照抄的设备 ID。

权限不足时：

```bash
sudo usermod -aG dialout "$USER"
```

注销重新登录，或在待用终端执行 `newgrp dialout`，然后重新激活 Conda、加载设备配置。`getent group dialout` 中已有用户名，不代表旧终端的组权限已经生效。

### 2. 保存本机配置

所有命令均从仓库根目录执行。将下面内容保存为根目录 `.env`，**先替换转接器前缀及夹爪端点文件路径**。`.env` 已被 Git 忽略，用于保存机器配置。

```bash
# 使用 setup.sh 编译的仓库内依赖。
export R1ARM_SDK_ROOT="$PWD/third_party/r1_arm_sdk"
export R1ARM_BRIDGE_LIB="$PWD/build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so"
export DEX1_SERVICE_ROOT="$PWD/third_party/dex1_1_service"
export UNITREE_SDK2_ROOT="$PWD/third_party/unitree_sdk2"

# 示例为本机转接器；换机器必须按 ls /dev/serial/by-id/ 的实际输出修改。
export R1ARM_SERIAL_PREFIX="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTANPO49-"
export PORT_RIGHT="${R1ARM_SERIAL_PREFIX}if01-port0"
export PORT_LEFT="${R1ARM_SERIAL_PREFIX}if03-port0"
export RIGHT_DEX1_PORT="${R1ARM_SERIAL_PREFIX}if00-port0"
export LEFT_DEX1_PORT="${R1ARM_SERIAL_PREFIX}if02-port0"
export PORT="$PORT_RIGHT"

export DEX1_NETWORK=lo
export DEX1_ENDPOINTS_FILE=/absolute/path/to/your/dex1_endpoints.json
```

以上路径对应 `setup.sh` 安装的仓库内依赖。串口前缀和夹爪端点路径应替换为当前设备的实际配置，原标定保持与各自实体对应。

**每个新终端**执行：

```bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate r1_teleop
source .env
source scripts/real_env.sh
```

[real_env.sh](scripts/real_env.sh) 只设置变量、检查文件和权限，不打开电机。应看到所需串口为 `Accessible`，且没有 `Calibration not readable`。脚本不自动加载 `.env`，必须先手动 source。

| 配置项 | 当前行为 |
|---|---|
| `R1ARM_SDK_ROOT` | 必须先指定；未指定时，环境脚本默认找项目相邻的 `R1_arm_SDK_x86/r1_arm`，不是 setup 编译的仓库内 SDK |
| `R1ARM_BRIDGE_LIB` | 推荐显式指定；否则环境脚本优先选已有 `build/dual_sdk_bridge`，再选标准 `build/r1arm_sdk_bridge` |
| `R1ARM_SERIAL_PREFIX` | 环境脚本与 `dual/` 的默认端口及角色检查共用此前缀 |
| `PORT_RIGHT` / `PORT_LEFT` | 环境变量用于检查；双臂入口默认按前缀生成端口，单独改接线须传 `--right-port/--left-port` |
| `PORT` | 单臂入口的串口默认值 |
| `CALIB_RIGHT` / `CALIB_LEFT` | 默认指向 SDK 的 `config/joint_calib_zero_右臂.txt` / `joint_calib_zero_左臂.txt` |
| `DEX1_SERVICE_ROOT` / `DEX1_SERVER` | 服务目录或可执行文件；默认服务位于目录内 `bin/dex1_1_gripper_server` |
| `UNITREE_SDK2_ROOT` | 夹爪服务所用 C++ DDS 依赖路径 |
| `DEX1_NETWORK` | 同机默认 `lo`，服务与 Python 客户端需一致，默认 domain 为 0 |
| `DEX1_ENDPOINTS_FILE` | 当前实体夹爪的逐侧端点 JSON；不设置则使用仓库默认文件 |

已有终端可能残留旧 SDK、串口或 bridge 变量，应重新 source 已更新的 `.env` 和 `real_env.sh`。更换前缀时同时更新四个端口及 `PORT`；仅改前缀不会覆盖 shell 中已有的端口变量。

### 3. 机械臂标定

机械臂标定格式为 `joint dir scale offset`，关节编号为 0～6：

```text
calibrated_q = dir × scale × raw_motor_q + offset
```

已有与当前实体及编码器状态匹配的原标定，可以直接继续使用。仓库代码、安装脚本和更换串口转接器均不能替代实体标定；不要因 USB 重连就自动重新标定。

默认文件位置：

```text
$R1ARM_SDK_ROOT/config/joint_calib_zero_右臂.txt
$R1ARM_SDK_ROOT/config/joint_calib_zero_左臂.txt
```

不同路径通过命令行指定，避免依赖旧 shell 中的 `CALIB` 别名：

```bash
# 单臂
python scripts/teleop.py --right --motion-mode --calib /absolute/path/right.txt

# 双臂；初次验证先关闭相机
python scripts/teleop.py --dual --motion-mode \
  --right-calib /absolute/path/right.txt --left-calib /absolute/path/left.txt
```

左右标定独立，左侧模型默认 J2/J6 `mirrored`、J4 `model`；物理标定与模型方向转换分开，不能在零点表中重复反号。home 为共同模型坐标 `0,1.2,0,-1.2,0,0,0`，不是七轴机械零位。

首次没有可用标定时，按机械臂 SDK 的 [标定工具说明](third_party/r1_arm_sdk/README.md) 确认真实参考姿态及每轴方向、比例，使用 `calibrate_joints` 生成本机文件。SDK 工具会连接真实电机，须支撑机械臂，并用 `--out` 写到新的文件；不要在任意摆放姿态直接保存为零位。

对**已有正确方向／比例的标定**重新采集物理全零，可使用本项目工具。下面第一条只检查文件；加 `--capture` 才打开串口并发送零增益探测，需人工将七轴准确置于物理全零并确认：

```bash
python dual/calibrate.py --side right --bridge-lib "$R1ARM_BRIDGE_LIB"
python dual/calibrate.py --side right --bridge-lib "$R1ARM_BRIDGE_LIB" --capture
```

左臂将 `--side right` 改为 `--side left`。默认生成独立候选及采样记录；只有显式加入 `--install` 才备份并替换指定文件。此工具要求已有标定，不能代替首次方向／比例确认。显式传 `--bridge-lib`，避免它的历史默认路径与标准构建目录不同。

角度越界时先核对机械姿态、标定文件、左右侧及编码器基准。程序保留启动检查；不要通过放宽限位、强行取模或自动采用候选文件使错误状态通过。

### 4. 夹爪端点与服务

左右电机可在独立总线上同为 ID7，DDS `side` 决定左右 topic，不由 ID7 自动推断。若实际电机 ID 不同，启动双侧服务时使用 `--right-id/--left-id`；ID15 为广播地址，不用于探测。

端点 JSON 的结构为：

```json
{
  "source": "本机逐侧实测记录",
  "right": {"closed": -4.4258, "open": 0.8784},
  "left": {"closed": -0.4819, "open": 4.8166}
}
```

以上数值仅说明格式，**不是通用标定**。单位为电机弧度，要求有限值且 `open > closed`；换夹爪、改变编码器基准或当前反馈明显不在端点区间时，需重新核对。指定文件缺失或内容无效时会报错，不回退到默认值。

需要采集端点时，先启动对应夹爪服务，退出所有遥操、回放及其他命令发布器，然后执行：

```bash
python scripts/capture_dex1_endpoints.py --network lo
```

按工具提示人工轻放到各侧闭合／张开端点，遇到制动阻力不要强行掰动。工具只订阅反馈，输出 `data/diagnostics/.../endpoints.json`，不会自动安装。检查后把 `.env` 中 `DEX1_ENDPOINTS_FILE` 指向该文件，并在各终端重新加载。端点和机械臂标定需单独备份，不依赖可清理的 build 目录。

## 🧪 仿真与启动前检查

先在 Conda 环境中检查 mock 仿真，不连接真实机械臂：

```bash
python scripts/teleop.py --sim --right --mock-xr
python scripts/teleop.py --sim --dual --mock-xr
```

检查真实 PICO 输入时去掉 `--mock-xr`。双臂无硬件控制流程也可短时运行：

```bash
python scripts/teleop.py --dual --mock-xr --no-cameras --duration 3
```

这只能验证软件流程，不能证明实体供电、串口回复、标定或实际运动正确。`--mock-xr` 不能与 `--motion-mode` 合用。

真机开始前确认：串口角色正确且可读写、左右标定与实体对应、bridge 可加载；夹爪服务各侧识别成功；XR 位姿与时间戳持续更新。检查时只运行一套占用对应串口的程序。

## 🛠️ 遥操作

### 0. 统一命令规则

```bash
python scripts/teleop.py --right --motion-mode
python scripts/teleop.py --left --motion-mode
python scripts/teleop.py --dual --motion-mode
python scripts/teleop.py --left --record --motion-mode --no-cameras --output-dir data/left_real
```

`--right`、`--left`、`--dual` 必须三选一；`--record` 开启录制，不加时完全不创建录制线程、相机和输出文件。单臂 8 维、双臂 16 维，左臂元数据保存 `control_side=left`。单臂真机回放自动读取记录侧别并使用对应的标定、串口及左臂坐标转换；显式指定与记录不符的 `--side` 会拒绝播放。

默认 X 开始／保存，B 暂停／恢复，A 回 home。`--auto-start` 仅与 `--record` 一起使用，启动成功后立即开始一条 episode；真机仍需 `--motion-mode` 和 MOVE 确认。`--duration N` 在真机模式下限制程序运行时间，退出时先停机再完成写盘。

仿真增加 `--sim`，三种臂选择与录制开关保持一致：

```bash
python scripts/teleop.py --sim --left --mock-xr --no-viewer --duration 3
python scripts/teleop.py --sim --dual --record --mock-xr --no-viewer \
  --auto-start --duration 1 --exit-after-duration --output-dir data/sim_dual
```

`python scripts/teleop.py --sim --help` 显示仿真专用参数。仿真使用 50 Hz 默认控制频率；`--exit-after-duration` 在达到指定录制时长、保存 episode 后退出。

以下均从仓库根目录运行，每个终端先完成 `.env` 和 `real_env.sh` 加载。

### 1. 终端一：启动夹爪

右臂单臂使用：

```bash
bash scripts/start_gripper.sh
```

双臂使用：

```bash
bash dual/start_grippers.sh
```

二选一，保持终端运行。应看到对应侧的 `Detected motors` 与 `Dex1-1 Gripper Server started.`；双臂需左右各一次。只打印 `Checking Dex1 port` 或 `Official Dex1` 不表示识别成功。

启动器检查端口占用，不主动杀掉已有服务；任一受管理服务退出时会停止另一侧。切换单／双臂前先 Ctrl+C 退出原服务。刚上电或重连后先等待设备就绪；出现无回复时检查供电、接线、ID 与端口，不反复并行启动。

### 2. 终端二：启动机械臂遥操

右臂单臂：

```bash
python scripts/teleop.py --right --motion-mode
```

双臂，先验证运动与手柄输入，不依赖相机：

```bash
python scripts/teleop.py --dual --motion-mode
```

核对打印的左右串口、标定、增益与归位提示，确认支撑、急停和运动空间后输入 `MOVE`。**当前单臂、双臂均默认回 home**，到位后再按 grip 进行相对跟踪。双臂先等待两侧夹爪反馈，先就绪的机械臂保持实测姿态，等待另一侧就绪后再开始归位。

- 不希望启动归位时加 `--no-home-on-start`；电机会保持当前姿态，仍需满足启动检查。
- 没有夹爪时加 `--no-enable-dex1`，并跳过夹爪服务；该选项不放宽机械臂检查。
- 仅加 `--record` 时会请求相机：左右单臂各两路，双臂三路；`--no-cameras` 可关闭图像。纯遥操不打开相机。
- 双臂可加 `--timing-report` 查看控制耗时；目标 90 Hz 不代表实体通信一定达到 90 Hz。

### 3. 手柄按键

| 输入 | 行为 |
|---|---|
| 左 / 右 grip | 按住后对应机械臂跟踪相对位姿；单臂使用所选侧手柄 |
| 左 / 右 trigger | 控制对应夹爪闭合比例 |
| A（右手柄） | 平滑回 home；双臂同时归位 |
| X（左手柄） | 录制开始／停止并保存；仅加 --record 时启用录制 |
| B（右手柄） | 录制期间暂停／恢复记录与跟踪；暂停后形成新 `record_segment` |
| Ctrl+C | 退出并停止控制；退出后继续支撑机械臂 |

三个模式均须加 `--record` 才启用 X/B 录制。保存一条 episode 后，等待保存完成提示再退出。默认 X 在左手柄、B/A 在右手柄。

当前 XR 保护包括有效性、跳变与时效检查；停更超过 250 ms 停止／保持，恢复稳定后重新锚定，不追补断流期间位移。未启用旧版 cutoff / beta 滤波参数或位姿预测。

## 📦 数据采集

采集入口本身包含遥操作，先退出已有控制器，保持夹爪服务运行。

### 1. 无相机录制，先检查动作链路

```bash
# 右臂
python scripts/teleop.py --right --record --motion-mode --no-cameras \
  --output-dir data/right_real

# 双臂
python scripts/teleop.py --dual --record --motion-mode --no-cameras --data-root data/dual_real
```

两条命令选择一种运行。X 开始／保存，B 暂停／恢复，A 回 home。三个模式使用同一控制／录制实现；没有 `--record` 时不创建录制线程和相机。

### 2. 启用图像

| 图像键 | 默认设备识别 | 请求规格 |
|---|---|---|
| `color_0_head_left` | RealSense D405 | 640×480、30 fps，曝光 8000 µs、gain 16 |
| `color_2_wrist_left` | V4L2 / JR0001 | 640×480、30 fps、MJPG；左臂／双臂使用 |
| `color_3_wrist_right` | V4L2 / JR0002 | 640×480、30 fps、MJPG |

默认配置在 [local_camera_client.py](teleop/local_camera_client.py)，设备名称和曝光需按实际设备、照明调整。`setup.sh` 已安装 `pyrealsense2`；RealSense 仍需设备正确连接并有访问权限。分别检查相机：

```bash
python scripts/verify_r1arm_right_cameras.py
python scripts/verify_r1arm_dual_cameras.py
```

两个验证程序按需依次运行，退出后再录制，避免设备争用。成功后：

```bash
# 单臂两路图像，要求请求的相机均可用
python scripts/teleop.py --right --record --motion-mode \
  --output-dir data/right_real

# 双臂三路图像，默认要求全部相机可用
python scripts/teleop.py --dual --record --motion-mode --data-root data/dual_real
```

可重复传 `--camera` 覆盖整套默认相机，例如 `--camera 'key=color_3_wrist_right,backend=v4l2,match=JR0002,size=640x480,fps=30'`；规格语法见入口 `--help` 及相机实现。相机预览可先运行 `scripts/view_r1arm_cameras.py`，退出预览后再录制，避免设备争用。

### 3. 输出格式

目录内依次生成 `episode_0000.mcap` 等文件。示例显式指定 `data/right_real` / `data/dual_real`；统一用 `--output-dir` 指定目录，`--data-root` 为同义参数。

```text
单臂：所选侧 J1–J7 + 对应夹爪 = 8 维
双臂：右臂 J1–J7 + 右夹爪 + 左臂 J1–J7 + 左夹爪 = 16 维
```

- `/r1arm/control_sample`：高频控制快照，包括目标、反馈和时序信息。
- `/whole_body/frame`：观测与图像，默认目标 30 Hz。
- 关节为共同模型坐标；夹爪保存逻辑 slide 位置，不能直接作为电机弧度发送。
- 暂停前后以 `record_segment` 区分，不能跨段直接拼接动作。
- 相机时间戳为接收时间，不保证等于曝光时刻；双臂串口并行也不代表硬件同步。

图像文件检查与预览：

```bash
python scripts/check_r1arm_images.py data/dual_real/episode_0000.mcap --preview
```

## 🔄 轨迹回放

回放不需要 PICO 或实时 XR 输入。真机回放前退出原遥操／录制程序，保持所需夹爪服务运行；episode 路径按实际文件替换。

### 1. 只读检查

```bash
python scripts/replay/replay_r1arm_real.py --episode data/right_real/episode_0000.mcap
python dual/replay.py --episode data/dual_real/episode_0000.mcap
```

不加 `--motion-mode` 只检查数据，不驱动真实机械臂，也不播放真机时间轴。

### 2. 真机回放

```bash
# 右臂
python scripts/replay/replay_r1arm_real.py \
  --episode data/right_real/episode_0000.mcap --motion-mode

# 双臂
python dual/replay.py --episode data/dual_real/episode_0000.mcap --motion-mode
```

默认优先使用高频控制流中的整形目标 `q_purpose`；旧记录缺少控制流时回退观测流。`--section state` 使用实测轨迹，`--speed 0.5` 慢放。回放先接近段首帧，暂停分段分别处理，不跨段直接插值。

单臂、双臂真机回放默认在独立进程同步显示录制图像，并逐帧输出关节和夹爪目标。单臂关节字段为 `q_motor_target_rad`，双臂为 `right.q_rad` / `left.q_rad`，均是限速后下发的模型坐标（rad）；夹爪字段 `gripper_slide` 为记录空间的 slide 目标。双臂还输出 `grippers_enabled`，表明是否真的启用夹爪。在 VS Code 按 `Ctrl+Shift+P`，执行 `Simple Browser: Show`，粘贴终端打印的 `http://127.0.0.1:端口` 地址；远程 SSH 时先在 Ports 面板转发该端口。MCAP 中有三路图像时显示三视角，缺少的视角不会补齐。加 `--preview-backend window` 可改用本地 OpenCV 窗口；`--no-print-actions` 关闭逐帧终端输出，`--no-show-observations` 关闭预览。浏览器预览通过终端 Ctrl+C 停止；OpenCV 窗口中 q / Esc 也可停止回放。接近、限速或反馈检查失败时先检查数据与设备，不放宽保护强行播放。

双臂纯预览（不打开串口或 DDS、不需要启动夹爪服务）：

```bash
python dual/replay.py --episode data/dual_real/episode_0000.mcap --preview-only
```

此模式会先启动预览并打印网址，打开 VS Code Simple Browser 后回到终端输入 `MOVE` 才开始播放图像和左右 action。默认按录制速度播放，可用 `--speed` 调整。浏览器会将三个视角横向排列，并按窗口大小等比缩放；`--preview-scale 0.5` 可进一步缩小画面。终端标记 `mode=preview`；真机模式标记 `mode=command`，并区分 `approach`、`playback`、`hold` 阶段。`--preview-only` 与 `--motion-mode` 互斥；两者都不加仍只检查数据。真机双臂默认启用夹爪，先在另一终端运行 `python -m dual.grippers --side both`；没有夹爪时用 `--no-enable-dex1`。

### 3. MuJoCo 中查看记录

```bash
python scripts/replay/replay_r1arm_sim.py --episode data/dual_real/episode_0000.mcap
```

此命令不驱动真机。`--no-viewer` 关闭机器人窗口，`--no-show-observations` 关闭记录图像预览。

## 🧑‍✈️ 参数与控制说明

下表为真机遥操／录制的主要默认值；完整参数以各入口 `--help` 为准。

| 参数 | 默认值 / 行为 |
|---|---|
| 控制频率 `--rate` | 90 Hz 目标频率 |
| 观测频率 `--frequency` | 录制入口默认 30 Hz |
| `--kp` / `--kd` | 60 / 10；支持七轴分别指定 |
| `--max-joint-speed` | 1 rad/s |
| 双臂 `--max-joint-acceleration` | 6 rad/s² |
| 整形器位置 / 速度增益 | 64 / 16 |
| `--max-command-lead` | 0.15 rad；与步长限制冲突时优先步长，不是总力矩上界 |
| 重力前馈 | 默认开，`--tau-clip 8` 逐关节限幅；不等于电机总力矩限制 |
| `--scale-factor` | 1.0 |
| `--home-on-start` | 默认开；共同模型 home 为 `0,1.2,0,-1.2,0,0,0` |
| `--reset-blend-frames` | 60，名义最短时间按 frames/50 Hz 计算；实际仍受速度和反馈约束 |
| 双臂 `--home-recovery-margin` | 0.25 rad，仅允许有限初始超限恢复 |
| `--baud` | 6000000 |
| `--feedback-timeout` / `--foc-timeout` | 启动反馈 3 s / FOC 1 s |
| `--enable-dex1` | 默认开；`--no-enable-dex1` 显式关闭 |
| `--grip-torque-limit` | -1，关闭额外夹持力矩锁定 |

启动检查、模型限位、反馈故障检查、指令步长和超前限制均应保留。双臂每侧有独立模型、标定和控制状态，串口事务并行完成；实际控制频率取决于通信和主机负载。

## 🩺 常见问题

| 现象 | 检查方法 |
|---|---|
| 安装完成但仍找不到 SDK / 标定 | 明确设置 `R1ARM_SDK_ROOT` 再 source 环境脚本；setup 不采集实体零位 |
| `Missing/inaccessible` | 查看实际 by-id、串口组权限与设备连接；更换转接器需更新序列号和全部端口变量 |
| 路径 `Accessible` 但电机无回复 | Accessible 仅说明路径和权限；继续查供电、线缆、通道角色、ID、转接器和占用 |
| `Port is already used` | 退出原控制器或服务；不要用多个版本争用同一总线 |
| 夹爪 `motor id=7 does not reply` | 先定位失败侧，核对夹爪供电、线缆、ID 与串口；DDS 网卡不决定串口探测是否回复 |
| 双夹爪一侧启动后两侧退出 | 启动器在单侧失败时清理另一侧；查看最先报错的一侧，不代表两侧同时损坏 |
| `lo is not multicast-capable` | 本机 lo 配置中的已知提示；以电机识别和 DDS 持续反馈判断运行情况 |
| `No Dex1 feedback ...state` | 查服务是否存活、left/right topic、domain 与 DDS 网卡是否一致 |
| `missing joints [1,...,6]` | 列表使用零起始索引，对应 ID1～6 / J2～J7；先查反馈通信，再检查标定，不盲目改 ID 或重标定 |
| J6 等角度超限 | 核对原标定、真实参考姿态与编码器基准；相差约一圈也不能未经核对自动取模 |
| PC Service 已连接但手柄不跟踪 | 确认头显唤醒、应用前台、控制器发送已开启、位姿及时间戳持续更新，再查网络 |
| `CXXABI` / `GLIBCXX` / 动态库缺失 | 激活正确 Conda 环境、加载 `real_env.sh`；核对 SDK、bridge 和 Dex1 库路径，确认架构一致 |
| 模块缺失 | 确认使用 r1_teleop 而非 base；若缺少模块，检查 setup.sh 是否成功完成 |
| 相机缺失导致双臂不能启动 | 纯遥操先用 `--no-cameras`，逐路验证后再启用图像；停止占用相机的其他程序 |
| mock 初始化与真机日志同时出现 | 双臂初始化左侧临时使用 mock，随后共用右侧 XR；单独一行 `Using mock XR input` 不代表最终没有真实输入 |

一次排查只改变一个条件。通信恢复不等于完整运动验收；依次确认反馈、标定、归位、单侧跟踪、双侧跟踪、夹爪开合，最后再增加相机和录制负载。

## 机械臂通信检查
通信预检由独立程序执行，不由遥操／录制自动调用；遥操／录制保留原有的反馈就绪和故障检查，不再要求连续 10 轮反馈。只读 replay 和 MuJoCo 仿真不需要执行真机通信预检。先完成本 README 的环境初始化，退出所有机械臂遥操／录制／回放程序，可靠支撑机械臂，再运行：

# 双臂预检（默认）
python scripts/check_r1arm_communication.py

# 只检查右臂或左臂，选择其中一条
python scripts/check_r1arm_communication.py --side right
python scripts/check_r1arm_communication.py --side left
每侧依次执行：全轴停止 → 等待 100 ms → 单独探测 ID 0–6（每 ID 连续 10 轮新反馈）→ 七轴连续 10 轮新反馈。探测按 50 Hz 的等待间隔执行，每个阶段最多等待 3 秒；中间缺反馈会重新计数。程序检查反馈计数增量，避免把曾经收到过反馈的 ready 标志当作本轮新反馈。故障码、反馈 timeout 或非有限值会使预检失败，不自动清故障。

显示全部 [PASS] 并正常退出后，可在同一终端启动前面的遥操、录制或回放命令。程序退出时会停止电机并关闭串口，不保持姿态。预检通过说明检查期间通信正常，不能保证重新打开串口后或后续运行中不再出现间歇问题。

本机 bridge 已重建。首次在其他构建目录／机器使用新增的单 ID 探测接口，或出现 Bridge lacks single-ID probing／Bridge lacks feedback diagnostics 时，需要重建实际使用的 bridge。默认本机构建目录的命令如下（环境初始化后执行）：

python scripts/build_r1arm_sdk_bridge.py --sdk-root "$ARM" --build-dir build/dual_sdk_bridge
source scripts/real_env.sh
预检失败会以非零状态退出，应先排查失败的 ID／故障码；不要在预检未通过时继续运动。无论检查通过、失败还是按 Ctrl+C 中断，程序都会尝试停止电机并关闭串口。

程序默认使用环境中的 PORT_RIGHT/PORT_LEFT、CALIB_RIGHT/CALIB_LEFT 和 bridge；也支持 --right-port/--left-port、--right-calib/--left-calib、--bridge-lib。预检与遥操应使用相同的串口、SDK 和 bridge。

需要单独检查右臂 ID 1 时，在没有遥操／回放进程占用串口的情况下运行 SDK 诊断程序：

"$ARM/bin/test_arm_lowlevel" \
  --probe-id 1 \
  --port "$PORT_RIGHT" \
  --baud 6000000 \
  --calib "$CALIB_RIGHT"
确认出现 Feedback ready from motor ID 1，且 Bus replies 持续增长、Bus timeouts 保持为 0；按 Ctrl+C 退出。该命令只验证 ID 1 的探测链路，不代表七轴都正常；全轴连续通信由上面的独立预检程序验证。探测期间机械臂不会主动保持，请可靠支撑。

连接问题先按下面顺序排查，避免把通信问题当作标定问题：

## 🧪 离线回归

```bash
python -m unittest discover -s tests
python -m unittest dual.tests.test_dual
```

不连接真实电机的回归不能替代硬件验收。部分双臂测试会读取 `R1ARM_SDK_ROOT` 下已有的左右标定文件；全新机器应先准备配置和文件。

## 🗂️ 项目结构

```text
assets/                         机械臂 URDF、网格和 MuJoCo 场景
robot/                          SDK bridge 封装、设备参数和 Dex1 客户端
teleop/                         XR 输入、IK、控制、相机和 MCAP
scripts/teleop.py               统一真机／仿真遥操与录制入口
dual/                           复用的机械臂控制、录制、双臂回放与标定实现
native/                         C++ bridge 与依赖适配补丁
third_party/                    固定版本第三方 submodule
tests/、dual/tests/              离线回归
mp4/                            演示 GIF 与设备照片
```

快速上手见 [简洁版 README](README.md)，双臂方向及数据布局见 [dual/README.md](dual/README.md)，依赖来源及补丁见 [native/dependency_patches/README.md](native/dependency_patches/README.md)。

## 🙏 致谢

感谢 [XRoboToolkit](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)、[XRoboToolkit Python binding](https://github.com/YanjieZe/XRoboToolkit-PC-Service-Pybind)、[R1-7A SDK](https://github.com/unitreerobotics/r1-7a_sdk)、[Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)、[Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)、[Dex1 服务](https://github.com/unitreerobotics/dex1_1_service)、[Placo](https://github.com/Rhoban/placo) 与 [MuJoCo](https://github.com/google-deepmind/mujoco) 等项目。

页面结构参考 [UniArmL1](https://github.com/unitreerobotics/UniArmL1)，硬件、控制入口、数据格式及功能说明以本仓库代码为准。

## 📄 许可证

本仓库当前未提供根目录 `LICENSE` 文件；第三方 SDK、模型、运行库及依赖的许可分别以其原项目声明为准。
