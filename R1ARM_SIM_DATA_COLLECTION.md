# R1Arm MuJoCo 遥操、MCAP 采集与回放

本文说明如何在 R1Arm 单臂或双臂 MuJoCo 仿真中使用 PICO 遥操、采集一条与
`r1a7_teleop` 对齐的 MCAP 数据，并在仿真中回放检查。

## 1. 环境准备

进入项目并激活环境：

```bash
cd /home/unitree/mjn_workspace/r1arm_teleop
conda activate r1_teleop
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
```

有桌面时直接使用 MuJoCo viewer。无显示器或通过 SSH 运行时，添加
`--no-viewer`，并建议使用 EGL：

```bash
export MUJOCO_GL=egl
```

采集脚本不连接真机、不依赖 R1 Arm SDK 的 `libmotor_controller.so`，只使用
MuJoCo、XRoboToolkit 和本项目的 IK/遥操代码。

## 2. 只做仿真遥操

连接 PICO：

```bash
python scripts/teleop_r1arm_sim.py
```

不连接 PICO，使用自动生成的手柄轨迹：

```bash
python scripts/teleop_r1arm_sim.py --mock-xr
```

默认控制右臂。右手柄侧握 `grip` 后移动手柄，机械臂才会跟踪；右手柄
`trigger` 控制 Dex1 开合。普通遥操脚本中 A 键用于软复位。

双臂仿真：

```bash
python scripts/teleop_r1arm_sim.py --use-dual-arm
```

## 3. 使用 PICO 采集 MCAP

### 单臂（默认右臂）

```bash
python scripts/record_r1arm_sim.py \
  --output-dir /home/unitree/datasets/r1arm/pick_demo \
  --camera-views head_left wrist_right \
  --task-description "pick an object" \
  --robot-id r1arm_sim
```

默认只采集一个头部视角和一个腕部视角。默认值位于
`scripts/record_r1arm_sim.py` 开头：

```python
DEFAULT_CAMERA_VIEWS = ("head_left", "wrist_right")
```

也可以直接通过命令行覆盖；相机数量等于 `--camera-views` 后提供的名称数量：

```bash
# 只录一个头部视角
python scripts/record_r1arm_sim.py --output-dir <目录> \
  --camera-views head_left

# 一个头部 + 一个左腕视角
python scripts/record_r1arm_sim.py --output-dir <目录> \
  --camera-views head_right wrist_left

# 完整四视角
python scripts/record_r1arm_sim.py --output-dir <目录> \
  --camera-views head_left head_right wrist_left wrist_right
```

可选名称为 `head_left`、`head_right`、`wrist_left`、`wrist_right`，同一名称不能
重复。视角列表及数量会写入 episode 元数据的 `camera_views` 和 `camera_count`。
用于同一个训练数据集的 episode 应保持相同视角列表；如果旧目录里已有四视角
文件，建议为新的两视角数据使用一个新目录，避免训练时特征数量不一致。

### 单臂（左臂）

```bash
python scripts/record_r1arm_sim.py \
  --side left \
  --output-dir /home/unitree/datasets/r1arm/left_arm_demo
```

### 双臂

```bash
python scripts/record_r1arm_sim.py \
  --use-dual-arm \
  --output-dir /home/unitree/datasets/r1arm/dual_arm_demo
```

采集时的 PICO 按键与 `r1a7_teleop/scripts/record_real.py` 一致：

| 输入 | 功能 |
|---|---|
| Y（左手柄） | 开始采集；再次按下会停止并保存当前 episode |
| B（右手柄） | 暂停/恢复采集，同时暂停遥操输出 |
| A（右手柄） | 与原始遥操一致：机械臂软复位到 home |
| grip | 激活对应一侧机械臂的末端跟踪 |
| trigger | 控制对应 Dex1：松开为开，按下为关 |

推荐操作顺序：

1. 启动脚本，等待终端显示 `ready`。
2. 遥操已经自动启动；握住要控制一侧的 grip，确认仿真机械臂能跟随手柄。
3. 按 PICO Y 开始采集，完成动作后再按 PICO Y。
4. 等待终端出现 `SAVED ... episode_XXXX.mcap` 后再退出。

采集脚本不读取终端键盘，所以在终端输入小写 `x` 或 `y` 不会触发任何功能。
X 键也不再是开始遥操的前置步骤。

`--output-dir` 直接指定保存 episode 的最终目录。例如：

```text
/home/unitree/datasets/r1arm/pick_demo/episode_0000.mcap
/home/unitree/datasets/r1arm/pick_demo/episode_0001.mcap
```

新 episode 会自动接着该目录中已有的最大编号保存，不会覆盖已有文件。如果不传
`--output-dir`，才会使用旧的默认规则：
`data/<task-dir-name>/<日期>_robot_<robot-id>_dex1/`。

停止采集后若出现 `flushing ... queued observation frame(s)`，表示后台仍在渲染
并写入所选视角。等待期间 grip/trigger/A 和机械臂遥操会继续更新，不再冻结；
但必须等到 `SAVED` 后再按 Y 开始下一条，才能保证两个 MCAP 文件完全分离。

## 4. 无 PICO 自动采集测试

以下命令适合先验证整个采集链路。默认右臂的 mock grip 在第 8 秒后才激活，
因此建议至少录 12 秒：

```bash
python scripts/record_r1arm_sim.py \
  --mock-xr \
  --auto-start \
  --duration 12 \
  --exit-after-duration \
  --output-dir /home/unitree/datasets/r1arm/mock_test
```

无显示环境：

```bash
MUJOCO_GL=egl python scripts/record_r1arm_sim.py \
  --mock-xr --no-viewer \
  --auto-start --duration 12 --exit-after-duration \
  --output-dir /home/unitree/datasets/r1arm/mock_headless
```

双臂自动测试只需再加 `--use-dual-arm`。

## 5. 在 MuJoCo 中回放

### 动作回放（默认）

动作回放读取 `actions.action`（`robot_q_desired` 是相同数据的兼容别名），
重新驱动 MuJoCo 位置执行器，适合验证
记录的控制命令能否复现运动：

```bash
python scripts/replay_r1arm_sim.py \
  --episode /home/unitree/datasets/r1arm/pick_demo/episode_0000.mcap
```

### 状态回放

状态回放逐帧设置 `states.joint_state`（`robot_q_current` 是兼容别名），显示的
是记录时的关节观测，轨迹与
采集结果最接近：

```bash
python scripts/replay_r1arm_sim.py \
  --episode <path>/episode_0000.mcap \
  --mode state
```

同时显示 MCAP 内保存的所有已选观测图像（自动适配 1～4 路布局）：

```bash
python scripts/replay_r1arm_sim.py \
  --episode <path>/episode_0000.mcap \
  --mode state \
  --show-observations
```

其他常用选项：

```bash
# 循环、2 倍速回放
python scripts/replay_r1arm_sim.py --episode <path>/episode_0000.mcap --loop --speed 2

# 只回放第 100～300 帧
python scripts/replay_r1arm_sim.py --episode <path>/episode_0000.mcap \
  --start-frame 100 --end-frame 300

# 无窗口快速校验文件与动力学回放
MUJOCO_GL=egl python scripts/replay_r1arm_sim.py \
  --episode <path>/episode_0000.mcap --no-viewer --speed 20
```

回放脚本会从 episode 元数据自动识别单臂/双臂、控制侧和 MJCF 路径，通常不必
再次传 `--use-dual-arm` 或 `--side`。

## 6. 与 r1a7_teleop 对齐的数据格式

每个 MCAP 包含：

| Topic | 内容 |
|---|---|
| `/episode/meta` | `{info, text, config}`，每条 episode 一条 |
| `/whole_body/frame` | 同一采样时刻的图像、状态和动作 |

`info.format` 为 `r1arm_sim_multicamera_mcap_json_v1`。MCAP 的 topic、图像
编码和 state/action 语义沿用 `r1a7_teleop`，但关节向量使用当前仿真的真实动作
空间。每帧包含：

```text
idx
sample_time_ns
image_source_frame_id
colors
states
actions
```

所选 `colors` 均为 base64 JPEG，可选键名为：

```text
color_0_head_left
color_1_head_right
color_2_wrist_left
color_3_wrist_right
```

仿真没有实体相机，因此这些键对应固定 MuJoCo 视角。实际写入哪些键以及它们的
顺序记录在 `/episode/meta.info.camera_order` 中。

单臂状态与动作都是 8 维，不含 body，也不会补一条不存在的机械臂：

```text
J1, J2, J3, J4, J5, J6, J7, dex1_gripper
```

双臂状态与动作都是 16 维，并严格沿用当前 MuJoCo 控制顺序：

```text
right_arm(7), right_gripper(1), left_arm(7), left_gripper(1)
```

每个 Dex1 在 MuJoCo 内有两个对称指关节，但它们接受同一个控制量，因此数据集
只保存一个逻辑夹爪自由度。状态使用两个指关节位置的平均值；回放时再把这一维
复制给两个 MuJoCo 指关节。

| 数据 | 字段 |
|---|---|
| 仿真实测状态 | `states.joint_state` |
| 状态兼容别名 | `states.robot_q_current` |
| 控制目标 | `actions.action` |
| 动作兼容别名 | `actions.robot_q_desired` |
| 拆分字段 | `arms`、`hand_state` 或 `hand_cmd` |

单臂的 `arms` 为 7 维、夹爪为 1 维；双臂的 `arms` 为 14 维、夹爪为 2 维。
具体顺序也会写入 `/episode/meta.config.joint_names`，并由回放脚本校验。

每个采样点先推进遥操和物理仿真，再一次性复制状态、动作、完整 MuJoCo 场景和
时间戳；后台所选图像由该快照渲染。因此图像、`states` 和 `actions` 属于同一
控制时刻，不会因为后台写盘产生错帧。

## 7. 常见问题

- **手柄移动但机械臂不动**：采集脚本启动后遥操已经激活，但必须持续握住对应
  手柄的 grip；只连接 PICO 或在终端输入 `x` 都不会更新机械臂目标。
- **`--mock-xr` 启动后右臂暂时不动**：mock 周期前 3 秒不激活，3～8 秒只激活
  左臂，8～13 秒才激活右臂。
- **没有桌面/报 OpenGL 错误**：使用 `MUJOCO_GL=egl` 和 `--no-viewer`。
- **保存时等待较久**：所选视角在后台渲染，控制采样仍按设定频率执行；停止
  后会把队列完整写完。遥操在此期间仍会继续，但不要在出现 `SAVED` 之前按 Y
  开始下一条。若更重视即时保存，可降低到 `--frequency 10`。
- **终端显示 neutral self collision warning**：这是 IK 模型的自碰撞提示，不是
  MCAP 采集失败；只要随后显示 `ready` 即可继续。
- **想检查 MCAP 和动作维度**：运行
  `python scripts/replay_r1arm_sim.py --episode <path>/episode_0000.mcap --no-viewer --speed 20`。
