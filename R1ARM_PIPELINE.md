# R1Arm 遥操 → 录制 → 回放 完整流程与参数参考

> 代码基线：`/home/ycb410/nt/r1arm_teleop`（2026-10-09，30 Hz 全链定居后）。
> 本文覆盖：从 PICO XR 端 → IK（Placo QP / DLS）→ `arm_q` → SDK 命令 → 电机 → state，以及录制/回放两条链路的**全部实现方式与参数**，含线程模型。

---

## 0. 端到端总览

```
┌─ PICO 头显/手柄 ─────────────────────────────────────────────┐
│ XRoboToolkit PC Service（gRPC，本机 127.0.0.1:60061）        │
│   原生 pybind 模块 xrobotoolkit_sdk（自带后台反馈线程）        │
│   · get_*_pose()  → 7 维位姿 [x,y,z,qx,qy,qz,qw]（~30 Hz 刷新）│
│   · get_time_stamp_ns() → 服务端帧时间戳（帧时钟）             │
│   · get_left/right_grip/trigger()、get_X/Y/A/B_button()       │
└──────────────────────────────────────────────────────────────┘
                        ↓ teleop/xr_client.py（ctypes 预载 .so + 薄封装）
┌─ 控制器基类 R1ArmTeleopControllerBase（teleop/r1arm_controller_base.py）─┐
│ step(dt)：                                                   │
│   ① read_controller_poses()  ← 帧时钟感知 + 跳变拒绝 + One-Euro │
│   ② _track_side()  相对位移 → 末端目标 pos/quat（R_TELEOP_TO_BASE）│
│   ③ _solve() → Placo QP（主）或 DLS（回退）                   │
│   ④ q_cmd/v_cmd/a_cmd 独立轨迹状态机 → self.q                 │
│   ⑤ _update_dex1_side() + _send_dex1_all()  （DDS 夹爪）      │
│   ⑥ _output_command()  → 实机子类覆写                        │
└──────────────────────────────────────────────────────────────┘
        ↓ self.q = 模型坐标（URDF 序，7 臂 + 2 指）
┌─ 真机子类 R1ArmRealTeleopController ──────────────────────────┐
│ arm_q("right") → 7 臂关节（模型坐标）  ← ★ 录制在这里取样       │
│ model_to_sdk_q()（J4 符号镜像）                               │
│ limit_sdk_position_command()  硬限位 + slew + 实测超前限制     │
│ _tau_ff()  重力前馈（pinocchio @ 指令姿态，clip ±8 Nm）        │
│ bus.command_position(cmd, kp, kd, tau=..., dq=dq_target) [v3]  │
│ _reconcile_ik_seed()  命令-IK 目标 gap > 0.05 rad 时重播种     │
└──────────────────────────────────────────────────────────────┘
        ↓ robot/r1arm_sdk.py（ctypes）
┌─ C++ 桥接 libr1arm_sdk_bridge.so ─────────────────────────────┐
│ ArmJointCmd{ mode=1, q, dq_target, kp, kd, tau } × 7         │
│ bus.step(commands)  ← 同步串口往返（6 Mbaud），检查新鲜反馈     │
└──────────────────────────────────────────────────────────────┘
        ↓ /dev/ttyUSB2 @ 6,000,000 baud
┌─ 电机驱动器 ──────────────────────────────────────────────────┐
│ FOC 位置环：tau = kp·(q_target − q_meas) + kd·(dq_t − dq) + tau_ff │
│ mode 0=停机 / 1=FOC 位置环（无独立力矩模式）                    │
└──────────────────────────────────────────────────────────────┘
        ↑ read_state() → q / dq / tau_est / mode / timeout / error_code / ready
        ↑ sdk_to_model_q() → 模型坐标 → 录进 states.joint_state
```

---

## 1. PICO XR 端

**文件**：`teleop/xr_client.py`（薄封装）、`robot/xr_interface.py`（`get_key_value`、`make_xr_client`）

- **初始化**：`preload_native_lib()` 用 `ctypes.CDLL(..., RTLD_GLOBAL)` 预载
  `third_party/XRoboToolkit-PC-Service-Pybind/lib/libPXREARobotSDK.so`，再 `import xrobotoolkit_sdk`；`XrClient.__init__` 调 `_xrt.init()` 连接本机 gRPC 服务（日志 `initialize sdk,connect127.0.0.1:60061`）。
- **原生模块自带后台线程**：日志 `watch server feedback thread start / client start server stream` —— 反馈流在 C++ 侧独立线程，JSON 解析后写入加锁的全局变量（`py_bindings.cpp`：`HeadsetPose` / `LeftControllerPose` / `RightControllerPose` / `TimeStampNs`，各配 mutex）。
- **Python 侧接口**：
  - `get_pose_by_name(name)` → `headset` / `left_controller` / `right_controller`，返回 7 维 `[x,y,z,qx,qy,qz,qw]`
  - `get_frame_stamp_ns()` → `int(self._xrt.get_time_stamp_ns())`，失败返回 `None`
  - `get_key_value_by_name()` → `left/right_trigger`、`left/right_grip`（float 0..1）
  - `get_button_state_by_name()` → X/Y/A/B、菜单键、摇杆按下
- **刷新率 ≈ 30 Hz**（服务端推送节奏）；`TimeStampNs` 每收到一帧更新一次。
- **坐标系**：PICO 原始坐标经 `R_HEADSET_TO_WORLD` 转世界坐标，再经 `R_TELEOP_TO_BASE`（`robot/r1arm_config.py`，A7 系 `+X前/+Z上` → URDF `−Y前/+Z上`）转到臂基座系。

---

## 2. 控制器基类：控制环与跟踪

**文件**：`teleop/r1arm_controller_base.py`
**主循环**（`run(rate)`）：`while running: t0=now; step(1/rate); sleep(1/rate − elapsed)` —— 周期从**循环顶**计时，耗时（IK + 串口）不累积。

### 2.1 `step(dt)` 分支顺序

1. `_reset_blending` 中 → `_step_reset_blend()` + `_output_command()` + return
2. `read_controller_poses()` 返回 invalid → `_track_side(side,{})` 请求状态机平滑刹停，再 `_output_command()` + return
3. A 键触发 → `reset()` + blend
4. `_track_side(side)` 逐侧跟踪，IK 解进入 per-side `q_cmd/v_cmd/a_cmd` 状态机
5. `_update_dex1_side()` + `_send_dex1_all()`
6. `clamp_configuration()` → `_output_command()`

### 2.2 `read_controller_poses()` —— 帧时钟感知 + One-Euro（2026-10-09）

```python
stamp = self.xr_client.get_frame_stamp_ns()
if stamp is not None and stamp == self._xr_last_stamp:
    return self._xr_pose_cache, True      # 服务端没出新帧 → 沿用上拍位姿
... 读三路位姿 → 保存 raw → _reject_xr_jumps() → timestamp-aware One-Euro（位置 + quaternion SLERP）→ 写缓存 + 记 stamp
```
- **动机**：XR 服务端 ~30 Hz 推帧、控制环 30/50 Hz 轮询；不感知帧时钟时同一帧被重复消费且节拍不均（33 ms 与 20 ms 不成整数比）→ IK 目标成为「阶梯 × 采样」的拍频调制信号。
- **退化**：`stamp is None`（API 不可用）→ 每拍照读，等同旧行为。
- 位姿无效时：`allow_xr_fallback=False`（真机模式）→ 返回 `({}, False)` hold；否则回退 mock 正弦。

### 2.3 `_reject_xr_jumps()` —— 跳变拒绝 + 搬位识别

| 参数 | 值 | 含义 |
|---|---|---|
| `_XR_JUMP_POS_M` | 0.30 m | 单帧位移超此值判跳变 |
| `_XR_JUMP_ROT_RAD` | 1.0 rad (57°) | 单帧转动超此值判跳变 |
| `_XR_JUMP_MAX_REJECTS` | 10 | 连续拒绝达此数 → 判为**真实搬位**，接受新位姿 |

行为：跳变帧 hold 上一有效位姿 + 清跟踪锚（`st.clear()`，grip 重锚会在下一拍重建相对参考）；连续 10 帧偏离则接受新位置。噪声量级（mm）远低于阈值，不会误触发。

### 2.4 `_track_side()` —— 相对位移跟踪

- `st.active = get_grip_state(side) > 0.9`（握 grip 才跟踪）
- 首次激活（`ref_ee_pos is None`）：记录 `ref_ee_pos / ref_ee_quat`（当前末端）、`ref_controller_pos / ref_controller_rot`（当前手柄）、`ref_heading_yaw`；并调 `solver.set_regularization_reference(q)` 把 Placo 姿态正则参考重锚到当前姿态。
- 之后：`delta_pos = ctrl_pos − ref_controller_pos`（按参考 yaw 旋转到基座系）→ `target_pos = ref_ee_pos + delta_pos_base × scale_factor`；姿态同理用 rotvec 增量。
- 松 grip / 位姿缺失 → 清跟踪锚并保持当前目标；重新握住时从当前姿态重新锚定。

### 2.5 `_trajectory_step()` —— 唯一 q/v/a 连续轨迹状态机

```python
q_ik_raw = Placo/DLS(raw solution)
q_cmd, v_cmd, a_cmd = trajectory_state(q_cmd, v_cmd, a_cmd, q_ik_raw, actual_dt)
q_motor = SDK_safety_limit(q_cmd, measured_q, previous_motor_q)
dq_target = diff(q_motor, previous_motor_q) / actual_dt
```

- **唯一运动整形状态**：每侧独立维护 `q_cmd/v_cmd/a_cmd`；Placo 与 DLS 的输出都只作为目标，动态约束只在此状态机执行。默认 `v_max = trajectory_max_speed_rad_s = 1.0 rad/s`、`a_max=6 rad/s²`、`jerk_max=80 rad/s³`，使用实际 top-to-top `dt`。
- 轨迹器用临界阻尼位置/速度反馈 `a_desired=clip(16·error−8·v_cmd, ±a_max)`，再施加加速度、jerk 和速度约束；避免仅按距离计算制动速度导致离散越点极限环。grip 松开时每周期以当前轨迹位置为停止目标；重新按下及目标反向都保留 q/v/a 状态。reset/home 共用同一轨迹器，并要求各受控臂都同时满足位置、速度、加速度阈值后才退出。
- SDK slew/lead/hard-limit 仅作安全兜底，正常情况下应不触发；状态机的 `q_purpose` 与最终 `q_motor_target` 分开记录。若 safety limiter 长期触发，说明 command/state 脱节，须先排查而不是把差值当普通训练 action。
- **限位**：**无**软着陆逻辑（2026-10-09 删除）；只依赖 ① Placo 的 `enable_joint_limits` ② 循环末尾的 `clamp_configuration()` 投影 ③ SDK 层的 `bounded = clip`。

### 2.6 复位与 blend

- `reset()`：把 `ARM_INIT` 作为共享轨迹目标；不额外插入 smoothstep cross-fade。
- `_step_reset_blend()`：在目标到达且速度降为零后结束 reset 并复位 Dex1 夹爪。
- `reset_blend_frames = 60` → 1.2 s。

---

## 3. Placo QP IK（主求解器）

**文件**：`teleop/r1arm_placo_solver.py`，`PlacoArmIKSolver`

### 3.1 构造（`__init__`）

```python
placo_robot = placo.RobotWrapper(urdf_path)
solver      = placo.KinematicsSolver(placo_robot)
solver.dt   = 1 / control_rate_hz      # ★ 与实际控制周期对齐
solver.mask_fbase(True)                 # 锁浮基座
solver.enable_joint_limits(True)        # QP 内关节限位约束
for name in ARM_JOINTS:                 # P2 源头驯服
    placo_robot.set_velocity_limit(name, joint_velocity_limits.get(name, 1.0))
solver.enable_velocity_limits(True)
```

**任务表**：

| 任务 | 类型 | 配置 | 作用 |
|---|---|---|---|
| `effector_task` | frame task @ `dex1_base_link` | `("ee", "soft", arm_task_weight=1.0)` | 末端位姿跟踪 |
| `m_task` | manipulability task | `("ee_manip", "soft", 1e-2)` | 远离奇异 |
| `hold_task` | joints task（Dex1 关节） | `("hold", "soft", 1.0)` | 夹爪关节保持 |
| `regularization_task` | joints task（7 臂关节） | `("joints_regularization", "soft", 1e-2)` | 拉向参考姿态 |

### 3.2 `solve(q, target_pos, target_quat)`

1. `placo_robot.state.q[7:] = q` → `update_kinematics()`
2. 组目标 `T`（旋转用 `target_quat`，缺省沿用当前末端朝向）→ `effector_task.T_world_frame = T`
3. `hold_task.set_joints(当前 Dex1 关节值)`、`regularization_task.set_joints(self._default_arm_joints)`
4. `solver.solve(True)` → 失败 `RuntimeError` → 返回 `None`（触发 DLS 回退）
5. 返回 `placo_robot.state.q[7:]`（URDF 序）

`set_regularization_reference(q)`：跟踪启动时把正则参考从「启动姿态」改为「当前姿态」。

---

## 4. DLS 回退求解器

**文件**：`teleop/ik_solver.py::solve_upper_arm_ik`（阻尼最小二乘，pinocchio 雅可比）
**触发**：Placo 返回 `None`（求解失败/异常）。

默认参数：`position_weight=1.0`、`orientation_weight=1.0`、`elbow_weight=1.0`、
`regularization_weight=0.05`、`damping=0.02`、`damping_max=0.5`、
`manipulability_threshold=0.05`、`joint_limit_margin=0.15`、`joint_limit_weight=2.0`、
`max_step=0.03`、`iterations=100`。

语义：原始 DLS 解同样只作为目标输入独立 q/v/a 轨迹状态机，速度/加速度连续性不会在 QP→DLS 时重置。

---

## 5. 真机输出链（`teleop/r1arm_real_controller.py`）

### 5.1 启动序列（`_on_start`）

```
bus.open(port, 6 Mbaud, calib)
  → _wait_for_feedback()        零力矩探测（probe_once），等 7 轴 ready，查故障码
  → _seed_model_from_feedback() self.q 从实测播种 + set_regularization_reference()
  → _enable_foc_hold()          零增益 → 提增益到 command_sdk = 实测姿态（误差≈0）
  → home_on_start ? reset()     软 blend 回 ARM_INIT
  → enable_dex1   ? Dex1Single(...)  DDS 发布/订阅
```

### 5.2 `_output_command()`（每拍）

```python
state        = bus.read_state()
target_sdk   = model_to_sdk_q(self.arm_q())            # ← 整形后的 IK 输出转 SDK 系
command      = limit_sdk_position_command(target_sdk, self.command_sdk, state.q,
                                          max_delta=self.max_delta,
                                          max_lead=self.max_command_lead)
bus.command_position(command, self.kp, self.kd, tau=self._tau_ff(command))
self.command_sdk = command ; self.last_state = state
self._reconcile_ik_seed(target_sdk, command)
```

### 5.3 `limit_sdk_position_command()`（SDK 层限幅）

```python
slew_interval = [previous - max_delta, previous + max_delta] ∩ hard_limits
lead_interval = [measured - max_lead, measured + max_lead]
command       = clip(target, slew_interval ∩ lead_interval)
```
- 若 measured 与 previous 已分离到两区间无交集，则单周期优先保持硬限位和 slew，逐周期追近实测值；这种反馈/命令脱节不能在一个周期同时满足两个限制。
- `max_delta = max_joint_speed / rate_hz`（默认 1.0 / 30 = 0.0333 rad/拍）
- `max_lead = 0.15 rad` —— 命令相对**实测**的最大超前量；它同时是力矩上界的第一项。
- **力矩三项上界**（脚本打印用）：`kp×lead + kd×max_speed + tau_clip` = 80×0.15 + 2×1.0 + 8 = **22.0 Nm**（电机峰值 34.4 Nm 的 64%）。

### 5.4 `_tau_ff()` —— 重力前馈

```python
q_full = zeros(model.nq)                  # URDF nq = 9（7 臂 + 2 Dex1 指）
q_full[:7] = sdk_to_model_q(command_sdk)
tau = pin.computeGeneralizedGravity(model, data, q_full)[:7]
return clip(model_to_sdk_q(tau), ±tau_clip)      # J4 符号镜像 + clip ±8 Nm
```
- 在**指令姿态**计算（非实测 → 无反馈环）。
- 实测 4 µs/调用，对 ±0.001 rad 命令噪声的力矩抖动 0.03 Nm（可忽略）。
- 补偿 URDF 模型重力；腕上相机质量不在 URDF → 只补 ~65–70%。

### 5.5 `_reconcile_ik_seed()`

命令与 IK 目标 `gap > 0.05 rad` 时，把 `self.q` 对应关节拉回**已下发命令**（`sdk_to_model_q`），防止内部目标跑在实机前面；blend 段跳过（q 由 blend 公式逐拍重写）。

---

## 6. C++ 桥接层

**文件**：`native/r1arm_sdk_bridge/r1arm_sdk_bridge.cpp`（编译产物 `build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so`）

```cpp
struct Bridge { ArmMotorBus bus; };
// 每个命令周期组装 7 个 ArmJointCmd
commands[i].mode = 1 ;           // FOC 位置环
commands[i].timed_out = false;
commands[i].q   = q[i];
commands[i].dq  = dq[i];         // v3 期望速度，与相邻位置目标差分一致
commands[i].tau = tau ? tau[i] : 0.f;
commands[i].kp  = kp[i]; commands[i].kd = kd[i];
bus.step(commands);              // 同步串口往返
// 之后逐关节检查 lastStepFeedback()，缺失则抛 "missing fresh feedback from Jx"
```

**导出 ABI**：
- `r1arm_bridge_version()` → 版本串
- `r1arm_bridge_create/destroy`
- `r1arm_bridge_open(handle, port, baud, calib, err, cap)`
- `r1arm_bridge_probe(handle, ...)` —— 零力矩探测
- `r1arm_bridge_command_position(handle, q, kp, kd, err, cap)` —— **v1 广播 ABI**
- `r1arm_bridge_command_position_v2(handle, q, kp[7], kd[7], tau[7]|NULL, err, cap)` —— **v2 逐关节 + 力矩前馈**
- `r1arm_bridge_command_position_v3(handle, q, dq, kp[7], kd[7], tau[7]|NULL, err, cap)` —— **v3 逐关节位置/速度目标 + 力矩前馈**
- `r1arm_bridge_read_state(handle, q, dq, tau_est, mode, timeout, error_code, ready, err, cap)`
- `r1arm_bridge_stop(handle, ...)` / `r1arm_bridge_close(handle, ...)`

**路由规则**（`robot/r1arm_sdk.py::command_position`）：仅位置标量命令 → v1；kp/kd/tau → v2；提供 dq 时 → v3。

**标定**：`joint_calib_candidate.txt`（每行 `joint dir scale offset`，例：`j0 dir=-1 scale=1 offset=4.18628`）。

---

## 7. 电机底层

- **串口**：`/dev/ttyUSB2 @ 6,000,000 baud`（实测 `read_state` + `command_position` 往返 ≈ **7.3 ms**，p95 9.4 ms）。
- **控制模式**：`mode` 字段只有 0 = 停机、1 = FOC 位置环；**无独立力矩模式**——力矩是位置环的输出：
  `tau_motor = kp·(q_target − q_meas) + kd·(dq_target − dq_meas) + tau_ff`
  当前实现使用 bridge v3 下发 q/dq 目标；期望速度是受安全限幅后的命令位置差分，不是测得的电机速度。`kd` 项基于 `dq_target - dq_meas`，应低速、托臂条件下逐步验证，不能仅凭接口存在就假设整机已调好阻抗。
- **故障码**：`error_code`（实测 0x8 = 瞬态欠压，曾在回放降速不当的爬行段触发）；`timeout` 标志位。
- **硬件规格**（厂商）：

| 关节 | 电机 | 峰值扭矩 | 最大速度 | 运动范围 |
|---|---|---|---|---|
| J1/J2 | 6010 | 34.4 Nm | >180°/s (3.14 rad/s) | −156~156° / 0~180° |
| J3/J4/J5 | 6014 | 34.4 Nm | >180°/s | −20~200° / 0~180° / −130~130° |
| J6/J7 | 5010 | 31.7 Nm | >180°/s | −30~30° / −90~90° |

- **实测负载**（保持段 `tau_est`）：J2 ≈ −2.5 Nm，J4 ≈ +5.3 Nm，J6 ≈ +1.25 Nm。

---

## 8. 反馈与 state

`read_state()` → `R1ArmSdkState(q, dq, tau_est, mode, timeout, error_code, ready)`（均 7 维 float32/uint8/uint32）。

- **`q`**：SDK 坐标；经 `sdk_to_model_q()`（乘 `MODEL_TO_SDK_SIGN = [1,1,1,−1,1,1,1]`，仅 J4 取反）转模型坐标后进 `states.joint_state`。
- **`dq` / `tau_est`**：SDK 原生；用于诊断与遥测。
- 录制时 state 取 `controller.last_state`（最近一次 50/30 Hz 读到的反馈）。

---

## 9. 录制链路

**文件**：`scripts/record_r1arm_real.py`（1145 行）

### 9.1 线程模型（关键）

| 线程 | 名字 | 频率 | 职责 |
|---|---|---|---|
| **主线程** | — | `--rate` (30 Hz) | 控制环 `step()` / `_output_command()`、按键处理、`_publish_preview_state()`（轻量拼图 + 原子发布）、`--duration` 判定 |
| **录制线程** | `r1arm-real-mcap-record` (daemon) | `--frequency` (30 Hz) | 相机快照 + 读取控制线程原子发布的最新 `ControlSample` + MCAP 写入 |
| **预览线程** | `r1arm-preview` (daemon) | `--preview-hz` (12 Hz) | `namedWindow` / `imshow` / `waitKey(1)`（Qt 事件泵 + q/ESC 退出）；等待期 10 ms 粒度 sleep |
| **相机 worker** ×N | `camera-<key>` (daemon) | 设备帧率 | 抓帧 → 解码 BGR → 加锁写 `_frame` + `_received_at_ns`；内容冻结检测 |

另：XR 原生模块内部的 gRPC 反馈线程（C++ 侧，非 Python 线程）。

### 9.2 录制循环（`_record_loop`）

```
interval = 1/frequency
while not stop:
    if not is_recording or paused: sleep 5ms; continue
    if now < next_record_tick: sleep min(剩余, 5ms); continue
    while next_record_tick <= now: next_record_tick += interval      # 补拍，不漂移
    frames, frame_id, received_at_ns, _ = camera.get()
    if frames is None or not fresh(received_at_ns):  每秒一次 stale 警告 → continue
    states, actions = _state_action()
    cam_times = camera.frame_times()                                  # P3
    with writer_lock: writer.add_frame(frames, states, actions,
                                       image_source_frame_id=frame_id,
                                       sample_time_ns=time.time_ns(),
                                       camera_frame_times_ns=cam_times)
```

**`_state_action(sample)`**：读取同一个 `control_seq` 的原子控制快照，不再独立读取可变的 `controller.q` 和 `last_state`。
- `actions.q_purpose` / 兼容别名 `actions.action` = 整形后的 IK 关节意图（模型坐标，7 轴 + 夹爪），训练主 action。
- `actions.q_ik_raw` = 该周期的原始 Placo/DLS 关节解（7 轴）。
- `actions.q_motor_target` = 经坐标转换、硬限位、slew、lead 后实际送给 SDK 的 7 轴位置目标（模型坐标）；`dq_target` 是最终位置命令的实际周期差分。
- `actions.dq_purpose` / `ddq_purpose` = 独立轨迹状态机的期望速度/加速度；`xr_raw` / `xr_filtered` 可用于分析输入滤波影响。
- `actions.ee_intent` = robot_base 下目标末端位置和 xyzw 四元数。
- `states.joint_state`、`dq`、`tau_est` = 同一 `control_seq` 读取的实测值；state 反馈时间单独记在 `feedback_time_ns`。
- 夹爪：`_gripper_state_command()` → 有 Dex1 时读 **DDS 真值**（`hand.get_record_data()` 的 `hand_state`/`hand_cmd`，经 `_dex1_open_ratio_to_sim` 转 sim 值）；无 Dex1 时回退仿真开度（**该回退会在保存时被质检点名**）
- 拼 8 维 `[7 臂 + 夹爪]`，各含 2 份手指槽位 → `adapter.frame_data()` 生成 states/actions 字典

### 9.3 MCAP 格式（`teleop/mcap_writer.py`）

**两个 topic**：

| topic | schema | 频率 |
|---|---|---|
| `/episode/meta` | `r1_a7.episode_meta` | 1 条 |
| `/whole_body/frame` | `r1_a7.four_camera_frame` | 每帧 |

**`/episode/meta`** = `{info, text, config}`：
- `info`：`format`（`r1arm_real_multicamera_mcap_json_v1`）、`created_at`、`frequency_hz`、`camera_order`、`camera_positions`
- `text`：`goal`（任务描述）
- `config`：`robot`、`eef`、`simulator`、`motion_mode`、`teleop_stack`、`dual_arm`、`control_side`、`urdf_path`、`serial_port`、`camera_keys`、`camera_source_streams`、`camera_inactive`、`camera_devices`、`mcap_color_keys`、`joint_names`（8）、`state_dim`/`action_dim`（8）、`state_layout`、`sim_q_layout`、`teleop_rate_hz`、`teleop_max_joint_speed`、`teleop_max_command_lead`、`teleop_kp`/`kd`（标量或 null）、`teleop_kp_list`/`kd_list`（恒 7 值）、`teleop_gravity_ff`、`teleop_tau_clip`、`ik_max_acc_per_s2`、`trajectory_max_speed_rad_s`、`ik_qp_velocity_limit`、**`field_contract`**

**`field_contract`**（P3，转换器据此选字段）：
```
train_action  = actions.q_purpose  # 整形后 IK 意图；actions.action 是兼容别名
q_ik_raw      = actions.q_ik_raw   # 原始 IK 解，7 维
q_motor       = actions.q_motor_target # 最终 SDK 位置目标，7 维
ee_intent     = actions.ee_intent  # 末端目标位姿，robot_base / quaternion_xyzw
train_state   = states.joint_state # 同 control_seq 的电机实测
diagnostics   = control_seq + feedback_time_ns + camera_frame_times_ns + dq/tau_est
frame         = "model"
j4_sign       = "negative (URDF convention)"
gripper_unit  = "sim value +0.0245 open / -0.02 closed"
```

**`/whole_body/frame`** 每条：

| 字段 | 说明 |
|---|---|
| `idx` | 从 0 递增 |
| `sample_time_ns` | 写入时刻（`time.time_ns()`，UTC epoch ns） |
| `image_source_frame_id` | 相机帧计数（最小值，用于识别重复帧） |
| `camera_frame_times_ns` | `{camera_key: 到达时刻 ns}`（P3 对齐） |
| `colors` | `{key: {encoding:"jpeg", height, width, channels, data:<base64>}}` |
| `states` | `joint_state`（8）、`robot_q_current`（同值）、`arms`（7）、`hand_state`（1） |
| `actions` | `action`（8）、`robot_q_desired`（同值）、`arms`（7）、`hand_cmd`（1） |

`log_time` = `publish_time` = `sample_time_ns`。

### 9.4 保存时质检（6 项，全部打印）

1. **action smoothness** — 逐关节 `|v|p95 / |a|p95 / |a|max`（30 Hz 差分）；`a_p95 > 25` 警告
2. **gripper** — 常数时点名两种成因（未启用 Dex1 → 0.0245 是仿真回退值；已启用但 trigger 没动）
3. **相机 inactive** — 本 run 丢失的相机
4. **相机 frozen** — 内容冻结的相机
5. **fps** — `frames/duration < 90% × frequency` 警告，点名最慢相机
6. **断点 / 对齐** — `pause breakpoints`（B 键暂停→恢复的帧号，chunk 切分点）+ `camera alignment（末帧）`（各路图像相对 sample 的偏移 ms）

---

## 10. 回放链路

**文件**：`scripts/replay_r1arm_real.py`

### 10.1 加载与参数推导

1. 读 episode → `arm_traj` / `gripper_traj` / `intervals`（由 `sample_time_ns` 差分，异常间隔用 `1/frequency_hz`）
2. `freq = args.frequency or recorded_hz`；`peak_speed` = 轨迹最大关节速度
3. **slew cap**：
   - `--section action` → 读取已整形的 `actions.q_purpose`；不做二次轨迹整形，仍保留 SDK 安全 slew/lead/关节限位
   - `--section state` → `max(peak×1.2, 1.0)`，上限 `--auto-speed-cap`（5.0）
4. `use_ff` = `--gravity-ff` / `--no-gravity-ff`，缺省读 `config.teleop_gravity_ff`（旧 episode 无字段则默认开）
5. 回放 action 时不串 `ReplayTrajectoryShaper`；逐周期按最终安全位置目标的有限差分计算 `dq_target` 并下发 SDK v3

### 10.2 执行

```
open bus → wait_for_feedback → enable_foc_hold(tau=tau_ff)
→ approach(arm_traj[0])                        独立限速 --approach-speed，走 SDK 层限幅
→ play loop（逐帧）：
      target_sdk = model_to_sdk_q(frame)
      cmd = limit_sdk_position_command(target_sdk, command_sdk, measured,
                                       max_delta = max_joint_speed/freq, max_lead=0.15)
      dq = (cmd - command_sdk) / actual_command_dt
      bus.command_position(cmd, kp, kd, tau=tau_ff(cmd), dq=dq)
      CSV: q_purpose / q_motor_target / dq_target / q,dq,tau_est / tracking errors
      command_gripper(gripper_traj[i])                        # 需 --enable-dex1
      按 sample_time_ns 调度（gap > --max-frame-gap 压缩）
→ approach(arm_traj[-1])  收敛
→ hold 最后姿态 --hold-time 秒（带 FF）
→ safe_stop
```

### 10.3 与录制端的参数继承

| 回放参数 | 来源 |
|---|---|
| `--kp` / `--kd` | 默认 80/2（不自动继承；episode 里存了 `teleop_kp_list` 备查） |
| 重力前馈 | `config.teleop_gravity_ff` |
| slew cap | `config.teleop_max_joint_speed` |
| 控制环 | `--rate`（默认 30，与 30 Hz episode 1:1） |
| 回放遥测 | 每轮 CSV 记录 q_purpose、q_motor_target、dq_target、实测 q/dq/tau_est 与误差 |

---

## 11. 参数总表

### 11.1 硬件 / 通信

| 项 | 值 |
|---|---|
| 串口 | `/dev/ttyUSB2`（默认脚本里是 `/dev/ttyUSB0`，实际用 2） |
| 波特率 | 6,000,000 |
| 标定 | `/home/ycb410/nt/R1_arm_SDK_x86/r1_arm/config/joint_calib_candidate.txt` |
| 桥接库 | `build/r1arm_sdk_bridge/libr1arm_sdk_bridge.so` |
| SDK root | `/home/ycb410/nt/R1_arm_SDK_x86/r1_arm` |
| URDF | `assets/r1arm/R1_Robotic_Arm.urdf`（nq = 9） |
| Dex1 网口 | `enp4s0`（DDS topic `rt/dex1/right/cmd|state`） |

### 11.2 关节定义

| 项 | 值 |
|---|---|
| 关节名 | `J1..J7` + `dex1_gripper_joint`（8 维） |
| `MODEL_TO_SDK_SIGN` | `[1, 1, 1, −1, 1, 1, 1]`（**仅 J4 取反**） |
| `ARM_INIT`（模型） | `[0, 1.2, 0, −1.2, 0, 0, 0]` → SDK `[0, 1.2, 0, 1.2, 0, 0, 0]` |
| `ARM_LIMITS`（模型） | J1 ±2.723 · J2 0~3.1416 · J3 −0.349~3.491 · **J4 −3.1415~0** · J5 ±2.269 · J6 ±0.524 · J7 ±1.571 |
| 末端 frame | `dex1_base_link` |
| 遥操坐标变换 | `R_TELEOP_TO_BASE`（A7 `+X前/+Z高` → URDF `−Y前/+Z高`） |
| Dex1 sim 值 | 开 `+0.0245` / 闭 `−0.02`；`DEX1_MAX_RANGE = 5.0`；`DEX1_SIM_SPEED = 0.08`；trigger 死区 `0.05` |

### 11.3 控制与整形（`robot/r1arm_config.py` + 脚本默认）

| 参数 | 默认 | 说明 |
|---|---|---|
| `--rate` | **30.0 Hz** | 控制环（IK + 执行） |
| `--frequency` | **30.0 Hz** | 录制采样 |
| `--kp` | **80.0** | 位置环增益（可逐关节 `80,80,80,60,...`） |
| `--kd` | **2** | 阻尼增益（可逐关节） |
| `--gravity-ff` | **on** | 重力前馈（`--no-gravity-ff` 关） |
| `--tau-clip` | 8.0 Nm | 前馈单关节限幅 |
| `--max-joint-speed` | 1.0 rad/s | SDK 层 slew（→ `max_delta = 1/rate`） |
| `--max-command-lead` | 0.15 rad | 命令相对实测的超前上限 |
| `trajectory_max_speed_rad_s` | 1.0 | 轨迹整形器速度上限（rad/s，直接使用，不依赖控制频率） |
| `ik_max_acc_per_s2` | 6.0 | IK 加速度上限（换算 `×50/rate` → 30 Hz 实际 **10 rad/s²**） |
| `ik_qp_velocity_limit` | `None` | QP 内关节速度上限（关闭；设置数值才启用） |
| `ik_joint_velocity_limits` | `None` | 逐关节覆盖，如 `{"J5": 0.8}` |
| `--ik-dt` | 0.0 | 覆盖 Placo `solver.dt`（0 = `1/rate`；当前 QP 速度限制关闭时不改变 raw IK 解） |
| `--reset-blend-frames` | 60 | A/home 复位时长（wall-clock，`frames/50` 秒 = 1.2 s） |
| `arm_task_weight` | 1.0 | Placo ee 任务权重 |
| `--trace / --debug` | off | 每秒打印实测/命令/误差 |

### 11.4 录制

| 参数 | 默认 | 说明 |
|---|---|---|
| `--record-button` | `X` | 开始/停止并保存 |
| `--pause-button` | `B` | 暂停/恢复（录制 + XR 跟踪） |
| `--enable-a-reset` | off | 允许 A 键软回 home |
| `--camera` | `DEFAULT_CAMERAS` | `key=...,backend=v4l2\|realsense,device=...,size=WxH,fps=N,fourcc=...` |
| `--camera-wait` | 8.0 s | 相机解析超时 |
| `--max-image-age` | 0.5 s | 图像新鲜度门限（超过则本拍跳过） |
| `--image-show` | `None` | auto：有 `DISPLAY` 即开；`--no-image-show` 关 |
| `--preview-hz` | 12.0 | 预览渲染频率 |
| `--image-preview-scale` | 0.75 | 预览缩放 |
| `--enable-dex1` | off | 启用 DDS 夹爪录制 |
| `--duration` | 0.0 | >0 时自动停止（秒） |
| `--auto-start` | off | 启动即开录 |
| `--exit-after-duration` | off | 到时长后退出 |

**相机默认 spec**：`color_0_head_left` = v4l2 `match=SN0002`, 640×480@30 MJPG；`color_3_wrist_right` = realsense, 640×480@30。

### 11.5 回放

| 参数 | 默认 | 说明 |
|---|---|---|
| `--section` | `action` | `action`（已整形 q_purpose）/ `state`（实测位姿） |
| `--rate` | 30.0 | 控制/接近环 |
| `--speed` | 1.0 | 时间轴倍率 |
| `--frequency` | 0.0 | 播放帧率（0 = 用 episode 的 `frequency_hz`） |
| `--control-hz` | 0.0 | >0 时线性插值到更密控制格；默认按原始采样时刻回放 |
| `--max-joint-speed` | `None` | None = 按 section 自动推导 |
| `--auto-speed-cap` | 5.0 | state 模式自动 cap 上限 |
| `--approach-speed` | 1.0 | 接近段关节速度 |
| `--approach-tol` | 0.05 rad | 到位容差 |
| `--approach-timeout` | 10.0 s | |
| `--max-frame-gap` | 2.0 s | 长间隔压缩 |
| `--hold-time` | 2.0 s | 末姿态保持 |
| `--telemetry-csv` | auto | 每轮回放遥测 CSV；默认在源 episode 旁生成唯一文件 |
| `--gravity-ff` / `--no-gravity-ff` | `None` | 缺省读 episode 元数据 |
| `--loop` | off | 循环回放 |
| `--start-frame` / `--end-frame` | 0 / −1 | 片段 |

### 11.6 内部常量

| 常量 | 值 | 位置 |
|---|---|---|
| XR 跳变阈值 | 0.30 m / 1.0 rad | `r1arm_controller_base.py` |
| XR 连续拒绝上限 | 10 帧 | 同上 |
| IK 种子回锚 gap | 0.05 rad | `r1arm_real_controller.py` |
| 相机稳定性窗口 | 1.5 s / ≥3 帧 | `local_camera_client.py` |
| 相机冻结判定 | 连续 60 帧内容相同 | 同上 |
| 相机预设 | 640×480, fps 30 | 同上 |
| `_ERROR_CAPACITY`（bridge 错误串） | 512 | `r1arm_sdk.py` |
| `JOINT_COUNT` | 7 | 同上 |
| 抓取力矩锁存 | `grip_torque_limit = −1.0`（**关闭**） | `dex1_single.py` |
| DDS 反馈读循环 | `sleep(0.002)` ≈ 500 Hz | 同上 |

---

## 12. 线程 / 进程清单

| 进程 | 线程 | 数量 | 说明 |
|---|---|---|---|
| `teleop_r1arm_real.py` | 主控制环 | 1 | 30 Hz `run()` 循环 |
| | XR 原生反馈线程 | 1（C++） | gRPC 流，写加锁全局位姿 |
| | XR 其它 | 若干（C++） | `client/server stream`、`watch feedback` |
| `record_r1arm_real.py` | 主控制环 | 1 | `step()` + 按键 + 预览发布 |
| | 录制线程 | 1 | 30 Hz 快照 + MCAP 写（`writer_lock`） |
| | 预览线程 | 1 | 12 Hz `imshow` / `waitKey` |
| | 相机 worker | 1/相机 | 抓帧 + 解码 + 冻结检测（每 worker 独立 `threading.Lock`） |
| | XR 原生反馈线程 | 1（C++） | 同 teleop |
| `replay_r1arm_real.py` | 主控制环 | 1 | 回放 + 接近段 |
| | XR | 无 | replay 不用 XR |
| `trajectory_test_r1arm.py` | 主环 + 遥测写线程 | 2 | 遥测走 `queue` + 后台 CSV 线程 |
| `bench_r1arm_ik.py` | 主环 | 1 | MuJoCo 后端时含 viewer 渲染 |

**共享状态与锁**：
- `record.writer_lock` — 保护 `writer.add_frame` / `start_episode` / `stop_episode`
- 相机 worker `_lock` — 保护 `_frame` / `_received_at_ns` / `frames_seen`
- 预览状态 — **无锁**，靠「控制线程整体替换 dict 引用 + 渲染线程读旧引用」的原子性
- 遥测 — `queue.Queue(maxsize=4096)`，满则丢行（不阻塞控制环）

---

## 13. 频率体系（三个 Hz 的含义）

| 频率 | 环节 | 代码 | 关系 |
|---|---|---|---|
| **~30 Hz** | XR 服务端推送帧 | 原生 SDK | 新帧才读入并滤波；无新帧时复用上次位姿 |
| **`--rate` 30 Hz** | 控制环（IK + 执行 + 命令） | `controller.run(rate)` | 每拍：取新/缓存 XR → tracking/IK → 整形 → 命令 → 反馈；因此高于 XR 频率时会对缓存位姿重复求解 |
| **`--frequency` 30 Hz** | 录制采样上限 | `_record_loop` | 读取新的原子控制样本；一帧绑定一个 `control_seq`，重复序号不写入 |

录制端按新的 control_seq 采样：每条样本把该控制周期的 action、最终命令和反馈绑定在一起，不重复写入同一个控制周期；相机仍按最近快照采样，并保存各路到达时间用于对齐分析。
（回放端 `--rate` 默认也是 30，与 30 Hz episode 1:1。）

---

## 14. 已知行为与限制（供使用时参考）

| 现象 | 原因 | 现状 |
|---|---|---|
| 电机 30 Hz 阶梯比 50 Hz 粗 | 每拍步长 0.033 vs 0.02 rad | 设计取舍；`--rate 30` 是当前选择 |
| 快速甩停后有短暂残余晃动 | 阶跃瞬态 + kp 追赶（非自持振荡） | 实测 <0.8 s 自行衰减 |
| 录像中夹爪偶发 2 s 不响应 | Dex1 DDS 网络拥塞（`hand_cmd` 动、`state` 冻） | **未修** |
| 腕部图像与 state 有 ~0–30 ms 偏移 | 相机曝光/传输延迟 | 已可测（`camera_frame_times_ns`） |
| 快速段回放比示教慢 ~25% | 30 Hz 采样 + 整形器速度削顶 | 已知；终审实验量化 |
| 悬垂残余 ~30–35% | URDF 无腕部相机质量 | 位置环承担 |
| 退出时 Qt `killTimer` 告警 | 预览线程窗口在主线程关闭 | 无害 |
