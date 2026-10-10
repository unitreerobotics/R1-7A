# R1Arm 项目变更史（2026-09-20 → 2026-10-09）

> 范围：`/home/ycb410/nt/r1arm_teleop`（真机遥操 / 录制 / 回放 / 执行层运控）。
> 记录方式：9/23 之前从仓库提交与文件时间推断（**非第一手日志**，标注来源）；9/28 之后为本会话实测记录。
> 统计（截至 2026-10-09，全部未提交）：**13 个已跟踪文件 +1529/−211**，**7 个新文件共 3944 行**，另 `R1ARM_REAL_MOTION_OPTIMIZATION_PLAN.md`。

---

## 一、时间线

### 2026-10-10 · 真机记录拆分 100 Hz 控制流与 30 Hz 图像观测流

- 真机 teleop 默认控制频率设为 100 Hz；录制 `--frequency` 仍为 30 Hz，分别表示控制周期和带图像的 observation 周期。
- MCAP 新增 `/r1arm/control_sample`，按完整 `control_seq` 保存 `q_purpose`、raw IK、电机位置/速度目标、实测关节位置/速度/力矩、XR 和末端诊断；不依赖相机成功出帧。
- 保留 `/whole_body/frame` 的旧结构与 30 Hz 采样，继续兼容现有 replay；meta 新增控制频率与高频 topic 说明。
- B 暂停/恢复间通过 `record_segment` 切分，保存时汇报样本数、序号缺口和周期统计。chunk 转换仍需按图像时间戳对齐，不能把 30 Hz frame 当成高频 chunk。
- 验证限于软件格式/单测；100 Hz 在真机上的控制周期余量和运行稳定性仍需由硬件实测确认。

### 2026-10-10 · 按用户要求撤销 XR cutoff 试验值

- 更正此前回退目标：XR One-Euro 默认恢复为用户指出的调参前基线 `min_cutoff=2.0 Hz, beta=0.15`（不是 8.0/1.0）；轨迹器 16/8 及其他参数保持不变。

### 2026-10-10 · 恢复轨迹器温和响应并选用 XR 滤波试验基线

- 将基线 `trajectory_position_gain/trajectory_velocity_gain` 从 20/9 恢复为 16/8；两者仍保持近临界阻尼关系，预期响应带宽略低，需由用户真机手感确认是否减少抖动及是否增加滞后。
- XR One-Euro 默认设为 `min_cutoff=5.0 Hz, beta=1.0`：作为 cutoff 5 与 8 之间的保守试验基线。已有不同手速 episode 中 raw/filtered 位姿差异较小，数据不足以证明它是全局最优值。
- 同步更新 R1Arm 配置、真机遥操/录制 CLI 默认值及 README/Pipeline；速度、加速度、jerk、电机增益与 SDK 限幅保持不变。

### 2026-10-10 · 整形器速度上限改用 rad/s

- 用 `trajectory_max_speed_rad_s` 替换 50 Hz 标定语义的 `ik_max_step_per_cycle`；整形器直接读取 rad/s，不再乘 50。实时遥操与录制入口未显式传 `--trajectory-max-speed` 时沿用 `--max-joint-speed`，录制 metadata 保存直接速度值。
- 保持默认 30 Hz 与 `1.0 rad/s` 数值不变；单测确认速度限制不随控制频率换算。

### 2026-10-09 · 增加 direct-IK 手感对照入口

- 新增 `teleop_r1arm_real_unshaped.py`：active Placo 结果直接进入 SDK；QP 速度限幅设为 `--max-joint-speed`，DLS 每周期只迭代一步并按相同速度限制步长。
- 仍保留 SDK 硬限位/slew/lead，以及 home/reset、松 grip 刹停的轨迹过渡；active tracking 不做独立加速度/jerk 整形。此实验版未做真机验证。

### 2026-10-09 · 修复 home/reset 目标附近振荡

- 固定目标时将“剩余距离限速”改为临界阻尼位置/速度反馈，之后仍受同一速度、加速度和 jerk 限制；补充固定 home 目标的收敛测试。
- reset 完成条件同时检查位置误差、速度和加速度；双臂模式要求两侧均停稳，避免尚在运动时恢复 grip 跟踪。
- 单元测试覆盖数值收敛；尚未真机回归，需托臂并保持急停可触及后低速验证。

### 2026-10-09 · 遥操连续性与 Dex1 输入修正

- SDK 命令每周期在 hard-limit、previous±slew、measured±lead 的可行区间内裁剪；区间无交集时保持 hard-limit/slew 并逐周期追近反馈，避免旧 limiter 在 lead clip 后突破 slew。
- 轨迹器统一推进 `q_cmd/v_cmd/a_cmd`，真实 `dt` 下限制速度、加速度和 jerk；松开 grip 刹停、重新 grip、目标反向及 reset/home 都保持状态连续。
- 每周期独立更新 XR trigger 与 Dex1 DDS，不再依赖机械臂 pose/IK 有效；录制样本附带 trigger 有效性/年龄、Dex1 命令与反馈开度及估算力矩，便于定位夹爪不响应。
- 修复尚未经过真机回归；首次运行仍需托住机械臂并保持急停可触及。

### 阶段 0 · 9/20–9/24 — 基础栈打通（来源：git log + 文件 mtime）

提交历史：

| 日期 | 提交 |
|---|---|
| 09-04 | first commit |
| 09-05 | 实现r1arm在仿真端的摇操测试 |
| 09-24 | Update R1 arm teleoperation and SDK support |

本阶段文件（09-23/24 mtime，均已包含在 9/24 提交或其后）：

- **C++ 桥接**：`native/r1arm_sdk_bridge/r1arm_sdk_bridge.cpp` + `scripts/build_r1arm_sdk_bridge.py`
  —— ctypes 可加载的 `libr1arm_sdk_bridge.so`，封装 `ArmMotorBus`（串口 6 Mbaud、标定文件、7 关节命令/反馈）。
- **仿真链**：`scripts/teleop_r1arm_sim.py`、`record_r1arm_sim.py`、`replay_mcap.py`、
  `teleop/r1arm_mujoco_controller.py`、`robot/mujoco_sim.py`。
- **IK 两套**：`teleop/ik_solver.py`（自写 DLS）、`teleop/placo_solver.py` / `r1arm_placo_solver.py`（Placo QP）。
- **控制器基类**：`teleop/r1a7_controller_base.py`（A7 双爪版）→ 后来派生出 `teleop/r1arm_controller_base.py`。
- **配置**：`robot/r1arm_config.py`（URDF/MJCF 路径、关节顺序、限位、`ARM_INIT`、遥操坐标变换 `R_TELEOP_TO_BASE`）。
- **数据集/MCAP**：`teleop/mcap_writer.py`、`teleop/r1arm_dataset.py`（帧格式校验与适配器）。

### 阶段 1 · 9/24–9/28 — 真机相机链路诊断与修复

**9/24**：新增 `scripts/probe_r1arm_cameras.py`（211 行）——枚举 `/dev/video*`、解析 `/dev/v4l/by-id` 稳定路径、V4L2 三档格式试抓、librealsense bgr8/yuyv 各重试 4 次、打印可粘贴的 `--camera` spec、RealSense 的 USB 2.x 警告。`scripts/replay_r1arm_sim.py` 加入（回放仿真 episode）。

**9/28**：新增 `scripts/view_r1arm_cameras.py`（133 行，实时预览）与 `teleop/local_camera_client.py`（620 行，**本地相机订阅层**）。

**诊断结论（影响后续全部设计）**：
- D405 腕部相机挂在 USB 2.0 链路上时**一帧都出不来**（bgr8/yuyv 各 4 次尝试全 0 帧）；移到 USB 3.x 口后 60 fps 稳定。
- 历史数据里 USB 2.0 上 D405 的两种死法：被启动稳定性窗口判死静默剔除；或通过窗口但**内容冻结**（13 帧同一张图，流还活着）→ 需要按内容检测。

`local_camera_client.py` 因此包含：每相机一个 worker 线程、环形缓冲、`_STABILITY_WINDOW_S = 1.5` / `_STABILITY_MIN_FRAMES = 3` 的启动稳定性窗口、`_FROZEN_AFTER = 60` 帧的内容冻结检测（8×6 灰度缩略图均值比较）、`health()` / `frame_ages()`。

### 阶段 2 · 9/29 — 真机录制 / 回放管线落地

新增 `scripts/record_r1arm_real.py`（现 1145 行）与 `scripts/replay_r1arm_real.py`（现 802 行）。

- **录制**：PICO 摇操 + 两路本地相机 → 与仿真格式兼容的 MCAP（`/episode/meta` + `/whole_body/frame`）。PICO 按键 X 开始/停止、B 暂停/恢复、A 软回 home（需 `--enable-a-reset`）。录制线程独立于控制环，按 `--frequency` 节拍抓「相机快照 + 当前 state/action」。
- **回放**：从 `sample_time_ns` 恢复帧间隔；接近段独立限速（`--approach-speed`）；action/state 两种模式；MOVE 手输确认；停机兜底。
- **IK 轨迹质量**（`r1arm_controller_base.py` / `r1arm_config.py`）：
  - `_clamp_ik_step()`：求解器输出每周期步长限制（`ik_max_step_per_cycle`，保证录进 action 的轨迹物理可执行）。
  - `set_regularization_reference()`：跟踪启动时把 posture 参考重锚到当前实际姿态，消除首帧跳变。
  - `reset_blend_frames`：A 键回 home 的线性 cross-fade。
- **格式兼容**：`mcap_writer.py` / `r1arm_dataset.py` 区分 `camera_order=None`（默认四相机）与显式 `()`（纯 state/action episode，跳过相机校验）；`replay_r1arm_sim.py` 支持无相机 episode。
- 产出第一条真机数据 `episode_0000.mcap`（17 s / 512 帧 / 30 Hz / 37 MB）。

### 阶段 3 · 9/30 — 执行层增益与重力前馈

- **C++ bridge v2 ABI**（`r1arm_sdk_bridge.cpp` +41 行）：
  `r1arm_bridge_command_position_v2(handle, q[7], kp[7], kd[7], tau[7]|NULL, err, cap)`
  —— 逐关节增益 + 力矩前馈通道；保留「命令周期必须有新鲜反馈」检查。
- **`robot/r1arm_sdk.py`**（+99 行）：`parse_gain` / `format_gain` / `as_gain_array`；
  `command_position` **智能路由**（标量且无 tau → 旧广播 ABI；任一是数组或带 tau → v2）；
  `joint_array` 先广播后校验（修掉标量 kd + 数组 kp 的混合 bug）。
- **下游接线**：`r1arm_real_controller.py` 参数类型放开为 `float | np.ndarray`；四个脚本 `--kp/--kd` 换 `parse_gain`；录制元数据新增 `teleop_kp_list/kd_list`。
- **重力前馈进 goto**（`goto_r1arm_pose.py`，+95 行）：`gravity_feedforward()` 用 pinocchio `computeGeneralizedGravity` 在**指令姿态**计算（非实测，无反馈环），URDF nq=9 补零填 7 关节、取前 7 项 tau、经 `model_to_sdk_q` 做 J4 符号镜像、clip ±`--tau-clip`。
- **实测结论**（真机）：悬垂误差严格 ∝ 1/kp；kd 对稳态零影响；FF 把 J4 悬垂 0.065 → 0.014 rad；残余 35–40% = 腕部 D405+SN0002 相机质量不在 URDF 里。

### 阶段 4 · 10/7 — 优化清单与回放插值

- 撰写 `R1ARM_REAL_MOTION_OPTIMIZATION_PLAN.md`（P0/P1/P2/P3 分级清单）。
- `replay_r1arm_real.py` 加入 `interp_trajectory()` + `--control-hz`：把 30 Hz 录制轨迹重采样到更密的控制格（段内均匀 tick、**段端点必达**、时间轴总时长与隐含速度严格不变、gap 存 per-command 增量）。**实测结论：100 Hz 插值不改变电机噪声**（步长假设排除）。
- `solver.dt` 与 `--rate` 对齐（`r1arm_placo_solver.py` 的 `control_rate_hz` 参数）：Placo 把 frame task 误差当速度积分，dt 与实际周期不一致时等效增益错误。

### 阶段 5 · 10/8 — P0 / P1 / P2 全栈实施

**P0（固定轨迹隔离运控问题）** — 新增 `scripts/trajectory_test_r1arm.py`（684 行）：

- level 0 原位保持 / 1 单关节 / 2 三关节相位错开 / 3 提速；quintic smoothstep 起停（v=a=0）、驻留 + 方向反转；`--home-first` 默认开（折叠位贴限位时先经同一链路爬回 home）。
- 逐周期异步遥测 CSV（`q_ref/dq_ref/ddq_ref/q_cmd/q/dq/tau_est/tau_ff/slew_hit/lead_hit/busy_ms/cycle_ms`）+ 逐关节统计报告（周期 p50/p95/max、静差、跟踪 RMS/P95、互相关滞后、限幅触发率）。
- **实测**：串口 `read+command` ≈ 7.3 ms；50 Hz 周期 p95 20.0 ms；**100 Hz 只剩 ~3 ms 余量 → 50 Hz 定案**。
- **增益矩阵九组**（level 2 + FF）：默认 80/2（J4 RMS 0.019、静差 0.016、滞后 120 ms、拉入过冲 1.24×）；安静档 C2（只软 J4：`80,80,80,60,80,80,80 / 2,2,2,3,2,2,2`，过冲 0.81×、听感最小）；精度档 100/2；排除 100/4（听感略响）、腕软 40–50（J6 滞后 240–300 ms）、全臂 60、肩肘加 kd。
- **FF 三重验证**：J4 RMS 0.063 → 0.019。

**P1（IK 目标可执行性）** — `r1arm_controller_base.py` / `r1arm_real_controller.py`：

- `--ik-dt` 接入 record 脚本（此前解析后未传）。
- XR 单帧跳变检测 `_reject_xr_jumps`：位置 > 0.30 m 或姿态 > 57° 判跳变 → hold 上一有效位姿 + 清跟踪锚；**连续 ≥10 帧偏离判为真实搬位**（手柄放下/挪动），接受新位置（真机实测修掉的 bug：原先把手柄放下会永久 hold + 每帧刷日志）。
- `_reconcile_ik_seed`：命令与 IK 目标 gap > 0.05 rad 时把 `self.q` 拉回已下发命令（防止内部目标跑在实机前面）；blend 段抑制误触发。
- reset blend 改**单调时钟**计时长（帧计数在 rate≠50 或周期抖动下不再等于标称秒）。
- Placo 失败 → DLS 回退语义明确（同样过步长钳制，最坏按限速爬行）。

**P2（输出整形与电机控制）** — `r1arm_config.py` / `r1arm_controller_base.py` / `r1arm_placo_solver.py`：

- **IK 层速度/加速度整形器**（`_clamp_ik_step` 升级）：`v_cmd = clip(v_prev ± a_max·dt, v_target, ±v_max)`，替代硬速度钳制——实测把 action 流 jerk p95 从 ~7000 rad/s³ 压到 ≤ a_max。
- **QP 源头驯服** `ik_qp_velocity_limit` + `ik_joint_velocity_limits`：`set_velocity_limit(name, v)` + `enable_velocity_limits(True)`，solver 原始 a_max 219 → 52 rad/s²；action 流更「原生」，整形器退为兜底。
- **限位软着陆**（多轮迭代后于 10/9 **全部删除**，见阶段 6）。
- **preview 移出控制线程**：新增 `r1arm-preview` 专职渲染线程（12 Hz，`--preview-hz`），控制环只做轻量拼图 + 原子发布 —— 修掉「record 比 teleop 抖」（GUI/GIL 竞争挤爆 20 ms 周期）。
- `--max-command-lead` 默认 1.0 → **0.15**（力矩三项合计 `kp×lead + kd×|dq| + tau_clip` 上界报告的由来）；`--gravity-ff`、`--tau-clip` 进 teleop/record；replay 移植 FF + `ReplayTrajectoryShaper`。
- episode 保存时**质检三表**：action smoothness（逐关节 a_p95/a_max，>25 警告）、gripper 通道（常数点名两种成因）、相机 inactive/frozen。
- 新增 `scripts/bench_r1arm_ik.py`（349 行）：测控制环各环节耗时、倍频扫描找 IK 频率上限、`--log` 录波（逐拍 arm_q / solver 原始意图 / 仿真实测 → 平滑度统计）。

### 阶段 6 · 10/9 — 30 Hz 全链定居 + P3

- **XR 输入滤波 + bridge v3 速度前馈**：跳变拒绝之后、相对位姿映射之前加入按 XR 帧时间戳推进的 One-Euro 位置/姿态滤波（姿态走 SLERP，可调 `--xr-filter-min-cutoff` / `--xr-filter-beta`）；采集帧同时写入原始和滤波后手柄位姿。C ABI v3 新增 `dq[7]`，实时遥操把最终安全限幅后的 SDK 位置目标按实际周期差分，作为与同一目标轨迹匹配的 `dq_target` 下发；MCAP 保存 `actions.dq_target`。原 v1/v2 ABI 保留兼容。已本机构建 bridge；尚未连接机械臂实测。
- **修复 home 后无 grip 自运动**：旧 jerk 整形器的制动距离/越点处理不完整，home 后可能越点反追形成往返极限环。现按剩余距离计算制动速度、越点即落在目标；未按 grip 时用状态机停车，停稳后维持停止目标，不再硬同步到上一条 SDK command。另恢复 7 关节标定文件完整性校验。

**30 Hz 基线**：
- `--rate` 默认 50 → **30**（teleop / record / replay）；`--reset-blend-frames` 100 → 60（blend 按 wall-clock 计秒）。
- `ik_max_step_per_cycle` 澄清为 **50 Hz 标定语义**（0.02 → 任意 rate 恒 1.0 rad/s）；`ik_max_acc_per_s2` 8 → 6。
- **XR 帧时钟感知**（`xr_client.get_frame_stamp_ns()` + `read_controller_poses` 缓存）：服务端 `TimeStampNs` 未变则沿用上一拍位姿 —— 消除「30 Hz 阶梯 × 50 Hz 采样」的拍频调制；stamp 不可用时退化为旧行为。
- **限位软着陆全部删除**（margin 公式、sqrt 刹停曲线、`room/dt` 硬界、`_arm_joint_limits` 辅助、`ARM_LIMITS` import）—— 关节限位完全交回 Placo `enable_joint_limits`（QP 内约束）+ SDK 层 `bounded = clip`。

**P3（数据可用性）**：
- **字段契约入元数据** `field_contract`：`train_action = actions.action`（模型坐标系、J4 负号、夹爪 sim 单位 +0.0245 开 / −0.02 闭）、`train_state = states.joint_state`。
- **相机帧时刻入帧** `camera_frame_times_ns`：逐路 worker 到达时刻（`time.time_ns()` 基准），保存时打印末帧各路偏移 —— 图像-action 偏移从假设变可测。
- **暂停断点标记** `_breakpoints`：B 键暂停/恢复处记帧号 + 时刻，保存时打印「chunk 应在第 N 帧切开」。
- 新录制命令的 6 项质检：smoothness / gripper / 相机 inactive / frozen / fps<90% / 断点 / 末帧对齐。

**本阶段实测（扒数据）**：
- 30 Hz 基线 episode（`hz30_baseline`，1432 帧 / 47.7 s）：action 流零单帧跳变、a_p95 ≈ 10（= a_max 上限）；state 侧 22 帧 >0.04 rad 的「跳」——**根因是 30 Hz 阶梯变粗（0.033 vs 50 Hz 的 0.02 rad）+ kp 追赶**，与限位无关（发生在 J4 = −0.73~−1.16，离模型限位 0.0 很远）。
- 7.5 Hz 停住振荡诊断：今天数据中残余振荡是**衰减瞬态**（<0.8 s 归零），非持续振荡；1.5 rad/s 时代因能量 ×2.25 而明显。
- 修复中抓到一个真 bug（已随软着陆删除而消失）：`k_note` **软着陆刹停曲线未封顶 v_max**，30 Hz 下在 margin 边缘解出 1.33 rad/s 超上限。

### 阶段 7 · 10/9 — 控制样本同步与统一轨迹连续性

- **原子控制样本**：真机控制线程每次完成 SDK 输出后发布带 `control_seq`、命令/反馈时刻、IK 意图、最终电机目标和同周期反馈的快照。录制线程只消费新序号，不再分别采样共享的 `controller.q` 与 `last_state`。
- **训练 action 契约**：新增 `actions.q_purpose`（整形后的 IK 关节意图，8 维，训练主 action）；保留 `actions.action` 作为兼容别名。并保存 `q_ik_raw`（7 维）、`q_motor_target`（7 维）、`ee_intent`（robot_base 目标位置 + xyzw 姿态）、`control_seq`、反馈时间、`dq` 和 `tau_est`。
- **独立 q/v/a 状态机**：Placo/DLS 解、grip 松开减速、重新接入与 home/reset 共用每关节轨迹状态；速度、加速度及 jerk 限制按真实 top-to-top dt。最终电机目标只更新 IK solver seed，不回写 `q_purpose` 轨迹。
- **录制质检**：动作导数使用有符号速度、控制时间戳和非均匀采样周期计算 acceleration/jerk，避免等速换向被绝对值差分漏检。
- **验证状态**：Python 静态编译和 bridge 本机构建通过；未进行真机 A/B，滤波延迟与速度目标效果需托臂低速测试。

---

## 二、逐文件改动清单（相对 HEAD `9cd1caf`）

### 已跟踪文件

| 文件 | 改动 | 要点 |
|---|---|---|
| `native/r1arm_sdk_bridge/r1arm_sdk_bridge.cpp` | +41 | v2 ABI（逐关节 kp/kd + tau 前馈） |
| `robot/r1arm_sdk.py` | +99/−? | `parse_gain`/`format_gain`/`as_gain_array`；`command_position` 双 ABI 路由；混合增益广播修复 |
| `robot/r1arm_config.py` | +21 | `ik_max_step_per_cycle`、`ik_max_acc_per_s2`、`ik_qp_velocity_limit`、`ik_joint_velocity_limits`（含 30 Hz 定居注释） |
| `teleop/r1arm_controller_base.py` | +225 | IK 速度/加速度整形器、XR 帧时钟感知 + 跳变检测、`_reconcile_ik_seed`、单调时钟 blend、（软着陆已删） |
| `teleop/r1arm_placo_solver.py` | +24 | `control_rate_hz`（dt 对齐）、QP 速度上限 |
| `teleop/r1arm_real_controller.py` | +77 | 逐关节增益类型、`_tau_ff`、`_reconcile_ik_seed`、`ik_dt` |
| `teleop/xr_client.py` | +14 | `get_frame_stamp_ns()` |
| `teleop/mcap_writer.py` | +18 | `camera_frame_times_ns`、`camera_order` 空元组语义 |
| `teleop/r1arm_dataset.py` | +6 | `expected_camera_keys` None/() 语义 |
| `scripts/goto_r1arm_pose.py` | +95 | `--gravity-ff` / `--tau-clip`、力矩三项上界报告 |
| `scripts/teleop_r1arm_real.py` | +68 | 逐关节增益、FF 默认开 + `--no-gravity-ff`、lead 0.15、rate 30、力矩报告 |
| `scripts/replay_r1arm_sim.py` | +6 | 无相机 episode 可回放 |
| `README.md` | +1046 | 真机全流程、per-joint 语法、gravity-ff、`--control-hz`、相机排障 |

### 新文件

| 文件 | 行数 | 作用 |
|---|---|---|
| `scripts/record_r1arm_real.py` | 1145 | 真机录制（PICO + 相机 → MCAP），线程：主控制环 / 录制 / 预览 / 相机 worker |
| `scripts/replay_r1arm_real.py` | 802 | 真机回放（FF + 整形 + 依赖 episode 元数据的参数继承） |
| `scripts/trajectory_test_r1arm.py` | 684 | P0 固定轨迹测试 + 逐周期遥测 |
| `teleop/local_camera_client.py` | 620 | 本地相机订阅（worker 线程、稳定性窗口、冻结检测、帧时刻） |
| `scripts/bench_r1arm_ik.py` | 349 | IK/控制环频率与耗时测量、录波 |
| `scripts/probe_r1arm_cameras.py` | 211 | 相机枚举与探帧 |
| `scripts/view_r1arm_cameras.py` | 133 | 相机实时预览 |
| `R1ARM_REAL_MOTION_OPTIMIZATION_PLAN.md` | — | P0–P3 优化清单与关账记录 |

---

## 三、还没尝试的优化点

### 阶段 7 · 10/9 — XR 平滑 + 单一 q/v/a 轨迹状态机 + SDK v3

- XR 按新 frame stamp 做 One-Euro 位置滤波与 SLERP 姿态滤波，默认 cutoff=2 Hz、beta=0.15；jump rejection 仍先运行，持续搬位被接受时重置滤波状态。record 同拍保存 `xr_raw/xr_filtered`。
- IK 原解作为目标进入唯一 per-side `q_cmd/v_cmd/a_cmd` 状态机，负责 tracking、grip release braking、regrip 和 reset/home 连续性。SDK slew/lead/hard joint limits 保留为安全兜底；grip release 不再硬冻结 q 到上一条 SDK command。
- bridge C ABI v3 新增期望速度数组 `dq[7]`；实时路径按最终安全裁剪后位置命令的实际周期差分 dq，并和同一 q 一起下发。v1/v2 保留。MCAP 增加 `dq_purpose/ddq_purpose/dq_target` 用于区分轨迹意图和最终电机目标。
- 本次实现尚未做真机实测。首次测试需托住机械臂、急停可触及；先用低速短动作验证松 grip 刹停、重新 grip 接入、home 过程连续性。

按「预期收益 ÷ 实施成本」排列。**加粗**为有明确机制支撑、值得优先试的。

### A. 执行层

1. **dq 前馈真机验证** —— 代码已新增 bridge v3，把安全限幅后的位置目标导数作为 `dq_target`，C++ 已本机构建；待无负载/低速真机验证方向、单位与跟踪收益。
2. **200 Hz 控制环实测** —— 串口预算 7.3 ms，100 Hz（10 ms）只剩 ~3 ms；若要更高频率需先确认 p95/p99 尾部，或换更低延迟的通信方式。
3. **库仑摩擦补偿** —— 重力 FF 只补了重力分量；J2/J4 在低速段的 stick-slip 是「慢速段不跟手」的候选成因。
4. **温升测量** —— P0 清单要求「记录电机温升」，至今未测；持续高频微动对电机热负载的影响未知。
5. **`--ik-dt` 保守化实验（0.03）** —— 已验证参数存在但从未真机跑过「保留指令密度、单独压 IK 每步增量」这一档。
6. **`arm_task_weight` 调参** —— Placo ee 任务权重（现 1.0）从未动过；降它可让解更保守（代价是跟随变肉）。

### B. IK / XR

7. **XR 滤波参数真机/数据集调优** —— 已加入基于 XR 帧时间戳的 One-Euro 位置/姿态滤波，默认最小截止频率 2 Hz、beta 0.15，原始与滤波后位姿均写入记录；待按实测噪声与动作滞后调参。
8. **Placo 奇异 / 不可达处理** —— 求解失败率、限位接近度从未统计；工作空间边界的目标投影/限速未实现（当前靠 DLS 回退 + 步长钳制兜底）。
9. **XR 帧时钟与控制循环重复求解** —— 当前缓存重复 XR 帧，但仍每个控制 tick 调用 tracking/IK；若 `--rate` 高于 XR 更新率，就会在保持输入上重复求解。需要决定保持周期求解还是仅新帧求解，并用日志验证控制语义。

### C. 数据 / 训练

10. **30 Hz sampling 的曲率损失** —— 终审实验实测：30 Hz 采样点线性重建的误差下界 0.009 rad，整形器再加 0.008（快速段速度削 ~25%）。是否值得提录制密度（50 Hz 存点）未定。
11. **LeRobot 转换脚本** —— `field_contract` 已写入元数据，但实际转换器（选 `ik_target`、夹爪归一化、断点切 chunk）尚未实现。
12. **断点的实际处理** —— 只打印了切分点；转换时的自动切 chunk 未实现。
13. **数据质检门槛自动化** —— 现在是保存时打印 6 项警告，但没有「异常 episode 自动拒绝进训练集」的机制。
14. **相机 60 fps + 录制对齐** —— D405 支持 60 fps（实测 5 s 满速、帧间隔 p99 16.8 ms），SN0002 固件锁 30；两路不同步时的对齐策略未设计。

### D. 已知缺陷（未修）

15. **Dex1 DDS 偶发 2 s 卡顿** —— 实测 `hand_cmd` 在动而 `state` 冻结（臂运动时网络拥塞）；候选修复：publisher QoS 改 RELIABLE、Dex1 独占网口。**未验证、未修**。
16. **record 脚本退出时 `QObject::killTimer: Timers cannot be stopped from another thread`** —— Qt 告警（预览线程的窗口在主线程关闭），无害但每次留告警。
17. **`goto_r1arm_pose.py` Phase 3 保持段** —— 直接发 `target` 不走 `limit_command()`，与爬行段不一致（计划文档 P0 第 3 条指出，未改）。
18. **CPU 侧 GIL 竞争余量** —— preview 已移出控制线程，但 `_publish_preview_state()` 的拼图仍在控制线程（<1 ms）；若仍有残余抖动，下一步是彻底移出（代价：跨线程大数组拷贝）。

### E. 已验证但可复用的工具

19. **`trajectory_test_r1arm.py` 的 level 3 提速档** —— 只在 P0 跑过一档（speedup 2）；更高速度下的误差增长率未扫全。
20. **bench 的 `--sweep` 上限** —— 实测到 400 Hz 仍 100% IK 覆盖；更高频档（800 Hz 失守）的瓶颈定位（串口 vs MuJoCo 步进）未细分。

---

## 四、关键实测数字速查

| 指标 | 数值 | 来源 |
|---|---|---|
| 串口 `read+command` | 7.3 ms（p95 9.4） | trajectory_test 遥测 |
| 控制周期 50 Hz | p95 20.0 ms，超限 2–17% | 同上 |
| 纯 IK 每拍 | 0.08–0.10 ms（→ 上限 ~10 kHz） | bench |
| XR 读每拍 | 0.04–0.07 ms | bench |
| 重力负载（实测） | J2 −2.5 / J4 +5.3 / J6 +1.25 Nm | goto 保持段 tau_est |
| FF 前后 J4 悬垂 | 0.065 → 0.014 rad（−75%） | goto 真机 |
| 增益矩阵 J4 RMS | 80/2: 0.019；C2: 0.027（但过冲 0.81×） | trajectory_test |
| solver 原始 a_max | 无 QP 限 219 → 有 52 rad/s² | bench 录波 |
| 30 Hz 采样重建误差 | 线性下界 0.009 rad；+整形器 0.008 rad | 终审离线实验 |
| 电机峰值扭矩 | 34.4 Nm（6010/6014）；速度 >180°/s | 厂商规格 |
