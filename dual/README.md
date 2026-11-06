# R1Arm 双臂基线

完整操作流程统一见 [主 README](../README.md)，避免维护两套命令。

```bash
# 终端一：左右 ID7 夹爪服务
bash dual/start_grippers.sh

# 终端二：三路图像、90 Hz 控制／30 Hz 观测录制
python scripts/teleop.py --dual --record --motion-mode --data-root data/dual_real

# 只读检查；确认后增加 --motion-mode；图像默认由独立进程显示
python dual/replay.py --episode data/dual_real/episode_0000.mcap

# 不驱动真机，播放图像和轨迹
python scripts/replay/replay_r1arm_sim.py --episode data/dual_real/episode_0000.mcap
```

上述命令前需激活 `r1_teleop` 并 source `scripts/real_env.sh`。
X 开始／保存，B 暂停／恢复，A 两臂回 home；左右 grip/trigger 控制对应侧。

## 左右模型映射

使用同一 R1Arm URDF，但左右模型、运动学数据、整形状态和标定独立。
标定总线到共同模型方向：

- 右：`[1, 1, 1, -1, 1, 1, 1]`
- 左：`[1, -1, 1, 1, 1, -1, 1]`

左默认 J2/J6 mirrored、J4 model，不能在零点文件中重复反号。
共同模型 home 为 `[0,1.2,0,-1.2,0,0,0]`。没有双臂碰撞规划。
普通启动位置检查容差 0.10 rad；启用 home 时支持最多 0.25 rad 初始有限超限恢复，不自动取模或修复错误标定。

## 录制和回放

MCAP 16D：右七轴＋右夹爪＋左七轴＋左夹爪。夹爪逻辑位置是 slide 单位，不是夹爪电机弧度。
两条串口由常驻线程并行执行，完成后发布成对控制快照；XR 每轮共享一份快照。
录制后台编码写盘；控制队列有界，溢出／磁盘错误明确失败；夹爪反馈无效时停止录制。
图像为头部 D405、左腕 JR0001、右腕 JR0002，640×480／30 fps。
回放优先高频控制流；暂停分段独立接近首帧；同步图像仅展示录制内容。

叠毛巾五步需要在同一进程内保持双臂控制。复位毛巾和双臂后，使用原始录制按 8、12、20、26 秒暂停；每段完成时持续下发保持指令并检查反馈，操作员在 120 秒内输入 `MOVE` 才继续，其他输入或 Ctrl-C 会安全停止。不要分别启动五个切片：每次进程退出都会停止电机，下一段起点可能因手臂下垂而失配。

```bash
python dual/replay.py --episode data/dual_real/episode_0002.mcap \
  --motion-mode --speed 0.5 --dex1-net lo --settle-to-recorded-state \
  --pause-at 8,12,20,26 --no-show-observations --no-print-actions
```

启动前确认双臂姿态在回放恢复范围内、夹爪服务和四路相机正常，保持实体急停可触及。暂停时不要移动机械臂；UI 退出只关闭画面，停止回放应在回放终端按 Ctrl-C。

`grippers.py` 是单／双夹爪共用启动实现，`scripts/start_gripper.sh` 默认只开右侧，`dual/start_grippers.sh` 默认开双侧。
重复启动检查不再调用可能卡住的 fuser；不主动杀掉已有服务。

SDK 和夹爪服务均为官方来源 submodule；本地设备适配在 `native/dependency_patches/` 中保存，安装时由 setup 应用，详见主 README。
