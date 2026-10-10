"""R1 单臂（7-DoF + Dex1）配置与常量。"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass

import numpy as np


DEFAULT_ASSETS_DIR = pathlib.Path(__file__).resolve().parents[1] / "assets"
DEFAULT_URDF_PATH = DEFAULT_ASSETS_DIR / "r1arm" / "R1_Robotic_Arm.urdf"
DEFAULT_MJCF_PATH = DEFAULT_ASSETS_DIR / "r1arm" / "scene.xml"
DEFAULT_DUAL_MJCF_PATH = DEFAULT_ASSETS_DIR / "r1arm" / "scene_dual_arm.xml"

# URDF 关节序：J1..J7 + Dex1 两指
ARM_JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
DEX1_JOINTS = ["dex1_finger_joint_1", "dex1_finger_joint_2"]
# 双臂 MuJoCo 场景中左臂前缀（右臂保持无前缀，兼容单臂）
LEFT_ARM_JOINTS = [f"left_{n}" for n in ARM_JOINTS]
LEFT_DEX1_JOINTS = [f"left_{n}" for n in DEX1_JOINTS]
# 与 A7 一致：IK 跟踪夹爪底座
EE_FRAME = "dex1_base_link"

# 取自 URDF <limit>
ARM_LIMITS = {
    "J1": (-2.723, 2.723),
    "J2": (0.0, 3.1416),
    "J3": (-0.349, 3.491),
    "J4": (-3.1415, 0.0),
    "J5": (-2.269, 2.269),
    "J6": (-0.524, 0.524),
    "J7": (-1.571, 1.571),
}

ARM_SAFE_RANGE_FRAC = 0.85
ARM_SAFE_LIMITS = {
    name: (
        float(lo + (hi - lo) * (1.0 - ARM_SAFE_RANGE_FRAC) / 2.0),
        float(hi - (hi - lo) * (1.0 - ARM_SAFE_RANGE_FRAC) / 2.0),
    )
    for name, (lo, hi) in ARM_LIMITS.items()
}

# 伸展到前方、肘略弯的可用初值（落在各关节限位内）
ARM_INIT = np.array([0.0, 1.2, 0.0, -1.2, 0.0, 0.0, 0.0], dtype=float)

# Dex1 仿真关节（与 A7 / robot.config 一致）：upper=开，lower=关
DEX1_SIM_OPEN = 0.0245
DEX1_SIM_CLOSE = -0.02
DEX1_SIM_SPEED = 0.08
DEX1_TRIGGER_DEADBAND = 0.05
# 真机 Dex1 电机量程（与 robot.eef.dex1 / config 一致）
DEX1_REAL_OPEN = 5.0
DEX1_REAL_CLOSE = 0.0

# PICO→遥操世界（与 A7 相同：+X 前、+Y 左、+Z 上）再映到 r1arm URDF 基座。
# r1arm 在 home 附近沿 -Y 伸出、+Z 向上，故：
#   teleop +X(前) → base -Y
#   teleop +Y(左) → base +X
#   teleop +Z(上) → base +Z
R_TELEOP_TO_BASE = np.array(
    [
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=float,
)


@dataclass
class R1ArmConfig:
    """R1 单臂遥操配置。"""

    urdf_path: str = str(DEFAULT_URDF_PATH)
    mjcf_path: str = str(DEFAULT_MJCF_PATH)
    ee_frame: str = EE_FRAME
    # True：左右手柄分别控左右臂（与 --use-dual-arm 绑定；此时忽略 control_side）
    dual_arm: bool = False
    # 单臂模式跟哪只手柄；dual_arm=True 时无效
    control_side: str = "right"
    scale_factor: float = 1.0
    arm_control_mode: str = "pose"  # "pose" or "position"
    arm_task_weight: float = 1.0
    # MuJoCo 位置伺服刚度
    position_kp: float = 200.0
    # Dex1 仿真开合参数；真机时由子类接 Dex1.set_gripper_ratios
    dex1_open: float = DEX1_SIM_OPEN
    dex1_close: float = DEX1_SIM_CLOSE
    dex1_speed: float = DEX1_SIM_SPEED
    trigger_deadband: float = DEX1_TRIGGER_DEADBAND
    # A 键回 home：最短名义持续帧数（实际到位时间由速度/加速度限制决定）
    reset_blend_frames: int = 8
    # 轨迹状态机速度上限，单位 rad/s；None = 不额外限速。
    # 〔2026-10-09 30Hz 定居〕全链 30Hz（XR帧时钟1:1 + IK + 执行 + 录制），
    # action 与 state 同拍统一。速度 1.0：1.5 曾激发停住瞬态振荡（能量×2.25
    # 且 30Hz 阶梯更粗），先以 1.0 为 30Hz 基线，跟踪灵敏度经底层参数
    # （kd/FF）调优后再评估提速。
    trajectory_max_speed_rad_s: float | None = 1.0
    # 独立 q_cmd/v_cmd/a_cmd 状态机加速度上限，单位 rad/s²。
    ik_max_acc_per_s2: float = 6.0
    # 独立轨迹状态机 jerk 上限，单位 rad/s³。
    ik_max_jerk_per_s3: float = 80.0
    # a_desired = position_gain * position_error - velocity_gain * velocity.
    trajectory_position_gain: float = 16.0
    trajectory_velocity_gain: float = 8.0
    # One-Euro XR pose filter, updated only when XR frame stamp advances.
    xr_filter_min_cutoff: float = 2.0
    xr_filter_beta: float = 0.15
    # Placo 内附加速度限制关闭；动态速度/加速度只由轨迹状态机负责。
    # 逐关节覆盖：ik_joint_velocity_limits = {"J5": 0.8, ...}（未列关节用默认）。
    ik_qp_velocity_limit: float | None = None
    # Optional acceleration bound embedded in the Placo QP (rad/s^2).
    ik_qp_acceleration_limit: float | None = None
    # Optional jerk bound embedded in the Placo QP (rad/s^3); disabled by default.
    ik_qp_jerk_limit: float | None = None
    ik_joint_velocity_limits: dict[str, float] | None = None


def sim_to_closed_ratio(sim_value: float, open_q: float = DEX1_SIM_OPEN, close_q: float = DEX1_SIM_CLOSE) -> float:
    """仿真关节值 → 0..1 闭合比（0=开，1=关），与 A7 RealTeleopController 一致。"""
    denom = open_q - close_q
    ratio = 0.0 if abs(denom) < 1e-9 else (open_q - float(sim_value)) / denom
    return float(np.clip(ratio, 0.0, 1.0))
