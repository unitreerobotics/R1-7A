"""R1 单臂（7-DoF）配置与常量。"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass

import numpy as np


DEFAULT_ASSETS_DIR = pathlib.Path(__file__).resolve().parents[1] / "assets"
DEFAULT_URDF_PATH = DEFAULT_ASSETS_DIR / "r1arm" / "R1_Robotic_Arm.urdf"
DEFAULT_MJCF_PATH = DEFAULT_ASSETS_DIR / "r1arm" / "scene.xml"

# URDF 关节序：J1..J7
ARM_JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
EE_FRAME = "Link7"

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
    # 默认跟右手柄（单臂）
    control_side: str = "right"
    scale_factor: float = 1.0
    arm_control_mode: str = "pose"  # "pose" or "position"
    arm_task_weight: float = 1.0
    # MuJoCo 位置伺服刚度
    position_kp: float = 200.0
    # 是否把 A7 遥操世界增量转到 r1arm 基座系（关掉则与 A7 裸用同一世界轴）
    align_teleop_frame: bool = True
