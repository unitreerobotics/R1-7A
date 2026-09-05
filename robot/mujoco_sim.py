"""MuJoCo 单臂+Dex1 仿真封装（位置伺服跟踪 IK / 夹爪目标）。"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from robot.r1arm_config import ARM_JOINTS, DEX1_JOINTS


class MujocoArmSim:
    """加载 r1arm MJCF，用 position actuator 跟踪目标关节角。"""

    def __init__(
        self,
        mjcf_path: str | Path,
        q_init: np.ndarray | None = None,
        show_viewer: bool = True,
        joint_names: list[str] | None = None,
    ):
        import mujoco
        import mujoco.viewer

        self._mujoco = mujoco
        self.mjcf_path = Path(mjcf_path)
        if not self.mjcf_path.exists():
            raise FileNotFoundError(f"MJCF not found: {self.mjcf_path}")

        self.model = mujoco.MjModel.from_xml_path(str(self.mjcf_path))
        self.data = mujoco.MjData(self.model)

        self.joint_qposadr: dict[str, int] = {}
        self.actuator_ids: dict[str, int] = {}
        for j in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if name:
                self.joint_qposadr[name] = int(self.model.jnt_qposadr[j])
        for a in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
            if name:
                joint_key = name[4:] if name.startswith("act_") else name
                self.actuator_ids[joint_key] = a

        ordered = list(joint_names or (ARM_JOINTS + DEX1_JOINTS))
        self.joint_names = [n for n in ordered if n in self.joint_qposadr]
        if q_init is not None:
            self.set_q(np.asarray(q_init, dtype=float), also_ctrl=True)
        else:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)

        mujoco.mj_forward(self.model, self.data)

        self.viewer = None
        self.show_viewer = show_viewer
        if show_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.azimuth = 140
            self.viewer.cam.elevation = -20
            self.viewer.cam.distance = 1.6
            self.viewer.cam.lookat[:] = [0.2, 0.0, 0.35]

    def set_q(self, q: np.ndarray, also_ctrl: bool = True) -> None:
        """按 joint_names 顺序写入 qpos（可选同步 ctrl）。"""
        q = np.asarray(q, dtype=float).reshape(-1)
        for i, name in enumerate(self.joint_names):
            if i >= q.size:
                break
            adr = self.joint_qposadr[name]
            self.data.qpos[adr] = q[i]
            if also_ctrl and name in self.actuator_ids:
                self.data.ctrl[self.actuator_ids[name]] = q[i]
        self._mujoco.mj_forward(self.model, self.data)

    def set_ctrl(self, q_des: np.ndarray) -> None:
        """设置位置伺服目标。"""
        q_des = np.asarray(q_des, dtype=float).reshape(-1)
        for i, name in enumerate(self.joint_names):
            if i >= q_des.size:
                break
            if name in self.actuator_ids:
                self.data.ctrl[self.actuator_ids[name]] = q_des[i]

    def get_q(self) -> np.ndarray:
        return np.array([self.data.qpos[self.joint_qposadr[n]] for n in self.joint_names], dtype=float)

    def step(self, n: int = 1) -> None:
        for _ in range(max(1, n)):
            self._mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            if not self.viewer.is_running():
                self.show_viewer = False
                self.viewer = None
            else:
                self.viewer.sync()

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
