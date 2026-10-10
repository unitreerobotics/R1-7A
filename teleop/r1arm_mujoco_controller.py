"""R1 单臂 / 双臂 MuJoCo 仿真遥操控制器。

Pinocchio/Placo 做 IK，MuJoCo 做物理仿真与可视化；每个控制周期把求解的
臂关节 + Dex1 开合写入 position actuator，并推进若干仿真步。
"""

from __future__ import annotations

from robot.mujoco_sim import MujocoArmSim
from robot.r1arm_config import (
    ARM_JOINTS,
    DEX1_JOINTS,
    LEFT_ARM_JOINTS,
    LEFT_DEX1_JOINTS,
    R1ArmConfig,
)
from teleop.r1arm_controller_base import R1ArmTeleopControllerBase


class R1ArmMujocoTeleopController(R1ArmTeleopControllerBase):
    """仿真端：IK + Dex1 → MuJoCo 位置伺服。"""

    def __init__(
        self,
        config: R1ArmConfig,
        mock_xr: bool = False,
        debug: bool = False,
        scale_factor: float | None = None,
        show_viewer: bool = True,
        sim_substeps: int = 10,
        ik_dt: float = 0.0,
    ):
        super().__init__(
            config=config,
            mock_xr=mock_xr,
            debug=debug,
            scale_factor=scale_factor,
        )
        if ik_dt > 0.0:
            solver_dt = float(ik_dt)
            for solver in (self.placo_solver, self.placo_solver_left):
                if solver is not None:
                    solver.control_rate_hz = 1.0 / solver_dt
                    solver.solver_dt = solver_dt
                    solver.solver.dt = solver_dt
        self.sim_substeps = max(1, int(sim_substeps))
        if self.dual_arm:
            joint_names = ARM_JOINTS + DEX1_JOINTS + LEFT_ARM_JOINTS + LEFT_DEX1_JOINTS
        else:
            joint_names = ARM_JOINTS + DEX1_JOINTS
        self.sim = MujocoArmSim(
            config.mjcf_path,
            q_init=self.sim_q(),
            show_viewer=show_viewer,
            joint_names=joint_names,
        )
        print(f"MuJoCo sim loaded: {config.mjcf_path}")

    def _output_command(self):
        self.sim.set_ctrl(self.sim_q())
        self.sim.step(self.sim_substeps)

    def reset(self):
        super().reset()
        self.sim.set_q(self.sim_q(), also_ctrl=True)

    def _on_stop(self):
        super()._on_stop()
        self.sim.close()
