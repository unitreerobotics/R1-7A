"""R1 单臂 MuJoCo 仿真遥操控制器。

Pinocchio/Placo 做 IK，MuJoCo 做物理仿真与可视化；每个控制周期把求解的
关节角写入 position actuator，并推进若干仿真步。
"""

from __future__ import annotations

from robot.mujoco_sim import MujocoArmSim
from robot.r1arm_config import R1ArmConfig
from teleop.r1arm_controller_base import R1ArmTeleopControllerBase


class R1ArmMujocoTeleopController(R1ArmTeleopControllerBase):
    """仿真端：IK → MuJoCo 位置伺服。"""

    def __init__(
        self,
        config: R1ArmConfig,
        mock_xr: bool = False,
        debug: bool = False,
        scale_factor: float | None = None,
        show_viewer: bool = True,
        sim_substeps: int = 10,
    ):
        super().__init__(
            config=config,
            mock_xr=mock_xr,
            debug=debug,
            scale_factor=scale_factor,
        )
        self.sim_substeps = max(1, int(sim_substeps))
        self.sim = MujocoArmSim(
            config.mjcf_path,
            q_init=self.arm_q(),
            show_viewer=show_viewer,
        )
        print(f"MuJoCo sim loaded: {config.mjcf_path}")

    def _output_command(self):
        self.sim.set_ctrl(self.arm_q())
        self.sim.step(self.sim_substeps)

    def reset(self):
        super().reset()
        self.sim.set_q(self.arm_q(), also_ctrl=True)

    def _on_stop(self):
        self.sim.close()
