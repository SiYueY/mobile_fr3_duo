"""TMR swerve steering and drive controller.

Maps a body-frame velocity command (vx, vy, yaw_rate) to the four swerve
modules using the standard swerve-drive inverse kinematics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class SwerveCommand:
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0


class TmrSwerveController:
    """Front/rear steering + drive wheels of the TMR v0.2 base."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        # module positions in the base frame (front x, rear x, ±y)
        self.front = np.array([0.3, -0.2])
        self.rear = np.array([-0.3, 0.2])
        self.wheel_radius = 0.05
        self._wheel_angle = {"tmrv0_2_joint_1": 0.0, "tmrv0_2_joint_3": 0.0}

    def _set(self, data: mujoco.MjData, joint: str, value: float) -> None:
        jid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, joint
        )
        aid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint}_actuator"
        )
        if jid >= 0 and aid >= 0:
            data.ctrl[aid] = value

    def apply(self, data: mujoco.MjData, cmd: SwerveCommand) -> None:
        """Position-control steering and drive wheel speed (position variant)."""
        for name, pos in (("front", self.front), ("rear", self.rear)):
            vx_w = cmd.vx - cmd.yaw_rate * pos[1]
            vy_w = cmd.vy + cmd.yaw_rate * pos[0]
            speed = math.hypot(vx_w, vy_w) / self.wheel_radius
            steer = math.atan2(vy_w, vx_w)
            self._set(data, f"tmrv0_2_joint_{0 if name == 'front' else 2}", steer)
            wheel = f"tmrv0_2_joint_{1 if name == 'front' else 3}"
            self._wheel_angle[wheel] += speed * 0.001  # position target in rad
            self._set(data, wheel, self._wheel_angle[wheel])


def demo(duration_s: float = 3.0) -> None:
    from pathlib import Path

    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).resolve().parent.parent / "build/scene_position.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    controller = TmrSwerveController(model)
    x0 = data.xpos[1].copy()
    steps = int(duration_s / model.opt.timestep)
    cmd = SwerveCommand(vx=0.5, vy=0.0, yaw_rate=0.0)
    for _ in range(steps):
        controller.apply(data, cmd)
        mujoco.mj_step(model, data)
    dx = np.linalg.norm((data.xpos[1] - x0)[:2])
    print(f"commanded 0.5 m/s forward for {duration_s}s -> moved {dx:.2f} m")


if __name__ == "__main__":
    demo()
