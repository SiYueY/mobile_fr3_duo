"""Kinematic TMR base control for the formal free-joint model."""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class BaseTwist:
    """Desired base-frame velocity in meters/s and radians/s."""

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0


class KinematicBaseController:
    """Prescribe TMR free-joint motion and synchronize wheel animation.

    Wheel contact is retained for collision visualization, but never used to
    produce the commanded base motion.  Call after ``mj_step`` when simulating
    arms, or instead of it for pure kinematic preview.
    """

    WHEEL_RADIUS = 0.05
    MODULES = (("tmrv0_2_joint_0", "tmrv0_2_joint_1", (0.3, -0.2)),
               ("tmrv0_2_joint_2", "tmrv0_2_joint_3", (-0.3, 0.2)))

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.base_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_freejoint")
        if self.base_joint < 0:
            raise ValueError("model has no base_freejoint")

    def advance(self, data: mujoco.MjData, command: BaseTwist, dt: float | None = None) -> None:
        dt = self.model.opt.timestep if dt is None else dt
        qadr = self.model.jnt_qposadr[self.base_joint]
        dadr = self.model.jnt_dofadr[self.base_joint]
        quat = data.qpos[qadr + 3:qadr + 7]
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, quat)
        world_linear = rotation.reshape(3, 3) @ np.array((command.vx, command.vy, 0.0))
        data.qpos[qadr:qadr + 3] += world_linear * dt
        mujoco.mju_quatIntegrate(quat, np.array((0.0, 0.0, command.yaw_rate)), dt)
        data.qvel[dadr:dadr + 3] = world_linear
        data.qvel[dadr + 3:dadr + 6] = (0.0, 0.0, command.yaw_rate)
        self._sync_wheels(data, command, dt)
        mujoco.mj_forward(self.model, data)

    def _sync_wheels(self, data: mujoco.MjData, command: BaseTwist, dt: float) -> None:
        for steering, wheel, (x, y) in self.MODULES:
            vx = command.vx - command.yaw_rate * y
            vy = command.vy + command.yaw_rate * x
            speed = math.hypot(vx, vy) / self.WHEEL_RADIUS
            steer_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, steering)
            wheel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, wheel)
            if steer_id >= 0 and speed > 1e-12:
                data.qpos[self.model.jnt_qposadr[steer_id]] = math.atan2(vy, vx)
            if wheel_id >= 0:
                qadr = self.model.jnt_qposadr[wheel_id]
                dadr = self.model.jnt_dofadr[wheel_id]
                data.qpos[qadr] += speed * dt
                data.qvel[dadr] = speed
