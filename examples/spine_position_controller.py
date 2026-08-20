"""Spine position controller (position-actuator variant)."""

from __future__ import annotations

import mujoco
import numpy as np


class SpinePositionController:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.jid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "franka_spine_vertical_joint"
        )
        self.aid = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            "franka_spine_vertical_joint_actuator",
        )

    def set_height(self, data: mujoco.MjData, height: float) -> None:
        data.ctrl[self.aid] = float(np.clip(height, 0.0, 0.85))

    def height(self, data: mujoco.MjData) -> float:
        return float(data.qpos[self.model.jnt_qposadr[self.jid]])


def demo(duration_s: float = 3.0) -> None:
    from pathlib import Path

    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).resolve().parent.parent / "models/scene_position.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    controller = SpinePositionController(model)
    controller.set_height(data, 0.6)
    for _ in range(int(duration_s / model.opt.timestep)):
        mujoco.mj_step(model, data)
    print(f"spine settled at {controller.height(data):.3f} m (target 0.6)")


if __name__ == "__main__":
    demo()
