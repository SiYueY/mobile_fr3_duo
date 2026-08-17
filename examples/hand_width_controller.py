"""Franka Hand width controller (width = q_finger1 + q_finger2 in [0, 0.08])."""

from __future__ import annotations

import mujoco
import numpy as np


class HandWidthController:
    def __init__(self, model: mujoco.MjModel, side: str = "left"):
        self.model = model
        self.side = side
        self.joint1 = f"{side}_fr3v2_1_finger_joint1"
        self.joint2 = f"{side}_fr3v2_1_finger_joint2"
        self.adr1 = model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, self.joint1)
        ]
        self.adr2 = model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, self.joint2)
        ]

    def set_width(self, data: mujoco.MjData, width: float) -> None:
        width = float(np.clip(width, 0.0, 0.08))
        q = width / 2
        data.qpos[self.adr1] = q
        data.qpos[self.adr2] = q

    def width(self, data: mujoco.MjData) -> float:
        return float(data.qpos[self.adr1] + data.qpos[self.adr2])


def demo() -> None:
    from pathlib import Path

    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).resolve().parent.parent / "mobile_fr3_duo.xml")
    )
    data = mujoco.MjData(model)
    controller = HandWidthController(model)
    for width in (0.08, 0.04, 0.0):
        controller.set_width(data, width)
        mujoco.mj_forward(model, data)
        print(f"requested width {width} -> measured {controller.width(data):.3f}")


if __name__ == "__main__":
    demo()
