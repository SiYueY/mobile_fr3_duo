"""Kinematic TMR base command example."""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from model_builder.kinematic_base import BaseTwist, KinematicBaseController  # noqa: E402


def demo(duration_s: float = 3.0) -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "models/scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    controller = KinematicBaseController(model)
    start = data.qpos[:3].copy()
    for _ in range(round(duration_s / model.opt.timestep)):
        mujoco.mj_step(model, data)
        controller.advance(data, BaseTwist(vx=0.5))
    print(f"moved {(data.qpos[:2] - start[:2]).tolist()} m")


if __name__ == "__main__":
    demo()
