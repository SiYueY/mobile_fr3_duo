"""Grasp scenario: cube/cylinder/plate pick-up attempt with lift and move.

Best-effort demo: the inflated official self-collision shells make robust
dynamic grasping near the hand fragile, so the example reports the outcome
instead of asserting success (see README "Known limitations").
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent

OBJECT_GEOMS = {
    "cube": (
        '<geom type="box" size="0.02 0.02 0.02" mass="0.05"/>'
        '<inertial pos="0 0 0" mass="0.05" '
        'fullinertia="1.3333e-5 1.3333e-5 1.3333e-5 0 0 0"/>',
        0.04,
    ),
    "cylinder": (
        '<geom type="cylinder" size="0.015 0.03" mass="0.04"/>'
        '<inertial pos="0 0 0" mass="0.04" '
        'fullinertia="1.05e-5 1.05e-5 4.5e-6 0 0 0"/>',
        0.03,
    ),
    "plate": (
        '<geom type="box" size="0.03 0.005 0.03" mass="0.02"/>'
        '<inertial pos="0 0 0" mass="0.02" '
        'fullinertia="1.5e-6 3e-6 1.5e-6 0 0 0"/>',
        0.01,
    ),
}


def _build_scene(shape: str) -> tuple[mujoco.MjModel, int]:
    geom, _ = OBJECT_GEOMS[shape]
    root = ET.parse(REPO_ROOT / "scene.xml").getroot()
    worldbody = root.find("worldbody")
    body = ET.SubElement(worldbody, "body", name="object", pos="0.8 0.42 0.8")
    ET.SubElement(body, "freejoint")
    for child in ET.fromstring(f"<root>{geom}</root>"):
        body.append(child)
    path = REPO_ROOT / ".grasp_scenario.xml"
    ET.ElementTree(root).write(path, encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(path))
    return model, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")


def run(shape: str, duration_s: float = 4.0) -> float:
    model, obj_id = _build_scene(shape)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 2)  # manipulation
    for name in ("left_fr3v2_1_finger_joint1", "left_fr3v2_1_finger_joint2"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = 0.02
    mujoco.mj_forward(model, data)

    # place the object just below the left hand with the fingers wide open
    lid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_fr3v2_1_hand_tcp")
    obj_pos = data.site_xpos[lid] - np.array([0.0, 0.0, 0.05])
    oid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "object")
    adr = model.jnt_qposadr[oid]
    data.qpos[adr : adr + 3] = obj_pos
    data.qpos[adr + 3 : adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)

    steps = int(duration_s / model.opt.timestep)
    for i in range(steps):
        for a in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or ""
            if "finger" in name:
                data.ctrl[a] = -20.0
            elif "spine" in name and i > steps // 2:
                data.ctrl[a] = 100.0
        mujoco.mj_step(model, data)
    (REPO_ROOT / ".grasp_scenario.xml").unlink(missing_ok=True)
    return float(data.xpos[obj_id][2])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--shape", choices=sorted(OBJECT_GEOMS), default="cube")
    ap.add_argument("--duration", type=float, default=4.0)
    args = ap.parse_args()
    z = run(args.shape, args.duration)
    print(f"{args.shape} final z: {z:.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
