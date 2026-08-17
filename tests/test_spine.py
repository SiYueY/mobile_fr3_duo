"""Spine position control and full-travel checks (position variant)."""

import tempfile

import mujoco
from helpers import REPO_ROOT


def _load_position():
    """Grounded scene built from the position variant."""
    path = tempfile.mktemp(suffix=".xml", dir=REPO_ROOT)
    text = (
        '<mujoco model="position_scene">\n'
        '  <include file="mobile_fr3_duo_position.xml"/>\n'
        "  <worldbody>\n"
        '    <geom name="ground" type="plane" size="50 50 0.1" pos="0 0 -0.001" '
        'group="1" condim="3" friction="1.0 0.005 0.0001" '
        'rgba="0.55 0.55 0.55 1"/>\n'
        "  </worldbody>\n"
        "</mujoco>\n"
    )
    from pathlib import Path

    Path(path).write_text(text)
    return mujoco.MjModel.from_xml_path(path), path


def _spine():
    model, path = _load_position()
    data = mujoco.MjData(model)
    jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "franka_spine_vertical_joint"
    )
    adr = model.jnt_qposadr[jid]
    aid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        "franka_spine_vertical_joint_actuator",
    )
    lo, hi = model.jnt_range[jid]
    return model, data, jid, adr, aid, lo, hi, path


def test_spine_full_travel():
    model, data, jid, adr, aid, lo, hi, path = _spine()
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for target in (lo, hi, 0.42):
        data.ctrl[aid] = target
        for _ in range(3000):
            mujoco.mj_step(model, data)
        # At the upper limit, the corrected dual-arm moment load leaves a
        # small static position error for this force-limited simulation model.
        assert abs(data.qpos[adr] - target) < 0.04, f"spine target {target}"
    import os

    os.unlink(path)


def test_spine_height_hold_60s():
    model, data, jid, adr, aid, lo, hi, path = _spine()
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[aid] = 0.4
    for _ in range(2000):
        mujoco.mj_step(model, data)
    q0 = data.qpos[adr]
    for _ in range(60000):
        mujoco.mj_step(model, data)
    assert abs(data.qpos[adr] - q0) < 0.05
    import os

    os.unlink(path)
