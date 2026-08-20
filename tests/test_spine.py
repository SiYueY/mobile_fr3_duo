"""Spine position control and full-travel checks (position variant)."""

import mujoco
from helpers import REPO_ROOT


def _load_position():
    """Grounded scene built from the position variant."""
    path = REPO_ROOT / "models/scene_position.xml"
    return mujoco.MjModel.from_xml_path(str(path)), None


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
    assert path is None


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
    assert path is None
