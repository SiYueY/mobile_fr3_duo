"""Dual-arm gravity compensation and joint-space PD (position variant)."""

import mujoco
import numpy as np


def test_arms_stable_under_gravity_compensation(position_model):
    """Position actuators hold the keyframe pose without NaN or runaway."""
    data = mujoco.MjData(position_model)
    mujoco.mj_resetDataKeyframe(position_model, data, 0)
    ref = data.qpos.copy()
    max_pos_err = 0.0
    for _ in range(5000):
        mujoco.mj_step(position_model, data)
        if not np.isfinite(data.qpos).all():
            raise AssertionError("NaN in qpos")
        for side in ("left", "right"):
            for i in range(1, 8):
                jid = mujoco.mj_name2id(
                    position_model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    f"{side}_fr3v2_1_joint{i}",
                )
                adr = position_model.jnt_qposadr[jid]
                err = abs(data.qpos[adr] - ref[adr])
                max_pos_err = max(max_pos_err, err)
    assert max_pos_err < 0.2, f"arms drifted: {max_pos_err}"
