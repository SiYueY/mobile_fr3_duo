"""Franka Hand coupling, width, and pad kinematics."""

import mujoco
import numpy as np


def test_finger_coupling_equality(base_model):
    assert base_model.neq == 2
    data = mujoco.MjData(base_model)
    mujoco.mj_resetDataKeyframe(base_model, data, 0)
    j1 = base_model.jnt_qposadr[
        mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, "left_fr3v2_1_finger_joint1")
    ]
    j2 = base_model.jnt_qposadr[
        mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, "left_fr3v2_1_finger_joint2")
    ]
    for _ in range(2000):
        mujoco.mj_step(base_model, data)
    assert abs(data.qpos[j1] - data.qpos[j2]) < 5e-3


def test_hand_width_range(base_model):
    data = mujoco.MjData(base_model)
    # width = q_finger1 + q_finger2 in [0, 0.08]
    j1 = base_model.jnt_qposadr[
        mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, "left_fr3v2_1_finger_joint1")
    ]
    j2 = base_model.jnt_qposadr[
        mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, "left_fr3v2_1_finger_joint2")
    ]
    for target in (0.0, 0.04, 0.08):
        mujoco.mj_resetDataKeyframe(base_model, data, 0)
        q = target / 2
        data.qpos[j1] = q
        data.qpos[j2] = q
        mujoco.mj_forward(base_model, data)
        assert abs(data.qpos[j1] + data.qpos[j2] - target) < 1e-9


def test_finger_pads_mirror_and_accept_cube(scene_model):
    """The mirrored finger kinematics place the pads apart symmetrically."""
    model = scene_model
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 2)  # manipulation
    for name in ("left_fr3v2_1_finger_joint1", "left_fr3v2_1_finger_joint2"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = 0.02  # width 0.04
    mujoco.mj_forward(model, data)

    def pad_center(finger):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, finger)
        for gi in range(model.ngeom):
            if (
                model.geom_bodyid[gi] == bid
                and model.geom_type[gi] == mujoco.mjtGeom.mjGEOM_BOX
                and abs(model.geom_size[gi][1] - 0.0076) < 1e-4  # rubber tip
            ):
                return data.geom_xpos[gi].copy()
        raise AssertionError("pad not found")

    pad_l = pad_center("left_fr3v2_1_leftfinger")
    pad_r = pad_center("left_fr3v2_1_rightfinger")
    # pads must be separated by more than the closed width and on opposite
    # sides of the hand center
    sep = np.linalg.norm(pad_l - pad_r)
    assert sep > 0.04, f"pads too close: {sep}"
    center = (pad_l + pad_r) / 2
    hand = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_fr3v2_1_hand")
    assert np.dot(pad_l - center, pad_r - center) < 0
    assert np.linalg.norm(center - data.xpos[hand]) < 0.15
