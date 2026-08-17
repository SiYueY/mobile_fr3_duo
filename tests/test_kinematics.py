"""FK cross-validation: MuJoCo vs analytic URDF FK (and PyKDL when present)."""

import mujoco
import numpy as np
import pytest
from helpers import (
    apply_base_transform,
    frame_error,
    mjcf_qpos_for_urdf,
    random_valid_qpos,
    set_urdf_qpos,
)


def _mjcf_frame(model, data, body_name):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return data.xpos[bid].copy(), data.xquat[bid].copy()


@pytest.mark.parametrize(
    "body",
    [
        "left_fr3v2_1_link0",
        "left_fr3v2_1_link3",
        "left_fr3v2_1_hand",
        "left_fr3v2_1_leftfinger",
        "right_fr3v2_1_link0",
        "right_fr3v2_1_link5",
        "right_fr3v2_1_hand",
        "right_fr3v2_1_rightfinger",
        "franka_spine",
        "head_link",
        "fr3_duo_mount_origin",
    ],
)
def test_fk_random_poses(base_model, urdf, body, seed=1234):
    rng = np.random.default_rng(seed)
    data = mujoco.MjData(base_model)
    for trial in range(50):
        values = random_valid_qpos(base_model, rng)
        set_urdf_qpos(base_model, data, urdf, values)
        mujoco.mj_forward(base_model, data)
        qvals = mjcf_qpos_for_urdf(base_model, data, urdf)
        urdf_frames = urdf.fk(qvals)
        assert body in urdf_frames, f"{body} missing from URDF FK"
        bid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        urdf_frames = apply_base_transform(urdf_frames, data.xpos[bid], data.xquat[bid])
        pos, quat = _mjcf_frame(base_model, data, body)
        pos_err, ang_err = frame_error(urdf_frames[body], pos, quat)
        assert pos_err <= 1e-6, f"{body} trial {trial}: pos err {pos_err}"
        assert ang_err <= 1e-6, f"{body} trial {trial}: ang err {ang_err}"


def test_fk_1000_samples(base_model, urdf):
    """1000 randomized legal states across both arms' TCPs."""
    rng = np.random.default_rng(99)
    data = mujoco.MjData(base_model)
    max_pos = 0.0
    max_ang = 0.0
    for _ in range(1000):
        values = random_valid_qpos(base_model, rng)
        set_urdf_qpos(base_model, data, urdf, values)
        mujoco.mj_forward(base_model, data)
        qvals = mjcf_qpos_for_urdf(base_model, data, urdf)
        frames = urdf.fk(qvals)
        bid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        frames = apply_base_transform(frames, data.xpos[bid], data.xquat[bid])
        for side in ("left", "right"):
            tcp = f"{side}_fr3v2_1_hand_tcp"
            pos, quat = _mjcf_frame(base_model, data, tcp)
            pos_err, ang_err = frame_error(frames[tcp], pos, quat)
            max_pos = max(max_pos, pos_err)
            max_ang = max(max_ang, ang_err)
    assert max_pos <= 1e-6, f"max pos err {max_pos}"
    assert max_ang <= 1e-6, f"max ang err {max_ang}"
