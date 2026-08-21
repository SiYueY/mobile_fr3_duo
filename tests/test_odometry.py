"""Kinematic TMR odometry is exact by construction."""

import sys

import mujoco
import numpy as np

from helpers import REPO_ROOT

sys.path.insert(0, str(REPO_ROOT / "tools"))
from model_builder.kinematic_base import BaseTwist, KinematicBaseController  # noqa: E402


def test_kinematic_twist_integrates_base_pose(scene_model):
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)
    controller = KinematicBaseController(scene_model)
    start = data.qpos[:3].copy()
    for _ in range(1000):
        controller.advance(data, BaseTwist(vx=0.2))
    assert np.allclose(data.qpos[:2] - start[:2], (0.2, 0.0), atol=2e-3)


def test_kinematic_turn_updates_heading_and_wheel_state(scene_model):
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)
    controller = KinematicBaseController(scene_model)
    for _ in range(500):
        controller.advance(data, BaseTwist(vx=0.1, yaw_rate=0.4))
    assert abs(data.qpos[3] - 1.0) > 1e-3
    wheel = mujoco.mj_name2id(scene_model, mujoco.mjtObj.mjOBJ_JOINT, "tmrv0_2_joint_1")
    assert abs(data.qvel[scene_model.jnt_dofadr[wheel]]) > 0
