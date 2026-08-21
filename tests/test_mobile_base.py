"""TMR is a kinematically controlled mobile base."""

import sys

import mujoco

from helpers import REPO_ROOT

sys.path.insert(0, str(REPO_ROOT / "tools"))
from model_builder.kinematic_base import BaseTwist, KinematicBaseController  # noqa: E402


def test_base_has_no_wheel_drive_actuators(base_model):
    for name in ("front_steering_motor", "front_wheel_motor", "rear_steering_motor", "rear_wheel_motor"):
        assert mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) == -1


def test_base_controller_preserves_kinematic_command(scene_model):
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)
    controller = KinematicBaseController(scene_model)
    for _ in range(100):
        mujoco.mj_step(scene_model, data)
        controller.advance(data, BaseTwist(vy=0.15))
    assert data.qpos[1] > 0.01


def test_no_planar_joint_in_formal_model(base_model):
    assert all("planar" not in (mujoco.mj_id2name(base_model, mujoco.mjtObj.mjOBJ_JOINT, i) or "") for i in range(base_model.njnt))
