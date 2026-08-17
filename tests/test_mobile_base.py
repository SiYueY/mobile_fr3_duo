"""TMR mobile base behavior: motion comes from wheel-ground contact."""

import mujoco
import numpy as np


def _aid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def _run_drive(scene_model, ctrl, steps=2000, keyframe=0):
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, keyframe)
    for _ in range(500):  # settle
        mujoco.mj_step(scene_model, data)
    for name, value in ctrl.items():
        data.ctrl[_aid(scene_model, name)] = value
    x0 = data.xpos[1].copy()
    yaw0 = data.qpos[3:7].copy()
    for _ in range(steps):
        mujoco.mj_step(scene_model, data)
    dx = data.xpos[1] - x0
    return dx, data, yaw0


def _ground_contacts(model, data):
    n = 0
    for c in range(data.ncon):
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[c].geom1)
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[c].geom2)
        if g1 == "ground" or g2 == "ground":
            n += 1
    return n


def test_forward_drive(scene_model):
    dx, data, _ = _run_drive(
        scene_model,
        {"front_wheel_motor": 40, "rear_wheel_motor": 40},
    )
    assert dx[0] > 0.1, f"expected forward motion, got {dx}"
    assert _ground_contacts(scene_model, data) >= 2


def test_backward_drive(scene_model):
    dx, _, _ = _run_drive(
        scene_model,
        {"front_wheel_motor": -40, "rear_wheel_motor": -40},
    )
    assert dx[0] < -0.1, f"expected backward motion, got {dx}"


def test_no_planar_joint_in_official_model(base_model):
    for j in range(base_model.njnt):
        name = mujoco.mj_id2name(base_model, mujoco.mjtObj.mjOBJ_JOINT, j)
        assert "planar" not in (name or "")


def test_stationary_drift_small(scene_model):
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)
    for _ in range(2000):
        mujoco.mj_step(scene_model, data)
    x0 = data.xpos[1].copy()
    for _ in range(3000):
        mujoco.mj_step(scene_model, data)
    drift = np.linalg.norm(data.xpos[1][:2] - x0[:2])
    assert drift < 0.05, f"unexpected drift {drift}"


def test_brake_after_drive(scene_model):
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)
    for _ in range(500):
        mujoco.mj_step(scene_model, data)
    data.ctrl[_aid(scene_model, "front_wheel_motor")] = 40
    data.ctrl[_aid(scene_model, "rear_wheel_motor")] = 40
    for _ in range(1500):
        mujoco.mj_step(scene_model, data)
    speed = np.linalg.norm(data.qvel[6:9])  # base linear velocity
    data.ctrl[_aid(scene_model, "front_wheel_motor")] = 0
    data.ctrl[_aid(scene_model, "rear_wheel_motor")] = 0
    for _ in range(1500):
        mujoco.mj_step(scene_model, data)
    speed_after = np.linalg.norm(data.qvel[6:9])
    assert speed_after < speed * 0.5, (speed, speed_after)
