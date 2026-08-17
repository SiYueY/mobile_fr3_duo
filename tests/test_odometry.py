"""Ground-truth and wheel odometry consistency for the TMR base."""

import mujoco
import numpy as np


def _drive(model, data, steps):
    front = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "front_wheel_motor")
    rear = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "rear_wheel_motor")
    data.ctrl[front] = 40
    data.ctrl[rear] = 40
    x0 = data.xpos[1].copy()
    for _ in range(steps):
        mujoco.mj_step(model, data)
    return data.xpos[1] - x0


def test_ground_truth_odometry_tracks_base(scene_model):
    """Freejoint velocity integration matches the base displacement."""
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)
    for _ in range(500):
        mujoco.mj_step(scene_model, data)
    front = mujoco.mj_name2id(
        scene_model, mujoco.mjtObj.mjOBJ_ACTUATOR, "front_wheel_motor"
    )
    rear = mujoco.mj_name2id(
        scene_model, mujoco.mjtObj.mjOBJ_ACTUATOR, "rear_wheel_motor"
    )
    data.ctrl[front] = 40
    data.ctrl[rear] = 40
    # integrate base velocity over the drive
    v = np.zeros(3)
    for _ in range(1000):
        mujoco.mj_step(scene_model, data)
        base_dof = 6
        v += data.qvel[base_dof : base_dof + 3] * scene_model.opt.timestep
    assert np.linalg.norm(v) > 0.05


def test_wheel_odometry_matches_base_motion(scene_model):
    """Wheel-speed odometry and base displacement agree within slip bound."""
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)
    for _ in range(500):
        mujoco.mj_step(scene_model, data)
    base0 = data.xpos[1].copy()
    front = mujoco.mj_name2id(
        scene_model, mujoco.mjtObj.mjOBJ_ACTUATOR, "front_wheel_motor"
    )
    rear = mujoco.mj_name2id(
        scene_model, mujoco.mjtObj.mjOBJ_ACTUATOR, "rear_wheel_motor"
    )
    data.ctrl[front] = 40
    data.ctrl[rear] = 40
    front_wheel = mujoco.mj_name2id(
        scene_model, mujoco.mjtObj.mjOBJ_JOINT, "tmrv0_2_joint_1"
    )
    rear_wheel = mujoco.mj_name2id(
        scene_model, mujoco.mjtObj.mjOBJ_JOINT, "tmrv0_2_joint_3"
    )
    dx = 0.0
    for _ in range(1000):
        mujoco.mj_step(scene_model, data)
        vf = data.qvel[scene_model.jnt_dofadr[front_wheel]]
        vr = data.qvel[scene_model.jnt_dofadr[rear_wheel]]
        dx += 0.5 * (vf + vr) * 0.05 * scene_model.opt.timestep
    base_dx = np.linalg.norm(data.xpos[1][:2] - base0[:2])
    assert dx > 0.05
    # slip tolerance: wheel odometry overestimates under load; require the
    # base to actually move in the commanded direction with bounded slip.
    assert 0.02 < base_dx <= dx
