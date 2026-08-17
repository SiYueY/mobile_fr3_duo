"""Long-horizon stability: no NaN, bounded energy, no runaway drift."""

import os

import mujoco
import numpy as np
import pytest

KEYFRAMES = ("home", "transport", "manipulation", "spine_min", "spine_max")


def _run(model, data, steps, ctrl_zero=True):
    max_vel = 0.0
    for _ in range(steps):
        if ctrl_zero:
            data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise AssertionError("NaN/Inf in state")
        max_vel = max(max_vel, float(np.abs(data.qvel).max()))
    return max_vel


@pytest.mark.parametrize("keyframe", KEYFRAMES)
def test_keyframe_60s_stable(scene_model, keyframe):
    data = mujoco.MjData(scene_model)
    kid = mujoco.mj_name2id(scene_model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
    mujoco.mj_resetDataKeyframe(scene_model, data, kid)
    x0 = data.xpos[1].copy()
    max_vel = _run(scene_model, data, 60000)
    assert max_vel < 50, f"unbounded velocity {max_vel}"
    drift = np.linalg.norm(data.xpos[1][:2] - x0[:2])
    assert drift < 0.5, f"excessive base drift {drift}"


def test_ten_minute_integration(scene_model):
    """10-minute full integration at 1 kHz (600k steps)."""
    if os.environ.get("MOBILE_FR3_QUICK") == "1":
        pytest.skip("quick mode")
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)
    max_vel = _run(scene_model, data, 600000)
    assert max_vel < 50
