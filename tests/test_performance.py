"""Performance baselines: step time P95, RTF, model size."""

import time

import mujoco
import numpy as np


def test_model_size_baseline(base_model):
    # The URDF-correct joint transform attaches movable joints directly to
    # their child bodies, eliminating the obsolete 28 intermediate frames.
    # The formal model includes the complete default sensor suite.
    assert base_model.nbody == 109
    assert base_model.njnt == 29
    assert base_model.nv == 34
    # TMR pose is prescribed kinematically, so its four wheel motors are not
    # part of the formal actuator interface.
    assert base_model.nu == 17
    assert base_model.ngeom == 272


def test_step_time_and_rtf(scene_model):
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)
    for _ in range(200):
        mujoco.mj_step(scene_model, data)
    times = []
    for _ in range(1000):
        t0 = time.perf_counter()
        mujoco.mj_step(scene_model, data)
        times.append(time.perf_counter() - t0)
    times = np.asarray(times)
    mean = times.mean()
    p95 = np.percentile(times, 95)
    rtf = scene_model.opt.timestep / mean
    # soft gate: mean step below 5 ms (RTF > 0.2) and P95 below 10 ms
    assert mean < 5e-3, f"mean step {mean * 1e3:.2f} ms too slow"
    assert p95 < 10e-3, f"p95 step {p95 * 1e3:.2f} ms too slow"
    print(f"mean {mean * 1e3:.2f} ms, p95 {p95 * 1e3:.2f} ms, RTF {rtf:.1f}")
