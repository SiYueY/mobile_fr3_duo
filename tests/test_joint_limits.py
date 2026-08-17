"""Joint limit and actuator range validation."""

import math

import mujoco


def test_all_limited_joints_respected(base_model):
    """Keyframe qpos never violates a joint limit."""
    for k in range(base_model.nkey):
        qpos = base_model.key_qpos[k]
        for j in range(base_model.njnt):
            if not base_model.jnt_limited[j]:
                continue
            lo, hi = base_model.jnt_range[j]
            v = qpos[base_model.jnt_qposadr[j]]
            assert lo - 1e-9 <= v <= hi + 1e-9, f"key {k} joint {j} value {v}"


def test_actuator_ctrl_limits(base_model):
    for i in range(base_model.nu):
        lo, hi = base_model.actuator_ctrlrange[i]
        assert lo < hi


def test_arm_effort_limits_match_urdf(base_model):
    """FR3 joint effort limits from the official joint_limits.yaml."""
    expected = {
        "joint1": 87.0,
        "joint2": 87.0,
        "joint3": 87.0,
        "joint4": 87.0,
        "joint5": 12.0,
        "joint6": 12.0,
        "joint7": 12.0,
    }
    for side in ("left", "right"):
        for joint_no, effort in expected.items():
            name = f"{side}_fr3v2_1_{joint_no}"
            jid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            found = False
            for i in range(base_model.nu):
                if base_model.actuator_trnid[i, 0] == jid:
                    lo, hi = base_model.actuator_ctrlrange[i]
                    assert math.isclose(hi, effort, rel_tol=1e-6), f"{name} effort"
                    found = True
            assert found, f"{name} actuator missing"
