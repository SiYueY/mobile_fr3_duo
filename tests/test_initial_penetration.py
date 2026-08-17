"""Keyframe initial-penetration checks."""

import mujoco


def _arm_body_ids(model):
    ids = set()
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
        if (
            "fr3v2_1" in name
            or "hand" in name
            or "finger" in name
            or "franka_spine" in name
            or "fr3_duo_mount" in name
            or "head" in name
        ):
            ids.add(i)
    return ids


def test_keyframes_no_deep_penetration(scene_model):
    data = mujoco.MjData(scene_model)
    arm_ids = _arm_body_ids(scene_model)
    for k in range(scene_model.nkey):
        name = mujoco.mj_id2name(scene_model, mujoco.mjtObj.mjOBJ_KEY, k)
        mujoco.mj_resetDataKeyframe(scene_model, data, k)
        mujoco.mj_forward(scene_model, data)
        worst = 0.0
        for c in range(data.ncon):
            b1 = scene_model.geom_bodyid[data.contact[c].geom1]
            b2 = scene_model.geom_bodyid[data.contact[c].geom2]
            if b1 in arm_ids and b2 in arm_ids:
                worst = min(worst, float(data.contact[c].dist))
        assert worst >= -0.005, f"keyframe {name}: deep penetration {worst}"


def test_keyframes_within_joint_limits(scene_model):
    data = mujoco.MjData(scene_model)
    for k in range(scene_model.nkey):
        mujoco.mj_resetDataKeyframe(scene_model, data, k)
        for j in range(scene_model.njnt):
            if not scene_model.jnt_limited[j]:
                continue
            lo, hi = scene_model.jnt_range[j]
            v = data.qpos[scene_model.jnt_qposadr[j]]
            assert lo - 1e-9 <= v <= hi + 1e-9
