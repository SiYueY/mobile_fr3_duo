"""Structural consistency between URDF and MJCF."""

import math

import mujoco
import numpy as np
import pytest
from helpers import apply_base_transform, mjcf_qpos_for_urdf
from scipy.spatial.transform import Rotation


def test_joint_counts(base_model, urdf):
    urdf_movable = [j for j in urdf.root.findall("joint") if j.get("type") != "fixed"]
    mjcf_movable = [
        mujoco.mj_id2name(base_model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(base_model.njnt)
        if base_model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
    ]
    assert len(mjcf_movable) == len(urdf_movable)
    urdf_names = {j.get("name") for j in urdf_movable}
    assert set(mjcf_movable) == urdf_names


def test_joint_types_axes_ranges(base_model, urdf):
    for j in urdf.root.findall("joint"):
        if j.get("type") == "fixed":
            continue
        name = j.get("name")
        jid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert jid >= 0, f"missing joint {name}"
        urdf_type = j.get("type")
        if urdf_type == "prismatic":
            assert base_model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_SLIDE
        else:
            assert base_model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE
        # The MJCF writes the URDF joint axis as-is on the intermediate
        # joint-frame body (rotation-first convention, matching KDL);
        # prismatic finger joints use the origin-rotated axis to keep the
        # mirrored finger motion.
        axis_urdf = np.array([float(v) for v in j.find("axis").get("xyz").split()])
        if urdf_type == "prismatic":
            origin = j.find("origin")
            rpy = (
                [float(v) for v in origin.get("rpy").split()]
                if origin is not None and origin.get("rpy")
                else [0.0, 0.0, 0.0]
            )
            expected = Rotation.from_euler("xyz", rpy).apply(axis_urdf)
        else:
            expected = axis_urdf
        assert np.allclose(base_model.jnt_axis[jid], expected, atol=1e-9)
        limit = j.find("limit")
        if limit is not None and limit.get("lower") is not None:
            assert base_model.jnt_limited[jid]
            assert math.isclose(
                base_model.jnt_range[jid][0], float(limit.get("lower")), rel_tol=1e-9
            )
            assert math.isclose(
                base_model.jnt_range[jid][1], float(limit.get("upper")), rel_tol=1e-9
            )


def test_actuator_effort_matches_urdf(base_model, urdf):
    """URDF joint effort maps to the motor actuator ctrl range."""
    for j in urdf.root.findall("joint"):
        limit = j.find("limit")
        if limit is None or limit.get("effort") is None:
            continue
        name = j.get("name")
        if (
            name.startswith("tmrv0_2_joint")
            or "caster" in name
            or "rocker" in name
            or "finger" in name
            or name == "franka_spine_vertical_joint"
        ):
            # TMR/hand/spine actuator ranges are simulation-only calibrations,
            # not the URDF gear-level efforts (see
            # source/parameter_sources.yaml).
            continue
        effort = float(limit.get("effort"))
        found = False
        for i in range(base_model.nu):
            if base_model.actuator_trnid[i, 0] == mujoco.mj_name2id(
                base_model, mujoco.mjtObj.mjOBJ_JOINT, name
            ):
                lo, hi = base_model.actuator_ctrlrange[i]
                assert math.isclose(hi, effort, rel_tol=1e-6), f"{name} effort"
                assert math.isclose(lo, -effort, rel_tol=1e-6), f"{name} effort"
                found = True
        if name.startswith(("left_", "right_")) and "finger" not in name:
            assert found, f"{name} has no actuator"


def test_keyframes_within_limits(base_model):
    for k in range(base_model.nkey):
        qpos = base_model.key_qpos[k]
        for j in range(base_model.njnt):
            if not base_model.jnt_limited[j]:
                continue
            adr = base_model.jnt_qposadr[j]
            assert base_model.jnt_range[j][0] - 1e-9 <= qpos[adr] <= base_model.jnt_range[j][1] + 1e-9


@pytest.mark.parametrize(
    "site",
    [
        "left_fr3v2_1_hand_tcp",
        "right_fr3v2_1_hand_tcp",
        "imu_mounting_point",
        "front_mounting_point",
        "rear_mounting_point",
        "left_mounting_point",
        "right_mounting_point",
        "lidar_front_mounting_point",
        "lidar_rear_mounting_point",
        "franka_spine_mounting_point",
        "fr3_duo_mount_mounting_point",
        "head_camera_mounting_point",
    ],
)
def test_mounting_points_exist(base_model, site):
    assert mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_SITE, site) >= 0


def test_tcp_matches_urdf(base_model, urdf):
    """TCP site frame equals the URDF tcp frame at every keyframe."""
    data = mujoco.MjData(base_model)
    for k in range(base_model.nkey):
        mujoco.mj_resetDataKeyframe(base_model, data, k)
        mujoco.mj_forward(base_model, data)
        qvals = mjcf_qpos_for_urdf(base_model, data, urdf)
        frames = urdf.fk(qvals)
        bid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        frames = apply_base_transform(frames, data.xpos[bid], data.xquat[bid])
        for side in ("left", "right"):
            tcp = f"{side}_fr3v2_1_hand_tcp"
            sid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_SITE, tcp)
            urdf_frame = frames[tcp]
            pos_err = np.linalg.norm(urdf_frame[0] - data.site_xpos[sid])
            assert pos_err < 1e-6, f"{tcp} keyframe {k}: pos err {pos_err}"
