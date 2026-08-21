"""Regression coverage for standalone official D435 and wrist-mount assets."""

import mujoco
import numpy as np

from helpers import load


def test_d435_model_loads_with_official_visual_mesh():
    model = load("realsense_d435/realsense_d435.xml")
    assert model.nmesh == 1
    assert int(model.mesh_facenum[0]) < 200_000
    assert {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(model.ncam)} == {
        "d435_rgb",
        "d435_depth",
    }


def test_d435_camera_contract_and_forward_axis():
    model = load("realsense_d435/realsense_d435.xml")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for name, fovy in (("d435_rgb", 42.5), ("d435_depth", 58.0)):
        camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        assert tuple(model.cam_resolution[camera]) == (1280, 720)
        assert abs(model.cam_fovy[camera] - fovy) < 1e-12
        # MuJoCo's viewing direction (-Z) matches the ROS optical +Z / D435
        # link +X direction after the documented camera-frame conversion.
        forward = data.cam_xmat[camera].reshape(3, 3) @ np.array([0.0, 0.0, -1.0])
        assert np.allclose(forward, [1.0, 0.0, 0.0], atol=1e-12)


def test_wrist_mount_loads_as_visual_only_mesh():
    model = load("wrist_camera_mount/wrist_camera_mount.xml")
    assert model.nmesh == 1
    geom = 0
    assert model.geom_contype[geom] == 0
    assert model.geom_conaffinity[geom] == 0


def test_mobile_duo_exposes_four_figure_fit_d435_cameras():
    model = load("mobile_fr3_duo.xml")
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, index)
        for index in range(model.ncam)
    }
    assert {
        "d435_left_rgb",
        "d435_left_depth",
        "d435_right_rgb",
        "d435_right_depth",
    } <= names
    assert not any(
        model.geom_group[index] == 4 and model.geom_contype[index] != 0
        for index in range(model.ngeom)
    )
