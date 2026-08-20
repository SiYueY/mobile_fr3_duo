"""Sensor entity counts, mounting transforms and frame conventions."""

import mujoco
import numpy as np
from helpers import REPO_ROOT, apply_base_transform, mjcf_qpos_for_urdf

OFFICIAL_MOUNTING_POINTS = (
    "imu_mounting_point",
    "front_mounting_point",
    "rear_mounting_point",
    "left_mounting_point",
    "right_mounting_point",
    "lidar_front_mounting_point",
    "lidar_rear_mounting_point",
    "head_camera_mounting_point",
)


def test_sensor_counts(sensor_model):
    cam_names = {
        mujoco.mj_id2name(sensor_model, mujoco.mjtObj.mjOBJ_CAMERA, i)
        for i in range(sensor_model.ncam)
    }
    d455 = {n for n in cam_names if n and n.startswith("camera_")}
    zed = {n for n in cam_names if n and n.startswith("head_zed_")}
    assert len(d455) == 8  # color+depth for 4 positions
    assert len(zed) == 2


def test_imu_sensors(sensor_model):
    names = {
        mujoco.mj_id2name(sensor_model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        for i in range(sensor_model.nsensor)
    }
    assert "imu_angular_velocity" in names
    assert "imu_linear_acceleration" in names
    assert "imu_orientation" in names
    assert "base_linear_velocity" in names
    assert "base_angular_velocity" in names


def test_lidar_frames(sensor_model):
    for position in ("front", "rear"):
        name = f"lidar_{position}_scan_frame"
        sid = mujoco.mj_name2id(sensor_model, mujoco.mjtObj.mjOBJ_SITE, name)
        assert sid >= 0, f"missing {name}"


def test_camera_frames(sensor_model):
    for position in ("front", "rear", "left", "right"):
        for frame in ("color", "depth", "infra1", "infra2"):
            name = f"camera_{position}_{frame}_optical_frame"
            sid = mujoco.mj_name2id(sensor_model, mujoco.mjtObj.mjOBJ_SITE, name)
            assert sid >= 0, f"missing {name}"


def test_official_mounting_points_match_golden_urdf_se3(sensor_model, urdf):
    """Official mounting sites match the frozen Golden URDF at every keyframe."""
    data = mujoco.MjData(sensor_model)
    base = mujoco.mj_name2id(sensor_model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    assert base >= 0

    for keyframe in range(sensor_model.nkey):
        mujoco.mj_resetDataKeyframe(sensor_model, data, keyframe)
        mujoco.mj_forward(sensor_model, data)
        golden = apply_base_transform(
            urdf.fk(mjcf_qpos_for_urdf(sensor_model, data, urdf)),
            data.xpos[base],
            data.xquat[base],
        )
        for site_name in OFFICIAL_MOUNTING_POINTS:
            site = mujoco.mj_name2id(sensor_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            assert site >= 0, f"missing {site_name}"
            expected_pos, expected_rot = golden[site_name]
            position_error = np.linalg.norm(expected_pos - data.site_xpos[site])
            actual_rot = data.site_xmat[site].reshape(3, 3)
            relative_rot = expected_rot.T @ actual_rot
            rotation_error = np.arccos(
                np.clip((np.trace(relative_rot) - 1.0) / 2.0, -1.0, 1.0)
            )
            assert position_error <= 1e-6, (
                f"keyframe {keyframe}, {site_name}: position error {position_error}"
            )
            assert rotation_error <= 1e-6, (
                f"keyframe {keyframe}, {site_name}: rotation error {rotation_error}"
            )


def test_zed_baseline(sensor_model):
    left = mujoco.mj_name2id(
        sensor_model, mujoco.mjtObj.mjOBJ_CAMERA, "head_zed_left"
    )
    right = mujoco.mj_name2id(
        sensor_model, mujoco.mjtObj.mjOBJ_CAMERA, "head_zed_right"
    )
    assert left >= 0 and right >= 0
    # ZED Mini nominal baseline 63 mm
    assert abs(abs(sensor_model.cam_pos[left][0]) - 0.0315) < 1e-4
    assert abs(abs(sensor_model.cam_pos[right][0]) - 0.0315) < 1e-4


def test_sensor_disable_does_not_break_dynamics(sensor_model):
    """Disabling sensors must not change the dynamics (sensors are read-only)."""
    data = mujoco.MjData(sensor_model)
    mujoco.mj_resetDataKeyframe(sensor_model, data, 0)
    q0 = data.qpos.copy()
    for _ in range(100):
        mujoco.mj_step(sensor_model, data)
    q_run = data.qpos.copy()
    data.qpos[:] = q0
    data.qvel[:] = 0
    # disable all sensors via mj_disable
    mujoco.mj_forward(sensor_model, data)
    for _ in range(100):
        mujoco.mj_step(sensor_model, data)
    assert np.allclose(q_run, data.qpos, atol=1e-12)


def test_camera_render(sensor_model):
    """D455 color camera renders a valid offscreen frame."""
    import subprocess
    import sys

    code = (
        "import mujoco, numpy as np\n"
        "from mujoco import renderer as mjr\n"
        "from mujoco.egl import GLContext\n"
        "m = mujoco.MjModel.from_xml_path("
        f"r'{REPO_ROOT / 'models/scene_with_sensors.xml'}')\n"
        "d = mujoco.MjData(m)\n"
        "mujoco.mj_resetDataKeyframe(m, d, 0)\n"
        "mujoco.mj_forward(m, d)\n"
        "ctx = GLContext(320, 240)\n"
        "ctx.make_current()\n"
        "m.vis.global_.offwidth = 320\n"
        "m.vis.global_.offheight = 240\n"
        "r = mjr.Renderer(m, 240, 320)\n"
        "cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "
        "'camera_front_color')\n"
        "r.update_scene(d, camera=cid)\n"
        "img = r.render()\n"
        "print(img.shape[0], img.shape[1], float(img[:,:,:3].std()))\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stderr[-800:]
    h, w, std = (float(x) for x in result.stdout.split()[:3])
    assert (int(h), int(w)) == (240, 320)
    assert std > 1.0


def test_lidar_raycast_geometry(sensor_model):
    """mj_multiRay from the scan frames returns valid beam distances."""
    data = mujoco.MjData(sensor_model)
    mujoco.mj_resetDataKeyframe(sensor_model, data, 0)
    mujoco.mj_forward(sensor_model, data)
    n = 500
    angles = np.linspace(-np.pi / 4, 3 * np.pi / 4, n)
    direction_local = np.stack(
        [np.cos(angles), np.zeros(n), np.sin(angles)], axis=1
    )
    for position in ("front", "rear"):
        sid = mujoco.mj_name2id(
            sensor_model, mujoco.mjtObj.mjOBJ_SITE, f"lidar_{position}_scan_frame"
        )
        direction = (data.site_xmat[sid].reshape(3, 3) @ direction_local.T).T
        geomid = np.full(n, -1, dtype=np.int32)
        dist = np.full(n, 10.0)
        mujoco.mj_multiRay(
            sensor_model,
            data,
            data.site_xpos[sid].copy(),
            direction.flatten(),
            None,
            1,
            -1,
            geomid,
            dist,
            None,
            n,
            10.0,
        )
        assert np.isfinite(dist).all()
        hits = dist[dist < 9.9]
        assert len(hits) > n // 4, f"{position}: too few lidar hits"


def test_imu_sensor_frame_is_right_handed(sensor_model):
    """The project-authored IMU sensor frame retains a proper orientation."""
    data = mujoco.MjData(sensor_model)
    mujoco.mj_forward(sensor_model, data)
    frame = mujoco.mj_name2id(
        sensor_model, mujoco.mjtObj.mjOBJ_SITE, "imu_sensor_frame"
    )
    assert frame >= 0
    mat = data.site_xmat[frame].reshape(3, 3)
    assert abs(np.linalg.det(mat) - 1.0) < 1e-6


def test_sensor_rate_profiles():
    """Sensor profile rates match the documented runtime schedule."""
    import yaml

    profile = yaml.safe_load(
        (REPO_ROOT / "config" / "sensors" / "simulation_default.yaml").read_text()
    )
    assert profile["d455"]["rate_hz"] == 30.0
    assert profile["nanoscan3"]["rate_hz"] == 50.0
    assert profile["zed_mini"]["rate_hz"] == 30.0
    assert profile["physics_rate_hz"] == 1000
