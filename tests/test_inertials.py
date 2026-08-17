"""Inertia validation against the official URDF."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from helpers import VISUAL_URDF
from scipy.spatial.transform import Rotation


def _urdf_inertia(link):
    inertial = link.find("inertial")
    if inertial is None:
        return None
    mass = float(inertial.find("mass").get("value"))
    o = inertial.find("origin")
    xyz = (
        np.array([float(v) for v in o.get("xyz").split()])
        if o is not None and o.get("xyz")
        else np.zeros(3)
    )
    rpy = (
        np.array([float(v) for v in o.get("rpy").split()])
        if o is not None and o.get("rpy")
        else np.zeros(3)
    )
    inertia = inertial.find("inertia")
    ixx = float(inertia.get("ixx"))
    iyy = float(inertia.get("iyy"))
    izz = float(inertia.get("izz"))
    ixy = float(inertia.get("ixy"))
    ixz = float(inertia.get("ixz"))
    iyz = float(inertia.get("iyz"))
    return mass, xyz, rpy, np.array(
        [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
    )


def test_total_mass_matches(base_model):
    urdf_root = ET.parse(VISUAL_URDF).getroot()
    urdf_mass = sum(
        float(link_el.find("inertial").find("mass").get("value"))
        for link_el in urdf_root.findall("link")
        if link_el.find("inertial") is not None
    )
    mjcf_mass = float(np.sum(base_model.body_mass))
    assert abs(mjcf_mass - urdf_mass) < 1e-6, (mjcf_mass, urdf_mass)


def test_per_body_inertia_matches(base_model):
    urdf_root = ET.parse(VISUAL_URDF).getroot()
    joints = {j.find("child").get("link"): j for j in urdf_root.findall("joint")}
    for link in urdf_root.findall("link"):
        data = _urdf_inertia(link)
        if data is None:
            continue
        mass, xyz, rpy, i_urdf = data
        name = link.get("name")
        bid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, name)
        r_origin = np.eye(3)
        if bid < 0 or base_model.body_mass[bid] == 0.0:
            bid = mujoco.mj_name2id(
                base_model, mujoco.mjtObj.mjOBJ_BODY, f"{name}_joint_frame"
            )
            if bid < 0:
                continue
            # Inertials of movable links live on the intermediate joint-frame
            # body; the COM is rotated by the joint origin rotation.
            joint = joints.get(name)
            origin = joint.find("origin") if joint is not None else None
            if origin is not None and origin.get("rpy"):
                r_origin = Rotation.from_euler(
                    "xyz", [float(v) for v in origin.get("rpy").split()]
                ).as_matrix()
            xyz = r_origin @ xyz
        assert abs(base_model.body_mass[bid] - mass) < 1e-9
        assert np.allclose(base_model.body_ipos[bid], xyz, atol=1e-9)
        # fullinertia is expressed in the body frame: rotate URDF tensor
        r = Rotation.from_euler("xyz", rpy).as_matrix()
        i_rot = r @ i_urdf @ r.T
        # MuJoCo reorients body frames to the principal axes of inertia and
        # exposes only the diagonal (nbody, 3); compare with the eigenvalues
        # of the rotated URDF tensor.
        eig_mj = np.sort(base_model.body_inertia[bid])
        eig_urdf = np.sort(np.linalg.eigvalsh(i_rot))
        assert np.allclose(eig_mj, eig_urdf, atol=1e-9), f"{name} inertia"


def test_inertia_positive_definite(base_model):
    for i in range(base_model.nbody):
        if base_model.body_mass[i] <= 0:
            continue
        eig = np.sort(base_model.body_inertia[i])
        assert (eig > 0).all(), f"body {i} not positive definite"
        a, b, c = eig
        assert a + b >= c - 1e-9
        assert a + c >= b - 1e-9
        assert b + c >= a - 1e-9
