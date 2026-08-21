"""Shared test helpers: URDF parsing, analytic FK, model loading."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parent.parent
MODEL_ROOT = REPO_ROOT / "models"
VISUAL_URDF = REPO_ROOT / "source" / "generated" / "mobile_fr3_duo.urdf"


def load(xml_name: str):
    return mujoco.MjModel.from_xml_path(str(MODEL_ROOT / xml_name))


class UrdfModel:
    """Parsed URDF with per-joint transforms for analytic FK."""

    def __init__(self, path: Path = VISUAL_URDF):
        self.root = ET.parse(path).getroot()
        self.links = {el.get("name"): el for el in self.root.findall("link")}
        self.joints: dict[str, ET.Element] = {}
        self.parents: dict[str, str] = {}
        for j in self.root.findall("joint"):
            self.joints[j.get("name")] = j
            self.parents[j.find("child").get("link")] = j.find("parent").get("link")
        children = set(self.parents)
        self.root_link = next(
            n for n in self.links if n not in children
        )

    def joint_origin(self, joint: ET.Element) -> tuple[np.ndarray, Rotation]:
        o = joint.find("origin")
        if o is None:
            return np.zeros(3), Rotation.identity()
        xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
        rpy = np.array([float(v) for v in o.get("rpy", "0 0 0").split()])
        return xyz, Rotation.from_euler("xyz", rpy)

    def fk(self, qpos_by_joint: dict[str, float], base_link: str = "base_link") -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """Compute world frames (pos, rot matrix) for all links."""
        frames: dict[str, tuple[np.ndarray, np.ndarray]] = {
            base_link: (np.zeros(3), np.eye(3))
        }
        order = [base_link]
        while order:
            parent = order.pop(0)
            pos_p, rot_p = frames[parent]
            for j in self.joints.values():
                if j.find("parent").get("link") != parent:
                    continue
                child = j.find("child").get("link")
                xyz, r_origin = self.joint_origin(j)
                jtype = j.get("type")
                r_origin_m = r_origin.as_matrix()
                if jtype in ("revolute", "continuous"):
                    # URDF joint origin is fixed in the parent frame; the
                    # coordinate rotation is applied in that joint frame.
                    # T_parent_child = T_origin @ R_axis(q).
                    axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                    q = qpos_by_joint.get(j.get("name"), 0.0)
                    r_axis = Rotation.from_rotvec(q * axis).as_matrix()
                    rot_child = rot_p @ (r_origin_m @ r_axis)
                    pos_child = pos_p + rot_p @ xyz
                elif jtype == "prismatic":
                    axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                    # Prismatic joints translate along the axis expressed in
                    # the joint frame, after applying the fixed origin.
                    axis = r_origin.apply(axis)
                    q = qpos_by_joint.get(j.get("name"), 0.0)
                    rot_child = rot_p @ r_origin_m
                    pos_child = pos_p + rot_p @ (xyz + q * axis)
                else:
                    rot_child = rot_p @ r_origin_m
                    pos_child = pos_p + rot_p @ xyz
                frames[child] = (pos_child, rot_child)
                order.append(child)
        return frames

    def movable_joints(self) -> list[ET.Element]:
        return [
            j
            for j in self.root.findall("joint")
            if j.get("type") in ("revolute", "continuous", "prismatic")
        ]


def mjcf_qpos_for_urdf(model: mujoco.MjModel, data: mujoco.MjData, urdf: UrdfModel) -> dict[str, float]:
    """Extract URDF joint values from an mjData qpos by name."""
    out = {}
    for j in urdf.movable_joints():
        name = j.get("name")
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            out[name] = float(data.qpos[model.jnt_qposadr[jid]])
    return out


def set_urdf_qpos(model: mujoco.MjModel, data: mujoco.MjData, urdf: UrdfModel, values: dict[str, float]) -> None:
    for name, q in values.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = q


def random_valid_qpos(model: mujoco.MjModel, rng: np.random.Generator) -> dict[str, float]:
    out = {}
    for j in range(model.njnt):
        if not model.jnt_limited[j]:
            continue
        lo, hi = model.jnt_range[j]
        out[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)] = rng.uniform(lo, hi)
    return out


def central_diff_jac(
    model: mujoco.MjModel,
    site_name: str,
    qpos: np.ndarray,
    eps: float = 1e-7,
) -> np.ndarray:
    """Numerical Jacobian of a site position via central differences."""
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    nv = model.nv
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    jac = np.zeros((3, nv))
    for k in range(nv):
        data.qpos[:] = qpos
        data.qvel[:] = 0.0
        data.qvel[k] = eps
        mujoco.mj_integratePos(model, data.qpos, data.qvel, 1.0)
        mujoco.mj_forward(model, data)
        p_plus = data.site_xpos[site_id].copy()
        data.qpos[:] = qpos
        data.qvel[:] = 0.0
        data.qvel[k] = -eps
        mujoco.mj_integratePos(model, data.qpos, data.qvel, 1.0)
        mujoco.mj_forward(model, data)
        p_minus = data.site_xpos[site_id].copy()
        jac[:, k] = (p_plus - p_minus) / (2 * eps)
    return jac


def frame_error(urdf_frame, mjcf_pos, mjcf_quat) -> tuple[float, float]:
    pos_err = float(np.linalg.norm(urdf_frame[0] - mjcf_pos))
    r_urdf = urdf_frame[1]
    r_mj = Rotation.from_quat([mjcf_quat[1], mjcf_quat[2], mjcf_quat[3], mjcf_quat[0]]).as_matrix()
    # rotation angle between the two orientations
    r_rel = r_urdf.T @ r_mj
    angle = float(np.arccos(np.clip((np.trace(r_rel) - 1) / 2, -1, 1)))
    return pos_err, angle


def apply_base_transform(
    frames: dict[str, tuple[np.ndarray, np.ndarray]],
    base_pos: np.ndarray,
    base_quat: np.ndarray,
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Transform URDF frames (relative to base_link) into the world frame."""
    r_base = Rotation.from_quat(
        [base_quat[1], base_quat[2], base_quat[3], base_quat[0]]
    ).as_matrix()
    out = {}
    for name, (pos, rot) in frames.items():
        out[name] = (base_pos + r_base @ pos, r_base @ rot)
    return out
