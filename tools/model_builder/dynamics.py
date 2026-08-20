"""URDF joint and inertial conversion for native MJCF bodies."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from urdf_common import fmt, fmt_vec, origin_attrib

from . import el


def inertial_mass(inertial: ET.Element) -> float | None:
    """Return a URDF inertial mass, if one is present."""
    mass = inertial.find("mass")
    return float(mass.get("value")) if mass is not None else None


def joint(joint_element: ET.Element) -> ET.Element:
    """Convert one movable URDF joint into its MJCF equivalent."""
    joint_type = joint_element.get("type")
    name = joint_element.get("name")
    if joint_type == "fixed":
        raise ValueError("fixed joints are represented by body nesting, not <joint>")
    mj_type = {"revolute": "hinge", "continuous": "hinge", "prismatic": "slide"}.get(joint_type)
    if mj_type is None:
        raise ValueError(f"unsupported joint type {joint_type}")

    axis = joint_element.find("axis")
    axis_values = np.asarray(
        [float(value) for value in (axis.get("xyz") if axis is not None else "0 0 1").split()]
    )
    # The child body keeps the URDF joint-origin rotation, making the URDF
    # axis already local to the emitted MJCF body.  Rotating it again would
    # reverse the mirrored finger's direction.
    attrs: dict[str, str] = {
        "name": name,
        "type": mj_type,
        "axis": fmt_vec(axis_values),
    }
    limit = joint_element.find("limit")
    if limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
        attrs["limited"] = "true"
        attrs["range"] = f"{fmt(float(limit.get('lower')))} {fmt(float(limit.get('upper')))}"
    dynamics = joint_element.find("dynamics")
    if dynamics is not None:
        if dynamics.get("damping") is not None:
            attrs["damping"] = fmt(float(dynamics.get("damping")))
        if dynamics.get("friction") is not None:
            attrs["frictionloss"] = fmt(float(dynamics.get("friction")))
    return el("joint", **attrs)


def inertial(inertial_element: ET.Element) -> ET.Element:
    """Convert a URDF inertia tensor into MJCF body-frame ``fullinertia``."""
    mass = inertial_mass(inertial_element)
    xyz, quat = _origin(inertial_element)
    inertia_element = inertial_element.find("inertia")
    if inertia_element is None:
        raise ValueError("inertial missing <inertia>")
    tensor = np.array([
        [float(inertia_element.get("ixx")), float(inertia_element.get("ixy")), float(inertia_element.get("ixz"))],
        [float(inertia_element.get("ixy")), float(inertia_element.get("iyy")), float(inertia_element.get("iyz"))],
        [float(inertia_element.get("ixz")), float(inertia_element.get("iyz")), float(inertia_element.get("izz"))],
    ])
    q = np.asarray(quat.split(), dtype=float)
    rotation = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    tensor = rotation @ tensor @ rotation.T
    full = " ".join(fmt(value, 12) for value in (tensor[0, 0], tensor[1, 1], tensor[2, 2], tensor[0, 1], tensor[0, 2], tensor[1, 2]))
    return el("inertial", pos=xyz, mass=fmt(mass, 12), fullinertia=full)


def _origin(element: ET.Element) -> tuple[str, str]:
    xyz, quat = origin_attrib(element)
    return fmt_vec(xyz), fmt_vec(quat)
