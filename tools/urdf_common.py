"""Shared URDF parsing helpers used by the model build pipeline."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def parse_vec(text: str, size: int = 3) -> np.ndarray:
    return np.array([float(x) for x in text.split()][:size])


def rpy_to_quat(xyz_rpy: tuple[list[float] | None, list[float] | None]) -> np.ndarray:
    """URDF origin rpy (fixed-axis XYZ) to MuJoCo quaternion (w x y z)."""
    _, rpy = xyz_rpy
    r = Rotation.from_euler("xyz", np.asarray(rpy or [0.0, 0.0, 0.0]))
    q = r.as_quat()  # x y z w
    return np.array([q[3], q[0], q[1], q[2]])


def origin_attrib(elem: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    """Return (xyz, quat) for a URDF element's <origin> (defaults identity)."""
    origin = elem.find("origin")
    if origin is None:
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    xyz = parse_vec(origin.get("xyz", "0 0 0"))
    rpy = parse_vec(origin.get("rpy", "0 0 0"))
    return xyz, rpy_to_quat((None, rpy.tolist()))


def fmt(v: float, ndigits: int = 10) -> str:
    """Format a float compactly with fixed precision."""
    if v == 0:
        return "0"
    s = f"{v:.{ndigits}g}"
    return s


def fmt_vec(v: np.ndarray, ndigits: int = 10) -> str:
    return " ".join(fmt(float(x), ndigits) for x in v)


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of quaternions (w x y z)."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def parse_urdf(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def load_links_and_joints(
    urdf: Path,
) -> tuple[dict[str, ET.Element], dict[str, ET.Element], str]:
    root = parse_urdf(urdf)
    links = {el.get("name"): el for el in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}
    # URDF root link: referenced as a joint parent but never as a child.
    children = {j.find("child").get("link") for j in root.findall("joint")}
    roots = [n for n in links if n not in children]
    return links, joints, roots[0] if roots else None


def total_mass(urdf: Path) -> float:
    links, _, _ = load_links_and_joints(urdf)
    total = 0.0
    for link in links.values():
        inertial = link.find("inertial")
        if inertial is not None:
            total += float(inertial.get("mass"))
    return total


def degrees_to_radians(v: float) -> float:
    return v * math.pi / 180.0
