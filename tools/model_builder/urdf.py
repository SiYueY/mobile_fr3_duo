"""URDF parsing and traversal primitives used by the MJCF builder."""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from pathlib import Path


class UrdfModel:
    """Parsed URDF structure needed for MJCF generation."""

    def __init__(self, urdf_path: Path):
        self.root = ET.parse(urdf_path).getroot()
        self.links: dict[str, ET.Element] = {
            el.get("name"): el for el in self.root.findall("link")
        }
        self.joints: dict[str, ET.Element] = {
            joint.get("name"): joint for joint in self.root.findall("joint")
        }
        children = {joint.find("child").get("link") for joint in self.joints.values()}
        self.root_link = next(name for name in self.links if name not in children)
        self.child_to_joint = {
            joint.find("child").get("link"): joint for joint in self.joints.values()
        }


def merge_sc_links(base: UrdfModel, self_collision: UrdfModel) -> UrdfModel:
    """Merge ``*_sc`` self-collision bodies from a --with-sc URDF into base."""
    merged = copy.deepcopy(base)
    for name, joint in self_collision.joints.items():
        child = joint.find("child").get("link")
        if child.endswith("_sc"):
            merged.joints[name] = joint
            merged.links[child] = self_collision.links[child]
            merged.child_to_joint[child] = joint
    return merged


def children(model: UrdfModel, parent: str) -> list[str]:
    """Return child links in their source URDF joint order."""
    return [
        joint.find("child").get("link")
        for joint in model.joints.values()
        if joint.find("parent").get("link") == parent
    ]


def active_joints_in_order(model: UrdfModel, planar: bool) -> list[str]:
    """Return non-fixed joints in the same order MuJoCo assigns DOFs."""
    order = ["planar_x_joint", "planar_y_joint", "planar_yaw_joint"] if planar else []

    def walk(name: str) -> None:
        for joint in model.joints.values():
            if joint.find("parent").get("link") != name:
                continue
            if joint.get("type") != "fixed":
                order.append(joint.get("name"))
            walk(joint.find("child").get("link"))

    walk(model.root_link)
    return order
