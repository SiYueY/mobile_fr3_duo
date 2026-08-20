"""Compose runtime MJCF modules and save flattened release XML."""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import yaml
from format_xml import format_element

KNOWN_MODULES = {
    "franka_tmr", "franka_spine", "franka_head", "franka_fr3", "franka_hand",
    "imu", "d455", "nanoscan3", "zed_mini",
}
MOUNT_TARGETS = {
    "imu_mounting_point",
    "front_mounting_point",
    "rear_mounting_point",
    "left_mounting_point",
    "right_mounting_point",
    "lidar_front_mounting_point",
    "lidar_rear_mounting_point",
    "head_camera_mounting_point",
}


def load_robot_config(path: Path) -> dict:
    """Validate the composition contract before generating any XML."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("robot"), dict):
        raise ValueError(f"invalid robot configuration: {path}")
    selected = set(data["robot"].values())
    for arm in data.get("arms", {}).values():
        if not isinstance(arm, dict):
            raise ValueError("each arm configuration must be a mapping")
        selected.update((arm.get("model"), arm.get("end_effector")))
    prefixes = [arm.get("prefix") for arm in data.get("arms", {}).values()]
    if len(prefixes) != len(set(prefixes)) or any(not isinstance(prefix, str) or not prefix for prefix in prefixes):
        raise ValueError("arm prefixes must be unique non-empty strings")
    for sensor in data.get("sensors", []):
        if not isinstance(sensor, dict):
            raise ValueError("each sensor configuration must be a mapping")
        selected.add(sensor.get("model"))
        mounts = sensor.get("mounts", [sensor.get("mount")])
        if not all(isinstance(mount, str) and mount in MOUNT_TARGETS for mount in mounts):
            raise ValueError(f"invalid sensor mount in {sensor}")
    unknown = {item for item in selected if not isinstance(item, str) or item not in KNOWN_MODULES}
    if unknown:
        raise ValueError(f"unknown runtime modules: {', '.join(sorted(map(str, unknown)))}")
    return data


def compose(source: ET.Element, out: Path, *, base_module: str) -> ET.Element:
    """Create a runtime attach composition while retaining robot-level logic."""
    root = ET.Element("mujoco", model=source.get("model"))
    compiler = copy.deepcopy(source.find("compiler"))
    # The module asset paths are relative to this composition file.  They stay
    # valid when a scene includes the robot XML from the same directory.
    compiler.set("meshdir", ".")
    root.append(compiler)
    for tag in ("option", "default"):
        root.append(copy.deepcopy(source.find(tag)))
    asset = ET.Element("asset")
    asset.append(ET.Element("model", name="mobile_base", file=base_module))
    root.append(asset)
    worldbody = ET.Element("worldbody")
    worldbody.append(ET.Element("attach", model="mobile_base", body="base_link", prefix=""))
    root.append(worldbody)
    for tag in ("contact", "equality", "actuator", "sensor", "keyframe"):
        root.append(copy.deepcopy(source.find(tag)))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(format_element(root), encoding="utf-8")
    return root


def flatten(composition_path: Path, output_path: Path) -> None:
    """Let MuJoCo expand attach meta-elements into a standalone XML release."""
    spec = mujoco.MjSpec.from_file(str(composition_path))
    spec.compile()
    flattened = ET.fromstring(spec.to_xml())
    _normalize_attached_defaults(flattened.find("default"))
    _rewrite_flattened_mesh_paths(flattened)
    output_path.write_text(format_element(flattened), encoding="utf-8")
    mujoco.MjModel.from_xml_path(str(output_path))


def _rewrite_flattened_mesh_paths(root: ET.Element) -> None:
    """Restore module-owned resource paths lost by MuJoCo attach flattening."""
    owners = (
        (("tmrv0_2",), "franka_tmr/assets/"),
        (("franka_spine",), "franka_spine/assets/"),
        (("fr3_duo", "franka_head"), "franka_head/assets/"),
        (("left_link", "right_link"), "franka_fr3/assets/"),
        (("left_hand", "right_hand", "left_finger", "right_finger"), "franka_hand/assets/"),
        (("d455",), "sensors/d455/assets/"),
        (("nanoscan3",), "sensors/nanoscan3/assets/"),
        (("zed_mini",), "sensors/zed_mini/assets/"),
    )
    compiler = root.find("compiler")
    if compiler is not None:
        compiler.set("meshdir", ".")
    for mesh in root.findall("./asset/mesh"):
        name = mesh.get("name", "")
        path = mesh.get("file", "")
        owner = next((target for tokens, target in owners if any(token in name for token in tokens)), None)
        if owner is None:
            raise ValueError(f"cannot determine flattened mesh owner: {name}")
        mesh.set("file", owner + path)


def _normalize_attached_defaults(default: ET.Element | None) -> None:
    """Repair nested anonymous defaults emitted by repeated XML attach.

    MuJoCo 3.9 compiles the composition correctly but serializes child module
    defaults as anonymous nested ``<default>`` elements, which are not valid
    when the saved XML is loaded again.  Hoist unique named defaults and drop
    duplicate definitions; all attached geom classes remain intact.
    """
    if default is None:
        return
    seen: set[str] = set()

    def add(parent: ET.Element, child: ET.Element, index: int | None = None) -> None:
        """Add one default, flattening anonymous wrappers on the way."""
        if child.get("class") is None:
            for nested in list(child):
                if nested.tag == "default":
                    add(parent, nested, index)
                    if index is not None:
                        index += 1
            return
        name = child.get("class")
        if name in seen:
            return
        seen.add(name)
        if index is None:
            parent.append(child)
        else:
            parent.insert(index, child)
        normalize_children(child)

    def normalize_children(parent: ET.Element) -> None:
        for child in list(parent):
            if child.tag != "default":
                continue
            index = list(parent).index(child)
            parent.remove(child)
            add(parent, child, index)

    normalize_children(default)
