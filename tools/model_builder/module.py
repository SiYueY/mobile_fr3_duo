"""Emit independently loadable, single-component MJCF modules.

Modules are distribution units, not a nested runtime composition mechanism.
Each XML contains only its own component subtree and a local copy of every
mesh it needs.  The complete robot is emitted separately by ``build_models``.
"""

from __future__ import annotations

import copy
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml
from utils.xml import format_element

from .canonical import CanonicalModel


@dataclass(frozen=True)
class ModuleSpec:
    key: str
    path: Path
    root_body: str
    module_type: str
    attachments: tuple[dict[str, str], ...]


def _find_body(root: ET.Element, name: str) -> ET.Element:
    body = next((item for item in root.iter("body") if item.get("name") == name), None)
    if body is None:
        raise ValueError(f"body not found while creating module: {name}")
    return body


def _rename_prefix(root: ET.Element, source_prefix: str) -> None:
    for element in root.iter():
        for key, value in tuple(element.attrib.items()):
            if value.startswith(source_prefix):
                element.set(key, value.removeprefix(source_prefix))


def _drop_component_children(root: ET.Element, names: set[str]) -> None:
    """Remove component roots below ``root`` without touching its own body."""
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "body" and child.get("name") in names:
                parent.remove(child)


def _referenced_meshes(root_body: ET.Element) -> set[str]:
    return {geom.get("mesh") for geom in root_body.iter("geom") if geom.get("mesh")}


def _referenced_materials(root_body: ET.Element) -> set[str]:
    return {geom.get("material") for geom in root_body.iter("geom") if geom.get("material")}


def _module_xml(repo_root: Path, output_dir: Path, source: ET.Element, body: ET.Element) -> ET.Element:
    """Create one local, standalone module and copy its mesh closure.

    Source packages are already partitioned by component, so the source prefix
    through ``/assets/`` can be removed.  Each module therefore has the simple
    portable layout ``assets/{visual,collision}/...``.
    """
    root = ET.Element("mujoco", model=body.get("name"))
    compiler = copy.deepcopy(source.find("compiler"))
    if compiler is None:
        raise ValueError("source model is missing compiler")
    compiler.set("meshdir", "assets")
    root.append(compiler)
    for tag in ("option", "default"):
        item = source.find(tag)
        if item is not None:
            root.append(copy.deepcopy(item))

    asset = ET.Element("asset")
    wanted = _referenced_meshes(body)
    wanted_materials = _referenced_materials(body)
    source_assets = source.find("asset")
    if source_assets is None:
        raise ValueError("source model is missing assets")
    for material in source_assets.findall("material"):
        if material.get("name") in wanted_materials:
            asset.append(copy.deepcopy(material))
    for mesh in source_assets.findall("mesh"):
        if mesh.get("name") not in wanted:
            continue
        local_mesh = copy.deepcopy(mesh)
        file_name = local_mesh.get("file")
        if not file_name or Path(file_name).is_absolute() or file_name.startswith("../"):
            raise ValueError(f"module mesh has unsafe source path: {file_name}")
        # ``repo_root/models`` is the source compiler's mesh directory.
        source_file = repo_root / "models" / file_name
        if not source_file.is_file():
            raise FileNotFoundError(f"module mesh source is missing: {source_file}")
        marker = "/assets/"
        if marker not in file_name:
            raise ValueError(f"module mesh is outside a component asset package: {file_name}")
        local_name = file_name.split(marker, maxsplit=1)[1]
        local_mesh.set("file", local_name)
        target = output_dir / "assets" / local_name
        target.parent.mkdir(parents=True, exist_ok=True)
        if source_file.resolve() != target.resolve():
            shutil.copy2(source_file, target)
        asset.append(local_mesh)
    root.append(asset)
    worldbody = ET.Element("worldbody")
    worldbody.append(body)
    root.append(worldbody)
    return root


def _write_module(repo_root: Path, spec: ModuleSpec, source: ET.Element, body: ET.Element) -> None:
    spec.path.parent.mkdir(parents=True, exist_ok=True)
    assets = spec.path.parent / "assets"
    # Remove only the namespace directories created by the superseded module
    # packager.  Do not remove visual/collision: for a module's own component
    # those files are also the immutable builder input.
    for name in ("franka_tmr", "franka_spine", "franka_head", "franka_fr3", "franka_hand", "realsense_d455", "imu", "nanoscan3", "zed_mini"):
        stale = assets / name
        if stale.is_dir():
            shutil.rmtree(stale)
    root = _module_xml(repo_root, spec.path.parent, source, body)
    spec.path.write_text(format_element(root), encoding="utf-8")
    metadata = {
        "name": spec.key,
        "type": spec.module_type,
        "source": {"type": "official-derived", "version": "franka_description@2.8.1"},
        "root_body": spec.root_body,
        "attachments": list(spec.attachments),
    }
    spec.path.with_suffix(".metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )


def _sensor_specs() -> tuple[tuple[str, str, str, str], ...]:
    return (
        ("realsense_d455", "camera_front_link", "camera_front_", "link"),
        ("nanoscan3", "lidar_front", "lidar_front_", "link"),
        ("zed_mini", "head_zed", "", "head_zed"),
        ("imu", "imu_link", "", "imu_link"),
    )


def build_modules(repo_root: Path, source_models: dict[str, ET.Element], canonical: CanonicalModel) -> dict[str, ModuleSpec]:
    """Build flat component modules with no child model references."""
    base = source_models["base"]
    sensors = source_models["sensors"]
    models_dir = repo_root / "models"
    for stale in models_dir.glob("*/dependencies"):
        shutil.rmtree(stale)
    for stale in models_dir.glob("sensors"):
        shutil.rmtree(stale)
    for stale in models_dir.glob("**/metadata.yaml"):
        stale.unlink()
    for stale in (
        models_dir / "franka_tmr/assets/visual/d455.stl",
        models_dir / "franka_tmr/assets/visual/NANS3.obj",
        models_dir / "franka_tmr/assets/visual/NANS3_collision.stl",
        models_dir / "franka_head/assets/visual/zedm.stl",
    ):
        stale.unlink(missing_ok=True)
    for stale in (models_dir / "franka_head/franka_head_body.xml",):
        stale.unlink(missing_ok=True)

    specs = {
        "franka_hand": ModuleSpec("franka_hand", models_dir / "franka_hand/franka_hand.xml", "fr3v2_1_hand", "end_effector", ({"name": "parent", "body": "fr3v2_1_hand"},)),
        "franka_fr3": ModuleSpec("franka_fr3", models_dir / "franka_fr3/franka_fr3.xml", "base", "arm", ({"name": "base_mount", "body": "base"}, {"name": "flange", "body": "fr3v2_1_link8"})),
        "franka_head": ModuleSpec("franka_head", models_dir / "franka_head/franka_head.xml", "fr3_duo_mount_mounting_point", "accessory", ({"name": "spine_mount", "body": "fr3_duo_mount_mounting_point"},)),
        "franka_spine": ModuleSpec("franka_spine", models_dir / "franka_spine/franka_spine.xml", "franka_spine", "accessory", ({"name": "base_mount", "body": "franka_spine"},)),
        "franka_tmr": ModuleSpec("franka_tmr", models_dir / "franka_tmr/franka_tmr.xml", "base_link", "mobile_base", ({"name": "world", "body": "base_link"},)),
    }
    for body_name in ("base_link", "franka_spine", "fr3_duo_mount_mounting_point", "left_base"):
        if not canonical.contains(body_name):
            raise ValueError(f"canonical URDF is missing module cut body: {body_name}")

    hand = copy.deepcopy(_find_body(base, "left_fr3v2_1_hand"))
    _rename_prefix(hand, "left_")
    _write_module(repo_root, specs["franka_hand"], base, hand)

    arm = copy.deepcopy(_find_body(base, "left_base"))
    _drop_component_children(arm, {"left_fr3v2_1_hand"})
    _rename_prefix(arm, "left_")
    _write_module(repo_root, specs["franka_fr3"], base, arm)

    head = copy.deepcopy(_find_body(base, "fr3_duo_mount_mounting_point"))
    _drop_component_children(head, {"left_base", "right_base", "head_zed"})
    _write_module(repo_root, specs["franka_head"], base, head)

    spine = copy.deepcopy(_find_body(base, "franka_spine"))
    _drop_component_children(spine, {"fr3_duo_mount_mounting_point"})
    _write_module(repo_root, specs["franka_spine"], base, spine)

    tmr = copy.deepcopy(_find_body(base, "base_link"))
    _drop_component_children(tmr, {
        "franka_spine", "imu_link", "camera_front_link", "camera_rear_link",
        "camera_left_link", "camera_right_link", "lidar_front", "lidar_rear",
    })
    _write_module(repo_root, specs["franka_tmr"], base, tmr)

    for key, source_body, prefix, root_name in _sensor_specs():
        body = copy.deepcopy(_find_body(sensors, source_body))
        _rename_prefix(body, prefix)
        body.set("name", root_name)
        spec = ModuleSpec(key, models_dir / key / f"{key}.xml", root_name, "sensor", ({"name": "mount", "body": root_name},))
        _write_module(repo_root, spec, sensors, body)
        specs[key] = spec
    return specs
