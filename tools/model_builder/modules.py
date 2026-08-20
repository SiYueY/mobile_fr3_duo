"""Build standalone runtime MJCF modules from the validated whole-robot IR."""

from __future__ import annotations

import copy
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml
from format_xml import format_element

from .canonical import CanonicalModel


@dataclass(frozen=True)
class ModuleSpec:
    key: str
    path: Path
    root_body: str
    meshdir: str
    module_type: str
    attachments: tuple[dict[str, str], ...]


def _find_body(root: ET.Element, name: str) -> ET.Element:
    found = next((body for body in root.iter("body") if body.get("name") == name), None)
    if found is None:
        raise ValueError(f"body not found while creating module: {name}")
    return found


def _find_parent(root: ET.Element, child: ET.Element) -> ET.Element:
    for parent in root.iter():
        if child in list(parent):
            return parent
    raise ValueError(f"body has no parent: {child.get('name')}")


def _replace_body_with_attach(
    root: ET.Element, body_name: str, model: str, module_body: str, prefix: str
) -> None:
    body = _find_body(root, body_name)
    parent = _find_parent(root, body)
    index = list(parent).index(body)
    parent.remove(body)
    parent.insert(index, ET.Element("attach", model=model, body=module_body, prefix=prefix))


def _rename_prefix(root: ET.Element, source_prefix: str) -> None:
    """Make one side of a mirrored subtree reusable through attach prefixing."""
    for element in root.iter():
        for key, value in tuple(element.attrib.items()):
            if value.startswith(source_prefix):
                element.set(key, value.removeprefix(source_prefix))


def _referenced_meshes(root_body: ET.Element) -> set[str]:
    return {geom.get("mesh") for geom in root_body.iter("geom") if geom.get("mesh")}


def _module_xml(
    source: ET.Element,
    body: ET.Element,
    meshdir: str,
    child_models: tuple[tuple[str, str], ...] = (),
) -> ET.Element:
    """Create a self-contained module root with just its referenced assets."""
    root = ET.Element("mujoco", model=body.get("name"))
    compiler = copy.deepcopy(source.find("compiler"))
    compiler.set("meshdir", meshdir)
    root.append(compiler)
    root.append(copy.deepcopy(source.find("option")))
    root.append(copy.deepcopy(source.find("default")))

    asset = ET.Element("asset")
    wanted = _referenced_meshes(body)
    source_assets = source.find("asset")
    for mesh in source_assets.findall("mesh"):
        if mesh.get("name") in wanted:
            local_mesh = copy.deepcopy(mesh)
            path = local_mesh.get("file", "")
            marker = "assets/"
            if marker not in path:
                raise ValueError(f"module mesh is not owned by an asset package: {path}")
            local_mesh.set("file", path[path.index(marker) + len(marker):])
            asset.append(local_mesh)
    for name, file in child_models:
        asset.append(ET.Element("model", name=name, file=file))
    root.append(asset)
    worldbody = ET.Element("worldbody")
    worldbody.append(body)
    root.append(worldbody)
    return root


def _metadata(spec: ModuleSpec) -> dict:
    return {
        "name": spec.key,
        "type": spec.module_type,
        "source": {"type": "official-derived", "version": "franka_description@2.8.1"},
        "root_body": spec.root_body,
        "attachments": list(spec.attachments),
    }


def _write_module(spec: ModuleSpec, root: ET.Element) -> None:
    spec.path.parent.mkdir(parents=True, exist_ok=True)
    spec.path.write_text(format_element(root), encoding="utf-8")
    metadata = spec.path.with_suffix(".metadata.yaml")
    metadata.write_text(yaml.safe_dump(_metadata(spec), sort_keys=False), encoding="utf-8")


def _bundle_dependency(owner: ModuleSpec, dependency: ModuleSpec) -> None:
    """Copy one already-built child module into its parent's distribution."""
    destination = owner.path.parent / "dependencies" / dependency.path.parent.name
    if destination.exists():
        shutil.rmtree(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(dependency.path.parent, destination, ignore=shutil.ignore_patterns("dependencies"))
    for nested in dependency.path.parent.glob("dependencies/*"):
        nested_target = destination / "dependencies" / nested.name
        nested_target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(nested, nested_target)


def build_modules(repo_root: Path, source_models: dict[str, ET.Element], canonical: CanonicalModel) -> dict[str, ModuleSpec]:
    """Emit runtime modules from base and sensor baseline MJCF trees.

    The body cuts follow frozen URDF links.  Each replacement happens at the
    original parent location, preserving the exact original transform and name
    after the compiler expands the attach meta-elements.
    """
    base = source_models["base"]
    sensors = source_models["sensors"]
    modules_dir = repo_root / "models"
    # Earlier development snapshots used a directory-level ``metadata.yaml``.
    # The published schema keeps one metadata file next to each module XML.
    for stale in modules_dir.glob("**/metadata.yaml"):
        stale.unlink()
    specs = {
        "hand": ModuleSpec("franka_hand", modules_dir / "franka_hand/franka_hand.xml", "fr3v2_1_hand", "assets", "end_effector", ({"name": "parent", "body": "fr3v2_1_hand"},)),
        "arm": ModuleSpec("franka_fr3", modules_dir / "franka_fr3/franka_fr3.xml", "base", "assets", "arm", ({"name": "base_mount", "body": "base"}, {"name": "flange", "body": "fr3v2_1_link8"})),
        "mount": ModuleSpec("franka_head", modules_dir / "franka_head/franka_head.xml", "fr3_duo_mount_mounting_point", "assets", "accessory", ({"name": "spine_mount", "body": "fr3_duo_mount_mounting_point"},)),
        "spine": ModuleSpec("franka_spine", modules_dir / "franka_spine/franka_spine.xml", "franka_spine", "assets", "accessory", ({"name": "base_mount", "body": "franka_spine"},)),
        "base": ModuleSpec("franka_tmr", modules_dir / "franka_tmr/franka_tmr.xml", "base_link", "assets", "mobile_base", ({"name": "world", "body": "base_link"},)),
        "head": ModuleSpec("franka_head_body", modules_dir / "franka_head/franka_head_body.xml", "head_link", "assets", "accessory", ({"name": "world", "body": "head_link"},)),
    }
    for name in ("base_link", "franka_spine", "fr3_duo_mount_mounting_point", "left_base"):
        if not canonical.contains(name):
            raise ValueError(f"canonical URDF is missing module cut body: {name}")

    hand = copy.deepcopy(_find_body(base, "left_fr3v2_1_hand"))
    _rename_prefix(hand, "left_")
    _write_module(specs["hand"], _module_xml(base, hand, specs["hand"].meshdir))
    sensor_specs = _write_sensor_modules(repo_root, sensors)

    arm = copy.deepcopy(_find_body(base, "left_base"))
    _rename_prefix(arm, "left_")
    _replace_body_with_attach(arm, "fr3v2_1_hand", "hand", "fr3v2_1_hand", "")
    _write_module(
        specs["arm"],
        _module_xml(base, arm, specs["arm"].meshdir, (("hand", "dependencies/franka_hand/franka_hand.xml"),)),
    )
    _bundle_dependency(specs["arm"], specs["hand"])

    mount_source = copy.deepcopy(_find_body(base, "fr3_duo_mount_mounting_point"))
    mount = copy.deepcopy(mount_source)
    _replace_body_with_attach(mount, "left_base", "fr3", "base", "left_")
    _replace_body_with_attach(mount, "right_base", "fr3", "base", "right_")
    _replace_body_with_attach(mount, "head_link", "head", "head_link", "")
    _write_module(
        specs["mount"],
        _module_xml(
            base,
            mount,
            specs["mount"].meshdir,
            (("fr3", "dependencies/franka_fr3/franka_fr3.xml"), ("head", "franka_head_body.xml")),
        ),
    )
    _bundle_dependency(specs["mount"], specs["arm"])

    sensor_mount = copy.deepcopy(mount_source)
    _replace_body_with_attach(sensor_mount, "left_base", "fr3", "base", "left_")
    _replace_body_with_attach(sensor_mount, "right_base", "fr3", "base", "right_")
    _replace_body_with_attach(sensor_mount, "head_link", "head", "head_link", "")
    mount_sensors = ModuleSpec("franka_head_sensors", modules_dir / "franka_head/franka_head_sensors.xml", "fr3_duo_mount_mounting_point", "assets", "accessory", ({"name": "spine_mount", "body": "fr3_duo_mount_mounting_point"},))
    _write_module(
        mount_sensors,
        _module_xml(
            base,
            sensor_mount,
            mount_sensors.meshdir,
            (("fr3", "dependencies/franka_fr3/franka_fr3.xml"), ("head", "franka_head_body_sensors.xml")),
        ),
    )
    _bundle_dependency(mount_sensors, specs["arm"])

    spine_source = copy.deepcopy(_find_body(base, "franka_spine"))
    spine = copy.deepcopy(spine_source)
    _replace_body_with_attach(spine, "fr3_duo_mount_mounting_point", "mount", "fr3_duo_mount_mounting_point", "")
    _write_module(
        specs["spine"],
        _module_xml(base, spine, specs["spine"].meshdir, (("mount", "dependencies/franka_head/franka_head.xml"),)),
    )
    _bundle_dependency(specs["spine"], specs["mount"])

    sensor_spine = copy.deepcopy(spine_source)
    _replace_body_with_attach(sensor_spine, "fr3_duo_mount_mounting_point", "mount", "fr3_duo_mount_mounting_point", "")
    spine_sensors = ModuleSpec("franka_spine_sensors", modules_dir / "franka_spine/franka_spine_sensors.xml", "franka_spine", "assets", "accessory", ({"name": "base_mount", "body": "franka_spine"},))
    _write_module(
        spine_sensors,
        _module_xml(base, sensor_spine, spine_sensors.meshdir, (("mount", "dependencies/franka_head/franka_head_sensors.xml"),)),
    )
    _bundle_dependency(spine_sensors, mount_sensors)

    base_body = copy.deepcopy(_find_body(base, "base_link"))
    _replace_body_with_attach(base_body, "franka_spine", "spine", "franka_spine", "")
    _write_module(
        specs["base"],
        _module_xml(base, base_body, specs["base"].meshdir, (("spine", "dependencies/franka_spine/franka_spine.xml"),)),
    )
    _bundle_dependency(specs["base"], specs["spine"])

    for variant in ("reduced", "planar"):
        variant_source = source_models[variant]
        variant_body = copy.deepcopy(_find_body(variant_source, "base_link"))
        _replace_body_with_attach(variant_body, "franka_spine", "spine", "franka_spine", "")
        variant_spec = ModuleSpec(
            f"tmrv0_2_{variant}",
            modules_dir / f"franka_tmr/franka_tmr_{variant}.xml",
            "base_link",
            "assets",
            "mobile_base",
            ({"name": "world", "body": "base_link"},),
        )
        _write_module(
            variant_spec,
            _module_xml(
                variant_source,
                variant_body,
                variant_spec.meshdir,
                (("spine", "dependencies/franka_spine/franka_spine.xml"),),
            ),
        )
        _bundle_dependency(variant_spec, specs["spine"])

    sensor_base = copy.deepcopy(base_body)
    _attach(sensor_base, "imu_mounting_point", "imu", "imu_link", "")
    for position in ("front", "rear", "left", "right"):
        _attach(sensor_base, f"{position}_mounting_point", "d455", "link", f"camera_{position}_")
    for position in ("front", "rear"):
        _attach(sensor_base, f"lidar_{position}_mounting_point", "nanoscan3", "link", f"lidar_{position}_")
    base_sensors = ModuleSpec("franka_tmr_sensors", modules_dir / "franka_tmr/franka_tmr_sensors.xml", "base_link", "assets", "mobile_base", ({"name": "world", "body": "base_link"},))
    _write_module(
        base_sensors,
        _module_xml(
            base,
            sensor_base,
            base_sensors.meshdir,
            (
                ("spine", "dependencies/franka_spine/franka_spine_sensors.xml"),
                ("imu", "dependencies/imu/imu.xml"),
                ("d455", "dependencies/d455/d455.xml"),
                ("nanoscan3", "dependencies/nanoscan3/nanoscan3.xml"),
            ),
        ),
    )
    _bundle_dependency(base_sensors, spine_sensors)
    for key in ("imu", "d455", "nanoscan3"):
        _bundle_dependency(base_sensors, sensor_specs[key])

    head = copy.deepcopy(_find_body(base, "head_link"))
    _write_module(specs["head"], _module_xml(base, head, specs["head"].meshdir))

    sensor_head = copy.deepcopy(head)
    _attach(sensor_head, "head_camera_mounting_point", "zed_mini", "head_zed", "")
    head_sensors = ModuleSpec("franka_head_body_sensors", modules_dir / "franka_head/franka_head_body_sensors.xml", "head_link", "assets", "accessory", ({"name": "world", "body": "head_link"},))
    _write_module(
        head_sensors,
        _module_xml(base, sensor_head, head_sensors.meshdir, (("zed_mini", "dependencies/zed_mini/zed_mini.xml"),)),
    )
    _bundle_dependency(head_sensors, sensor_specs["zed_mini"])
    # Refresh the sensor dependency closure now that the head sensor variant
    # exists in the head package.
    _bundle_dependency(spine_sensors, mount_sensors)
    _bundle_dependency(base_sensors, spine_sensors)

    return specs


def _attach(parent_root: ET.Element, parent_name: str, model: str, body: str, prefix: str) -> None:
    _find_body(parent_root, parent_name).append(
        ET.Element("attach", model=model, body=body, prefix=prefix)
    )


def _write_sensor_modules(repo_root: Path, source: ET.Element) -> dict[str, ModuleSpec]:
    specs = (
        ("d455", "camera_front_link", "camera_front_", "link"),
        ("nanoscan3", "lidar_front", "lidar_front_", "link"),
        ("zed_mini", "head_zed", "", "head_zed"),
        ("imu", "imu_link", "", "imu_link"),
    )
    output: dict[str, ModuleSpec] = {}
    for key, source_body, prefix, root_name in specs:
        body = copy.deepcopy(_find_body(source, source_body))
        _rename_prefix(body, prefix)
        body.set("name", root_name)
        spec = ModuleSpec(key, repo_root / f"models/sensors/{key}/{key}.xml", root_name, "assets", "sensor", ({"name": "mount", "body": root_name},))
        _write_module(spec, _module_xml(source, body, spec.meshdir))
        output[key] = spec
    return output
