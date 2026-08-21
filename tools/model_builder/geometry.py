"""MJCF defaults, assets, and URDF geometry conversion."""

from __future__ import annotations

import json
import hashlib
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from _source.convert_visual_meshes import COMPONENT_MAP
from utils.urdf import fmt, fmt_vec, origin_attrib

from . import BuildContext, el

REPO_ROOT = Path(__file__).resolve().parents[2]
CONVERSION_MANIFEST = REPO_ROOT / "source" / "generated" / "asset_conversion.json"
SENSOR_CONVERSION_MANIFEST = REPO_ROOT / "source" / "generated" / "sensor_asset_conversion.json"


def _mesh_basename(mesh_file: str) -> tuple[str, str, str] | None:
    """Return the component, kind, and source stem for a Franka mesh URI."""
    if not mesh_file.startswith("package://franka_description/meshes/"):
        return None
    rel = mesh_file.removeprefix("package://franka_description/meshes/")
    parts = rel.split("/")
    if len(parts) == 3:
        dirname, kind, filename = parts
    elif len(parts) == 4:
        dirname, kind, filename = f"{parts[0]}/{parts[1]}", parts[2], parts[3]
    else:
        return None
    component = COMPONENT_MAP.get(dirname)
    if component is None:
        return None
    stem = filename.rsplit(".", 1)[0]
    return component, kind, stem


def load_visual_conversion(path: Path = CONVERSION_MANIFEST) -> dict[str, list[dict[str, object]]]:
    """Load visual source URI -> committed OBJ paths from the converter manifest.

    The production builder deliberately consumes only this committed manifest,
    never a developer's external ``franka_description`` checkout.  A malformed
    or old single-output manifest is an actionable source-preparation error.
    """
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"missing visual conversion manifest: {path}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"invalid visual conversion manifest: {path}: {exc}") from exc

    if not isinstance(data, dict):
        raise RuntimeError(f"invalid visual conversion manifest root: {path}")

    converted: dict[str, list[dict[str, object]]] = {}
    for uri, record in data.items():
        if not isinstance(uri, str) or not uri.endswith(".dae"):
            continue
        outputs = record.get("outputs") if isinstance(record, dict) else None
        if not isinstance(outputs, list) or not outputs:
            raise RuntimeError(
                f"{path}: {uri} has no outputs[]; rerun tools/prepare_source.py"
            )
        entries: list[dict[str, object]] = []
        for output in outputs:
            output_path = output.get("path") if isinstance(output, dict) else None
            if not isinstance(output_path, str) or not output_path.startswith("models/"):
                raise RuntimeError(f"{path}: {uri} has an invalid output path: {output_path!r}")
            rel = output_path.removeprefix("models/")
            if not rel.endswith(".obj") or Path(rel).is_absolute() or ".." in Path(rel).parts:
                raise RuntimeError(f"{path}: {uri} has an unsafe OBJ output path: {output_path!r}")
            if not (REPO_ROOT / output_path).is_file():
                raise RuntimeError(f"{path}: converted output is missing: {output_path}")
            entry = dict(output)
            entry["path"] = rel
            entries.append(entry)
        converted[uri] = entries
    return converted


def load_sensor_appearances(
    path: Path = SENSOR_CONVERSION_MANIFEST,
) -> dict[str, dict[str, object]]:
    """Load explicit, official sensor appearance records from source prep."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"missing sensor conversion manifest: {path}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"invalid sensor conversion manifest root: {path}")
    appearances: dict[str, dict[str, object]] = {}
    for key in ("realsense_d455", "sick_nanoscan3_visual", "zed_mini"):
        record = data.get(key)
        appearance = record.get("appearance") if isinstance(record, dict) else None
        if not isinstance(appearance, dict) or _material_rgba({"material": appearance}) is None:
            raise RuntimeError(f"{path}: {key} has no valid official appearance")
        appearances[key] = appearance
    return appearances


def mesh_assets(
    mesh_file: str, visual_conversion: dict[str, list[dict[str, object]]] | None = None
) -> list[tuple[str, str]] | None:
    """Map a Franka mesh URI to one or more committed MJCF assets."""
    source = _mesh_basename(mesh_file)
    if source is None:
        return None
    component, kind, stem = source
    if kind != "visual":
        return [(f"{stem}_{kind}", f"{component}/{kind}/{stem}.stl")]

    if visual_conversion is None:
        visual_conversion = load_visual_conversion()
    outputs = visual_conversion.get(mesh_file)
    if not outputs:
        raise RuntimeError(
            f"visual mesh {mesh_file} has no converted OBJ outputs in {CONVERSION_MANIFEST}"
        )
    return [
        (f"{stem}_visual_{index}", str(output["path"]))
        for index, output in enumerate(outputs)
    ]


def _material_rgba(output: dict[str, object]) -> tuple[float, float, float, float] | None:
    """Read a normalised RGBA record from a converted DAE output."""
    material = output.get("material")
    if not isinstance(material, dict):
        return None
    rgba = material.get("rgba")
    if not isinstance(rgba, list) or len(rgba) != 4:
        return None
    try:
        values = tuple(float(value) for value in rgba)
    except (TypeError, ValueError):
        return None
    if any(value < 0 or value > 1 for value in values):
        return None
    return values  # type: ignore[return-value]


def _material_asset_name(output: dict[str, object]) -> str | None:
    rgba = _material_rgba(output)
    if rgba is None:
        return None
    source = output.get("material")
    assert isinstance(source, dict)
    source_name = source.get("name")
    stem = re.sub(r"[^A-Za-z0-9_]+", "_", str(source_name or "dae")).strip("_")
    digest = hashlib.sha256(" ".join(f"{value:.8g}" for value in rgba).encode()).hexdigest()[:10]
    return f"dae_{stem or 'material'}_{digest}"


def sensor_material_name(key: str, appearances: dict[str, dict[str, object]]) -> str:
    appearance = appearances[key]
    rgba = _material_rgba({"material": appearance})
    assert rgba is not None
    source_name = appearance.get("name")
    stem = re.sub(r"[^A-Za-z0-9_]+", "_", str(source_name or key)).strip("_")
    digest = hashlib.sha256(" ".join(f"{value:.8g}" for value in rgba).encode()).hexdigest()[:10]
    return f"sensor_{stem or key}_{digest}"


def visual_mesh_assets(
    mesh_file: str, visual_conversion: dict[str, list[dict[str, object]]] | None = None
) -> list[tuple[str, str, str | None]] | None:
    """Return mesh assets plus their optional DAE-derived MJCF material."""
    mapped = mesh_assets(mesh_file, visual_conversion)
    if mapped is None:
        return None
    if visual_conversion is None:
        visual_conversion = load_visual_conversion()
    outputs = visual_conversion.get(mesh_file, [])
    return [
        (name, path, _material_asset_name(output))
        for (name, path), output in zip(mapped, outputs, strict=True)
    ]


def _origin(elem: ET.Element) -> tuple[str, str]:
    xyz, quat = origin_attrib(elem)
    return fmt_vec(xyz), fmt_vec(quat)


def defaults() -> ET.Element:
    default = el("default")
    default.append(el("geom", density="0"))
    for class_name, attrs in (
        ("visual", dict(contype="0", conaffinity="0", group="2", density="0")),
        ("collision", dict(group="3", contype="1", conaffinity="1", condim="3", friction="1.0 0.005 0.0001", density="0")),
        ("wheel", dict(group="3", contype="1", conaffinity="1", condim="3", friction="1.2 0.005 0.0001", density="0")),
        ("finger_pad", dict(group="3", contype="1", conaffinity="1", condim="4", friction="1.0 0.005 0.0001", solref="0.02 1", solimp="0.9 0.95 0.001", density="0")),
    ):
        class_default = el("default", **{"class": class_name})
        class_default.append(el("geom", **attrs))
        default.append(class_default)
    return default


def assets(ctx: BuildContext) -> ET.Element:
    asset = el("asset")
    material_seen: set[str] = set()
    for outputs in ctx.visual_conversion.values():
        for output in outputs:
            name = _material_asset_name(output)
            rgba = _material_rgba(output)
            if name is None or rgba is None or name in material_seen:
                continue
            material_seen.add(name)
            asset.append(el("material", name=name, rgba=fmt_vec(rgba)))
    for key, appearance in ctx.sensor_appearances.items():
        rgba = _material_rgba({"material": appearance})
        assert rgba is not None
        asset.append(el("material", name=sensor_material_name(key, ctx.sensor_appearances), rgba=fmt_vec(rgba)))
    seen: set[str] = set()
    for link in ctx.urdf.links.values():
        for geom_kind in ("visual", "collision"):
            for geom in link.findall(geom_kind):
                mesh = geom.find("./geometry/mesh")
                if mesh is None:
                    continue
                mapped = mesh_assets(mesh.get("filename"), ctx.visual_conversion)
                if mapped is None:
                    continue
                for name, path in mapped:
                    if name in seen:
                        continue
                    seen.add(name)
                    # Split visual parts can be open surfaces.  They have no
                    # physical contribution (the visual geom class is
                    # density-free), but MuJoCo still validates mesh inertia
                    # while compiling the asset.
                    attrs = {"name": name, "file": path}
                    if path.endswith(".obj"):
                        attrs["inertia"] = "shell"
                    asset.append(el("mesh", **attrs))
    for name, path in (
            ("realsense_d455", "realsense_d455/assets/visual/d455.obj"),
            ("nanoscan3_visual", "nanoscan3/assets/visual/NANS3.obj"),
            ("zed_mini", "zed_mini/assets/visual/zedm.obj"),
    ):
        asset.append(el("mesh", name=name, file=path))
    return asset


def visual(
    geom: ET.Element, visual_conversion: dict[str, list[dict[str, object]]] | None = None
) -> list[ET.Element]:
    pos, quat = _origin(geom)
    child = list(geom.find("geometry"))[0]
    rgba = None
    material = geom.find("material")
    if material is not None:
        color = material.find("color")
        if color is not None and color.get("rgba"):
            rgba = color.get("rgba")
    attrs = {"class": "visual", "pos": pos, "quat": quat}
    if rgba is not None:
        attrs["rgba"] = rgba
    if child.tag == "mesh":
        mapped = visual_mesh_assets(child.get("filename"), visual_conversion)
        if mapped is None:
            return []
        return [
            el("geom", **attrs, type="mesh", mesh=name, material=material_name)
            for name, _, material_name in mapped
        ]
    if child.tag == "box":
        half = " ".join(fmt(float(value) / 2.0) for value in child.get("size").split())
        return [el("geom", **attrs, type="box", size=half)]
    if child.tag == "cylinder":
        return [el("geom", **attrs, type="cylinder", size=f"{fmt(float(child.get('radius')))} {fmt(float(child.get('length')) / 2.0)}")]
    if child.tag == "sphere":
        return [el("geom", **attrs, type="sphere", size=child.get("radius"))]
    return []


def collision(geom: ET.Element, link_name: str) -> ET.Element | None:
    pos, quat = _origin(geom)
    child = list(geom.find("geometry"))[0]
    wheels = {"argo_drive_front_link", "argo_drive_rear_link", "caster_front_left_link", "caster_rear_right_link"}
    cls = "wheel" if link_name in wheels else ("finger_pad" if link_name.endswith("finger") else "collision")
    if child.tag == "mesh":
        mapped = mesh_assets(child.get("filename"))
        return None if mapped is None else el("geom", **{"class": cls}, type="mesh", mesh=mapped[0][0], pos=pos, quat=quat)
    if child.tag == "box":
        half = " ".join(fmt(float(value) / 2.0) for value in child.get("size").split())
        return el("geom", **{"class": cls}, type="box", size=half, pos=pos, quat=quat)
    if child.tag == "cylinder":
        return el("geom", **{"class": cls}, type="cylinder", size=f"{fmt(float(child.get('radius')))} {fmt(float(child.get('length')) / 2.0)}", pos=pos, quat=quat)
    if child.tag == "sphere":
        return el("geom", **{"class": cls}, type="sphere", size=child.get("radius"), pos=pos, quat=quat)
    return None
