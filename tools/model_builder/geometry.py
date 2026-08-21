"""MJCF defaults, assets, and URDF geometry conversion."""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

from convert_visual_meshes import COMPONENT_MAP
from urdf_common import fmt, fmt_vec, origin_attrib

from . import BuildContext, el

REPO_ROOT = Path(__file__).resolve().parents[2]
CONVERSION_MANIFEST = REPO_ROOT / "source" / "generated" / "asset_conversion.json"


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


def load_visual_conversion(path: Path = CONVERSION_MANIFEST) -> dict[str, list[str]]:
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

    converted: dict[str, list[str]] = {}
    for uri, record in data.items():
        if not isinstance(uri, str) or not uri.endswith(".dae"):
            continue
        outputs = record.get("outputs") if isinstance(record, dict) else None
        if not isinstance(outputs, list) or not outputs:
            raise RuntimeError(
                f"{path}: {uri} has no outputs[]; rerun tools/convert_visual_meshes.py --force"
            )
        paths: list[str] = []
        for output in outputs:
            output_path = output.get("path") if isinstance(output, dict) else None
            if not isinstance(output_path, str) or not output_path.startswith("models/"):
                raise RuntimeError(f"{path}: {uri} has an invalid output path: {output_path!r}")
            rel = output_path.removeprefix("models/")
            if not rel.endswith(".obj") or Path(rel).is_absolute() or ".." in Path(rel).parts:
                raise RuntimeError(f"{path}: {uri} has an unsafe OBJ output path: {output_path!r}")
            if not (REPO_ROOT / output_path).is_file():
                raise RuntimeError(f"{path}: converted output is missing: {output_path}")
            paths.append(rel)
        converted[uri] = paths
    return converted


def mesh_assets(
    mesh_file: str, visual_conversion: dict[str, list[str]] | None = None
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
    paths = visual_conversion.get(mesh_file)
    if not paths:
        raise RuntimeError(
            f"visual mesh {mesh_file} has no converted OBJ outputs in {CONVERSION_MANIFEST}"
        )
    return [(f"{stem}_visual_{index}", path) for index, path in enumerate(paths)]


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
        ("sensor_collision", dict(group="4", contype="1", conaffinity="1", density="0")),
    ):
        class_default = el("default", **{"class": class_name})
        class_default.append(el("geom", **attrs))
        default.append(class_default)
    return default


def assets(ctx: BuildContext) -> ET.Element:
    asset = el("asset")
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
    if ctx.opts.sensors:
        for name, path in (
            ("d455", "d455/assets/visual/d455.stl"),
            ("nanoscan3_visual", "nanoscan3/assets/visual/NANS3.obj"),
            ("nanoscan3_collision", "nanoscan3/assets/visual/NANS3_collision.stl"),
            ("zed_mini", "zed_mini/assets/visual/zedm.stl"),
        ):
            asset.append(el("mesh", name=name, file=path))
    return asset


def visual(
    geom: ET.Element, visual_conversion: dict[str, list[str]] | None = None
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
        mapped = mesh_assets(child.get("filename"), visual_conversion)
        return [] if mapped is None else [el("geom", **attrs, type="mesh", mesh=name) for name, _ in mapped]
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
