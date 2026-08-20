"""MJCF defaults, assets, and URDF geometry conversion."""

from __future__ import annotations

import xml.etree.ElementTree as ET

from convert_visual_meshes import COMPONENT_MAP
from urdf_common import fmt, fmt_vec, origin_attrib

from . import BuildContext, el


def mesh_asset(mesh_file: str) -> tuple[str, str] | None:
    """Map a Franka package mesh URI to its committed MJCF asset."""
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
    suffix = ".obj" if kind == "visual" else ".stl"
    return f"{stem}_{kind}", f"{component}/{kind}/{stem}{suffix}"


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
                mapped = mesh_asset(mesh.get("filename"))
                if mapped is None or mapped[0] in seen:
                    continue
                seen.add(mapped[0])
                asset.append(el("mesh", name=mapped[0], file=mapped[1]))
    if ctx.opts.sensors:
        for name, path in (
            ("d455", "sensors/realsense_d455/d455.stl"),
            ("nanoscan3_visual", "sensors/sick_nanoscan3/NANS3.obj"),
            ("nanoscan3_collision", "sensors/sick_nanoscan3/NANS3_collision.stl"),
            ("zed_mini", "sensors/zed_mini/zedm.stl"),
        ):
            asset.append(el("mesh", name=name, file=path))
    return asset


def visual(geom: ET.Element) -> ET.Element | None:
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
        mapped = mesh_asset(child.get("filename"))
        return None if mapped is None else el("geom", **attrs, type="mesh", mesh=mapped[0])
    if child.tag == "box":
        half = " ".join(fmt(float(value) / 2.0) for value in child.get("size").split())
        return el("geom", **attrs, type="box", size=half)
    if child.tag == "cylinder":
        return el("geom", **attrs, type="cylinder", size=f"{fmt(float(child.get('radius')))} {fmt(float(child.get('length')) / 2.0)}")
    if child.tag == "sphere":
        return el("geom", **attrs, type="sphere", size=child.get("radius"))
    return None


def collision(geom: ET.Element, link_name: str) -> ET.Element | None:
    pos, quat = _origin(geom)
    child = list(geom.find("geometry"))[0]
    wheels = {"argo_drive_front_link", "argo_drive_rear_link", "caster_front_left_link", "caster_rear_right_link"}
    cls = "wheel" if link_name in wheels else ("finger_pad" if link_name.endswith("finger") else "collision")
    if child.tag == "mesh":
        mapped = mesh_asset(child.get("filename"))
        return None if mapped is None else el("geom", **{"class": cls}, type="mesh", mesh=mapped[0], pos=pos, quat=quat)
    if child.tag == "box":
        half = " ".join(fmt(float(value) / 2.0) for value in child.get("size").split())
        return el("geom", **{"class": cls}, type="box", size=half, pos=pos, quat=quat)
    if child.tag == "cylinder":
        return el("geom", **{"class": cls}, type="cylinder", size=f"{fmt(float(child.get('radius')))} {fmt(float(child.get('length')) / 2.0)}", pos=pos, quat=quat)
    if child.tag == "sphere":
        return el("geom", **{"class": cls}, type="sphere", size=child.get("radius"), pos=pos, quat=quat)
    return None
