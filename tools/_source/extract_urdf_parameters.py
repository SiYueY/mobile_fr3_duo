"""Extract normalized model parameters from the generated official URDFs.

Writes the source manifests:
inertial_manifest, frame_manifest, asset_manifest, name_mapping.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

TOOLS_ROOT = Path(__file__).resolve().parents[1]
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from utils.urdf import load_links_and_joints, origin_attrib, quat_multiply

REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATED = REPO_ROOT / "source" / "generated"
SOURCE = REPO_ROOT / "source"

MOUNTING_POINTS = [
    "imu_mounting_point",
    "front_mounting_point",
    "rear_mounting_point",
    "left_mounting_point",
    "right_mounting_point",
    "lidar_front_mounting_point",
    "lidar_rear_mounting_point",
    "head_camera_mounting_point",
    "franka_spine_mounting_point",
    "fr3_duo_mount_mounting_point",
]

TCP_FRAMES = [
    "left_fr3v2_1_hand_tcp",
    "right_fr3v2_1_hand_tcp",
    "left_fr3v2_1_link8",
    "right_fr3v2_1_link8",
]


def build_chain(urdf: Path) -> dict[str, dict]:
    links, joints, root = load_links_and_joints(urdf)
    child_to_joint = {j.find("child").get("link"): j for j in joints.values()}

    chain: dict[str, dict] = {}
    for name, link in links.items():
        inertial = link.find("inertial")
        entry = {
            "name": name,
            "parent": None,
            "joint": None,
            "origin_xyz": [0.0, 0.0, 0.0],
            "origin_rpy": [0.0, 0.0, 0.0],
            "abs_xyz": [0.0, 0.0, 0.0],
            "abs_quat": [1.0, 0.0, 0.0, 0.0],
            "mass": _inertial_mass(inertial),
            "has_inertial": inertial is not None,
            "n_visual": len(link.findall("visual")),
            "n_collision": len(link.findall("collision")),
        }
        chain[name] = entry

    # Compute absolute transforms via parent chain.
    order = [root]
    visited = {root}
    while order:
        name = order.pop(0)
        entry = chain[name]
        parent = entry["parent"]
        if parent is not None:
            p = chain[parent]
            xyz, quat = origin_attrib(child_to_joint[name])
            entry["origin_xyz"] = xyz.tolist()
            entry["origin_rpy"] = _quat_to_rpy(quat)
            p_xyz = np.asarray(p["abs_xyz"], dtype=float)
            p_quat = np.asarray(p["abs_quat"], dtype=float)
            entry["abs_xyz"] = (p_xyz + quat_rotate(p_quat, xyz)).tolist()
            entry["abs_quat"] = quat_multiply(p_quat, quat).tolist()
        for j in joints.values():
            if j.find("parent").get("link") == name:
                child = j.find("child").get("link")
                if child not in visited:
                    chain[child]["parent"] = name
                    chain[child]["joint"] = j.get("name")
                    order.append(child)
                    visited.add(child)
    return chain


def _inertial_mass(inertial) -> float | None:
    if inertial is None:
        return None
    mass = inertial.find("mass")
    return float(mass.get("value")) if mass is not None else None


def _quat_to_rpy(q: list[float]) -> list[float]:
    from scipy.spatial.transform import Rotation

    r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    return r.as_euler("xyz", degrees=False).tolist()


def quat_rotate(q: list[float], v: list[float]) -> list[float]:
    from scipy.spatial.transform import Rotation

    return Rotation.from_quat([q[1], q[2], q[3], q[0]]).apply(v).tolist()


def collect_joints(urdf: Path) -> list[dict]:
    links, joints, _ = load_links_and_joints(urdf)
    out = []
    for j in joints.values():
        limit = j.find("limit")
        dyn = j.find("dynamics")
        axis = j.find("axis")
        entry = {
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "origin_xyz": origin_attrib(j)[0].tolist(),
            "origin_rpy": _quat_to_rpy(origin_attrib(j)[1].tolist()),
            "axis": axis.get("xyz").split() if axis is not None else None,
            "lower": float(limit.get("lower")) if limit is not None and limit.get("lower") is not None else None,
            "upper": float(limit.get("upper")) if limit is not None and limit.get("upper") is not None else None,
            "effort": float(limit.get("effort")) if limit is not None and limit.get("effort") is not None else None,
            "velocity": float(limit.get("velocity")) if limit is not None and limit.get("velocity") is not None else None,
            "damping": float(dyn.get("damping")) if dyn is not None and dyn.get("damping") is not None else None,
            "friction": float(dyn.get("friction")) if dyn is not None and dyn.get("friction") is not None else None,
            "motor_inertia": float(dyn.get("motor_inertia")) if dyn is not None and dyn.get("motor_inertia") is not None else None,
            "gear_ratio": float(dyn.get("gear_ratio")) if dyn is not None and dyn.get("gear_ratio") is not None else None,
        }
        out.append(entry)
    return out


def collect_inertials(urdf: Path) -> list[dict]:
    links, _, _ = load_links_and_joints(urdf)
    out = []
    for link in links.values():
        inertial = link.find("inertial")
        if inertial is None:
            continue
        out.append(
            {
                "link": link.get("name"),
                "mass": _inertial_mass(inertial),
                "com_xyz": parse_vec(inertial),
                "quat": origin_attrib(inertial)[1].tolist(),
                **{
                    k: float(v)
                    for k, v in _inertia_attrs(inertial).items()
                },
            }
        )
    return out


def _inertia_attrs(inertial) -> dict[str, float]:
    inertia = inertial.find("inertia")
    if inertia is None:
        return {}
    return {k: v for k, v in inertia.attrib.items() if k in {"ixx", "iyy", "izz", "ixy", "ixz", "iyz"}}


def parse_vec(elem) -> list[float]:
    origin = elem.find("origin")
    if origin is None:
        return [0.0, 0.0, 0.0]
    return [float(x) for x in origin.get("xyz", "0 0 0").split()[:3]]


def collect_assets(*urdf_paths: Path) -> list[dict]:
    seen: dict[str, dict] = {}
    for urdf in urdf_paths:
        root = __import__("xml.etree.ElementTree", fromlist=["ElementTree"]).parse(urdf)
        for geom_kind in ("visual", "collision"):
            for link in root.findall("link"):
                for g in link.findall(geom_kind):
                    mesh = g.find("./geometry/mesh")
                    if mesh is None:
                        continue
                    filename = mesh.get("filename")
                    if filename in seen:
                        continue
                    seen[filename] = {
                        "source": filename,
                        "kind": geom_kind,
                        "urdf": urdf.name,
                    }
    return sorted(seen.values(), key=lambda a: a["source"])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--urdf",
        type=Path,
        default=GENERATED / "mobile_fr3_duo.urdf",
    )
    args = ap.parse_args()

    GENERATED.mkdir(parents=True, exist_ok=True)
    chain = build_chain(args.urdf)
    joints = collect_joints(args.urdf)
    inertials = collect_inertials(args.urdf)
    assets = collect_assets(args.urdf, args.urdf)

    payload = {
        "root": next(iter(chain.values()))["name"] if chain else None,
        "links": list(chain.values()),
        "joints": joints,
        "inertials": inertials,
        "assets": assets,
        "total_mass": sum(i["mass"] for i in inertials),
        "n_links": len(chain),
        "n_joints": len(joints),
        "n_inertials": len(inertials),
    }
    _write_manifests(payload, chain)
    print(f"extracted {payload['n_links']} links / {payload['n_joints']} joints "
          f"/ {payload['n_inertials']} inertials / total mass {payload['total_mass']:.3f} kg")
    return 0


def _write_manifests(payload: dict, chain: dict[str, dict]) -> None:
    _dump_yaml(SOURCE / "inertial_manifest.yaml", {
        "generated_from": "franka_description@2.8.1",
        "inertials": payload["inertials"],
    })
    frames = []
    for name in MOUNTING_POINTS + TCP_FRAMES:
        if name in chain:
            frames.append({
                "name": name,
                "parent": chain[name]["parent"],
                "xyz": [round(x, 9) for x in chain[name]["abs_xyz"]],
                "quat": [round(x, 9) for x in chain[name]["abs_quat"]],
            })
    _dump_yaml(SOURCE / "frame_manifest.yaml", {
        "generated_from": "franka_description@2.8.1",
        "frames": frames,
    })
    _dump_yaml(SOURCE / "asset_manifest.yaml", {
        "generated_from": "franka_description@2.8.1",
        "assets": payload["assets"],
    })
    _dump_yaml(SOURCE / "name_mapping.yaml", {
        "note": "URDF names are preserved 1:1 in the native MJCF; "
                "actuators/sensors add semantic suffixes.",
        "links": {el["name"]: el["name"] for el in payload["links"]},
        "joints": {j["name"]: j["name"] for j in payload["joints"]},
    })


def _dump_yaml(path: Path, data: dict) -> None:
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, sort_keys=False, allow_unicode=True)


if __name__ == "__main__":
    raise SystemExit(main())
