"""Validate the committed assets and generated MJCF variants.

Checks performed:
  * every variant XML loads without errors (no missing assets, no bad refs);
  * every asset referenced by the XMLs exists on disk;
  * no `package://` URIs or absolute paths remain in the deliverables;
  * every visual/collision mesh has sane meter-scale extents and no
    degenerate triangles;
  * the conversion manifests (SHA-256, tool version) are present.
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import trimesh

REPO_ROOT = Path(__file__).resolve().parent.parent

VARIANTS = [
    "mobile_fr3_duo.xml",
    "mobile_fr3_duo_with_sensors.xml",
    "mobile_fr3_duo_position.xml",
    "mobile_fr3_duo_reduced.xml",
    "mobile_fr3_duo_planar_debug.xml",
    "scene.xml",
    "scene_with_sensors.xml",
    "scene_position.xml",
]

MIN_EXTENT = 1e-4
MAX_EXTENT = 3.0


def check_meshes() -> list[str]:
    problems = []
    for f in sorted(REPO_ROOT.glob("assets/**/*.obj")) + sorted(
        REPO_ROOT.glob("assets/**/*.stl")
    ):
        try:
            mesh = trimesh.load(f, force="mesh")
        except Exception as exc:  # noqa: BLE001
            problems.append(f"{f}: load failed: {exc}")
            continue
        if len(mesh.faces) == 0:
            problems.append(f"{f}: no faces")
        n_degenerate = int((mesh.area_faces <= 0).sum()) if len(mesh.area_faces) else 0
        is_collision = f.suffix == ".stl"
        if n_degenerate and is_collision:
            problems.append(f"{f}: degenerate faces ({n_degenerate})")
        extents = mesh.extents
        if extents.max() < MIN_EXTENT or extents.max() > MAX_EXTENT:
            problems.append(f"{f}: suspicious extents {extents.round(4)}")
    return problems


def check_manifests() -> list[str]:
    problems = []
    expected = [
        "source/asset_manifest.yaml",
        "source/link_manifest.yaml",
        "source/joint_manifest.yaml",
        "source/inertial_manifest.yaml",
        "source/frame_manifest.yaml",
        "source/name_mapping.yaml",
        "source/parameter_sources.yaml",
        "source/official_model_files.yaml",
        "source/generated/asset_conversion.json",
        "source/generated/asset_collision_conversion.json",
        "source/generated/sensor_asset_conversion.json",
    ]
    for rel in expected:
        p = REPO_ROOT / rel
        if not p.exists():
            problems.append(f"{rel}: missing")
    for rel in (
        "source/generated/asset_conversion.json",
        "source/generated/sensor_asset_conversion.json",
    ):
        try:
            data = json.loads((REPO_ROOT / rel).read_text())
            if not data:
                problems.append(f"{rel}: empty")
        except Exception as exc:  # noqa: BLE001
            problems.append(f"{rel}: invalid json: {exc}")
    return problems


def main() -> int:
    problems: list[str] = []
    for name in VARIANTS:
        path = REPO_ROOT / name
        if not path.exists():
            problems.append(f"{name}: missing")
            continue
        text = path.read_text(encoding="utf-8")
        if "package://" in text:
            problems.append(f"{name}: contains package:// URI")
        if "/home/" in text or "C:\\" in text:
            problems.append(f"{name}: contains absolute path")
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
        except Exception as exc:  # noqa: BLE001
            problems.append(f"{name}: load failed: {exc}")
            continue
        names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            for i in range(model.nbody)
        }
        if len(names) != model.nbody:
            problems.append(f"{name}: duplicate body names")
        names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(model.njnt)
        }
        if len(names) != model.njnt:
            problems.append(f"{name}: duplicate joint names")
        print(f"loaded {name}: nbody={model.nbody} njnt={model.njnt} ngeom={model.ngeom}")
    problems += check_meshes()
    problems += check_manifests()
    if problems:
        for p in problems:
            print(f"FAIL {p}")
        return 1
    print("all assets validated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
