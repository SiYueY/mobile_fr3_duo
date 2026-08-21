"""Validate formal Mobile FR3 Duo MJCF assets and source manifests."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import trimesh

REPO_ROOT = Path(__file__).resolve().parent.parent
MODEL_ROOT = REPO_ROOT / "models"
FORMAL_MODELS = ("mobile_fr3_duo.xml", "scene.xml")
MIN_EXTENT, MAX_EXTENT = 1e-4, 3.0


def check_meshes() -> list[str]:
    problems: list[str] = []
    for mesh_path in sorted(MODEL_ROOT.glob("**/assets/**/*.obj")) + sorted(MODEL_ROOT.glob("**/assets/**/*.stl")):
        try:
            mesh = trimesh.load(mesh_path, force="mesh")
        except Exception as exc:  # noqa: BLE001
            problems.append(f"{mesh_path}: load failed: {exc}")
            continue
        if len(mesh.faces) == 0:
            problems.append(f"{mesh_path}: no faces")
        if mesh_path.suffix == ".stl" and len(mesh.area_faces) and int((mesh.area_faces <= 0).sum()):
            problems.append(f"{mesh_path}: degenerate faces")
        extent = mesh.extents.max()
        if extent < MIN_EXTENT or extent > MAX_EXTENT:
            problems.append(f"{mesh_path}: suspicious extent {extent:.5g}")
    return problems


def check_manifests() -> list[str]:
    problems: list[str] = []
    if (REPO_ROOT / "assets").exists():
        problems.append("assets/: legacy root asset directory must not exist")
    if (MODEL_ROOT / "sensors").exists():
        problems.append("models/sensors/: sensor modules must be top-level packages")
    for path in MODEL_ROOT.glob("*/dependencies"):
        problems.append(f"{path.relative_to(REPO_ROOT)}: nested dependencies are forbidden")
    for rel in (
        "source/asset_manifest.yaml", "source/link_manifest.yaml", "source/joint_manifest.yaml",
        "source/inertial_manifest.yaml", "source/frame_manifest.yaml", "source/name_mapping.yaml",
        "source/parameter_sources.yaml", "source/official_model_files.yaml",
        "source/generated/asset_conversion.json", "source/generated/asset_collision_conversion.json",
        "source/generated/sensor_asset_conversion.json",
    ):
        if not (REPO_ROOT / rel).exists():
            problems.append(f"{rel}: missing")
    for rel in (
        "source/generated/asset_conversion.json", "source/generated/asset_collision_conversion.json",
        "source/generated/sensor_asset_conversion.json",
    ):
        try:
            data = json.loads((REPO_ROOT / rel).read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            problems.append(f"{rel}: invalid json: {exc}")
            continue
        for record in data.values() if isinstance(data, dict) else ():
            if not isinstance(record, dict):
                continue
            paths = [record.get("path"), record.get("asset")]
            paths.extend(item.get("path") for item in record.get("outputs", []) if isinstance(item, dict))
            for asset in filter(None, paths):
                if not isinstance(asset, str) or not asset.startswith("models/"):
                    problems.append(f"{rel}: invalid asset path {asset!r}")
                elif not (REPO_ROOT / asset).is_file():
                    problems.append(f"{rel}: missing {asset}")
    return problems


def main() -> int:
    problems: list[str] = []
    for name in FORMAL_MODELS:
        path = MODEL_ROOT / name
        if not path.exists():
            problems.append(f"{name}: missing")
            continue
        text = path.read_text(encoding="utf-8")
        if "package://" in text or "/home/" in text or "C:\\" in text:
            problems.append(f"{name}: contains non-portable resource path")
        if name == "mobile_fr3_duo.xml" and ("<attach" in text or "<model " in text):
            problems.append("mobile_fr3_duo.xml: must be a direct complete MJCF")
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
        except Exception as exc:  # noqa: BLE001
            problems.append(f"{name}: load failed: {exc}")
            continue
        for kind, count in ((mujoco.mjtObj.mjOBJ_BODY, model.nbody), (mujoco.mjtObj.mjOBJ_JOINT, model.njnt)):
            names = {mujoco.mj_id2name(model, kind, index) for index in range(count)}
            if len(names) != count:
                problems.append(f"{name}: duplicate MuJoCo names")
        print(f"loaded {name}: nbody={model.nbody} njnt={model.njnt} ngeom={model.ngeom}")
    problems.extend(check_meshes())
    problems.extend(check_manifests())
    extras = sorted(path.name for path in MODEL_ROOT.glob("*.xml") if path.name not in FORMAL_MODELS)
    if extras:
        problems.append(f"models/: non-formal top-level XML: {', '.join(extras)}")
    if problems:
        print(*[f"FAIL {problem}" for problem in problems], sep="\n")
        return 1
    print("all assets validated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
