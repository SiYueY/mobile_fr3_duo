"""Convert official visual DAE meshes to OBJ assets in meters.

DAE files may declare non-meter units (e.g. mm for tmrv0_2/franka_spine).
We normalize every mesh to meters so the MuJoCo assets are unit-consistent
with the kinematic frames and the official collision STLs (which are meters).
Records input/output SHA-256, applied unit scale and tool version in
source/generated/asset_conversion.json.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

import trimesh
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CACHE = Path("/home/siyuey/workspace/mujoco/_third_party_cache/franka_description")
ASSETS = REPO_ROOT / "assets"
GENERATED = REPO_ROOT / "source" / "generated"

# package://franka_description/meshes/<...>/<kind>/<file> -> assets/<component>/<kind>/<file>
COMPONENT_MAP = {
    "accessories/fr3_duo_mount_v0_3": "fr3_duo_mount_v0_3",
    "accessories/franka_head_v0_2": "franka_head_v0_2",
    "accessories/franka_spine_v0_1": "franka_spine_v0_1",
    "robots/fr3v2_1": "fr3v2_1",
    "robots/tmrv0_2": "tmrv0_2",
    "robot_ee/franka_hand_white": "franka_hand",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dae_unit_scale(path: Path) -> float:
    """Return the meters-per-unit scale declared by a COLLADA file."""
    text = path.read_text(errors="ignore")
    m = re.search(r'<unit[^>]*meter\s*=\s*"([0-9.eE+-]+)"', text, re.IGNORECASE)
    if m is None:
        # COLLADA default is meters.
        return 1.0
    return float(m.group(1))


def source_path(package_uri: str) -> Path | None:
    if not package_uri.startswith("package://franka_description/meshes/"):
        return None
    rel = package_uri.removeprefix("package://franka_description/meshes/")
    return CACHE / "meshes" / rel


def target_path(package_uri: str, suffix: str) -> Path | None:
    rel = package_uri.removeprefix("package://franka_description/meshes/")
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
    out_name = Path(filename).stem + suffix
    return ASSETS / component / kind / out_name


def convert_all(force: bool = False) -> dict:
    manifest = yaml.safe_load((REPO_ROOT / "source" / "asset_manifest.yaml").read_text())
    records: dict[str, dict] = {}
    for asset in manifest["assets"]:
        uri = asset["source"]
        if not uri.endswith(".dae"):
            continue
        src = source_path(uri)
        dst = target_path(uri, ".obj")
        if src is None or dst is None:
            print(f"skip (unmapped): {uri}")
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        if dst.exists() and not force:
            records[uri] = {
                "status": "up-to-date",
                "input_sha256": sha256(src),
                "output_sha256": sha256(dst),
            }
            continue
        mesh = trimesh.load(src, force="mesh")
        unit_scale = dae_unit_scale(src)
        if unit_scale != 1.0:
            mesh.apply_scale(unit_scale)
        mesh.export(dst)
        records[uri] = {
            "status": "converted",
            "tool": f"trimesh {trimesh.__version__}",
            "unit_scale": unit_scale,
            "input_sha256": sha256(src),
            "output_sha256": sha256(dst),
            "n_vertices": int(len(mesh.vertices)),
            "n_faces": int(len(mesh.faces)),
            "bounds": [mesh.bounds.tolist()],
        }
        print(f"converted {dst.relative_to(REPO_ROOT)}")
    return records


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--force", action="store_true", help="reconvert existing assets")
    args = ap.parse_args()
    GENERATED.mkdir(parents=True, exist_ok=True)
    records = convert_all(force=args.force)
    out = GENERATED / "asset_conversion.json"
    out.write_text(json.dumps(records, indent=2, sort_keys=True))
    print(f"wrote {out} ({len(records)} visual assets)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
