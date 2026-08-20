"""Copy official collision STL assets into their self-contained modules."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path

import trimesh
import yaml
from convert_visual_meshes import source_path, target_path

REPO_ROOT = Path(__file__).resolve().parent.parent
GENERATED = REPO_ROOT / "source" / "generated"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_stl(path: Path) -> dict:
    mesh = trimesh.load(path, force="mesh")
    return {
        "n_vertices": int(len(mesh.vertices)),
        "n_faces": int(len(mesh.faces)),
        "is_watertight": bool(mesh.is_watertight),
        "min_face_area": float(mesh.area_faces.min()) if len(mesh.area_faces) else 0.0,
        "n_degenerate": int((mesh.area_faces <= 0).sum()) if len(mesh.area_faces) else 0,
    }


def convert_all(franka_root: Path, force: bool = False) -> dict:
    manifest = yaml.safe_load((REPO_ROOT / "source" / "asset_manifest.yaml").read_text())
    records: dict[str, dict] = {}
    for asset in manifest["assets"]:
        uri = asset["source"]
        if not uri.endswith(".stl"):
            continue
        src = source_path(uri, franka_root)
        dst = target_path(uri, ".stl")
        if src is None or dst is None:
            print(f"skip (unmapped): {uri}")
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        if dst.exists() and not force:
            records[uri] = {
                "status": "up-to-date",
                "path": dst.relative_to(REPO_ROOT).as_posix(),
                "input_sha256": sha256(src),
                "output_sha256": sha256(dst),
            }
            continue
        shutil.copyfile(src, dst)
        validation = validate_stl(dst)
        records[uri] = {
            "status": "copied",
            "path": dst.relative_to(REPO_ROOT).as_posix(),
            **validation,
            "input_sha256": sha256(src),
            "output_sha256": sha256(dst),
        }
        print(f"copied {dst.relative_to(REPO_ROOT)} "
              f"[watertight={validation['is_watertight']}]")
    return records


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--franka-root",
        type=Path,
        default=(Path(os.environ["FRANKA_DESCRIPTION_ROOT"]) if "FRANKA_DESCRIPTION_ROOT" in os.environ else None),
        help="fixed franka_description checkout (or FRANKA_DESCRIPTION_ROOT)",
    )
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    if args.franka_root is None:
        ap.error("pass --franka-root or set FRANKA_DESCRIPTION_ROOT")
    GENERATED.mkdir(parents=True, exist_ok=True)
    records = convert_all(args.franka_root, force=args.force)
    out = GENERATED / "asset_collision_conversion.json"
    out.write_text(json.dumps(records, indent=2, sort_keys=True))
    print(f"wrote {out} ({len(records)} collision assets)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
