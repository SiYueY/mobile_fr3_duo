"""Import fixed-tag sensor meshes into self-contained sensor modules.

All imported assets are normalized to meters so no MuJoCo scale attribute is
needed and mesh auto-centering stays compensated.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path

import trimesh

REPO_ROOT = Path(__file__).resolve().parents[2]
SENSOR_ASSETS = REPO_ROOT / "models"
GENERATED = REPO_ROOT / "source" / "generated"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_stl(
    src: Path,
    name: str,
    cache: Path,
    scale: float = 1.0,
    appearance: dict[str, object] | None = None,
) -> dict:
    dst = SENSOR_ASSETS / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, dst)
    record = {
        "source": str(src.relative_to(cache)),
        "asset": f"models/{name}",
        "scale": scale,
        "input_sha256": sha256(src),
        "output_sha256": sha256(dst),
    }
    if appearance is not None:
        record["appearance"] = appearance
    return record


def convert_stl_meters(
    src: Path, name: str, cache: Path, scale: float, appearance: dict[str, object]
) -> dict:
    """Copy an STL mesh applying a unit conversion so output is in meters."""
    dst = SENSOR_ASSETS / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    mesh = trimesh.load(src, force="mesh")
    mesh.apply_scale(scale)
    mesh.export(dst)
    return {
        "source": str(src.relative_to(cache)),
        "asset": f"models/{name}",
        "scale": scale,
        "input_sha256": sha256(src),
        "output_sha256": sha256(dst),
        "n_vertices": int(len(mesh.vertices)),
        "n_faces": int(len(mesh.faces)),
        "bounds": [mesh.bounds.tolist()],
        "appearance": appearance,
    }


def convert_stl_obj(
    src: Path, name: str, cache: Path, scale: float, appearance: dict[str, object]
) -> dict:
    """Convert an upstream STL visual mesh to a metre-scale OBJ mesh."""
    dst = SENSOR_ASSETS / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    mesh = trimesh.load(src, force="mesh")
    mesh.apply_scale(scale)
    mesh.export(dst, include_texture=False)
    return {
        "source": str(src.relative_to(cache)),
        "asset": f"models/{name}",
        "scale": scale,
        "input_sha256": sha256(src),
        "output_sha256": sha256(dst),
        "n_vertices": int(len(mesh.vertices)),
        "n_faces": int(len(mesh.faces)),
        "bounds": [mesh.bounds.tolist()],
        "appearance": appearance,
    }


def convert_dae(src: Path, name: str, cache: Path, appearance: dict[str, object]) -> dict:
    dst = SENSOR_ASSETS / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    mesh = trimesh.load(src, force="mesh")
    # MJCF materials are the sole rendering authority.  Keep the OBJ as a
    # portable geometry carrier and do not generate a sidecar MTL file.
    mesh.export(dst, include_texture=False)
    return {
        "source": str(src.relative_to(cache)),
        "asset": f"models/{name}",
        "input_sha256": sha256(src),
        "output_sha256": sha256(dst),
        "n_vertices": int(len(mesh.vertices)),
        "n_faces": int(len(mesh.faces)),
        "appearance": appearance,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--cache",
        type=Path,
        default=(Path(os.environ["MOBILE_FR3_CACHE_DIR"]) if "MOBILE_FR3_CACHE_DIR" in os.environ else None),
        help="third-party source cache (or MOBILE_FR3_CACHE_DIR)",
    )
    args = ap.parse_args()
    if args.cache is None:
        ap.error("pass --cache or set MOBILE_FR3_CACHE_DIR")
    cache = args.cache
    records = {
        "realsense_d455": convert_stl_obj(
            cache / "realsense-ros" / "realsense2_description" / "meshes" / "d455.stl",
            "realsense_d455/assets/visual/d455.obj",
            cache,
            scale=0.001,
            appearance={"name": "aluminum", "rgba": [0.5, 0.5, 0.5, 1.0]},
        ),
        "sick_nanoscan3_visual": convert_dae(
            cache / "sick_safetyscanners2" / "description" / "meshes" / "NANS3.dae",
            "nanoscan3/assets/visual/NANS3.obj",
            cache,
            # Every official NANS3 COLLADA material slot has the same
            # diffuse colour, so one explicit MJCF material faithfully
            # represents the source without needless geometry splitting.
            appearance={"name": "nanoscan3_darkgrey", "rgba": [0.2, 0.2, 0.2, 1.0]},
        ),
        "zed_mini": convert_stl_obj(
            cache / "zed-ros2-description" / "meshes" / "zedm.stl",
            "zed_mini/assets/visual/zedm.obj",
            cache,
            scale=1.0,
            appearance={"name": "cam_mat", "rgba": [0.0, 0.0, 0.0, 1.0]},
        ),
    }
    GENERATED.mkdir(parents=True, exist_ok=True)
    (GENERATED / "sensor_asset_conversion.json").write_text(
        json.dumps(records, indent=2, sort_keys=True)
    )
    for name, rec in records.items():
        print(f"imported {name}: {rec['asset']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
