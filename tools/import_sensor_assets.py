"""Import fixed-tag sensor meshes (realsense/zed/sick) into assets/sensors/.

All imported assets are normalized to meters so no MuJoCo scale attribute is
needed and mesh auto-centering stays compensated.
"""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

import trimesh

REPO_ROOT = Path(__file__).resolve().parent.parent
CACHE = Path("/home/siyuey/workspace/mujoco/_third_party_cache")
ASSETS = REPO_ROOT / "assets" / "sensors"
GENERATED = REPO_ROOT / "source" / "generated"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_stl(src: Path, name: str, scale: float = 1.0) -> dict:
    dst = ASSETS / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, dst)
    return {
        "source": str(src.relative_to(CACHE)),
        "asset": f"sensors/{name}",
        "scale": scale,
        "input_sha256": sha256(src),
        "output_sha256": sha256(dst),
    }


def convert_stl_meters(src: Path, name: str, scale: float) -> dict:
    """Copy an STL mesh applying a unit conversion so output is in meters."""
    dst = ASSETS / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    mesh = trimesh.load(src, force="mesh")
    mesh.apply_scale(scale)
    mesh.export(dst)
    return {
        "source": str(src.relative_to(CACHE)),
        "asset": f"sensors/{name}",
        "scale": scale,
        "input_sha256": sha256(src),
        "output_sha256": sha256(dst),
        "n_vertices": int(len(mesh.vertices)),
        "n_faces": int(len(mesh.faces)),
        "bounds": [mesh.bounds.tolist()],
    }


def convert_dae(src: Path, name: str) -> dict:
    dst = ASSETS / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    mesh = trimesh.load(src, force="mesh")
    mesh.export(dst)
    return {
        "source": str(src.relative_to(CACHE)),
        "asset": f"sensors/{name}",
        "input_sha256": sha256(src),
        "output_sha256": sha256(dst),
        "n_vertices": int(len(mesh.vertices)),
        "n_faces": int(len(mesh.faces)),
    }


def main() -> int:
    records = {
        "realsense_d455": convert_stl_meters(
            CACHE / "realsense-ros" / "realsense2_description" / "meshes" / "d455.stl",
            "realsense_d455/d455.stl",
            scale=0.001,
        ),
        "sick_nanoscan3_visual": convert_dae(
            CACHE / "sick_safetyscanners2" / "description" / "meshes" / "NANS3.dae",
            "sick_nanoscan3/NANS3.obj",
        ),
        "sick_nanoscan3_collision": copy_stl(
            CACHE / "sick_safetyscanners2" / "description" / "meshes" / "NANS3_collision.stl",
            "sick_nanoscan3/NANS3_collision.stl",
        ),
        "zed_mini": copy_stl(
            CACHE / "zed-ros2-description" / "meshes" / "zedm.stl",
            "zed_mini/zedm.stl",
            scale=1.0,
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
