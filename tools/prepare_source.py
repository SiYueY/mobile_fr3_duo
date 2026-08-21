"""Prepare frozen source inputs from an explicit franka_description checkout.

This is the public source-preparation entry point.  It regenerates frozen
URDFs, mesh conversion manifests, and sensor assets before module/model build.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def _run(command: list[str], environment: dict[str, str]) -> None:
    subprocess.run(command, cwd=REPO_ROOT, env=environment, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--franka-root", type=Path, required=True)
    parser.add_argument(
        "--cache",
        type=Path,
        required=True,
        help="third-party cache containing franka_description and sensor source checkouts",
    )
    args = parser.parse_args()
    root = args.franka_root.resolve()
    if not (root / "robots").is_dir():
        parser.error(f"not a franka_description checkout: {root}")
    cache = args.cache.resolve()
    cached_franka = cache / "franka_description"
    if not cached_franka.is_dir():
        parser.error(f"cache is missing franka_description: {cached_franka}")
    if cached_franka.resolve() != root:
        parser.error("--cache/franka_description must be the same checkout as --franka-root")
    environment = os.environ | {
        "FRANKA_DESCRIPTION_ROOT": str(root),
        "MOBILE_FR3_CACHE_DIR": str(cache),
    }
    _run(["bash", "tools/_source/generate_urdf.sh"], environment)
    _run([sys.executable, "tools/_source/convert_visual_meshes.py", "--franka-root", str(root)], environment)
    _run([sys.executable, "tools/_source/convert_collision_meshes.py", "--franka-root", str(root)], environment)
    _run([sys.executable, "tools/_source/import_sensor_assets.py", "--cache", str(cache)], environment)
    _run([sys.executable, "tools/_source/extract_urdf_parameters.py"], environment)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
