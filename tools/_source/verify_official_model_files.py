"""Verify official third-party model files against the fixed-tag manifest.

Usage:
  python tools/verify_official_model_files.py            # check local cache
  python tools/verify_official_model_files.py --write-manifest

The manifest (source/official_model_files.yaml) records the fixed upstream
tag, the resolved git commit, and the SHA-256 of every official file consumed
by the build pipeline. The deliverable repository never bundles these files;
the check is used in CI against the development cache.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import subprocess
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
MANIFEST = REPO_ROOT / "source" / "official_model_files.yaml"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git_check(cache: Path, repo: str, *args: str) -> str | None:
    try:
        out = subprocess.run(
            ["git", "-C", str(cache / repo), *args],
            capture_output=True,
            text=True,
            check=True,
        )
        return out.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def build_manifest(cache: Path) -> dict:
    manifest = {"generated_by": "tools/verify_official_model_files.py", "repos": {}}
    for repo, tag in (
        ("franka_description", "2.8.1"),
        ("franka_ros2", "v2.5.1"),
        ("realsense-ros", "4.58.3"),
        ("zed-ros2-description", "0.1.5"),
        ("sick_safetyscanners2", "1.0.5"),
    ):
        root = cache / repo
        commit = git_check(cache, repo, "rev-parse", "HEAD")
        files = {}
        if repo == "franka_description":
            relative = [
                "robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.urdf.xacro",
                "robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.srdf.xacro",
                "robots/fr3v2_1/fr3v2_1.urdf.xacro",
                "robots/fr3v2_1/fr3v2_1.srdf.xacro",
                "robots/common/franka_arm.srdf.xacro",
                "end_effectors/franka_hand/franka_hand.urdf.xacro",
                "end_effectors/franka_hand/franka_hand.srdf.xacro",
                "end_effectors/common/franka_hand.xacro",
                "robots/tmrv0_2/tmrv0_2.xacro",
                "robots/fr3v2_1/joint_limits.yaml",
                "robots/fr3v2_1/inertials.yaml",
                "robots/fr3v2_1/kinematics.yaml",
                "robots/fr3v2_1/dynamics.yaml",
                "robots/mobile_fr3_duo_v0_2/inertials.yaml",
            ]
            for uri in (m for m in _asset_sources()):
                relative.append(uri.removeprefix("package://franka_description/meshes/"))
        elif repo == "realsense-ros":
            relative = ["realsense2_description/meshes/d455.stl"]
        elif repo == "zed-ros2-description":
            relative = ["meshes/zedm.stl"]
        elif repo == "sick_safetyscanners2":
            relative = ["description/meshes/NANS3.dae", "description/meshes/NANS3_collision.stl"]
        else:
            relative = []
        for rel in sorted(set(relative)):
            p = root / rel
            if p.exists() and p.is_file():
                files[rel] = sha256(p)
        manifest["repos"][repo] = {
            "tag": tag,
            "commit": commit,
            "files": files,
        }
    return manifest


def _asset_sources():
    assets = yaml.safe_load((REPO_ROOT / "source" / "asset_manifest.yaml").read_text())
    return [a["source"] for a in assets["assets"]]


def check(cache: Path) -> tuple[list[str], dict]:
    manifest = yaml.safe_load(MANIFEST.read_text())
    problems: list[str] = []
    status = {}
    for repo, spec in manifest["repos"].items():
        root = cache / repo
        tag = git_check(cache, repo, "describe", "--tags", "--exact-match")
        commit = git_check(cache, repo, "rev-parse", "HEAD")
        repo_state = "official"
        if not root.is_dir():
            repo_state = "missing"
        elif commit != spec.get("commit") or tag != spec.get("tag"):
            repo_state = "wrong-tag"
        if repo_state != "official":
            problems.append(f"{repo}: {repo_state} (expected tag {spec['tag']})")
        status[repo] = repo_state
        for rel, expected in spec.get("files", {}).items():
            p = root / rel
            if not p.exists():
                problems.append(f"{repo}/{rel}: missing")
                continue
            actual = sha256(p)
            if actual != expected:
                problems.append(f"{repo}/{rel}: modified (sha256 mismatch)")
    return problems, status


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--cache",
        type=Path,
        default=(Path(os.environ["MOBILE_FR3_CACHE_DIR"]) if "MOBILE_FR3_CACHE_DIR" in os.environ else None),
        help="third-party source cache (or MOBILE_FR3_CACHE_DIR)",
    )
    ap.add_argument("--write-manifest", action="store_true")
    ap.add_argument(
        "--offline",
        action="store_true",
        help="validate the manifest structure only (CI without the cache)",
    )
    args = ap.parse_args()
    if args.offline and args.write_manifest:
        ap.error("--offline cannot be combined with --write-manifest")
    if args.offline:
        if not MANIFEST.exists():
            print(f"manifest missing: {MANIFEST}; run with --write-manifest first")
            return 2
        manifest = yaml.safe_load(MANIFEST.read_text())
        for repo, spec in manifest.get("repos", {}).items():
            assert spec.get("tag"), f"{repo}: missing tag"
            assert spec.get("commit"), f"{repo}: missing commit"
            assert isinstance(spec.get("files"), dict), f"{repo}: invalid file list"
        print("offline manifest validation passed")
        return 0
    if args.cache is None:
        ap.error("pass --cache or set MOBILE_FR3_CACHE_DIR (or use --offline)")
    if args.write_manifest:
        MANIFEST.parent.mkdir(parents=True, exist_ok=True)
        manifest = build_manifest(args.cache)
        MANIFEST.write_text(yaml.safe_dump(manifest, sort_keys=True, allow_unicode=True))
        print(f"wrote {MANIFEST}")
        return 0
    if not MANIFEST.exists():
        print(f"manifest missing: {MANIFEST}; run with --write-manifest first")
        return 2
    problems, status = check(args.cache)
    for repo, state in status.items():
        print(f"{repo}: {state}")
    if problems:
        for p in problems:
            print(f"  FAIL {p}")
        return 1
    print("all official model files verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
