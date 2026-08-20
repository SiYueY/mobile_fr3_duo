"""Assemble the v1.0.0 release package (document stage 8).

Collects the native model deliverables into dist/mobile_fr3_duo_v1.0.0/ and
produces a tarball with a SHA-256 checksum and a content manifest.

Usage:
  python tools/package_release.py
"""

from __future__ import annotations

import hashlib
import shutil
import tarfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
VERSION = "v1.0.0"
DIST = REPO_ROOT / "dist"
PKG = DIST / f"mobile_fr3_duo_{VERSION}"
TARBALL = DIST / f"mobile_fr3_duo_{VERSION}.tar.gz"


def _top_level_entries() -> list[str]:
    return [
        "models",
        "mobile_fr3_duo.png",
        "mobile_fr3_duo_manipulation.png",
        "mobile_fr3_duo_with_sensors.png",
        "README.md",
        "CHANGELOG.md",
        "LICENSE",
        "mobile_fr3_duo.md",
        "Makefile",
        "pyproject.toml",
        "uv.lock",
        "source",
        "config",
        "examples",
        "tests",
        "tools",
        "docs",
    ]


def main() -> int:
    if PKG.exists():
        shutil.rmtree(PKG)
    DIST.mkdir(parents=True, exist_ok=True)
    PKG.mkdir(parents=True)
    for entry in _top_level_entries():
        src = REPO_ROOT / entry
        dst = PKG / entry
        if src.is_dir():
            shutil.copytree(src, dst, ignore=shutil.ignore_patterns("__pycache__"))
        else:
            shutil.copy2(src, dst)
    # content manifest
    manifest_lines = [f"# Mobile FR3 Duo {VERSION} release package"]
    for p in sorted(PKG.rglob("*")):
        if p.is_file():
            rel = p.relative_to(PKG)
            digest = hashlib.sha256(p.read_bytes()).hexdigest()
            manifest_lines.append(f"{digest}  {rel}")
    (PKG / "SHA256SUMS.txt").write_text("\n".join(manifest_lines) + "\n")
    # tarball
    with tarfile.open(TARBALL, "w:gz") as tar:
        tar.add(PKG, arcname=PKG.name)
    tar_sha = hashlib.sha256(TARBALL.read_bytes()).hexdigest()
    print(f"package: {TARBALL}")
    print(f"files:   {len([p for p in PKG.rglob('*') if p.is_file()])}")
    print(f"sha256:  {tar_sha}")
    (DIST / f"mobile_fr3_duo_{VERSION}.tar.gz.sha256").write_text(
        f"{tar_sha}  mobile_fr3_duo_{VERSION}.tar.gz\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
