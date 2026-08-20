"""The production MJCF build consumes only committed project inputs."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import yaml
from helpers import REPO_ROOT


def test_frozen_collision_exclusions_are_valid():
    path = REPO_ROOT / "source" / "generated" / "collision_exclusions.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    pairs = payload["disable_collisions"]

    assert payload["source"].startswith("franka_description@2.8.1/")
    assert pairs == sorted(pairs)
    assert len(pairs) == len({tuple(pair) for pair in pairs})
    assert all(len(pair) == 2 and all(isinstance(link, str) for link in pair) for pair in pairs)


def test_build_model_without_external_source_checkout(tmp_path):
    clone = tmp_path / "mobile_fr3_duo"
    shutil.copytree(
        REPO_ROOT,
        clone,
        ignore=shutil.ignore_patterns(".git", ".venv", "__pycache__", ".pytest_cache"),
    )
    environment = os.environ.copy()
    environment.pop("MOBILE_FR3_CACHE_DIR", None)
    result = subprocess.run(
        [sys.executable, "tools/build_model.py"],
        cwd=clone,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    output = clone / "mobile_fr3_duo.xml"
    assert output.is_file()
    assert ET.parse(output).find("contact") is not None
    model = mujoco.MjModel.from_xml_path(str(output))
    assert model.nbody > 0
