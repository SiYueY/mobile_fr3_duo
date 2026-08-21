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


def test_build_robot_without_external_source_checkout(tmp_path):
    clone = tmp_path / "mobile_fr3_duo"
    shutil.copytree(
        REPO_ROOT,
        clone,
        ignore=shutil.ignore_patterns(".git", ".venv", "__pycache__", ".pytest_cache"),
    )
    environment = os.environ.copy()
    environment.pop("MOBILE_FR3_CACHE_DIR", None)
    result = subprocess.run(
        [sys.executable, "tools/build_robot.py"],
        cwd=clone,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    output = clone / "models" / "mobile_fr3_duo.xml"
    assert output.is_file()
    assert ET.parse(output).find("contact") is not None
    model = mujoco.MjModel.from_xml_path(str(output))
    assert model.nbody > 0


def test_temporary_variants_do_not_enter_models():
    formal = REPO_ROOT / "models" / "mobile_fr3_duo.xml"
    before = formal.read_bytes()
    result = subprocess.run(
        [sys.executable, "tools/build_robot.py", "--variant", "position"],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert (REPO_ROOT / "build" / "mobile_fr3_duo_position.xml").is_file()
    assert not (REPO_ROOT / "models" / "mobile_fr3_duo_position.xml").exists()
    assert formal.read_bytes() == before


def test_source_preparation_requires_explicit_cache_path():
    """Preparation CLIs never fall back to a developer-specific cache path."""
    environment = os.environ.copy()
    environment.pop("MOBILE_FR3_CACHE_DIR", None)
    environment.pop("FRANKA_DESCRIPTION_ROOT", None)
    commands = (
        (["bash", "tools/generate_urdf.sh"], "MOBILE_FR3_CACHE_DIR"),
        ([sys.executable, "tools/convert_visual_meshes.py"], "FRANKA_DESCRIPTION_ROOT"),
        ([sys.executable, "tools/convert_collision_meshes.py"], "FRANKA_DESCRIPTION_ROOT"),
        ([sys.executable, "tools/import_sensor_assets.py"], "MOBILE_FR3_CACHE_DIR"),
        ([sys.executable, "tools/verify_official_model_files.py"], "MOBILE_FR3_CACHE_DIR"),
    )
    for command, expected in commands:
        result = subprocess.run(
            command,
            cwd=REPO_ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        assert result.returncode == 2, result.stderr
        assert expected in result.stderr


def test_frozen_production_inputs_contain_no_absolute_paths():
    generated = REPO_ROOT / "source" / "generated"
    assert not (generated / "mobile_fr3_duo_raw.xml").exists()
    for path in (
        generated / "mobile_fr3_duo_visual.urdf",
        generated / "mobile_fr3_duo_self_collision.urdf",
        generated / "mobile_fr3_duo_reduced.urdf",
        generated / "collision_exclusions.yaml",
    ):
        text = path.read_text(encoding="utf-8")
        assert "/home/" not in text
        assert "C:\\" not in text
