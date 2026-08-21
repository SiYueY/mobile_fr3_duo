import os
import subprocess
import sys
from pathlib import Path

import mujoco
import pytest

sys.path.insert(0, os.path.dirname(__file__))

from helpers import REPO_ROOT, VISUAL_URDF, UrdfModel, load  # noqa: E402


@pytest.fixture(scope="session")
def urdf() -> UrdfModel:
    assert VISUAL_URDF.exists(), "visual URDF not generated"
    return UrdfModel()


@pytest.fixture(scope="session")
def base_model():
    return load("mobile_fr3_duo.xml")


@pytest.fixture(scope="session")
def sensor_model():
    return load("mobile_fr3_duo.xml")


@pytest.fixture(scope="session")
def scene_model():
    return load("scene.xml")


@pytest.fixture(scope="session")
def position_model():
    """Position control is a temporary build artifact, never a release XML."""
    subprocess.run(
        [sys.executable, str(REPO_ROOT / "tools" / "build_robot.py"), "--variant", "position"],
        cwd=REPO_ROOT,
        check=True,
    )
    return mujoco.MjModel.from_xml_path(str(Path(REPO_ROOT) / "build/mobile_fr3_duo_position.xml"))


@pytest.fixture(scope="session")
def repo_root() -> str:
    return str(REPO_ROOT)
