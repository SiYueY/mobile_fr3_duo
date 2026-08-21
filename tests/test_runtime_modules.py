"""Independent component modules and the formal whole-robot contract."""

from __future__ import annotations

import shutil
import sys
from xml.etree import ElementTree as ET

import mujoco
import pytest
import yaml
from helpers import MODEL_ROOT, REPO_ROOT

sys.path.insert(0, str(REPO_ROOT / "tools"))

from model_builder.composition import load_robot_config  # noqa: E402

MODULES = sorted((REPO_ROOT / "models").glob("*/*.metadata.yaml"))


def test_modules_load_and_metadata_interfaces_exist(tmp_path):
    assert MODULES
    for metadata_path in MODULES:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        package = tmp_path / f"{metadata_path.parent.name}_{metadata_path.stem}"
        shutil.copytree(metadata_path.parent, package)
        xml_path = package / metadata_path.with_suffix("").with_suffix(".xml").name
        assert xml_path.exists(), metadata_path
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, metadata["root_body"])
        assert root_id >= 0, metadata_path
        for interface in metadata["attachments"]:
            assert mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, interface["body"]
            ) >= 0, (metadata_path, interface)


def test_formal_model_is_a_direct_complete_mjcf():
    xml = ET.parse(MODEL_ROOT / "mobile_fr3_duo.xml").getroot()
    assert not xml.findall(".//attach")
    assert not xml.findall(".//asset/model")


def test_component_modules_do_not_embed_other_components():
    arm = mujoco.MjModel.from_xml_path(str(MODEL_ROOT / "franka_fr3/franka_fr3.xml"))
    assert mujoco.mj_name2id(arm, mujoco.mjtObj.mjOBJ_BODY, "fr3v2_1_hand") == -1
    base = mujoco.MjModel.from_xml_path(str(MODEL_ROOT / "franka_tmr/franka_tmr.xml"))
    assert mujoco.mj_name2id(base, mujoco.mjtObj.mjOBJ_BODY, "franka_spine") == -1


def test_formal_composition_has_full_sensor_set():
    model = mujoco.MjModel.from_xml_path(str(MODEL_ROOT / "mobile_fr3_duo.xml"))
    assert model.nbody == 109
    assert model.nsensor == 84


def test_invalid_runtime_config_is_rejected(tmp_path):
    payload = yaml.safe_load(
        (REPO_ROOT / "config/robot/mobile_fr3_duo.yaml").read_text(encoding="utf-8")
    )
    payload["arms"]["right"]["prefix"] = payload["arms"]["left"]["prefix"]
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="prefixes"):
        load_robot_config(path)

    payload["arms"]["right"]["prefix"] = "right_"
    payload["sensors"][0]["mount"] = "not_a_mounting_point"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="invalid sensor mount"):
        load_robot_config(path)
