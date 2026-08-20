"""Runtime attach modules, composition contract, and flattened releases."""

from __future__ import annotations

import shutil
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import pytest
import yaml
from helpers import MODEL_ROOT, REPO_ROOT

sys.path.insert(0, str(REPO_ROOT / "tools"))

from model_builder.composition import load_robot_config  # noqa: E402

MODULES = sorted(
    path
    for path in (REPO_ROOT / "models").glob("**/*.metadata.yaml")
    if "dependencies" not in path.parts
)
RELEASES = (
    "mobile_fr3_duo.xml",
    "mobile_fr3_duo_position.xml",
    "mobile_fr3_duo_with_sensors.xml",
    "mobile_fr3_duo_reduced.xml",
    "mobile_fr3_duo_planar_debug.xml",
)


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


@pytest.mark.parametrize("name", RELEASES)
def test_composition_and_flattened_release_have_same_model_shape(name):
    composed = mujoco.MjModel.from_xml_path(str(MODEL_ROOT / name))
    flattened = mujoco.MjModel.from_xml_path(
        str(MODEL_ROOT / f"{Path(name).stem}_flattened.xml")
    )
    assert (composed.nbody, composed.njnt, composed.ngeom, composed.nu, composed.nsensor) == (
        flattened.nbody,
        flattened.njnt,
        flattened.ngeom,
        flattened.nu,
        flattened.nsensor,
    )


def test_composition_has_unique_attach_prefixes():
    xml = ET.parse(MODEL_ROOT / "mobile_fr3_duo.xml").getroot()
    attaches = xml.findall(".//attach")
    assert len(attaches) == 1
    assert attaches[0].get("model") == "mobile_base"


def test_flattened_release_loads_without_module_xml(tmp_path):
    release = tmp_path / "release"
    model_dir = release / "models"
    shutil.copytree(MODEL_ROOT, model_dir)
    for path in model_dir.glob("**/*.xml"):
        if path.name != "mobile_fr3_duo_flattened.xml":
            path.unlink()
    root = ET.parse(model_dir / "mobile_fr3_duo_flattened.xml").getroot()
    assert not root.findall(".//model")
    assert not root.findall(".//attach")
    model = mujoco.MjModel.from_xml_path(str(model_dir / "mobile_fr3_duo_flattened.xml"))
    assert model.nbody == 101


def test_invalid_runtime_config_is_rejected(tmp_path):
    payload = yaml.safe_load(
        (REPO_ROOT / "config/robots/mobile_fr3_duo.yaml").read_text(encoding="utf-8")
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
