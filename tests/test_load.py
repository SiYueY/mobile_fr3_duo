"""Loading tests: all variants load, names unique, keyframes consistent."""

import xml.etree.ElementTree as ET

import mujoco
import pytest
from helpers import MODEL_ROOT, load

VARIANTS = [
    "mobile_fr3_duo.xml",
    "scene.xml",
]


@pytest.mark.parametrize("xml", VARIANTS)
def test_variant_loads(xml):
    model = load(xml)
    assert model.nbody > 0
    assert model.njnt >= 20


@pytest.mark.parametrize("xml", VARIANTS)
def test_no_duplicate_names(xml):
    model = load(xml)
    counts = {
        mujoco.mjtObj.mjOBJ_BODY: model.nbody,
        mujoco.mjtObj.mjOBJ_JOINT: model.njnt,
        mujoco.mjtObj.mjOBJ_GEOM: model.ngeom,
        mujoco.mjtObj.mjOBJ_SITE: model.nsite,
        mujoco.mjtObj.mjOBJ_ACTUATOR: model.nu,
        mujoco.mjtObj.mjOBJ_SENSOR: model.nsensor,
    }
    for obj_type, count in counts.items():
        names = [
            mujoco.mj_id2name(model, obj_type, i)
            for i in range(count)
        ]
        names = [n for n in names if n]
        assert len(names) == len(set(names)), f"duplicate {obj_type} names in {xml}"


def test_keyframe_lengths(base_model):
    assert base_model.nkey == 6
    for k in range(base_model.nkey):
        assert base_model.key_qpos[k].shape[0] == base_model.nq
        assert base_model.key_ctrl[k].shape[0] == base_model.nu


def test_actuator_targets_exist(base_model):
    for i in range(base_model.nu):
        jid = base_model.actuator_trnid[i, 0]
        assert jid >= 0 and jid < base_model.njnt


def test_no_state_override_or_helpers():
    """No qpos/qvel overrides or planar proxies in the official variants."""
    for xml in ("mobile_fr3_duo.xml",):
        text = (MODEL_ROOT / xml).read_text(encoding="utf-8")
        assert "<qpos" not in text
        assert "<qvel" not in text
    text = (MODEL_ROOT / "mobile_fr3_duo.xml").read_text(encoding="utf-8")
    assert "planar" not in text


@pytest.mark.parametrize(
    ("scene", "robot"),
    (
        ("scene.xml", "mobile_fr3_duo.xml"),
    ),
)
def test_scene_is_thin_environment_include(scene, robot):
    """Scenes own only Menagerie-style environment elements and one include."""
    root = ET.parse(MODEL_ROOT / scene).getroot()
    includes = root.findall("include")
    assert [include.get("file") for include in includes] == [robot]
    assert (MODEL_ROOT / robot).is_file()
    assert root.find("./asset/model") is None
    assert root.find("./worldbody/attach") is None
    for tag in ("contact", "equality", "actuator", "sensor", "keyframe"):
        assert root.find(tag) is None

    assert root.find("./visual/headlight").attrib == {
        "diffuse": "0.6 0.6 0.6",
        "ambient": "0.3 0.3 0.3",
        "specular": "0 0 0",
    }
    assert root.find("./visual/rgba").get("haze") == "0.15 0.25 0.35 1"
    assert root.find("./asset/texture[@type='skybox']") is not None
    assert root.find("./asset/texture[@name='groundplane']") is not None
    assert root.find("./asset/material[@name='groundplane']") is not None
    ground = root.find("./worldbody/geom[@name='ground']")
    assert ground is not None
    assert ground.get("material") == "groundplane"
    assert ground.get("pos") == "0 0 -0.001"
    assert ground.get("group") == "1"
    assert ground.get("condim") == "3"
    assert root.find("./worldbody/camera[@name='preview']") is not None


@pytest.mark.parametrize(
    ("scene", "robot"),
    (
        ("scene.xml", "mobile_fr3_duo.xml"),
    ),
)
def test_scene_adds_only_ground_to_robot(scene, robot):
    scene_model = load(scene)
    robot_model = load(robot)
    assert scene_model.nbody == robot_model.nbody
    assert scene_model.njnt == robot_model.njnt
    assert scene_model.ngeom == robot_model.ngeom + 1
    assert scene_model.nu == robot_model.nu
    assert scene_model.nsensor == robot_model.nsensor


@pytest.mark.parametrize(
    "scene",
    ("scene.xml",),
)
def test_scene_loads_from_command_line_relative_path(scene):
    """Match ``simulate ./models/scene.xml`` resource resolution."""
    model = mujoco.MjModel.from_xml_path(f"./models/{scene}")
    assert model.nbody > 0
