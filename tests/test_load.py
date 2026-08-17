"""Loading tests: all variants load, names unique, keyframes consistent."""

import mujoco
import pytest
from helpers import REPO_ROOT, load

VARIANTS = [
    "mobile_fr3_duo.xml",
    "mobile_fr3_duo_with_sensors.xml",
    "mobile_fr3_duo_position.xml",
    "mobile_fr3_duo_reduced.xml",
    "mobile_fr3_duo_planar_debug.xml",
    "scene.xml",
    "scene_with_sensors.xml",
    "scene_position.xml",
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
    for xml in ("mobile_fr3_duo.xml", "mobile_fr3_duo_with_sensors.xml"):
        text = (REPO_ROOT / xml).read_text(encoding="utf-8")
        assert "<qpos" not in text
        assert "<qvel" not in text
    text = (REPO_ROOT / "mobile_fr3_duo.xml").read_text(encoding="utf-8")
    assert "planar" not in text
