"""Builder YAML configuration is complete and is reflected in generated MJCF."""

from __future__ import annotations

import sys

import mujoco
import numpy as np
import pytest
import yaml
from helpers import REPO_ROOT, load

sys.path.insert(0, str(REPO_ROOT / "tools"))

from model_builder.config import EXPECTED_KEYFRAMES  # noqa: E402
from model_builder.config import load as load_builder_config


def _actuator_range(model: mujoco.MjModel, name: str) -> np.ndarray:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    assert aid >= 0, f"missing actuator {name}"
    return model.actuator_ctrlrange[aid]


def test_builder_config_is_complete_and_validated(tmp_path):
    config = load_builder_config(REPO_ROOT / "config")

    assert tuple(config.keyframes) == EXPECTED_KEYFRAMES
    assert len(config.tmr) == 4
    assert len(config.planar) == 3
    for pose in config.keyframes.values():
        assert len(pose["arm_left"]) == len(pose["arm_right"]) == 7
    with pytest.raises(ValueError, match="missing Builder configuration"):
        load_builder_config(tmp_path)


def test_generated_actuator_limits_match_builder_config(base_model):
    config = load_builder_config(REPO_ROOT / "config")
    for spec in (*config.tmr, config.spine):
        assert np.allclose(_actuator_range(base_model, spec.name), spec.ctrlrange)
    for side in ("left", "right"):
        name = f"{side}_fr3v2_1_finger_motor"
        assert np.allclose(_actuator_range(base_model, name), config.hand_ctrlrange)
        aid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert np.allclose(base_model.actuator_forcerange[aid], config.hand_forcerange)

    planar_model = load("mobile_fr3_duo_planar_debug.xml")
    for spec in config.planar:
        assert np.allclose(_actuator_range(planar_model, spec.name), spec.ctrlrange)


def test_generated_keyframes_match_builder_config(base_model):
    config = load_builder_config(REPO_ROOT / "config")
    for kid, (name, pose) in enumerate(config.keyframes.items()):
        assert mujoco.mj_id2name(base_model, mujoco.mjtObj.mjOBJ_KEY, kid) == name
        for joint, expected in (("franka_spine_vertical_joint", pose["spine"]),):
            jid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            assert base_model.key_qpos[kid, base_model.jnt_qposadr[jid]] == pytest.approx(expected)
        for side, arm in (("left", pose["arm_left"]), ("right", pose["arm_right"])):
            for index, expected in enumerate(arm, start=1):
                joint = f"{side}_fr3v2_1_joint{index}"
                jid = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, joint)
                assert base_model.key_qpos[kid, base_model.jnt_qposadr[jid]] == pytest.approx(expected)


def test_runtime_control_limits_are_not_mjcf_ctrlranges():
    """Runtime command policy is separate from Builder actuator capability."""
    controls = {
        path.name: yaml.safe_load(path.read_text(encoding="utf-8"))
        for path in (REPO_ROOT / "config" / "control").glob("*_control.yaml")
    }
    for payload in controls.values():
        assert "ctrlrange" not in str(payload)
    assert controls["tmr_control.yaml"]["wheel"]["command_limit"] == [-50.0, 50.0]
    assert controls["hand_control.yaml"]["hand"]["command_limit"] == [-20.0, 20.0]
    assert controls["spine_control.yaml"]["spine"]["command_limit"] == [-100.0, 100.0]


def test_builder_has_no_legacy_module_duplicates():
    source = (REPO_ROOT / "tools" / "build_model.py").read_text(encoding="utf-8")
    for method in (
        "_defaults",
        "_assets",
        "_children",
        "_emit_joint",
        "_emit_inertial",
        "_emit_inertial_in_frame",
        "_emit_visual",
        "_emit_collision",
        "_contacts",
        "_equalities",
    ):
        assert f"def {method}(" not in source
