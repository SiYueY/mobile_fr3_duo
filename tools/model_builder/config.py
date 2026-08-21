"""Validated Builder-only model configuration loaded from committed YAML."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

EXPECTED_TMR_JOINTS = (
    "tmrv0_2_joint_0",
    "tmrv0_2_joint_1",
    "tmrv0_2_joint_2",
    "tmrv0_2_joint_3",
)
EXPECTED_KEYFRAMES = (
    "home",
    "transport",
    "manipulation",
    "wide_workspace",
    "spine_min",
    "spine_max",
)


@dataclass(frozen=True)
class ActuatorSpec:
    joint: str
    name: str
    ctrlrange: tuple[float, float]


@dataclass(frozen=True)
class BuilderConfig:
    tmr: tuple[ActuatorSpec, ...]
    spine: ActuatorSpec
    hand_ctrlrange: tuple[float, float]
    hand_forcerange: tuple[float, float]
    keyframes: dict[str, dict[str, float | list[float]]]


def load(config_dir: Path) -> BuilderConfig:
    """Load Builder inputs and reject incomplete or malformed configuration."""
    actuator_data = _mapping(_read(config_dir / "actuator.yaml"), "actuator.yaml")
    robot_data = _mapping(_read(config_dir / "mobile_fr3_duo.yaml"), "mobile_fr3_duo.yaml")
    tmr = _actuators(actuator_data.get("tmr"), "tmr", EXPECTED_TMR_JOINTS, names=True)
    spine = _actuator(actuator_data.get("spine"), "spine", names=True)
    if spine.joint != "franka_spine_vertical_joint" or spine.name != "franka_spine_motor":
        raise ValueError("spine must define franka_spine_vertical_joint/franka_spine_motor")
    hand = _mapping(actuator_data.get("hand"), "hand")
    keyframes = _keyframes(robot_data.get("keyframes"))
    return BuilderConfig(
        tmr=tmr,
        spine=spine,
        hand_ctrlrange=_range(hand.get("ctrlrange"), "hand.ctrlrange"),
        hand_forcerange=_range(hand.get("forcerange"), "hand.forcerange"),
        keyframes=keyframes,
    )


def _read(path: Path) -> Any:
    if not path.is_file():
        raise ValueError(f"missing Builder configuration: {path}")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _actuators(value: Any, label: str, joints: tuple[str, ...], *, names: bool) -> tuple[ActuatorSpec, ...]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a list")
    specs = tuple(_actuator(item, f"{label}[{index}]", names=names) for index, item in enumerate(value))
    if tuple(spec.joint for spec in specs) != joints:
        raise ValueError(f"{label} must define joints in order: {', '.join(joints)}")
    return specs


def _actuator(value: Any, label: str, *, names: bool) -> ActuatorSpec:
    data = _mapping(value, label)
    joint = data.get("joint")
    if not isinstance(joint, str) or not joint:
        raise ValueError(f"{label}.joint must be a non-empty string")
    name = data.get("name", f"{joint}_actuator")
    if names and (not isinstance(name, str) or not name):
        raise ValueError(f"{label}.name must be a non-empty string")
    return ActuatorSpec(joint=joint, name=name, ctrlrange=_range(data.get("ctrlrange"), f"{label}.ctrlrange"))


def _range(value: Any, label: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2 or not all(isinstance(v, (int, float)) for v in value):
        raise ValueError(f"{label} must be a two-number list")
    lower, upper = float(value[0]), float(value[1])
    if lower >= upper:
        raise ValueError(f"{label} must have lower < upper")
    return lower, upper


def _keyframes(value: Any) -> dict[str, dict[str, float | list[float]]]:
    data = _mapping(value, "keyframes")
    if tuple(data) != EXPECTED_KEYFRAMES:
        raise ValueError(f"keyframes must be ordered as: {', '.join(EXPECTED_KEYFRAMES)}")
    validated: dict[str, dict[str, float | list[float]]] = {}
    for name, pose_value in data.items():
        pose = _mapping(pose_value, f"keyframes.{name}")
        if set(pose) != {"spine", "finger", "arm_left", "arm_right"}:
            raise ValueError(f"keyframes.{name} must define spine, finger, arm_left, arm_right")
        validated[name] = {
            "spine": _number(pose["spine"], f"keyframes.{name}.spine"),
            "finger": _number(pose["finger"], f"keyframes.{name}.finger"),
            "arm_left": _arm(pose["arm_left"], f"keyframes.{name}.arm_left"),
            "arm_right": _arm(pose["arm_right"], f"keyframes.{name}.arm_right"),
        }
    return validated


def _number(value: Any, label: str) -> float:
    if not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    return float(value)


def _arm(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 7:
        raise ValueError(f"{label} must contain seven joint values")
    return [_number(item, label) for item in value]
