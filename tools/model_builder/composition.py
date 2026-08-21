"""Compose runtime MJCF modules into the formal robot model."""

from __future__ import annotations

from pathlib import Path

import yaml

KNOWN_MODULES = {
    "franka_tmr", "franka_spine", "franka_head", "franka_fr3", "franka_hand",
    "imu", "d455", "nanoscan3", "zed_mini",
}
MOUNT_TARGETS = {
    "imu_mounting_point",
    "front_mounting_point",
    "rear_mounting_point",
    "left_mounting_point",
    "right_mounting_point",
    "lidar_front_mounting_point",
    "lidar_rear_mounting_point",
    "head_camera_mounting_point",
}


def load_robot_config(path: Path) -> dict:
    """Validate the composition contract before generating any XML."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("robot"), dict):
        raise ValueError(f"invalid robot configuration: {path}")
    selected = set(data["robot"].values())
    for arm in data.get("arms", {}).values():
        if not isinstance(arm, dict):
            raise ValueError("each arm configuration must be a mapping")
        selected.update((arm.get("model"), arm.get("end_effector")))
    prefixes = [arm.get("prefix") for arm in data.get("arms", {}).values()]
    if len(prefixes) != len(set(prefixes)) or any(not isinstance(prefix, str) or not prefix for prefix in prefixes):
        raise ValueError("arm prefixes must be unique non-empty strings")
    for sensor in data.get("sensors", []):
        if not isinstance(sensor, dict):
            raise ValueError("each sensor configuration must be a mapping")
        selected.add(sensor.get("model"))
        mounts = sensor.get("mounts", [sensor.get("mount")])
        if not all(isinstance(mount, str) and mount in MOUNT_TARGETS for mount in mounts):
            raise ValueError(f"invalid sensor mount in {sensor}")
    unknown = {item for item in selected if not isinstance(item, str) or item not in KNOWN_MODULES}
    if unknown:
        raise ValueError(f"unknown runtime modules: {', '.join(sorted(map(str, unknown)))}")
    return data
