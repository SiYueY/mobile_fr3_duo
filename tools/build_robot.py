"""Compose Mobile FR3 Duo runtime modules and flattened release models."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from types import SimpleNamespace

from build_model import REPO_ROOT, ModelBuilder, _build_scene
from format_xml import format_element
from model_builder.composition import compose, flatten, load_robot_config

VARIANTS = {
    "base": ("mobile_fr3_duo.xml", dict(position=False, sensors=False, reduced=False, planar=False)),
    "position": ("mobile_fr3_duo_position.xml", dict(position=True, sensors=False, reduced=False, planar=False)),
    "sensors": ("mobile_fr3_duo_with_sensors.xml", dict(position=False, sensors=True, reduced=False, planar=False)),
    "reduced": ("mobile_fr3_duo_reduced.xml", dict(position=False, sensors=False, reduced=True, planar=False)),
    "planar": ("mobile_fr3_duo_planar_debug.xml", dict(position=False, sensors=False, reduced=False, planar=True)),
}


def _ensure_scene_model_alias(out_dir: Path) -> None:
    """Create the portable relative-path alias needed by nested MJCF includes.

    MuJoCo 3.9 resolves an asset-level ``<model file>`` inside an included
    robot relative to the command-line scene path a second time.  Thus
    ``simulate ./models/scene.xml`` asks for ``models/models/franka_tmr``.
    This symlink supplies that path without duplicating a module or its assets.
    """
    target = out_dir / "franka_tmr"
    alias = out_dir / "models" / "franka_tmr"
    alias.parent.mkdir(exist_ok=True)
    if alias.is_symlink():
        if alias.resolve() == target.resolve():
            return
        raise ValueError(f"unexpected scene compatibility alias: {alias} -> {os.readlink(alias)}")
    if alias.exists():
        raise ValueError(f"scene compatibility alias path already exists: {alias}")
    alias.symlink_to("../franka_tmr", target_is_directory=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all", action="store_true", help="build every release variant")
    parser.add_argument("--variant", choices=tuple(VARIANTS), action="append")
    args = parser.parse_args()
    load_robot_config(REPO_ROOT / "config/robots/mobile_fr3_duo.yaml")
    selected = args.variant or tuple(VARIANTS)
    out_dir = REPO_ROOT / "models"
    _ensure_scene_model_alias(out_dir)
    for variant in selected:
        filename, values = VARIANTS[variant]
        opts = SimpleNamespace(**values, spawn_z=0.0 if values["planar"] else 0.002)
        source = ModelBuilder(opts).build()
        if values["sensors"]:
            base_module = "franka_tmr/franka_tmr_sensors.xml"
        elif values["planar"]:
            base_module = "franka_tmr/franka_tmr_planar.xml"
        elif values["reduced"]:
            base_module = "franka_tmr/franka_tmr_reduced.xml"
        else:
            base_module = "franka_tmr/franka_tmr.xml"
        path = out_dir / filename
        compose(source, path, base_module=base_module)
        flatten(path, path.with_name(f"{path.stem}_flattened.xml"))
        print(f"built {path.relative_to(REPO_ROOT)}")
    if args.all:
        for robot, scene in (
            ("mobile_fr3_duo.xml", "scene.xml"),
            ("mobile_fr3_duo_with_sensors.xml", "scene_with_sensors.xml"),
            ("mobile_fr3_duo_position.xml", "scene_position.xml"),
        ):
            (out_dir / scene).write_text(
                format_element(_build_scene(robot)), encoding="utf-8"
            )
            flat_robot = f"{Path(robot).stem}_flattened.xml"
            flat_scene = f"{Path(scene).stem}_flattened.xml"
            (out_dir / flat_scene).write_text(
                format_element(_build_scene(flat_robot)), encoding="utf-8"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
