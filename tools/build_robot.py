"""Build the one formal Mobile FR3 Duo model and optional temporary variants."""

from __future__ import annotations

import argparse
from pathlib import Path
from types import SimpleNamespace

from build_model import REPO_ROOT, ModelBuilder, _build_scene
from format_xml import format_element
from model_builder.composition import load_robot_config

TEMP_VARIANTS = {
    "position": ("mobile_fr3_duo_position.xml", dict(position=True, sensors=False, reduced=False, planar=False)),
    "reduced": ("mobile_fr3_duo_reduced.xml", dict(position=False, sensors=False, reduced=True, planar=False)),
    "planar": ("mobile_fr3_duo_planar_debug.xml", dict(position=False, sensors=False, reduced=False, planar=True)),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--variant",
        choices=tuple(TEMP_VARIANTS),
        action="append",
        help="build a non-release variant under build/ (repeatable)",
    )
    args = parser.parse_args()
    load_robot_config(REPO_ROOT / "config/robot/mobile_fr3_duo.yaml")
    models_dir = REPO_ROOT / "models"

    # The committed robot is the full digital twin: motor actuators and every
    # supported sensor instance.  It is intentionally the only top-level
    # robot asset in models/.
    formal_values = dict(position=False, sensors=True, reduced=False, planar=False)
    formal_source = ModelBuilder(SimpleNamespace(**formal_values, spawn_z=0.002)).build()
    formal_path = models_dir / "mobile_fr3_duo.xml"
    compiler = formal_source.find("compiler")
    assert compiler is not None
    compiler.set("meshdir", ".")
    formal_path.write_text(format_element(formal_source), encoding="utf-8")
    (models_dir / "scene.xml").write_text(
        format_element(_build_scene("mobile_fr3_duo.xml")), encoding="utf-8"
    )
    print(f"built {formal_path.relative_to(REPO_ROOT)}")

    if not args.variant:
        return 0

    out_dir = REPO_ROOT / "build"
    for variant in args.variant:
        filename, values = TEMP_VARIANTS[variant]
        opts = SimpleNamespace(**values, spawn_z=0.0 if values["planar"] else 0.002)
        source = ModelBuilder(opts).build()
        path = out_dir / filename
        compiler = source.find("compiler")
        assert compiler is not None
        compiler.set("meshdir", "../models")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(format_element(source), encoding="utf-8")
        scene = path.with_name(f"scene_{variant}.xml")
        scene.write_text(format_element(_build_scene(filename)), encoding="utf-8")
        print(f"built {path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
