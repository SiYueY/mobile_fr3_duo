"""Build all independent modules plus the one formal robot and scene."""

from __future__ import annotations

from model_builder.builder import REPO_ROOT, ModelBuilder, _build_scene
from model_builder.canonical import CanonicalModel
from model_builder.module import build_modules
from utils.xml import format_element


def main() -> int:
    builder = ModelBuilder()
    source = builder.build()
    canonical = CanonicalModel.from_urdf(builder.urdf, "franka_description@2.8.1")
    build_modules(REPO_ROOT, {"base": source, "sensors": source}, canonical)
    compiler = source.find("compiler")
    assert compiler is not None
    compiler.set("meshdir", ".")
    models = REPO_ROOT / "models"
    (models / "mobile_fr3_duo.xml").write_text(format_element(source), encoding="utf-8")
    (models / "scene.xml").write_text(format_element(_build_scene("mobile_fr3_duo.xml")), encoding="utf-8")
    print("built models/mobile_fr3_duo.xml and models/scene.xml")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
