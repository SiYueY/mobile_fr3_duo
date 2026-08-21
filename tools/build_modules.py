"""Generate standalone runtime MJCF modules from frozen official URDF inputs."""

from __future__ import annotations

import argparse
from types import SimpleNamespace

from build_model import REPO_ROOT, ModelBuilder
from model_builder.canonical import CanonicalModel
from model_builder.modules import build_modules


def _source_model(*, sensors: bool = False):
    opts = SimpleNamespace(position=False, sensors=sensors, reduced=False, planar=False, spawn_z=0.002)
    return ModelBuilder(opts).build()


def main() -> int:
    argparse.ArgumentParser(description=__doc__).parse_args()
    builder = ModelBuilder(SimpleNamespace(position=False, sensors=False, reduced=False, planar=False, spawn_z=0.002))
    canonical = CanonicalModel.from_urdf(builder.urdf, "franka_description@2.8.1")
    build_modules(
        REPO_ROOT,
        {
            "base": builder.build(),
            "sensors": _source_model(sensors=True),
        },
        canonical,
    )
    print("built runtime modules under models/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
