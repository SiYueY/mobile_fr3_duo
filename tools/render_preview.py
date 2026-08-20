"""Render fixed preview images for the model variants."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
from mujoco import renderer as mjr
from mujoco.egl import GLContext
from pngio import write_png

REPO_ROOT = Path(__file__).resolve().parent.parent


def _render(model: mujoco.MjModel, data: mujoco.MjData, width=1280, height=720):
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height
    renderer = mjr.Renderer(model, height, width)
    renderer.update_scene(data, camera=-1)
    image = renderer.render()
    renderer.close()
    return image[:, :, :3]


def render_preview(out: Path, xml: str, keyframe: int = 0) -> None:
    ctx = GLContext(1280, 720)
    try:
        ctx.make_current()
        model = mujoco.MjModel.from_xml_path(
            str(REPO_ROOT / "models" / xml)
        )
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, keyframe)
        mujoco.mj_forward(model, data)
        model.vis.global_.offwidth = 1280
        model.vis.global_.offheight = 720
        renderer = mjr.Renderer(model, 720, 1280)
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "preview")
        renderer.update_scene(data, camera=cam_id if cam_id >= 0 else -1)
        image = renderer.render()
        renderer.close()
        write_png(out, image)
        print(f"wrote {out}")
    finally:
        import contextlib

        with contextlib.suppress(Exception):
            ctx.free()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=REPO_ROOT)
    args = ap.parse_args()
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    render_preview(out / "mobile_fr3_duo.png", "scene.xml", 0)
    render_preview(out / "mobile_fr3_duo_manipulation.png", "scene.xml", 2)
    render_preview(out / "mobile_fr3_duo_with_sensors.png", "scene_with_sensors.xml", 0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
