"""Offscreen camera worker rendering with an independent mjData."""

from __future__ import annotations

import threading
import time
from pathlib import Path

import mujoco
import numpy as np
from mujoco import renderer as mjr
from mujoco.egl import GLContext

REPO_ROOT = Path(__file__).resolve().parent.parent


class CameraWorker:
    """Renders one camera at a fixed rate on its own mjData + GL context."""

    def __init__(
        self,
        model: mujoco.MjModel,
        camera_name: str,
        rate_hz: float,
        width: int = 640,
        height: int = 480,
    ):
        self.model = model
        self.camera_name = camera_name
        self.rate = rate_hz
        self.width = width
        self.height = height
        self.data = mujoco.MjData(model)
        self._ctx = GLContext(width, height)
        self._latest: dict | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        self._ctx.make_current()
        model = self.model
        model.vis.global_.offwidth = self.width
        model.vis.global_.offheight = self.height
        renderer = mjr.Renderer(model, self.height, self.width)
        cam_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name
        )
        period = 1.0 / self.rate
        next_t = time.time()
        while not self._stop.is_set():
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
                continue
            next_t += period
            renderer.update_scene(self.data, camera=cam_id)
            rgb = renderer.render()
            self._latest = {
                "time": float(self.data.time),
                "rgb": np.ascontiguousarray(rgb[:, :, :3]),
            }
        renderer.close()

    def stop(self) -> None:
        self._stop.set()


def demo(duration_s: float = 1.5, rate_hz: float = 30.0) -> None:
    model = mujoco.MjModel.from_xml_path(
        str(REPO_ROOT / "scene_with_sensors.xml")
    )
    worker = CameraWorker(model, "camera_front_color", rate_hz)
    worker.start()
    t0 = time.time()
    frames = 0
    while time.time() - t0 < duration_s:
        if worker._latest is not None:
            frames += 1
        time.sleep(0.02)
    worker.stop()
    print(f"rendered {frames} frames at ~{rate_hz} Hz")


if __name__ == "__main__":
    demo()
