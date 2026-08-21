"""nanoScan3 LiDAR raycast worker using mujoco.mj_multiRay."""

from __future__ import annotations

import threading
import time
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent


class LidarWorker:
    """Raycasts the nanoScan3 scan pattern from snapshots at 50 Hz."""

    def __init__(self, model: mujoco.MjModel, rate_hz: float = 50.0):
        self.model = model
        self.rate = rate_hz
        # nominal nanoScan3 config (document section 20)
        self.angle_min = -np.pi / 4
        self.angle_max = 3 * np.pi / 4
        self.range_min = 0.05
        self.range_max = 10.0
        self.beams = 500
        self._latest: dict | None = None
        self._stop = threading.Event()

    def _scan(self, site_name: str) -> np.ndarray:
        model = self.model
        data = self._data
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid < 0:
            return np.zeros(0)
        # ray origins/directions in the scan frame
        angles = np.linspace(self.angle_min, self.angle_max, self.beams)
        dir_local = np.stack(
            [np.cos(angles), np.zeros_like(angles), np.sin(angles)], axis=1
        )
        geoms = np.full(self.beams, -1, dtype=np.int32)
        dist = np.full(self.beams, self.range_max, dtype=np.float64)
        pos = data.site_xpos[sid].copy()
        mat = data.site_xmat[sid].reshape(3, 3)
        direction = (mat @ dir_local.T).T
        bodyexclude = -1
        mujoco.mj_multiRay(
            model,
            data,
            pos,
            direction.flatten(),
            None,
            True,
            bodyexclude,
            geoms,
            dist,
            None,
            self.beams,
            self.range_max,
        )
        return dist

    def start(self) -> None:
        self._data = mujoco.MjData(self.model)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        period = 1.0 / self.rate
        next_t = time.time()
        while not self._stop.is_set():
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
                continue
            next_t += period
            front = self._scan("lidar_front_scan_frame")
            rear = self._scan("lidar_rear_scan_frame")
            self._latest = {
                "time": float(self._data.time),
                "front": front,
                "rear": rear,
            }

    def stop(self) -> None:
        self._stop.set()


def demo(duration_s: float = 1.5) -> None:
    model = mujoco.MjModel.from_xml_path(
        str(REPO_ROOT / "models/scene.xml")
    )
    worker = LidarWorker(model)
    worker.start()
    t0 = time.time()
    scans = 0
    n = 0
    while time.time() - t0 < duration_s:
        if worker._latest is not None:
            scans += 1
            n = len(worker._latest["front"])
        time.sleep(0.02)
    worker.stop()
    print(f"performed {scans} lidar scans of {n} beams each")


if __name__ == "__main__":
    demo()
