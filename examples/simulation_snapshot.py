"""SimulationSnapshot and the multi-threaded runtime skeleton.

Design-document section 27:
  * Physics thread owns the primary mjData at 1 kHz and publishes snapshots;
  * Camera workers use independent mjData + render contexts (30 Hz D455,
    30/60 Hz ZED, 30-60 Hz viewer);
  * LiDAR worker raycasts from snapshots with mj_multiRay at 50 Hz;
  * the primary mjData is never written by workers.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent


@dataclass
class SimulationSnapshot:
    """Immutable copy of the physics state for downstream workers."""

    simulation_time: float
    qpos: np.ndarray
    qvel: np.ndarray
    actuator_force: np.ndarray
    body_pose: dict[str, tuple[np.ndarray, np.ndarray]]
    mocap_state: np.ndarray = field(default_factory=lambda: np.zeros(0))
    sensor_reference_state: dict[str, float] = field(default_factory=dict)

    @classmethod
    def from_data(
        cls, model: mujoco.MjModel, data: mujoco.MjData
    ) -> SimulationSnapshot:
        body_pose = {}
        for i in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name:
                body_pose[name] = (data.xpos[i].copy(), data.xquat[i].copy())
        return cls(
            simulation_time=float(data.time),
            qpos=data.qpos.copy(),
            qvel=data.qvel.copy(),
            actuator_force=data.qfrc_actuator.copy(),
            body_pose=body_pose,
        )


class PhysicsWorker:
    """Owns the primary mjData and publishes snapshots at 1 kHz."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.data = mujoco.MjData(model)
        self._lock = threading.Lock()
        self._latest: SimulationSnapshot | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                mujoco.mj_step(self.model, self.data)
                self._latest = SimulationSnapshot.from_data(self.model, self.data)

    def snapshot(self) -> SimulationSnapshot | None:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()


def demo(duration_s: float = 2.0) -> None:
    model = mujoco.MjModel.from_xml_path(
        str(REPO_ROOT / "models/scene.xml")
    )
    physics = PhysicsWorker(model)
    physics.start()
    t0 = time.time()
    last = None
    while time.time() - t0 < duration_s:
        snap = physics.snapshot()
        if snap is not None and snap is not last:
            last = snap
            print(
                f"t={snap.simulation_time:7.3f} base="
                f"{snap.body_pose['base_link'][0][:2].round(3)}"
            )
        time.sleep(0.05)
    physics.stop()


if __name__ == "__main__":
    demo()
