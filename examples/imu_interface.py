"""OLV-IMU01 nominal interface: read gyro/accelerometer/orientation."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent


class ImuInterface:
    """Reads the IMU sensors from the formal complete robot model."""

    def __init__(self, model: mujoco.MjModel, rate_hz: float = 200.0):
        self.model = model
        self.rate_hz = rate_hz
        self._sensor_ids = {}
        for name in (
            "imu_angular_velocity",
            "imu_linear_acceleration",
            "imu_orientation",
        ):
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            assert sid >= 0, f"missing sensor {name}"
            self._sensor_ids[name] = sid

    def read(self, data: mujoco.MjData) -> dict[str, np.ndarray]:
        out = {}
        for name, sid in self._sensor_ids.items():
            adr = self.model.sensor_adr[sid]
            dim = self.model.sensor_dim[sid]
            out[name] = data.sensordata[adr : adr + dim].copy()
        return out


def demo(duration_s: float = 0.5) -> None:
    model = mujoco.MjModel.from_xml_path(
        str(REPO_ROOT / "models/mobile_fr3_duo.xml")
    )
    data = mujoco.MjData(model)
    imu = ImuInterface(model)
    samples = 0
    for _ in range(int(duration_s / model.opt.timestep)):
        mujoco.mj_step(model, data)
        samples += 1
    reading = imu.read(data)
    print(
        f"IMU ({samples} steps): gyro={reading['imu_angular_velocity'].round(3)}, "
        f"accel={reading['imu_linear_acceleration'].round(2)}, "
        f"orientation={reading['imu_orientation'].round(3)}"
    )


if __name__ == "__main__":
    demo()
