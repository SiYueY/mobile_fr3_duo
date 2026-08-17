"""Dual-arm joint-space PD with gravity compensation (position variant)."""

from __future__ import annotations

import mujoco
import numpy as np


class DualArmJointController:
    """PD controller for the 14 FR3 joints, holding a target qpos."""

    def __init__(self, model: mujoco.MjModel, kp: float = 1200.0, kd: float = 120.0):
        self.model = model
        self.kp = kp
        self.kd = kd
        self.target = np.zeros(model.nq)
        self._joints = []
        for side in ("left", "right"):
            for i in range(1, 8):
                name = f"{side}_fr3v2_1_joint{i}"
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                self._joints.append((jid, model.jnt_qposadr[jid]))

    def set_target_from_keyframe(self, keyframe: int = 0) -> None:
        self.target[:] = self.model.key_qpos[keyframe]

    def apply(self, data: mujoco.MjData, dt: float) -> np.ndarray:
        """Returns the commanded joint torques (motor variant)."""
        torque = np.zeros(self.model.nu)
        mujoco.mj_forward(self.model, data)
        # Compensate gravity for every actuated joint (including the spine),
        # then add arm-space PD.  A keyframe only stores one static torque
        # vector; recomputing qfrc_bias here remains valid after the base
        # settles onto the ground or the arm moves slightly.
        for aid in range(self.model.nu):
            jid = self.model.actuator_trnid[aid, 0]
            if jid >= 0:
                torque[aid] = data.qfrc_bias[self.model.jnt_dofadr[jid]]
        for jid, adr in self._joints:
            dof = self.model.jnt_dofadr[jid]
            err = self.target[adr] - data.qpos[adr]
            err_dot = -data.qvel[dof]
            joint_torque = (
                self.kp * err + self.kd * err_dot + data.qfrc_bias[dof]
            )
            aid = next(
                i
                for i in range(self.model.nu)
                if self.model.actuator_trnid[i, 0] == jid
            )
            torque[aid] = joint_torque
        np.clip(
            torque,
            self.model.actuator_ctrlrange[:, 0],
            self.model.actuator_ctrlrange[:, 1],
            out=torque,
        )
        return torque


def demo(duration_s: float = 5.0) -> None:
    import pathlib

    root = pathlib.Path(__file__).resolve().parent.parent
    model = mujoco.MjModel.from_xml_path(str(root / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    controller = DualArmJointController(model)
    controller.set_target_from_keyframe(0)
    max_err = 0.0
    steps = int(duration_s / model.opt.timestep)
    for _ in range(steps):
        data.ctrl[:] = controller.apply(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        for _, adr in controller._joints:
            max_err = max(max_err, abs(controller.target[adr] - data.qpos[adr]))
    print(f"max joint tracking error over {duration_s}s: {max_err:.3f} rad")


def viewer_demo() -> None:
    """Launch an interactive viewer with home pose actively held."""
    import pathlib
    import time
    import mujoco.viewer

    root = pathlib.Path(__file__).resolve().parent.parent
    model = mujoco.MjModel.from_xml_path(str(root / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    controller = DualArmJointController(model)
    controller.set_target_from_keyframe(0)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            data.ctrl[:] = controller.apply(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", action="store_true", help="launch an interactive viewer")
    args = parser.parse_args()
    if args.viewer:
        viewer_demo()
    else:
        demo()
