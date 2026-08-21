"""Emit the one formal Mobile FR3 Duo MJCF from frozen official inputs."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml
from utils.xml import format_element
from model_builder import BuildContext, contacts, dynamics, geometry
from model_builder.config import BuilderConfig
from model_builder.config import load as load_builder_config
from model_builder.urdf import UrdfModel, children
from scipy.spatial.transform import Rotation
from utils.urdf import fmt, fmt_vec, origin_attrib

REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATED = REPO_ROOT / "source" / "generated"

CANONICAL_URDF = GENERATED / "mobile_fr3_duo.urdf"
COLLISION_EXCLUSIONS = GENERATED / "collision_exclusions.yaml"

SPAWN_CLEARANCE = 0.002  # meters, simulation-only

ARM_PREFIXES = ["left_fr3v2_1", "right_fr3v2_1"]
HAND_PREFIXES = ["left", "right"]

def _el(tag: str, **attrs) -> ET.Element:
    elem = ET.Element(tag)
    for k, v in attrs.items():
        if v is not None:
            elem.set(k, str(v))
    return elem


def _geom_origin(elem) -> tuple[str, str]:
    xyz, quat = origin_attrib(elem)
    return fmt_vec(xyz), fmt_vec(quat)


class ModelBuilder:
    def __init__(self):
        self.urdf = UrdfModel(CANONICAL_URDF)
        self.config: BuilderConfig = load_builder_config(REPO_ROOT / "config")
        self.simulation = yaml.safe_load((REPO_ROOT / "config" / "simulation.yaml").read_text(encoding="utf-8"))
        self.context = BuildContext(
            self.urdf,
            COLLISION_EXCLUSIONS,
            geometry.load_visual_conversion(),
            geometry.load_sensor_appearances(),
        )

    # ------------------------------------------------------------------
    def build(self) -> ET.Element:
        mujoco_root = _el(
            "mujoco",
            model="mobile_fr3_duo",
        )
        mujoco_root.append(
            _el(
                "compiler",
                angle="radian",
                meshdir="models",
                autolimits="true",
                inertiafromgeom="false",
                fusestatic="false",
            )
        )
        mujoco_root.append(
            _el(
                "option",
                timestep=self.simulation["timestep"],
                integrator=self.simulation["integrator"],
                solver=self.simulation["solver"],
                iterations=self.simulation["iterations"],
                ls_iterations=self.simulation["ls_iterations"],
                gravity=fmt_vec(np.asarray(self.simulation["gravity"])),
            )
        )
        mujoco_root.append(geometry.defaults())
        mujoco_root.append(geometry.assets(self.context))

        spawn_z = SPAWN_CLEARANCE
        worldbody = _el("worldbody")
        worldbody.append(self._emit_body(self.urdf.root_link, spawn_z, is_root=True))
        self._attach_sensor_entities(worldbody)
        mujoco_root.append(worldbody)

        mujoco_root.append(contacts.contacts(self.context))
        mujoco_root.append(contacts.equalities(self.context))
        mujoco_root.append(self._actuators())
        mujoco_root.append(self._sensors())
        mujoco_root.append(self._keyframes(spawn_z))
        self._fill_gravity_comp_ctrl(mujoco_root)
        return mujoco_root

    # ------------------------------------------------------------------
    def _emit_body(self, name: str, spawn_z: float, is_root: bool = False) -> ET.Element:
        link = self.urdf.links[name]
        joint = self.urdf.child_to_joint.get(name)
        if is_root or joint is None:
            pos = f"0 0 {fmt(spawn_z)}"
            quat = "1 0 0 0"
        else:
            pos, quat = _geom_origin(joint)
        body = _el("body", name=name, pos=pos, quat=quat)

        if is_root:
            body.append(_el("freejoint", name="base_freejoint"))
        movable_joint = joint is not None and joint.get("type") != "fixed"
        if movable_joint:
            # In URDF, the joint origin is fixed in the parent frame and the
            # coordinate rotation follows it: T_parent_child = T_origin @
            # R_axis(q).  Placing the joint on this child body preserves that
            # order.  A parent-origin wrapper would instead rotate the origin
            # translation, visibly pulling the two sides of each joint apart.
            body.append(dynamics.joint(joint))

        inertial = link.find("inertial")
        if inertial is not None and dynamics.inertial_mass(inertial) not in (None, 0):
            body.append(dynamics.inertial(inertial))

        for geom in link.findall("visual"):
            for g in geometry.visual(geom, self.context.visual_conversion):
                body.append(g)
        for geom in link.findall("collision"):
            g = geometry.collision(geom, name)
            if g is not None:
                body.append(g)

        # Mounting-point and TCP links also expose same-named sites.
        if (
            name.endswith("_mounting_point")
            or name.endswith("_hand_tcp")
            or name == "franka_spine_mounting_point"
        ):
            body.append(_el("site", name=name, size="0.002"))
        if is_root:
            body.append(_el("site", name="base_sensor_frame", size="0.002"))

        # Children in URDF joint order.
        for child in children(self.urdf, name):
            body.append(self._emit_body(child, spawn_z))
        return body

    # ------------------------------------------------------------------
    def _actuators(self) -> ET.Element:
        actuator = _el("actuator")
        actuator.append(
            _el(
                "motor",
                name=self.config.spine.name,
                joint=self.config.spine.joint,
                gear="1",
                ctrllimited="true",
                ctrlrange=f"{fmt(self.config.spine.ctrlrange[0])} {fmt(self.config.spine.ctrlrange[1])}",
            )
        )
        for prefix in ARM_PREFIXES:
            for i in range(1, 8):
                jname = f"{prefix}_joint{i}"
                j = self.urdf.joints.get(jname)
                if j is None:
                    continue
                limit = j.find("limit")
                effort = float(limit.get("effort")) if limit is not None and limit.get("effort") else 87.0
                dyn = j.find("dynamics")
                armature = 0.0
                if dyn is not None and dyn.get("motor_inertia") and dyn.get("gear_ratio"):
                    armature = float(dyn.get("motor_inertia")) * float(dyn.get("gear_ratio")) ** 2
                attrs = {
                    "name": f"{jname}_motor",
                    "joint": jname,
                    "gear": "1",
                    "ctrllimited": "true",
                    "ctrlrange": f"{fmt(-effort)} {fmt(effort)}",
                    "forcerange": f"{fmt(-effort)} {fmt(effort)}",
                }
                if armature > 0:
                    attrs["armature"] = fmt(armature)
                actuator.append(_el("motor", **attrs))
        for side in HAND_PREFIXES:
            jname = f"{side}_fr3v2_1_finger_joint1"
            if jname in self.urdf.joints:
                actuator.append(
                    _el(
                        "motor",
                        name=f"{side}_fr3v2_1_finger_motor",
                        joint=jname,
                        gear="1",
                        ctrllimited="true",
                        ctrlrange=f"{fmt(self.config.hand_ctrlrange[0])} {fmt(self.config.hand_ctrlrange[1])}",
                        forcerange=f"{fmt(self.config.hand_forcerange[0])} {fmt(self.config.hand_forcerange[1])}",
                    )
                )
        return actuator

    # ------------------------------------------------------------------
    def _sensors(self) -> ET.Element:
        sensor = _el("sensor")
        dof_order = self._active_joints_in_order()
        for jname in dof_order:
            sensor.append(_el("jointpos", name=f"{jname}_pos", joint=jname))
            sensor.append(_el("jointvel", name=f"{jname}_vel", joint=jname))
        for jname in self._actuator_joints():
            sensor.append(_el("jointactuatorfrc", name=f"{jname}_actuator_force", joint=jname))
        # Base state.
        sensor.append(_el("framepos", name="base_pos", objtype="body", objname="base_link"))
        sensor.append(_el("framequat", name="base_quat", objtype="body", objname="base_link"))
        sensor.append(_el("velocimeter", name="base_linear_velocity", site="base_sensor_frame"))
        sensor.append(_el("gyro", name="base_angular_velocity", site="base_sensor_frame"))
        sensor.append(_el("gyro", name="imu_angular_velocity", site="imu_sensor_frame"))
        sensor.append(_el("accelerometer", name="imu_linear_acceleration", site="imu_sensor_frame"))
        sensor.append(_el("framequat", name="imu_orientation", objtype="site", objname="imu_sensor_frame"))
        return sensor

    def _active_joints_in_order(self) -> list[str]:
        """Non-fixed joints in the same order MuJoCo assigns DOFs."""
        order: list[str] = []

        def walk(name: str) -> None:
            for j in self.urdf.joints.values():
                if j.find("parent").get("link") != name:
                    continue
                if j.get("type") != "fixed":
                    order.append(j.get("name"))
                walk(j.find("child").get("link"))

        walk(self.urdf.root_link)
        return order

    def _actuator_joints(self) -> list[str]:
        out = []
        out.append(self.config.spine.joint)
        out += [f"{p}_joint{i}" for p in ARM_PREFIXES for i in range(1, 8)]
        out += [f"{s}_fr3v2_1_finger_joint1" for s in HAND_PREFIXES]
        return [j for j in out if j in self.urdf.joints]

    # ------------------------------------------------------------------
    def _keyframes(self, spawn_z: float) -> ET.Element:
        keyframe = _el("keyframe")
        dof_order = self._active_joints_in_order()
        qpos_prefix = [0.0, 0.0, spawn_z, 1.0, 0.0, 0.0, 0.0]
        for name, pose in self.config.keyframes.items():
            qpos = list(qpos_prefix)
            for jname in dof_order:
                if jname in ("tmrv0_2_joint_0", "tmrv0_2_joint_2") or jname in ("tmrv0_2_joint_1", "tmrv0_2_joint_3"):
                    qpos.append(0.0)
                elif jname == "franka_spine_vertical_joint":
                    qpos.append(pose["spine"])
                elif jname.endswith("finger_joint1") or jname.endswith("finger_joint2"):
                    qpos.append(pose["finger"])
                elif "_joint" in jname:
                    try:
                        idx = int(jname.rsplit("joint", 1)[1]) - 1
                    except ValueError:
                        qpos.append(0.0)
                        continue
                    if jname.startswith("left_"):
                        arm_key = "arm_left"
                    elif jname.startswith("right_"):
                        arm_key = "arm_right"
                    else:
                        arm_key = "arm"
                    qpos.append(pose[arm_key][idx])
                else:
                    qpos.append(0.0)
            ctrl = self._keyframe_ctrl(qpos, dof_order)
            kf = _el("key", name=name, qpos=" ".join(fmt(v) for v in qpos))
            kf.set("ctrl", " ".join(fmt(v) for v in ctrl))
            keyframe.append(kf)
        return keyframe

    def _keyframe_ctrl(self, qpos: list[float], dof_order: list[str]) -> list[float]:
        return [0.0] * len(self._actuator_joints())

    def _fill_gravity_comp_ctrl(self, mujoco_root: ET.Element) -> None:
        """Set motor keyframe ctrl to gravity-compensating torques.

        With ctrl matching the keyframe qpos, loading a keyframe in simulate
        holds the pose instead of letting the arms collapse under gravity.
        """
        probe = _write_probe(mujoco_root)
        try:
            model = mujoco.MjModel.from_xml_path(str(probe))
            data = mujoco.MjData(model)
            keyframe = mujoco_root.find("keyframe")
            for k, kf in enumerate(keyframe.findall("key")):
                data.qpos[:] = model.key_qpos[k]
                mujoco.mj_forward(model, data)
                ctrl = []
                for i in range(model.nu):
                    jid = model.actuator_trnid[i, 0]
                    dof = model.jnt_dofadr[jid]
                    torque = float(data.qfrc_bias[dof])
                    lo, hi = model.actuator_ctrlrange[i]
                    ctrl.append(min(max(torque, lo), hi))
                kf.set("ctrl", " ".join(fmt(v) for v in ctrl))
        finally:
            probe.unlink(missing_ok=True)

    # ------------------------------------------------------------------
    def _attach_sensor_entities(self, worldbody: ET.Element) -> None:
        """Append sensor device bodies to the official mounting-point bodies."""
        self._attach_imu(worldbody)
        for position in ("front", "rear", "left", "right"):
            self._attach_d455(worldbody, position)
        for position in ("front", "rear"):
            self._attach_nanoscan3(worldbody, position)
        self._attach_zed(worldbody)

    def _find_body(self, body: ET.Element, name: str) -> ET.Element | None:
        if body.get("name") == name:
            return body
        for child in body.findall("body"):
            found = self._find_body(child, name)
            if found is not None:
                return found
        return None

    def _attach_imu(self, worldbody: ET.Element) -> None:
        parent = self._find_body(worldbody, "imu_mounting_point")
        if parent is None:
            return
        body = _el("body", name="imu_link")
        body.append(
            _el("geom", **{"class": "visual"}, type="box", size="0.015 0.015 0.005")
        )
        body.append(_el("site", name="imu_sensor_frame", size="0.002"))
        parent.append(body)

    def _attach_d455(self, worldbody: ET.Element, position: str) -> None:
        parent = self._find_body(worldbody, f"{position}_mounting_point")
        if parent is None:
            return
        prefix = f"camera_{position}"
        body = _el(
            "body",
            name=f"{prefix}_link",
            pos="0.01115 0.0475 0.0145",
        )
        body.append(
            _el(
                "geom",
                **{"class": "visual"},
                type="mesh",
                mesh="realsense_d455",
                material=geometry.sensor_material_name("realsense_d455", self.context.sensor_appearances),
                pos="0.00465 -0.0475 0",
                quat=_fmt_quat((math.pi / 2, 0, math.pi / 2)),
            )
        )
        optical_rpy = (-math.pi / 2, 0, -math.pi / 2)
        for frame, y_off in (
            ("depth", 0.0),
            ("infra1", 0.0),
            ("infra2", -0.095),
            ("color", -0.059),
        ):
            body.append(
                _el(
                    "site",
                    name=f"{prefix}_{frame}_frame",
                    size="0.001",
                    pos=f"0 {fmt(y_off)} 0",
                )
            )
            body.append(
                _el(
                    "site",
                    name=f"{prefix}_{frame}_optical_frame",
                    size="0.001",
                    pos=f"0 {fmt(y_off)} 0",
                    quat=_fmt_quat(optical_rpy),
                )
            )
        q_cam = _camera_quat(optical_rpy)
        for frame, y_off in (("color", -0.059), ("depth", 0.0)):
            body.append(
                _el(
                    "camera",
                    name=f"{prefix}_{frame}",
                    # lens at the housing front face so the camera renders
                    # instead of being inside the housing mesh
                    pos=f"0.005 {fmt(y_off)} 0",
                    quat=q_cam,
                    fovy="55",
                    resolution="1280 720",
                )
            )
        parent.append(body)

    def _attach_nanoscan3(self, worldbody: ET.Element, position: str) -> None:
        parent = self._find_body(worldbody, f"lidar_{position}_mounting_point")
        if parent is None:
            return
        prefix = f"lidar_{position}"
        body = _el(
            "body",
            name=prefix,
            pos="-0.0803 0.053 0",
            quat=_fmt_quat((0, 0, -math.pi / 2)),
        )
        body.append(
            _el(
                "geom",
                **{"class": "visual"},
                type="mesh",
                mesh="nanoscan3_visual",
                material=geometry.sensor_material_name("sick_nanoscan3_visual", self.context.sensor_appearances),
                pos="-0.01 -0.08 -0.04",
                quat=_fmt_quat((0, 0, math.pi / 2)),
            )
        )
        body.append(_el("site", name=f"{prefix}_scan_frame", size="0.002"))
        parent.append(body)

    def _attach_zed(self, worldbody: ET.Element) -> None:
        parent = self._find_body(worldbody, "head_camera_mounting_point")
        if parent is None:
            return
        body = _el("body", name="head_zed")
        body.append(
            _el(
                "geom",
                **{"class": "visual"},
                type="mesh",
                mesh="zed_mini",
                material=geometry.sensor_material_name("zed_mini", self.context.sensor_appearances),
            )
        )
        baseline = 0.063
        q_cam = _camera_quat((0, 0, 0))
        for side, sign in (("left", -1.0), ("right", 1.0)):
            x = sign * baseline / 2.0
            body.append(
                _el(
                    "site",
                    name=f"head_zed_{side}_camera_frame",
                    size="0.001",
                    pos=f"{fmt(x)} 0 0",
                )
            )
            body.append(
                _el(
                    "site",
                    name=f"head_zed_{side}_camera_optical_frame",
                    size="0.001",
                    pos=f"{fmt(x)} 0 0",
                )
            )
            body.append(
                _el(
                    "camera",
                    name=f"head_zed_{side}",
                    pos=f"{fmt(x)} 0 0",
                    quat=q_cam,
                    fovy="60",
                    resolution="1280 720",
                )
            )
        body.append(_el("site", name="head_zed_center_frame", size="0.001"))
        parent.append(body)



def _fmt_quat(rpy: tuple[float, float, float]) -> str:
    """URDF rpy (fixed-axis xyz) to MuJoCo quaternion string (w x y z)."""
    r = Rotation.from_euler("xyz", np.asarray(rpy))
    q = r.as_quat()
    return fmt_vec(np.array([q[3], q[0], q[1], q[2]]))


def _camera_quat(rpy: tuple[float, float, float]) -> str:
    """Camera orientation from a ROS optical frame (z forward).

    MuJoCo cameras look along -z, so rotate the optical frame 180 deg about y.
    """
    optical = Rotation.from_euler("xyz", np.asarray(rpy))
    cam = optical * Rotation.from_euler("y", math.pi)
    q = cam.as_quat()
    return fmt_vec(np.array([q[3], q[0], q[1], q[2]]))

def _write_probe(model_xml: ET.Element) -> Path:
    """Write a temporary probe XML at repo root so meshdir resolves."""
    probe = REPO_ROOT / ".build_probe.xml"
    probe.write_text(format_element(model_xml), encoding="utf-8")
    return probe


def compute_spawn_z(model_xml: ET.Element) -> float:
    """Compile with base at z=0 and return the z offset that clears the ground."""
    xml_path = _write_probe(model_xml)
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        min_surface_z = None
        for i in range(model.ngeom):
            if model.geom_group[i] != 3:
                continue
            z = _geom_surface_min_z(model, data, i)
            if min_surface_z is None or z < min_surface_z:
                min_surface_z = z
        if min_surface_z is None:
            return 0.0
        return -min_surface_z + SPAWN_CLEARANCE
    finally:
        xml_path.unlink(missing_ok=True)


def _geom_surface_min_z(model: mujoco.MjModel, data: mujoco.MjData, i: int) -> float:
    """Analytic min world-z of a group-3 geom surface."""
    body_id = model.geom_bodyid[i]
    pos = model.geom_pos[i]
    quat = model.geom_quat[i]
    size = model.geom_size[i]
    gtype = model.geom_type[i]
    r_world = data.xmat[body_id].reshape(3, 3)
    r_geom = _quat_to_mat(quat)
    center = data.xpos[body_id] + r_world @ pos
    r = r_world @ r_geom
    if gtype == mujoco.mjtGeom.mjGEOM_SPHERE:
        return center[2] - size[0]
    if gtype == mujoco.mjtGeom.mjGEOM_CYLINDER:
        radius, half = size[0], size[1]
        extent = radius * math.sqrt(r[2, 0] ** 2 + r[2, 1] ** 2) + abs(r[2, 2]) * half
        return center[2] - extent
    if gtype == mujoco.mjtGeom.mjGEOM_BOX:
        extent = sum(abs(r[2, j]) * size[j] for j in range(3))
        return center[2] - extent
    # Mesh: exact min-z over the compiled vertex array. MuJoCo bakes the mesh
    # normalization (mesh_pos/mesh_quat) into the geom pos/quat at compile
    # time, so the world surface is geom_xpos + R(geom_quat) @ vert.
    mesh_id = model.geom_dataid[i]
    if mesh_id < 0:
        return center[2]
    adr = model.mesh_vertadr[mesh_id]
    num = model.mesh_vertnum[mesh_id]
    verts = model.mesh_vert[adr : adr + num].reshape(-1, 3)
    r_geom = _quat_to_mat(model.geom_quat[i])
    world = data.geom_xpos[i] + (r_geom @ verts.T).T
    return float(world[:, 2].min())


def _quat_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def render_probe(model_xml: ET.Element, out_png: Path) -> None:
    xml_path = _write_probe(model_xml)
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        from mujoco import MjRenderContextOffscreen

        renderer = MjRenderContextOffscreen(model, 1280, 720)
        renderer.update_scene(data, camera=-1)
        renderer.render()
        pixels = renderer.read_pixels()
        import numpy as np

        image = np.flipud(pixels)
        from PIL import Image

        Image.fromarray(image).save(out_png)
    finally:
        xml_path.unlink(missing_ok=True)


def add_ground_and_light(worldbody: ET.Element) -> None:
    worldbody.append(
        _el(
            "geom",
            name="ground",
            type="plane",
            size="0 0 0.05",
            pos="0 0 -0.001",
            group="1",
            condim="3",
            friction="1.0 0.005 0.0001",
            margin="0.002",
            solref="0.002 1",
            solimp="0.99 0.999 0.0001",
            material="groundplane",
        )
    )
    worldbody.append(
        _el(
            "light",
            pos="0 0 1.5",
            dir="0 0 -1",
            directional="true",
        )
    )


def _build_scene(include_file: str) -> ET.Element:
    root = _el("mujoco", model="scene")
    root.append(_el("include", file=include_file))
    visual = _el("visual")
    visual.append(_el("headlight", diffuse="0.6 0.6 0.6", ambient="0.3 0.3 0.3", specular="0 0 0"))
    visual.append(_el("rgba", haze="0.15 0.25 0.35 1"))
    visual.append(_el("global", azimuth="120", elevation="-20"))
    root.append(visual)
    asset = _el("asset")
    asset.append(_el("texture", type="skybox", builtin="gradient", rgb1="0.3 0.5 0.7", rgb2="0 0 0", width="512", height="3072"))
    asset.append(_el("texture", type="2d", name="groundplane", builtin="checker", mark="edge", rgb1="0.2 0.3 0.4", rgb2="0.1 0.2 0.3", markrgb="0.8 0.8 0.8", width="300", height="300"))
    asset.append(_el("material", name="groundplane", texture="groundplane", texuniform="true", texrepeat="5 5", reflectance="0.2"))
    root.append(asset)
    worldbody = _el("worldbody")
    add_ground_and_light(worldbody)
    worldbody.append(
        _el(
            "camera",
            name="preview",
            pos="2.2 -1.6 1.4",
            xyaxes="0.6644 0.7474 0 -0.1225 0.1089 0.9865",
            fovy="45",
        )
    )
    root.append(worldbody)
    return root
