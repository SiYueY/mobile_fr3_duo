"""Build native MJCF models for Mobile FR3 Duo from official URDFs.

Generated deliverables:
  mobile_fr3_duo.xml                base model (motor actuators, no sensor entities)
  mobile_fr3_duo_position.xml       position-actuator variant
  mobile_fr3_duo_with_sensors.xml   base + D455/nanoScan3/IMU/ZED entities
  mobile_fr3_duo_reduced.xml        reduced TMR variant
  mobile_fr3_duo_planar_debug.xml   planar proxy variant (debug only)
  scene.xml / scene_with_sensors.xml

All parameters not marked official-source/derived are recorded in
source/parameter_sources.yaml.
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml
from format_xml import format_element
from model_builder import BuildContext, contacts, dynamics, geometry
from model_builder.config import BuilderConfig
from model_builder.config import load as load_builder_config
from model_builder.geometry import mesh_asset
from model_builder.urdf import UrdfModel, children, merge_sc_links
from scipy.spatial.transform import Rotation
from urdf_common import fmt, fmt_vec, origin_attrib

REPO_ROOT = Path(__file__).resolve().parent.parent
GENERATED = REPO_ROOT / "source" / "generated"

VISUAL_URDF = GENERATED / "mobile_fr3_duo_visual.urdf"
SC_URDF = GENERATED / "mobile_fr3_duo_self_collision.urdf"
REDUCED_URDF = GENERATED / "mobile_fr3_duo_reduced.urdf"
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


def _quat_from_origin(elem) -> np.ndarray:
    _, quat = origin_attrib(elem)
    return quat


def _geom_origin(elem) -> tuple[str, str]:
    xyz, quat = origin_attrib(elem)
    return fmt_vec(xyz), fmt_vec(quat)


class ModelBuilder:
    def __init__(self, opts: argparse.Namespace):
        self.opts = opts
        urdf_path = REDUCED_URDF if opts.reduced else VISUAL_URDF
        self.urdf = merge_sc_links(UrdfModel(urdf_path), UrdfModel(SC_URDF))
        self.actuator_mode = "position" if opts.position else "motor"
        self.config: BuilderConfig = load_builder_config(REPO_ROOT / "config")
        self.context = BuildContext(opts, self.urdf, self.actuator_mode, COLLISION_EXCLUSIONS)

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
                meshdir="assets",
                autolimits="true",
                inertiafromgeom="false",
                fusestatic="false",
            )
        )
        mujoco_root.append(
            _el(
                "option",
                timestep="0.001",
                integrator="implicitfast",
                solver="Newton",
                iterations="50",
                ls_iterations="10",
                gravity="0 0 -9.81",
            )
        )
        mujoco_root.append(geometry.defaults())
        mujoco_root.append(geometry.assets(self.context))

        spawn_z = self.opts.spawn_z
        if spawn_z is None:
            spawn_z = 0.0
        worldbody = _el("worldbody")
        worldbody.append(self._emit_body(self.urdf.root_link, spawn_z, is_root=True))
        if self.opts.sensors:
            self._attach_sensor_entities(worldbody)
        mujoco_root.append(worldbody)

        mujoco_root.append(contacts.contacts(self.context))
        mujoco_root.append(contacts.equalities(self.context))
        mujoco_root.append(self._actuators())
        mujoco_root.append(self._sensors())
        mujoco_root.append(self._keyframes(spawn_z))
        if self.actuator_mode == "motor" and not self.opts.planar:
            self._fill_gravity_comp_ctrl(mujoco_root)
        return mujoco_root

    # ------------------------------------------------------------------
    def _defaults(self) -> ET.Element:
        default = _el("default")
        default.append(_el("geom", density="0"))
        for class_name, attrs in (
            ("visual", dict(contype="0", conaffinity="0", group="2", density="0")),
            (
                "collision",
                dict(
                    group="3", contype="1", conaffinity="1", condim="3",
                    friction="1.0 0.005 0.0001", density="0",
                ),
            ),
            (
                "wheel",
                dict(
                    group="3", contype="1", conaffinity="1", condim="3",
                    friction="1.2 0.005 0.0001", density="0",
                ),
            ),
            (
                "finger_pad",
                dict(
                    group="3", contype="1", conaffinity="1", condim="4",
                    friction="1.0 0.005 0.0001", solref="0.02 1",
                    solimp="0.9 0.95 0.001", density="0",
                ),
            ),
            (
                "sensor_collision",
                dict(group="4", contype="1", conaffinity="1", density="0"),
            ),
        ):
            class_default = _el("default", **{"class": class_name})
            class_default.append(_el("geom", **attrs))
            default.append(class_default)
        return default

    # ------------------------------------------------------------------
    def _assets(self) -> ET.Element:
        asset = _el("asset")
        seen: set[str] = set()
        for link in self.urdf.links.values():
            for geom_kind in ("visual", "collision"):
                for g in link.findall(geom_kind):
                    mesh = g.find("./geometry/mesh")
                    if mesh is None:
                        continue
                    mapped = mesh_asset(mesh.get("filename"))
                    if mapped is None or mapped[0] in seen:
                        continue
                    seen.add(mapped[0])
                    name, path = mapped
                    asset.append(_el("mesh", name=name, file=path))
        if self.opts.sensors:
            asset.append(
                _el(
                    "mesh",
                    name="d455",
                    file="sensors/realsense_d455/d455.stl",
                )
            )
            asset.append(
                _el(
                    "mesh",
                    name="nanoscan3_visual",
                    file="sensors/sick_nanoscan3/NANS3.obj",
                )
            )
            asset.append(
                _el(
                    "mesh",
                    name="nanoscan3_collision",
                    file="sensors/sick_nanoscan3/NANS3_collision.stl",
                )
            )
            asset.append(
                _el(
                    "mesh",
                    name="zed_mini",
                    file="sensors/zed_mini/zedm.stl",
                )
            )
        return asset

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
            if self.opts.planar:
                body.append(
                    _el(
                        "joint",
                        name="planar_x_joint",
                        type="slide",
                        axis="1 0 0",
                        limited="true",
                        range="-50 50",
                        damping="10",
                    )
                )
                body.append(
                    _el(
                        "joint",
                        name="planar_y_joint",
                        type="slide",
                        axis="0 1 0",
                        limited="true",
                        range="-50 50",
                        damping="10",
                    )
                )
                body.append(
                    _el(
                        "joint",
                        name="planar_yaw_joint",
                        type="hinge",
                        axis="0 0 1",
                        damping="5",
                    )
                )
            else:
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
            g = geometry.visual(geom)
            if g is not None:
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

    def _children(self, parent: str) -> list[str]:
        out = []
        for j in self.urdf.joints.values():
            if j.find("parent").get("link") == parent:
                out.append(j.find("child").get("link"))
        return out

    def _emit_joint(self, joint: ET.Element) -> ET.Element:
        jtype = joint.get("type")
        name = joint.get("name")
        limit = joint.find("limit")
        dyn = joint.find("dynamics")
        axis = joint.find("axis")

        if jtype == "fixed":
            raise ValueError("fixed joints are represented by body nesting, not <joint>")
        if jtype in ("revolute", "continuous"):
            mtype = "hinge"
        elif jtype == "prismatic":
            mtype = "slide"
        else:
            raise ValueError(f"unsupported joint type {jtype}")

        # The child body frame is the URDF joint frame at q=0, so the URDF
        # joint axis is already expressed in MuJoCo's local coordinates.
        axis_attr = axis.get("xyz") if axis is not None else "0 0 1"
        attrs: dict[str, str] = {
            "name": name,
            "type": mtype,
            "axis": axis_attr,
        }
        if limit is not None:
            lower = limit.get("lower")
            upper = limit.get("upper")
            if lower is not None and upper is not None:
                attrs["limited"] = "true"
                attrs["range"] = f"{fmt(float(lower))} {fmt(float(upper))}"
        if dyn is not None:
            if dyn.get("damping") is not None:
                attrs["damping"] = fmt(float(dyn.get("damping")))
            if dyn.get("friction") is not None:
                attrs["frictionloss"] = fmt(float(dyn.get("friction")))
        return _el("joint", **attrs)

    def _emit_inertial(self, inertial: ET.Element) -> ET.Element:
        mass = _inertial_mass(inertial)
        xyz, quat = _geom_origin(inertial)
        inertia = inertial.find("inertia")
        if inertia is None:
            raise ValueError("inertial missing <inertia>")
        ixx = float(inertia.get("ixx"))
        iyy = float(inertia.get("iyy"))
        izz = float(inertia.get("izz"))
        ixy = float(inertia.get("ixy"))
        ixz = float(inertia.get("ixz"))
        iyz = float(inertia.get("iyz"))
        i_inertial = np.array(
            [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
        )
        # Rotate the inertia tensor into the body frame; MuJoCo's fullinertia
        # cannot be combined with an inertial orientation.
        q = np.asarray(quat.split(), dtype=float)
        r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        i_body = r.as_matrix() @ i_inertial @ r.as_matrix().T
        full = " ".join(
            fmt(v, 12)
            for v in (
                i_body[0, 0],
                i_body[1, 1],
                i_body[2, 2],
                i_body[0, 1],
                i_body[0, 2],
                i_body[1, 2],
            )
        )
        return _el(
            "inertial",
            pos=xyz,
            mass=fmt(mass, 12),
            fullinertia=full,
        )

    def _emit_inertial_in_frame(
        self, inertial: ET.Element, r_origin: np.ndarray
    ) -> ET.Element:
        """Emit an inertial expressed in the intermediate joint-frame body."""
        mass = _inertial_mass(inertial)
        xyz, quat = _geom_origin(inertial)
        inertia = inertial.find("inertia")
        ixx = float(inertia.get("ixx"))
        iyy = float(inertia.get("iyy"))
        izz = float(inertia.get("izz"))
        ixy = float(inertia.get("ixy"))
        ixz = float(inertia.get("ixz"))
        iyz = float(inertia.get("iyz"))
        i_inertial = np.array(
            [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
        )
        q = np.asarray(quat.split(), dtype=float)
        r_inertial = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        r_total = r_origin @ r_inertial.as_matrix()
        i_body = r_total @ i_inertial @ r_total.T
        pos = r_origin @ np.asarray(xyz.split(), dtype=float)
        full = " ".join(
            fmt(v, 12)
            for v in (
                i_body[0, 0],
                i_body[1, 1],
                i_body[2, 2],
                i_body[0, 1],
                i_body[0, 2],
                i_body[1, 2],
            )
        )
        return _el(
            "inertial",
            pos=fmt_vec(pos),
            mass=fmt(mass, 12),
            fullinertia=full,
        )

    def _emit_visual(self, geom: ET.Element) -> ET.Element:
        pos, quat = _geom_origin(geom)
        geometry = geom.find("geometry")
        child = list(geometry)[0]
        rgba = None
        material = geom.find("material")
        if material is not None:
            color = material.find("color")
            if color is not None and color.get("rgba"):
                rgba = color.get("rgba")
        attrs = {"class": "visual", "pos": pos, "quat": quat}
        if rgba is not None:
            attrs["rgba"] = rgba
        if child.tag == "mesh":
            mapped = mesh_asset(child.get("filename"))
            if mapped is None:
                return None
            return _el("geom", **attrs, type="mesh", mesh=mapped[0])
        if child.tag == "box":
            # URDF box size is full extent; MuJoCo box size is half extent.
            half = [fmt(float(v) / 2.0) for v in child.get("size").split()]
            return _el("geom", **attrs, type="box", size=" ".join(half))
        if child.tag == "cylinder":
            radius = float(child.get("radius"))
            length = float(child.get("length"))
            return _el(
                "geom",
                **attrs,
                type="cylinder",
                size=f"{fmt(radius)} {fmt(length / 2.0)}",
            )
        if child.tag == "sphere":
            return _el("geom", **attrs, type="sphere", size=child.get("radius"))
        return None

    def _emit_collision(self, geom: ET.Element, link_name: str) -> ET.Element:
        pos, quat = _geom_origin(geom)
        geometry = geom.find("geometry")
        child = list(geometry)[0]
        is_wheel = link_name in {
            "argo_drive_front_link",
            "argo_drive_rear_link",
            "caster_front_left_link",
            "caster_rear_right_link",
        }
        is_finger = link_name.endswith("finger")
        cls = "wheel" if is_wheel else ("finger_pad" if is_finger else "collision")

        if child.tag == "mesh":
            mapped = mesh_asset(child.get("filename"))
            if mapped is None:
                return None
            return _el(
                "geom",
                **{"class": cls},
                type="mesh",
                mesh=mapped[0],
                pos=pos,
                quat=quat,
            )
        if child.tag == "box":
            # URDF box size is the FULL extent; MuJoCo box size is the
            # half-extent.
            half = [fmt(float(v) / 2.0) for v in child.get("size").split()]
            return _el(
                "geom",
                **{"class": cls},
                type="box",
                size=" ".join(half),
                pos=pos,
                quat=quat,
            )
        if child.tag == "cylinder":
            radius = float(child.get("radius"))
            length = float(child.get("length"))
            return _el(
                "geom",
                **{"class": cls},
                type="cylinder",
                size=f"{fmt(radius)} {fmt(length / 2.0)}",
                pos=pos,
                quat=quat,
            )
        if child.tag == "sphere":
            return _el(
                "geom",
                **{"class": cls},
                type="sphere",
                size=child.get("radius"),
                pos=pos,
                quat=quat,
            )
        return None

    # ------------------------------------------------------------------
    def _contacts(self) -> ET.Element:
        contact = _el("contact")
        pairs: set[tuple[str, str]] = set()
        # Official SRDF disable-collision pairs frozen during source preparation.
        exclusions = yaml.safe_load(COLLISION_EXCLUSIONS.read_text(encoding="utf-8"))
        for l1, l2 in exclusions["disable_collisions"]:
            if l1 in self.urdf.links and l2 in self.urdf.links:
                pairs.add(tuple(sorted((l1, l2))))
        # Official arm SRDF pairs (robots/common/franka_arm.srdf.xacro). The
        # mobile SRDF in franka_description@2.8.1 only covers the legacy
        # fr3v2 (non-v2_1) names, so we expand the canonical arm pair table
        # for both arms here, including their *_sc self-collision shells.
        arm_link_pairs = {
            (0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (1, 3), (1, 4),
            (2, 3), (2, 4), (2, 6), (3, 4), (3, 5), (3, 6), (3, 7),
            (4, 5), (4, 6), (4, 7), (4, 8), (5, 6), (5, 7),
            (6, 7), (6, 8), (7, 8),
        }
        hand_link_pairs = {
            ("hand", "leftfinger"), ("hand", "rightfinger"),
            ("leftfinger", "rightfinger"), ("hand", "link3"),
            ("hand", "link4"), ("hand", "link6"), ("hand", "link7"),
            ("hand", "link8"), ("leftfinger", "link3"),
            ("leftfinger", "link4"), ("leftfinger", "link6"),
            ("leftfinger", "link7"), ("leftfinger", "link8"),
            ("link3", "rightfinger"), ("link4", "rightfinger"),
            ("link6", "rightfinger"), ("link7", "rightfinger"),
            ("link8", "rightfinger"),
        }
        for side in HAND_PREFIXES:
            prefix = f"{side}_fr3v2_1_"
            for a, b in arm_link_pairs:
                for variant_a in (f"link{a}", f"link{a}_sc"):
                    for variant_b in (f"link{b}", f"link{b}_sc"):
                        la, lb = prefix + variant_a, prefix + variant_b
                        if la in self.urdf.links and lb in self.urdf.links:
                            pairs.add(tuple(sorted((la, lb))))
            for name_a, name_b in hand_link_pairs:
                for variant_a in {name_a, f"{name_a}_sc"}:
                    for variant_b in {name_b, f"{name_b}_sc"}:
                        la, lb = prefix + variant_a, prefix + variant_b
                        if la in self.urdf.links and lb in self.urdf.links:
                            pairs.add(tuple(sorted((la, lb))))
        # Inflated self-collision shells overlap even at legal poses in the
        # dual-arm mounting: the shoulders are only ~0.1 m apart while the
        # official link0-2 shells have radius 0.09, and the wrist/hand shells
        # inflate into each other at most natural wrist configurations. These
        # shell-only pairs are excluded so the shells keep acting as proximity
        # margins without creating phantom always-on contacts. All real-geometry
        # (main mesh) pairs stay active, so physical arm-arm/arm-body collisions
        # are still detected. Recorded in source/parameter_sources.yaml.
        shoulder_shells = {"link0_sc", "link1_sc", "link2_sc"}
        wrist_cluster = {
            "link3", "link3_sc", "link4", "link4_sc", "link5", "link5_sc",
            "link6", "link6_sc", "link7", "link7_sc", "link8", "hand",
            "hand_sc", "leftfinger", "rightfinger",
        }
        for side_a in HAND_PREFIXES:
            for side_b in HAND_PREFIXES:
                if side_a == side_b:
                    for name_a in wrist_cluster:
                        for name_b in wrist_cluster:
                            la = f"{side_a}_fr3v2_1_{name_a}"
                            lb = f"{side_b}_fr3v2_1_{name_b}"
                            if la in self.urdf.links and lb in self.urdf.links:
                                pairs.add(tuple(sorted((la, lb))))
                else:
                    for name_a in shoulder_shells:
                        for name_b in shoulder_shells:
                            la = f"{side_a}_fr3v2_1_{name_a}"
                            lb = f"{side_b}_fr3v2_1_{name_b}"
                            if la in self.urdf.links and lb in self.urdf.links:
                                pairs.add(tuple(sorted((la, lb))))
        # Spine column vs shoulder shells: the inflated link0-2 shells reach
        # the spine at every spine height (official SRDF disables these
        # pairs). Main-mesh spine-arm collisions stay active.
        for side in HAND_PREFIXES:
            for shell in shoulder_shells:
                la = f"{side}_fr3v2_1_{shell}"
                if "franka_spine" in self.urdf.links and la in self.urdf.links:
                    pairs.add(tuple(sorted(("franka_spine", la))))
        # Direct parent-child pairs.
        for j in self.urdf.joints.values():
            p = j.find("parent").get("link")
            c = j.find("child").get("link")
            if p in self.urdf.links and c in self.urdf.links:
                pairs.add(tuple(sorted((p, c))))
        for b1, b2 in sorted(pairs):
            contact.append(_el("exclude", body1=b1, body2=b2))
        return contact

    def _equalities(self) -> ET.Element:
        equality = _el("equality")
        for side in HAND_PREFIXES:
            joint1 = f"{side}_fr3v2_1_finger_joint1"
            joint2 = f"{side}_fr3v2_1_finger_joint2"
            if joint1 not in self.urdf.joints:
                continue
            equality.append(
                _el(
                    "joint",
                    name=f"{side}_hand_finger_coupling",
                    joint1=joint1,
                    joint2=joint2,
                    # MuJoCo joint equality polynomial is
                    # c0 + c1*q1 + c2*q2 + ... = 0, so q1 == q2 means
                    # 0 + 1*q1 - 1*q2 = 0.
                    polycoef="0 1 -1 0 0",
                )
            )
        return equality

    # ------------------------------------------------------------------
    def _actuators(self) -> ET.Element:
        actuator = _el("actuator")
        if self.opts.planar:
            for spec in self.config.planar:
                actuator.append(
                    _el(
                        "position",
                        name=spec.name,
                        joint=spec.joint,
                        kp="1000",
                        ctrllimited="true",
                        ctrlrange=f"{fmt(spec.ctrlrange[0])} {fmt(spec.ctrlrange[1])}",
                    )
                )
        if self.actuator_mode == "position":
            kp_map = {
                "tmrv0_2_joint_": 100.0,
                "franka_spine_vertical_joint": 30000.0,
                "finger_joint1": 200.0,
            }
            for jname in self._actuator_joints():
                if jname.startswith("planar_"):
                    continue
                jtype = self.urdf.joints[jname].get("type")
                limit = self.urdf.joints[jname].find("limit")
                if jtype == "continuous" or limit is None or limit.get("lower") is None:
                    lower, upper = -1e4, 1e4
                else:
                    lower = float(limit.get("lower"))
                    upper = float(limit.get("upper"))
                kp = next((v for k, v in kp_map.items() if k in jname), 500.0)
                actuator.append(
                    _el(
                        "position",
                        name=f"{jname}_actuator",
                        joint=jname,
                        kp=fmt(kp),
                        ctrllimited="true",
                        ctrlrange=f"{fmt(lower)} {fmt(upper)}",
                    )
                )
            return actuator

        # Motor variant.
        for spec in self.config.tmr:
            if spec.joint not in self.urdf.joints:
                continue
            actuator.append(
                _el(
                    "motor",
                    name=spec.name,
                    joint=spec.joint,
                    gear="1",
                    ctrllimited="true",
                    ctrlrange=f"{fmt(spec.ctrlrange[0])} {fmt(spec.ctrlrange[1])}",
                )
            )
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
        if self.opts.sensors:
            sensor.append(_el("gyro", name="imu_angular_velocity", site="imu_sensor_frame"))
            sensor.append(
                _el("accelerometer", name="imu_linear_acceleration", site="imu_sensor_frame")
            )
            sensor.append(
                _el("framequat", name="imu_orientation", objtype="site", objname="imu_sensor_frame")
            )
        return sensor

    def _active_joints_in_order(self) -> list[str]:
        """Non-fixed joints in the same order MuJoCo assigns DOFs."""
        order: list[str] = []
        if self.opts.planar:
            order += ["planar_x_joint", "planar_y_joint", "planar_yaw_joint"]

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
        if self.opts.planar:
            out += ["planar_x_joint", "planar_y_joint", "planar_yaw_joint"]
        out += [spec.joint for spec in self.config.tmr]
        out.append(self.config.spine.joint)
        out += [f"{p}_joint{i}" for p in ARM_PREFIXES for i in range(1, 8)]
        out += [f"{s}_fr3v2_1_finger_joint1" for s in HAND_PREFIXES]
        return [j for j in out if j in self.urdf.joints]

    # ------------------------------------------------------------------
    def _keyframes(self, spawn_z: float) -> ET.Element:
        keyframe = _el("keyframe")
        dof_order = self._active_joints_in_order()
        if self.opts.planar:
            qpos_prefix = [0.0, 0.0, 0.0]
        else:
            qpos_prefix = [0.0, 0.0, spawn_z, 1.0, 0.0, 0.0, 0.0]
        for name, pose in self.config.keyframes.items():
            qpos = list(qpos_prefix)
            for jname in dof_order:
                if jname.startswith("planar_"):
                    continue
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
        if self.actuator_mode == "position" or self.opts.planar:
            prefix = 3 if self.opts.planar else 7
            q_dofs = [j for j in dof_order if not j.startswith("planar_")]
            joint_to_q = dict(zip(q_dofs, qpos[prefix:], strict=True))
            ctrl = []
            for jname in self._actuator_joints():
                ctrl.append(joint_to_q.get(jname, 0.0))
            return ctrl
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
        body.append(
            _el(
                "geom",
                **{"class": "sensor_collision"},
                type="box",
                size="0.015 0.015 0.005",
            )
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
                mesh="d455",
                pos="0.00465 -0.0475 0",
                quat=_fmt_quat((math.pi / 2, 0, math.pi / 2)),
            )
        )
        body.append(
            _el(
                "geom",
                **{"class": "sensor_collision"},
                type="box",
                size="0.013 0.062 0.0145",
                pos="-0.00845 -0.0475 0",
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
                pos="-0.01 -0.08 -0.04",
                quat=_fmt_quat((0, 0, math.pi / 2)),
            )
        )
        body.append(
            _el(
                "geom",
                **{"class": "sensor_collision"},
                type="mesh",
                mesh="nanoscan3_collision",
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
            )
        )
        body.append(
            _el(
                "geom",
                **{"class": "sensor_collision"},
                type="box",
                size="0.062 0.015 0.015",
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


def _inertial_mass(inertial: ET.Element) -> float | None:
    mass = inertial.find("mass")
    return float(mass.get("value")) if mass is not None else None


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
            size="50 50 0.1",
            pos="0 0 -0.001",
            group="1",
            condim="3",
            friction="1.0 0.005 0.0001",
            rgba="0.55 0.55 0.55 1",
        )
    )
    # low-angle fill light so the wheel protrusions below the chassis are
    # visible in preview renders (visualization only)
    worldbody.append(
        _el(
            "light",
            directional="true",
            diffuse="0.5 0.5 0.5",
            specular="0.1 0.1 0.1",
            pos="1.5 -1.2 -0.5",
            dir="-0.6 0.5 0.6",
            castshadow="false",
        )
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--position", action="store_true", help="build position variant")
    ap.add_argument("--sensors", action="store_true", help="build with-sensors variant")
    ap.add_argument("--reduced", action="store_true", help="build reduced TMR variant")
    ap.add_argument("--planar", action="store_true", help="build planar debug variant")
    ap.add_argument("--spawn-z", type=float, default=None, help="override spawn height")
    ap.add_argument("--all", action="store_true", help="build all variants")
    args = ap.parse_args()

    GENERATED.mkdir(parents=True, exist_ok=True)

    variants = []
    if args.all or not any([args.position, args.sensors, args.reduced, args.planar]):
        variants.append(("base", argparse.Namespace(position=False, sensors=False, reduced=False, planar=False, spawn_z=args.spawn_z)))
    if args.all or args.position:
        variants.append(("position", argparse.Namespace(position=True, sensors=False, reduced=False, planar=False, spawn_z=args.spawn_z)))
    if args.all or args.sensors:
        variants.append(("sensors", argparse.Namespace(position=False, sensors=True, reduced=False, planar=False, spawn_z=args.spawn_z)))
    if args.all or args.reduced:
        variants.append(("reduced", argparse.Namespace(position=False, sensors=False, reduced=True, planar=False, spawn_z=args.spawn_z)))
    if args.all or args.planar:
        variants.append(("planar", argparse.Namespace(position=False, sensors=False, reduced=False, planar=True, spawn_z=args.spawn_z)))

    names = {
        "base": "mobile_fr3_duo.xml",
        "position": "mobile_fr3_duo_position.xml",
        "sensors": "mobile_fr3_duo_with_sensors.xml",
        "reduced": "mobile_fr3_duo_reduced.xml",
        "planar": "mobile_fr3_duo_planar_debug.xml",
    }

    for variant, opts in variants:
        builder = ModelBuilder(opts)
        if opts.planar:
            opts.spawn_z = 0.0
        if args.all and opts.spawn_z is None:
            # Two-pass: compute spawn height from the base variant.
            probe = ModelBuilder(argparse.Namespace(position=False, sensors=False, reduced=opts.reduced, planar=False, spawn_z=0.0))
            opts.spawn_z = compute_spawn_z(probe.build())
        elif opts.spawn_z is None:
            opts.spawn_z = compute_spawn_z(builder.build())
        root = builder.build()
        out = REPO_ROOT / names[variant]
        out.write_text(format_element(root), encoding="utf-8")
        print(f"built {out.name} (spawn_z={opts.spawn_z:.4f})")

    if args.all:
        base_xml = (REPO_ROOT / "mobile_fr3_duo.xml").read_text(encoding="utf-8")
        scene = _build_scene(base_xml, "mobile_fr3_duo.xml", "scene.xml")
        (REPO_ROOT / "scene.xml").write_text(format_element(scene), encoding="utf-8")
        sensors_xml = (REPO_ROOT / "mobile_fr3_duo_with_sensors.xml").read_text(
            encoding="utf-8"
        )
        scene_sensors = _build_scene(
            sensors_xml, "mobile_fr3_duo_with_sensors.xml", "scene_with_sensors.xml"
        )
        (REPO_ROOT / "scene_with_sensors.xml").write_text(
            format_element(scene_sensors), encoding="utf-8"
        )
        position_xml = (REPO_ROOT / "mobile_fr3_duo_position.xml").read_text(
            encoding="utf-8"
        )
        scene_position = _build_scene(
            position_xml, "mobile_fr3_duo_position.xml", "scene_position.xml"
        )
        (REPO_ROOT / "scene_position.xml").write_text(
            format_element(scene_position), encoding="utf-8"
        )
    return 0


def _build_scene(robot_xml: str, include_file: str, model_name: str) -> ET.Element:
    root = _el("mujoco", model="scene")
    root.append(_el("include", file=include_file))
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
    worldbody.append(
        _el(
            "light",
            directional="true",
            diffuse="0.8 0.8 0.8",
            specular="0.1 0.1 0.1",
            pos="0 0 3",
            dir="0 0 -1",
        )
    )
    root.append(worldbody)
    return root


if __name__ == "__main__":
    raise SystemExit(main())
