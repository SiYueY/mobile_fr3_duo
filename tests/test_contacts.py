"""Contact configuration: excludes and real arm-arm collisions."""

import xml.etree.ElementTree as ET

import mujoco
from helpers import MODEL_ROOT


def test_contact_excludes_present(base_model):
    root = ET.parse(MODEL_ROOT / "mobile_fr3_duo.xml").getroot()
    contact = root.find("contact")
    assert contact is not None
    excludes = contact.findall("exclude")
    assert len(excludes) > 300


def test_no_global_arm_exclusion(base_model):
    """Cross-arm and arm-vs-spine pairs are not globally excluded."""
    root = ET.parse(MODEL_ROOT / "mobile_fr3_duo.xml").getroot()
    excluded = {
        (e.get("body1"), e.get("body2"))
        for e in root.find("contact").findall("exclude")
    }
    forbidden_global = {
        ("left_fr3v2_1_link1", "right_fr3v2_1_link1"),
        ("left_fr3v2_1_link3", "right_fr3v2_1_link3"),
        ("left_fr3v2_1_hand", "right_fr3v2_1_hand"),
        ("left_fr3v2_1_link2", "franka_spine"),
        ("left_fr3v2_1_link5", "right_fr3v2_1_link5"),
    }
    for pair in forbidden_global:
        assert tuple(sorted(pair)) not in excluded, f"{pair} excluded"


def test_arm_arm_collision_detected(scene_model):
    """Driving both arms together produces real link-link contacts."""
    data = mujoco.MjData(scene_model)
    mujoco.mj_resetDataKeyframe(scene_model, data, 0)

    def qadr(name):
        return scene_model.jnt_qposadr[
            mujoco.mj_name2id(scene_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        ]

    # Pose found by a seeded scan on the URDF-correct kinematic chain.
    ql = [2.78456, -0.50312, 2.504965, -1.69463, -2.170263, 2.74022, -0.73034]
    qr = [-0.891964, 0.835503, -2.218115, -2.632409, -2.444344, 4.602039, 1.97178]
    for side, q in (("left", ql), ("right", qr)):
        for i, v in enumerate(q):
            data.qpos[qadr(f"{side}_fr3v2_1_joint{i + 1}")] = v
    mujoco.mj_forward(scene_model, data)
    arm_ids = set()
    for i in range(scene_model.nbody):
        n = mujoco.mj_id2name(scene_model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
        if "fr3v2_1" in n or "hand" in n or "finger" in n:
            arm_ids.add(i)
    arm_arm = False
    for c in range(data.ncon):
        b1 = scene_model.geom_bodyid[data.contact[c].geom1]
        b2 = scene_model.geom_bodyid[data.contact[c].geom2]
        if b1 in arm_ids and b2 in arm_ids and data.contact[c].dist < -0.002:
            arm_arm = True
            break
    assert arm_arm, "expected arm-arm contact when arms are driven together"
