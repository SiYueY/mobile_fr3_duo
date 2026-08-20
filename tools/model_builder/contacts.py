"""Contact exclusions and hand-finger equality constraints."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import yaml

from . import BuildContext, el


def contacts(ctx: BuildContext) -> ET.Element:
    contact = el("contact")
    pairs: set[tuple[str, str]] = set()
    exclusions = yaml.safe_load(ctx.collision_exclusions.read_text(encoding="utf-8"))
    for first, second in exclusions["disable_collisions"]:
        if first in ctx.urdf.links and second in ctx.urdf.links:
            pairs.add(tuple(sorted((first, second))))
    arm_pairs = {(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (1, 3), (1, 4), (2, 3), (2, 4), (2, 6), (3, 4), (3, 5), (3, 6), (3, 7), (4, 5), (4, 6), (4, 7), (4, 8), (5, 6), (5, 7), (6, 7), (6, 8), (7, 8)}
    hand_pairs = {("hand", "leftfinger"), ("hand", "rightfinger"), ("leftfinger", "rightfinger"), ("hand", "link3"), ("hand", "link4"), ("hand", "link6"), ("hand", "link7"), ("hand", "link8"), ("leftfinger", "link3"), ("leftfinger", "link4"), ("leftfinger", "link6"), ("leftfinger", "link7"), ("leftfinger", "link8"), ("link3", "rightfinger"), ("link4", "rightfinger"), ("link6", "rightfinger"), ("link7", "rightfinger"), ("link8", "rightfinger")}
    for side in ("left", "right"):
        prefix = f"{side}_fr3v2_1_"
        for first, second in arm_pairs:
            for a in (f"link{first}", f"link{first}_sc"):
                for b in (f"link{second}", f"link{second}_sc"):
                    if prefix + a in ctx.urdf.links and prefix + b in ctx.urdf.links:
                        pairs.add(tuple(sorted((prefix + a, prefix + b))))
        for first, second in hand_pairs:
            for a in {first, f"{first}_sc"}:
                for b in {second, f"{second}_sc"}:
                    if prefix + a in ctx.urdf.links and prefix + b in ctx.urdf.links:
                        pairs.add(tuple(sorted((prefix + a, prefix + b))))
    shoulders = {"link0_sc", "link1_sc", "link2_sc"}
    wrists = {"link3", "link3_sc", "link4", "link4_sc", "link5", "link5_sc", "link6", "link6_sc", "link7", "link7_sc", "link8", "hand", "hand_sc", "leftfinger", "rightfinger"}
    for left in ("left", "right"):
        for right in ("left", "right"):
            names_a, names_b = (wrists, wrists) if left == right else (shoulders, shoulders)
            for first in names_a:
                for second in names_b:
                    a, b = f"{left}_fr3v2_1_{first}", f"{right}_fr3v2_1_{second}"
                    if a in ctx.urdf.links and b in ctx.urdf.links:
                        pairs.add(tuple(sorted((a, b))))
    for side in ("left", "right"):
        for shell in shoulders:
            arm = f"{side}_fr3v2_1_{shell}"
            if "franka_spine" in ctx.urdf.links and arm in ctx.urdf.links:
                pairs.add(tuple(sorted(("franka_spine", arm))))
    for joint in ctx.urdf.joints.values():
        first, second = joint.find("parent").get("link"), joint.find("child").get("link")
        if first in ctx.urdf.links and second in ctx.urdf.links:
            pairs.add(tuple(sorted((first, second))))
    for first, second in sorted(pairs):
        contact.append(el("exclude", body1=first, body2=second))
    return contact


def equalities(ctx: BuildContext) -> ET.Element:
    equality = el("equality")
    for side in ("left", "right"):
        first, second = f"{side}_fr3v2_1_finger_joint1", f"{side}_fr3v2_1_finger_joint2"
        if first in ctx.urdf.joints:
            equality.append(el("joint", name=f"{side}_hand_finger_coupling", joint1=first, joint2=second, polycoef="0 1 -1 0 0"))
    return equality
