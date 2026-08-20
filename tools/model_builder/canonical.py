"""Canonical, source-traceable view of the frozen official URDF."""

from __future__ import annotations

from dataclasses import dataclass

from .urdf import UrdfModel


@dataclass(frozen=True)
class CanonicalBody:
    name: str
    parent: str | None
    joint: str | None


@dataclass(frozen=True)
class CanonicalModel:
    """Ordered body graph used to validate module cuts against the URDF."""

    source: str
    bodies: tuple[CanonicalBody, ...]

    @classmethod
    def from_urdf(cls, model: UrdfModel, source: str) -> CanonicalModel:
        bodies = []
        for name in model.links:
            joint = model.child_to_joint.get(name)
            parent = joint.find("parent").get("link") if joint is not None else None
            bodies.append(CanonicalBody(name, parent, joint.get("name") if joint is not None else None))
        return cls(source=source, bodies=tuple(bodies))

    def contains(self, body: str) -> bool:
        return any(item.name == body for item in self.bodies)
