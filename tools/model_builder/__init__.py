"""Internal building blocks for the Mobile FR3 Duo MJCF generator."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class BuildContext:
    """Immutable inputs shared by all model-generation sections."""

    opts: Any
    urdf: Any
    actuator_mode: str
    collision_exclusions: Path


def el(tag: str, **attrs: object):
    """Create an XML element, omitting attributes whose value is ``None``."""
    import xml.etree.ElementTree as ET

    elem = ET.Element(tag)
    for key, value in attrs.items():
        if value is not None:
            elem.set(key, str(value))
    return elem
