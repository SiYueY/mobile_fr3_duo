"""Freeze SRDF disable-collision pairs for the standalone MJCF builder."""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source", required=True)
    args = parser.parse_args()

    srdf = ET.parse(sys.stdin).getroot()
    urdf = ET.parse(args.urdf).getroot()
    links = {link.get("name") for link in urdf.findall("link")}
    pairs = sorted(
        {
            tuple(sorted((item.get("link1"), item.get("link2"))))
            for item in srdf.findall(".//disable_collisions")
            if item.get("link1") in links and item.get("link2") in links
        }
    )
    lines = [
        "generated_by: tools/extract_collision_exclusions.py",
        f"source: {args.source}",
        "disable_collisions:",
    ]
    lines.extend(f"  - [{link1}, {link2}]" for link1, link2 in pairs)
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
