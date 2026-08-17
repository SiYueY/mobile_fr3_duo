"""Generate the raw MuJoCo import of the official URDF as an intermediate.

The MuJoCo built-in URDF importer parses the generated visual URDF and
produces the "raw" reference MJCF (document section 11.4). The native MJCF
deliverables in the repository root are hand-built from the same URDF; this
raw file is kept only as an intermediate cross-check baseline.

Usage:
  python tools/generate_raw_mjcf.py
"""

from __future__ import annotations

import re
import tempfile
from pathlib import Path

import mujoco

REPO_ROOT = Path(__file__).resolve().parent.parent
FD_CACHE = Path(
    "/home/siyuey/workspace/mujoco/_third_party_cache/franka_description"
)
URDF = REPO_ROOT / "source" / "generated" / "mobile_fr3_duo_visual.urdf"
OUT = REPO_ROOT / "source" / "generated" / "mobile_fr3_duo_raw.xml"


def main() -> int:
    text = URDF.read_text(encoding="utf-8")
    if not FD_CACHE.exists():
        print("error: franka_description cache not available for mesh paths")
        return 1
    text = re.sub(
        r"package://franka_description/([^\"']+)",
        lambda m: str((FD_CACHE / m.group(1)).resolve()),
        text,
    )
    with tempfile.TemporaryDirectory() as tmp:
        tmp_urdf = Path(tmp) / "robot.urdf"
        tmp_urdf.write_text(text, encoding="utf-8")
        try:
            model = mujoco.MjModel.from_xml_path(str(tmp_urdf))
        except Exception as exc:  # noqa: BLE001
            print(f"URDF import failed: {exc}")
            return 1
        OUT.parent.mkdir(parents=True, exist_ok=True)
        mujoco.mj_saveLastXML(str(OUT), model)
        print(f"wrote {OUT} (nbody={model.nbody} njnt={model.njnt})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
