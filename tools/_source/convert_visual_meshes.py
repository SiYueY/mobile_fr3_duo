"""Convert official visual DAE meshes to OBJ assets in meters.

COLLADA files may contain multiple scene geometries/material parts and may
also declare non-meter units (for example millimeters). This tool preserves
the DAE scene geometry split instead of flattening the whole file into one
Trimesh, bakes scene-node transforms into each exported mesh, and normalizes
all exported geometry to meters.

Output naming is deterministic for a fixed source DAE and trimesh version:

- one mesh:  ``link0.obj``
- many meshes: ``link0_0.obj``, ``link0_1.obj``, ...

Input/output SHA-256 hashes, applied unit scale, tool version, geometry counts,
and per-output statistics are recorded in
``source/generated/asset_conversion.json``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import trimesh
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
MODELS = REPO_ROOT / "models"
GENERATED = REPO_ROOT / "source" / "generated"
MIN_OUTPUT_EXTENT = 1e-4  # meters; matches tools/validate_model.py

# These CAD assemblies contain hundreds of material-less helper geometries.
# Preserve their source-part count in the manifest while exporting one visual
# mesh, which avoids an otherwise useless explosion of MJCF assets and geoms.
MERGED_VISUAL_SOURCES = {
    "package://franka_description/meshes/robots/tmrv0_2/visual/tmrv0_2.dae",
}

# package://franka_description/meshes/<...>/<kind>/<file>
#     -> models/<module>/assets/<kind>/<file>
COMPONENT_MAP = {
    "accessories/fr3_duo_mount_v0_3": "franka_head/assets",
    "accessories/franka_head_v0_2": "franka_head/assets",
    "accessories/franka_spine_v0_1": "franka_spine/assets",
    "robots/fr3v2_1": "franka_fr3/assets",
    "robots/tmrv0_2": "franka_tmr/assets",
    "robot_ee/franka_hand_white": "franka_hand/assets",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dae_unit_scale(path: Path) -> float:
    """Return the meters-per-unit scale declared by a COLLADA file."""
    text = path.read_text(errors="ignore")
    match = re.search(
        r'<unit[^>]*meter\s*=\s*"([0-9.eE+-]+)"',
        text,
        re.IGNORECASE,
    )
    if match is None:
        # COLLADA default unit is meter.
        return 1.0
    return float(match.group(1))


def _collada_id(value: str | None) -> str | None:
    """Normalise a COLLADA URI such as ``#paint_white`` to its identifier."""
    if not value:
        return None
    return value.removeprefix("#")


def collada_materials(path: Path) -> dict[str, dict[str, object]]:
    """Map COLLADA geometry identifiers to their bound diffuse materials.

    Trimesh does not consistently retain COLLADA materials after
    ``Scene.dump()``.  Parse the standard ``effect -> material ->
    instance_geometry/bind_material`` chain directly so visual appearance is
    independent of loader implementation details.  A geometry with multiple
    bindings is intentionally left unassigned: one exported geometry cannot
    safely represent several different material slots without a further face
    split.
    """
    root = ET.parse(path).getroot()
    effects: dict[str, list[float]] = {}
    for effect in root.findall(".//{*}library_effects/{*}effect"):
        effect_id = effect.get("id")
        color = effect.find(".//{*}profile_COMMON//{*}diffuse/{*}color")
        if not effect_id or color is None or not color.text:
            continue
        try:
            rgba = [float(value) for value in color.text.split()]
        except ValueError:
            continue
        if len(rgba) == 3:
            rgba.append(1.0)
        if len(rgba) == 4:
            effects[effect_id] = [max(0.0, min(1.0, value)) for value in rgba]

    materials: dict[str, dict[str, object]] = {}
    for material in root.findall(".//{*}library_materials/{*}material"):
        material_id = material.get("id")
        instance_effect = material.find("{*}instance_effect")
        if not material_id or instance_effect is None:
            continue
        rgba = effects.get(_collada_id(instance_effect.get("url")) or "")
        if rgba is None:
            continue
        materials[material_id] = {
            "name": material.get("name") or material_id,
            "rgba": rgba,
        }

    bindings: dict[str, list[dict[str, object]]] = {}
    for instance in root.findall(".//{*}instance_geometry"):
        geometry_id = _collada_id(instance.get("url"))
        if not geometry_id:
            continue
        bound = []
        for item in instance.findall(".//{*}bind_material/{*}technique_common/{*}instance_material"):
            material = materials.get(_collada_id(item.get("target")) or "")
            if material is not None and material not in bound:
                bound.append(material)
        if bound:
            bindings.setdefault(geometry_id, []).extend(
                material for material in bound if material not in bindings.get(geometry_id, [])
            )

    # A source geometry with a unique material is safe to map to one emitted
    # mesh.  Ambiguous multi-slot geometries need a primitive-level split and
    # are intentionally not assigned a misleading colour.
    return {geometry_id: bound[0] for geometry_id, bound in bindings.items() if len(bound) == 1}


def source_path(package_uri: str, franka_root: Path) -> Path | None:
    prefix = "package://franka_description/meshes/"
    if not package_uri.startswith(prefix):
        return None

    rel = package_uri.removeprefix(prefix)
    return franka_root / "meshes" / rel


def target_path(package_uri: str, suffix: str) -> Path | None:
    prefix = "package://franka_description/meshes/"
    if not package_uri.startswith(prefix):
        return None

    rel = package_uri.removeprefix(prefix)
    parts = rel.split("/")

    if len(parts) == 3:
        dirname, kind, filename = parts
    elif len(parts) == 4:
        dirname = f"{parts[0]}/{parts[1]}"
        kind, filename = parts[2], parts[3]
    else:
        return None

    component = COMPONENT_MAP.get(dirname)
    if component is None:
        return None

    out_name = Path(filename).stem + suffix
    return MODELS / component / kind / out_name


def load_mesh_parts(src: Path) -> list[trimesh.Trimesh]:
    """Load a DAE and return transformed mesh instances without concatenation.

    ``Scene.dump()`` copies every scene geometry instance and applies its scene
    transform to the copy. This is important for COLLADA files where geometry
    placement is represented by the scene graph rather than baked vertices.
    """
    scene = trimesh.load_scene(src)

    meshes = [
        geometry
        for geometry in scene.dump()
        if isinstance(geometry, trimesh.Trimesh) and not geometry.is_empty
    ]

    if not meshes:
        raise ValueError(f"no triangle mesh geometry found in {src}")

    return meshes


def normalize_mesh_parts(
    meshes: list[trimesh.Trimesh], unit_scale: float
) -> list[trimesh.Trimesh]:
    """Scale scene parts to meters and discard sub-threshold geometry.

    Tiny COLLADA helper geometry becomes separate OBJ assets after scene
    splitting.  Keeping it would violate the repository's per-asset minimum
    extent check while adding no visible geometry at simulation scale.
    """
    normalized: list[trimesh.Trimesh] = []
    for mesh in meshes:
        if unit_scale != 1.0:
            mesh.apply_scale(unit_scale)
        if mesh.extents.max() >= MIN_OUTPUT_EXTENT:
            normalized.append(mesh)
    if not normalized:
        raise ValueError("no mesh parts remain after meter-scale normalization")
    return normalized


def apply_output_policy(
    package_uri: str, meshes: list[trimesh.Trimesh]
) -> list[trimesh.Trimesh]:
    """Apply explicit per-source output granularity after normalization."""
    if package_uri not in MERGED_VISUAL_SOURCES:
        return meshes
    return [trimesh.util.concatenate(meshes)]


def output_paths(base: Path, count: int) -> list[Path]:
    """Return output paths for one source DAE."""
    if count <= 0:
        raise ValueError("mesh count must be positive")

    if count == 1:
        return [base]

    return [base.with_name(f"{base.stem}_{index}{base.suffix}") for index in range(count)]


def previous_output_paths(base: Path) -> set[Path]:
    """Return all files that could have been generated for ``base``.

    This includes the historical single-file output and the new numbered
    multi-mesh outputs. It deliberately does not match unrelated files whose
    stem merely starts with the same text.
    """
    paths = set()

    if base.exists():
        paths.add(base)

    pattern = re.compile(rf"^{re.escape(base.stem)}_[0-9]+{re.escape(base.suffix)}$")
    for path in base.parent.glob(f"{base.stem}_*{base.suffix}"):
        if pattern.fullmatch(path.name):
            paths.add(path)

    return paths


def remove_stale_outputs(base: Path, expected: set[Path]) -> None:
    """Remove obsolete outputs from an earlier conversion layout."""
    for path in sorted(previous_output_paths(base) - expected):
        path.unlink()
        print(f"removed stale {path.relative_to(REPO_ROOT)}")


def material_metadata(mesh: trimesh.Trimesh) -> dict[str, object] | None:
    """Return a portable COLLADA material record exposed by trimesh.

    OBJ is used only as the geometry carrier in this repository.  Its source
    material must consequently be captured in the conversion manifest so the
    MJCF builder can recreate the appearance explicitly.  ``SimpleMaterial``
    exposes ``diffuse`` while newer trimesh PBR materials expose
    ``baseColorFactor``; both are normalised to an RGBA tuple in [0, 1].
    """
    visual = getattr(mesh, "visual", None)
    material = getattr(visual, "material", None)
    if material is None:
        return None

    name = getattr(material, "name", None)
    raw_rgba = getattr(material, "baseColorFactor", None)
    if raw_rgba is None:
        raw_rgba = getattr(material, "diffuse", None)
    if raw_rgba is None:
        return {"name": str(name)} if name else None

    try:
        rgba = [float(value) for value in raw_rgba]
    except (TypeError, ValueError):
        return {"name": str(name)} if name else None
    if len(rgba) == 3:
        rgba.append(1.0)
    if len(rgba) != 4:
        return {"name": str(name)} if name else None
    # trimesh SimpleMaterial stores byte colours, while PBR factors are unit
    # floats.  Keep the manifest independent of that implementation detail.
    if max(rgba) > 1.0:
        rgba = [value / 255.0 for value in rgba]
    rgba = [max(0.0, min(1.0, value)) for value in rgba]
    record: dict[str, object] = {"rgba": rgba}
    if name:
        record["name"] = str(name)
    return record


def mesh_record(
    path: Path,
    mesh: trimesh.Trimesh,
    collada_material: dict[str, dict[str, object]] | None = None,
) -> dict:
    record = {
        "path": path.relative_to(REPO_ROOT).as_posix(),
        "sha256": sha256(path),
        "n_vertices": int(len(mesh.vertices)),
        "n_faces": int(len(mesh.faces)),
        "bounds": mesh.bounds.tolist(),
    }

    material = material_metadata(mesh)
    source_geometry = mesh.metadata.get("name")
    if collada_material and source_geometry is not None:
        material = collada_material.get(str(source_geometry), material)
    if material is not None:
        record["material"] = material

    source_node = mesh.metadata.get("node")
    if source_geometry is not None:
        record["source_geometry"] = str(source_geometry)
    if source_node is not None:
        record["source_node"] = str(source_node)

    return record


def existing_mesh_record(path: Path) -> dict:
    """Return the required per-output record for an already-exported OBJ."""
    mesh = trimesh.load(path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh) or mesh.is_empty:
        raise ValueError(f"invalid converted OBJ: {path}")
    return mesh_record(path, mesh)


def preserve_source_metadata(
    outputs: list[dict], previous_record: dict | None
) -> list[dict]:
    """Keep DAE-only provenance when refreshing an up-to-date manifest."""
    previous_outputs = previous_record.get("outputs", []) if previous_record else []
    for index, output in enumerate(outputs):
        if index >= len(previous_outputs) or not isinstance(previous_outputs[index], dict):
            continue
        for key in ("material", "source_geometry", "source_node"):
            if key not in output and key in previous_outputs[index]:
                output[key] = previous_outputs[index][key]
    return outputs


def convert_all(franka_root: Path, force: bool = False) -> dict:
    manifest_path = REPO_ROOT / "source" / "asset_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text())
    conversion_path = GENERATED / "asset_conversion.json"
    try:
        previous_records = json.loads(conversion_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        previous_records = {}
    if not isinstance(previous_records, dict):
        previous_records = {}
    records: dict[str, dict] = {}

    for asset in manifest["assets"]:
        uri = asset["source"]
        if not uri.endswith(".dae"):
            continue

        src = source_path(uri, franka_root)
        base_dst = target_path(uri, ".obj")
        if src is None or base_dst is None:
            print(f"skip (unmapped): {uri}")
            continue

        if not src.is_file():
            raise FileNotFoundError(f"source mesh does not exist: {src}")

        base_dst.parent.mkdir(parents=True, exist_ok=True)

        unit_scale = dae_unit_scale(src)
        collada_material = collada_materials(src)
        source_meshes = normalize_mesh_parts(load_mesh_parts(src), unit_scale)
        meshes = apply_output_policy(uri, source_meshes)
        destinations = output_paths(base_dst, len(meshes))
        expected = set(destinations)

        # If the DAE used to produce a single merged OBJ and now produces
        # numbered parts (or vice versa), remove the obsolete layout.
        remove_stale_outputs(base_dst, expected)

        source_hash = sha256(src)
        all_exist = all(path.is_file() for path in destinations)

        if all_exist and not force:
            outputs = preserve_source_metadata(
                [
                    mesh_record(path, mesh, collada_material)
                    for path, mesh in zip(destinations, meshes, strict=True)
                ],
                previous_records.get(uri),
            )
            records[uri] = {
                "status": "up-to-date",
                "tool": f"trimesh {trimesh.__version__}",
                "unit_scale": unit_scale,
                "input_sha256": source_hash,
                "source_mesh_count": len(source_meshes),
                "mesh_count": len(meshes),
                "n_vertices": sum(item["n_vertices"] for item in outputs),
                "n_faces": sum(item["n_faces"] for item in outputs),
                "outputs": outputs,
            }
            continue

        output_records = []
        for mesh, dst in zip(meshes, destinations, strict=True):
            # OBJ remains geometry-only.  COLLADA appearance is retained in
            # the manifest and emitted as MJCF assets by the model builder.
            mesh.export(dst, include_texture=False)
            output_records.append(mesh_record(dst, mesh, collada_material))
            print(f"converted {dst.relative_to(REPO_ROOT)}")

        records[uri] = {
            "status": "converted",
            "tool": f"trimesh {trimesh.__version__}",
            "unit_scale": unit_scale,
            "input_sha256": source_hash,
            "source_mesh_count": len(source_meshes),
            "mesh_count": len(meshes),
            "n_vertices": sum(item["n_vertices"] for item in output_records),
            "n_faces": sum(item["n_faces"] for item in output_records),
            "outputs": output_records,
        }

    return records


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--franka-root",
        type=Path,
        default=(
            Path(os.environ["FRANKA_DESCRIPTION_ROOT"])
            if "FRANKA_DESCRIPTION_ROOT" in os.environ
            else None
        ),
        help="fixed franka_description checkout (or FRANKA_DESCRIPTION_ROOT)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="reconvert existing assets",
    )
    args = parser.parse_args()

    if args.franka_root is None:
        parser.error("pass --franka-root or set FRANKA_DESCRIPTION_ROOT")

    GENERATED.mkdir(parents=True, exist_ok=True)

    records = convert_all(args.franka_root, force=args.force)
    out = GENERATED / "asset_conversion.json"
    out.write_text(json.dumps(records, indent=2, sort_keys=True) + "\n")
    print(f"wrote {out} ({len(records)} visual assets)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
