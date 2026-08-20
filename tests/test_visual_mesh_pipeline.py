"""Unit coverage for split visual-mesh conversion and MJCF emission."""

from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from helpers import REPO_ROOT

sys.path.insert(0, str(REPO_ROOT / "tools"))

import convert_visual_meshes as converter  # noqa: E402
from model_builder import geometry  # noqa: E402


def test_load_mesh_parts_bakes_scene_node_transform(monkeypatch):
    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    scene = trimesh.Scene()
    scene.add_geometry(mesh, transform=trimesh.transformations.translation_matrix((2, 0, 0)))
    monkeypatch.setattr(converter.trimesh, "load_scene", lambda _: scene)

    parts = converter.load_mesh_parts(REPO_ROOT / "unused.dae")

    assert len(parts) == 1
    assert np.allclose(parts[0].bounds, [[1.5, -0.5, -0.5], [2.5, 0.5, 0.5]])


def test_output_layout_and_stale_cleanup(tmp_path, monkeypatch):
    monkeypatch.setattr(converter, "REPO_ROOT", tmp_path)
    base = tmp_path / "link6.obj"
    base.write_text("old merged mesh")
    stale = tmp_path / "link6_12.obj"
    stale.write_text("old part")
    unrelated = tmp_path / "link6_cover.obj"
    unrelated.write_text("keep")

    outputs = converter.output_paths(base, 2)
    converter.remove_stale_outputs(base, set(outputs))

    assert outputs == [tmp_path / "link6_0.obj", tmp_path / "link6_1.obj"]
    assert not base.exists()
    assert not stale.exists()
    assert unrelated.exists()


def test_mesh_record_has_per_output_statistics(tmp_path):
    original_root = converter.REPO_ROOT
    converter.REPO_ROOT = tmp_path
    path = tmp_path / "part.obj"
    mesh = trimesh.creation.box(extents=(2, 4, 6))
    mesh.export(path)

    try:
        record = converter.mesh_record(path, mesh)
    finally:
        converter.REPO_ROOT = original_root

    assert record["path"].endswith("part.obj")
    assert len(record["sha256"]) == 64
    assert record["n_vertices"] == len(mesh.vertices)
    assert record["n_faces"] == len(mesh.faces)
    assert record["bounds"] == [[-1.0, -2.0, -3.0], [1.0, 2.0, 3.0]]


def test_dae_unit_scale_reads_declared_meter_value(tmp_path):
    dae = tmp_path / "mesh.dae"
    dae.write_text('<COLLADA><asset><unit meter="0.001"/></asset></COLLADA>')

    assert converter.dae_unit_scale(dae) == 0.001


def test_normalize_mesh_parts_scales_and_filters_tiny_geometry():
    retained = trimesh.creation.box(extents=(0.2, 0.2, 0.2))
    discarded = trimesh.creation.box(extents=(0.01, 0.01, 0.01))

    parts = converter.normalize_mesh_parts([retained, discarded], 0.001)

    assert parts == [retained]
    assert np.allclose(retained.extents, [0.0002, 0.0002, 0.0002])


def test_tmr_output_policy_merges_materialless_cad_parts():
    parts = [
        trimesh.creation.box(extents=(1, 1, 1)),
        trimesh.creation.box(extents=(2, 2, 2)),
    ]
    uri = "package://franka_description/meshes/robots/tmrv0_2/visual/tmrv0_2.dae"

    merged = converter.apply_output_policy(uri, parts)

    assert len(merged) == 1
    assert len(merged[0].faces) == sum(len(part.faces) for part in parts)
    assert converter.apply_output_policy("package://example/visual/other.dae", parts) is parts


def test_up_to_date_records_preserve_dae_provenance():
    outputs = [{"path": "models/franka_fr3/assets/visual/link0_0.obj", "sha256": "new"}]
    previous = {
        "outputs": [
            {
                "path": "models/franka_fr3/assets/visual/link0_0.obj",
                "sha256": "old",
                "source_geometry": "link0_mesh",
                "source_node": "link0_node",
            }
        ]
    }

    assert converter.preserve_source_metadata(outputs, previous) == [
        {
            "path": "models/franka_fr3/assets/visual/link0_0.obj",
            "sha256": "new",
            "source_geometry": "link0_mesh",
            "source_node": "link0_node",
        }
    ]


def test_visual_manifest_drives_split_assets_and_geoms(tmp_path, monkeypatch):
    monkeypatch.setattr(geometry, "REPO_ROOT", tmp_path)
    output_dir = tmp_path / "models" / "franka_fr3" / "assets" / "visual"
    output_dir.mkdir(parents=True)
    for index in range(2):
        (output_dir / f"link6_{index}.obj").write_text("v 0 0 0\n")
    uri = "package://franka_description/meshes/robots/fr3v2_1/visual/link6.dae"
    manifest = tmp_path / "asset_conversion.json"
    manifest.write_text(
        json.dumps(
            {
                uri: {
                    "status": "converted",
                    "mesh_count": 2,
                    "outputs": [
                        {"path": "models/franka_fr3/assets/visual/link6_0.obj"},
                        {"path": "models/franka_fr3/assets/visual/link6_1.obj"},
                    ],
                }
            }
        )
    )

    conversion = geometry.load_visual_conversion(manifest)
    assert geometry.mesh_assets(uri, conversion) == [
        ("link6_visual_0", "franka_fr3/assets/visual/link6_0.obj"),
        ("link6_visual_1", "franka_fr3/assets/visual/link6_1.obj"),
    ]
    collision_uri = "package://franka_description/meshes/robots/fr3v2_1/collision/link6.stl"
    assert geometry.mesh_assets(collision_uri, conversion) == [
        ("link6_collision", "franka_fr3/assets/collision/link6.stl")
    ]

    visual = ET.fromstring(
        f'''<visual><origin xyz="1 2 3" rpy="0 0 0"/><geometry><mesh filename="{uri}"/></geometry></visual>'''
    )
    geoms = geometry.visual(visual, conversion)
    assert [geom.get("mesh") for geom in geoms] == ["link6_visual_0", "link6_visual_1"]
    assert all(geom.get("pos") == "1 2 3" for geom in geoms)
    assert all(geom.get("class") == "visual" for geom in geoms)
