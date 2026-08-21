#!/usr/bin/env python3
"""Import frozen official D435 and Franka wrist-mount visual assets.

The input files are intentionally supplied explicitly: they are downloaded from
the upstream release page during source preparation and their hashes are stored
in the generated provenance files.  Collada is converted directly because
MuJoCo accepts OBJ/STL but not DAE.
"""

from __future__ import annotations

import argparse
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def dae_to_obj(source: Path, destination: Path) -> tuple[int, int]:
    """Write the D435 scene's triangle geometry to a single metre-scale OBJ."""
    root = ET.parse(source).getroot()
    vertices = 0
    faces = 0
    with destination.open("w", encoding="utf-8") as output:
        output.write("# Official Intel RealSense D435 visual mesh\n")
        output.write("# Source uses metres and Z-up; no scale or axis conversion.\n")
        offset = 0
        for geometry in root.findall(".//c:geometry", NS):
            mesh = geometry.find("c:mesh", NS)
            if mesh is None:
                continue
            sources = {}
            for source_node in mesh.findall("c:source", NS):
                array = source_node.find("c:float_array", NS)
                accessor = source_node.find(".//c:accessor", NS)
                if array is None or accessor is None:
                    continue
                stride = int(accessor.attrib.get("stride", "1"))
                values = [float(value) for value in (array.text or "").split()]
                sources[f"#{source_node.attrib['id']}"] = (values, stride)
            vertex_sources = {}
            for vertex_node in mesh.findall("c:vertices", NS):
                position = vertex_node.find("c:input[@semantic='POSITION']", NS)
                if position is not None:
                    vertex_sources[f"#{vertex_node.attrib['id']}"] = position.attrib["source"]
            for triangles in mesh.findall("c:triangles", NS):
                inputs = triangles.findall("c:input", NS)
                vertex_input = next((item for item in inputs if item.attrib["semantic"] == "VERTEX"), None)
                if vertex_input is None:
                    continue
                input_offset = int(vertex_input.attrib.get("offset", "0"))
                stride = max(int(item.attrib.get("offset", "0")) for item in inputs) + 1
                position_source = vertex_sources[vertex_input.attrib["source"]]
                values, position_stride = sources[position_source]
                output.write(f"o {geometry.attrib.get('name', geometry.attrib['id'])}\n")
                for index in range(0, len(values), position_stride):
                    output.write("v {:.9g} {:.9g} {:.9g}\n".format(*values[index : index + 3]))
                indices = [int(value) for value in (triangles.findtext("c:p", default="", namespaces=NS)).split()]
                for index in range(0, len(indices), stride * 3):
                    triangle = [indices[index + input_offset + stride * corner] + 1 + offset for corner in range(3)]
                    output.write("f {} {} {}\n".format(*triangle))
                    faces += 1
                offset += len(values) // position_stride
                vertices += len(values) // position_stride
    return vertices, faces


def stl_to_obj(source: Path, destination: Path, voxel_mm: float = 0.15) -> tuple[int, int]:
    """Convert the official binary STL to a MuJoCo-sized OBJ.

    The original printable STL has 295k facets (above MuJoCo's per-mesh limit).
    Vertex clustering at 0.15 mm preserves its mechanical dimensions while
    reducing tessellation only; no primitive replacement is introduced.
    """
    raw = source.read_bytes()
    triangles = struct.unpack("<I", raw[80:84])[0]
    expected = 84 + triangles * 50
    if len(raw) != expected:
        raise ValueError(f"unsupported STL layout: expected {expected}, got {len(raw)}")
    vertices = np.ndarray(
        (triangles, 3, 3), dtype="<f4", buffer=raw, offset=96, strides=(50, 12, 4)
    ).copy()
    # STL is in millimetres.  Quantise in its source units then export metres.
    cluster_ids = np.rint(vertices.reshape(-1, 3) / voxel_mm).astype(np.int64)
    unique, inverse = np.unique(cluster_ids, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), 3), dtype=np.float64)
    counts = np.bincount(inverse, minlength=len(unique))
    np.add.at(sums, inverse, vertices.reshape(-1, 3))
    clustered = sums / counts[:, None]
    faces = inverse.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    canonical = np.sort(faces, axis=1)
    _, keep = np.unique(canonical, axis=0, return_index=True)
    faces = faces[np.sort(keep)]
    with destination.open("w", encoding="utf-8") as output:
        output.write("# Franka official RealSense D435 Wrist-Cam Mount\n")
        output.write("# STL millimetres converted to metres; 0.15 mm vertex-cluster tessellation reduction.\n")
        for vertex in clustered:
            output.write("v {:.9g} {:.9g} {:.9g}\n".format(*(vertex * 0.001)))
        for face in faces:
            output.write(f"f {face[0] + 1} {face[1] + 1} {face[2] + 1}\n")
    return len(clustered), len(faces)


def reduce_obj(path: Path, voxel_m: float = 0.00005) -> tuple[int, int]:
    """Vertex-cluster an OBJ whose only records are vertices and triangles."""
    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if fields[:1] == ["v"]:
            vertices.append([float(value) for value in fields[1:4]])
        elif fields[:1] == ["f"]:
            faces.append([int(value.split("/")[0]) - 1 for value in fields[1:4]])
    points = np.asarray(vertices)
    face_array = np.asarray(faces, dtype=np.int64)
    cluster_ids = np.rint(points / voxel_m).astype(np.int64)
    unique, inverse = np.unique(cluster_ids, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), 3), dtype=np.float64)
    counts = np.bincount(inverse, minlength=len(unique))
    np.add.at(sums, inverse, points)
    clustered = sums / counts[:, None]
    face_array = inverse[face_array]
    face_array = face_array[(face_array[:, 0] != face_array[:, 1]) & (face_array[:, 1] != face_array[:, 2]) & (face_array[:, 0] != face_array[:, 2])]
    canonical = np.sort(face_array, axis=1)
    _, keep = np.unique(canonical, axis=0, return_index=True)
    face_array = face_array[np.sort(keep)]
    with path.open("w", encoding="utf-8") as output:
        output.write("# Official Intel RealSense D435 visual mesh\n")
        output.write("# Source metres/Z-up; 0.05 mm vertex-cluster tessellation reduction for MuJoCo.\n")
        for vertex in clustered:
            output.write("v {:.9g} {:.9g} {:.9g}\n".format(*vertex))
        for face in face_array:
            output.write(f"f {face[0] + 1} {face[1] + 1} {face[2] + 1}\n")
    return len(clustered), len(face_array)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--d435-dae", type=Path, required=True)
    parser.add_argument("--mount-stl", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()

    d435_asset = args.output_root / "models/realsense_d435/assets/visual/d435.obj"
    mount_asset = args.output_root / "models/wrist_camera_mount/assets/visual/franka_d435_wrist_camera_mount.obj"
    d435_asset.parent.mkdir(parents=True, exist_ok=True)
    mount_asset.parent.mkdir(parents=True, exist_ok=True)
    vertices, faces = dae_to_obj(args.d435_dae, d435_asset)
    d435_vertices, d435_faces = reduce_obj(d435_asset)
    mount_vertices, mount_faces = stl_to_obj(args.mount_stl, mount_asset)
    print(f"d435 source: {vertices} vertices, {faces} triangles")
    print(f"d435 reduced: {d435_vertices} vertices, {d435_faces} triangles")
    print(f"mount: {mount_vertices} vertices, {mount_faces} triangles")


if __name__ == "__main__":
    main()
