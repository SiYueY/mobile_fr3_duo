#!/usr/bin/env python3
"""Convert metre-scale binary STL visual assets to deduplicated OBJ meshes."""

from __future__ import annotations

import argparse
import hashlib
import struct
from pathlib import Path

import numpy as np


def convert(source: Path, destination: Path) -> tuple[int, int, str]:
    raw = source.read_bytes()
    triangles = struct.unpack("<I", raw[80:84])[0]
    if len(raw) != 84 + 50 * triangles:
        raise ValueError(f"{source} is not a supported binary STL")
    vertices = np.ndarray(
        (triangles, 3, 3), dtype="<f4", buffer=raw, offset=96, strides=(50, 12, 4)
    ).copy()
    unique, inverse = np.unique(vertices.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as output:
        output.write(f"# Converted from {source.name}; source coordinates are metres.\n")
        for vertex in unique:
            output.write("v {:.9g} {:.9g} {:.9g}\n".format(*vertex))
        for face in faces:
            output.write(f"f {face[0] + 1} {face[1] + 1} {face[2] + 1}\n")
    return len(unique), len(faces), hashlib.sha256(destination.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    vertices, faces, digest = convert(args.source, args.destination)
    print(f"vertices={vertices} faces={faces} sha256={digest}")


if __name__ == "__main__":
    main()
