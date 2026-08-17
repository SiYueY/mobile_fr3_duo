"""Minimal dependency-free PNG writer for RGB(A) numpy arrays."""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np


def _chunk(tag: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + tag
        + payload
        + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
    )


def write_png(path: Path | str, rgb: np.ndarray) -> None:
    """Write an (H, W, 3) uint8 array as an 8-bit RGB PNG."""
    rgb = np.asarray(rgb)
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"expected (H, W, 3) uint8, got {rgb.shape}")
    h, w = rgb.shape[:2]
    raw = b"".join(
        b"\x00" + rgb[y, :, :].tobytes() for y in range(h)
    )
    png = (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(raw, 6))
        + _chunk(b"IEND", b"")
    )
    Path(path).write_bytes(png)
