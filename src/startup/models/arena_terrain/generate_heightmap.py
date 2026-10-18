"""Regenerate the deterministic crater PNG using only Python's standard library."""
from pathlib import Path
import math
import struct
import zlib

RESOLUTION = 513
WIDTH, LENGTH, DEPTH = 6.88, 5.0, 0.25
# World x, world y, radius, depth (metres). Starting/construction zones stay flat.
CRATERS = ((3.6, 0.8, 0.65, 0.25), (5.6, 0.8, 0.5, 0.18))


def elevation(x, y):
    z = 0.0
    for cx, cy, radius, depth in CRATERS:
        r = math.hypot(x - cx, y - cy) / radius
        if r < 1.0:
            z -= depth * (1.0 - r * r) ** 2
    return z


def chunk(kind, data):
    return (struct.pack(">I", len(data)) + kind + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff))


def main():
    pixels = bytearray()
    for row in range(RESOLUTION):
        y = LENGTH / 2 - row * LENGTH / (RESOLUTION - 1)
        pixels.append(0)  # PNG scanline filter: none
        for col in range(RESOLUTION):
            x = col * WIDTH / (RESOLUTION - 1)
            pixels.append(round(255 * (elevation(x, y) + DEPTH) / DEPTH))
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", RESOLUTION, RESOLUTION, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(pixels), 9))
    png += chunk(b"IEND", b"")
    target = Path(__file__).parent / "materials/textures/craters.png"
    target.write_bytes(png)
    print(f"Wrote {target}")


if __name__ == "__main__":
    main()
