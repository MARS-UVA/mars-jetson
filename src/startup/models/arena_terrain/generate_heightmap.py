"""Regenerate the crater PNG and aligned terrain mesh with the standard library."""
from pathlib import Path
import math
import struct
import zlib

RESOLUTION = 513
MESH_RESOLUTION = 257  # Resolve the diagram's smallest, 35 cm diameter bowls.
WIDTH, LENGTH, DEPTH = 6.88, 5.0, 0.25
# World x, world y, radius, depth (metres), digitized from the arena diagram.
# Diagram coordinates span x=0..6.88, y=0..5; world y is shifted by -2.5 m.
# Convert all seven rock markers and all three crater markers into bowls.
# Legend midpoints: rocks become 35 cm diameter, craters remain 45 cm diameter.
# Depths are chosen for simulation: 18 cm for small bowls, 25 cm for large bowls.
CRATERS = (
    (0.51, 1.93, 0.225, 0.25),   # Upper-left crater.
    (6.13, 1.80, 0.225, 0.25),   # Upper-right crater.
    (3.25, -1.32, 0.225, 0.25),  # Crater beside the starting/construction zones.
    (1.92, 1.87, 0.175, 0.18),   # Rock near the top of the excavation zone.
    (3.47, 1.98, 0.175, 0.18),   # Rock near the top of the obstacle zone.
    (0.35, 0.30, 0.175, 0.18),   # Rock along the left wall.
    (2.80, 0.27, 0.175, 0.18),   # Rock just right of the zone divider.
    (4.58, 0.16, 0.175, 0.18),   # Rock right of the central column.
    (6.44, 0.63, 0.175, 0.18),   # Rock along the right wall.
    (2.28, -1.73, 0.175, 0.18),  # Rock just outside the starting zone.
)


def elevation(x, y):
    z = 0.0
    for cx, cy, radius, depth in CRATERS:
        r = math.hypot(x - cx, y - cy) / radius
        if r < 1.0:
            # Broader deep bowls with steeper sides and a smooth join at the rim.
            z -= depth * (1.0 - r * r) ** 1.5
    return z


def chunk(kind, data):
    return (struct.pack(">I", len(data)) + kind + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff))


def write_mesh():
    # Local mesh bounds are centred on the model's world pose (3.44, 0, 0).
    # Unlike Ogre heightmaps, meshes preserve rectangular dimensions and poses.
    target = Path(__file__).parent / 'meshes/terrain.obj'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.with_suffix('.mtl').write_text(
        'newmtl soil\nKa 0.55 0.55 0.55\nKd 0.65 0.65 0.65\n')
    with target.open('w') as mesh:
        mesh.write('# Arena floor: 6.88 m x 5 m, upward-facing triangles\n')
        mesh.write('mtllib terrain.mtl\no terrain\nusemtl soil\n')
        for row in range(MESH_RESOLUTION):
            y = -LENGTH / 2 + row * LENGTH / (MESH_RESOLUTION - 1)
            for col in range(MESH_RESOLUTION):
                x = col * WIDTH / (MESH_RESOLUTION - 1)
                mesh.write(f'v {x - WIDTH / 2:.6f} {y:.6f} {elevation(x, y):.6f}\n')
        # DART requires one normal per vertex for static triangle meshes.
        epsilon = 0.0001
        for row in range(MESH_RESOLUTION):
            y = -LENGTH / 2 + row * LENGTH / (MESH_RESOLUTION - 1)
            for col in range(MESH_RESOLUTION):
                x = col * WIDTH / (MESH_RESOLUTION - 1)
                nx = -(elevation(x + epsilon, y) - elevation(x - epsilon, y)) / (2 * epsilon)
                ny = -(elevation(x, y + epsilon) - elevation(x, y - epsilon)) / (2 * epsilon)
                length = math.sqrt(nx * nx + ny * ny + 1)
                mesh.write(f'vn {nx / length:.6f} {ny / length:.6f} {1 / length:.6f}\n')
        for row in range(MESH_RESOLUTION - 1):
            for col in range(MESH_RESOLUTION - 1):
                a = row * MESH_RESOLUTION + col + 1
                b, c, d = a + 1, a + MESH_RESOLUTION, a + MESH_RESOLUTION + 1
                mesh.write(f'f {a}//{a} {b}//{b} {d}//{d}\n'
                           f'f {a}//{a} {d}//{d} {c}//{c}\n')
    print(f'Wrote {target}')


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
    write_mesh()


if __name__ == "__main__":
    main()
