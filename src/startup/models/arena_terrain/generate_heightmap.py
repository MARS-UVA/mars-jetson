"""Regenerate the crater PNG and aligned terrain mesh with the standard library."""
from pathlib import Path
from bisect import bisect_left, bisect_right
import argparse
import json
import math
import re
import xml.etree.ElementTree as ET
import struct
import zlib

RESOLUTION = 513
MESH_MAX_DEPTH = 8  # Finest cells: ~2.7 x 2 cm, matching the former 257 x 257 grid.
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


def elevation(x, y, craters=CRATERS):
    z = 0.0
    for cx, cy, radius, depth in craters:
        r = math.hypot(x - cx, y - cy) / radius
        if r < 1.0:
            # Broader deep bowls with steeper sides and a smooth join at the rim.
            z -= depth * (1.0 - r * r) ** 1.5
    return z


def chunk(kind, data):
    return (struct.pack(">I", len(data)) + kind + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff))


def build_mesh(craters=CRATERS):
    """Refine crater cells and stitch their edges to the coarse, flat floor."""
    # Doubled integer coordinates also represent the centers of the finest cells.
    extent = 2 << MESH_MAX_DEPTH
    leaves = []

    def subdivide(x, y, size):
        left, right = x * WIDTH / extent, (x + size) * WIDTH / extent
        bottom = y * LENGTH / extent - LENGTH / 2
        top = (y + size) * LENGTH / extent - LENGTH / 2
        intersects = any(
            (cx - min(max(cx, left), right)) ** 2
            + (cy - min(max(cy, bottom), top)) ** 2 < radius ** 2
            for cx, cy, radius, _ in craters
        )
        if size > 2 and intersects:
            half = size // 2
            for dx, dy in ((0, 0), (half, 0), (half, half), (0, half)):
                subdivide(x + dx, y + dy, half)
        else:
            leaves.append((x, y, size))

    subdivide(0, 0, extent)

    # Shared boundary vertices prevent cracks where coarse and fine cells meet.
    horizontal, vertical = {}, {}
    for x, y, size in leaves:
        for px, py in ((x, y), (x + size, y), (x + size, y + size), (x, y + size)):
            horizontal.setdefault(py, set()).add(px)
            vertical.setdefault(px, set()).add(py)
    horizontal = {y: sorted(xs) for y, xs in horizontal.items()}
    vertical = {x: sorted(ys) for x, ys in vertical.items()}
    vertices, faces, indices = [], [], {}

    def vertex(px, py):
        key = px, py
        if key not in indices:
            wx, wy = px * WIDTH / extent, py * LENGTH / extent - LENGTH / 2
            indices[key] = len(vertices)
            vertices.append((wx - WIDTH / 2, wy, elevation(wx, wy, craters)))
        return indices[key]

    def side(values, low, high, reverse=False):
        if reverse:
            return reversed(values[bisect_right(values, low):bisect_right(values, high)])
        return values[bisect_left(values, low):bisect_left(values, high)]

    for x, y, size in leaves:
        right, top = x + size, y + size
        # Counterclockwise perimeter, with each corner included exactly once.
        boundary = [vertex(px, y) for px in side(horizontal[y], x, right)]
        boundary += [vertex(right, py) for py in side(vertical[right], y, top)]
        boundary += [vertex(px, top) for px in side(horizontal[top], x, right, True)]
        boundary += [vertex(x, py) for py in side(vertical[x], y, top, True)]
        if len(boundary) == 4:
            a, b, c, d = boundary
            faces.extend(((a, b, c), (a, c, d)))
        else:
            center = vertex(x + size // 2, y + size // 2)
            faces.extend((center, a, b) for a, b in zip(boundary, boundary[1:] + boundary[:1]))
    return vertices, faces


def write_mesh(output_dir=None, craters=CRATERS):
    # Local mesh bounds are centred on the model's world pose (3.44, 0, 0).
    # Unlike Ogre heightmaps, meshes preserve rectangular dimensions and poses.
    target = (Path(output_dir) if output_dir is not None else Path(__file__).parent) / 'meshes/terrain.obj'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.with_suffix('.mtl').write_text(
        'newmtl soil\nKa 0.55 0.55 0.55\nKd 0.65 0.65 0.65\n')
    vertices, faces = build_mesh(craters)
    with target.open('w') as mesh:
        mesh.write('# Arena floor: 6.88 m x 5 m, upward-facing triangles\n')
        mesh.write('mtllib terrain.mtl\no terrain\nusemtl soil\n')
        for x, y, z in vertices:
            mesh.write(f'v {x:.6f} {y:.6f} {z:.6f}\n')
        # DART requires one normal per vertex for static triangle meshes.
        epsilon = 0.0001
        for x, y, _ in vertices:
            x += WIDTH / 2
            nx = -(elevation(x + epsilon, y, craters) - elevation(x - epsilon, y, craters)) / (2 * epsilon)
            ny = -(elevation(x, y + epsilon, craters) - elevation(x, y - epsilon, craters)) / (2 * epsilon)
            length = math.sqrt(nx * nx + ny * ny + 1)
            mesh.write(f'vn {nx / length:.6f} {ny / length:.6f} {1 / length:.6f}\n')
        for triangle in faces:
            mesh.write('f ' + ' '.join(f'{i + 1}//{i + 1}' for i in triangle) + '\n')
    print(f'Wrote {target}: {len(vertices)} vertices, {len(faces)} triangles')


def load_layout(path):
    """Read [world x, world y, radius, depth] entries in metres."""
    data = json.loads(Path(path).read_text())
    if not isinstance(data, list):
        raise ValueError('Layout must be a JSON array of [x, y, radius, depth] entries')
    craters = []
    for index, entry in enumerate(data):
        if (not isinstance(entry, list) or len(entry) != 4
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in entry)):
            raise ValueError(f'Crater {index}: expected four finite numbers')
        x, y, radius, depth = entry
        if radius <= 0 or depth <= 0:
            raise ValueError(f'Crater {index}: radius and depth must be positive')
        if not (radius <= x <= WIDTH - radius
                and -LENGTH / 2 + radius <= y <= LENGTH / 2 - radius):
            raise ValueError(f'Crater {index}: crater must fit inside the arena')
        craters.append(tuple(entry))
    return tuple(craters)


def write_model_files(output_dir):
    """Reuse the terrain physics and material settings with independent mesh URIs."""
    source = Path(__file__).parent
    model = ET.parse(source / 'model.sdf')
    model.find('model').set('name', output_dir.name)
    for uri in model.findall('.//mesh/uri'):
        uri.text = f'model://{output_dir.name}/meshes/terrain.obj'
    model.write(output_dir / 'model.sdf', encoding='utf-8', xml_declaration=True)
    config = ET.parse(source / 'model.config')
    config.find('name').text = output_dir.name
    config.find('description').text = 'Generated arena terrain; crater coordinates are stored in layout.json.'
    config.write(output_dir / 'model.config', encoding='utf-8', xml_declaration=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', type=Path, help='JSON array of [x, y, radius, depth] in metres')
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).parent,
                        help='Model directory; its folder name becomes the Gazebo model name')
    args = parser.parse_args(argv)
    output_dir = args.output_dir.resolve()
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*', output_dir.name):
        parser.error('Output directory name must contain only letters, numbers, underscores, dots or hyphens')
    try:
        craters = load_layout(args.layout) if args.layout else CRATERS
    except (OSError, ValueError) as error:
        parser.error(str(error))
    output_dir.mkdir(parents=True, exist_ok=True)
    # Overlapping bowls add their depths; scale the PNG to avoid byte overflow.
    png_depth = max(DEPTH, max((-elevation(col * WIDTH / (RESOLUTION - 1),
                        LENGTH / 2 - row * LENGTH / (RESOLUTION - 1), craters)
                        for row in range(RESOLUTION) for col in range(RESOLUTION)), default=0))
    pixels = bytearray()
    for row in range(RESOLUTION):
        y = LENGTH / 2 - row * LENGTH / (RESOLUTION - 1)
        pixels.append(0)  # PNG scanline filter: none
        for col in range(RESOLUTION):
            x = col * WIDTH / (RESOLUTION - 1)
            pixels.append(round(255 * (elevation(x, y, craters) + png_depth) / png_depth))
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", RESOLUTION, RESOLUTION, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(pixels), 9))
    png += chunk(b"IEND", b"")
    target = output_dir / "materials/textures/craters.png"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(png)
    print(f"Wrote {target}")
    write_mesh(output_dir, craters)
    write_model_files(output_dir)
    (output_dir / 'layout.json').write_text(json.dumps(craters, indent=2) + '\n')


if __name__ == "__main__":
    main()
