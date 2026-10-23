"""Check diagram obstacle conversion, safe zones and generated mesh placement."""
from pathlib import Path
from collections import Counter
import math
import runpy
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
TERRAIN = ROOT / 'models/arena_terrain'
MODEL = runpy.run_path(str(TERRAIN / 'generate_heightmap.py'))


@pytest.fixture(scope='module')
def mesh():
    vertices, normals, faces = [], [], []
    for line in (TERRAIN / 'meshes/terrain.obj').read_text().splitlines():
        parts = line.split()
        if parts[0] == 'v':
            vertices.append(tuple(map(float, parts[1:])))
        elif parts[0] == 'vn':
            normals.append(tuple(map(float, parts[1:])))
        elif parts[0] == 'f':
            corners = [tuple(map(int, part.split('//'))) for part in parts[1:]]
            assert all(vertex == normal for vertex, normal in corners)
            faces.append(tuple(vertex - 1 for vertex, _ in corners))
    return vertices, normals, faces


def test_diagram_markers_have_correct_diameters_and_depths():
    craters = MODEL['CRATERS']
    assert len(craters) == 10
    assert sum(radius == 0.175 for _, _, radius, _ in craters) == 7
    assert sum(radius == 0.225 for _, _, radius, _ in craters) == 3
    for x, y, radius, depth in craters:
        assert MODEL['elevation'](x, y) == pytest.approx(-depth)
        assert MODEL['elevation'](x + radius, y) == pytest.approx(0, abs=1e-9)
        assert radius <= x <= MODEL['WIDTH'] - radius
        assert -MODEL['LENGTH'] / 2 + radius <= y <= MODEL['LENGTH'] / 2 - radius
        # Circle vs rectangle intersection for each protected zone.
        for left, right, bottom, top in [(0, 2, -2.5, -0.5), (3.88, 6.88, -2.5, -1)]:
            nearest_x, nearest_y = min(max(x, left), right), min(max(y, bottom), top)
            assert (x - nearest_x)**2 + (y - nearest_y)**2 > radius**2


def test_world_uses_only_fixed_terrain_and_matching_geometry():
    world = ET.parse(ROOT / 'world/arena_nasa_craters.world')
    assert not world.findall('.//population')
    assert not any(i.findtext('uri') == 'model://obstacle' for i in world.findall('.//include'))
    terrain = next(i for i in world.findall('.//include')
                   if i.findtext('uri') == 'model://arena_terrain')
    assert list(map(float, terrain.findtext('pose').split())) == [3.44, 0, 0, 0, 0, 0]
    model = ET.parse(TERRAIN / 'model.sdf')
    assert model.findtext('.//collision/geometry/mesh/uri') == model.findtext('.//visual/geometry/mesh/uri')


def test_generated_mesh_resolves_all_ten_craters_and_arena_bounds(mesh):
    vertices, _, faces = mesh
    # Keep the generated asset compact instead of subdividing the entire floor.
    assert len(vertices) < 10000
    assert len(faces) < 131072 / 10
    assert (TERRAIN / 'meshes/terrain.obj').stat().st_size < 1000000
    assert min(v[0] for v in vertices) == -3.44
    assert max(v[0] for v in vertices) == 3.44
    assert min(v[1] for v in vertices) == -2.5
    assert max(v[1] for v in vertices) == 2.5
    for x, y, _, depth in MODEL['CRATERS']:
        nearest = min(vertices, key=lambda v: (v[0] + 3.44 - x)**2 + (v[1] - y)**2)
        assert nearest[2] == pytest.approx(-depth, abs=0.005)
    for x, y, z in vertices:
        world_x = x + 3.44
        if (0 <= world_x <= 2 and y <= -0.5) or (world_x >= 3.88 and y <= -1):
            assert z == 0


def test_mesh_has_no_cracks_and_covers_the_arena_once(mesh):
    vertices, normals, faces = mesh
    assert len(normals) == len(vertices)
    for nx, ny, nz in normals:
        assert nx * nx + ny * ny + nz * nz == pytest.approx(1, abs=2e-6)
        assert nz > 0
    edges = Counter()
    area = 0.0
    for a, b, c in faces:
        assert all(0 <= index < len(vertices) for index in (a, b, c))
        p, q, r = vertices[a], vertices[b], vertices[c]
        twice_area = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        assert twice_area > 0, 'Degenerate or downward-facing triangle'
        area += twice_area / 2
        edges.update(tuple(sorted(edge)) for edge in ((a, b), (b, c), (c, a)))
    assert area == pytest.approx(MODEL['WIDTH'] * MODEL['LENGTH'], abs=1e-8)
    for (a, b), count in edges.items():
        assert count in (1, 2)
        if count == 1:
            # Only the outside perimeter may have an unpaired edge.
            p, q = vertices[a], vertices[b]
            assert any(p[axis] == q[axis] == bound for axis, bound in (
                (0, -3.44), (0, 3.44), (1, -2.5), (1, 2.5)))


def test_mesh_preserves_the_previous_uniform_grid_surface(mesh):
    vertices, _, faces = mesh

    def previous_height(x, y):
        # Interpolate the two triangles in each cell of the former 257 x 257 mesh.
        ux = min(max((x + MODEL['WIDTH'] / 2) * 256 / MODEL['WIDTH'], 0), 256)
        uy = min(max((y + MODEL['LENGTH'] / 2) * 256 / MODEL['LENGTH'], 0), 256)
        col, row = min(math.floor(ux), 255), min(math.floor(uy), 255)
        fx, fy = ux - col, uy - row

        def height(dx, dy):
            return MODEL['elevation']((col + dx) * MODEL['WIDTH'] / 256,
                                      (row + dy) * MODEL['LENGTH'] / 256 - MODEL['LENGTH'] / 2)

        if fx >= fy:
            return (1 - fx) * height(0, 0) + (fx - fy) * height(1, 0) + fy * height(1, 1)
        return (1 - fy) * height(0, 0) + fx * height(1, 1) + (fy - fx) * height(0, 1)

    for face in faces:
        triangle = [vertices[index] for index in face]
        # Sample every triangle at its centroid and edge midpoints.
        samples = [tuple(sum(p[axis] for p in triangle) / 3 for axis in range(3))]
        samples += [tuple((p[axis] + q[axis]) / 2 for axis in range(3))
                    for p, q in zip(triangle, triangle[1:] + triangle[:1])]
        for x, y, z in samples:
            assert z == pytest.approx(previous_height(x, y), abs=3e-6)
