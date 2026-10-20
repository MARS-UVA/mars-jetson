"""Check diagram obstacle conversion, safe zones and generated mesh placement."""
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
TERRAIN = ROOT / 'models/arena_terrain'
MODEL = runpy.run_path(str(TERRAIN / 'generate_heightmap.py'))


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


def test_generated_mesh_resolves_all_ten_craters_and_arena_bounds():
    vertices = [tuple(map(float, line.split()[1:]))
                for line in (TERRAIN / 'meshes/terrain.obj').read_text().splitlines()
                if line.startswith('v ')]
    assert len(vertices) == MODEL['MESH_RESOLUTION']**2
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
