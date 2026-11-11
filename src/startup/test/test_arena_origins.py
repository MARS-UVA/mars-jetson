"""World corners, robot spawn, and detector-frame priors share one coordinate system."""
from pathlib import Path
import math
import runpy
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRIORS = runpy.run_path(str(ROOT / 'startup/marker_priors.py'))['ARENA_APRILTAG_PRIORS']
WORLDS = sorted((ROOT / 'world').glob('arena*.world'))


def pose(element):
    return list(map(float, element.findtext('pose', '0 0 0 0 0 0').split()))


def included(world, name):
    return next(i for i in world.findall('include') if i.findtext('uri') == f'model://{name}')


def rotation(r, p, y):
    c, s = math.cos, math.sin
    return [
        [c(y)*c(p), c(y)*s(p)*s(r)-s(y)*c(r), c(y)*s(p)*c(r)+s(y)*s(r)],
        [s(y)*c(p), s(y)*s(p)*s(r)+c(y)*c(r), s(y)*s(p)*c(r)-c(y)*s(r)],
        [-s(p), c(p)*s(r), c(p)*c(r)],
    ]


@pytest.mark.parametrize('path', WORLDS, ids=lambda p: p.stem)
def test_interior_corner_is_zero_and_dimensions_are_preserved(path):
    world = ET.parse(path).getroot().find('world')
    name = 'arena_frame_ucf' if path.stem == 'arena_ucf' else 'arena_frame_nasa'
    frame_pose = pose(included(world, name))
    frame = ET.parse(ROOT / f'models/{name}/model.sdf').getroot().find('model')
    walls = {}
    for link in frame.findall('link'):
        if not link.get('name').endswith('_wall'):
            continue
        collision = link.find('collision')
        center = [sum(v) for v in zip(frame_pose, pose(frame), pose(link), pose(collision))]
        size = list(map(float, collision.findtext('geometry/box/size').split()))
        walls[link.get('name')] = center, size
    left, right, bottom, top = [walls[n] for n in ('left_wall', 'right_wall', 'bottom_wall', 'top_wall')]
    assert left[0][0] + left[1][0]/2 == pytest.approx(0, abs=1e-9)
    assert bottom[0][1] + bottom[1][1]/2 == pytest.approx(0, abs=1e-9)
    width, height = (9.04, 4.47) if path.stem == 'arena_ucf' else (6.88, 5)
    assert right[0][0] - right[1][0]/2 == pytest.approx(width)
    assert top[0][1] - top[1][1]/2 == pytest.approx(height)


@pytest.mark.parametrize('path', WORLDS, ids=lambda p: p.stem)
def test_prior_matches_textured_face_and_detector_axes(path):
    world = ET.parse(path).getroot().find('world')
    model_name = 'Apriltag36_11_00001' if path.name == 'arena_nasa.world' else 'apriltag_board'
    board = pose(included(world, model_name))
    r = rotation(*board[3:])
    fields = PRIORS[path.name].split()
    assert fields[0] == '1'
    prior = list(map(float, fields[1:]))
    # +Z face of the 1 cm box is the observed face. Include overrides the
    # model pose; tag 1's link and visual origins are both zero.
    expected_position = [board[i] + .005*r[i][2] for i in range(3)]
    assert prior[:3] == pytest.approx(expected_position, abs=1e-8)
    # Gazebo's +Z box UVs: image-right=-local Y, image-up=+local X.
    # The apriltag_ros PnP frame therefore has outward Z=+local Z.
    expected_rotation = [[-row[1], row[0], row[2]] for row in r]
    for actual, expected in zip(rotation(*prior[3:]), expected_rotation):
        assert actual == pytest.approx(expected, abs=1e-9)
    assert rotation(*prior[3:])[0][2] > .99, 'Prior must face into the arena (+X)'


def test_default_arena_has_one_tag_matching_its_prior():
    world = ET.parse(ROOT / 'world/arena_nasa.world').getroot().find('world')
    tag_includes = [i for i in world.findall('include')
                    if 'apriltag' in i.findtext('uri', '').lower()]
    assert len(tag_includes) == 1
    assert tag_includes[0].findtext('uri') == 'model://Apriltag36_11_00001'
    tag = ET.parse(ROOT / 'models/Apriltag36_11_00001/model.sdf').getroot().find('model')
    visuals = tag.findall('.//visual')
    assert len(visuals) == 1
    assert visuals[0].findtext('material/pbr/metal/albedo_map').endswith('tag36_11_00001.png')
    assert pose(tag_includes[0])[:3] == [0, 1, .25]
    assert len(PRIORS['arena_nasa.world'].split('|')) == 1
    assert PRIORS['arena_nasa.world'].split()[0] == '1'


def test_crater_mesh_bounds_match_corner_based_world():
    world = ET.parse(ROOT / 'world/arena_nasa_craters.world').getroot().find('world')
    offset = pose(included(world, 'arena_terrain'))
    vertices = [list(map(float, line.split()[1:]))
                for line in (ROOT / 'models/arena_terrain/meshes/terrain.obj').read_text().splitlines()
                if line.startswith('v ')]
    for axis, extent in [(0, 6.88), (1, 5)]:
        assert min(v[axis] for v in vertices) + offset[axis] == pytest.approx(0)
        assert max(v[axis] for v in vertices) + offset[axis] == pytest.approx(extent)
