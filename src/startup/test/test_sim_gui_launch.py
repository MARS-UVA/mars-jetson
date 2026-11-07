"""Check GUI controls across startup, Gazebo, RTAB-Map, and RGB-D launches."""

from pathlib import Path
import runpy

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest


ROOT = Path(__file__).resolve().parents[1]


def load(filename, settings):
    module = runpy.run_path(str(ROOT / 'launch' / filename))
    description = module['generate_launch_description']()
    context = LaunchContext()
    context.launch_configurations.update(settings)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return module, description, context


def included(description, argument):
    return next(action for action in description.entities
                if isinstance(action, IncludeLaunchDescription)
                and argument in dict(action.launch_arguments))


def arguments(action, context):
    return {
        name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for name, value in action.launch_arguments
    }


@pytest.mark.parametrize('no_gui', [None, 'false', 'true'])
@pytest.mark.parametrize('nav_view', [None, 'false', 'true'])
@pytest.mark.parametrize('nav2', ['false', 'true'])
def test_gui_controls_and_navigation_override(no_gui, nav_view, nav2):
    settings = dict(robot_backend='gazebo', enable_rtabmap='true', enable_nav2=nav2)
    if no_gui is not None:
        settings['no_gui'] = no_gui
    if nav_view is not None:
        settings['enable_nav_view'] = nav_view
    startup, description, context = load('launch.py', settings)
    headless = no_gui == 'true'
    expected_view = not headless
    if nav2 == 'true' and nav_view is not None:
        expected_view = nav_view == 'true'

    gazebo_arguments = arguments(included(description, 'rviz_config'), context)
    assert gazebo_arguments['no_gui'] == ('true' if headless else 'false')
    assert gazebo_arguments['enable_rviz'] == ('true' if expected_view else 'false')
    # Navigation servers still launch when their windows are disabled.
    assert len(startup['launch_nav2'](context)) == (1 if nav2 == 'true' else 0)

    gazebo, description, context = load('gazebo.launch.py', gazebo_arguments)
    robot_rviz = next(action for action in description.entities
                      if isinstance(action, Node) and action.node_package == 'rviz2')
    assert robot_rviz.condition.evaluate(context) is (not headless)

    gz_args = arguments(included(description, 'gz_args'), context)['gz_args'].split()
    assert ('-s' in gz_args) is headless
    assert ('--headless-rendering' in gz_args) is headless
    assert '-r' in gz_args
    assert Path(gz_args[-1]).is_file()

    mapping = gazebo['launch_rtabmap'](context)
    mapping_arguments = arguments(mapping[0], context)
    assert mapping_arguments['rtabmap_viz'] == ('false' if headless else 'true')
    assert mapping_arguments['icp_odometry'] == 'true'
    assert mapping_arguments['subscribe_scan_cloud'] == 'true'

    rgbd = included(description, 'enable_rviz')
    assert rgbd.condition.evaluate(context) is True
    _, description, context = load('rgbd_sim.launch.py', arguments(rgbd, context))
    nodes = [action for action in description.entities if isinstance(action, Node)]
    sensor_view = next(node for node in nodes if node.node_package == 'rviz2')
    assert sensor_view.condition.evaluate(context) is expected_view
    bridges = [node for node in nodes if node.node_package == 'ros_gz_bridge']
    assert len(bridges) == 3
    assert all(node.condition is None for node in bridges)


@pytest.mark.parametrize('no_gui', ['false', 'true'])
def test_direct_gazebo_launch_respects_no_gui(no_gui):
    _, description, context = load('gazebo.launch.py', {'no_gui': no_gui})
    rgbd_arguments = arguments(included(description, 'enable_rviz'), context)
    assert rgbd_arguments['enable_rviz'] == ('false' if no_gui == 'true' else 'true')


@pytest.mark.parametrize('world', sorted((ROOT / 'world').glob('arena*.world')),
                         ids=lambda p: p.stem)
def test_world_selects_matching_prior_and_shifted_robot_spawn(world):
    from startup.marker_priors import ARENA_APRILTAG_PRIORS

    gazebo, description, context = load('gazebo.launch.py', {
        'choose_sim_map': world.name, 'enable_rtabmap': 'true',
    })
    mapping = arguments(gazebo['launch_rtabmap'](context)[0], context)
    config_path = Path(mapping['cfg'])
    try:
        assert f'Marker\\Priors = {ARENA_APRILTAG_PRIORS[world.name]}' in config_path.read_text()
    finally:
        config_path.unlink()
    spawn = next(action for action in description.entities
                 if isinstance(action, Node) and action.node_package == 'ros_gz_sim')
    command = [perform_substitutions(context, normalize_to_list_of_substitutions(part))
               for part in spawn.cmd]
    x, y = [float(command[command.index(flag) + 1]) for flag in ('-x', '-y')]
    assert (x, y) == ((0.95, 0.735) if world.stem == 'arena_ucf' else (1, 1))


@pytest.mark.parametrize('prior', ['', '1 1 2 0.25 0 0 0'])
def test_explicit_prior_override_is_preserved(prior):
    gazebo, _, context = load('gazebo.launch.py', {
        'enable_rtabmap': 'true', 'rtabmap_marker_priors': prior,
    })
    mapping = arguments(gazebo['launch_rtabmap'](context)[0], context)
    if prior:
        path = Path(mapping['cfg'])
        try:
            assert f'Marker\\Priors = {prior}' in path.read_text()
        finally:
            path.unlink()
    else:
        assert mapping['cfg'] == ''
