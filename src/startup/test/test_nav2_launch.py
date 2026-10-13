"""Check optional startup, parameter rewriting, and the complete command path."""
from pathlib import Path
import runpy

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]


def context(**settings):
    result = LaunchContext()
    result.launch_configurations['ros_namespace'] = '/'
    result.launch_configurations.update(settings)
    return result


def test_nav2_is_disabled_by_default_and_skips_enabled_requirements():
    launch = runpy.run_path(str(ROOT / 'launch/launch.py'))
    description = launch['generate_launch_description']()
    ctx = context()
    argument = next(action for action in description.entities
                    if isinstance(action, DeclareLaunchArgument)
                    and action.name == 'enable_nav2')
    argument.execute(ctx)
    assert ctx.launch_configurations['enable_nav2'] == 'false'
    assert launch['launch_nav2'](ctx) == []


@pytest.mark.parametrize('missing', ['enable_rtabmap', 'enable_rgbd', 'enable_lidar'])
def test_enabled_navigation_requires_mapping_and_sensors(missing):
    launch = runpy.run_path(str(ROOT / 'launch/launch.py'))
    settings = dict(enable_nav2='true', robot_backend='gazebo',
                    enable_rtabmap='true', enable_rgbd='true', enable_lidar='true')
    settings[missing] = 'false'
    with pytest.raises(RuntimeError, match=missing):
        launch['launch_nav2'](context(**settings))


def test_integrated_navigation_requires_gazebo():
    launch = runpy.run_path(str(ROOT / 'launch/launch.py'))
    with pytest.raises(RuntimeError, match='robot_backend'):
        launch['launch_nav2'](context(enable_nav2='true', robot_backend='serial'))


@pytest.mark.parametrize('sim_time,odom_topic', [
    ('true', '/rtabmap/icp_odom'), ('false', '/rtabmap/visual_odom')])
def test_navigation_routes_commands_and_rewrites_all_parameters(sim_time, odom_topic):
    launch = runpy.run_path(str(ROOT / 'launch/nav2.launch.py'))
    ctx = context(params_file=str(ROOT / 'config/nav2.yaml'),
                  use_sim_time=sim_time, odom_topic=odom_topic, autostart='true')
    nodes = launch['launch_navigation'](ctx)
    assert all(isinstance(node, Node) for node in nodes)
    for node in nodes:
        # Expand the real launch substitutions without starting any processes.
        node._perform_substitutions(ctx)
    by_name = {node.node_name.lstrip('/'): node for node in nodes}
    assert set(by_name) == {
        'controller_server', 'planner_server', 'smoother_server',
        'behavior_server', 'bt_navigator', 'velocity_smoother',
        'lifecycle_manager_navigation'}
    for name in ('controller_server', 'behavior_server'):
        assert by_name[name].expanded_remapping_rules == [
            ('cmd_vel', '/cmd_vel/navigation_raw')]
    assert by_name['velocity_smoother'].expanded_remapping_rules == [
        ('cmd_vel', '/cmd_vel/navigation_raw'),
        ('cmd_vel_smoothed', '/cmd_vel/navigation')]
    filename = by_name['controller_server']._Node__expanded_parameter_arguments[0][0]
    params = yaml.safe_load(Path(filename).read_text())
    def check(value):
        if not isinstance(value, dict):
            return
        for key, item in value.items():
            if key == 'use_sim_time':
                assert item == (sim_time == 'true')
            if key == 'odom_topic':
                assert item == odom_topic
            check(item)
    check(params)
    assert params['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'] == (sim_time == 'true')
    lifecycle_file = by_name['lifecycle_manager_navigation']._Node__expanded_parameter_arguments[0][0]
    # launch_ros serializes normalized string arrays as YAML tuples.
    lifecycle = yaml.full_load(Path(lifecycle_file).read_text())
    manager = lifecycle['/lifecycle_manager_navigation']['ros__parameters']
    assert manager['autostart'] is True
    assert set(manager['node_names']) == set(by_name) - {'lifecycle_manager_navigation'}


def test_enabled_navigation_includes_matching_odometry_and_clock():
    from launch.utilities import perform_substitutions
    launch = runpy.run_path(str(ROOT / 'launch/launch.py'))
    ctx = context(enable_nav2='true', robot_backend='gazebo',
                  enable_rtabmap='true', enable_rgbd='true', enable_lidar='true',
                  use_sim_time='true')
    actions = launch['launch_nav2'](ctx)
    assert len(actions) == 1
    arguments = dict(actions[0].launch_arguments)
    assert arguments['odom_topic'] == '/rtabmap/icp_odom'
    assert perform_substitutions(ctx, [arguments['use_sim_time']]) == 'true'
