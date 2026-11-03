"""Check optional Nav2 integration, prerequisites, and RViz selection."""
from pathlib import Path
import runpy

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.substitutions import FindPackageShare


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


def test_deploy_arguments_supply_sensor_defaults_and_simulation_clock():
    launch = runpy.run_path(str(ROOT / 'launch/launch.py'))
    ctx = context(robot_backend='gazebo', enable_rtabmap='true', enable_nav2='true')
    for action in launch['generate_launch_description']().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    assert ctx.launch_configurations['enable_rgbd'] == 'true'
    assert ctx.launch_configurations['enable_lidar'] == 'true'
    navigation = launch['launch_nav2'](ctx)
    arguments = dict(navigation[0].launch_arguments)
    assert perform_substitutions(ctx, [arguments['use_sim_time']]) == 'true'
    assert perform_substitutions(ctx, [arguments['enable_local_lidar']]) == 'false'


@pytest.mark.parametrize('sim_time', ['true', 'false'])
@pytest.mark.parametrize('local_lidar', ['true', 'false'])
def test_enabled_navigation_includes_new_package_and_matching_arguments(sim_time, local_lidar):
    launch = runpy.run_path(str(ROOT / 'launch/launch.py'))
    ctx = context(enable_nav2='true', robot_backend='gazebo',
                  enable_rtabmap='true', enable_rgbd='true', enable_lidar='true',
                  use_sim_time=sim_time, enable_local_lidar=local_lidar)
    actions = launch['launch_nav2'](ctx)
    assert len(actions) == 1
    arguments = dict(actions[0].launch_arguments)
    assert arguments['odom_topic'] == '/rtabmap/icp_odom'
    assert perform_substitutions(ctx, [arguments['use_sim_time']]) == sim_time
    assert perform_substitutions(ctx, [arguments['enable_local_lidar']]) == local_lidar
    source = actions[0].launch_description_source
    source.get_launch_description(ctx)
    path = Path(source.location)
    package_share = Path(FindPackageShare('robot_navigation').perform(ctx))
    assert path == package_share / 'launch/navigation.launch.py'
    assert path.is_file()


@pytest.mark.parametrize('enabled', ['true', '1', 'false', '0'])
def test_rviz_selection_only_resolves_navigation_package_when_enabled(enabled, monkeypatch):
    launch = runpy.run_path(str(ROOT / 'launch/launch.py'))
    description = launch['generate_launch_description']()
    ctx = context(enable_nav2=enabled)
    gazebo = next(action for action in description.entities
                  if isinstance(action, IncludeLaunchDescription)
                  and 'rviz_config' in dict(action.launch_arguments))
    original_find = FindPackageShare.find

    def find_package(self, package_name):
        if enabled in ('false', '0') and package_name == 'robot_navigation':
            raise AssertionError('Nav2 package resolved while navigation is disabled')
        return original_find(self, package_name)

    monkeypatch.setattr(FindPackageShare, 'find', find_package)
    rviz = dict(gazebo.launch_arguments)['rviz_config'].perform(ctx)
    if enabled in ('true', '1'):
        package_share = Path(FindPackageShare('robot_navigation').perform(ctx))
        expected = package_share / 'rviz/navigation.rviz'
    else:
        expected = Path(FindPackageShare('startup').perform(ctx)) / 'config/RGBDRViz.rviz'
    assert Path(rviz) == expected
    assert expected.is_file()
