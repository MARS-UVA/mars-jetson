"""Check split parameter loading, launch overrides, and the command path."""

from pathlib import Path
import runpy
import shutil

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = runpy.run_path(str(ROOT / 'launch/navigation.launch.py'))
SERVERS = {
    'controller_server', 'planner_server', 'smoother_server',
    'behavior_server', 'bt_navigator', 'velocity_smoother',
}


def expanded_nodes(**settings):
    context = LaunchContext()
    context.launch_configurations.update({
        'ros_namespace': '/',
        'config_dir': str(ROOT / 'config'),
        'use_sim_time': 'false',
        'odom_topic': '/rtabmap/icp_odom',
        'autostart': 'true',
        **settings,
    })
    nodes = LAUNCH['launch_navigation'](context)
    for node in nodes:
        # Resolve the real ROS launch substitutions without starting processes.
        node._perform_substitutions(context)
    return {node.node_name.lstrip('/'): node for node in nodes}


def merge(base, overlay):
    # ROS applies parameter files in order, one leaf parameter at a time.
    for key, value in overlay.items():
        base[key] = merge(base.get(key, {}), value) if isinstance(value, dict) else value
    return base


def parameters(node):
    result = {}
    for filename, _ in node._Node__expanded_parameter_arguments:
        # launch_ros serializes normalized string arrays as YAML tuples.
        merge(result, yaml.full_load(Path(filename).read_text()))
    return result


@pytest.mark.parametrize('sim_time,odom_topic,autostart', [
    ('true', '/rtabmap/icp_odom', 'true'),
    ('false', '/test/odometry', 'false'),
])
@pytest.mark.parametrize('local_lidar', ['false', 'true'])
def test_split_parameters_and_command_path(sim_time, odom_topic, autostart, local_lidar):
    nodes = expanded_nodes(
        use_sim_time=sim_time, odom_topic=odom_topic,
        autostart=autostart, enable_local_lidar=local_lidar,
    )
    expected = SERVERS | {'lifecycle_manager_navigation'}
    if local_lidar == 'true':
        expected.add('navigation_cloud_filter')
    assert set(nodes) == expected

    for name in SERVERS:
        server = parameters(nodes[name])[name]['ros__parameters']
        assert server['use_sim_time'] is (sim_time == 'true')
        if name in ('controller_server', 'bt_navigator', 'velocity_smoother'):
            assert server['odom_topic'] == odom_topic

    controller = parameters(nodes['controller_server'])
    planner = parameters(nodes['planner_server'])
    local = controller['local_costmap']['local_costmap']['ros__parameters']
    global_map = planner['global_costmap']['global_costmap']['ros__parameters']
    assert local['use_sim_time'] is (sim_time == 'true')
    assert global_map['use_sim_time'] is (sim_time == 'true')
    assert local['global_frame'] == 'odom'
    assert global_map['global_frame'] == 'map'
    assert global_map['static_layer']['map_topic'] == '/rtabmap/map'
    expected_sources = 'rgbd_blue rgbd_orange'
    if local_lidar == 'true':
        expected_sources += ' lidar'
        cloud_filter = parameters(nodes['navigation_cloud_filter'])[
            'navigation_cloud_filter']['ros__parameters']
        assert cloud_filter['use_sim_time'] is (sim_time == 'true')
        assert cloud_filter['output_topic'] == local['voxel_layer']['lidar']['topic']
        assert nodes['navigation_cloud_filter'].node_package == 'robot_navigation'
    assert local['voxel_layer']['observation_sources'] == expected_sources

    for name in ('controller_server', 'behavior_server'):
        assert nodes[name].expanded_remapping_rules == [
            ('cmd_vel', '/cmd_vel/navigation_raw')]
    assert nodes['velocity_smoother'].expanded_remapping_rules == [
        ('cmd_vel', '/cmd_vel/navigation_raw'),
        ('cmd_vel_smoothed', '/cmd_vel/navigation'),
    ]
    manager = parameters(nodes['lifecycle_manager_navigation'])[
        '/lifecycle_manager_navigation']['ros__parameters']
    assert manager['use_sim_time'] is (sim_time == 'true')
    assert manager['autostart'] is (autostart == 'true')
    assert set(manager['node_names']) == SERVERS


@pytest.mark.parametrize('controller,plugin', [
    (None, 'nav2_mppi_controller::MPPIController'),
    ('mppi', 'nav2_mppi_controller::MPPIController'),
    ('rpp', 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'),
])
def test_controller_selection_keeps_plugin_name_and_shared_checkers(controller, plugin):
    settings = {} if controller is None else {'controller': controller}
    nodes = expanded_nodes(use_sim_time='true', **settings)
    server = parameters(nodes['controller_server'])['controller_server']['ros__parameters']
    # The behavior tree calls FollowPath by name, so the name must not change.
    assert server['controller_plugins'] == ['FollowPath']
    assert server['FollowPath']['plugin'] == plugin
    assert server['goal_checker']['xy_goal_tolerance'] == 0.20
    assert server['progress_checker']['movement_time_allowance'] == 20.0
    assert server['use_sim_time'] is True
    local = parameters(nodes['controller_server'])['local_costmap']['local_costmap']
    if controller == 'rpp':
        # RPP reads obstacle distance back out of the inflation curve.
        assert (server['FollowPath']['inflation_cost_scaling_factor']
                == local['ros__parameters']['inflation_layer']['cost_scaling_factor'])
        assert server['FollowPath']['allow_reversing'] is False


def test_unknown_controller_is_rejected():
    with pytest.raises(RuntimeError, match='controller must be one of'):
        expanded_nodes(controller='dwb')


def test_launch_defaults_resolve_installed_configuration():
    context = LaunchContext()
    for action in LAUNCH['generate_launch_description']().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    config = Path(context.launch_configurations['config_dir'])
    assert config.is_dir()
    assert context.launch_configurations['arena_name'] == ''
    assert context.launch_configurations['use_sim_time'] == 'false'
    assert context.launch_configurations['enable_local_lidar'] == 'false'
    assert context.launch_configurations['autostart'] == 'true'
    assert context.launch_configurations['odom_topic'] == '/rtabmap/icp_odom'
    assert context.launch_configurations['controller'] == 'mppi'
    nodes = expanded_nodes(config_dir=str(config))
    assert set(nodes) == SERVERS | {'lifecycle_manager_navigation'}


def test_custom_configuration_directory(tmp_path):
    config = tmp_path / 'config'
    shutil.copytree(ROOT / 'config', config)
    planner_file = config / 'planner.yaml'
    custom = yaml.safe_load(planner_file.read_text())
    custom['planner_server']['ros__parameters']['GridBased']['tolerance'] = 0.35
    planner_file.write_text(yaml.safe_dump(custom))
    nodes = expanded_nodes(config_dir=str(config))
    planner = parameters(nodes['planner_server'])['planner_server']['ros__parameters']
    assert planner['GridBased']['tolerance'] == 0.35


@pytest.mark.parametrize('sim_time', ['false', 'true'])
def test_arena_override(sim_time):
    nodes = expanded_nodes(arena_name='another_arena', use_sim_time=sim_time)
    arena = parameters(nodes['arena_markers_node'])['arena_markers_node']['ros__parameters']
    assert arena['arena_name'] == 'another_arena'
    assert arena['use_sim_time'] is (sim_time == 'true')
    assert set(nodes) == SERVERS | {'lifecycle_manager_navigation', 'arena_markers_node'}


def test_empty_arena_omits_markers():
    assert 'arena_markers_node' not in expanded_nodes(arena_name='')
