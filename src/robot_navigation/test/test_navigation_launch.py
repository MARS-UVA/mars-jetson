"""Check split parameter loading, launch overrides, and the command path."""

from pathlib import Path
import runpy
import shutil
from xml.etree import ElementTree

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


def parameters(node):
    result = {}
    for filename, _ in node._Node__expanded_parameter_arguments:
        # launch_ros serializes normalized string arrays as YAML tuples.
        result.update(yaml.full_load(Path(filename).read_text()))
    return result


def launch_overrides(node):
    node_key = f'/{node.node_name.lstrip("/")}'
    result = {}
    for filename, _ in node._Node__expanded_parameter_arguments:
        settings = yaml.full_load(Path(filename).read_text())
        if node_key in settings:
            result.update(settings[node_key]['ros__parameters'])
    return result


@pytest.mark.parametrize('planner_choice,planner_id,plugin_type', [
    ('smac', 'SmacPlanner', 'nav2_smac_planner::SmacPlannerHybrid'),
    ('theta_star', 'ThetaStar', 'nav2_theta_star_planner::ThetaStarPlanner'),
    (
        'smoothed_lazy_theta_star', 'LazyThetaStar',
        'nav2_lazy_theta_star_planner::LazyThetaStarPlanner',
    ),
])
@pytest.mark.parametrize('sim_time,odom_topic,autostart', [
    ('true', '/rtabmap/icp_odom', 'true'),
    ('false', '/test/odometry', 'false'),
])
@pytest.mark.parametrize('local_lidar', ['false', 'true'])
def test_split_parameters_and_command_path(
    sim_time, odom_topic, autostart, local_lidar,
    planner_choice, planner_id, plugin_type,
):
    nodes = expanded_nodes(
        use_sim_time=sim_time, odom_topic=odom_topic,
        autostart=autostart, enable_local_lidar=local_lidar,
        global_planner=planner_choice,
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
    planner_params = planner['planner_server']['ros__parameters']
    assert list(launch_overrides(nodes['planner_server'])['planner_plugins']) == [
        planner_id]
    assert planner_params[planner_id]['plugin'] == plugin_type
    if planner_id == 'LazyThetaStar':
        lazy_theta_star = planner_params[planner_id]
        assert lazy_theta_star['interpolation_step'] == 0.05
        assert lazy_theta_star['smoothing_radius'] > 0.0
        assert lazy_theta_star['turn_penalty_weight'] >= 0.0

    bt_navigator_overrides = launch_overrides(nodes['bt_navigator'])
    behavior_tree = ElementTree.parse(
        bt_navigator_overrides['default_nav_to_pose_bt_xml'])
    planner_selector = behavior_tree.find('.//PlannerSelector')
    assert planner_selector is not None
    assert planner_selector.attrib['default_planner'] == planner_id
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


def test_launch_defaults_resolve_installed_configuration():
    context = LaunchContext()
    for action in LAUNCH['generate_launch_description']().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    config = Path(context.launch_configurations['config_dir'])
    assert config.is_dir()
    assert context.launch_configurations['use_sim_time'] == 'false'
    assert context.launch_configurations['enable_local_lidar'] == 'false'
    assert context.launch_configurations['autostart'] == 'true'
    assert context.launch_configurations['odom_topic'] == '/rtabmap/icp_odom'
    nodes = expanded_nodes(config_dir=str(config))
    assert set(nodes) == SERVERS | {'lifecycle_manager_navigation'}


def test_custom_configuration_directory(tmp_path):
    config = tmp_path / 'config'
    shutil.copytree(ROOT / 'config', config)
    planner_file = config / 'planner.yaml'
    custom = yaml.safe_load(planner_file.read_text())
    custom['planner_server']['ros__parameters']['LazyThetaStar']['smoothing_radius'] = 0.35
    planner_file.write_text(yaml.safe_dump(custom))
    nodes = expanded_nodes(config_dir=str(config))
    planner = parameters(nodes['planner_server'])['planner_server']['ros__parameters']
    assert planner['LazyThetaStar']['smoothing_radius'] == 0.35
