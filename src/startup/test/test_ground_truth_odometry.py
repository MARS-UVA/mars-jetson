"""Ground truth supplies simulation odometry without another TF authority."""

from pathlib import Path
import runpy
import subprocess
import xml.etree.ElementTree as ET
from unittest.mock import Mock, patch

from launch_ros.actions import Node
import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('backend', ['gazebo', 'serial'])
def test_ground_truth_plugin_only_in_simulation(backend):
    xml = subprocess.check_output([
        'xacro', str(ROOT / 'urdf/robot/urdf/robot.urdf.xacro'),
        f'robot_backend:={backend}', 'serial_port:=/dev/null',
        'camera_update_rate:=15',
    ], text=True)
    robot = ET.fromstring(xml)
    plugin = robot.find('.//plugin[@name="gz::sim::systems::OdometryPublisher"]')
    if backend == 'serial':
        assert plugin is None
        return
    assert plugin.findtext('odom_frame') == 'odom'
    assert plugin.findtext('robot_base_frame') == 'base_link'
    assert plugin.findtext('dimensions') == '3'
    assert plugin.findtext('odom_publish_frequency') == '50'
    assert plugin.findtext('odom_topic') == '/ground_truth/odom'
    assert plugin.findtext('tf_topic') == '/ground_truth/tf'


def test_odometry_and_tf_bridges_are_gazebo_to_ros():
    module = runpy.run_path(str(ROOT / 'launch/gazebo.launch.py'))
    generate = module['generate_launch_description']
    factory = Mock(side_effect=Node)
    with patch.dict(generate.__globals__, Node=factory):
        generate()
    for call in factory.call_args_list:
        args = call.kwargs.get('arguments', [])
        if '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' in args:
            assert '/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' in args
            assert ('/ground_truth/tf', '/tf') in call.kwargs['remappings']
            break
    else:
        pytest.fail('Missing ground-truth bridge')
