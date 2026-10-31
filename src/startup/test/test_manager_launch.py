"""Startup uses the consolidated manager and arbiter, without legacy muxes."""

from pathlib import Path
import runpy

from launch import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


def test_robot_launch_uses_manager_and_shared_command_arbiter():
    module = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'launch/robot.launch.py'))
    description = module['generate_launch_description']()
    context = LaunchContext()
    nodes = [node for node in description.entities if isinstance(node, Node)]
    executables = [(perform_substitutions(context, normalize_to_list_of_substitutions(node.node_package)),
                    perform_substitutions(context, normalize_to_list_of_substitutions(node.node_executable))) for node in nodes]
    assert ('robot_super_intelligence', 'state_manager') in executables
    assert ('robot_super_intelligence', 'command_mux') in executables
    assert ('digdump_actions', 'dig_node') in executables
    assert ('digdump_actions', 'dump_node') in executables
    assert not any(package in ('robot_state_controller', 'topic_tools')
                   for package, _ in executables)
