# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    def robot_state_publisher(context):
        robot_description_file = PathJoinSubstitution(
            [
                FindPackageShare('lunabotics_simulation'),
                'urdf',
                'robot',
                'urdf',
                'robot_gazebo.urdf.xacro',
            ]
        )
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                robot_description_file,
            ]
        )
        robot_description = {'robot_description': robot_description_content}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [node_robot_state_publisher]


    # Nodes important for running the simulation and controlling the robot
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'Bruno', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
    )
    
    arm_drum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_drum_controller'],
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Nodes important for connecting the robot to the Control Station and running robot movements
    teleop = Node(
                    package='teleop',
                    executable='teleop',
                    name='teleop',
                    output='screen',
                    parameters=[{
                        'linear_axis': 'left_y',
                        'turn_axis': 'left_x_inverted',
                        'full_forward_magnitude': 0.6,
                        'deadband': 0.05
                    }],
                    arguments=['--ros-args', '--log-level', 'WARN'],
                    respawn=True
                )
    
    digdump = Node(
                package='digdump',
                executable='action_server',
                name='digdump',
                output='screen',
                parameters=[{
                    'actuator_speed_aerial': 1.0,
                    'actuator_speed_ground': 0.75,
                    'dig_speed': 0.75,
                    'dump_speed': 1.0,
                    'drive_speed': 0.25,
                    'dig_time': 3.0,
                    'dump_time': 6.0,
                    'move_time': 6.0,
                    'actuator_extend_length_aerial': 0.69,
                    'actuator_extend_length_ground': 0.9,
                }],
                respawn=True
            )
    network_client = Node(
                package='network_communication',
                executable='udp_client',
                name='client_node',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                respawn=True
            )
    network_server = Node(
                package='network_communication',
                executable='udp_server',
                name='server_node',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                respawn=True
            )
    serial = Node(
                package='serial_node',
                executable='op_reader',
                name='serial_node',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                parameters=[
                    {'mock_serial':EnvironmentVariable('MOCK_SERIAL', default_value='1')}
                ],
                respawn=True
            )
    controller = Node(
                package='robot_controller',
                executable='robot_controller',
                name='robot_controller',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                respawn=True
            )
    cmd_vel_mux = Node(
                package='topic_tools',
                executable='mux',
                name='cmd_vel_mux',
                output='screen',
                arguments=['/cmd_vel', '/cmd_vel/teleop', '/cmd_vel/autonomy'],
                respawn=True
            )
    arm_drum_mux = Node(
                package='topic_tools',
                executable='mux',
                name='arm_drum_mux',
                output='screen',
                arguments=['/arm_drum_controller/commands', '/arm_drum_control/teleop', '/arm_drum_control/autonomy'],
                respawn=True
            )
    robot_state_controller = Node(
                package='robot_state_controller',
                executable='robot_state_controller',
                name='robot_state_controller',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                respawn=True
            )

    ld = LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[PathJoinSubstitution([FindPackageShare('lunabotics_simulation'), 'models'])]
        ),
        bridge,
        twist_stamper,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [f' -r -v 1 ', 
                    PathJoinSubstitution([
                        FindPackageShare('lunabotics_simulation'),
                        'world',
                        'arena_nasa.world'
                    ])
            ])]
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner, diff_drive_controller_spawner, arm_drum_controller_spawner],
            )
        ),
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
        teleop,
        digdump,
        network_client,
        network_server,
        serial,
        controller,
        cmd_vel_mux,
        arm_drum_mux,
        robot_state_controller
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
