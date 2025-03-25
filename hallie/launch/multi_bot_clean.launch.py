#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node


def generate_launch_description():
    # Environment variable: e.g., 'burger', 'waffle_pi', etc.
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')

    # Configurable parameters
    number_of_robots = 2
    namespace_prefix = 'tb3'
    pose = [[-0.5, 0.0], [0.5, 0.0]]

    # Paths
    tb3_gazebo_path = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')

    model_folder = f'turtlebot3_{TURTLEBOT3_MODEL}'
    original_sdf_path = os.path.join(tb3_gazebo_path, 'models', model_folder, 'model.sdf')
    tmp_sdf_folder = os.path.join(os.getenv('HOME'), 'ros2_ws', 'tmp_sdf')
    os.makedirs(tmp_sdf_folder, exist_ok=True)

    # World path
    world_file = os.path.join(tb3_gazebo_path, 'worlds', 'turtlebot3_world.world')

    # Launch core Gazebo components
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmds = []
    spawn_robot_cmds = []

    for i in range(number_of_robots):
        ns = f"{namespace_prefix}_{i+1}"

        # Modify the .sdf file with namespace-specific frames
        tree = ET.parse(original_sdf_path)
        root = tree.getroot()

        for tag in root.iter('odometry_frame'):
            tag.text = f'{ns}/odom'
        for tag in root.iter('robot_base_frame'):
            tag.text = f'{ns}/base_footprint'
        for tag in root.iter('frame_name'):
            tag.text = f'{ns}/base_scan'

        modified_sdf = '<?xml version="1.0" ?>\n' + ET.tostring(root, encoding='unicode')
        sdf_path = os.path.join(tmp_sdf_folder, f'{ns}.sdf')

        with open(sdf_path, 'w') as f:
            f.write(modified_sdf)

        # Create the spawn_entity Node directly
        spawn_robot_cmds.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', f'{TURTLEBOT3_MODEL}_{i+1}',
                    '-file', sdf_path,
                    '-x', str(pose[i][0]),
                    '-y', str(pose[i][1]),
                    '-z', '0.01',
                    '-robot_namespace', ns
                ],
                output='screen'
            )
        )

        # Optional: add robot_state_publisher_cmds[i] here if needed
        robot_state_publisher_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_gazebo_path, 'launch', 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'frame_prefix': ns
                }.items()
            )
        )

    # Main launch description
    ld = LaunchDescription()

    # Add core Gazebo processes
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # Cleanup on shutdown
    ld.add_action(
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, context: [
                    os.remove(os.path.join(tmp_sdf_folder, f"{namespace_prefix}_{i+1}.sdf"))
                    for i in range(number_of_robots)
                ]
            )
        )
    )

    # Group robot actions per namespace
    for i in range(number_of_robots):
        ns = f"{namespace_prefix}_{i+1}"
        group = GroupAction([
            PushRosNamespace(ns),
            robot_state_publisher_cmds[i],
            spawn_robot_cmds[i]
        ])
        ld.add_action(group)

    return ld


