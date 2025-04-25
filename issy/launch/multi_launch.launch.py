#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Declare a launch argument called “map_path” that defaults to empty.
    map_arg = DeclareLaunchArgument(
        'map_path',
        default_value='home/ros2_ws/src/waitforme/Gallery_Map.yaml',
        description='$HOME/ros2_ws/src/waitforme/Gallery_Map.yaml'
    )

    # 2) Gazebo (Gallery_Test2.world)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'worlds',
                'Gallery_Test2.world'
            )
        }.items()
    )

    # 3) astar_planner in package “andy”
    astar_node = Node(
        package='andy',
        executable='astar_planner',
        name='astar_planner',
        output='screen'
    )

    # 4) Nav2 + RViz, but use LaunchConfiguration('map_path') instead of a package file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_path'),
            'use_sim_time': 'True'
        }.items()
    )

    # 5) localization_node in “tom”
    localization_node = Node(
        package='tom',
        executable='localication_node',
        name='localication_node',
        output='screen'
    )

    # 6) movementlogic in “issy”
    movement_node = Node(
        package='issy',
        executable='movementlogic',
        name='movementlogic',
        output='screen'
    )

    return LaunchDescription([
        map_arg,           # must come before anything that uses it :contentReference[oaicite:0]{index=0}
        gazebo_launch,
        astar_node,
        nav2_launch,       # now picks up whatever you pass as --map_path
        localization_node,
        movement_node,
    ])
