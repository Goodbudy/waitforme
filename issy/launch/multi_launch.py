#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    space = 10.0

    set_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle_pi'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'Gallery_Test2.launch.py'
            )
        ),
        launch_arguments={}.items()
    )

    astar_node = Node(
        package='andy',
        executable='astar_planner',
        output='screen'
    )
    delayed_astar = TimerAction(
        period=space,
        actions=[astar_node]
    )

    detection_node = Node(
        package='tom',
        executable='detection_node',
        output='screen'
    )
    delayed_detection = TimerAction(
        period=space + 10.0,
        actions=[detection_node]
    )

    map_file = os.path.join(
        os.environ['HOME'], 'ros2_ws', 'src', 'waitforme', 'GalleryMapHD.yaml'
    )
    params_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'params',
        'nav2_params.yaml'
    )

    # Add a GroupAction to launch nav2 manually with map_server and lifecycle manager
    nav2_bringup = GroupAction(
        actions=[
            PushRosNamespace('tb1'),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[params_file, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl']
                }]
            )
        ]
    )

    delayed_nav2 = TimerAction(
        period=space + 20.0,
        actions=[nav2_bringup]
    )

    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            )
        ),
        launch_arguments={
            'rviz_config': rviz_config,
            'use_sim_time': 'True'
        }.items()
    )
    delayed_rviz = TimerAction(
        period=space + 30.0,
        actions=[rviz_launch]
    )

    localisation_node = Node(
        package='tom',
        executable='localisation_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    delayed_localisation = TimerAction(
        period=space + 50.0,
        actions=[localisation_node]
    )

    movement_node = Node(
        package='issy',
        executable='movementlogic',
        output='screen'
    )
    delayed_movement = TimerAction(
        period=space + 60.0,
        actions=[movement_node]
    )

    return LaunchDescription([
        set_model,
        gazebo_launch,
        delayed_astar,
        delayed_nav2,
        delayed_rviz,
        delayed_localisation,
        delayed_detection,
        delayed_movement,
    ])
