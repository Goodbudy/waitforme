#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    space = 10.0
    # 0) Set TurtleBot3 model environment variable
    set_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle_pi'
    )

    # 1) Launch Gazebo with Gallery_Test2
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

    # 2) Start the A* planner node after 5 minutes (300s)
    astar_node = Node(
        package='andy',
        executable='astar_planner',
        output='screen'
    )
    delayed_astar = TimerAction(
        period=space,
        actions=[astar_node]
    )

    # 6) Run object detection
    detection_node = Node(
        package='tom',
        executable='detection_node',
        output='screen'
    )
    delayed_detection = TimerAction(
        period=space + 0.0,
        actions=[detection_node]
    )

    # 3) Bringup Nav2 after 5m10s (310s)
    map_file = os.path.join(
        os.environ['HOME'], 'ros2_ws', 'src', 'waitforme', 'GalleryMapHD.yaml'
    )
    params_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'params',
        'nav2_params.yaml'
    )
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )
    delayed_nav2 = TimerAction(
        period=space + 20.0,
        actions=[nav2_bringup]
    )

    # 4) Open RViz after 5m20s (320s)
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

    # 5) Run localization node
    localisation_node = Node(
        package='tom',
        executable='localisation_node',
        output='screen'
    )
    delayed_localisation = TimerAction(
        period=space + 40.0,
        actions=[localisation_node]
    )

    # 6) Finally run movementlogic node after 5m40s (340s)
    movement_node = Node(
        package='issy',
        executable='movementlogic',
        output='screen'
    )
    delayed_movement = TimerAction(
        period=space + 50.0,
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
