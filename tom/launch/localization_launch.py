import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set environment variable
    model_env = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle_pi')

    # Path to turtlebot3_world launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot3_gazebo'), '/launch/turtlebot3_world.launch.py'
        ])
    )

    # Path to navigation2 launch file
    map_file = os.path.join(os.getenv('HOME'), 'ros2_ws/src/waitforme/tom/turtle_map2.yaml')
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot3_navigation2'), '/launch/navigation2.launch.py'
        ]),
        launch_arguments={'use_sim_time': 'True', 'map': map_file}.items()
    )

    # Run localization node
    localization_node = ExecuteProcess(
        cmd=['ros2', 'run', 'tom', 'localization_node'],
        output='screen'
    )

    return LaunchDescription([
        model_env,
        gazebo_launch,
        navigation_launch,
        localization_node
    ])
