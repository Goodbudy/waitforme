
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    import os
    os.environ["TURTLEBOT3_MODEL"] = "waffle_pi"  # Change to "waffle" or "waffle_pi" if needed


    # Get package paths
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_navigation2 = get_package_share_directory('turtlebot3_navigation2')
    pkg_tom = get_package_share_directory('tom')  # Your package

    # World and map paths
    world_file = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.sdf')
    map_file = os.path.join(pkg_tom, 'maps', 'turtle_map2.yaml')  # Ensure your map is here

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Launch the map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    # Launch AMCL for localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Run the AutoLocalize node
    auto_localize = Node(
        package='tom',
        executable='localization_node',
        name='auto_localize',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        map_server,
        amcl,
        auto_localize,
    ])

