import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    robots = [
        {'name': 'tb1'},
        {'name': 'tb2'},
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare use_sim_time argument
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))

    # Load real map path
    real_map = os.path.join(
        get_package_share_directory('turtlebot3_multi_robot'),
        'worlds', 'real_map4.yaml')

    # Launch the GoalManager node (non-namespaced)
    ld.add_action(Node(
        package='turtlebot3_multi_robot',
        executable='manager',
        name='manager',
        output='screen'
    ))

    for robot in robots:
        ns = f"/{robot['name']}"

        # Detection node
        ld.add_action(Node(
            package='tom',
            executable='detection_node',
            namespace=ns,
            output='screen',
            parameters=[{
                'map_yaml_path': real_map,
                'use_sim_time': use_sim_time
            }]
        ))

        # A* planner node
        ld.add_action(Node(
            package='andy',
            executable='astar_planner',
            namespace=ns,
            output='screen',
            parameters=[{
                'map_yaml_path': real_map,
                'use_sim_time': use_sim_time
            }]
        ))

        # Movement logic node
        delayed_movement = TimerAction(
            period=10.0,
            actions=[Node(
                package='issy',
                executable='movementlogic',
                namespace=ns,
                output='screen'
            )]
        )
        ld.add_action(delayed_movement)

    return ld
