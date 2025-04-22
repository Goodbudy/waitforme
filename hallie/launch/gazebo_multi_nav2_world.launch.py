#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
import tempfile


def generate_launch_description():
    ld = LaunchDescription()

    # Define parameters
    TURTLEBOT3_MODEL = 'waffle_pi'
    number_of_robots = 2
    namespace_prefix = 'tb'

    # Get paths
    gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    hallie_pkg_dir = get_package_share_directory('hallie')

    sdf_template_path = os.path.join(gazebo_pkg_dir, 'models', f'turtlebot3_{TURTLEBOT3_MODEL}', 'model.sdf')
    urdf_path = os.path.join(gazebo_pkg_dir, 'urdf', f'turtlebot3_{TURTLEBOT3_MODEL}.urdf')
    world_path = os.path.join(gazebo_pkg_dir, 'worlds', 'turtlebot3_world.world')
    nav_launch_dir = os.path.join(hallie_pkg_dir, 'launch', 'nav2_bringup')
    tmp_sdf_folder = os.path.join(os.getenv('HOME'), 'ros2_ws', 'tmp_sdf')
    os.makedirs(tmp_sdf_folder, exist_ok=True)

    # Declare launch args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(hallie_pkg_dir, 'rviz', 'multi_nav2_default_view.rviz'))
    params_file = LaunchConfiguration('nav_params_file', default=os.path.join(hallie_pkg_dir, 'params', 'nav2_params.yaml'))

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_drive', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('rviz_config_file', default_value=rviz_config_file, description='Path to RVIZ config'))
    ld.add_action(DeclareLaunchArgument('nav_params_file', default_value=params_file, description='Path to nav2 parameters YAML'))

    # Add Gazebo
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    ))

    pose = [(-1.5, -0.5), (-1.5, 0.5)]

    for i in range(number_of_robots):
        ns = f"{namespace_prefix}{i+1}"

        # Modify .sdf
        tree = ET.parse(sdf_template_path)
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

        # Spawn robot
        spawn_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', ns,
                '-file', sdf_path,
                '-x', str(pose[i][0]),
                '-y', str(pose[i][1]),
                '-z', '0.01',
                '-robot_namespace', ns
            ],
            output='screen'
        )

        # State publisher
        state_pub_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf_path],
            output='screen'
        )

        # Nav bringup
        nav_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'namespace': ns,
                'use_namespace': 'True',
                'map': '',
                'slam': 'False',
                'map_server': 'False',
                'params_file': params_file,
                'autostart': 'true',
                'use_sim_time': use_sim_time
            }.items()
        )

        ld.add_action(GroupAction([
            PushRosNamespace(ns),
            state_pub_cmd,
            spawn_cmd,
            nav_cmd
        ]))

    # RViz and drive nodes launched after robot spawn (with initial pose topic)
    for i in range(number_of_robots):
        ns = f"{namespace_prefix}{i+1}"

        # Publish initial pose to /<namespace>/initialpose
        x_pose, y_pose = pose[i]
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
                  str(x_pose) + ', y: ' + str(y_pose) + \
                  ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', f'/{ns}/initialpose',
                 'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'rviz_launch.py')),
            condition=IfCondition(enable_rviz),
            launch_arguments={
                'namespace': ns,
                'use_namespace': 'True',
                'rviz_config': rviz_config_file,
                'log_level': 'warn'
            }.items()
        )

        drive_cmd = Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_drive',
            namespace=ns,
            output='screen',
            condition=IfCondition(enable_drive)
        )

        ld.add_action(initial_pose_cmd)
        ld.add_action(rviz_cmd)
        ld.add_action(drive_cmd)

    return ld

