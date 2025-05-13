#!/usr/bin/env python3
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
#
# Authors: Arshad Mehmood (modified by Andrew Coorey)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()
    space = 10.0

    # Robot definitions
    robots = [
        {'name': 'tb1', 'x_pose': '3.0', 'y_pose': '3.0'},
        {'name': 'tb2', 'x_pose': '2.9', 'y_pose': '1.6'},
        # add more robots here
    ]

    TURTLEBOT3_MODEL = 'waffle'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true',
                                       description='Use simulator time'))
    ld.add_action(DeclareLaunchArgument('enable_drive', default_value='false',
                                       description='Enable robot drive node'))
    ld.add_action(DeclareLaunchArgument('enable_rviz', default_value='true',
                                       description='Enable RViz'))
    ld.add_action(DeclareLaunchArgument('rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('turtlebot3_multi_robot'),
            'rviz', 'multi_nav2_default_view.rviz'),
        description='Path to RViz config'))
    ld.add_action(DeclareLaunchArgument('nav_params_file',
        default_value=os.path.join(
            get_package_share_directory('turtlebot3_multi_robot'),
            'params', 'nav2_params.yaml'),
        description='Path to Nav2 parameters'))

    # Gazebo server & client
    world_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds', 'Gallery_Test2.world')
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_file}.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gzclient.launch.py'))
        )
    )

    # Map server & lifecycle
    ld.add_action(Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(
                get_package_share_directory('turtlebot3_multi_robot'),
                'worlds', 'GalleryMapHD.yaml')
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager', name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    ))

    # Spawn robots sequentially
    last_spawn = None
    package_dir = get_package_share_directory('turtlebot3_multi_robot')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_multi_robot'),
        'urdf', f'turtlebot3_{TURTLEBOT3_MODEL}.urdf')

    for robot in robots:
        ns = f"/{robot['name']}"

        # State publisher
        state_pub = Node(
            package='robot_state_publisher', namespace=[ns],
            executable='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'publish_frequency': 10.0}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            arguments=[urdf_file]
        )
        spawn_entity = Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=[
                '-file', os.path.join(
                    get_package_share_directory('turtlebot3_multi_robot'),
                    'models', f'turtlebot3_{TURTLEBOT3_MODEL}', 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', ns,
                '-x', robot['x_pose'], '-y', robot['y_pose'], '-z', '0.01',
                '-Y', '0.0', '-unpause'
            ]
        )
        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'False', 'namespace': ns, 'use_namespace': 'True',
                'map': '', 'map_server': 'False',
                'params_file': LaunchConfiguration('nav_params_file'),
                'default_bt_xml_filename': os.path.join(
                    get_package_share_directory('nav2_bt_navigator'),
                    'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
                ),
                'autostart': 'true', 'use_sim_time': use_sim_time,
                'log_level': 'warn'
            }.items()
        )

        if last_spawn is None:
            ld.add_action(state_pub)
            ld.add_action(spawn_entity)
            ld.add_action(bringup)
        else:
            ld.add_action(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_spawn,
                        on_exit=[state_pub, spawn_entity, bringup]
                    )
                )
            )
        last_spawn = spawn_entity

    # Delayed nodes for andy and tom
    astar_node = Node(package='andy', executable='astar_planner', output='screen')
    detection_node = Node(package='tom', executable='detection_node', output='screen')
    ld.add_action(TimerAction(period=space, actions=[astar_node]))
    ld.add_action(TimerAction(period=space + 10.0, actions=[detection_node]))

    # After all robots spawned, start RViz, drive, and movement logic per robot
    last_spawn_event = last_spawn
    for robot in robots:
        ns = f"/{robot['name']}"
        # Initial pose publisher
        msg = (
            f"{{header: {{frame_id: map}}, pose: {{pose: {{position: {{x: {robot['x_pose']},"
            f" y: {robot['y_pose']}, z: 0.1}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}}}"
        )
        init_pose = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable',
                f"{ns}/initialpose", 'geometry_msgs/PoseWithCovarianceStamped', msg
            ],
            output='screen'
        )

        rviz = TimerAction(
            period=space + 20.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'rviz_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time, 'namespace': ns,
                    'use_namespace': 'True', 'rviz_config':
                    LaunchConfiguration('rviz_config_file'),
                    'log_level': 'warn'
                }.items(),
                condition=IfCondition(LaunchConfiguration('enable_rviz'))
            )]
        )
        drive = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=[ns], output='screen',
            condition=IfCondition(LaunchConfiguration('enable_drive'))
        )
        movement = Node(
            package='issy', executable='movementlogic',
            namespace=[ns], output='screen'
        )

        ld.add_action(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_spawn_event,
                    on_exit=[init_pose, rviz, drive, movement]
                )
            )
        )
        last_spawn_event = init_pose

    return ld