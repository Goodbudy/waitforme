import os

from ament_index_python.packages import get_packages_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    nav2_yaml_1 = os.path.join(get_packages_share_directory(
        'hallie'), 'config', 'tb3_1_amcl_config.yaml')
    nav2_yaml_2 = os.path.join(get_packages_share_directory(
        'hallie'), 'config', 'tb3_2_amcl_config.yaml')
    # nav2_yaml_3 = os.path.join(get_packages_share_directory(
    #     'hallie'), 'config', 'tb3_3_amcl_config.yaml')
    # nav2_yaml_4 = os.path.join(get_packages_share_directory(
    #     'hallie'), 'config', 'tb3_4_amcl_config.yaml')
    # nav2_yaml_5 = os.path.join(get_packages_share_directory(
    #     'hallie'), 'config', 'tb3_5_amcl_config.yaml')
    map_file = os.path.join(get_packages_share_directory(
        'map_server'), 'config', 'map.yaml')

    return LaunchDescription([

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time':True},
                        {'topic_name': "map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}]
        ),

        Node(
            namespace="tb3_1",
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml_1]  
        ),
        Node(
            namespace="tb3_2",
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml_2]  
        ),
        # Node(
        #     namespace="tb3_3",
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=[nav2_yaml_3]  
        # ),
        # Node(
        #     namespace="tb3_4",
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=[nav2_yaml_4]  
        # ),
        # Node(
        #     namespace="tb3_5",
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=[nav2_yaml_5]  
        # ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time':True},
                        {'autostart':True},
                        {'bond_timeout':0.0},
                        {'node_names':['map_server','tb3_1/amcl','tb3_2/amcl']}] # ,'tb3_3/amcl','tb3_4/amcl','tb3_5/amcl']}] 
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='tb3_1',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='tb3_2',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='tb3_3',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='tb3_4',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='tb3_5',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace='tb3_1',
            name='rviz2_tb3_1',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('hallie'), 'rviz', 'tb3_1_config.rviz')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='tb3_2',
            name='rviz2_tb3_2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('hallie'), 'rviz', 'tb3_2_config.rviz')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='tb3_3',
            name='rviz2_tb3_3',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('hallie'), 'rviz', 'tb3_3_config.rviz')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='tb3_4',
            name='rviz2_tb3_4',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('hallie'), 'rviz', 'tb3_4_config.rviz')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='tb3_5',
            name='rviz2_tb3_5',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('hallie'), 'rviz', 'tb3_5_config.rviz')],
        )

    ])