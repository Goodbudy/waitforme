#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    test_node = Node(
        package='turtlebot3_multi_robot',
        executable='Utest_single_goal_distribution',
        name='goal_distribution_test',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([test_node])
