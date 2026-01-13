# Copyright (c) 2026 BCK - MIT License

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('make_easy_loop_closures')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        Node(
            package='make_easy_loop_closures',
            executable='loop_closure_node.py',
            name='melc',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
    ])
