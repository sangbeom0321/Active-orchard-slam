#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('aos')
    
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'esdf_test.rviz'),
        description='Path to RViz2 config file'
    )
    
    # ESDF test node
    esdf_test_node = Node(
        package='aos',
        executable='esdf_test_node',
        name='esdf_test_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )
    
    # Bag play command (optional - for testing with recorded data)
    bag_play_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--clock', '--loop', '/root/mnt/slam_isaasc_bag/rosbag2_2025_06_28-07_26_31_0.db3'],
        output='screen',
        condition=LaunchConfiguration('use_bag')
    )
    
    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        esdf_test_node,
        rviz_node,
    ])
