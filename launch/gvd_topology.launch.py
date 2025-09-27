#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    connect_radius_arg = DeclareLaunchArgument(
        'connect_radius', default_value='5', description='Graph connection radius (cells)'
    )
    ns_arg = DeclareLaunchArgument(
        'ns', default_value='gvd', description='Namespace for published topics'
    )
    occ_topic_arg = DeclareLaunchArgument(
        'occupancy_topic', default_value='/orbit_planner/pcd_occupancy', description='OccupancyGrid topic'
    )
    goal_topic_arg = DeclareLaunchArgument(
        'goal_topic', default_value='/gvd/goal', description='Goal PoseStamped topic'
    )

    node = Node(
        package='aos',
        executable='gvd_topology_node',
        name='gvd_topology_node',
        output='screen',
        parameters=[{
            'connect_radius': LaunchConfiguration('connect_radius'),
            'publish_namespace': LaunchConfiguration('ns'),
            'subscribe_topic': LaunchConfiguration('occupancy_topic'),
            'goal_topic': LaunchConfiguration('goal_topic')
        }]
    )

    return LaunchDescription([
        connect_radius_arg,
        ns_arg,
        occ_topic_arg,
        goal_topic_arg,
        node
    ])



