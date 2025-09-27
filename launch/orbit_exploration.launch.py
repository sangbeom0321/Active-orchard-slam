#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare(package='aos').find('aos')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('aos'),
            'config',
            'orbit_planner_params.yaml'
        ]),
        description='Path to config file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('aos'),
            'rviz',
            'orbit_planner.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    # GVD topology arguments
    connect_radius_arg = DeclareLaunchArgument(
        'connect_radius', 
        default_value='5', 
        description='Graph connection radius (cells)'
    )
    ns_arg = DeclareLaunchArgument(
        'ns', 
        default_value='gvd', 
        description='Namespace for published topics'
    )
    occ_topic_arg = DeclareLaunchArgument(
        'occupancy_topic', 
        default_value='/orbit_planner/pcd_occupancy', 
        description='OccupancyGrid topic'
    )
    goal_topic_arg = DeclareLaunchArgument(
        'goal_topic', 
        default_value='/gvd/goal', 
        description='Goal PoseStamped topic'
    )
    
    #Launch LIO-SAM (assuming it's available)
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lio_sam'),
                'launch',
                'run.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # Launch orbit planner node
    orbit_planner_node = Node(
        package='aos',
        executable='orbit_planner_node',
        name='orbit_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/lio_sam/mapping/map_global', '/pointcloud_map'),
            ('/lio_sam/mapping/odometry', '/robot_odometry')
        ]
    )
    
    # Launch GVD topology node
    gvd_topology_node = Node(
        package='aos',
        executable='gvd_topology_node',
        name='gvd_topology_node',
        output='screen',
        parameters=[{
            'connect_radius': LaunchConfiguration('connect_radius'),
            'publish_namespace': LaunchConfiguration('ns'),
            'subscribe_topic': LaunchConfiguration('occupancy_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Launch control node (assuming control_4ws is available)
    control_node = Node(
        package='control_4ws',
        executable='control_4ws_node',
        name='control_4ws',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/orbit_planner/trajectory', '/path_to_follow'),
            ('/orbit_planner/goal', '/current_goal')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        rviz_config_arg,
        connect_radius_arg,
        ns_arg,
        occ_topic_arg,
        goal_topic_arg,
        # lio_sam_launch,
        orbit_planner_node,
        gvd_topology_node,
        # rviz_node,
        # control_node
    ])