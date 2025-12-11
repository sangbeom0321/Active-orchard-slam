#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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
            'aos_planner_params.yaml'
        ]),
        description='Path to config file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('aos'),
            'rviz',
            'aos_planner.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    #Launch LIO-SAM (assuming it's available)
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lio_sam_wo'),
                'launch',
                'run.launch.py'
            ])
        ]),
    )

    
    # Launch AOS GVD node
    aos_gvd_node = Node(
        package='aos',
        executable='aos_gvd_node',
        name='aos_gvd_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )
    
    # Launch AOS path generation node
    aos_path_gen_node = Node(
        package='aos',
        executable='aos_path_gen_node',
        name='aos_path_gen_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )
    
    # Launch AOS path linearization node
    aos_path_linearization_node = Node(
        package='aos',
        executable='aos_path_linearization_node',
        name='aos_path_linearization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )
    
    # Launch RViz
    # Note: Using unique name to avoid conflicts if rviz2 is already running
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='aos_rviz2',  # Changed from 'rviz2' to avoid name conflicts
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    local_path_pub = Node(
        package='aomr_bringup',
        executable='local_path_publisher',
        name='local_path_publisher',
    )

    aos_state_machine_node = Node(
        package='aos', 
        executable='aos_state_machine_node',
        name='aos_state_machine_node'
    )

    # Launch AOS seed generation node
    aos_seed_gen_node = Node(
        package='aos',
        executable='aos_seed_gen_node',
        name='aos_seed_gen_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # Launch GPS to UTM conversion node
    gps_to_utm_node = Node(
        package='aos',
        executable='gps_to_utm_node',
        name='gps_to_utm_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        rviz_config_arg,
        ## ===
        # lio_sam_launch,
        gps_to_utm_node,
        aos_seed_gen_node,
        aos_gvd_node,
        aos_state_machine_node,
        aos_path_gen_node,
        aos_path_linearization_node,
        local_path_pub,
        rviz_node,
    ])