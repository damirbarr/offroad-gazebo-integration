#!/usr/bin/env python3
"""
Launch RViz2 with LiDAR visualization configuration.

This launch file starts RViz2 with the pre-configured LiDAR visualization setup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RViz2 with LiDAR config"""
    
    # Declare arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='lidar_view.rviz',
        description='RViz2 configuration file name'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get configuration
    config_file = PathJoinSubstitution([
        FindPackageShare('offroad_gazebo_integration'),
        'config',
        'rviz',
        LaunchConfiguration('config')
    ])
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_lidar',
        output='screen',
        arguments=['-d', config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        # Arguments
        config_arg,
        use_sim_time_arg,
        
        # Nodes
        rviz_node,
    ])
