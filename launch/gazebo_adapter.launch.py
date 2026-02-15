#!/usr/bin/env python3
"""
Launch the Gazebo adapter for av-simulation integration.

This launch file starts the adapter node that bridges Gazebo simulation
with the Ottopia av-simulation framework.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Gazebo adapter"""
    
    # Declare arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='simulation.yaml',
        description='Configuration file name'
    )
    
    vehicle_namespace_arg = DeclareLaunchArgument(
        'vehicle_namespace',
        default_value='/vehicle',
        description='ROS namespace for vehicle topics'
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
        LaunchConfiguration('config')
    ])
    
    # Gazebo adapter node
    adapter_node = Node(
        package='offroad_gazebo_integration',
        executable='gazebo_adapter_node',
        name='gazebo_adapter',
        output='screen',
        parameters=[
            config_file,
            {
                'vehicle_namespace': LaunchConfiguration('vehicle_namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )
    
    return LaunchDescription([
        # Arguments
        config_arg,
        vehicle_namespace_arg,
        use_sim_time_arg,
        
        # Nodes
        adapter_node,
    ])
