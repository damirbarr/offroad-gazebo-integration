#!/usr/bin/env python3
"""
Launch UDP bridge for Gazebo-av-simulation integration
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    av_sim_ip_arg = DeclareLaunchArgument(
        'av_sim_ip',
        default_value='127.0.0.1',
        description='IP address of av-simulation UDP adapter'
    )
    
    av_sim_command_port_arg = DeclareLaunchArgument(
        'av_sim_command_port',
        default_value='9001',
        description='UDP port for receiving commands from av-simulation'
    )
    
    av_sim_sensor_port_arg = DeclareLaunchArgument(
        'av_sim_sensor_port',
        default_value='9002',
        description='UDP port for sending sensor data to av-simulation'
    )
    
    send_rate_arg = DeclareLaunchArgument(
        'send_rate',
        default_value='50.0',
        description='Sensor data send rate in Hz'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )
    
    # UDP Bridge Node
    udp_bridge_node = Node(
        package='offroad_gazebo_integration',
        executable='udp_bridge',
        name='udp_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'av_sim_ip': LaunchConfiguration('av_sim_ip'),
            'av_sim_command_port': LaunchConfiguration('av_sim_command_port'),
            'av_sim_sensor_port': LaunchConfiguration('av_sim_sensor_port'),
            'send_rate': LaunchConfiguration('send_rate'),
        }],
        remappings=[
            # Map to actual topics from inspection world
            ('/vehicle/cmd_vel', '/cmd_vel'),
            ('/vehicle/cmd_steering', '/cmd_steering'),
            ('/vehicle/odom', '/odom'),
            ('/vehicle/imu', '/imu/data'),
            ('/vehicle/gps', '/mavros/global_position/global'),
        ]
    )
    
    return LaunchDescription([
        av_sim_ip_arg,
        av_sim_command_port_arg,
        av_sim_sensor_port_arg,
        send_rate_arg,
        log_level_arg,
        udp_bridge_node,
    ])
