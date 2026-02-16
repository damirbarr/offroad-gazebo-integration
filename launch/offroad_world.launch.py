#!/usr/bin/env python3
"""
Launch Gazebo with off-road terrain world.

This launch file starts Gazebo Sim with a configured off-road environment.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for off-road Gazebo world"""
    
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='desert_terrain',
        description='World file name (without .sdf extension)'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run simulation without GUI'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    vehicle_model_arg = DeclareLaunchArgument(
        'vehicle_model',
        default_value='offroad_4x4',
        description='Vehicle model to spawn'
    )
    
    vehicle_x_arg = DeclareLaunchArgument(
        'vehicle_x',
        default_value='0.0',
        description='Vehicle spawn X position'
    )
    
    vehicle_y_arg = DeclareLaunchArgument(
        'vehicle_y',
        default_value='0.0',
        description='Vehicle spawn Y position'
    )
    
    vehicle_z_arg = DeclareLaunchArgument(
        'vehicle_z',
        default_value='0.5',
        description='Vehicle spawn Z position'
    )
    
    # Get configuration
    world_name = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Package paths
    pkg_share = FindPackageShare('offroad_gazebo_integration')
    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        [world_name, '.sdf']
    ])
    
    # Gazebo server (always runs)
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
        shell=False
    )
    
    # Gazebo client (GUI - only if not headless)
    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=LaunchConfigurationEquals('headless', 'false'),
        shell=False
    )
    
    # ROS-Gazebo bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Vehicle command
            '/vehicle/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/vehicle/cmd_steering@std_msgs/msg/Float64[gz.msgs.Double',
            # Vehicle state
            '/vehicle/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Sensors
            '/vehicle/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/vehicle/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/vehicle/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/vehicle/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Robot state publisher (for TF tree)
    # This would load the vehicle URDF and publish transforms
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{
    #         'robot_description': vehicle_urdf,
    #         'use_sim_time': use_sim_time
    #     }]
    # )
    
    return LaunchDescription([
        # Arguments
        world_arg,
        headless_arg,
        use_sim_time_arg,
        vehicle_model_arg,
        vehicle_x_arg,
        vehicle_y_arg,
        vehicle_z_arg,
        
        # Nodes and processes
        gazebo_server,
        gazebo_client,
        bridge_node,
        # robot_state_publisher,  # Uncomment when vehicle URDF is available
    ])
