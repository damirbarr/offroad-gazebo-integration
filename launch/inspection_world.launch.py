#!/usr/bin/env python3
"""
Launch Gazebo with CPR inspection world.

This launch file starts Gazebo Sim with the CPR inspection world environment,
featuring a water table and inspection geometry.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for inspection world"""
    
    # Declare arguments
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
        default_value='heron',
        description='Vehicle model to spawn (use heron for water)'
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
        default_value='0.1',
        description='Vehicle spawn Z position (0.1 for water surface)'
    )
    
    # Get configuration
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Package paths
    pkg_share = FindPackageShare('offroad_gazebo_integration')
    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'inspection_world.world'
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
    
    return LaunchDescription([
        # Arguments
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
    ])
