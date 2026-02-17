#!/usr/bin/env python3
"""
Launch Gazebo with CPR inspection world.

This launch file starts Gazebo Sim with the CPR inspection world environment,
featuring a water table and inspection geometry.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
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
        default_value='inspection_robot',
        description='Vehicle model to spawn (inspection_robot, simple_boat, or custom model)'
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
        default_value='2.0',
        description='Vehicle spawn Z position (higher to ensure it lands on platform)'
    )
    
    # Get configuration
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vehicle_model = LaunchConfiguration('vehicle_model')
    vehicle_x = LaunchConfiguration('vehicle_x')
    vehicle_y = LaunchConfiguration('vehicle_y')
    vehicle_z = LaunchConfiguration('vehicle_z')
    
    # Package paths
    pkg_share = FindPackageShare('offroad_gazebo_integration')
    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'inspection_world.world'
    ])
    
    # Gazebo server (always runs)
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', world_file],
        output='screen',
        shell=False
    )
    
    # Gazebo client (GUI - only if not headless)
    gazebo_client = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen',
        condition=LaunchConfigurationEquals('headless', 'false'),
        shell=False
    )
    
    # ROS-Gazebo bridge (delayed to start after robot spawns)
    bridge_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    # Clock
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    # Robot command (differential drive)  
                    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    # Odometry
                    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                    # IMU (for heading conversion)
                    '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    # GPS
                    '/mavros/global_position/global@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
                    # LiDAR - keep as intermediate topic
                    '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                respawn=True
            )
        ]
    )
    
    # Spawn vehicle function
    def spawn_vehicle_func(context):
        model = context.launch_configurations['vehicle_model']
        x = context.launch_configurations['vehicle_x']
        y = context.launch_configurations['vehicle_y']
        z = context.launch_configurations['vehicle_z']
        
        spawn_cmd = ExecuteProcess(
            cmd=[
                'ign', 'service',
                '-s', '/world/inspection_world/create',
                '--reqtype', 'ignition.msgs.EntityFactory',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '30000',
                '--req',
                f'sdf_filename: "model://{model}", name: "vehicle", pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
            ],
            output='screen',
            shell=False
        )
        return [spawn_cmd]
    
    # Spawn vehicle (delayed to ensure Gazebo is ready)
    spawn_vehicle = TimerAction(
        period=8.0,
        actions=[OpaqueFunction(function=spawn_vehicle_func)]
    )
    
    # Sensor converter node (converts IMU to heading, GPS to velocity)
    sensor_converter = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='offroad_gazebo_integration',
                executable='sensor_converter',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # LaserScan to PointCloud2 converter
    laserscan_converter = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='offroad_gazebo_integration',
                executable='laserscan_to_pointcloud',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
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
        spawn_vehicle,
        sensor_converter,
        laserscan_converter,
    ])
