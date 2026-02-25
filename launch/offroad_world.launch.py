#!/usr/bin/env python3
"""
Launch Gazebo with off-road terrain world.

Aligned with inspection_world.launch.py: same bridge topics, delayed spawn,
sensor_converter, and laserscan_to_pointcloud so the UDP stack works unchanged.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for off-road Gazebo world"""

    # Declare arguments (aligned with inspection_world.launch.py)
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='desert_terrain.sdf',
        description='World file name (e.g. desert_terrain.sdf)'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='desert_terrain',
        description='World name for spawn service (must match <world name=""> in SDF)'
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
        default_value='inspection_robot',
        description='Vehicle model to spawn (e.g. inspection_robot, offroad_4x4)'
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
    world_file_name = LaunchConfiguration('world_file')
    world_name = LaunchConfiguration('world_name')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vehicle_model = LaunchConfiguration('vehicle_model')
    vehicle_x = LaunchConfiguration('vehicle_x')
    vehicle_y = LaunchConfiguration('vehicle_y')
    vehicle_z = LaunchConfiguration('vehicle_z')

    # Package paths
    pkg_share = FindPackageShare('offroad_gazebo_integration')
    world_path = PathJoinSubstitution([
        pkg_share,
        'worlds',
        world_file_name,
    ])

    # Gazebo server (always runs)
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', world_path],
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

    # ROS-Gazebo bridge (delayed, same topics as inspection for UDP stack)
    bridge_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                    '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    '/mavros/global_position/global@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
                    '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                respawn=True
            )
        ]
    )

    # Spawn vehicle function (same pattern as inspection)
    def spawn_vehicle_func(context):
        model = context.launch_configurations['vehicle_model']
        wname = context.launch_configurations['world_name']
        x = context.launch_configurations['vehicle_x']
        y = context.launch_configurations['vehicle_y']
        z = context.launch_configurations['vehicle_z']

        spawn_cmd = ExecuteProcess(
            cmd=[
                'ign', 'service',
                '-s', f'/world/{wname}/create',
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

    spawn_vehicle = TimerAction(
        period=8.0,
        actions=[OpaqueFunction(function=spawn_vehicle_func)]
    )

    # Sensor converter (same as inspection)
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
        world_file_arg,
        world_name_arg,
        headless_arg,
        use_sim_time_arg,
        vehicle_model_arg,
        vehicle_x_arg,
        vehicle_y_arg,
        vehicle_z_arg,
        gazebo_server,
        gazebo_client,
        bridge_node,
        spawn_vehicle,
        sensor_converter,
        laserscan_converter,
    ])
