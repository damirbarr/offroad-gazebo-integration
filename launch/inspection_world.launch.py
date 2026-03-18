#!/usr/bin/env python3
"""
Launch Gazebo with CPR inspection world.

This launch file starts Gazebo Sim with the CPR inspection world environment,
featuring a water table and inspection geometry.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


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
        description='Vehicle model to spawn (inspection_robot, prius_vehicle, simple_boat, or custom model)'
    )
    
    vehicle_x_arg = DeclareLaunchArgument(
        'vehicle_x',
        default_value='-15.0',
        description='Vehicle spawn X position (default keeps land vehicles clear of the water table)'
    )
    
    vehicle_y_arg = DeclareLaunchArgument(
        'vehicle_y',
        default_value='0.0',
        description='Vehicle spawn Y position'
    )
    
    vehicle_z_arg = DeclareLaunchArgument(
        'vehicle_z',
        default_value='0.5',
        description='Vehicle spawn Z position above the ground plane'
    )

    enable_video_streaming_arg = DeclareLaunchArgument(
        'enable_video_streaming',
        default_value='true',
        description='Enable the Prius six-camera RTP streamer for Linux Player'
    )

    linux_player_ip_arg = DeclareLaunchArgument(
        'linux_player_ip',
        default_value='127.0.0.1',
        description='Destination IP for Linux Player RTP camera streams'
    )
    
    # Get configuration
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vehicle_model = LaunchConfiguration('vehicle_model')
    vehicle_x = LaunchConfiguration('vehicle_x')
    vehicle_y = LaunchConfiguration('vehicle_y')
    vehicle_z = LaunchConfiguration('vehicle_z')
    enable_video_streaming = LaunchConfiguration('enable_video_streaming')
    linux_player_ip = LaunchConfiguration('linux_player_ip')
    
    # Package paths
    pkg_share = FindPackageShare('offroad_gazebo_integration')
    plugin_path = PathJoinSubstitution([
        FindPackagePrefix('offroad_gazebo_integration'),
        'lib',
    ])
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'simulation.yaml'
    ])
    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'inspection_world.world'
    ])

    camera_bridge_arguments = [
        '/camera/main/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/left_side/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/left_mirror/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/right_side/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/right_mirror/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/rear/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
    ]

    ignition_plugin_path = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        [plugin_path, ':', EnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default_value='')],
    )

    gz_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        [plugin_path, ':', EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')],
    )
    
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
                    # Vehicle command topic shared by tank and Ackermann models
                    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    '/cmd_drive@geometry_msgs/msg/Vector3]ignition.msgs.Vector3d',
                    '/cmd_gear@std_msgs/msg/Int32]ignition.msgs.Int32',
                    # Odometry
                    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                    # IMU (for heading conversion)
                    '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    # GPS
                    '/mavros/global_position/global@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
                    # LiDAR - keep as intermediate topic
                    '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                ] + camera_bridge_arguments,
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

    # Topic relay for stable /vehicle/* and /ego/odometry aliases
    topic_relay = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='offroad_gazebo_integration',
                executable='topic_relay',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    video_streamer = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='offroad_gazebo_integration',
                executable='video_streamer',
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'",
                        vehicle_model,
                        "' == 'prius_vehicle' and '",
                        enable_video_streaming,
                        "'.lower() in ['true', '1', 'yes']",
                    ])
                ),
                parameters=[
                    config_file,
                    {
                        'use_sim_time': use_sim_time,
                        'video_streaming.enabled': enable_video_streaming,
                        'video_streaming.destination_host': linux_player_ip,
                    }
                ]
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
        enable_video_streaming_arg,
        linux_player_ip_arg,
        ignition_plugin_path,
        gz_plugin_path,
        
        # Nodes and processes
        gazebo_server,
        gazebo_client,
        bridge_node,
        spawn_vehicle,
        sensor_converter,
        laserscan_converter,
        topic_relay,
        video_streamer,
    ])
