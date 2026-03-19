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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
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
    
    # Get configuration
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vehicle_model = LaunchConfiguration('vehicle_model')
    vehicle_x = LaunchConfiguration('vehicle_x')
    vehicle_y = LaunchConfiguration('vehicle_y')
    vehicle_z = LaunchConfiguration('vehicle_z')
    
    # Package paths
    pkg_share = FindPackageShare('offroad_gazebo_integration')
    plugin_path = PathJoinSubstitution([
        FindPackagePrefix('offroad_gazebo_integration'),
        'lib',
    ])
    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'inspection_world.world'
    ])

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
    # Config chosen by vehicle_model: inspection_robot uses lidar_link, prius/ackermann use base_link
    def make_bridge_node(context):
        from ament_index_python.packages import get_package_share_directory
        import os
        model = context.launch_configurations.get('vehicle_model', 'inspection_robot')
        config_name = 'ros_gz_bridge_prius.yaml' if model in ('prius_vehicle', 'ackermann_vehicle') else 'ros_gz_bridge.yaml'
        config_path = os.path.join(get_package_share_directory('offroad_gazebo_integration'), 'config', config_name)
        use_sim_val = context.launch_configurations.get('use_sim_time', 'true')
        use_sim_time_bool = use_sim_val in ('true', '1', 'yes')
        return [
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=['--ros-args', '-p', f'config_file:={config_path}'],
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time_bool}],
                        respawn=True
                    )
                ]
            )
        ]

    bridge_node = OpaqueFunction(function=make_bridge_node)
    
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

    # Static TF: base_link -> velodyne (lidar frame for RViz)
    # Pose varies by vehicle: inspection (0.3,0,0.35), prius (0,0,1.55), ackermann (0,0,0.5)
    def make_static_tf(context):
        model = context.launch_configurations.get('vehicle_model', 'inspection_robot')
        poses = {
            'inspection_robot': ('0.3', '0', '0.35'),
            'prius_vehicle': ('0', '0', '1.55'),
            'ackermann_vehicle': ('0', '0', '0.5'),
        }
        x, y, z = poses.get(model, poses['inspection_robot'])
        use_sim_val = context.launch_configurations.get('use_sim_time', 'true')
        use_sim_time_bool = use_sim_val in ('true', '1', 'yes')
        return [
            TimerAction(
                period=9.0,
                actions=[
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='base_link_to_velodyne',
                        arguments=[x, y, z, '0', '0', '0', 'base_link', 'velodyne'],
                        parameters=[{'use_sim_time': use_sim_time_bool}]
                    )
                ]
            )
        ]

    static_tf = OpaqueFunction(function=make_static_tf)
    
    return LaunchDescription([
        # Arguments
        headless_arg,
        use_sim_time_arg,
        vehicle_model_arg,
        vehicle_x_arg,
        vehicle_y_arg,
        vehicle_z_arg,
        ignition_plugin_path,
        gz_plugin_path,
        
        # Nodes and processes
        gazebo_server,
        gazebo_client,
        spawn_vehicle,
        static_tf,
        bridge_node,
        sensor_converter,
        topic_relay,
    ])
