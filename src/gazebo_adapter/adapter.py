#!/usr/bin/env python3
"""
Gazebo Simulation Adapter for Ottopia av-simulation framework

This adapter bridges Gazebo Sim with the av-simulation ros-adapter interface,
providing off-road terrain simulation capabilities.
"""

import rclpy
from rclpy.node import Node
from typing import Dict, Optional
import numpy as np

# ROS message types
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# Assuming these interfaces exist in av-simulation
# These would be imported from: from av_simulation.ros_adapter import ...
# For now, defining minimal interface contracts


class SimulationState:
    """Container for simulation state"""
    def __init__(self):
        self.timestamp: float = 0.0
        self.vehicle_pose: np.ndarray = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.vehicle_velocity: np.ndarray = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        self.is_valid: bool = True


class VehicleCommand:
    """Container for vehicle commands"""
    def __init__(self):
        self.linear_velocity: float = 0.0
        self.angular_velocity: float = 0.0
        self.steering_angle: float = 0.0
        self.timestamp: float = 0.0


class SensorData:
    """Container for sensor data"""
    def __init__(self):
        self.timestamp: float = 0.0
        self.imu: Optional[Dict] = None
        self.gps: Optional[Dict] = None
        self.lidar: Optional[np.ndarray] = None
        self.camera: Optional[np.ndarray] = None


class SimulationAdapter:
    """Base interface for simulation adapters (from av-simulation)"""
    def initialize(self, config: Dict) -> bool:
        raise NotImplementedError
    
    def reset(self) -> bool:
        raise NotImplementedError
    
    def step(self, dt: float) -> SimulationState:
        raise NotImplementedError
    
    def set_vehicle_command(self, cmd: VehicleCommand) -> bool:
        raise NotImplementedError
    
    def get_sensor_data(self) -> SensorData:
        raise NotImplementedError
    
    def shutdown(self) -> None:
        raise NotImplementedError


class GazeboAdapter(SimulationAdapter, Node):
    """
    Gazebo simulation adapter for av-simulation framework.
    
    Implements the SimulationAdapter interface and provides:
    - Vehicle state tracking
    - Sensor data aggregation
    - Command forwarding to Gazebo
    - Coordinate frame transformations
    """
    
    def __init__(self, config: Optional[Dict] = None):
        """Initialize the Gazebo adapter"""
        Node.__init__(self, 'gazebo_adapter')
        
        # Configuration
        self.config = config or {}
        self.vehicle_namespace = self.config.get('vehicle_namespace', '/vehicle')
        self.update_rate = self.config.get('update_rate', 100.0)  # Hz
        
        # State tracking
        self._last_odom: Optional[Odometry] = None
        self._last_imu: Optional[Imu] = None
        self._last_gps: Optional[NavSatFix] = None
        self._last_lidar: Optional[PointCloud2] = None
        self._last_camera: Optional[Image] = None
        self._simulation_time = 0.0
        
        # Publishers (commands to Gazebo)
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            f'{self.vehicle_namespace}/cmd_vel',
            10
        )
        self._cmd_steering_pub = self.create_publisher(
            Float64,
            f'{self.vehicle_namespace}/cmd_steering',
            10
        )
        
        # Subscribers (state from Gazebo)
        self._odom_sub = self.create_subscription(
            Odometry,
            f'{self.vehicle_namespace}/odom',
            self._odom_callback,
            10
        )
        self._imu_sub = self.create_subscription(
            Imu,
            f'{self.vehicle_namespace}/imu',
            self._imu_callback,
            10
        )
        self._gps_sub = self.create_subscription(
            NavSatFix,
            f'{self.vehicle_namespace}/gps',
            self._gps_callback,
            10
        )
        self._lidar_sub = self.create_subscription(
            PointCloud2,
            f'{self.vehicle_namespace}/lidar/points',
            self._lidar_callback,
            10
        )
        self._camera_sub = self.create_subscription(
            Image,
            f'{self.vehicle_namespace}/camera/image_raw',
            self._camera_callback,
            10
        )
        
        self._initialized = False
        self.get_logger().info('Gazebo adapter created')
    
    def initialize(self, config: Dict) -> bool:
        """
        Initialize the adapter with configuration.
        
        Args:
            config: Configuration dictionary
            
        Returns:
            True if initialization successful
        """
        try:
            self.config.update(config)
            self.get_logger().info(f'Initializing Gazebo adapter with config: {config}')
            
            # Wait for first odometry message to ensure Gazebo is running
            self.get_logger().info('Waiting for Gazebo simulation to start...')
            timeout = 30.0  # seconds
            start_time = self.get_clock().now()
            
            while self._last_odom is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > timeout:
                    self.get_logger().error('Timeout waiting for Gazebo odometry')
                    return False
            
            self._initialized = True
            self.get_logger().info('Gazebo adapter initialized successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Initialization failed: {e}')
            return False
    
    def reset(self) -> bool:
        """
        Reset the simulation to initial state.
        
        Returns:
            True if reset successful
        """
        try:
            self.get_logger().info('Resetting simulation')
            
            # Clear cached state
            self._last_odom = None
            self._last_imu = None
            self._last_gps = None
            self._last_lidar = None
            self._last_camera = None
            self._simulation_time = 0.0
            
            # TODO: Call Gazebo reset service when available
            # For now, send zero commands
            zero_cmd = Twist()
            self._cmd_vel_pub.publish(zero_cmd)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Reset failed: {e}')
            return False
    
    def step(self, dt: float) -> SimulationState:
        """
        Step the simulation forward by dt seconds.
        
        Args:
            dt: Time step in seconds
            
        Returns:
            Current simulation state
        """
        # Process ROS callbacks
        rclpy.spin_once(self, timeout_sec=0.001)
        
        self._simulation_time += dt
        
        # Build state from latest sensor data
        state = SimulationState()
        state.timestamp = self._simulation_time
        
        if self._last_odom is not None:
            # Extract pose
            pose = self._last_odom.pose.pose
            state.vehicle_pose[0] = pose.position.x
            state.vehicle_pose[1] = pose.position.y
            state.vehicle_pose[2] = pose.position.z
            
            # Convert quaternion to euler angles
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w
            
            # Roll (x-axis rotation)
            sinr_cosp = 2 * (qw * qx + qy * qz)
            cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
            state.vehicle_pose[3] = np.arctan2(sinr_cosp, cosr_cosp)
            
            # Pitch (y-axis rotation)
            sinp = 2 * (qw * qy - qz * qx)
            if abs(sinp) >= 1:
                state.vehicle_pose[4] = np.copysign(np.pi / 2, sinp)
            else:
                state.vehicle_pose[4] = np.arcsin(sinp)
            
            # Yaw (z-axis rotation)
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            state.vehicle_pose[5] = np.arctan2(siny_cosp, cosy_cosp)
            
            # Extract velocity
            twist = self._last_odom.twist.twist
            state.vehicle_velocity[0] = twist.linear.x
            state.vehicle_velocity[1] = twist.linear.y
            state.vehicle_velocity[2] = twist.linear.z
            state.vehicle_velocity[3] = twist.angular.x
            state.vehicle_velocity[4] = twist.angular.y
            state.vehicle_velocity[5] = twist.angular.z
        else:
            state.is_valid = False
        
        return state
    
    def set_vehicle_command(self, cmd: VehicleCommand) -> bool:
        """
        Send vehicle command to simulation.
        
        Args:
            cmd: Vehicle command
            
        Returns:
            True if command sent successfully
        """
        try:
            # Publish velocity command
            twist = Twist()
            twist.linear.x = cmd.linear_velocity
            twist.angular.z = cmd.angular_velocity
            self._cmd_vel_pub.publish(twist)
            
            # Publish steering command
            steering = Float64()
            steering.data = cmd.steering_angle
            self._cmd_steering_pub.publish(steering)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            return False
    
    def get_sensor_data(self) -> SensorData:
        """
        Get latest sensor data from simulation.
        
        Returns:
            Aggregated sensor data
        """
        data = SensorData()
        data.timestamp = self._simulation_time
        
        # IMU data
        if self._last_imu is not None:
            data.imu = {
                'linear_acceleration': np.array([
                    self._last_imu.linear_acceleration.x,
                    self._last_imu.linear_acceleration.y,
                    self._last_imu.linear_acceleration.z
                ]),
                'angular_velocity': np.array([
                    self._last_imu.angular_velocity.x,
                    self._last_imu.angular_velocity.y,
                    self._last_imu.angular_velocity.z
                ])
            }
        
        # GPS data
        if self._last_gps is not None:
            data.gps = {
                'latitude': self._last_gps.latitude,
                'longitude': self._last_gps.longitude,
                'altitude': self._last_gps.altitude
            }
        
        # LIDAR data
        if self._last_lidar is not None:
            # TODO: Parse PointCloud2 message to numpy array
            data.lidar = None  # Placeholder
        
        # Camera data
        if self._last_camera is not None:
            # TODO: Convert Image message to numpy array
            data.camera = None  # Placeholder
        
        return data
    
    def shutdown(self) -> None:
        """Shutdown the adapter and cleanup resources"""
        self.get_logger().info('Shutting down Gazebo adapter')
        
        # Send zero commands
        zero_cmd = Twist()
        self._cmd_vel_pub.publish(zero_cmd)
        
        # Destroy node
        self.destroy_node()
    
    # Callbacks
    
    def _odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        self._last_odom = msg
    
    def _imu_callback(self, msg: Imu):
        """Handle IMU updates"""
        self._last_imu = msg
    
    def _gps_callback(self, msg: NavSatFix):
        """Handle GPS updates"""
        self._last_gps = msg
    
    def _lidar_callback(self, msg: PointCloud2):
        """Handle LIDAR updates"""
        self._last_lidar = msg
    
    def _camera_callback(self, msg: Image):
        """Handle camera updates"""
        self._last_camera = msg


def main():
    """Standalone entry point for testing"""
    rclpy.init()
    
    adapter = GazeboAdapter()
    config = {
        'vehicle_namespace': '/vehicle',
        'update_rate': 100.0
    }
    
    if adapter.initialize(config):
        print('Adapter initialized, spinning...')
        try:
            rclpy.spin(adapter)
        except KeyboardInterrupt:
            pass
        finally:
            adapter.shutdown()
            rclpy.shutdown()
    else:
        print('Failed to initialize adapter')


if __name__ == '__main__':
    main()
