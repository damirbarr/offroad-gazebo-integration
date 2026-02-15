#!/usr/bin/env python3
"""
ROS-Gazebo Bridge Management

Manages the ros_gz_bridge for topic translation between ROS2 and Gazebo Transport
"""

import subprocess
from typing import List, Dict
import yaml


class BridgeConfig:
    """Configuration for a single bridge topic"""
    def __init__(self, 
                 ros_topic: str, 
                 gz_topic: str, 
                 ros_type: str, 
                 gz_type: str, 
                 direction: str = "bidirectional"):
        self.ros_topic = ros_topic
        self.gz_topic = gz_topic
        self.ros_type = ros_type
        self.gz_type = gz_type
        self.direction = direction  # "ros_to_gz", "gz_to_ros", or "bidirectional"
    
    def to_dict(self) -> Dict:
        """Convert to dictionary format for YAML export"""
        return {
            'topic_name': self.ros_topic,
            'gz_topic_name': self.gz_topic,
            'ros_type_name': self.ros_type,
            'gz_type_name': self.gz_type,
            'direction': self.direction
        }


class GazeboROSBridge:
    """
    Manages ROS-Gazebo topic bridging for off-road simulation.
    
    Provides convenient methods to configure and launch the ros_gz_bridge
    with appropriate topic mappings for vehicle control and sensor data.
    """
    
    def __init__(self):
        self.bridges: List[BridgeConfig] = []
        self.process = None
    
    def add_vehicle_control_bridges(self, vehicle_namespace: str = "/vehicle"):
        """
        Add standard vehicle control topic bridges.
        
        Args:
            vehicle_namespace: ROS namespace for vehicle topics
        """
        # Velocity command (ROS -> Gazebo)
        self.bridges.append(BridgeConfig(
            ros_topic=f"{vehicle_namespace}/cmd_vel",
            gz_topic=f"{vehicle_namespace}/cmd_vel",
            ros_type="geometry_msgs/msg/Twist",
            gz_type="gz.msgs.Twist",
            direction="ros_to_gz"
        ))
        
        # Steering command (ROS -> Gazebo)
        self.bridges.append(BridgeConfig(
            ros_topic=f"{vehicle_namespace}/cmd_steering",
            gz_topic=f"{vehicle_namespace}/cmd_steering",
            ros_type="std_msgs/msg/Float64",
            gz_type="gz.msgs.Double",
            direction="ros_to_gz"
        ))
    
    def add_vehicle_state_bridges(self, vehicle_namespace: str = "/vehicle"):
        """
        Add standard vehicle state topic bridges.
        
        Args:
            vehicle_namespace: ROS namespace for vehicle topics
        """
        # Odometry (Gazebo -> ROS)
        self.bridges.append(BridgeConfig(
            ros_topic=f"{vehicle_namespace}/odom",
            gz_topic=f"{vehicle_namespace}/odom",
            ros_type="nav_msgs/msg/Odometry",
            gz_type="gz.msgs.Odometry",
            direction="gz_to_ros"
        ))
    
    def add_sensor_bridges(self, vehicle_namespace: str = "/vehicle"):
        """
        Add standard sensor topic bridges.
        
        Args:
            vehicle_namespace: ROS namespace for vehicle topics
        """
        # IMU (Gazebo -> ROS)
        self.bridges.append(BridgeConfig(
            ros_topic=f"{vehicle_namespace}/imu",
            gz_topic=f"{vehicle_namespace}/imu",
            ros_type="sensor_msgs/msg/Imu",
            gz_type="gz.msgs.IMU",
            direction="gz_to_ros"
        ))
        
        # GPS (Gazebo -> ROS)
        self.bridges.append(BridgeConfig(
            ros_topic=f"{vehicle_namespace}/gps",
            gz_topic=f"{vehicle_namespace}/gps",
            ros_type="sensor_msgs/msg/NavSatFix",
            gz_type="gz.msgs.NavSat",
            direction="gz_to_ros"
        ))
        
        # LIDAR (Gazebo -> ROS)
        self.bridges.append(BridgeConfig(
            ros_topic=f"{vehicle_namespace}/lidar/points",
            gz_topic=f"{vehicle_namespace}/lidar/points",
            ros_type="sensor_msgs/msg/PointCloud2",
            gz_type="gz.msgs.PointCloudPacked",
            direction="gz_to_ros"
        ))
        
        # Camera (Gazebo -> ROS)
        self.bridges.append(BridgeConfig(
            ros_topic=f"{vehicle_namespace}/camera/image_raw",
            gz_topic=f"{vehicle_namespace}/camera/image_raw",
            ros_type="sensor_msgs/msg/Image",
            gz_type="gz.msgs.Image",
            direction="gz_to_ros"
        ))
    
    def add_clock_bridge(self):
        """Add simulation clock bridge for time synchronization"""
        self.bridges.append(BridgeConfig(
            ros_topic="/clock",
            gz_topic="/clock",
            ros_type="rosgraph_msgs/msg/Clock",
            gz_type="gz.msgs.Clock",
            direction="gz_to_ros"
        ))
    
    def configure_standard_bridges(self, vehicle_namespace: str = "/vehicle"):
        """
        Configure all standard bridges for off-road autonomy.
        
        Args:
            vehicle_namespace: ROS namespace for vehicle topics
        """
        self.add_vehicle_control_bridges(vehicle_namespace)
        self.add_vehicle_state_bridges(vehicle_namespace)
        self.add_sensor_bridges(vehicle_namespace)
        self.add_clock_bridge()
    
    def export_config(self, filepath: str):
        """
        Export bridge configuration to YAML file.
        
        Args:
            filepath: Path to output YAML file
        """
        config = [bridge.to_dict() for bridge in self.bridges]
        
        with open(filepath, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
    
    def launch(self, config_file: str = None):
        """
        Launch the ros_gz_bridge process.
        
        Args:
            config_file: Path to bridge configuration YAML file.
                        If None, uses inline configuration.
        
        Returns:
            Subprocess handle
        """
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
        
        if config_file:
            cmd.append(f"--ros-args")
            cmd.append(f"-p")
            cmd.append(f"config_file:={config_file}")
        else:
            # Build inline bridge specifications
            for bridge in self.bridges:
                spec = (f"{bridge.ros_topic}@{bridge.ros_type}"
                       f"[{bridge.gz_topic}@{bridge.gz_type}")
                cmd.append(spec)
        
        self.process = subprocess.Popen(cmd)
        return self.process
    
    def stop(self):
        """Stop the bridge process"""
        if self.process:
            self.process.terminate()
            self.process.wait()
            self.process = None


def main():
    """Example usage"""
    bridge = GazeboROSBridge()
    bridge.configure_standard_bridges(vehicle_namespace="/vehicle")
    bridge.export_config("/tmp/ros_gz_bridge.yaml")
    print("Bridge configuration exported to /tmp/ros_gz_bridge.yaml")


if __name__ == "__main__":
    main()
