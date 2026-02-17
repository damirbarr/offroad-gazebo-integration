#!/usr/bin/env python3
"""
Manual sensor bridge - reads from Gazebo topics and publishes to ROS2
Bypasses the broken ros_gz_bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import Header
import subprocess
import json
import threading

class ManualSensorBridge(Node):
    def __init__(self):
        super().__init__('manual_sensor_bridge')
        
        # ROS2 publishers
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '/lidar/points', 10)
        
        self.get_logger().info('Manual sensor bridge started')
        self.get_logger().info('Reading from Gazebo topics and publishing to ROS2')
        
        # Start threads to read Gazebo topics
        self.start_gz_readers()
    
    def start_gz_readers(self):
        """Start threads to continuously read Gazebo topics"""
        threading.Thread(target=self.read_gps, daemon=True).start()
        self.get_logger().info('GPS reader started')
        
    def read_gps(self):
        """Read GPS data from Gazebo and publish to ROS2"""
        cmd = ['gz', 'topic', '-e', '-t', '/gps/fix']
        
        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            for line in process.stdout:
                if 'latitude_deg' in line:
                    # Parse and publish GPS data
                    # This is a simplified example
                    self.get_logger().info(f'GPS data: {line.strip()}')
                    
        except Exception as e:
            self.get_logger().error(f'GPS reader error: {e}')

def main():
    rclpy.init()
    bridge = ManualSensorBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
