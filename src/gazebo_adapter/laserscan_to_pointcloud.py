#!/usr/bin/env python3
"""
LaserScan to PointCloud2 Converter

Converts sensor_msgs/LaserScan to sensor_msgs/PointCloud2 for Velodyne compatibility
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct


class LaserScanToPointCloud(Node):
    def __init__(self):
        super().__init__('laserscan_to_pointcloud')
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.scan_callback,
            10
        )
        
        # Publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/velodyne_points',
            10
        )
        
        self.get_logger().info('LaserScan to PointCloud2 converter started')
        self.get_logger().info('  Input: /lidar (LaserScan)')
        self.get_logger().info('  Output: /velodyne_points (PointCloud2)')
    
    def scan_callback(self, scan_msg: LaserScan):
        """Convert LaserScan to PointCloud2"""
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header = scan_msg.header
        cloud_msg.header.frame_id = 'velodyne'
        
        # Define point cloud fields (x, y, z, intensity)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 fields * 4 bytes each
        cloud_msg.is_dense = True
        
        # Convert scan to points
        points = []
        angle = scan_msg.angle_min
        
        for i, r in enumerate(scan_msg.ranges):
            # Skip invalid points
            if r < scan_msg.range_min or r > scan_msg.range_max:
                angle += scan_msg.angle_increment
                continue
            
            # Convert polar to Cartesian
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            z = 0.0  # 2D scan in horizontal plane
            
            # Get intensity if available
            intensity = scan_msg.intensities[i] if len(scan_msg.intensities) > i else 0.0
            
            # Pack point data
            points.append(struct.pack('ffff', x, y, z, intensity))
            
            angle += scan_msg.angle_increment
        
        # Set cloud data
        cloud_msg.width = len(points)
        cloud_msg.height = 1
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = b''.join(points)
        
        # Publish
        self.cloud_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    converter = LaserScanToPointCloud()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
