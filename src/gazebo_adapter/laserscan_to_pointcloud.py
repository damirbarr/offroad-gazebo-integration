#!/usr/bin/env python3
"""
LaserScan to PointCloud2 Converter

Converts sensor_msgs/LaserScan to sensor_msgs/PointCloud2 for Velodyne compatibility
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np
import struct


class LaserScanToPointCloud(Node):
    def __init__(self):
        super().__init__('laserscan_to_pointcloud')

        # Static transform for RViz: define velodyne frame at origin of odom
        self.static_broadcaster = StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'odom'
        static_tf.child_frame_id = 'velodyne'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_tf)

        # Subscriber: bridge 3D gpu_lidar cloud to velodyne_points
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.cloud_callback,
            10
        )

        # Publisher (API-compatible topic)
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/velodyne_points',
            10
        )

        self.get_logger().info('PointCloud relay started')
        self.get_logger().info('  Input: /lidar/points (PointCloud2 from gpu_lidar)')
        self.get_logger().info('  Output: /velodyne_points (PointCloud2, frame velodyne)')

    def cloud_callback(self, cloud_msg: PointCloud2):
        """Relay gpu_lidar point cloud to /velodyne_points with velodyne frame"""
        out = PointCloud2()
        out.header = cloud_msg.header
        out.header.frame_id = 'velodyne'
        out.height = cloud_msg.height
        out.width = cloud_msg.width
        out.fields = cloud_msg.fields
        out.is_bigendian = cloud_msg.is_bigendian
        out.point_step = cloud_msg.point_step
        out.row_step = cloud_msg.row_step
        out.is_dense = cloud_msg.is_dense
        out.data = cloud_msg.data

        self.cloud_pub.publish(out)


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
