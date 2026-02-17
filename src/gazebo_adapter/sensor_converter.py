#!/usr/bin/env python3
"""
Sensor Converter Node

Converts Gazebo sensor data to match av-simulation expected topics:
- IMU orientation -> GPS heading (Float64)
- GPS velocity -> TwistStamped
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped, Vector3
import math


class SensorConverter(Node):
    def __init__(self):
        super().__init__('sensor_converter')
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            10
        )
        
        # Publishers
        self.heading_pub = self.create_publisher(
            Float64,
            '/mavros/global_position/compass_hdg',
            10
        )
        
        self.gps_vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/global_position/raw/gps_vel',
            10
        )
        
        self.last_gps_msg = None
        
        self.get_logger().info('Sensor converter started')
        self.get_logger().info('  IMU -> Heading: /mavros/global_position/compass_hdg')
        self.get_logger().info('  GPS vel -> /mavros/global_position/raw/gps_vel')
    
    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle in radians"""
        # Calculate yaw from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def imu_callback(self, msg: Imu):
        """Convert IMU orientation to GPS heading"""
        # Extract quaternion
        q = msg.orientation
        
        # Convert to yaw (radians)
        yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        # Convert to degrees (0-360)
        heading_deg = (math.degrees(yaw_rad) + 360) % 360
        
        # Publish heading
        heading_msg = Float64()
        heading_msg.data = heading_deg
        self.heading_pub.publish(heading_msg)
    
    def gps_callback(self, msg: NavSatFix):
        """Store GPS message for velocity calculation"""
        self.last_gps_msg = msg
        
        # If GPS has covariance, we can extract velocity from it
        # For now, we'll publish zero velocity if no velocity data available
        # In a real GPS, velocity would come from the GPS receiver
        
        vel_msg = TwistStamped()
        vel_msg.header.stamp = msg.header.stamp
        vel_msg.header.frame_id = 'base_link'
        
        # TODO: Calculate velocity from position changes or get from GPS directly
        # For now, publish zero velocity
        vel_msg.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        vel_msg.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        
        self.gps_vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    converter = SensorConverter()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
