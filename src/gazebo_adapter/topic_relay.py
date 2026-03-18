#!/usr/bin/env python3
"""
Relay raw simulator topics to the stable vehicle-facing contract.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from std_msgs.msg import Int32


class TopicRelay(Node):
    def __init__(self):
        super().__init__('topic_relay')

        self._publishers = []

        self._add_relay(Twist, '/vehicle/cmd_vel', '/cmd_vel')
        self._add_relay(Vector3, '/vehicle/cmd_drive', '/cmd_drive')
        self._add_relay(Int32, '/vehicle/cmd_gear', '/cmd_gear')

        self._add_relay(Odometry, '/odom', '/vehicle/odom')
        self._add_relay(Odometry, '/odom', '/ego/odometry')
        self._add_relay(Imu, '/imu/data', '/vehicle/imu')
        self._add_relay(NavSatFix, '/mavros/global_position/global', '/vehicle/gps')
        self._add_relay(PointCloud2, '/velodyne_points', '/vehicle/lidar/points')

        self.get_logger().info('Topic relay started')
        self.get_logger().info('  /vehicle/cmd_vel -> /cmd_vel')
        self.get_logger().info('  /vehicle/cmd_drive -> /cmd_drive')
        self.get_logger().info('  /vehicle/cmd_gear -> /cmd_gear')
        self.get_logger().info('  /odom -> /vehicle/odom, /ego/odometry')
        self.get_logger().info('  /imu/data -> /vehicle/imu')
        self.get_logger().info('  /mavros/global_position/global -> /vehicle/gps')
        self.get_logger().info('  /velodyne_points -> /vehicle/lidar/points')

    def _add_relay(self, msg_type, source_topic, target_topic):
        publisher = self.create_publisher(msg_type, target_topic, 10)
        self._publishers.append(publisher)

        def callback(msg, relay_publisher=publisher):
            relay_publisher.publish(msg)

        self.create_subscription(msg_type, source_topic, callback, 10)


def main(args=None):
    rclpy.init(args=args)
    relay = TopicRelay()

    try:
        rclpy.spin(relay)
    except KeyboardInterrupt:
        pass
    finally:
        relay.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
