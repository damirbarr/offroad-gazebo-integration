#!/usr/bin/env python3
"""
UDP Bridge for Gazebo-av-simulation integration

Receives vehicle commands from av-simulation UDP adapter via UDP
Sends sensor data back to av-simulation via UDP
Communicates with Gazebo via ROS topics
"""

import rclpy
from rclpy.node import Node
import socket
import struct
import threading
import time
from typing import Optional
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class UdpBridge(Node):
    """
    UDP Bridge between av-simulation and Gazebo
    
    Protocol:
    - Receives control commands on UDP port (default 9001)
    - Sends sensor data on UDP port (default 9002)
    
    Message Format (binary, little-endian):
    
    Control Command (from av-simulation):
    - uint32: message_type (0x01 = control)
    - uint64: timestamp_ns
    - float: throttle (-1.0 to 1.0)
    - float: brake (0.0 to 1.0)
    - float: steering (-1.0 to 1.0)
    - float: gear (0=park, 1=reverse, 2=neutral, 3=drive)
    
    Sensor Data (to av-simulation):
    - uint32: message_type (0x02 = sensor_data)
    - uint64: timestamp_ns
    - float[3]: position (x, y, z)
    - float[3]: velocity (vx, vy, vz)
    - float[4]: orientation_quat (x, y, z, w)
    - float[3]: angular_velocity (wx, wy, wz)
    - float[3]: linear_acceleration (ax, ay, az)
    - float[3]: gps (lat, lon, alt)
    """
    
    # Message types
    MSG_CONTROL = 0x01
    MSG_SENSOR = 0x02
    MSG_HEARTBEAT = 0x03
    
    # Pack formats
    CONTROL_FMT = '<IQffff'  # type, timestamp, throttle, brake, steering, gear
    CONTROL_SIZE = struct.calcsize(CONTROL_FMT)
    
    SENSOR_FMT = '<IQ' + 'fff' + 'fff' + 'ffff' + 'fff' + 'fff' + 'fff'
    # type, ts, pos(3), vel(3), quat(4), ang_vel(3), lin_acc(3), gps(3)
    SENSOR_SIZE = struct.calcsize(SENSOR_FMT)
    
    def __init__(self):
        super().__init__('udp_bridge')
        
        # Parameters
        self.declare_parameter('av_sim_ip', '127.0.0.1')
        self.declare_parameter('av_sim_command_port', 9001)
        self.declare_parameter('av_sim_sensor_port', 9002)
        self.declare_parameter('send_rate', 50.0)  # Hz
        
        av_sim_ip = self.get_parameter('av_sim_ip').value
        cmd_port = self.get_parameter('av_sim_command_port').value
        sensor_port = self.get_parameter('av_sim_sensor_port').value
        send_rate = self.get_parameter('send_rate').value
        
        # UDP sockets
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind(('0.0.0.0', cmd_port))
        self.command_socket.settimeout(0.1)  # Non-blocking with timeout
        
        self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.av_sim_address = (av_sim_ip, sensor_port)
        
        # ROS publishers (commands to Gazebo)
        self.cmd_vel_pub = self.create_publisher(Twist, '/vehicle/cmd_vel', 10)
        self.cmd_steering_pub = self.create_publisher(Float64, '/vehicle/cmd_steering', 10)
        
        # ROS subscribers (sensor data from Gazebo)
        self.create_subscription(Odometry, '/vehicle/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/vehicle/imu', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/vehicle/gps', self.gps_callback, 10)
        
        # State storage
        self.latest_odom: Optional[Odometry] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_gps: Optional[NavSatFix] = None
        self.state_lock = threading.Lock()
        
        # Start UDP receiver thread
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        
        # Start sensor send timer
        send_period = 1.0 / send_rate
        self.send_timer = self.create_timer(send_period, self._send_sensor_data)
        
        self.get_logger().info(f'UDP Bridge started:')
        self.get_logger().info(f'  Receiving commands on 0.0.0.0:{cmd_port}')
        self.get_logger().info(f'  Sending sensors to {av_sim_ip}:{sensor_port}')
        self.get_logger().info(f'  Send rate: {send_rate} Hz')
    
    def _receive_loop(self):
        """Receive and process UDP commands"""
        while self.running and rclpy.ok():
            try:
                data, addr = self.command_socket.recvfrom(4096)
                self._process_command(data)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'UDP receive error: {e}')
    
    def _process_command(self, data: bytes):
        """Parse and execute control command"""
        if len(data) < self.CONTROL_SIZE:
            self.get_logger().warn(f'Invalid command size: {len(data)}')
            return
        
        try:
            msg_type, timestamp, throttle, brake, steering, gear = struct.unpack(
                self.CONTROL_FMT, data[:self.CONTROL_SIZE]
            )
            
            if msg_type != self.MSG_CONTROL:
                return
            
            # Convert to ROS commands
            # Throttle/brake → linear velocity
            linear_velocity = throttle if abs(throttle) > abs(brake) else -brake
            linear_velocity *= 10.0  # Scale to m/s (adjust as needed)
            
            # Steering → angular velocity (simple conversion)
            angular_velocity = steering * 0.5  # Scale factor
            
            # Publish Twist
            twist = Twist()
            twist.linear.x = float(linear_velocity)
            twist.angular.z = float(angular_velocity)
            self.cmd_vel_pub.publish(twist)
            
            # Publish steering angle
            steering_msg = Float64()
            steering_msg.data = float(steering)
            self.cmd_steering_pub.publish(steering_msg)
            
        except Exception as e:
            self.get_logger().error(f'Command processing error: {e}')
    
    def _send_sensor_data(self):
        """Send sensor data to av-simulation"""
        with self.state_lock:
            if self.latest_odom is None or self.latest_imu is None:
                return  # Wait until we have data
            
            # Extract data
            pos = self.latest_odom.pose.pose.position
            vel = self.latest_odom.twist.twist.linear
            quat = self.latest_odom.pose.pose.orientation
            ang_vel = self.latest_imu.angular_velocity
            lin_acc = self.latest_imu.linear_acceleration
            
            if self.latest_gps:
                gps_lat = self.latest_gps.latitude
                gps_lon = self.latest_gps.longitude
                gps_alt = self.latest_gps.altitude
            else:
                gps_lat = gps_lon = gps_alt = 0.0
        
        try:
            # Pack sensor data
            timestamp_ns = int(time.time() * 1e9)
            data = struct.pack(
                self.SENSOR_FMT,
                self.MSG_SENSOR,
                timestamp_ns,
                pos.x, pos.y, pos.z,
                vel.x, vel.y, vel.z,
                quat.x, quat.y, quat.z, quat.w,
                ang_vel.x, ang_vel.y, ang_vel.z,
                lin_acc.x, lin_acc.y, lin_acc.z,
                gps_lat, gps_lon, gps_alt
            )
            
            # Send to av-simulation
            self.sensor_socket.sendto(data, self.av_sim_address)
            
        except Exception as e:
            self.get_logger().error(f'Sensor send error: {e}')
    
    def odom_callback(self, msg: Odometry):
        with self.state_lock:
            self.latest_odom = msg
    
    def imu_callback(self, msg: Imu):
        with self.state_lock:
            self.latest_imu = msg
    
    def gps_callback(self, msg: NavSatFix):
        with self.state_lock:
            self.latest_gps = msg
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        if hasattr(self, 'receive_thread'):
            self.receive_thread.join(timeout=1.0)
        self.command_socket.close()
        self.sensor_socket.close()


def main(args=None):
    rclpy.init(args=args)
    bridge = UdpBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
