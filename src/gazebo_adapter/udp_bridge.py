#!/usr/bin/env python3
"""
UDP Bridge for Gazebo-av-simulation integration (JSON protocol)

Receives JSON control commands from av-simulation's UdpAdapter via UDP.
Sends JSON sensor feedback back to av-simulation via UDP.
Communicates with Gazebo via ROS2 topics.
"""

import json
import math
import rclpy
from rclpy.node import Node
import socket
import threading
import time
from typing import Optional

from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


def quaternion_to_yaw(x, y, z, w):
    """Extract yaw (radians) from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class UdpBridge(Node):
    """
    UDP Bridge between av-simulation and Gazebo using JSON protocol.

    Receiving from av-simulation (port 9001):
        {"type":"control", "timestamp":…, "throttle":…, "steering":…,
         "gear":…, "turn_left":…, "turn_right":…, "high_beam":…,
         "low_beam":…, "horn":…, "hazard":…}

    Sending to av-simulation (port 9002):
        {"type":"feedback", "timestamp":…, "speed":…, "rpm":…,
         "gps":{"latitude":…, "longitude":…, "azimuth":…},
         "object_detected":false, "vehicle_state":…}
    """

    # Simple RPM estimation constants
    WHEEL_RADIUS = 0.35        # meters
    FINAL_DRIVE_RATIO = 3.5
    RPM_PER_SPEED = 60.0 / (2.0 * math.pi * WHEEL_RADIUS) * FINAL_DRIVE_RATIO

    def __init__(self):
        super().__init__('udp_bridge')

        # Parameters
        self.declare_parameter('av_sim_ip', '127.0.0.1')
        self.declare_parameter('av_sim_command_port', 9001)
        self.declare_parameter('av_sim_sensor_port', 9002)
        self.declare_parameter('send_rate', 50.0)
        self.declare_parameter('max_linear_speed', 10.0)
        self.declare_parameter('max_angular_speed', 2.0)

        av_sim_ip = self.get_parameter('av_sim_ip').value
        cmd_port = self.get_parameter('av_sim_command_port').value
        sensor_port = self.get_parameter('av_sim_sensor_port').value
        send_rate = self.get_parameter('send_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # UDP sockets
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind(('0.0.0.0', cmd_port))
        self.command_socket.settimeout(0.1)

        self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.av_sim_address = (av_sim_ip, sensor_port)

        # ROS publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/vehicle/cmd_vel', 10)

        # ROS subscribers
        self.create_subscription(Odometry, '/vehicle/odom', self.odom_callback, 10)
        self.create_subscription(NavSatFix, '/vehicle/gps', self.gps_callback, 10)

        # State
        self.latest_odom: Optional[Odometry] = None
        self.latest_gps: Optional[NavSatFix] = None
        self.vehicle_state = 1  # RemoteDriving
        self.state_lock = threading.Lock()

        # Receiver thread
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()

        # Sender timer
        self.send_timer = self.create_timer(1.0 / send_rate, self._send_sensor_data)

        self.get_logger().info(
            f'UDP Bridge (JSON) started: recv :{cmd_port}, send {av_sim_ip}:{sensor_port} @ {send_rate} Hz'
        )

    # ── Receive ──────────────────────────────────────────────

    def _receive_loop(self):
        while self.running and rclpy.ok():
            try:
                data, addr = self.command_socket.recvfrom(4096)
                # Log ALL incoming packets (debug level)
                self.get_logger().debug(
                    f'UDP RX from {addr[0]}:{addr[1]} - {len(data)} bytes: {data[:100]}'
                )
                self._process_command(data)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'UDP receive error: {e}')

    def _process_command(self, data: bytes):
        try:
            msg = json.loads(data.decode('utf-8'))
            self.get_logger().debug(f'JSON parsed OK: type={msg.get("type")}')
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            self.get_logger().warn(f'Invalid JSON command: {e} - raw data: {data}')
            return

        if msg.get('type') != 'control':
            self.get_logger().debug(f'Ignored non-control message: type={msg.get("type")}')
            return

        throttle = float(msg.get('throttle', 0.0))
        steering = float(msg.get('steering', 0.0))

        # Map to differential drive (tank steering)
        # throttle → forward/backward speed
        # steering → rotation rate (left/right wheels at different speeds)
        twist = Twist()
        twist.linear.x = throttle * self.max_linear_speed
        twist.angular.z = steering * self.max_angular_speed
        self.cmd_vel_pub.publish(twist)
        
        # Log received command for debugging
        self.get_logger().info(
            f'UDP CMD: throttle={throttle:.3f}, steering={steering:.3f} '
            f'→ vel={twist.linear.x:.3f} m/s, ang={twist.angular.z:.3f} rad/s'
        )

    # ── Send ─────────────────────────────────────────────────

    def _send_sensor_data(self):
        with self.state_lock:
            odom = self.latest_odom
            gps = self.latest_gps

        if odom is None:
            return

        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        speed = math.sqrt(vx * vx + vy * vy)
        rpm = int(abs(speed) * self.RPM_PER_SPEED)

        q = odom.pose.pose.orientation
        azimuth = math.degrees(quaternion_to_yaw(q.x, q.y, q.z, q.w))

        feedback = {
            'type': 'feedback',
            'timestamp': int(time.time() * 1000),
            'speed': round(speed, 3),
            'rpm': rpm,
            'gps': {
                'latitude': gps.latitude if gps else 0.0,
                'longitude': gps.longitude if gps else 0.0,
                'azimuth': round(azimuth, 3),
            },
            'object_detected': False,
            'vehicle_state': self.vehicle_state,
        }

        try:
            payload = json.dumps(feedback).encode('utf-8')
            self.sensor_socket.sendto(payload, self.av_sim_address)
            # Log feedback occasionally (every 2 seconds at 50Hz = every 100 messages)
            if not hasattr(self, '_feedback_counter'):
                self._feedback_counter = 0
            self._feedback_counter += 1
            if self._feedback_counter % 100 == 0:
                self.get_logger().info(
                    f'UDP FEEDBACK: speed={feedback["speed"]:.3f} m/s, '
                    f'rpm={feedback["rpm"]}, gps=({feedback["gps"]["latitude"]:.6f}, '
                    f'{feedback["gps"]["longitude"]:.6f})'
                )
        except Exception as e:
            self.get_logger().error(f'Sensor send error: {e}')

    # ── ROS callbacks ────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        with self.state_lock:
            self.latest_odom = msg

    def gps_callback(self, msg: NavSatFix):
        with self.state_lock:
            self.latest_gps = msg

    # ── Shutdown ─────────────────────────────────────────────

    def shutdown(self):
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
