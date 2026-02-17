#!/usr/bin/env python3
"""
Keyboard Teleoperation for Inspection Robot

Controls the robot using keyboard input:
- W/S: Forward/Backward
- A/D: Turn Left/Right
- Space: Stop
- Q: Quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.linear_increment = 0.1
        self.angular_increment = 0.1
        
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.get_logger().info('Keyboard Teleop Started!')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Controls:')
        self.get_logger().info('  W/S     - Forward/Backward')
        self.get_logger().info('  A/D     - Turn Left/Right')
        self.get_logger().info('  Q/E     - Increase/Decrease speed')
        self.get_logger().info('  Space   - Stop')
        self.get_logger().info('  X       - Quit')
        self.get_logger().info('=' * 50)
        self.print_status()
    
    def print_status(self):
        """Print current velocity and speed settings"""
        self.get_logger().info(f'Linear: {self.current_linear:.2f} m/s | Angular: {self.current_angular:.2f} rad/s')
        self.get_logger().info(f'Max speeds - Linear: {self.linear_speed:.2f} m/s | Angular: {self.angular_speed:.2f} rad/s')
    
    def get_key(self):
        """Get keyboard input (non-blocking)"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None
    
    def publish_velocity(self):
        """Publish current velocity command"""
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.cmd_pub.publish(msg)
        
        # Debug output every few publishes
        if not hasattr(self, '_publish_count'):
            self._publish_count = 0
        self._publish_count += 1
        
        if self._publish_count % 10 == 0:
            self.get_logger().info(f'Publishing: linear={self.current_linear:.2f}, angular={self.current_angular:.2f}')
    
    def run(self):
        """Main control loop"""
        # Set terminal to raw mode for character-by-character input
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            
            rate = self.create_rate(10)  # 10 Hz
            
            while rclpy.ok():
                key = self.get_key()
                
                if key:
                    if key.lower() == 'w':
                        self.current_linear = min(self.current_linear + self.linear_increment, self.linear_speed)
                        self.get_logger().info(f'Forward: {self.current_linear:.2f} m/s')
                    
                    elif key.lower() == 's':
                        self.current_linear = max(self.current_linear - self.linear_increment, -self.linear_speed)
                        self.get_logger().info(f'Backward: {self.current_linear:.2f} m/s')
                    
                    elif key.lower() == 'a':
                        self.current_angular = min(self.current_angular + self.angular_increment, self.angular_speed)
                        self.get_logger().info(f'Turn Left: {self.current_angular:.2f} rad/s')
                    
                    elif key.lower() == 'd':
                        self.current_angular = max(self.current_angular - self.angular_increment, -self.angular_speed)
                        self.get_logger().info(f'Turn Right: {self.current_angular:.2f} rad/s')
                    
                    elif key == ' ':
                        self.current_linear = 0.0
                        self.current_angular = 0.0
                        self.get_logger().info('STOPPED')
                    
                    elif key.lower() == 'q':
                        self.linear_speed = min(self.linear_speed + 0.1, 2.0)
                        self.angular_speed = min(self.angular_speed + 0.1, 1.5)
                        self.get_logger().info(f'Speed UP: Linear={self.linear_speed:.2f}, Angular={self.angular_speed:.2f}')
                    
                    elif key.lower() == 'e':
                        self.linear_speed = max(self.linear_speed - 0.1, 0.1)
                        self.angular_speed = max(self.angular_speed - 0.1, 0.1)
                        self.get_logger().info(f'Speed DOWN: Linear={self.linear_speed:.2f}, Angular={self.angular_speed:.2f}')
                    
                    elif key.lower() == 'x':
                        self.get_logger().info('Exiting...')
                        break
                
                # Continuously publish current velocity
                self.publish_velocity()
                rate.sleep()
                
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
            # Send stop command
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.publish_velocity()


def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
