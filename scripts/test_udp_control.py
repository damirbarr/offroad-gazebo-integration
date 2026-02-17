#!/usr/bin/env python3
"""
Test UDP control commands to Gazebo robot.
Sends commands to the UDP bridge and shows feedback.
"""

import socket
import json
import time
import sys
import select

def send_command(sock, host, port, throttle, steering, gear=3):
    """Send a control command via UDP"""
    cmd = {
        "type": "control",
        "timestamp": int(time.time() * 1000),
        "throttle": throttle,
        "steering": steering,
        "gear": gear,
        "turn_left": False,
        "turn_right": False,
        "high_beam": False,
        "low_beam": True,
        "horn": False,
        "hazard": False
    }
    
    payload = json.dumps(cmd).encode('utf-8')
    sock.sendto(payload, (host, port))
    print(f"→ Sent: throttle={throttle:.2f}, steering={steering:.2f}")

def receive_feedback(sock, timeout=0.1):
    """Receive feedback from UDP bridge"""
    sock.settimeout(timeout)
    try:
        data, addr = sock.recvfrom(4096)
        feedback = json.loads(data.decode('utf-8'))
        if feedback.get('type') == 'feedback':
            print(f"← Feedback: speed={feedback['speed']:.3f} m/s, "
                  f"rpm={feedback['rpm']}, "
                  f"gps=({feedback['gps']['latitude']:.6f}, {feedback['gps']['longitude']:.6f})")
            return feedback
    except socket.timeout:
        pass
    except Exception as e:
        print(f"Error receiving: {e}")
    return None

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  Interactive mode: ./test_udp_control.py <gazebo_ip>")
        print("  Test mode:        ./test_udp_control.py <gazebo_ip> test")
        print("")
        print("Example: ./test_udp_control.py 192.168.10.92")
        sys.exit(1)
    
    gazebo_ip = sys.argv[1]
    test_mode = len(sys.argv) > 2 and sys.argv[2] == 'test'
    
    # UDP ports
    command_port = 9001  # Send commands to Gazebo
    feedback_port = 9002  # Receive feedback from Gazebo
    
    # Create sockets
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Bind feedback socket
    try:
        recv_sock.bind(('0.0.0.0', feedback_port))
        print(f"✓ Listening for feedback on port {feedback_port}")
    except Exception as e:
        print(f"✗ Failed to bind feedback socket: {e}")
        print(f"  Make sure port {feedback_port} is not in use")
        sys.exit(1)
    
    print(f"✓ Sending commands to {gazebo_ip}:{command_port}")
    print("")
    
    if test_mode:
        print("=== Running test sequence ===\n")
        
        # Test 1: Forward
        print("Test 1: Forward movement (5 seconds)")
        for i in range(50):
            send_command(send_sock, gazebo_ip, command_port, throttle=0.3, steering=0.0)
            receive_feedback(recv_sock)
            time.sleep(0.1)
        
        # Stop
        print("\nStopping...")
        for i in range(10):
            send_command(send_sock, gazebo_ip, command_port, throttle=0.0, steering=0.0)
            receive_feedback(recv_sock)
            time.sleep(0.1)
        
        # Test 2: Turn left
        print("\nTest 2: Turn left while moving (3 seconds)")
        for i in range(30):
            send_command(send_sock, gazebo_ip, command_port, throttle=0.2, steering=0.5)
            receive_feedback(recv_sock)
            time.sleep(0.1)
        
        # Stop
        print("\nStopping...")
        for i in range(10):
            send_command(send_sock, gazebo_ip, command_port, throttle=0.0, steering=0.0)
            receive_feedback(recv_sock)
            time.sleep(0.1)
        
        print("\n=== Test complete ===")
        
    else:
        print("=== Interactive Control Mode ===")
        print("Commands:")
        print("  w/s - Forward/Backward (increase/decrease throttle)")
        print("  a/d - Turn left/right (increase/decrease steering)")
        print("  x   - Stop (zero throttle and steering)")
        print("  q   - Quit")
        print("")
        
        throttle = 0.0
        steering = 0.0
        
        # Set stdin to non-blocking
        import termios
        import tty
        
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            print("Ready! Press keys to control...")
            
            while True:
                # Check for keyboard input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    
                    if ch == 'q':
                        print("\nQuitting...")
                        break
                    elif ch == 'w':
                        throttle = min(1.0, throttle + 0.1)
                        print(f"Throttle: {throttle:.2f}")
                    elif ch == 's':
                        throttle = max(-1.0, throttle - 0.1)
                        print(f"Throttle: {throttle:.2f}")
                    elif ch == 'a':
                        steering = min(1.0, steering + 0.1)
                        print(f"Steering: {steering:.2f}")
                    elif ch == 'd':
                        steering = max(-1.0, steering - 0.1)
                        print(f"Steering: {steering:.2f}")
                    elif ch == 'x':
                        throttle = 0.0
                        steering = 0.0
                        print("STOP")
                
                # Send current command
                send_command(send_sock, gazebo_ip, command_port, throttle, steering)
                
                # Receive and display feedback
                receive_feedback(recv_sock)
                
                time.sleep(0.05)  # 20 Hz
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    send_sock.close()
    recv_sock.close()

if __name__ == '__main__':
    main()
