#!/usr/bin/env python3
"""
Simple UDP command sender (no feedback listening).
Use this when port 9002 is already in use.
"""

import socket
import json
import time
import sys

def send_command(sock, host, port, throttle, steering):
    """Send a control command via UDP"""
    cmd = {
        "type": "control",
        "timestamp": int(time.time() * 1000),
        "throttle": throttle,
        "steering": steering,
        "gear": 3,
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

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  Test mode:        ./test_udp_send_only.py <gazebo_ip> test")
        print("  Interactive mode: ./test_udp_send_only.py <gazebo_ip>")
        print("")
        print("Example: ./test_udp_send_only.py 127.0.0.1 test")
        print("")
        print("Watch logs: docker logs -f gazebo-sim-inspection | grep 'UDP CMD'")
        sys.exit(1)
    
    gazebo_ip = sys.argv[1]
    test_mode = len(sys.argv) > 2 and sys.argv[2] == 'test'
    command_port = 9001
    
    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"✓ Sending commands to {gazebo_ip}:{command_port}")
    print(f"  Watch logs with: docker logs -f gazebo-sim-inspection | grep -E 'UDP CMD|speed'")
    print("")
    
    if test_mode:
        print("=== Running test sequence ===\n")
        
        # Test 1: Forward
        print("Test 1: Forward movement (5 seconds)")
        for i in range(50):
            send_command(sock, gazebo_ip, command_port, throttle=0.3, steering=0.0)
            time.sleep(0.1)
        
        # Stop
        print("\nStopping...")
        for i in range(10):
            send_command(sock, gazebo_ip, command_port, throttle=0.0, steering=0.0)
            time.sleep(0.1)
        
        # Test 2: Turn left
        print("\nTest 2: Turn left while moving (3 seconds)")
        for i in range(30):
            send_command(sock, gazebo_ip, command_port, throttle=0.2, steering=0.5)
            time.sleep(0.1)
        
        # Stop
        print("\nStopping...")
        for i in range(10):
            send_command(sock, gazebo_ip, command_port, throttle=0.0, steering=0.0)
            time.sleep(0.1)
        
        # Test 3: Backward
        print("\nTest 3: Backward movement (2 seconds)")
        for i in range(20):
            send_command(sock, gazebo_ip, command_port, throttle=-0.2, steering=0.0)
            time.sleep(0.1)
        
        # Stop
        print("\nStopping...")
        for i in range(10):
            send_command(sock, gazebo_ip, command_port, throttle=0.0, steering=0.0)
            time.sleep(0.1)
        
        print("\n=== Test complete ===")
        print("Check the logs to see the robot's response!")
        
    else:
        print("=== Interactive Control Mode ===")
        print("Commands:")
        print("  w/s - Forward/Backward (increase/decrease throttle)")
        print("  a/d - Turn left/right (increase/decrease steering)")
        print("  x   - Stop (zero throttle and steering)")
        print("  q   - Quit")
        print("")
        print("Watch another terminal for logs:")
        print("  docker logs -f gazebo-sim-inspection | grep -E 'UDP CMD|speed'")
        print("")
        
        throttle = 0.0
        steering = 0.0
        
        # Set stdin to non-blocking
        import termios
        import tty
        import select
        
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
                send_command(sock, gazebo_ip, command_port, throttle, steering)
                time.sleep(0.05)  # 20 Hz
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    sock.close()

if __name__ == '__main__':
    main()
