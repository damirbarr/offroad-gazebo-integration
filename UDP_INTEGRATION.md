# UDP Integration with av-simulation

This document explains how to integrate `offroad-gazebo-integration` with Ottopia's `av-simulation` framework using UDP communication.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    av-simulation                             │
│                                                               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  UDP Adapter (udp_gazebo)                             │  │
│  │  - Sends control commands via UDP                     │  │
│  │  - Receives sensor data via UDP                       │  │
│  └──────────────────┬───────────────────────────────────┘  │
└────────────────────┼──────────────────────────────────────┘
                     │
                     │ UDP (ports 9001/9002)
                     │
┌────────────────────▼──────────────────────────────────────┐
│            offroad-gazebo-integration                      │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐ │
│  │  UDP Bridge                                           │ │
│  │  - Receives commands on UDP port 9001                │ │
│  │  - Sends sensor data on UDP port 9002                │ │
│  │  - Converts to/from ROS2 messages                    │ │
│  └──────────────────┬───────────────────────────────────┘ │
│                     │ ROS2 Topics                          │
│  ┌──────────────────▼───────────────────────────────────┐ │
│  │  ros_gz_bridge                                        │ │
│  └──────────────────┬───────────────────────────────────┘ │
└────────────────────┼──────────────────────────────────────┘
                     │
                     │ Gazebo Transport
                     │
┌────────────────────▼──────────────────────────────────────┐
│                  Gazebo Sim                                │
│  - Physics simulation                                      │
│  - Off-road terrain                                        │
│  - Vehicle dynamics                                        │
│  - Sensor simulation                                       │
└───────────────────────────────────────────────────────────┘
```

## UDP Protocol

### Message Types

- **0x01**: Control command (av-simulation → Gazebo)
- **0x02**: Sensor data (Gazebo → av-simulation)
- **0x03**: Heartbeat (bidirectional)

### Control Command Format (Port 9001)

Binary packet, little-endian, 28 bytes:

| Offset | Type   | Field        | Description                    |
|--------|--------|--------------|--------------------------------|
| 0      | uint32 | message_type | 0x01 (control)                 |
| 4      | uint64 | timestamp_ns | Timestamp in nanoseconds       |
| 12     | float  | throttle     | -1.0 to 1.0                    |
| 16     | float  | brake        | 0.0 to 1.0                     |
| 20     | float  | steering     | -1.0 to 1.0                    |
| 24     | float  | gear         | 0=P, 1=R, 2=N, 3=D             |

### Sensor Data Format (Port 9002)

Binary packet, little-endian, 100 bytes:

| Offset | Type     | Field              | Description                    |
|--------|----------|--------------------|--------------------------------|
| 0      | uint32   | message_type       | 0x02 (sensor_data)             |
| 4      | uint64   | timestamp_ns       | Timestamp in nanoseconds       |
| 12     | float[3] | position           | x, y, z (meters)               |
| 24     | float[3] | velocity           | vx, vy, vz (m/s)               |
| 36     | float[4] | orientation_quat   | x, y, z, w (quaternion)        |
| 52     | float[3] | angular_velocity   | wx, wy, wz (rad/s)             |
| 64     | float[3] | linear_acceleration| ax, ay, az (m/s²)              |
| 76     | float[3] | gps                | lat, lon, alt (degrees/meters) |

## Configuration

### av-simulation Config

Add to your av-simulation config file:

```ini
[udp_gazebo]
server_ip = 127.0.0.1           # IP where Gazebo bridge is running
server_port = 9001              # Port for sending commands to Gazebo
client_ip = 127.0.0.1           # IP for receiving sensor data
client_port = 9002              # Port for receiving sensor data from Gazebo
send_interval_ms = 10           # Command send interval (100 Hz)
receive_timeout_ms = 100        # Sensor data receive timeout
heartbeat_timeout_ms = 5000     # Connection heartbeat timeout
polling_interval_ms = 1         # UDP polling interval
max_packet_size = 4096          # Maximum UDP packet size
```

### Launch av-simulation

```bash
cd /path/to/av-simulation
./av_simulation /path/to/configs/ --adapter=udp_gazebo
```

### offroad-gazebo-integration Launch

#### Terminal 1: Gazebo World

```bash
source ~/av_ws/install/setup.bash
ros2 launch offroad_gazebo_integration offroad_world.launch.py
```

#### Terminal 2: UDP Bridge

```bash
source ~/av_ws/install/setup.bash
ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
  av_sim_ip:=127.0.0.1 \
  av_sim_command_port:=9001 \
  av_sim_sensor_port:=9002 \
  send_rate:=50.0
```

## Network Configuration

### Same Machine (localhost)

Use `127.0.0.1` for all IPs. This is the default and simplest setup.

### Different Machines

**Example:** av-simulation on 192.168.1.100, Gazebo on 192.168.1.200

**av-simulation config:**
```ini
[udp_gazebo]
server_ip = 192.168.1.200     # Gazebo machine
server_port = 9001
client_ip = 192.168.1.100     # Local IP
client_port = 9002
```

**UDP bridge launch:**
```bash
ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
  av_sim_ip:=192.168.1.100 \
  av_sim_command_port:=9001 \
  av_sim_sensor_port:=9002
```

**Firewall:**
```bash
# On Gazebo machine (192.168.1.200)
sudo ufw allow 9001/udp
sudo ufw allow 9002/udp
```

## Testing

### Check UDP Traffic

```bash
# On Gazebo machine
sudo tcpdump -i any udp port 9001 -X

# Should see incoming control commands from av-simulation
```

### Monitor Topics

```bash
# Check commands being published
ros2 topic echo /vehicle/cmd_vel
ros2 topic echo /vehicle/cmd_steering

# Check sensor data being received
ros2 topic echo /vehicle/odom
ros2 topic echo /vehicle/imu
ros2 topic echo /vehicle/gps
```

### Send Test Command

```python
import socket
import struct
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = ('127.0.0.1', 9001)

# Control command: throttle=0.5, brake=0.0, steering=0.0, gear=3 (drive)
msg = struct.pack('<IQffff', 
    0x01,                      # message_type
    int(time.time() * 1e9),    # timestamp
    0.5,                       # throttle
    0.0,                       # brake
    0.0,                       # steering
    3.0                        # gear
)

sock.sendto(msg, target)
print("Sent test command")
```

## Troubleshooting

### No communication

1. **Check if UDP bridge is running:**
   ```bash
   ros2 node list | grep udp_bridge
   ```

2. **Check if av-simulation is sending:**
   ```bash
   sudo netstat -anu | grep 9001
   ```

3. **Firewall blocking:**
   ```bash
   sudo ufw status
   sudo ufw allow 9001/udp
   sudo ufw allow 9002/udp
   ```

### Vehicle not moving in Gazebo

1. **Check ROS topics:**
   ```bash
   ros2 topic echo /vehicle/cmd_vel
   ```

2. **Check Gazebo bridge:**
   ```bash
   ros2 node list | grep ros_gz_bridge
   ```

3. **Verify ros_gz_bridge config:** Ensure topics are mapped correctly

### High latency

1. **Reduce send_rate** in udp_bridge launch (default 50 Hz)
2. **Increase send_interval_ms** in av-simulation config
3. **Check network:** `ping` between machines if distributed

### Packet loss

UDP is unreliable by design. To minimize:
- Use localhost when possible
- Reduce send rate if network is congested
- Increase max_packet_size if using rich sensor data

## Performance

**Typical latency:** 2-10ms on localhost, <20ms on LAN

**Bandwidth:**
- Control: ~2.8 KB/s @ 100 Hz
- Sensors: ~5.0 KB/s @ 50 Hz
- Total: ~8 KB/s (negligible)

## Development

### Adding New Control Fields

1. Update CONTROL_FMT in `udp_bridge.py`
2. Update _process_command() to handle new fields
3. Update av-simulation's udp_adapter.cpp
4. Update this documentation

### Adding New Sensors

1. Subscribe to sensor topic in UdpBridge.__init__()
2. Store in callback (with state_lock)
3. Pack in _send_sensor_data()
4. Update SENSOR_FMT and SENSOR_SIZE
5. Update av-simulation's udp_adapter.cpp parser
6. Update this documentation

## See Also

- `UDP_ADAPTER_README.md` in av-simulation repo
- `README.md` for Gazebo setup
- `QUICKSTART.md` for getting started
