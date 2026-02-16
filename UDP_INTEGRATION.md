# UDP Integration with av-simulation

This document explains how `offroad-gazebo-integration` communicates with Ottopia's `av-simulation` framework over UDP using a **JSON protocol**.

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    av-simulation                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │  UdpAdapter (udp_gazebo)                           │  │
│  │  - Sends JSON control commands via UDP             │  │
│  │  - Receives JSON sensor feedback via UDP           │  │
│  └──────────────────┬─────────────────────────────────┘  │
└─────────────────────┼────────────────────────────────────┘
                      │ UDP (JSON, ports 9001/9002)
┌─────────────────────▼────────────────────────────────────┐
│            offroad-gazebo-integration                     │
│  ┌────────────────────────────────────────────────────┐  │
│  │  UdpBridge (udp_bridge.py)                         │  │
│  │  - Receives control JSON on port 9001              │  │
│  │  - Sends feedback JSON on port 9002                │  │
│  │  - Converts to/from ROS2 messages                  │  │
│  └──────────────────┬─────────────────────────────────┘  │
│                     │ ROS2 Topics                         │
│  ┌──────────────────▼─────────────────────────────────┐  │
│  │  ros_gz_bridge → Gazebo Sim                        │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

## JSON Protocol

All messages are UTF-8 encoded JSON objects sent as single UDP datagrams.

### Control Command (av-simulation → Gazebo, port 9001)

```json
{
  "type": "control",
  "timestamp": 1708099200000,
  "throttle": 0.5,
  "steering": -0.3,
  "gear": 3,
  "turn_left": false,
  "turn_right": true,
  "high_beam": false,
  "low_beam": true,
  "horn": false,
  "hazard": false
}
```

| Field       | Type    | Description                                |
|-------------|---------|--------------------------------------------|
| type        | string  | Always `"control"`                         |
| timestamp   | int     | Epoch milliseconds                         |
| throttle    | float   | Gas pedal / brake, −1.0 to 1.0            |
| steering    | float   | Steering wheel, −1.0 (left) to 1.0 (right)|
| gear        | int     | 0=Park, 1=Reverse, 2=Neutral, 3=Drive     |
| turn_left   | bool    | Left turn signal                           |
| turn_right  | bool    | Right turn signal                          |
| high_beam   | bool    | High-beam lights                           |
| low_beam    | bool    | Low-beam lights                            |
| horn        | bool    | Horn active                                |
| hazard      | bool    | Hazard lights                              |

### Sensor Feedback (Gazebo → av-simulation, port 9002)

```json
{
  "type": "feedback",
  "timestamp": 1708099200050,
  "speed": 3.45,
  "rpm": 1200,
  "gps": {
    "latitude": 31.7683,
    "longitude": 35.2137,
    "azimuth": 45.0
  },
  "object_detected": false,
  "vehicle_state": 1
}
```

| Field           | Type   | Description                              |
|-----------------|--------|------------------------------------------|
| type            | string | Always `"feedback"`                      |
| timestamp       | int    | Epoch milliseconds                       |
| speed           | float  | Vehicle speed in m/s (≥ 0)               |
| rpm             | int    | Estimated engine RPM                     |
| gps.latitude    | float  | Degrees                                  |
| gps.longitude   | float  | Degrees                                  |
| gps.azimuth     | float  | Heading in degrees (from quaternion yaw) |
| object_detected | bool   | Obstacle detected flag                   |
| vehicle_state   | int    | Vehicle state enum (1 = RemoteDriving)   |

## Configuration

### av-simulation

```ini
[udp_gazebo]
server_ip = 127.0.0.1
server_port = 9001
client_ip = 127.0.0.1
client_port = 9002
send_interval_ms = 10
heartbeat_timeout_ms = 5000
```

```bash
./av_simulation /path/to/configs/ --adapter=udp_gazebo
```

### offroad-gazebo-integration

```bash
# Terminal 1: Gazebo world
ros2 launch offroad_gazebo_integration offroad_world.launch.py

# Terminal 2: UDP bridge
ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
  av_sim_ip:=127.0.0.1 \
  av_sim_command_port:=9001 \
  av_sim_sensor_port:=9002 \
  send_rate:=50.0
```

## Testing

### Send a test control command

```python
import json, socket, time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msg = json.dumps({
    "type": "control",
    "timestamp": int(time.time() * 1000),
    "throttle": 0.5, "steering": 0.0, "gear": 3,
    "turn_left": False, "turn_right": False,
    "high_beam": False, "low_beam": True,
    "horn": False, "hazard": False,
}).encode()
sock.sendto(msg, ("127.0.0.1", 9001))
```

### Monitor ROS topics

```bash
ros2 topic echo /vehicle/cmd_vel
ros2 topic echo /vehicle/odom
```

### Run unit tests

```bash
cd offroad-gazebo-integration
python -m pytest test/test_adapter.py -v
```

## Troubleshooting

| Symptom | Check |
|---------|-------|
| No communication | `sudo tcpdump -i any udp port 9001 -X` |
| Vehicle not moving | `ros2 topic echo /vehicle/cmd_vel` |
| JSON parse errors | Ensure both sides use UTF-8 JSON (no binary packing) |
| Connection timeout | av-simulation heartbeat_timeout_ms, network/firewall |
