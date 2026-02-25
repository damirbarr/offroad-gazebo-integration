# Quick Start Guide

Get Gazebo off-road simulation running with av-simulation in **less than 5 minutes**.

## Prerequisites

- **Docker** installed and running
- **av-simulation** built with the UDP adapter from PR [gazebo-udp-adapter](https://github.com/ottopia-tech/av-simulation/tree/gazebo-udp-adapter)

## Step 1: Build av-simulation with UDP Adapter

```bash
cd /path/to/av-simulation
git checkout gazebo-udp-adapter
conan install . --build=missing
cmake --preset conan-release
cmake --build --preset conan-release
```

## Step 2: Configure av-simulation

Add to your av-simulation config file (e.g., `configs/simulation.ini`):

```ini
[udp_gazebo]
server_ip = 127.0.0.1           # Gazebo bridge IP (use host.docker.internal for Docker)
server_port = 9001              # Port for sending commands to Gazebo
client_ip = 127.0.0.1           # Local IP for receiving sensor data
client_port = 9002              # Port for receiving from Gazebo
send_interval_ms = 10           # 100 Hz command rate
receive_timeout_ms = 100
heartbeat_timeout_ms = 5000
polling_interval_ms = 1
max_packet_size = 4096
```

**For Docker:** If running Gazebo in Docker and av-simulation on host:
```ini
server_ip = host.docker.internal  # Docker host machine
```

## Step 3: Build Gazebo Docker Image

```bash
cd /path/to/offroad-gazebo-integration
make build
```

This builds a Docker image with:
- ROS2 Humble
- Gazebo Harmonic
- All simulation dependencies
- offroad-gazebo-integration package

Takes ~10 minutes on first build (Docker caches for future builds).

## Step 4: Run Gazebo Simulation

**Option A: All-in-one (Recommended)**

```bash
make run
```

This starts:
1. Inspection world (Gazebo) with vehicle and sensors
2. UDP bridge for av-simulation communication

## Step 5: Run av-simulation

In a new terminal (on host machine):

```bash
cd /path/to/av-simulation
./build/Release/av_simulation /path/to/configs/ --adapter=udp_gazebo
```

You should see:
```
[INFO] Creating UDP Gazebo adapter
[INFO]   Server: 127.0.0.1:9001
[INFO]   Client: 127.0.0.1:9002
[INFO] UDP Adapter initialized
```

## Step 6: Control the Vehicle

Use av-simulation's API or joystick to send commands. The vehicle will move in Gazebo!

**Example with API:**
```bash
# From av-simulation API
curl -X POST http://localhost:8080/drive \
  -d '{"throttle": 0.5, "steering": 0.0}'
```

---

## Troubleshooting

### "Cannot connect to UDP server"

**Check if Gazebo bridge is running:**
```bash
docker ps | grep gazebo-sim
```

**Check if ports are open:**
```bash
nc -zvu 127.0.0.1 9001  # Command port
nc -zvu 127.0.0.1 9002  # Sensor port
```

**Check logs:**
```bash
docker logs gazebo-sim
```

### "No sensor data received"

**Verify Gazebo topics are publishing:**
```bash
# Run a container and check topics (exit when done)
docker run --rm -it --entrypoint bash offroad-gazebo-integration:latest -c "\
  source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
  ros2 topic list && ros2 topic echo /odom"
```
Or with the simulation running, use `docker exec -it gazebo-sim bash` then source and run `ros2 topic list`.

### Vehicle not moving

1. **Check Gazebo GUI** (if running with display):
   - Is the vehicle spawned?
   - Is physics running?

2. **Check ROS commands:**
   With the simulation running: `docker exec -it gazebo-sim bash`, then `source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic echo /cmd_vel`
   Should show velocity commands when av-simulation sends throttle.

3. **Check UDP bridge:**
   Look for "Received control command" in logs

### Docker networking issues

**If running av-simulation in a different Docker container:**

Use bridge network:
```bash
# Create network
docker network create sim-network

# Run Gazebo with network
docker run --network sim-network --name gazebo-sim ...

# Run av-simulation with network
docker run --network sim-network --name av-sim ...

# Use container name as IP
server_ip = gazebo-sim
```

---

## Configuration Options

### Custom UDP Ports

```bash
make run AV_SIM_CMD_PORT=10001 AV_SIM_SENSOR_PORT=10002
```

Update av-simulation config to match.

### Custom av-simulation IP

If av-simulation is on a different machine (e.g., 192.168.1.100):

```bash
make run AV_SIM_IP=192.168.1.100
```

### Different Terrain

Edit `worlds/desert_terrain.sdf` or create new world files.

---

## Performance Tips

**Running headless (no GUI):**
```bash
# Add headless:=true to launch
make run USE_VNC=false
```

**Reduce sensor rate:**
```bash
make run AV_SIM_SENSOR_PORT=9002  # Default is 50 Hz
# Or edit config/simulation.yaml
```

**Hardware acceleration:**
Ensure Docker has GPU access for better Gazebo performance:
```bash
docker run --gpus all ...
```

---

## Development Workflow

### Modify Terrain

1. Edit `worlds/desert_terrain.sdf`
2. Rebuild: `make build`
3. Run: `make run`

### Modify UDP Protocol

1. Edit `src/gazebo_adapter/udp_bridge.py`
2. Update av-simulation's `udp_adapter.cpp` to match
3. Rebuild both
4. Test end-to-end

### Add Sensors

1. Edit Gazebo world file (add sensor plugins)
2. Update `udp_bridge.py` to subscribe to new topics
3. Update UDP protocol struct in documentation
4. Update av-simulation's parser

---

## Next Steps

- **Read UDP_INTEGRATION.md** for detailed protocol documentation
- **Customize terrain** in `worlds/` directory
- **Add vehicle models** in `models/` directory
- **Tune physics** in `config/simulation.yaml`

---

## Help

**Makefile commands:**
```bash
make help              # Show all available commands
```

**Check connectivity:**
```bash
make check-avsim       # Test if av-simulation is reachable
```

**View logs:**
```bash
docker logs -f gazebo-sim
```

**Interactive debugging:**
With simulation running: `docker exec -it gazebo-sim bash`. Or run a one-off container: `docker run --rm -it --entrypoint bash offroad-gazebo-integration:latest`.

---

## Architecture

```
┌─────────────────────────────────────────┐
│     av-simulation (host or container)    │
│     UDP Adapter (udp_gazebo)             │
└──────────────┬──────────────────────────┘
               │ UDP (9001/9002)
               │
┌──────────────▼──────────────────────────┐
│  Docker Container: offroad-gazebo-int    │
│                                           │
│  ┌────────────────────────────────────┐ │
│  │  UDP Bridge (Python)                │ │
│  │  - Receives commands on :9001       │ │
│  │  - Sends sensors on :9002           │ │
│  └──────────┬─────────────────────────┘ │
│             │ ROS2 Topics                │
│  ┌──────────▼─────────────────────────┐ │
│  │  ros_gz_bridge                      │ │
│  └──────────┬─────────────────────────┘ │
│             │ Gazebo Transport           │
│  ┌──────────▼─────────────────────────┐ │
│  │  Gazebo Sim (Harmonic)              │ │
│  │  - Physics engine                   │ │
│  │  - Off-road terrain                 │ │
│  │  - Vehicle simulation               │ │
│  └─────────────────────────────────────┘ │
└───────────────────────────────────────────┘
```

**Data Flow:**
1. av-simulation sends control → UDP bridge → Gazebo (vehicle moves)
2. Gazebo sensors → ros_gz_bridge → UDP bridge → av-simulation

---

Happy off-roading! 🚙💨
