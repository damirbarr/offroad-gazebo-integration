# Quick Start Guide

Get up and running with off-road autonomy simulation in 5 minutes.

## Prerequisites

```bash
# Ubuntu 22.04 (Jammy) or later
# ROS 2 Humble or later

# Install ROS2 (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install Gazebo Sim
sudo apt install gz-harmonic

# Install ROS-Gazebo bridge
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim

# Build tools
sudo apt install python3-colcon-common-extensions python3-pip
pip3 install numpy pillow pyyaml
```

## Installation

```bash
# Create workspace
mkdir -p ~/av_ws/src
cd ~/av_ws/src

# Clone this repository
git clone https://github.com/damirbarr/offroad-gazebo-integration.git

# Clone av-simulation (replace with actual repo)
# git clone https://github.com/ottopia-tech/av-simulation.git

# Build
cd ~/av_ws
colcon build

# Source workspace
source install/setup.bash
```

## Running Your First Simulation

### Terminal 1: Launch Gazebo World

```bash
source ~/av_ws/install/setup.bash
ros2 launch offroad_gazebo_integration offroad_world.launch.py
```

This will:
- Start Gazebo Sim with desert terrain
- Launch the ROS-Gazebo bridge
- Spawn vehicle at origin

### Terminal 2: Launch Gazebo Adapter (for av-simulation)

```bash
source ~/av_ws/install/setup.bash
ros2 launch offroad_gazebo_integration gazebo_adapter.launch.py
```

This starts the adapter that connects Gazebo to the av-simulation framework.

### Terminal 3: Send Test Commands

```bash
source ~/av_ws/install/setup.bash

# Drive forward
ros2 topic pub /vehicle/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  --rate 10

# Stop (Ctrl+C the above, then)
ros2 topic pub /vehicle/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  --once
```

### Terminal 4: Monitor Sensor Data

```bash
source ~/av_ws/install/setup.bash

# Watch odometry
ros2 topic echo /vehicle/odom

# Watch IMU
ros2 topic echo /vehicle/imu

# Watch GPS
ros2 topic echo /vehicle/gps

# List all topics
ros2 topic list
```

## Using with av-simulation

Once av-simulation is installed:

```bash
# Launch complete stack
ros2 launch av_simulation simulation.launch.py \
  adapter:=gazebo \
  world:=desert_terrain
```

## Customizing Terrain

### Use Different World

```bash
ros2 launch offroad_gazebo_integration offroad_world.launch.py \
  world:=rocky_mountain
```

Available worlds:
- `desert_terrain` (default) - Sandy desert environment
- `forest_trail` - Wooded trail (TODO)
- `rocky_mountain` - Steep rocky terrain (TODO)

### Generate Custom Terrain

```python
from gazebo_adapter.terrain import TerrainManager, TerrainType

manager = TerrainManager()

config = {
    'heightmap_size': (512, 512),
    'height_range': (0, 20),
    'roughness': 0.8,
    'size': (200, 200, 20),
    'type': 'mud',
    'seed': 12345,
    'heightmap_path': '/tmp/custom_terrain.png'
}

manager.generate_terrain_world(
    world_name='custom_offroad',
    terrain_config=config,
    output_path='/tmp/custom_world.sdf'
)
```

Then launch:

```bash
gz sim /tmp/custom_world.sdf
```

## Troubleshooting

### Gazebo doesn't start

Check Gazebo version:
```bash
gz sim --version
```

Should be Gazebo Sim (Harmonic) or later.

### No topics visible

Check bridge is running:
```bash
ros2 node list | grep bridge
ros2 topic list
```

### Vehicle falls through terrain

The terrain heightmap may not be loading. Check:
```bash
gz model --list
```

### Poor performance

Try:
```bash
# Headless mode (no GUI)
ros2 launch offroad_gazebo_integration offroad_world.launch.py headless:=true

# Or reduce physics rate in config/simulation.yaml
```

## Next Steps

1. **Read the full README.md** for detailed configuration options
2. **Check out worlds/** for example terrain files
3. **Modify config/simulation.yaml** for sensor tuning
4. **Write your autonomy stack** and integrate via av-simulation
5. **Create custom vehicles** by modifying models/

## Getting Help

- Issues: https://github.com/damirbarr/offroad-gazebo-integration/issues
- Gazebo Docs: https://gazebosim.org/docs
- ROS2 Docs: https://docs.ros.org/en/humble/

Happy off-roading! 🚙💨
