# CPR Inspection World Integration

This document describes the integration of the Clearpath Robotics (CPR) inspection world into the offroad-gazebo-integration repository.

## Overview

The inspection world is a Gazebo simulation environment designed for testing marine/water-based robots. It features a water table with inspection geometry including platforms, structures, and underwater elements.

## Source

Original repository: [clearpathrobotics/cpr_gazebo](https://github.com/clearpathrobotics/cpr_gazebo/tree/noetic-devel/cpr_inspection_gazebo)

## Integrated Components

### 1. World File
- **Location**: `worlds/inspection_world.world`
- **Description**: Main world file with water physics, lighting, and model references
- **Features**:
  - Underwater current simulation plugin
  - Directional lighting for realistic rendering
  - Water table model inclusion
  - Spherical coordinates (latitude/longitude)

### 2. Models
- **Location**: `models/water_table/`
- **Contents**:
  - `model.config`: Model metadata
  - `model.sdf`: Water table SDF definition
  - `meshes/`: Mesh files for visual and collision geometry
    - `inspection_world.dae`: Main inspection structure mesh
    - `inspection_water.dae`: Water surface mesh

### 3. Meshes
- **Location**: `meshes/`
- **Contents**:
  - `inspection_world.dae`: Primary geometry mesh (790KB)
  - `inspection_water.dae`: Water mesh (4KB)
  - Texture files:
    - `InspectionBakeLarge.jpg`: Main baked texture (18MB)
    - `ConcreteWall001_COL_VAR1_3K.jpg`: Concrete texture (1.2MB)
    - `MetalSpottyDiscoloration001_COL_3K_METALNESS.jpg`: Metal texture (3.6MB)
    - `WoodPlanksWorn33_COL_VAR1_3K.jpg`: Wood texture (2.4MB)

### 4. URDF/Xacro Files
- **Location**: `urdf/inspection_geometry.urdf.xacro`
- **Description**: Robot description format of inspection world geometry
- **Usage**: Can be included in robot description for TF tree and collision detection

## Launch File

A dedicated launch file has been created for easy world deployment:

**File**: `launch/inspection_world.launch.py`

### Usage

```bash
# Launch with GUI
ros2 launch offroad_gazebo_integration inspection_world.launch.py

# Launch headless (no GUI)
ros2 launch offroad_gazebo_integration inspection_world.launch.py headless:=true

# Specify vehicle spawn position
ros2 launch offroad_gazebo_integration inspection_world.launch.py \
    vehicle_x:=5.0 vehicle_y:=5.0 vehicle_z:=0.1
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `headless` | `false` | Run without GUI |
| `use_sim_time` | `true` | Use simulation clock |
| `vehicle_model` | `heron` | Vehicle model (use marine vehicles for water) |
| `vehicle_x` | `0.0` | Spawn X position |
| `vehicle_y` | `0.0` | Spawn Y position |
| `vehicle_z` | `0.1` | Spawn Z position (at water surface) |

## Environment Details

### Water Physics

The world includes an underwater current plugin (`libuuv_underwater_current_ros_plugin.so`) which simulates water currents with configurable:
- Velocity (0-5 m/s range)
- Horizontal and vertical angles
- Noise parameters

### Lighting

Two directional lights provide realistic illumination:
- **Sun**: Primary overhead light with shadows
- **Sun diffuse**: Secondary light for ambient illumination

### Coordinates

The world is set at real-world coordinates:
- **Latitude**: 57.0271155°N
- **Longitude**: 115.426770°W
- **Elevation**: 600m

## Recommended Vehicles

This world is designed for water-based vehicles such as:
- **Heron USV** (Unmanned Surface Vehicle)
- Other marine robots with buoyancy and hydrodynamics

## Integration Notes

1. **Model Path**: The water_table model is referenced as `model://water_table`, which Gazebo resolves from the `models/` directory
2. **Mesh References**: The URDF xacro uses ROS package paths (`package://offroad_gazebo_integration/meshes/...`)
3. **Dependencies**: Requires UUV Simulator plugins for water physics
4. **Scale**: The world uses a 60x60m water surface area

## Building and Installation

The integrated files are automatically installed when building the package:

```bash
cd ~/workspace/offroad-gazebo-integration
colcon build --packages-select offroad_gazebo_integration
source install/setup.bash
```

The CMakeLists.txt includes installation rules for:
- `worlds/` directory
- `models/` directory
- `meshes/` directory
- `urdf/` directory
- `launch/` directory

## Testing

After building, you can test the world:

```bash
# Source the workspace
source install/setup.bash

# Launch the inspection world
ros2 launch offroad_gazebo_integration inspection_world.launch.py
```

You should see a water table with inspection structures in the Gazebo GUI.

## Future Enhancements

Potential improvements:
1. Add Heron USV model integration
2. Configure buoyancy parameters
3. Add underwater cameras and sensors
4. Create mission scenarios for inspection tasks
5. Add dynamic obstacles or moving elements

## Credits

Original world created by Clearpath Robotics:
- Author: Chris Iverach-Brereton (civerachb@clearpathrobotics.com)
- Repository: https://github.com/clearpathrobotics/cpr_gazebo
