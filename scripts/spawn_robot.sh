#!/bin/bash
# Manually spawn the inspection robot
# Usage: ./spawn_robot.sh [x] [y] [z]

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=/workspace/install/offroad_gazebo_integration/share/offroad_gazebo_integration/models:${IGN_GAZEBO_RESOURCE_PATH}

X=${1:-5.0}
Y=${2:-5.0}
Z=${3:-2.0}

echo "Spawning inspection_robot at ($X, $Y, $Z)..."

ign service \
  -s /world/inspection_world/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 30000 \
  --req "sdf_filename: 'model://inspection_robot', name: 'vehicle', pose: {position: {x: $X, y: $Y, z: $Z}}"

echo "Done!"
