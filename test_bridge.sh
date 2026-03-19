#!/bin/bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

TOPICS=(
  /odom
  /vehicle/odom
  /ego/odometry
  /imu/data
  /vehicle/imu
  /mavros/global_position/global
  /vehicle/gps
  /mavros/global_position/compass_hdg
  /mavros/global_position/raw/gps_vel
  /velodyne_points
  /vehicle/lidar/points
)

echo "=== Raw Gazebo topics ==="
gz topic -l | grep -E "(/clock|/cmd_|/odom|/imu|/gps|/lidar)" | sort

echo
echo "=== ROS2 topics (key contract) ==="
ros2 topic list -t | sort | grep -E "^(/clock|/odom|/vehicle/odom|/ego/odometry|/imu/data|/vehicle/imu|/mavros/global_position/global|/vehicle/gps|/mavros/global_position/compass_hdg|/mavros/global_position/raw/gps_vel|/velodyne_points|/vehicle/lidar/points)(\\s|$)" || true

for topic in "${TOPICS[@]}"; do
  echo
  echo "=== ${topic} ==="
  if ros2 topic list | grep -qx "${topic}"; then
    ros2 topic info "${topic}"
  else
    echo "MISSING"
  fi
done

echo
echo "=== Sample GPS message ==="
timeout 8 ros2 topic echo /mavros/global_position/global --once | sed -n '1,20p'

echo
echo "=== Sample odometry message ==="
timeout 8 ros2 topic echo /vehicle/odom --once | sed -n '1,20p'
