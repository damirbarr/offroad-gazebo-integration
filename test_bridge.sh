#!/bin/bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

echo "=== Gazebo topics (raw) ==="
gz topic -l | grep -E "(gps|imu|lidar)" | head -10

echo ""
echo "=== ROS2 topics ==="
ros2 topic list | grep -E "(gps|imu|lidar)"

echo ""
echo "=== Check bridge node ==="
ros2 node info /ros_gz_bridge

echo ""
echo "=== Test: Listen to Gazebo lidar for 2 seconds ==="
timeout 2 gz topic -e -t /lidar/points | head -5
