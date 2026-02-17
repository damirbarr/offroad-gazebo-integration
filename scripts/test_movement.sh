#!/bin/bash
# Test robot movement from keyboard teleop to Gazebo

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

echo "=== 1. Testing ROS topic publish ==="
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once
echo "Published to ROS /cmd_vel"

sleep 2

echo ""
echo "=== 2. Check if bridge forwarded to Gazebo ==="
ign topic -i -t /cmd_vel

echo ""
echo "=== 3. Test direct Gazebo command ==="
echo "Sending direct Gazebo command to move robot..."
ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 1.0}' &
GAZEBO_PID=$!

echo "Waiting 5 seconds... watch the robot in GUI!"
sleep 5

kill $GAZEBO_PID 2>/dev/null

echo ""
echo "=== 4. Stop robot ==="
ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.0}' --once

echo ""
echo "Done! Did the robot move?"
