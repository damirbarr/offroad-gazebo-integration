#!/bin/bash
set -e

export DISPLAY=${DISPLAY:-:99}

SCREEN_W=1920
SCREEN_H=1080
HALF_W=$((SCREEN_W / 2))

find_window() {
  local pattern="$1"
  wmctrl -l | grep -i "$pattern" | head -n 1 | awk '{print $1}'
}

# Wait for both Gazebo and RViz windows to appear (up to ~30 seconds)
for i in $(seq 1 30); do
  GAZEBO_WIN=$(find_window "Gazebo" || true)
  RVIZ_WIN=$(find_window "RViz" || true)
  if [ -n "$GAZEBO_WIN" ] && [ -n "$RVIZ_WIN" ]; then
    break
  fi
  sleep 1
done

# If we did not find both windows, exit quietly
if [ -z "$GAZEBO_WIN" ] || [ -z "$RVIZ_WIN" ]; then
  exit 0
fi

# Tile Gazebo on the left, RViz on the right
wmctrl -i -r "$GAZEBO_WIN" -e 0,0,0,$HALF_W,$SCREEN_H || true
wmctrl -i -r "$RVIZ_WIN" -e 0,$HALF_W,0,$HALF_W,$SCREEN_H || true

