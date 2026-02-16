#!/bin/bash
# Start Gazebo with noVNC - view GUI at http://localhost:8080/vnc.html
set -e

export DISPLAY=:99
VNC_PORT=5900
NOVNC_PORT=8080

# Start virtual display
Xvfb :99 -screen 0 1920x1080x24 &
sleep 3

# Start VNC server (connect to virtual display)
x11vnc -display :99 -rfbport $VNC_PORT -forever -shared -nonap -bg
sleep 1

# Start noVNC (websockify proxies VNC to browser)
websockify -v --web=/usr/share/novnc $NOVNC_PORT localhost:$VNC_PORT &
sleep 1

echo ""
echo "=========================================="
echo "  Gazebo GUI: http://localhost:8080/vnc.html"
echo "  Open this URL in your browser"
echo "=========================================="
echo ""

# Run the actual command (Gazebo + bridge)
exec "$@"
