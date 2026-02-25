# Makefile for offroad-gazebo-integration
# Quick start for Gazebo off-road simulation with av-simulation integration

.PHONY: help build run run-inspection run-world rviz rviz-config stop clean check-avsim foxglove

# Docker image name
IMAGE_NAME := offroad-gazebo-integration
IMAGE_TAG := latest
CONTAINER_NAME := gazebo-sim

# Network settings for av-simulation integration
AV_SIM_IP ?= 192.168.10.128 # THE AV-SIMULATION IP
AV_SIM_CMD_PORT ?= 9001
AV_SIM_SENSOR_PORT ?= 9002

# VNC: set to false to disable VNC/GUI (default: true, always has VNC)
USE_VNC ?= true

# Logging: info (default) or debug (verbose packet inspection)
LOG_LEVEL ?= debug

help:
	@echo "╔════════════════════════════════════════════════════════════════╗"
	@echo "║  Offroad Gazebo Integration - Quick Start                      ║"
	@echo "╚════════════════════════════════════════════════════════════════╝"
	@echo ""
	@echo "Prerequisites:"
	@echo "  • Docker installed"
	@echo "  • av-simulation built with UDP adapter"
	@echo ""
	@echo "Quick Start:"
	@echo "  1. make build         - Build Docker image (first time only)"
	@echo "  2. make run           - Run inspection world + UDP bridge"
	@echo "  3. make rviz          - Launch RViz2 with LiDAR visualization (separate terminal)"
	@echo "  4. In another terminal: Run av-simulation with udp_gazebo adapter"
	@echo ""
	@echo "Commands:"
	@echo "  make build            - Build Docker image with ROS2 Humble + Gazebo"
	@echo "  make run              - Run inspection world + UDP bridge (VNC enabled)"
	@echo "  make run-inspection   - Same as make run (alias)"
	@echo "  make run-world        - Run offroad world (desert_terrain) + UDP bridge"
	@echo "  make rviz             - Launch RViz2 with LiDAR config (requires 'make run' first)"
	@echo "  make rviz-config      - Show path to RViz2 config file"
	@echo "  make foxglove         - Launch Foxglove Studio web viewer (alternative to RViz2)"
	@echo "  make stop             - Stop running container"
	@echo "  make clean            - Remove Docker image"
	@echo ""
	@echo "Configuration:"
	@echo "  AV_SIM_IP=$(AV_SIM_IP)"
	@echo "  AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT)"
	@echo "  AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT)"
	@echo "  USE_VNC=$(USE_VNC) (default: true, GUI at http://localhost:8080/vnc.html)"
	@echo "  LOG_LEVEL=$(LOG_LEVEL) (info or debug)"
	@echo ""
	@echo "Examples:"
	@echo "  make run AV_SIM_IP=192.168.10.128"
	@echo "  make run AV_SIM_IP=192.168.10.128 LOG_LEVEL=debug"
	@echo "  make run USE_VNC=false  # No GUI"
	@echo "  make rviz               # View LiDAR in RViz2"
	@echo ""

build:
	@echo "Building Docker image..."
	docker build -t $(IMAGE_NAME):$(IMAGE_TAG) .
	@echo "✓ Image built: $(IMAGE_NAME):$(IMAGE_TAG)"

run: build
	@echo "Starting CPR inspection world + UDP bridge..."
	@echo "  • Gazebo GUI: $(if $(filter false,$(USE_VNC)),disabled,open http://localhost:8080/vnc.html)"
	@echo "  • World: Inspection platforms with obstacles"
	@echo "  • Vehicle: Ackermann steering car at (-15, 0, 0.5) with GPS, IMU, and LiDAR"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	@echo ""
	@echo "Topics: /odom, /imu/data, /lidar, /mavros/global_position/global, /cmd_vel"
	docker run -it --rm \
		--name $(CONTAINER_NAME) \
		-p 8080:8080 \
		-p 9001:9001/udp \
		-p 9002:9002/udp \
		-e AV_SIM_IP=$(AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		$(if $(filter false,$(USE_VNC)),bash,run_with_vnc.sh bash) -c " \
			echo 'Launching inspection world...' && \
			ros2 launch offroad_gazebo_integration inspection_world.launch.py headless:=false \
				vehicle_model:=inspection_robot \
				vehicle_x:=-15.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 & \
			sleep 10 && \
			echo 'Launching UDP bridge...' && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				log_level:=$(LOG_LEVEL) \
		"

# Alias: run-inspection is the same as run
run-inspection: run

run-world: build
	@echo "Starting offroad world (desert_terrain) + UDP bridge..."
	@echo "  • Gazebo GUI: $(if $(filter false,$(USE_VNC)),disabled,open http://localhost:8080/vnc.html)"
	@echo "  • World: desert_terrain (same topics as inspection: /odom, /imu/data, /lidar, /cmd_vel)"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-world \
		-p 8080:8080 \
		-p 9001:9001/udp \
		-p 9002:9002/udp \
		-e AV_SIM_IP=$(AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		$(if $(filter false,$(USE_VNC)),bash,run_with_vnc.sh bash) -c " \
			echo 'Launching offroad world...' && \
			ros2 launch offroad_gazebo_integration offroad_world.launch.py headless:=false \
				world_file:=desert_terrain.sdf \
				world_name:=desert_terrain \
				vehicle_model:=inspection_robot \
				vehicle_x:=0.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 & \
			sleep 10 && \
			echo 'Launching UDP bridge...' && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				log_level:=$(LOG_LEVEL) \
		"

stop:
	@echo "Stopping containers..."
	-docker stop $(CONTAINER_NAME) $(CONTAINER_NAME)-world 2>/dev/null || true
	@echo "✓ Stopped"

clean:
	@echo "Removing Docker image..."
	docker rmi $(IMAGE_NAME):$(IMAGE_TAG) || true
	@echo "✓ Cleaned"

# RViz2 visualization targets
RVIZ_CONFIG := $(CURDIR)/config/rviz/lidar_view.rviz

rviz-config:
	@echo "RViz2 config file location:"
	@echo "  $(RVIZ_CONFIG)"
	@echo ""
	@echo "LiDAR Topic: /velodyne_points (PointCloud2)"
	@echo "Frame ID: velodyne"

rviz:
	@echo "╔════════════════════════════════════════════════════════════════╗"
	@echo "║  Launching RViz2 with LiDAR visualization                      ║"
	@echo "╚════════════════════════════════════════════════════════════════╝"
	@echo ""
	@echo "Prerequisites:"
	@echo "  • 'make run' must be running in another terminal"
	@echo "  • ROS2 Humble must be sourced"
	@echo ""
	@echo "LiDAR Topic: /velodyne_points (PointCloud2)"
	@echo "Config: $(RVIZ_CONFIG)"
	@echo ""
	@if [ ! -f "$(RVIZ_CONFIG)" ]; then \
		echo "✗ Error: RViz config file not found at $(RVIZ_CONFIG)"; \
		exit 1; \
	fi
	@echo "Starting RViz2..."
	ros2 run rviz2 rviz2 -d $(RVIZ_CONFIG)

# Foxglove Studio web-based visualization (alternative to RViz2)
foxglove:
	@echo "╔════════════════════════════════════════════════════════════════╗"
	@echo "║  Foxglove Studio - Web-based visualization                     ║"
	@echo "╚════════════════════════════════════════════════════════════════╝"
	@echo ""
	@echo "Options for web-based visualization:"
	@echo ""
	@echo "1. Foxglove Studio (recommended):"
	@echo "   • Install: https://foxglove.dev/download"
	@echo "   • Or use web app: https://app.foxglove.dev"
	@echo "   • Install foxglove_bridge: sudo apt install ros-humble-foxglove-bridge"
	@echo "   • Launch: ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
	@echo "   • Connect to: ws://localhost:8765"
	@echo ""
	@echo "2. Webviz (browser-based):"
	@echo "   • Docker: docker run -p 8080:8080 cruise/webviz"
	@echo "   • Open: http://localhost:8080"
	@echo "   • Requires rosbridge_server: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
	@echo ""
	@echo "3. RVizWeb (web-based RViz):"
	@echo "   • Install: sudo apt install ros-humble-rvizweb"
	@echo "   • Launch: ros2 launch rvizweb rvizweb.launch.xml"
	@echo "   • Open: http://localhost:8000/rvizweb/www/index.html"
	@echo ""
	@echo "LiDAR Topic: /velodyne_points (sensor_msgs/PointCloud2)"

# Check if av-simulation is reachable
check-avsim:
	@echo "Checking av-simulation connectivity..."
	@echo "  Testing UDP port $(AV_SIM_CMD_PORT) on $(AV_SIM_IP)..."
	@nc -zvu $(AV_SIM_IP) $(AV_SIM_CMD_PORT) 2>&1 | grep -q succeeded && echo "✓ Port reachable" || echo "✗ Port not reachable (av-simulation might not be running)"
