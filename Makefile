# Makefile for offroad-gazebo-integration
# Quick start for Gazebo off-road simulation with av-simulation integration

.PHONY: help build run run-udp run-world run-world-headless run-inspection run-inspection-headless teleop clean stop

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
	@echo "  2. make run           - Run full stack (Gazebo + UDP bridge)"
	@echo "  3. In another terminal: Run av-simulation with udp_gazebo adapter"
	@echo ""
	@echo "Commands:"
	@echo "  make build            - Build Docker image with ROS2 Humble + Gazebo"
	@echo "  make run              - Run Gazebo world + UDP bridge (all-in-one)"
	@echo "  make run-inspection   - Run inspection world + UDP bridge (VNC enabled)"
	@echo "  make run-inspection-background - Run in background"
	@echo "  make run-world        - Run only Gazebo world (with GUI)"
	@echo "  make teleop           - Run keyboard teleop (requires inspection world running)"
	@echo "  make run-udp          - Run only UDP bridge (requires Gazebo running)"
	@echo "  make shell            - Open interactive bash shell in container"
	@echo "  make clean            - Remove Docker image"
	@echo "  make stop             - Stop running container"
	@echo ""
	@echo "Configuration:"
	@echo "  AV_SIM_IP=$(AV_SIM_IP)"
	@echo "  AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT)"
	@echo "  AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT)"
	@echo "  USE_VNC=$(USE_VNC) (default: true, GUI at http://localhost:8080/vnc.html)"
	@echo "  LOG_LEVEL=$(LOG_LEVEL) (info or debug)"
	@echo ""
	@echo "GUI (macOS/Linux):"
	@echo "  make run-world              - Gazebo UI in browser at http://localhost:8080/vnc.html"
	@echo "  make run-world HEADLESS=true - without UI"
	@echo ""
	@echo "Examples:"
	@echo "  make run-inspection AV_SIM_IP=192.168.10.128"
	@echo "  make run-inspection AV_SIM_IP=192.168.10.128 LOG_LEVEL=debug"
	@echo "  make run-inspection USE_VNC=false  # No GUI"
	@echo "  make run-inspection-background     # Run in background"
	@echo ""

build:
	@echo "Building Docker image..."
	docker build -t $(IMAGE_NAME):$(IMAGE_TAG) .
	@echo "✓ Image built: $(IMAGE_NAME):$(IMAGE_TAG)"

run: build
	@echo "Starting Gazebo + UDP bridge..."
	@echo "  • Gazebo GUI: $(if $(filter true,$(HEADLESS)),disabled,open http://localhost:8080/vnc.html)"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	docker run $(if $(filter true,$(HEADLESS)),-i,-it) --rm \
		--name $(CONTAINER_NAME) \
		-p 8080:8080 \
		-p 9001:9001/udp \
		-p 9002:9002/udp \
		-e AV_SIM_IP=$(AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		$(if $(filter true,$(HEADLESS)),bash,run_with_vnc.sh bash) -c " \
			echo 'Launching Gazebo world...' && \
			ros2 launch offroad_gazebo_integration inspection_world.launch.py headless:=$(HEADLESS) & \
			sleep 5 && \
			echo 'Launching UDP bridge...' && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
		"

run-world: build
	@echo "Starting Gazebo world only..."
	@echo "  • Gazebo GUI: $(if $(filter true,$(HEADLESS)),disabled,open http://localhost:8080/vnc.html)"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-world \
		-p 8080:8080 \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		$(if $(filter true,$(HEADLESS)),,run_with_vnc.sh )ros2 launch offroad_gazebo_integration offroad_world.launch.py headless:=$(HEADLESS)

run-world-headless: build
	@echo "Starting Gazebo world (headless, no GUI)..."
	docker run -it --rm \
		--name $(CONTAINER_NAME)-world \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		ros2 launch offroad_gazebo_integration offroad_world.launch.py headless:=true

run-inspection: build
	@echo "Starting CPR inspection world + UDP bridge..."
	@echo "  • Gazebo GUI: $(if $(filter false,$(USE_VNC)),disabled,open http://localhost:8080/vnc.html)"
	@echo "  • World: Inspection platforms with obstacles"
	@echo "  • Robot: Auto-spawns in 8 seconds at (-10, 0, 0.5) with GPS, IMU, and LiDAR"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	@echo ""
	@echo "Topics: /odom, /imu/data, /lidar, /mavros/global_position/global, /cmd_vel"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-inspection \
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
				vehicle_x:=-10.0 \
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

run-inspection-background: build
	@echo "Starting CPR inspection world + UDP bridge in background..."
	@echo "  • Gazebo GUI: $(if $(filter false,$(USE_VNC)),disabled,open http://localhost:8080/vnc.html)"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	docker run -d --rm \
		--name $(CONTAINER_NAME)-inspection \
		-p 8080:8080 \
		-p 9001:9001/udp \
		-p 9002:9002/udp \
		-e AV_SIM_IP=$(AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		$(if $(filter false,$(USE_VNC)),bash,run_with_vnc.sh bash) -c " \
			ros2 launch offroad_gazebo_integration inspection_world.launch.py headless:=false \
				vehicle_x:=-10.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 & \
			sleep 10 && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				log_level:=$(LOG_LEVEL) \
		"
	@echo "Container started in background. Use 'docker logs -f $(CONTAINER_NAME)-inspection' to view logs"

teleop: build
	@echo "Starting keyboard teleoperation..."
	@echo "  Ensure inspection world is running with 'make run-inspection'"
	@echo ""
	@echo "Controls:"
	@echo "  W/S - Forward/Backward"
	@echo "  A/D - Turn Left/Right"
	@echo "  Space - Stop"
	@echo "  Q/E - Increase/Decrease speed"
	@echo "  X - Quit"
	docker exec -it $(CONTAINER_NAME)-inspection bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run offroad_gazebo_integration teleop_keyboard"

run-udp: build
	@echo "Starting UDP bridge only..."
	@echo "  (Ensure Gazebo is already running)"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-udp \
		-p 9001:9001/udp \
		-p 9002:9002/udp \
		-e AV_SIM_IP=$(AV_SIM_IP) \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
			av_sim_ip:=$(AV_SIM_IP) \
			av_sim_command_port:=$(AV_SIM_CMD_PORT) \
			av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT)

shell: build
	@echo "Opening interactive shell..."
	@echo "  Tip: run 'run_with_vnc.sh ros2 launch offroad_gazebo_integration offroad_world.launch.py' for GUI at http://localhost:8080/vnc.html"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-shell \
		-p 8080:8080 \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash

stop:
	@echo "Stopping containers..."
	-docker stop $(CONTAINER_NAME) $(CONTAINER_NAME)-world $(CONTAINER_NAME)-inspection $(CONTAINER_NAME)-udp $(CONTAINER_NAME)-shell 2>/dev/null || true
	@echo "✓ Stopped"

clean:
	@echo "Removing Docker image..."
	docker rmi $(IMAGE_NAME):$(IMAGE_TAG) || true
	@echo "✓ Cleaned"

# Development targets
dev-build: build
	@echo "✓ Development build complete"

dev-test: build
	@echo "Running tests..."
	docker run --rm \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash -c "source /workspace/install/setup.bash && colcon test"

# Check if av-simulation is reachable
check-avsim:
	@echo "Checking av-simulation connectivity..."
	@echo "  Testing UDP port $(AV_SIM_CMD_PORT) on $(AV_SIM_IP)..."
	@nc -zvu $(AV_SIM_IP) $(AV_SIM_CMD_PORT) 2>&1 | grep -q succeeded && echo "✓ Port reachable" || echo "✗ Port not reachable (av-simulation might not be running)"
