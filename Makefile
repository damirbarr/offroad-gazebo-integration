# Makefile for offroad-gazebo-integration
# Quick start for Gazebo off-road simulation with av-simulation integration

.PHONY: help build run run-udp run-world clean stop

# Docker image name
IMAGE_NAME := offroad-gazebo-integration
IMAGE_TAG := latest
CONTAINER_NAME := gazebo-sim

# Network settings for av-simulation integration
AV_SIM_IP ?= host.docker.internal
AV_SIM_CMD_PORT ?= 9001
AV_SIM_SENSOR_PORT ?= 9002

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
	@echo "  make run-world        - Run only Gazebo world"
	@echo "  make run-udp          - Run only UDP bridge (requires Gazebo running)"
	@echo "  make shell            - Open interactive bash shell in container"
	@echo "  make clean            - Remove Docker image"
	@echo "  make stop             - Stop running container"
	@echo ""
	@echo "Configuration:"
	@echo "  AV_SIM_IP=$(AV_SIM_IP)"
	@echo "  AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT)"
	@echo "  AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT)"
	@echo ""
	@echo "Example with custom IP:"
	@echo "  make run AV_SIM_IP=192.168.1.100"
	@echo ""

build:
	@echo "Building Docker image..."
	docker build -t $(IMAGE_NAME):$(IMAGE_TAG) .
	@echo "✓ Image built: $(IMAGE_NAME):$(IMAGE_TAG)"

run: build
	@echo "Starting Gazebo + UDP bridge..."
	@echo "  • Gazebo world at http://localhost:8080 (if GUI enabled)"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	docker run -it --rm \
		--name $(CONTAINER_NAME) \
		--network host \
		-e DISPLAY=$${DISPLAY} \
		-e AV_SIM_IP=$(AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash -c " \
			echo 'Launching Gazebo world...' && \
			ros2 launch offroad_gazebo_integration offroad_world.launch.py & \
			sleep 5 && \
			echo 'Launching UDP bridge...' && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
		"

run-world: build
	@echo "Starting Gazebo world only..."
	docker run -it --rm \
		--name $(CONTAINER_NAME)-world \
		--network host \
		-e DISPLAY=$${DISPLAY} \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		ros2 launch offroad_gazebo_integration offroad_world.launch.py

run-udp: build
	@echo "Starting UDP bridge only..."
	@echo "  (Ensure Gazebo is already running)"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-udp \
		--network host \
		-e AV_SIM_IP=$(AV_SIM_IP) \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
			av_sim_ip:=$(AV_SIM_IP) \
			av_sim_command_port:=$(AV_SIM_CMD_PORT) \
			av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT)

shell: build
	@echo "Opening interactive shell..."
	docker run -it --rm \
		--name $(CONTAINER_NAME)-shell \
		--network host \
		-e DISPLAY=$${DISPLAY} \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash

stop:
	@echo "Stopping containers..."
	-docker stop $(CONTAINER_NAME) $(CONTAINER_NAME)-world $(CONTAINER_NAME)-udp $(CONTAINER_NAME)-shell 2>/dev/null || true
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
