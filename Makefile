# Makefile for offroad-gazebo-integration
# Quick start for Gazebo off-road simulation with av-simulation integration

.PHONY: help build run run-inspection run-prius run-world run-world-prius clean stop check-avsim

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

# When Gazebo runs in Docker, loopback must resolve back to the host machine.
DOCKER_AV_SIM_IP := $(if $(filter 127.0.0.1 localhost,$(AV_SIM_IP)),host.docker.internal,$(AV_SIM_IP))

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
	@echo "  3. In another terminal: Run av-simulation with udp_gazebo adapter"
	@echo ""
	@echo "Commands:"
	@echo "  make build            - Build Docker image with ROS2 Humble + Gazebo"
	@echo "  make run              - Run inspection world + UDP bridge (VNC enabled)"
	@echo "  make run-inspection   - Same as make run (alias)"
	@echo "  make run-prius        - Run inspection world with Prius drive-by-wire vehicle"
	@echo "  make run-world        - Run offroad world (desert_terrain) + UDP bridge"
	@echo "  make run-world-prius  - Run offroad world with Prius drive-by-wire vehicle"
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
	@echo "Examples:"
	@echo "  make run AV_SIM_IP=192.168.10.128"
	@echo "  make run-prius AV_SIM_IP=127.0.0.1  # Host av-simulation on the default UDP ports"
	@echo "  make run AV_SIM_IP=192.168.10.128 LOG_LEVEL=debug"
	@echo "  make run USE_VNC=false  # No GUI"
	@echo ""

build:
	@echo "Building Docker image..."
	docker build -t $(IMAGE_NAME):$(IMAGE_TAG) .
	@echo "✓ Image built: $(IMAGE_NAME):$(IMAGE_TAG)"

run: build
	@echo "Starting CPR inspection world + UDP bridge..."
	@echo "  • Gazebo GUI: $(if $(filter false,$(USE_VNC)),disabled,open http://localhost:8080/vnc.html)"
	@echo "  • World: Inspection platforms with obstacles"
	@echo "  • Vehicle: inspection_robot tank-drive platform at (-15, 0, 0.5)"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(DOCKER_AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	@echo "  • Host port ownership: Gazebo publishes $(AV_SIM_CMD_PORT)/udp; av-simulation keeps $(AV_SIM_SENSOR_PORT)/udp"
	@echo ""
	@echo "Topics: /odom, /imu/data, /lidar, /mavros/global_position/global, /cmd_vel"
	docker run -it --rm \
		--name $(CONTAINER_NAME) \
		--add-host=host.docker.internal:host-gateway \
		-p 8080:8080 \
		-p $(AV_SIM_CMD_PORT):$(AV_SIM_CMD_PORT)/udp \
		-e AV_SIM_IP=$(DOCKER_AV_SIM_IP) \
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
				av_sim_ip:=$(DOCKER_AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				drive_mode:=tank \
				log_level:=$(LOG_LEVEL) \
		"

# Alias: run-inspection is the same as run
run-inspection: run

run-prius: build
	@echo "Starting CPR inspection world + Prius drive-by-wire vehicle..."
	@echo "  • Gazebo GUI: $(if $(filter false,$(USE_VNC)),disabled,open http://localhost:8080/vnc.html)"
	@echo "  • World: Inspection platforms with obstacles"
	@echo "  • Vehicle: prius_vehicle car at (-15, 0, 0.5) with GPS, IMU, and LiDAR"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(DOCKER_AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	@echo "  • Host port ownership: Gazebo publishes $(AV_SIM_CMD_PORT)/udp; av-simulation keeps $(AV_SIM_SENSOR_PORT)/udp"
	@echo ""
	@echo "Topics: /odom, /imu/data, /lidar, /mavros/global_position/global, /cmd_drive, /cmd_gear"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-prius \
		--add-host=host.docker.internal:host-gateway \
		-p 8080:8080 \
		-p $(AV_SIM_CMD_PORT):$(AV_SIM_CMD_PORT)/udp \
		-e AV_SIM_IP=$(DOCKER_AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		$(if $(filter false,$(USE_VNC)),bash,run_with_vnc.sh bash) -c " \
			echo 'Launching inspection world...' && \
			ros2 launch offroad_gazebo_integration inspection_world.launch.py headless:=false \
				vehicle_model:=prius_vehicle \
				vehicle_x:=-15.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 & \
			sleep 10 && \
			echo 'Launching UDP bridge...' && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(DOCKER_AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				drive_mode:=prius \
				log_level:=$(LOG_LEVEL) \
		"

run-world: build
	@echo "Starting offroad world (desert_terrain) + UDP bridge..."
	@echo "  • Gazebo GUI: $(if $(filter false,$(USE_VNC)),disabled,open http://localhost:8080/vnc.html)"
	@echo "  • World: desert_terrain with inspection_robot tank-drive vehicle"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(DOCKER_AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	@echo "  • Host port ownership: Gazebo publishes $(AV_SIM_CMD_PORT)/udp; av-simulation keeps $(AV_SIM_SENSOR_PORT)/udp"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-world \
		--add-host=host.docker.internal:host-gateway \
		-p 8080:8080 \
		-p $(AV_SIM_CMD_PORT):$(AV_SIM_CMD_PORT)/udp \
		-e AV_SIM_IP=$(DOCKER_AV_SIM_IP) \
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
				av_sim_ip:=$(DOCKER_AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				drive_mode:=tank \
				log_level:=$(LOG_LEVEL) \
		"

run-world-prius: build
	@echo "Starting offroad world (desert_terrain) + Prius drive-by-wire vehicle..."
	@echo "  • Gazebo GUI: $(if $(filter false,$(USE_VNC)),disabled,open http://localhost:8080/vnc.html)"
	@echo "  • World: desert_terrain with prius_vehicle drive-by-wire car"
	@echo "  • UDP bridge: receiving on 0.0.0.0:$(AV_SIM_CMD_PORT), sending to $(DOCKER_AV_SIM_IP):$(AV_SIM_SENSOR_PORT)"
	@echo "  • Host port ownership: Gazebo publishes $(AV_SIM_CMD_PORT)/udp; av-simulation keeps $(AV_SIM_SENSOR_PORT)/udp"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-world-prius \
		--add-host=host.docker.internal:host-gateway \
		-p 8080:8080 \
		-p $(AV_SIM_CMD_PORT):$(AV_SIM_CMD_PORT)/udp \
		-e AV_SIM_IP=$(DOCKER_AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		$(if $(filter false,$(USE_VNC)),bash,run_with_vnc.sh bash) -c " \
			echo 'Launching offroad world...' && \
			ros2 launch offroad_gazebo_integration offroad_world.launch.py headless:=false \
				world_file:=desert_terrain.sdf \
				world_name:=desert_terrain \
				vehicle_model:=prius_vehicle \
				vehicle_x:=0.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 & \
			sleep 10 && \
			echo 'Launching UDP bridge...' && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(DOCKER_AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				drive_mode:=prius \
				log_level:=$(LOG_LEVEL) \
		"

stop:
	@echo "Stopping containers..."
	-docker stop $(CONTAINER_NAME) $(CONTAINER_NAME)-prius $(CONTAINER_NAME)-world $(CONTAINER_NAME)-world-prius 2>/dev/null || true
	@echo "✓ Stopped"

clean:
	@echo "Removing Docker image..."
	docker rmi $(IMAGE_NAME):$(IMAGE_TAG) || true
	@echo "✓ Cleaned"

# Check if av-simulation is reachable
check-avsim:
	@echo "Checking av-simulation connectivity..."
	@echo "  Testing UDP port $(AV_SIM_CMD_PORT) on $(AV_SIM_IP)..."
	@nc -zvu $(AV_SIM_IP) $(AV_SIM_CMD_PORT) 2>&1 | grep -q succeeded && echo "✓ Port reachable" || echo "✗ Port not reachable (av-simulation might not be running)"
