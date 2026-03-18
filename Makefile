# Makefile for offroad-gazebo-integration
# Quick start for Gazebo off-road simulation with av-simulation integration

.PHONY: help build check-image run run-inspection run-prius run-world run-world-prius report-topics report-topics-prius clean stop check-avsim

# Docker image name
IMAGE_NAME := offroad-gazebo-integration
IMAGE_TAG := latest
IMAGE_REF := $(IMAGE_NAME):$(IMAGE_TAG)
CONTAINER_NAME := gazebo-sim

# Network settings for av-simulation integration
AV_SIM_IP ?= 192.168.10.128 # THE AV-SIMULATION IP
AV_SIM_CMD_PORT ?= 9001
AV_SIM_SENSOR_PORT ?= 9002

# VNC: set to false to disable VNC/GUI (default: true, always has VNC)
USE_VNC ?= true

# Logging: info (default) or debug (verbose packet inspection)
LOG_LEVEL ?= info

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
	@echo "  1. make build         - Build Docker image"
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
	@echo "  make report-topics    - Launch inspection_robot headless and print the published topic contract"
	@echo "  make report-topics-prius - Launch prius_vehicle headless and print the published topic contract"
	@echo "  make clean            - Remove Docker image"
	@echo "  make stop             - Stop running container"
	@echo ""
	@echo "Note: run targets use the existing Docker image and will ask you to run make build if it is missing."
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
	@echo "  make run AV_SIM_IP=192.168.10.128 LOG_LEVEL=debug  # Detailed UDP bridge tracing"
	@echo "  make run USE_VNC=false  # No GUI"
	@echo ""

build:
	@echo "Building Docker image..."
	docker build -t $(IMAGE_REF) .
	@echo "✓ Image built: $(IMAGE_REF)"

check-image:
	@docker image inspect $(IMAGE_REF) >/dev/null 2>&1 || { \
		echo "Docker image $(IMAGE_REF) not found. Run 'make build' first."; \
		exit 1; \
	}

run: check-image
	@echo "Starting inspection world with inspection_robot (tank). GUI=$(if $(filter false,$(USE_VNC)),disabled,http://localhost:8080/vnc.html), UDP=0.0.0.0:$(AV_SIM_CMD_PORT)->$(DOCKER_AV_SIM_IP):$(AV_SIM_SENSOR_PORT), LOG_LEVEL=$(LOG_LEVEL)"
	docker run -it --rm \
		--name $(CONTAINER_NAME) \
		--add-host=host.docker.internal:host-gateway \
		-p 8080:8080 \
		-p $(AV_SIM_CMD_PORT):$(AV_SIM_CMD_PORT)/udp \
		-e AV_SIM_IP=$(DOCKER_AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_REF) \
		$(if $(filter false,$(USE_VNC)),bash,run_with_vnc.sh bash) -c " \
			ros2 launch offroad_gazebo_integration inspection_world.launch.py headless:=false \
				vehicle_model:=inspection_robot \
				vehicle_x:=-15.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 & \
			sleep 10 && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(DOCKER_AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				drive_mode:=tank \
				log_level:=$(LOG_LEVEL) \
		"

# Alias: run-inspection is the same as run
run-inspection: run

run-prius: check-image
	@echo "Starting inspection world with prius_vehicle (Prius mode). GUI=$(if $(filter false,$(USE_VNC)),disabled,http://localhost:8080/vnc.html), UDP=0.0.0.0:$(AV_SIM_CMD_PORT)->$(DOCKER_AV_SIM_IP):$(AV_SIM_SENSOR_PORT), LOG_LEVEL=$(LOG_LEVEL)"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-prius \
		--add-host=host.docker.internal:host-gateway \
		-p 8080:8080 \
		-p $(AV_SIM_CMD_PORT):$(AV_SIM_CMD_PORT)/udp \
		-e AV_SIM_IP=$(DOCKER_AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_REF) \
		$(if $(filter false,$(USE_VNC)),bash,run_with_vnc.sh bash) -c " \
			ros2 launch offroad_gazebo_integration inspection_world.launch.py headless:=false \
				vehicle_model:=prius_vehicle \
				vehicle_x:=-15.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 & \
			sleep 10 && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(DOCKER_AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				drive_mode:=prius \
				log_level:=$(LOG_LEVEL) \
		"

run-world: check-image
	@echo "Starting offroad world desert_terrain with inspection_robot (tank). GUI=$(if $(filter false,$(USE_VNC)),disabled,http://localhost:8080/vnc.html), UDP=0.0.0.0:$(AV_SIM_CMD_PORT)->$(DOCKER_AV_SIM_IP):$(AV_SIM_SENSOR_PORT), LOG_LEVEL=$(LOG_LEVEL)"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-world \
		--add-host=host.docker.internal:host-gateway \
		-p 8080:8080 \
		-p $(AV_SIM_CMD_PORT):$(AV_SIM_CMD_PORT)/udp \
		-e AV_SIM_IP=$(DOCKER_AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_REF) \
		$(if $(filter false,$(USE_VNC)),bash,run_with_vnc.sh bash) -c " \
			ros2 launch offroad_gazebo_integration offroad_world.launch.py headless:=false \
				world_file:=desert_terrain.sdf \
				world_name:=desert_terrain \
				vehicle_model:=inspection_robot \
				vehicle_x:=0.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 & \
			sleep 10 && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(DOCKER_AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				drive_mode:=tank \
				log_level:=$(LOG_LEVEL) \
		"

run-world-prius: check-image
	@echo "Starting offroad world desert_terrain with prius_vehicle (Prius mode). GUI=$(if $(filter false,$(USE_VNC)),disabled,http://localhost:8080/vnc.html), UDP=0.0.0.0:$(AV_SIM_CMD_PORT)->$(DOCKER_AV_SIM_IP):$(AV_SIM_SENSOR_PORT), LOG_LEVEL=$(LOG_LEVEL)"
	docker run -it --rm \
		--name $(CONTAINER_NAME)-world-prius \
		--add-host=host.docker.internal:host-gateway \
		-p 8080:8080 \
		-p $(AV_SIM_CMD_PORT):$(AV_SIM_CMD_PORT)/udp \
		-e AV_SIM_IP=$(DOCKER_AV_SIM_IP) \
		-e AV_SIM_CMD_PORT=$(AV_SIM_CMD_PORT) \
		-e AV_SIM_SENSOR_PORT=$(AV_SIM_SENSOR_PORT) \
		$(IMAGE_REF) \
		$(if $(filter false,$(USE_VNC)),bash,run_with_vnc.sh bash) -c " \
			ros2 launch offroad_gazebo_integration offroad_world.launch.py headless:=false \
				world_file:=desert_terrain.sdf \
				world_name:=desert_terrain \
				vehicle_model:=prius_vehicle \
				vehicle_x:=0.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 & \
			sleep 10 && \
			ros2 launch offroad_gazebo_integration udp_bridge.launch.py \
				av_sim_ip:=$(DOCKER_AV_SIM_IP) \
				av_sim_command_port:=$(AV_SIM_CMD_PORT) \
				av_sim_sensor_port:=$(AV_SIM_SENSOR_PORT) \
				drive_mode:=prius \
				log_level:=$(LOG_LEVEL) \
		"

report-topics: check-image
	@echo "Reporting published topics for inspection_robot"
	docker run --rm \
		--name $(CONTAINER_NAME)-report \
		$(IMAGE_REF) \
		bash -lc " \
			ros2 launch offroad_gazebo_integration inspection_world.launch.py headless:=true \
				vehicle_model:=inspection_robot \
				vehicle_x:=-15.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 >/tmp/world.log 2>&1 & \
			LAUNCH_PID=\$$! && \
			sleep 28 && \
			/workspace/src/offroad_gazebo_integration/test_bridge.sh && \
			kill \$$LAUNCH_PID 2>/dev/null || true && \
			wait \$$LAUNCH_PID 2>/dev/null || true \
		"

report-topics-prius: check-image
	@echo "Reporting published topics for prius_vehicle"
	docker run --rm \
		--name $(CONTAINER_NAME)-report-prius \
		$(IMAGE_REF) \
		bash -lc " \
			ros2 launch offroad_gazebo_integration inspection_world.launch.py headless:=true \
				vehicle_model:=prius_vehicle \
				vehicle_x:=-15.0 \
				vehicle_y:=0.0 \
				vehicle_z:=0.5 >/tmp/world.log 2>&1 & \
			LAUNCH_PID=\$$! && \
			sleep 28 && \
			/workspace/src/offroad_gazebo_integration/test_bridge.sh && \
			kill \$$LAUNCH_PID 2>/dev/null || true && \
			wait \$$LAUNCH_PID 2>/dev/null || true \
		"

stop:
	@echo "Stopping containers..."
	-docker stop $(CONTAINER_NAME) $(CONTAINER_NAME)-prius $(CONTAINER_NAME)-world $(CONTAINER_NAME)-world-prius 2>/dev/null || true
	@echo "✓ Stopped"

clean:
	@echo "Removing Docker image..."
	docker rmi $(IMAGE_REF) || true
	@echo "✓ Cleaned"

# Check if av-simulation is reachable
check-avsim:
	@echo "Checking av-simulation connectivity..."
	@echo "  Testing UDP port $(AV_SIM_CMD_PORT) on $(AV_SIM_IP)..."
	@nc -zvu $(AV_SIM_IP) $(AV_SIM_CMD_PORT) 2>&1 | grep -q succeeded && echo "✓ Port reachable" || echo "✗ Port not reachable (av-simulation might not be running)"
