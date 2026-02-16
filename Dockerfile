# Dockerfile for offroad-gazebo-integration
# ROS2 Humble + Gazebo Harmonic + all dependencies

FROM ros:humble-ros-core-jammy

# Install system dependencies + VNC for GUI in browser (macOS/Docker)
RUN apt-get update && apt-get install -y \
    wget \
    gnupg \
    lsb-release \
    curl \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y gz-harmonic && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    pillow \
    pyyaml

# Create workspace
WORKDIR /workspace
RUN mkdir -p /workspace/src

# Copy package
COPY . /workspace/src/offroad_gazebo_integration/

# Copy VNC startup script
COPY scripts/run_with_vnc.sh /usr/local/bin/run_with_vnc.sh
RUN chmod +x /usr/local/bin/run_with_vnc.sh

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /workspace && \
    colcon build --symlink-install"

# Setup entrypoint
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
