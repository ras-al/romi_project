# Dockerfile for ROS2 Humble with Gazebo Fortress
# Includes all packages required for ROS2-Gazebo communication

FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=42
ENV GZ_VERSION=fortress

# Build arguments for user setup
# Defaults can be overridden via docker-compose.yml or --build-arg
ARG USERNAME=ros2_user
ARG USER_UID=1000
ARG USER_GID=1000

# Add Gazebo repository
RUN apt-get update && apt-get install -y \
    wget \
    gnupg \
    lsb-release \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Fortress and ROS2-Gazebo integration packages
RUN apt-get update && apt-get install -y \
    # Gazebo Fortress
    gz-fortress \
    # ROS2 Gazebo packages for communication (may need to be built from source if not available)
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    # ROS2 Control packages
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-diff-drive-controller \
    # Additional ROS2 packages for robot simulation
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-urdf \
    ros-humble-rviz2 \
    # Qt5 development packages for building qt_cpp
    libqt5serialport5-dev \
    libqt5gamepad5-dev \
    # OpenGL development libraries (for helper_opengl)
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    # Build tools and utilities
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    build-essential \
    git \
    vim \
    curl \
    # Network tools for ROS2 DDS
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages that might be needed
RUN pip3 install --no-cache-dir \
    transforms3d \
    pyyaml

# Create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory and set ownership
RUN mkdir -p /ros2_ws && chown -R $USERNAME:$USERNAME /ros2_ws

# Source ROS2 setup in bashrc for the user
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "export GZ_VERSION=fortress" >> /home/$USERNAME/.bashrc && \
    echo "export ROS_DOMAIN_ID=42" >> /home/$USERNAME/.bashrc && \
    echo "# Alias for Gazebo Fortress (ign is the command, gz is for newer versions)" >> /home/$USERNAME/.bashrc && \
    echo "alias gz='ign'" >> /home/$USERNAME/.bashrc

# Set up entrypoint
COPY ros2_docker/docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh && chown $USERNAME:$USERNAME /docker_entrypoint.sh

# Set working directory
WORKDIR /ros2_ws

# Switch to non-root user
USER $USERNAME

# Verify installations (optional - continue even if packages not found in ros2 pkg list)
RUN bash -c "source /opt/ros/humble/setup.bash && (ros2 pkg list | grep -q ros-gz && echo '✅ ROS2 Gazebo Fortress packages found' || echo '⚠️  Note: ros-gz packages may need to be built from source')" || true

ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]

