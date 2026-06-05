# ROS2 Humble with Gazebo Fortress Docker Container

This Docker container provides a complete development environment for ROS2 Humble with Gazebo Fortress (Ignition Gazebo 6), including all packages required for ROS2-Gazebo communication and robot simulation.

## Overview

The container includes:
- **ROS2 Humble** (desktop-full)
- **Gazebo Fortress** (Ignition Gazebo 6)
- **ROS2-Gazebo integration packages** (`ros-gz`, `ros-gz-bridge`, `ros-gz-sim`)
- **ROS2 Control** packages for robot control
- **Qt5 development libraries** for building Qt-based projects
- **Build tools** (colcon, vcstool, etc.)
- **Non-root user setup** matching your host user

## Prerequisites

1. **Docker** and **Docker Compose** installed
   ```bash
   # Check Docker version
   docker --version
   docker compose version
   ```

2. **X11 forwarding** configured (for GUI applications like Gazebo and RViz)
   ```bash
   # Add to ~/.bashrc to allow X11 connections automatically
   echo "xhost +local:docker" >> ~/.bashrc
   source ~/.bashrc
   
   # Or run once manually
   xhost +local:docker
   ```

3. **User ID and Group ID** (automatically detected from system)
   - The container automatically uses your system username from the `USER` environment variable
   - User ID and Group ID are automatically detected from `UID` and `GID` environment variables
   - If not set, defaults to `ros2_user` with UID/GID 1000
   - Check your IDs: `id -u` and `id -g`
   - Check your username: `echo $USER`

## Building the Container

### Option 1: Using Docker Compose (Recommended)

```bash
cd ros2_docker
docker compose build
```

### Option 2: Using Docker directly

```bash
cd ros2_docker
docker build \
  --build-arg USERNAME=$(whoami) \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -f Dockerfile \
  -t ros2_gazebo:latest \
  ..
```

## Rebuilding the Container

If you need to rebuild the container (e.g., after changing the Dockerfile or installing new packages):

### Option 1: Rebuild without cache (Recommended for clean rebuild)

```bash
cd ros2_docker
docker compose build --no-cache
```

### Option 2: Rebuild with cache (faster, uses cached layers)

```bash
cd ros2_docker
docker compose build
```

### Option 3: Rebuild and restart container

```bash
cd ros2_docker
# Stop existing container
docker compose down

# Rebuild and start
docker compose up -d --build
```

### Option 4: Force rebuild using Docker directly

```bash
cd ros2_docker
docker build \
  --build-arg USERNAME=${USER:-ros2_user} \
  --build-arg USER_UID=${UID:-1000} \
  --build-arg USER_GID=${GID:-1000} \
  --no-cache \
  -f Dockerfile \
  -t ros2_docker-ros2_gazebo \
  ..
```

**When to rebuild:**
- After modifying the `Dockerfile`
- After adding new system packages or dependencies
- When you want to ensure a clean build from scratch
- If the container is behaving unexpectedly and you suspect build issues

**Note:** 
- `--no-cache` forces a complete rebuild without using cached layers (slower but ensures everything is fresh)
- Without `--no-cache`, Docker will reuse cached layers for faster builds
- Rebuilding does not affect your mounted volumes or workspace files

## Running the Container

### Start the Container

```bash
cd ros2_docker
docker compose up -d
```

### Access the Container

```bash
# Interactive shell
docker compose exec ros2_gazebo bash

# Or using docker directly
docker exec -it ros2_gazebo_container bash
```

### Stop the Container

```bash
docker compose down
```

### Remove/Delete the Container

**Option 1: Stop and remove container (keeps volumes)**
```bash
docker compose down
```

**Option 2: Stop and remove container with volumes**
```bash
# Remove container and all associated volumes
docker compose down -v
```

**Option 3: Remove container, volumes, and images**
```bash
# Stop and remove container with volumes
docker compose down -v

# Remove the Docker image (optional)
docker rmi ros2_docker-ros2_gazebo
# Or if built with a different name:
docker images | grep ros2
docker rmi <image_id_or_name>
```

**Option 4: Complete cleanup (remove everything)**
```bash
# Stop and remove container with volumes
docker compose down -v

# Remove the image
docker rmi ros2_docker-ros2_gazebo

# Remove any orphaned volumes (if any)
docker volume prune
```

**Note:** 
- `docker compose down` stops and removes the container but keeps volumes (data persists)
- `docker compose down -v` also removes volumes (data is lost)
- Removing the image requires rebuilding the container next time

## Usage Examples

### Launch Gazebo with an Empty World

```bash
docker compose exec ros2_gazebo bash
source /opt/ros/humble/setup.bash
ign gazebo empty.sdf
```

### Launch a Differential Drive Robot Example

```bash
docker compose exec ros2_gazebo bash
source /opt/ros/humble/setup.bash

# Launch the example
ros2 launch ros_gz_sim_demos diff_drive.launch.py

# In another terminal, control the robot
docker compose exec ros2_gazebo bash
source /opt/ros/humble/setup.bash
ros2 topic pub --rate 10 /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### Build a ROS2 Workspace

```bash
docker compose exec ros2_gazebo bash
cd /ros2_ws/exploration/romi_robot_demos
colcon build
source install/setup.bash
```

### Launch RViz2

```bash
docker compose exec ros2_gazebo bash
source /opt/ros/humble/setup.bash
rviz2
```

## Container Features

### Environment Variables

- `ROS_DOMAIN_ID=42` - ROS2 DDS domain ID
- `GZ_VERSION=fortress` - Gazebo version
- `GZ_SIM_RESOURCE_PATH` - Paths to Gazebo resources
- `GZ_SIM_SYSTEM_PLUGIN_PATH` - Paths to Gazebo system plugins

### Mounted Volumes

- **Workspace**: `/ros2_ws/exploration/romi_robot_demos` (home directory mounted as `/ros2_ws`)
- **X11**: `/tmp/.X11-unix` (for GUI applications)
- **Gazebo Models**: Persistent volume for custom models

### User Configuration

The container runs as a non-root user matching your host user:
- Same UID/GID as host user
- Sudo access (passwordless)
- Home directory at `/home/<username>`

## Installed Packages

### ROS2 Packages
- `ros-humble-desktop-full` (base image)
- `ros-humble-ros-gz`
- `ros-humble-ros-gz-bridge`
- `ros-humble-ros-gz-sim`
- `ros-humble-ros2-control`
- `ros-humble-ros2-controllers`
- `ros-humble-controller-manager`
- `ros-humble-joint-state-broadcaster`
- `ros-humble-diff-drive-controller`
- `ros-humble-robot-state-publisher`
- `ros-humble-rviz2`
- `ros-humble-xacro`
- `ros-humble-urdf`

### Gazebo
- `gz-fortress` (Gazebo Fortress/Ignition Gazebo 6)

### Development Tools
- Qt5 development libraries
- OpenGL development libraries
- Build tools (colcon, vcstool, cmake, etc.)
- Python packages (transforms3d, pyyaml)

## File Structure

```
ros2_docker/
├── Dockerfile              # Container definition
├── docker-compose.yml      # Docker Compose configuration
├── docker_entrypoint.sh    # Entrypoint script
├── README.md               # This file
└── verify_setup.sh         # Setup verification script
```

## Quick Reference

### Common Commands

```bash
# Build container
docker compose build

# Start container
docker compose up -d

# Access container
docker compose exec ros2_gazebo bash

# View logs
docker compose logs -f ros2_gazebo

# Stop container
docker compose down

# Rebuild from scratch
docker compose build --no-cache
```

### Useful Aliases

Add to your `~/.bashrc`:
```bash
alias ros2-docker='docker compose -f /path/to/ros2_docker/docker-compose.yml exec ros2_gazebo bash'
```

## Additional Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Fortress Documentation](https://gazebosim.org/docs/fortress)
- [ROS2-Gazebo Integration](https://github.com/gazebosim/ros_gz)

## Notes

- The container uses `network_mode: host` for better ROS2 DDS communication
- X11 forwarding is required for GUI applications
- The workspace is mounted as a volume, so changes persist
- Gazebo models are stored in a persistent Docker volume

