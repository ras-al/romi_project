# Romi Project

This repository contains the complete simulation and autonomous exploration stack for the Romi differential drive robot. The project is split into two main components: a fully containerized Docker environment, and a ROS 2 workspace containing the simulation, mapping, and control logic.

## Project Structure

- **[`romi_ws/`](romi_ws/)**: The main ROS 2 Humble workspace. It contains the robot's URDF model, the Gazebo Ignition Fortress simulation world, the custom odometry plugins, and the Python scripts for autonomous exploration and 3D data collection. Please see the [Workspace README](romi_ws/README.md) for detailed technical documentation on the architecture and current challenges.
- **[`ros2_docker/`](ros2_docker/)**: A comprehensive Docker Compose environment that automatically handles all ROS 2, Gazebo, and GUI (X11) dependencies. It bind-mounts the `romi_ws` so you can develop on your host machine while running the code inside the container. Please see the [Docker README](ros2_docker/README.md) for setup and build instructions.

## Quick Start

1. **Build and start the container:**
   ```bash
   cd ros2_docker
   docker compose up -d
   ```
2. **Access the container and build the workspace:**
   ```bash
   docker compose exec ros2_gazebo bash
   cd /ros2_ws/Documents/robotics/romi_project/romi_ws
   colcon build
   source install/setup.bash
   ```
3. **Launch the autonomous simulation:**
   ```bash
   ros2 launch romi_gazebo romi_control.launch.py
   ```
