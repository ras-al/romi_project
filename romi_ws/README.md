# Romi Autonomous Exploration & Mapping

This workspace contains the complete ROS 2 (Humble) + Gazebo Ignition (Fortress) simulation environment for the Romi differential drive robot. It features a fully autonomous exploration algorithm, a robust 2D SLAM pipeline, and a comprehensive 3D dataset recording system.

## Architecture Overview

1. **Simulation Engine**: Gazebo Ignition Fortress (`ros_gz_sim`). The primary environment is a warehouse depot (`tugbot_depot.sdf`) where the Romi robot is spawned.
2. **Robot Model**: A custom URDF (`romi_meshes.urdf`) containing the physical specifications of Romi. The model is equipped with:
   - **Differential Drive Encoders**: A custom C++ plugin (`romi_encoder_plugin.cpp`) simulates precise odometry and interfaces with ROS 2 via `cmd_vel` and `odom`.
   - **Sensor Suite**:
     - 2D LiDAR (360° scan)
     - RealSense Depth & RGB Camera
     - IMU
3. **Bridge Node**: `ros_gz_bridge` is used to pass messages (Odometry, LiDAR, Camera, TF) between the Ignition transport layer and ROS 2 topics.
4. **Mapping**: `slam_toolbox` (async) consumes the filtered LiDAR scans to generate a high-fidelity 2D occupancy grid of the environment.

## Autonomous Exploration State Machine
`autonomous_explorer.py` is the brain of the operation, executing a reactive, coverage-aware state machine.

### The Coverage Grid
Rather than wandering randomly, the robot maintains a lightweight `CoverageGrid` that divides the world into `0.5m` cells. As the robot moves, it marks its current cell as "visited". 
When an obstacle blocks the path, the robot evaluates four candidate turn directions (hard left, soft left, soft right, hard right). For each direction, it casts a 3.0m lookahead cone and counts how many *unvisited* cells lie in that path. The robot always turns toward the direction with the highest coverage score, ensuring it explores new territory instead of revisiting old areas.

### The States
- `EXPLORING`: The robot moves forward, applying a subtle wall-following hysteresis (drifting slightly away if walls get too close) and random noise to avoid infinite straight lines.
- `TURNING`: The robot pivots in place toward the highest-scoring unexplored direction until the path ahead is fully clear.
- `REVERSING`: Triggered by the **Emergency Collision Guard**. If *any* part of the robot's front bumper (front, front-left, or front-right) comes within `0.22m` of an obstacle, the robot immediately hits the brakes and reverses for 1.0s before turning.
- `RECOVERING`: A fallback "Stuck Guard". If the odometry detects that the robot hasn't moved at least `0.06m` over the last 60 control ticks, it assumes the robot is physically snagged and executes a 2.0s forced spin to break free.
- `DONE`: Reached when the `coverage_stop_cells` threshold is met (e.g., 600 cells). The robot stops, triggers the map saver, flushes all recordings, and shuts down safely.

But these are not working as expected. It always gets stock or goes in circles.

## Data Collection Pipeline
For 3D reconstruction teams (NeRF, Gaussian Splatting, COLMAP), `data_recorder.py` runs passively in the background, triggered on launch.

When the exploration finishes, the `data/romi_capture_YYYYMMDD_HHMMSS/` directory contains:
- `pointclouds/`: Binary little-endian PLY files (`cloud_XXXXXX.ply`) containing XYZ points and RGB color data. Binary PLYs are 5x smaller and 10x faster to write than ASCII.
- `images/`: Raw JPEG frames (`frame_XXXXXX.jpg`) synced exactly with the point clouds. Essential for texture generation in COLMAP.
- `camera_info.json`: The RealSense camera intrinsics (`fx`, `fy`, `cx`, `cy`, width, height).
- `images.txt`: Extrinsic camera poses written directly in COLMAP text format (`QW QX QY QZ TX TY TZ 1 filename`). This saves pipeline engineers from writing custom conversion scripts.
- `odometry.csv` & `ground_truth.csv`: Raw trajectory data for evaluation.

## Historical Technical Challenges & Solutions

Developing a stable autonomous differential drive robot in Gazebo presented several major physics and timing challenges.

### 1. Odometry Drift & "Infinity Shooting"
**Problem**: The odometry map (red path in RViz) would occasionally shoot off to infinity in a perfectly straight line, completely destroying the map.

### 2. The Collision Recovery Loop ("Rounding Corners")
**Problem**: When the robot hit an obstacle, it would reverse, but then spend entirely too long spinning in place, often getting stuck "rounding" the corner for 5+ seconds.

### 3. The "Rubber Eraser" Effect
**Problem**: The robot was moving erratically and stuttering across the floor, making autonomous navigation impossible.
