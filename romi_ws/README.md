# Romi Autonomous Exploration & Mapping

This workspace contains the complete ROS 2 (Humble) + Gazebo Ignition (Fortress) simulation environment for the Romi differential drive robot. It features a fully autonomous exploration algorithm based on reactive potential-field steering, a robust 2D SLAM pipeline, and a comprehensive 3D dataset recording system.

## Architecture Overview

1. **Simulation Engine**: Gazebo Ignition Fortress (`ros_gz_sim`). The primary environment is a warehouse depot (`tugbot_depot.sdf`) where the Romi robot is spawned.
2. **Robot Model**: A custom URDF (`romi_meshes.urdf`) containing the physical specifications of Romi. The model is equipped with:
   - **Differential Drive Encoders**: A custom C++ plugin (`romi_encoder_plugin.cpp`) simulates precise odometry and interfaces with ROS 2 via `cmd_vel` and `odom`.
   - **Sensor Suite**:
     - 2D LiDAR (360° scan)
     - RealSense Depth & RGB Camera
     - IMU
3. **State Estimation**: An Extended Kalman Filter (`robot_localization`) fuses wheel odometry and IMU angular velocities to produce highly stable continuous transforms (`/odometry/filtered`).
4. **Bridge Node**: `ros_gz_bridge` is used to pass messages (Odometry, LiDAR, Camera, TF) between the Ignition transport layer and ROS 2 topics.
5. **Mapping**: `slam_toolbox` (async) consumes the filtered LiDAR scans and EKF odometry to generate a high-fidelity 2D occupancy grid of the environment.

## Autonomous Exploration Architecture

The robot's navigation is governed by `autonomous_explorer.py`. Previous iterations relied on a fragile multi-state sequence. The current architecture employs a continuous, physics-aware **Reactive Potential-Field Steering** system to gracefully maneuver through complex environments.

### Physics-Aware Navigation
Navigation thresholds are strictly derived from the robot's physical dimensions (extracted from `tugbot_depot.sdf`):
- **Chassis Dimensions**: 0.16m × 0.12m × 0.04m
- **Body Extent**: 0.08m from the LiDAR center to the lateral edge.
- **Minimum Safe Passage**: Computed as body width plus a dynamic clearance margin (`MIN_SIDE_DIST = 0.20m`). 

A dedicated **Gap-Width Check** ensures the robot proactively refuses to enter passages narrower than its physical dimensions, resolving the issue of lateral collisions against vertical supports.

### Reactive Control Loop
The main control loop evaluates 7 discrete radial sectors from the 2D LiDAR to generate continuous steering vectors:
- **Repulsive Steering**: Obstacles within the influence radius exert a repulsive force inversely proportional to distance ($\propto \frac{1}{d^2}$). Lateral obstacles push the robot toward the center of passages, enabling smooth, continuous curvature around corners rather than abrupt stop-and-spin maneuvers.
- **Dynamic Velocity Profiling**: Linear velocity scales proportionally with forward clearance. Wide-open spaces allow maximum velocity, while confined spaces induce a controlled crawl.

### State Machine
The state machine has been streamlined to just two primary states:
1. `DRIVING`: The default state. The robot continuously computes potential-field steering and velocity. If the forward path is obstructed, it evaluates the surrounding grid to prioritize turning toward unexplored areas using the `CoverageGrid` logic.
2. `REVERSING`: Triggered strictly as a safety override. If an obstacle breaches the `emergency_threshold` (0.20m) or if the scan-based stuck timer detects proximity for > 0.5s, the robot executes an immediate reverse trajectory. If reversing fails to clear the obstruction, it recursively flips direction to disentangle itself from complex geometric traps.

### Coverage Grid
To ensure exhaustive exploration, the robot tracks visited regions using a discrete `CoverageGrid`. When forced to decide between multiple clear paths, it scores directions by casting a lookahead vector and counting unexplored cells, naturally guiding the robot into uncharted territory.

## Data Collection Pipeline

For 3D reconstruction pipelines (NeRF, Gaussian Splatting, COLMAP), `data_recorder.py` runs synchronously in the background. To optimize data quality and storage, recording automatically pauses during `REVERSING` or tight maneuvers, capturing only progressive exploration data.

When the exploration finishes, the `data/romi_capture_YYYYMMDD_HHMMSS/` directory contains:
- `pointclouds/`: Binary little-endian PLY files (`cloud_XXXXXX.ply`) containing XYZ points and RGB color data. Binary PLYs are 5x smaller and 10x faster to write than ASCII.
- `images/`: Raw JPEG frames (`frame_XXXXXX.jpg`) synced exactly with the point clouds. Essential for texture generation in COLMAP.
- `camera_info.json`: The RealSense camera intrinsics (`fx`, `fy`, `cx`, `cy`, width, height).
- `images.txt`: Extrinsic camera poses written directly in COLMAP text format (`QW QX QY QZ TX TY TZ 1 filename`). This saves pipeline engineers from writing custom conversion scripts.
- `odometry.csv`, `odometry_raw.csv`, `imu.csv` & `ground_truth.csv`: High-frequency synchronized trajectory and sensor data.

## Deployment Notes

1. **EKF Covariance Tuning**: Ensure all covariances in `ekf_params.yaml` use explicit floating-point representation (`0.0`) to satisfy strict ROS 2 type-checking.
2. **Session Recording**: The data recorder manages a single persistent session per launch. Pausing and resuming operations append to the existing trajectory, maintaining continuous frame indices.
