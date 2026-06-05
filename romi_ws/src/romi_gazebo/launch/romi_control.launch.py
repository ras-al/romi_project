#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_romi_gazebo = get_package_share_directory('romi_gazebo')

    # Get the path to SDF, URDF, and RViz config
    sdf_file = os.path.join(pkg_romi_gazebo, 'models', 'tugbot_depot.sdf')
    robot_description_file = os.path.join(pkg_romi_gazebo, 'models', 'romi_meshes.urdf')
    rviz_config = os.path.join(pkg_romi_gazebo, 'launch', 'romi_rviz.rviz')

    with open(robot_description_file, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    # ── Gazebo Simulation ─────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {sdf_file}'
        }.items(),
    )

    # ── ROS-Gazebo Bridge ─────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Velocity commands (bidirectional)
            '/model/romi/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry (bidirectional)
            '/model/romi/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # Depth camera point cloud (GZ → ROS)
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # LiDAR scan (GZ → ROS)
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Depth camera image (GZ → ROS)
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # IMU data (GZ → ROS)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Ground truth pose (GZ → ROS)
            '/world/world_demo/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
        ],
        parameters=[{
            'qos_overrides./model/romi.subscriber.reliability': 'reliable'
        }],
        output='screen'
    )

    # ── Joint State Publisher ─────────────────────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # ── Robot State Publisher ─────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # ── Odom → base_link TF Broadcaster ──────────────────────────
    odom_tf = Node(
        package='romi_gazebo',
        executable='odom_to_tf.py',
        output='screen'
    )

    # ── Static TF Transforms ─────────────────────────────────────
    base_to_sensor_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.04', '0', '0', '0', 'base_link', 'romi/sensor_mount'],
        output='screen'
    )

    sensor_mount_to_realsense_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.02', '0', '0', '0', 'romi/sensor_mount', 'romi/sensor_mount/realsense_d435'],
        output='screen'
    )

    sensor_mount_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', 'romi/sensor_mount', 'romi/sensor_mount/lidar'],
        output='screen'
    )

    # ── Data Recorder (service-triggered) ─────────────────────────
    data_recorder = Node(
        package='romi_gazebo',
        executable='data_recorder.py',
        output='screen'
    )

    # ── RViz2 Visualization ───────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        joint_state_publisher,
        robot_state_publisher,
        odom_tf,
        base_to_sensor_mount_tf,
        sensor_mount_to_realsense_tf,
        sensor_mount_to_lidar_tf,
        data_recorder,
        rviz2,
    ])
