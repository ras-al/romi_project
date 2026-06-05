#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_romi_gazebo = get_package_share_directory('romi_gazebo')
    
    # Get the path to the SDF file
    sdf_file = os.path.join(pkg_romi_gazebo, 'models', 'tugbot_depot.sdf')
    robot_description_file = os.path.join(pkg_romi_gazebo, 'models', 'romi_meshes.urdf')

    with open(robot_description_file, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()
    
    # Launch Gazebo with the world file (similar to working example)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {sdf_file}'
        }.items(),
    )
    
    # Bridge for cmd_vel and odometry (like the working example)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/romi/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/romi/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        parameters=[{
            'qos_overrides./model/romi.subscriber.reliability': 'reliable'
        }],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    odom_tf = Node(
        package='romi_gazebo',
        executable='odom_to_tf.py',
        output='screen'
    )

    base_to_sensor_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.04', '0', '0', '3.14159', 'base_link', 'romi/sensor_mount'],
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
    
    return LaunchDescription([
        gz_sim,
        bridge,
        robot_state_publisher,
        odom_tf,
        base_to_sensor_mount_tf,
        sensor_mount_to_realsense_tf,
        sensor_mount_to_lidar_tf,
    ])

