#!/usr/bin/env python3
 
import os
from ament_index_python.packages import get_package_share_directory
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
    # ── Args ──────────────────────────────────────────────────────
    explore_arg = DeclareLaunchArgument(
        'explore', default_value='false',
        description='Enable autonomous exploration')
 
    # ── Paths ─────────────────────────────────────────────────────
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')
    pkg_romi_gazebo = get_package_share_directory('romi_gazebo')
 
    sdf_file         = os.path.join(pkg_romi_gazebo, 'models', 'tugbot_depot.sdf')
    urdf_file        = os.path.join(pkg_romi_gazebo, 'models', 'romi_meshes.urdf')
    rviz_config      = os.path.join(pkg_romi_gazebo, 'launch', 'romi_rviz.rviz')
    slam_params_file = os.path.join(pkg_romi_gazebo, 'config', 'slam_params.yaml')
 
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()
 
    sim_time = {'use_sim_time': True}
 
    # ── Gazebo ────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items(),
    )
 
    # ── ROS ↔ Gazebo bridge ───────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/romi/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/romi/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/world_demo/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
        ],
        parameters=[{'qos_overrides./model/romi.subscriber.reliability': 'reliable'}],
        output='screen'
    )
 
    # ── Robot description ─────────────────────────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': robot_description}, sim_time],
        output='screen'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, sim_time],
        output='screen'
    )
 
    # ── TF chain ──────────────────────────────────────────────────
    # Seed map→odom so SLAM doesn't hit "no transform available" on
    # the first few scans.  SLAM overrides this immediately.
    map_to_odom_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[sim_time],
        output='screen'
    )
 
    odom_tf = Node(
        package='romi_gazebo',
        executable='odom_to_tf.py',
        parameters=[sim_time],
        output='screen'
    )
 
    base_to_sensor_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.04',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'romi/sensor_mount'],
        parameters=[sim_time],
        output='screen'
    )
 
    sensor_mount_to_realsense = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.02',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'romi/sensor_mount',
                   '--child-frame-id', 'romi/sensor_mount/realsense_d435'],
        parameters=[sim_time],
        output='screen'
    )
 
    # The LiDAR is 0.05 m above the sensor mount (from SDF pose).
    # This TF is what makes the scan-to-base projection correct.
    sensor_mount_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.05',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'romi/sensor_mount',
                   '--child-frame-id', 'romi/sensor_mount/lidar'],
        parameters=[sim_time],
        output='screen'
    )
 
    # ── FIX: Scan frame remapper ──────────────────────────────────
    # Gazebo Fortress sets LaserScan.header.frame_id =
    #   "romi::sensor_mount::lidar"  (colon-separated scoped name)
    # Our TF tree uses the slash-separated ROS name:
    #   "romi/sensor_mount/lidar"
    # SLAM drops every scan whose frame_id it can't find in TF.
    # This node republishes /lidar/scan as /lidar/scan_fixed with the
    # corrected frame_id so SLAM can resolve the transform.
    scan_remapper = Node(
        package='romi_gazebo',
        executable='scan_frame_remapper.py',
        parameters=[sim_time, {
            'input_topic':     '/lidar/scan',
            'output_topic':    '/lidar/scan_fixed',
            'target_frame_id': 'romi/sensor_mount/lidar',
        }],
        output='screen'
    )
 
    # ── SLAM Toolbox ──────────────────────────────────────────────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file],
        output='screen'
    )
 
    # ── Data recorder ─────────────────────────────────────────────
    data_recorder = Node(
        package='romi_gazebo',
        executable='data_recorder.py',
        parameters=[sim_time],
        output='screen'
    )
 
    # ── Explorer ──────────────────────────────────────────────────
    explorer = Node(
        package='romi_gazebo',
        executable='autonomous_explorer.py',
        parameters=[sim_time, {
            'angular_speed': 0.4,       # was 0.7 — slower turns = cleaner scans
            'linear_speed':  0.18,
            'turn_duration': 2.5,       # longer turn to fully clear obstacles
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore'))
    )
 
    # ── RViz ──────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[sim_time],
        output='screen'
    )
 
    return LaunchDescription([
        explore_arg,
        gz_sim,
        bridge,
        joint_state_publisher,
        robot_state_publisher,
        map_to_odom_init,
        odom_tf,
        base_to_sensor_mount,
        sensor_mount_to_realsense,
        sensor_mount_to_lidar,
        scan_remapper,          # ← new: fixes frame_id before SLAM sees it
        slam_toolbox,
        data_recorder,
        explorer,
        rviz2,
    ])
