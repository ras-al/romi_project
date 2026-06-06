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
    # ── Launch Arguments ──────────────────────────────────────────
    explore_arg = DeclareLaunchArgument(
        'explore', default_value='false',
        description='Set to true to enable autonomous exploration')
 
    # ── Package paths ─────────────────────────────────────────────
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')
    pkg_romi_gazebo = get_package_share_directory('romi_gazebo')
 
    sdf_file             = os.path.join(pkg_romi_gazebo, 'models',  'tugbot_depot.sdf')
    robot_description_file = os.path.join(pkg_romi_gazebo, 'models', 'romi_meshes.urdf')
    rviz_config          = os.path.join(pkg_romi_gazebo, 'launch',  'romi_rviz.rviz')
    slam_params_file     = os.path.join(pkg_romi_gazebo, 'config',  'slam_params.yaml')
 
    with open(robot_description_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()
 
    sim_time = {'use_sim_time': True}
 
    # ── Gazebo ────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items(),
    )
 
    # ── ROS-Gazebo Bridge ─────────────────────────────────────────
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
 
    # ── Joint / Robot State Publishers ───────────────────────────
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
 
    # ── TF chain: map → odom → base_link → sensor frames ─────────
    # FIX: add a static map→odom initialiser so SLAM doesn't start
    # with a garbage offset. slam_toolbox will immediately override
    # this, but having it prevents the "no transform available" error
    # during the first few seconds that corrupts early scan matching.
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
 
    base_to_sensor_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.04',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'romi/sensor_mount'],
        parameters=[sim_time],
        output='screen'
    )
 
    sensor_mount_to_realsense_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.02',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'romi/sensor_mount',
                   '--child-frame-id', 'romi/sensor_mount/realsense_d435'],
        parameters=[sim_time],
        output='screen'
    )
 
    # FIX: lidar frame must match what slam_toolbox sees in LaserScan.header.frame_id
    # Gazebo Fortress publishes LaserScan with frame_id = "<model>::<link>::<sensor>"
    # We remap it to the ROS-friendly 'lidar_link' with this static TF.
    sensor_mount_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.05',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'romi/sensor_mount',
                   '--child-frame-id', 'romi/sensor_mount/lidar'],
        parameters=[sim_time],
        output='screen'
    )
 
    # ── SLAM Toolbox ─────────────────────────────────────────────
    # Using the params file so we can tune without recompiling
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file],
        output='screen'
    )
 
    # ── Data Recorder ─────────────────────────────────────────────
    data_recorder = Node(
        package='romi_gazebo',
        executable='data_recorder.py',
        parameters=[sim_time],
        output='screen'
    )
 
    # ── Autonomous Explorer (opt-in) ──────────────────────────────
    explorer = Node(
        package='romi_gazebo',
        executable='autonomous_explorer.py',
        parameters=[sim_time],
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore'))
    )
 
    # ── RViz2 ─────────────────────────────────────────────────────
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
        map_to_odom_init,      # <── new: prevents early TF gap
        odom_tf,
        base_to_sensor_mount_tf,
        sensor_mount_to_realsense_tf,
        sensor_mount_to_lidar_tf,
        slam_toolbox,
        data_recorder,
        explorer,
        rviz2,
    ])
