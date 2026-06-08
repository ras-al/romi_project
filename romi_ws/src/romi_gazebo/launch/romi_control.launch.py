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

    pkg_gz   = get_package_share_directory('ros_gz_sim')
    pkg_romi = get_package_share_directory('romi_gazebo')

    sdf_file    = os.path.join(pkg_romi, 'models', 'tugbot_depot.sdf')
    urdf_file   = os.path.join(pkg_romi, 'models', 'romi_meshes.urdf')
    rviz_cfg    = os.path.join(pkg_romi, 'launch',  'romi_rviz.rviz')
    slam_params = os.path.join(pkg_romi, 'config',  'slam_params.yaml')
    ekf_params  = os.path.join(pkg_romi, 'config',  'ekf_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    sim = {'use_sim_time': True}

    launch_args = [
        DeclareLaunchArgument('explore', default_value='true'),
        DeclareLaunchArgument('coverage_stop_cells', default_value='600'),
        DeclareLaunchArgument('exploration_timeout', default_value='0.0'),
    ]

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items())

    # ── Bridge ────────────────────────────────────────────────────────────────
    # depth_camera/depth_image is the raw 32FC1 depth image from the RGBD sensor
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/romi/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/romi/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/world_demo/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
        ],
        parameters=[{'qos_overrides./model/romi.subscriber.reliability': 'reliable'}],
        output='screen')

    # ── Robot description ─────────────────────────────────────────────────────
    jsp = Node(package='joint_state_publisher', executable='joint_state_publisher',
               parameters=[{'robot_description': robot_desc}, sim], output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc}, sim], output='screen')

    # ── odom→base_link TF (proven reliable) ───────────────────────────────────
    odom_tf = Node(package='romi_gazebo', executable='odom_to_tf.py',
                   parameters=[sim], output='screen')

    # ── EKF sensor fusion (filtered odometry only, NO TF) ─────────────────────
    # The EKF fuses wheel odometry + IMU for a smoother pose estimate.
    # publish_tf is FALSE — odom_to_tf.py handles the TF.
    # The filtered output is used by the data recorder for cleaner trajectories.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
        output='screen')

    # ── Static TF chain ───────────────────────────────────────────────────────
    tf_b2m = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x','0','--y','0','--z','0.04',
                   '--roll','0','--pitch','0','--yaw','0',
                   '--frame-id','base_link',
                   '--child-frame-id','romi/sensor_mount'],
        parameters=[sim], output='screen')

    tf_m2rs = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x','0','--y','0','--z','0.02',
                   '--roll','0','--pitch','0','--yaw','0',
                   '--frame-id','romi/sensor_mount',
                   '--child-frame-id','romi/sensor_mount/realsense_d435'],
        parameters=[sim], output='screen')

    tf_m2lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x','0','--y','0','--z','0.05',
                   '--roll','0','--pitch','0','--yaw','0',
                   '--frame-id','romi/sensor_mount',
                   '--child-frame-id','romi/sensor_mount/lidar'],
        parameters=[sim], output='screen')

    # IMU link: sensor_mount → imu_link (z=0.03 from SDF imu_joint)
    tf_mount2imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x','0','--y','0','--z','0.03',
                   '--roll','0','--pitch','0','--yaw','0',
                   '--frame-id','romi/sensor_mount',
                   '--child-frame-id','imu_link'],
        parameters=[sim], output='screen')

    # ── Scan frame remapper ───────────────────────────────────────────────────
    remapper = Node(
        package='romi_gazebo', executable='scan_frame_remapper.py',
        parameters=[sim, {
            'input_topic':     '/lidar/scan',
            'output_topic':    '/lidar/scan_fixed',
            'target_frame_id': 'romi/sensor_mount/lidar',
        }], output='screen')

    # ── SLAM Toolbox ─────────────────────────────────────────────────────────
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', parameters=[slam_params], output='screen')

    # ── Data recorder ─────────────────────────────────────────────────────────
    recorder = Node(
        package='romi_gazebo', executable='data_recorder.py',
        parameters=[sim, {
            'auto_start': True,
            'odom_topic': '/odometry/filtered',
            'depth_topic': '/depth_camera/depth_image',
        }], output='screen')

    # ── Explorer ──────────────────────────────────────────────────────────────
    explorer = Node(
        package='romi_gazebo', executable='autonomous_explorer.py',
        parameters=[sim, {
            'linear_speed':         0.18,   # slower = more time to react
            'angular_speed':        1.5,
            'obstacle_threshold':   0.50,   # start steering away
            'emergency_threshold':  0.20,   # reverse (0.20 = robot edge + 0.12m)
            'side_threshold':       0.35,   # side steering zone
            'coverage_stop_cells':  LaunchConfiguration('coverage_stop_cells'),
            'exploration_timeout':  LaunchConfiguration('exploration_timeout'),
            'progress_watchdog_s':  60.0,
            'odom_topic': '/odometry/filtered',
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore')))

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], parameters=[sim], output='screen')

    return LaunchDescription([
        *launch_args,
        gz, bridge, jsp, rsp,
        odom_tf,                            # ← proven TF broadcaster (odom→base_link)
        ekf,                                # ← filtered odometry only (no TF)
        tf_b2m, tf_m2rs, tf_m2lidar,
        tf_mount2imu,
        remapper, slam,
        recorder, explorer, rviz,
    ])
