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
 
    explore_arg = DeclareLaunchArgument(
        'explore', default_value='true',          # default ON now
        description='Enable autonomous exploration')
    timeout_arg = DeclareLaunchArgument(
        'coverage_stop_cells', default_value='600',
        description='Stop when this many 0.5 m grid cells are visited')
 
    pkg_ros_gz  = get_package_share_directory('ros_gz_sim')
    pkg_romi    = get_package_share_directory('romi_gazebo')
 
    sdf_file    = os.path.join(pkg_romi, 'models',  'tugbot_depot.sdf')
    urdf_file   = os.path.join(pkg_romi, 'models',  'romi_meshes.urdf')
    rviz_cfg    = os.path.join(pkg_romi, 'launch',  'romi_rviz.rviz')
    slam_params = os.path.join(pkg_romi, 'config',  'slam_params.yaml')
 
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_desc = f.read()
 
    sim = {'use_sim_time': True}
 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items())
 
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/romi/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/romi/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/world_demo/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
        ],
        parameters=[{'qos_overrides./model/romi.subscriber.reliability': 'reliable'}],
        output='screen')
 
    jsp = Node(package='joint_state_publisher', executable='joint_state_publisher',
               parameters=[{'robot_description': robot_desc}, sim], output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc}, sim], output='screen')
 
    odom_tf = Node(package='romi_gazebo', executable='odom_to_tf.py',
                   parameters=[sim], output='screen')
 
    tf_base_to_mount = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x','0','--y','0','--z','0.04',
                   '--roll','0','--pitch','0','--yaw','0',
                   '--frame-id','base_link','--child-frame-id','romi/sensor_mount'],
        parameters=[sim], output='screen')
 
    tf_mount_to_rs = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x','0','--y','0','--z','0.02',
                   '--roll','0','--pitch','0','--yaw','0',
                   '--frame-id','romi/sensor_mount',
                   '--child-frame-id','romi/sensor_mount/realsense_d435'],
        parameters=[sim], output='screen')
 
    tf_mount_to_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x','0','--y','0','--z','0.05',
                   '--roll','0','--pitch','0','--yaw','0',
                   '--frame-id','romi/sensor_mount',
                   '--child-frame-id','romi/sensor_mount/lidar'],
        parameters=[sim], output='screen')
 
    scan_remapper = Node(
        package='romi_gazebo', executable='scan_frame_remapper.py',
        parameters=[sim, {
            'input_topic':     '/lidar/scan',
            'output_topic':    '/lidar/scan_fixed',
            'target_frame_id': 'romi/sensor_mount/lidar',
        }], output='screen')
 
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', parameters=[slam_params], output='screen')
 
    recorder = Node(
        package='romi_gazebo', executable='data_recorder.py',
        parameters=[sim, {'auto_start': True}], output='screen')
 
    explorer = Node(
        package='romi_gazebo', executable='autonomous_explorer.py',
        parameters=[sim, {
            'linear_speed':       0.22,
            'angular_speed':      0.55,
            'turn_duration':      2.8,
            'obstacle_threshold': 1.2,
            'coverage_stop_cells': LaunchConfiguration('coverage_stop_cells'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore')))
 
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], parameters=[sim], output='screen')
 
    return LaunchDescription([
        explore_arg, timeout_arg,
        gz_sim, bridge,
        jsp, rsp,
        odom_tf,
        tf_base_to_mount, tf_mount_to_rs, tf_mount_to_lidar,
        scan_remapper, slam,
        recorder, explorer, rviz,
    ])
