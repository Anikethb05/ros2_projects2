#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('new_diffbot_slam')
    world_file = os.path.join(pkg_share, 'worlds', 'world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'diffbot.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'diffbot', '-file', Command(['xacro ', urdf_file]), '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'robot_description': Command(['xacro ', urdf_file])}]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Obstacle Avoidance
        Node(
            package='new_diffbot_slam',
            executable='obstacle_avoidance',
            name='obstacle_avoidance_node',
            namespace='diffbot',
            output='screen'
        ),

        # Kimera-VIO-ROS2
        Node(
            package='kimera_vio_ros',
            executable='kimera_vio_ros_node',
            name='kimera_vio_node',
            namespace='diffbot',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'left_cam_topic': '/diffbot/camera/image_raw',
                'right_cam_topic': '/diffbot/camera/image_raw',  # Mono camera
                'imu_topic': '/diffbot/imu',
                'params_folder': os.path.join(get_package_share_directory('new_diffbot_slam'), 'params', 'Euroc'),
                'frontend_type': 'MonoVisionImuFrontend'
            }],
            output='screen'
        ),

        # SLAM Toolbox (Fallback)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='diffbot',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan'
            }],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])