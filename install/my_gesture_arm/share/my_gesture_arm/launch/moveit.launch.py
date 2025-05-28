#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_gesture_arm'),
                'config',
                'my_gesture_arm.srdf'
            ]),
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        move_group_node
    ])
