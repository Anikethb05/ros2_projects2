import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Robot name and package
    robot_name = 'my_gesture_arm'
    package_name = 'my_gesture_arm'

    # Path to model and world files
    model_file_path = os.path.join(
        get_package_share_directory(package_name),
        'model',
        'my_gesture_arm.xacro'  
    )
    
    world_file_path = os.path.join(
        get_package_share_directory(package_name),
        'model',
        'empty_world.world'
    )

    # Process XACRO file
    robot_description_config = xacro.process_file(model_file_path)
    robot_description = robot_description_config.toxml()

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_file_path}.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Joint State Publisher GUI
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity_node
    ])