import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
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
    
    # Check if world file exists, otherwise use empty world
    world_file_path = os.path.join(
        get_package_share_directory(package_name),
        'model',
        'empty_world.world'
    )
    if not os.path.exists(world_file_path):
        world_file_path = os.path.join(
            get_package_share_directory('gazebo_ros'),
            'worlds',
            'empty.world'
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
            'use_sim_time': True
        }],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-z', '0.1'  # Slight elevation to avoid ground clipping
        ],
        output='screen'
    )
    
    # RViz for visualization
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'robot_config.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Optional: Add keyboard teleop node to control the robot
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',
        name='teleop',
        remappings=[('/cmd_vel', '/gesture_arm/cmd_vel')],
        output='screen'
    )

    # Define the launch entities
    launch_entities = [
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        rviz_node
    ]
    
    # Only add teleop if the package is available
    try:
        get_package_share_directory('teleop_twist_keyboard')
        launch_entities.append(teleop_node)
    except:
        pass  # Skip teleop if not installed

    return LaunchDescription(launch_entities)