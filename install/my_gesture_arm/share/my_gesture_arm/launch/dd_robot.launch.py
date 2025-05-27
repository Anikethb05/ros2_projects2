import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
import xacro

def generate_launch_description():
    # Robot name and package
    robot_name = 'my_gesture_arm'
    package_name = 'my_gesture_arm'

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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
    robot_description_config = Command(['xacro ', model_file_path])
    robot_description = {'robot_description': robot_description_config}

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
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_config}],
        output='screen'
    )

    # ROS2 Control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory(package_name), 'config', 'controllers.yaml')
        ],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', robot_name,
            '-z', '0.0'
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
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Teleop controller
    teleop_node = Node(
        package='my_gesture_arm',
        executable='teleop_controller.py',
        name='teleop_controller',
        output='screen'
    )

    # Load controllers with delay
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(use_sim_time)
    )
    load_shoulder_yaw_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'shoulder_yaw_joint_controller'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(use_sim_time)
    )
    load_shoulder_pitch_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'shoulder_pitch_joint_controller'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(use_sim_time)
    )
    load_elbow_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'elbow_joint_controller'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(use_sim_time)
    )
    load_wrist_roll_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wrist_roll_joint_controller'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(use_sim_time)
    )
    load_wrist_pitch_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wrist_pitch_joint_controller'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(use_sim_time)
    )
    load_left_finger_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'left_finger_joint_controller'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(use_sim_time)
    )
    load_right_finger_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'right_finger_joint_controller'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        condition=IfCondition(use_sim_time)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        gazebo_launch,
        robot_state_publisher_node,
        control_node,
        spawn_entity_node,
        rviz_node,
        teleop_node,
        load_joint_state_broadcaster,
        load_shoulder_yaw_controller,
        load_shoulder_pitch_controller,
        load_elbow_controller,
        load_wrist_roll_controller,
        load_wrist_pitch_controller,
        load_left_finger_controller,
        load_right_finger_controller
    ])