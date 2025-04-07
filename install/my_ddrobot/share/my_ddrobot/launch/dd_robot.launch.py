import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robotXacroName='my_dd_robot'

    namePkg = 'my_ddrobot'

    modelFileRelPath = 'model/my_dd_robot_model.xacro'
    worldFileRelPath = 'model/empty_world.world'

    pathModelFile = os.path.join(get_package_share_directory(namePkg),modelFileRelPath)
    pathWorldFile = os.path.join(get_package_share_directory(namePkg),worldFileRelPath)

    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_ros_pkg_launch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))

    gazeboLaunch=IncludeLaunchDescription(gazebo_ros_pkg_launch,launch_arguments={'world': pathWorldFile}.items())

    spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py', 
                          arguments=['-topic','robot_description','-entity',robotXacroName], output='screen')

    nodeRobotStatePub = Node(package='robot_state_publisher', executable='robot_state_publisher',
                             output='screen', parameters=[{'robot_description': robotDescription,
                                                           'use_sim_time':True}])

    launchDescriptionObj=LaunchDescription()
    launchDescriptionObj.add_action(gazeboLaunch)     
    launchDescriptionObj.add_action(spawnModelNode)   
    launchDescriptionObj.add_action(nodeRobotStatePub)

    return launchDescriptionObj