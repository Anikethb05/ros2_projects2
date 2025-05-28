cmake_minimum_required(VERSION 3.8)
project(my_gesture_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclpy REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(gazebo_ros REQUIRED) 
find_package(robot_state_publisher REQUIRED) 
find_package(rviz2 REQUIRED) 
find_package(controller_manager REQUIRED) 
find_package(ros2_control REQUIRED) 
find_package(position_controllers REQUIRED) 
find_package(joint_state_broadcaster REQUIRED)

install(
  DIRECTORY launch model
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/teleop_controller.py
    scripts/autonomous_robot_controller.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()