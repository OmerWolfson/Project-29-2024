cmake_minimum_required(VERSION 3.8)
project(g4_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(velocity_controllers REQUIRED) 
find_package(robot_localization REQUIRED)

add_executable(my_diff_drive_controller src/my_diff_drive_controller.cpp)
ament_target_dependencies(my_diff_drive_controller rclcpp std_msgs geometry_msgs) 

add_executable(isaac_diff_drive_controller src/isaac_diff_drive_controller.cpp)
ament_target_dependencies(isaac_diff_drive_controller rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  my_diff_drive_controller isaac_diff_drive_controller
  DESTINATION lib/${PROJECT_NAME} #/
)

# Install directories
install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
