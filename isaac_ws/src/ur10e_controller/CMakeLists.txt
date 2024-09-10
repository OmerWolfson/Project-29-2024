cmake_minimum_required(VERSION 3.8)
project(ur10e_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ur10e_position_controller src/ur10e_position_controller.cpp)
ament_target_dependencies(ur10e_position_controller rclcpp sensor_msgs)

add_executable(spin_arm src/spin_arm.cpp)
ament_target_dependencies(spin_arm rclcpp sensor_msgs)

install(TARGETS
ur10e_position_controller spin_arm
  DESTINATION lib/${PROJECT_NAME} #/
)

# Install directories
install(
  DIRECTORY urdf # config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()