cmake_minimum_required(VERSION 3.8)
project(mauro_prototype_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(velocity_controllers REQUIRED)   
find_package(project29_interfaces REQUIRED)

include_directories(include)

add_library(mauro_prototype_control
  src/prototype_joy_controller_class.cpp
  # other source files
)

add_executable(teleop_joy_node src/teleop_joy_node.cpp)
ament_target_dependencies(teleop_joy_node rclcpp geometry_msgs sensor_msgs) 

add_executable(prototype_joy_driver src/prototype_joy_driver.cpp)
ament_target_dependencies(prototype_joy_driver rclcpp project29_interfaces sensor_msgs) # std_msgs
target_link_libraries(prototype_joy_driver mauro_prototype_control)

install(TARGETS
  teleop_joy_node # nodes
  prototype_joy_driver
  mauro_prototype_control # classes
  DESTINATION lib/${PROJECT_NAME} #/
)

# Install directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()