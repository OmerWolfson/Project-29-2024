cmake_minimum_required(VERSION 3.8)
project(sensors_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/prototype_camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(driver_mpu6050 src/driver_mpu6050.cpp)
ament_target_dependencies(driver_mpu6050 rclcpp sensor_msgs) # std_msgs

install(TARGETS
  driver_mpu6050 # nodes
  DESTINATION lib/${PROJECT_NAME} #/
)

ament_package()
