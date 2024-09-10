cmake_minimum_required(VERSION 3.8)
project(g3_cartographer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cartographer_ros REQUIRED)

install(DIRECTORY config include launch rviz src urdf 
  DESTINATION share/${PROJECT_NAME}/
)

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_dependencies(${dependencies})
ament_package()