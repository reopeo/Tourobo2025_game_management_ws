cmake_minimum_required(VERSION 3.5)
project(game_state_interfaces)
find_package(rosidl_default_generators REQUIRED)

file(GLOB ROS2_MSG RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
file(GLOB ROS2_SRV RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${ROS2_MSG}
  ${ROS2_SRV}
  DEPENDENCIES
    builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()