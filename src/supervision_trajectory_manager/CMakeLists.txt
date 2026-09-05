cmake_minimum_required(VERSION 3.8)
project(supervision_trajectory_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(a_space_virtual_net REQUIRED)
find_package(collision_detection REQUIRED)
find_package(deconfliction_manager REQUIRED)
find_package(static_trajectory_manager REQUIRED)

add_executable(
  supervision_trajectory_manager_node
  src/supervision_trajectory_manager_node.cpp
)

ament_target_dependencies(
  supervision_trajectory_manager_node
  rclcpp
  builtin_interfaces
  geometry_msgs
  std_msgs
  visualization_msgs
  a_space_virtual_net
  collision_detection
  deconfliction_manager
  static_trajectory_manager
)

target_compile_features(
  supervision_trajectory_manager_node
  PUBLIC cxx_std_17
)

install(
  TARGETS supervision_trajectory_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
