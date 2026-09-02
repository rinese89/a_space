cmake_minimum_required(VERSION 3.8)
project(static_trajectory_conflict_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(static_trajectory_manager REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SegmentCollision.msg"
  "msg/TrajectoryCollision.msg"
  "msg/CollisionStaticTrajectory.msg"
  "msg/CollisionStaticTrajectoryArray.msg"
  DEPENDENCIES
    geometry_msgs
    std_msgs
    static_trajectory_manager
)

add_executable(
  static_trajectory_conflict_manager_node
  src/static_trajectory_conflict_manager_node.cpp
)

ament_target_dependencies(
  static_trajectory_conflict_manager_node
  rclcpp
  builtin_interfaces
  geometry_msgs
  std_msgs
  visualization_msgs
  static_trajectory_manager
)

target_compile_features(
  static_trajectory_conflict_manager_node
  PUBLIC cxx_std_17
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  static_trajectory_conflict_manager_node
  "${cpp_typesupport_target}"
)

install(
  TARGETS static_trajectory_conflict_manager_node
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
