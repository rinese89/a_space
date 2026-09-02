cmake_minimum_required(VERSION 3.8)
project(trajectory_endtime_adjustment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(static_trajectory_manager REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RegisterOriginalTrajectory.srv"
  DEPENDENCIES static_trajectory_manager
)

add_executable(
  trajectory_endtime_adjustment_node
  src/trajectory_endtime_adjustment_node.cpp
)

ament_target_dependencies(
  trajectory_endtime_adjustment_node
  rclcpp
  builtin_interfaces
  static_trajectory_manager
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(
  trajectory_endtime_adjustment_node
  "${cpp_typesupport_target}"
)

target_compile_features(
  trajectory_endtime_adjustment_node
  PUBLIC cxx_std_17
)

install(
  TARGETS trajectory_endtime_adjustment_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
