cmake_minimum_required(VERSION 3.8)
project(control_manager_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(controllers_pkg REQUIRED)
find_package(static_trajectory_manager REQUIRED)
find_package(flight_zone_supervision REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ControlTrajectory.srv"
)

add_executable(
  control_manager_node
  src/control_manager_node.cpp
)

ament_target_dependencies(
  control_manager_node
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  controllers_pkg
  static_trajectory_manager
  flight_zone_supervision
)

target_compile_features(
  control_manager_node
  PUBLIC cxx_std_17
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  control_manager_node
  "${cpp_typesupport_target}"
)

install(
  TARGETS control_manager_node
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
