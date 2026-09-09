cmake_minimum_required(VERSION 3.8)
project(flight_zone_supervision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(collision_detection REQUIRED)
find_package(distance_control REQUIRED)
find_package(static_trajectory_manager REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SupervisionControl.action"
  DEPENDENCIES
    static_trajectory_manager
)

add_executable(supervision_node
  src/supervision_node.cpp
)

target_compile_features(
  supervision_node
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  supervision_node
  rclcpp
  rclcpp_action
  collision_detection
  distance_control
  static_trajectory_manager
  geometry_msgs
  std_msgs
  visualization_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  supervision_node
  "${cpp_typesupport_target}"
)

install(
  TARGETS
    supervision_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    action
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
