cmake_minimum_required(VERSION 3.8)
project(distance_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(flight_zone_msgs REQUIRED)
find_package(static_trajectory_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UasPairDistance.msg"
  "msg/UasRelatedDistanceArray.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

add_executable(distance_control_node
  src/distance_control_node.cpp
)

target_compile_features(distance_control_node PUBLIC cxx_std_17)

ament_target_dependencies(distance_control_node
  rclcpp
  flight_zone_msgs
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
  distance_control_node
  "${cpp_typesupport_target}"
)

install(
  TARGETS distance_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
