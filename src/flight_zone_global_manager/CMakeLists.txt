cmake_minimum_required(VERSION 3.8)
project(flight_zone_global_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flight_zone_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(
  flight_zone_server_node
  src/flight_zone_server_node.cpp
)

target_compile_features(flight_zone_server_node PUBLIC cxx_std_17)

ament_target_dependencies(
  flight_zone_server_node
  ament_index_cpp
  builtin_interfaces
  rclcpp
  flight_zone_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

target_link_libraries(
  flight_zone_server_node
  yaml-cpp
)

add_executable(
  flight_zone_monitor_node
  src/flight_zone_monitor_node.cpp
)

target_compile_features(flight_zone_monitor_node PUBLIC cxx_std_17)

ament_target_dependencies(
  flight_zone_monitor_node
  rclcpp
  flight_zone_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  visualization_msgs
)

install(
  TARGETS
    flight_zone_server_node
    flight_zone_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
