cmake_minimum_required(VERSION 3.8)
project(trajectory_server_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(collision_detection REQUIRED)
find_package(static_trajectory_manager REQUIRED)
find_package(static_trajectory_conflict_manager REQUIRED)
find_package(trajectory_endtime_adjustment REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(trajectory_server_node src/trajectory_server_node.cpp)

ament_target_dependencies(trajectory_server_node
  rclcpp
  px4_msgs
  collision_detection
  static_trajectory_manager
  static_trajectory_conflict_manager
  trajectory_endtime_adjustment
  geometry_msgs
  std_msgs
  visualization_msgs
)

target_compile_features(trajectory_server_node PUBLIC cxx_std_17)

install(TARGETS trajectory_server_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
