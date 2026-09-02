cmake_minimum_required(VERSION 3.8)
project(static_trajectory_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(flight_zone_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectorySegment.msg"
  "msg/StaticTrajectory.msg"
  "msg/StaticTrajectoryArray.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

add_executable(static_trajectory_manager_node
  src/static_trajectory_manager_node.cpp
)

ament_target_dependencies(static_trajectory_manager_node
  rclcpp
  std_msgs
  builtin_interfaces
  geometry_msgs
  visualization_msgs
  flight_zone_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(static_trajectory_manager_node
  yaml-cpp
  "${cpp_typesupport_target}"
)

install(TARGETS static_trajectory_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
