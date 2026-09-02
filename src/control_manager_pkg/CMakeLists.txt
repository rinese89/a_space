cmake_minimum_required(VERSION 3.8)
project(control_manager_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ControlTrajectory.srv"
)

add_executable(control_manager_node
  src/control_manager_node.cpp
)

ament_target_dependencies(control_manager_node
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  controllers_pkg
  static_trajectory_manager
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(control_manager_node
  "${cpp_typesupport_target}"
)

install(TARGETS control_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
