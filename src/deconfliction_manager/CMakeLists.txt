cmake_minimum_required(VERSION 3.8)
project(deconfliction_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(collision_detection REQUIRED)
find_package(static_trajectory_manager REQUIRED)
find_package(a_space_virtual_net REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CroppedNetNode.msg"
  "msg/CroppedNetSegment.msg"
  "msg/CroppedNetTrajectory.msg"
  "msg/RequestedSupervisionTrajectory.msg"
  "msg/RequestedSupervisionTrajectoryArray.msg"
  DEPENDENCIES
    geometry_msgs
    std_msgs
    collision_detection
    a_space_virtual_net
    static_trajectory_manager
)

add_executable(
  deconfliction_manager_node
  src/deconfliction_manager_node.cpp
)

ament_target_dependencies(
  deconfliction_manager_node
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  collision_detection
  static_trajectory_manager
  a_space_virtual_net
)

target_compile_features(
  deconfliction_manager_node
  PUBLIC cxx_std_17
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  deconfliction_manager_node
  "${cpp_typesupport_target}"
)

install(
  TARGETS deconfliction_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
