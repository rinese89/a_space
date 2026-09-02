cmake_minimum_required(VERSION 3.8)
project(updated_flow_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(static_trajectory_manager REQUIRED)
find_package(static_trajectory_conflict_manager REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlowTrajectoryStatus.msg"
  "msg/UpdatedFlowStatus.msg"
  DEPENDENCIES std_msgs
)

add_executable(updated_flow_server_node src/updated_flow_server_node.cpp)

ament_target_dependencies(updated_flow_server_node
  rclcpp
  std_msgs
  static_trajectory_manager
  static_trajectory_conflict_manager
)

target_compile_features(updated_flow_server_node PUBLIC cxx_std_17)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(updated_flow_server_node "${cpp_typesupport_target}")

install(TARGETS updated_flow_server_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
