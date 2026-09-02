cmake_minimum_required(VERSION 3.8)
project(controllers_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmTakeoff.srv"
  "action/FollowWaypoints.action"
  DEPENDENCIES geometry_msgs
  )

add_executable(control_waypoints_node src/control_waypoints_node.cpp)
ament_target_dependencies(control_waypoints_node rclcpp rclcpp_action px4_msgs geometry_msgs nav_msgs std_msgs visualization_msgs std_srvs tf2_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
  control_waypoints_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
DESTINATION share/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(control_waypoints_node "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

ament_package()
