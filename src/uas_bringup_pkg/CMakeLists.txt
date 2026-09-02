cmake_minimum_required(VERSION 3.8)
project(uas_bringup_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(drone_odom_broadcaster src/drone_odom_broadcaster.cpp)
ament_target_dependencies(drone_odom_broadcaster rclcpp px4_msgs tf2_msgs tf2_ros geometry_msgs tf2 nav_msgs std_srvs Eigen3)

install(TARGETS
  drone_odom_broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
