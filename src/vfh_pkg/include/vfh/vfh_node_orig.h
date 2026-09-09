/*
 * Copyright (c) 2012, Stefano Rosa, Luca Carlone
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VFH_NODE_H_
#define VFH_NODE_H_

#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <array>
#include <algorithm>
#include <vector>
#include <thread>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include "vfh/vfh_algorithm.h"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

#define DEG2RAD(a) ((a) * M_PI / 180.0)

class VFH_node : public rclcpp::Node
{
public:
    VFH_node();
    ~VFH_node();

    void update();
    void stop_to_cmd_vel();
    void stop_to_px4();
    void request_control();

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void goalPose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg);

    void doTransform();

    void publishCommand(float picked_angle, float chosen_speed);
    void publishTrajectorySetpointHolonomic(float picked_angle, float chosen_speed);

    void publishVFHVisualization();
    void addReferenceCircle(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id);
    void addReferenceSectors(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id);
    void addPrimaryHistogram(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id);
    void addBinaryHistogram(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id);
    void addCandidateDirections(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id);
    void addPickedDirection(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id);
    void publishGoalPosition();

    double m_cell_size;
    int m_window_diameter;
    double m_robot_radius;
    int m_sectors_number;
    int hist_size;

    double sector_angle;

    bool use_amcl;
    bool pub_cmd_vel;
    bool wandering_mode;

    bool pub_px4_setpoint_;
    double drone_cruise_speed_;
    double drone_max_speed_;
    double drone_vz_;

    std::string px4_namespace_;
    std::string traj_sp_topic_;
    std::string offboard_mode_topic_;
    std::string control_service_topic_;

    double laser_resolution = 0.0;
    double current_yaw_ = 0.0;
    bool odom_received_ = false;

    float robot_linear_vel = 0.0f;
    float robot_angular_vel = 0.0f;

    double safety_radio_mark = 0.0;
    float l = 0.08f;

    std::vector<double> m_laser_ranges;
    std::unique_ptr<VFH_Algorithm> m_vfh;

    geometry_msgs::msg::Point goal_position;
    geometry_msgs::msg::Point p_out;

    double desired_dist = 0.0;
    double desired_angle = 0.0;
    bool goal_flag = false;

    int chosen_speed = 0;
    int chosen_turnrate = 0;

    size_t last_marker_count_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalPose_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_sp_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_line_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vfh_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grid_marker_pub_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr control_client_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

#endif /* VFH_NODE_H_ */