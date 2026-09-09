#include "vfh/vfh_node.h"

#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <csignal>
#include <cstdlib>

using namespace std::chrono_literals;

static inline float wrap_pi(float a)
{
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a <= -M_PI) a += 2.0f * M_PI;
    return a;
}

VFH_node::VFH_node()
: Node("vfh_node")
{
    m_robot_radius    = this->declare_parameter("m_robot_radius", 0.5);
    m_cell_size       = this->declare_parameter("m_cell_size", 0.05);
    m_window_diameter = this->declare_parameter("m_window_diameter", 60);
    m_sectors_number  = this->declare_parameter("sectors_number", 72);

    use_amcl       = this->declare_parameter("use_amcl", false);
    wandering_mode = this->declare_parameter("wandering_mode", false);
    pub_cmd_vel    = this->declare_parameter("pub_cmd_vel", false);

    pub_px4_setpoint_   = this->declare_parameter("pub_px4_setpoint", true);
    drone_cruise_speed_ = this->declare_parameter("drone_cruise_speed", 1.0);
    drone_max_speed_    = this->declare_parameter("drone_max_speed", 3.0);
    drone_vz_           = this->declare_parameter("drone_vz", 0.0);

    px4_namespace_ = this->declare_parameter<std::string>("px4_namespace", "");

    if (px4_namespace_.empty()) {
        traj_sp_topic_         = "/fmu/in/trajectory_setpoint";
        offboard_mode_topic_   = "/fmu/in/offboard_control_mode";
        control_service_topic_ = "/enable_control";
    } else {
        if (px4_namespace_.front() == '/') {
            traj_sp_topic_         = px4_namespace_ + "/fmu/in/trajectory_setpoint";
            offboard_mode_topic_   = px4_namespace_ + "/fmu/in/offboard_control_mode";
            control_service_topic_ = px4_namespace_ + "/enable_control";
        } else {
            traj_sp_topic_         = "/" + px4_namespace_ + "/fmu/in/trajectory_setpoint";
            offboard_mode_topic_   = "/" + px4_namespace_ + "/fmu/in/offboard_control_mode";
            control_service_topic_ = "/" + px4_namespace_ + "/enable_control";
        }
    }

    m_vfh = std::make_unique<VFH_Algorithm>(
        m_robot_radius,
        m_cell_size,
        m_window_diameter,
        m_sectors_number
    );

    sector_angle = 2.0 * M_PI / static_cast<double>(m_sectors_number);

    RCLCPP_INFO(this->get_logger(), "Starting VFH");
    RCLCPP_INFO(this->get_logger(), "pub_cmd_vel=%s | pub_px4_setpoint=%s",
                pub_cmd_vel ? "true" : "false",
                pub_px4_setpoint_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Control service: %s", control_service_topic_.c_str());

    if (pub_px4_setpoint_) {
        RCLCPP_INFO(this->get_logger(),
                    "PX4 topics: %s | %s",
                    offboard_mode_topic_.c_str(),
                    traj_sp_topic_.c_str());
    }

    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", scan_qos, std::bind(&VFH_node::scanCallback, this, std::placeholders::_1));

    if (use_amcl) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/amcl_odom", qos_profile, std::bind(&VFH_node::odomCallback, this, std::placeholders::_1));
    } else {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_profile, std::bind(&VFH_node::odomCallback, this, std::placeholders::_1));
    }

    goalPose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", qos_profile, std::bind(&VFH_node::goalPose_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    goal_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_line_marker", 10);
    vfh_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vfh_visualization", 10);

    traj_sp_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(traj_sp_topic_, 10);
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_mode_topic_, 10);

    control_client_ = this->create_client<std_srvs::srv::Trigger>(control_service_topic_);

    request_control();
}

VFH_node::~VFH_node() {}

void VFH_node::request_control()
{
    if (!control_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(),
                    "Servicio de control no disponible: %s",
                    control_service_topic_.c_str());
        return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

    control_client_->async_send_request(
        req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            auto res = future.get();
            if (res->success) {
                RCLCPP_INFO(this->get_logger(), "Control concedido");
            } else {
                RCLCPP_WARN(this->get_logger(), "Control rechazado: %s", res->message.c_str());
            }
        });
}

void VFH_node::goalPose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg)
{
    if (!wandering_mode) {
        RCLCPP_INFO(this->get_logger(), "Got goal pose.");
        goal_position = goal_pose_msg->pose.position;
        doTransform();
        RCLCPP_INFO(this->get_logger(), "Desired Distance: %.3f; Desired Angle: %.3f",
                    desired_dist, desired_angle);
        goal_flag = true;
    }
}

void VFH_node::doTransform()
{
    //std::string target_frame = use_amcl ? "map" : "odom";
    std::string target_frame = "map";
    std::string source_frame = "base_link";
    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = tf_buffer->lookupTransform(source_frame, target_frame, rclcpp::Time(0));
        tf2::doTransform(goal_position, p_out, transform);

        desired_dist = std::sqrt(std::pow(p_out.x, 2) + std::pow(p_out.y, 2));
        desired_angle = std::atan2(p_out.y, p_out.x);

        if (desired_angle < 0) {
            desired_angle += 2 * M_PI;
        }
        if (desired_angle > 2 * M_PI) {
            desired_angle -= 2 * M_PI;
        }
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "We could not obtain the transform: %s", ex.what());
    }
}

void VFH_node::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;
    odom_received_ = true;

    if (!wandering_mode) {
        double xdiff = goal_position.x - odom_msg->pose.pose.position.x;
        double ydiff = goal_position.y - odom_msg->pose.pose.position.y;

        double theta = yaw;
        p_out.x = std::cos(theta) * xdiff + std::sin(theta) * ydiff;
        p_out.y = -std::sin(theta) * xdiff + std::cos(theta) * ydiff;

        desired_dist = std::sqrt(std::pow(p_out.x, 2) + std::pow(p_out.y, 2));
        desired_angle = std::atan2(p_out.y, p_out.x);

        if (desired_angle < 0) {
            desired_angle += 2 * M_PI;
        }
        if (desired_angle > 2 * M_PI) {
            desired_angle -= 2 * M_PI;
        }
    }

    robot_linear_vel = static_cast<float>(odom_msg->twist.twist.linear.x);
    robot_angular_vel = static_cast<float>(odom_msg->twist.twist.angular.z);
}

void VFH_node::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    int n = static_cast<int>(scan_msg->ranges.size());

    m_laser_ranges.resize(n);
    laser_resolution = scan_msg->angle_increment;

    m_laser_ranges.assign(n, std::numeric_limits<double>::infinity());

    for (int i = 0; i < n; ++i) {
        double r = scan_msg->ranges[i];

        if (!std::isfinite(r) || r < scan_msg->range_min || r > scan_msg->range_max) {
            continue;
        }
        m_laser_ranges[i] = r;
    }

    update();
}

void VFH_node::update()
{
    if (wandering_mode) {
        desired_angle = 0.0;
        desired_dist = 1.0;
    }

    m_vfh->Update_VFH(m_laser_ranges, laser_resolution, robot_linear_vel, desired_angle, desired_dist);

    publishVFHVisualization();

    if (m_vfh->selectDirection()) {
        if (pub_cmd_vel) {
            publishCommand(m_vfh->Picked_Angle, m_vfh->Chosen_Speed);
        }

        if (pub_px4_setpoint_) {
            publishTrajectorySetpointHolonomic(m_vfh->Picked_Angle, m_vfh->Chosen_Speed);
        }
    }

    publishGoalPosition();
}

void VFH_node::stop_to_cmd_vel()
{
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;

    cmd_vel_pub_->publish(stop_msg);

    RCLCPP_INFO(this->get_logger(), "Stop command sent to cmd_vel");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void VFH_node::stop_to_px4()
{
    if (!pub_px4_setpoint_) {
        return;
    }

    const uint64_t ts = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);
    const float qnan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = ts;
    mode.position = false;
    mode.velocity = true;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    mode.thrust_and_torque = false;
    mode.direct_actuator = false;

    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = ts;
    sp.position = {qnan, qnan, qnan};
    sp.velocity = {0.0f, 0.0f, 0.0f};
    sp.acceleration = {qnan, qnan, qnan};
    sp.jerk = {qnan, qnan, qnan};
    sp.yaw = qnan;
    sp.yawspeed = 0.0f;

    offboard_mode_pub_->publish(mode);
    traj_sp_pub_->publish(sp);

    RCLCPP_INFO(this->get_logger(), "Stop command sent to PX4 trajectory_setpoint");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void VFH_node::publishCommand(float picked_angle, float chosen_speed)
{
    geometry_msgs::msg::Twist cmd;

    const float e = wrap_pi(picked_angle);

    const float deadband      = 0.03f;
    const float align_thresh  = 0.35f;
    const float k_omega_go    = 3.0f;
    const float k_omega_align = 5.0f;
    const float max_omega     = 2.0f;
    const float min_v         = 0.05f;
    const float max_v         = chosen_speed;
    const float min_dist      = 0.05f;

    float e_db = (std::fabs(e) < deadband) ? 0.0f : e;

    if (std::fabs(e_db) > align_thresh) {
        cmd.linear.x = 0.0;
        float omega = k_omega_align * e_db;
        omega = std::clamp(omega, -max_omega, max_omega);
        cmd.angular.z = omega;
    }

    if ((std::fabs(e_db) > align_thresh) && (desired_dist < min_dist)) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }
    else {
        float v = max_v * std::cos(e_db);
        v = std::clamp(v, 0.0f, max_v);
        if (v > 0.0f) {
            v = std::max(v, min_v);
        }

        float omega = k_omega_go * e_db;
        omega = std::clamp(omega, -max_omega, max_omega);

        cmd.linear.x = v;
        cmd.angular.z = omega;
    }

    if (pub_cmd_vel) {
        cmd_vel_pub_->publish(cmd);
    }
}

void VFH_node::publishTrajectorySetpointHolonomic(float picked_angle, float chosen_speed)
{
    if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No odometry received yet; PX4 setpoint not published.");
        return;
    }

    const float e = wrap_pi(picked_angle);
    const float deadband = 0.03f;
    const float min_dist = 0.05f;

    float e_db = (std::fabs(e) < deadband) ? 0.0f : e;

    float v_cmd = std::clamp(
        static_cast<float>(chosen_speed),
        0.0f,
        static_cast<float>(drone_max_speed_));

    if (drone_cruise_speed_ > 0.0) {
        v_cmd = std::min(v_cmd, static_cast<float>(drone_cruise_speed_));
    }

    float vx_body = 0.0f;
    float vy_body = 0.0f;

    if (desired_dist < min_dist) {
        vx_body = 0.0f;
        vy_body = 0.0f;
    } else {
        vx_body = v_cmd * std::cos(e_db);
        vy_body = v_cmd * std::sin(e_db);
    }

    const float cy = std::cos(static_cast<float>(current_yaw_));
    const float sy = std::sin(static_cast<float>(current_yaw_));

    const float vx_enu = cy * vx_body - sy * vy_body;
    const float vy_enu = sy * vx_body + cy * vy_body;
    const float vz_enu = static_cast<float>(drone_vz_);

    const float vx_ned = vy_enu;
    const float vy_ned = vx_enu;
    const float vz_ned = -vz_enu;

    const uint64_t ts = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);
    const float qnan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = ts;
    mode.position = false;
    mode.velocity = true;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    mode.thrust_and_torque = false;
    mode.direct_actuator = false;

    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = ts;
    sp.position = {qnan, qnan, qnan};
    sp.velocity = {vx_ned, vy_ned, vz_ned};
    sp.acceleration = {qnan, qnan, qnan};
    sp.jerk = {qnan, qnan, qnan};
    sp.yaw = qnan;
    sp.yawspeed = 0.0f;

    offboard_mode_pub_->publish(mode);
    traj_sp_pub_->publish(sp);
}

void VFH_node::publishVFHVisualization()
{
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    auto stamp = rclcpp::Time(0);

    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    addReferenceCircle(array, stamp, id);
    addReferenceSectors(array, stamp, id);
    addPrimaryHistogram(array, stamp, id);
    addBinaryHistogram(array, stamp, id);

    if (goal_flag) {
        addCandidateDirections(array, stamp, id);
        addPickedDirection(array, stamp, id);
    }

    vfh_markers_pub_->publish(array);
}

void VFH_node::addReferenceCircle(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id)
{
    visualization_msgs::msg::Marker circle;

    circle.header.frame_id = "base_link";
    circle.header.stamp = stamp;

    circle.ns = "vfh_reference_circle";
    circle.id = id++;

    circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
    circle.action = visualization_msgs::msg::Marker::ADD;

    circle.pose.orientation.w = 1.0;
    circle.scale.x = 0.02;

    circle.color.r = 0.8f;
    circle.color.g = 0.8f;
    circle.color.b = 0.8f;
    circle.color.a = 1.0f;

    const double R_max = (m_window_diameter / 2.0) * m_cell_size;
    const int N = m_sectors_number;

    for (int i = 0; i <= N; ++i) {
        double theta = i * sector_angle;

        geometry_msgs::msg::Point p;
        p.x = R_max * std::cos(theta);
        p.y = R_max * std::sin(theta);
        p.z = 0.0;

        circle.points.push_back(p);
    }

    array.markers.push_back(circle);
}

void VFH_node::addReferenceSectors(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id)
{
    visualization_msgs::msg::Marker sectors;

    sectors.header.frame_id = "base_link";
    sectors.header.stamp = stamp;

    sectors.ns = "vfh_reference_sectors";
    sectors.id = id++;

    sectors.type = visualization_msgs::msg::Marker::LINE_LIST;
    sectors.action = visualization_msgs::msg::Marker::ADD;

    sectors.pose.orientation.w = 1.0;
    sectors.scale.x = 0.01;

    sectors.color.r = 0.0f;
    sectors.color.g = 1.0f;
    sectors.color.b = 0.0f;
    sectors.color.a = 0.6f;

    const double R_max = (m_window_diameter / 2.0) * m_cell_size;

    for (int i = 0; i < m_sectors_number; ++i) {
        double theta = i * sector_angle;

        geometry_msgs::msg::Point p0, p1;
        p0.x = 0.0;
        p0.y = 0.0;
        p0.z = 0.0;

        p1.x = R_max * std::cos(theta);
        p1.y = R_max * std::sin(theta);
        p1.z = 0.0;

        sectors.points.push_back(p0);
        sectors.points.push_back(p1);
    }

    array.markers.push_back(sectors);
}

void VFH_node::addPrimaryHistogram(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "base_link";
    marker.header.stamp = stamp;

    marker.ns = "vfh_primary_histogram";
    marker.id = id++;

    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    hist_size = m_sectors_number;

    const double R_max = (m_window_diameter / 2.0) * m_cell_size;

    double max_val = 0.0;
    for (int i = 0; i < hist_size; ++i) {
        max_val = std::max(max_val, static_cast<double>(m_vfh->Hist_primary[i]));
    }

    if (max_val < 1e-6) {
        max_val = 1.0;
    }

    geometry_msgs::msg::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = 0.0;

    for (int i = 0; i < hist_size; ++i) {
        double r = (m_vfh->Hist_primary[i] / max_val) * R_max;
        double theta_center = (i + 0.5) * sector_angle;
        double half_angle = sector_angle / 2.0;

        geometry_msgs::msg::Point p_left, p_right;

        p_left.x = r * std::cos(theta_center - half_angle);
        p_left.y = r * std::sin(theta_center - half_angle);
        p_left.z = 0.0;

        p_right.x = r * std::cos(theta_center + half_angle);
        p_right.y = r * std::sin(theta_center + half_angle);
        p_right.z = 0.0;

        marker.points.push_back(center);
        marker.points.push_back(p_left);
        marker.points.push_back(p_right);
    }

    array.markers.push_back(marker);
}

void VFH_node::addBinaryHistogram(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "base_link";
    marker.header.stamp = stamp;

    marker.ns = "vfh_binary_histogram";
    marker.id = id++;

    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.15;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    hist_size = m_sectors_number;

    const double EPS = 0.02 * sector_angle;
    const double R_max = (m_window_diameter / 2.0) * m_cell_size;
    const int ARC_STEPS = 6;

    for (int i = 0; i < hist_size; ++i) {
        std_msgs::msg::ColorRGBA color;
        if (m_vfh->Hist_binary[i] > 0.5) {
            color.r = 1.0f;
            color.g = 0.0f;
            color.b = 0.0f;
            color.a = 1.0f;
        } else {
            color.r = 0.0f;
            color.g = 1.0f;
            color.b = 0.0f;
            color.a = 1.0f;
        }

        const double theta_start = i * sector_angle + EPS;
        const double theta_end = (i + 1) * sector_angle - EPS;

        for (int k = 0; k < ARC_STEPS; ++k) {
            const double t0 = static_cast<double>(k) / ARC_STEPS;
            const double t1 = static_cast<double>(k + 1) / ARC_STEPS;

            const double a0 = theta_start + t0 * (theta_end - theta_start);
            const double a1 = theta_start + t1 * (theta_end - theta_start);

            geometry_msgs::msg::Point p0, p1;

            p0.x = R_max * std::cos(a0);
            p0.y = R_max * std::sin(a0);
            p0.z = 0.05;

            p1.x = R_max * std::cos(a1);
            p1.y = R_max * std::sin(a1);
            p1.z = 0.05;

            marker.points.push_back(p0);
            marker.points.push_back(p1);

            marker.colors.push_back(color);
            marker.colors.push_back(color);
        }
    }

    array.markers.push_back(marker);
}

void VFH_node::addCandidateDirections(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id)
{
    for (size_t i = 0; i < m_vfh->Candidate_Angle.size(); ++i) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "base_link";
        marker.header.stamp = stamp;
        marker.ns = "vfh_candidates";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, m_vfh->Candidate_Angle[i]);
        marker.pose.orientation = tf2::toMsg(q);

        marker.scale.x = 1.0;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        marker.lifetime = rclcpp::Duration(0, 0);

        array.markers.push_back(marker);
    }
}

void VFH_node::addPickedDirection(visualization_msgs::msg::MarkerArray& array, const rclcpp::Time& stamp, int& id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "base_link";
    marker.header.stamp = stamp;

    marker.ns = "vfh_picked";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, m_vfh->Picked_Angle);
    marker.pose.orientation = tf2::toMsg(q);

    marker.scale.x = 1.2;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    array.markers.push_back(marker);
}

void VFH_node::publishGoalPosition()
{
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "base_link";
    line_marker.header.stamp = this->get_clock()->now();
    line_marker.ns = "goal_line";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.02;

    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;

    geometry_msgs::msg::Point end = p_out;
    end.z = 0.0;

    line_marker.points.push_back(start);
    line_marker.points.push_back(end);

    goal_line_pub_->publish(line_marker);
}

std::shared_ptr<VFH_node> node = nullptr;

void signal_handler(int signum)
{
    if (rclcpp::ok() && node) {
        node->stop_to_cmd_vel();
        node->stop_to_px4();
    }
    rclcpp::shutdown();
    std::exit(signum);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    node = std::make_shared<VFH_node>();

    std::signal(SIGINT, signal_handler);

    rclcpp::spin(node);

    return 0;
}