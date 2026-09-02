#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <controllers_pkg/srv/arm_takeoff.hpp>
#include <controllers_pkg/action/follow_waypoints.hpp>

#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>

using namespace std::chrono_literals;

class ControlWaypointsNode : public rclcpp::Node
{
public:
    using FollowWaypoints = controllers_pkg::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ServerGoalHandle<FollowWaypoints>;

    ControlWaypointsNode() : Node("control_waypoints_node")
    {
        // ============================================================
        // IDENTIDAD Y NOMBRES ROS
        // ============================================================
        namespace_path_ = normalise_name(this->get_namespace());
        vehicle_id_ = last_name_segment(namespace_path_);

        if (namespace_path_.empty()) {
            throw std::runtime_error(
                "control_waypoints_node must be launched inside a UAS namespace, "
                "for example '/test/ua_1'");
        }

        if (vehicle_id_.empty()) {
            throw std::runtime_error(
                "Unable to derive the UAS identifier from the node namespace");
        }

        // Las interfaces nativas de PX4 utilizan nombres absolutos bajo un
        // namespace DDS plano, porque PX4_UXRCE_DDS_NS no debe contener '/'.
        //
        // Ejemplo para el UAS ROS /test/ua_1:
        //
        //   PX4: /test_ua_1/fmu/in/vehicle_command
        //   ROS: /test/ua_1/odom
        //        /test/ua_1/arm_takeoff
        //        /test/ua_1/follow_waypoints
        //
        // El launch sobrescribe estos valores para cada aeronave.
        vehicle_command_topic_ = this->declare_parameter<std::string>(
            "vehicle_command_topic",
            "/test_ua_1/fmu/in/vehicle_command");
        offboard_control_mode_topic_ = this->declare_parameter<std::string>(
            "offboard_control_mode_topic",
            "/test_ua_1/fmu/in/offboard_control_mode");
        trajectory_setpoint_topic_ = this->declare_parameter<std::string>(
            "trajectory_setpoint_topic",
            "/test_ua_1/fmu/in/trajectory_setpoint");
        vehicle_status_topic_ = this->declare_parameter<std::string>(
            "vehicle_status_topic",
            "/test_ua_1/fmu/out/vehicle_status");
        odom_topic_ = this->declare_parameter<std::string>(
            "odom_topic", "odom");
        marker_topic_ = this->declare_parameter<std::string>(
            "marker_topic", "mission_markers");
        arm_takeoff_service_ = this->declare_parameter<std::string>(
            "arm_takeoff_service", "arm_takeoff");
        follow_waypoints_action_ = this->declare_parameter<std::string>(
            "follow_waypoints_action", "follow_waypoints");

        // Los frame_id no reciben automáticamente el namespace ROS.
        // marker_frame_suffix='odom' se convierte en:
        //   <flight_zone_id>/<ua_N>/odom
        marker_frame_suffix_ = this->declare_parameter<std::string>(
            "marker_frame_suffix", "odom");
        explicit_marker_frame_ = this->declare_parameter<std::string>(
            "marker_frame", "");

        // Los goals FollowWaypoints se interpretan en mission_frame.
        // Antes del control se transforman al odom local del UAS.
        mission_frame_ = normalise_name(
            this->declare_parameter<std::string>(
                "mission_frame", "map"));

        odom_frame_suffix_ = normalise_name(
            this->declare_parameter<std::string>(
                "odom_frame_suffix", "odom"));

        transform_timeout_s_ = this->declare_parameter<double>(
            "transform_timeout_s", 0.20);

        target_system_ = this->declare_parameter<int>(
            "target_system", 1);

        // ============================================================
        // PID Y LÍMITES
        // ============================================================
        kp_x_ = this->declare_parameter<double>("kp_x", 0.9);
        ki_x_ = this->declare_parameter<double>("ki_x", 0.0);
        kd_x_ = this->declare_parameter<double>("kd_x", 0.15);

        kp_y_ = this->declare_parameter<double>("kp_y", 0.9);
        ki_y_ = this->declare_parameter<double>("ki_y", 0.0);
        kd_y_ = this->declare_parameter<double>("kd_y", 0.15);

        kp_z_ = this->declare_parameter<double>("kp_z", 0.8);
        ki_z_ = this->declare_parameter<double>("ki_z", 0.0);
        kd_z_ = this->declare_parameter<double>("kd_z", 0.10);

        max_vx_ = this->declare_parameter<double>("max_vx", 1.5);
        max_vy_ = this->declare_parameter<double>("max_vy", 1.5);
        max_vz_ = this->declare_parameter<double>("max_vz", 0.8);
        max_speed_xy_ = this->declare_parameter<double>(
            "max_speed_xy", 1.8);

        integral_limit_x_ = this->declare_parameter<double>(
            "integral_limit_x", 2.0);
        integral_limit_y_ = this->declare_parameter<double>(
            "integral_limit_y", 2.0);
        integral_limit_z_ = this->declare_parameter<double>(
            "integral_limit_z", 2.0);

        control_period_ms_ = this->declare_parameter<double>(
            "control_period_ms", 50.0);
        takeoff_tolerance_xy_ = this->declare_parameter<double>(
            "takeoff_tolerance_xy", 0.30);
        takeoff_tolerance_z_ = this->declare_parameter<double>(
            "takeoff_tolerance_z", 0.20);

        landing_tolerance_xy_ = this->declare_parameter<double>(
            "landing_tolerance_xy", 0.30);
        landing_command_retry_s_ = this->declare_parameter<double>(
            "landing_command_retry_s", 1.0);

        validate_parameters();

        odom_frame_ = namespace_path_ + "/" + odom_frame_suffix_;

        marker_frame_ = explicit_marker_frame_.empty() ?
            namespace_path_ + "/" + marker_frame_suffix_ :
            normalise_name(explicit_marker_frame_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
            this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_);

        // ============================================================
        // PUBLICADORES PX4 Y VISUALIZACIÓN
        // ============================================================
        vehicle_command_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
            vehicle_command_topic_, 10);

        ocm_pub_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            offboard_control_mode_topic_, 10);

        setpoint_pub_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            trajectory_setpoint_topic_, 10);

        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic_, 10);

        // ============================================================
        // SUSCRIPCIONES
        // ============================================================
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(
                &ControlWaypointsNode::odom_callback,
                this,
                std::placeholders::_1));

        rclcpp::QoS sensor_qos(
            rclcpp::QoSInitialization::from_rmw(
                rmw_qos_profile_sensor_data));
        sensor_qos.best_effort();

        vehicle_status_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleStatus>(
            vehicle_status_topic_,
            sensor_qos,
            std::bind(
                &ControlWaypointsNode::vehicle_status_callback,
                this,
                std::placeholders::_1));

        // ============================================================
        // SERVICIO Y ACCIÓN
        // ============================================================
        arm_takeoff_srv_ =
            this->create_service<controllers_pkg::srv::ArmTakeoff>(
            arm_takeoff_service_,
            std::bind(
                &ControlWaypointsNode::arm_takeoff_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        action_server_ = rclcpp_action::create_server<FollowWaypoints>(
            this,
            follow_waypoints_action_,
            std::bind(
                &ControlWaypointsNode::handle_goal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            std::bind(
                &ControlWaypointsNode::handle_cancel,
                this,
                std::placeholders::_1),
            std::bind(
                &ControlWaypointsNode::handle_accepted,
                this,
                std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(
                static_cast<int>(control_period_ms_)),
            std::bind(&ControlWaypointsNode::tick, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Control waypoints ready | ROS namespace='/%s' | "
            "vehicle='%s' | PX4 status='%s' | ROS odom='%s' | "
            "marker_frame='%s' | mission_frame='%s' -> "
            "odom_frame='%s' | target_system=%d",
            namespace_path_.c_str(),
            vehicle_id_.c_str(),
            vehicle_status_topic_.c_str(),
            odom_topic_.c_str(),
            marker_frame_.c_str(),
            mission_frame_.c_str(),
            odom_frame_.c_str(),
            target_system_);
    }

private:
    enum class ControlState
    {
        IDLE,
        TAKEOFF,
        HOLD,
        MISSION,
        LANDING
    };

    static std::string normalise_name(std::string value)
    {
        while (!value.empty() && value.front() == '/') {
            value.erase(value.begin());
        }

        while (!value.empty() && value.back() == '/') {
            value.pop_back();
        }

        return value;
    }

    static std::string last_name_segment(const std::string & value)
    {
        const auto separator = value.find_last_of('/');
        return separator == std::string::npos ?
            value : value.substr(separator + 1U);
    }

    static void validate_absolute_graph_name(
        const std::string & value,
        const std::string & parameter_name)
    {
        if (value.empty()) {
            throw std::runtime_error(
                "Parameter '" + parameter_name + "' must not be empty");
        }

        if (value.front() != '/') {
            throw std::runtime_error(
                "Parameter '" + parameter_name +
                "' must be absolute because it addresses a topic "
                "under the flat PX4 DDS namespace");
        }

        if (value.size() == 1U || value.back() == '/' ||
            value.find("//") != std::string::npos)
        {
            throw std::runtime_error(
                "Parameter '" + parameter_name +
                "' is not a valid absolute ROS graph name");
        }
    }

    static void validate_relative_graph_name(
        const std::string & value,
        const std::string & parameter_name)
    {
        if (value.empty()) {
            throw std::runtime_error(
                "Parameter '" + parameter_name + "' must not be empty");
        }

        if (value.front() == '/') {
            throw std::runtime_error(
                "Parameter '" + parameter_name +
                "' must be relative so that the UAS namespace is applied");
        }

        if (value.find("//") != std::string::npos) {
            throw std::runtime_error(
                "Parameter '" + parameter_name +
                "' contains an empty ROS name segment");
        }
    }

    void validate_parameters()
    {
        validate_absolute_graph_name(
            vehicle_command_topic_, "vehicle_command_topic");
        validate_absolute_graph_name(
            offboard_control_mode_topic_, "offboard_control_mode_topic");
        validate_absolute_graph_name(
            trajectory_setpoint_topic_, "trajectory_setpoint_topic");
        validate_absolute_graph_name(
            vehicle_status_topic_, "vehicle_status_topic");
        validate_relative_graph_name(
            odom_topic_, "odom_topic");
        validate_relative_graph_name(
            marker_topic_, "marker_topic");
        validate_relative_graph_name(
            arm_takeoff_service_, "arm_takeoff_service");
        validate_relative_graph_name(
            follow_waypoints_action_, "follow_waypoints_action");
        validate_relative_graph_name(
            marker_frame_suffix_, "marker_frame_suffix");
        validate_relative_graph_name(
            odom_frame_suffix_, "odom_frame_suffix");

        if (mission_frame_.empty() ||
            mission_frame_.find("//") != std::string::npos)
        {
            throw std::runtime_error(
                "mission_frame must be a valid TF frame name");
        }

        if (transform_timeout_s_ < 0.0) {
            throw std::runtime_error(
                "transform_timeout_s must not be negative");
        }

        if (target_system_ < 1 || target_system_ > 255) {
            throw std::runtime_error(
                "target_system must be in the range [1, 255]");
        }

        if (
            control_period_ms_ <= 0.0 ||
            takeoff_tolerance_xy_ <= 0.0 ||
            takeoff_tolerance_z_ <= 0.0 ||
            landing_tolerance_xy_ <= 0.0 ||
            landing_command_retry_s_ <= 0.0)
        {
            throw std::runtime_error(
                "control_period_ms, takeoff/landing tolerances and landing retry period must be positive");
        }

        if (
            max_vx_ <= 0.0 ||
            max_vy_ <= 0.0 ||
            max_vz_ <= 0.0 ||
            max_speed_xy_ <= 0.0)
        {
            throw std::runtime_error(
                "Velocity limits must be positive");
        }

        if (
            integral_limit_x_ < 0.0 ||
            integral_limit_y_ < 0.0 ||
            integral_limit_z_ < 0.0)
        {
            throw std::runtime_error(
                "Integral limits must not be negative");
        }
    }

    // ============================================================
    // UTILIDADES
    // ============================================================
    uint64_t now_us() const
    {
        return this->now().nanoseconds() / 1000;
    }

    static double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(value, high));
    }

    static double norm2d(double x, double y)
    {
        return std::sqrt(x * x + y * y);
    }

    static double nan_f64()
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    // yaw ENU: 0 rad = Este, CCW positivo
    // yaw NED: 0 rad = Norte, CW positivo
    static double yaw_enu_to_ned(double yaw_enu)
    {
        return 1.5707963267948966 - yaw_enu;
    }

    // ============================================================
    // CALLBACKS DE ESTADO
    // ============================================================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        has_odom_ = true;
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        nav_state_ = msg->nav_state;
        arming_state_ = msg->arming_state;
        has_vehicle_status_ = true;
    }

    // ============================================================
    // SERVICIO ARM + TAKEOFF
    // ============================================================
    void arm_takeoff_callback(
        const std::shared_ptr<controllers_pkg::srv::ArmTakeoff::Request> req,
        std::shared_ptr<controllers_pkg::srv::ArmTakeoff::Response> res)
    {
        if (!has_odom_) {
            res->accepted = false;
            res->message = "No hay odometría disponible";
            return;
        }

        if (active_goal_handle_) {
            res->accepted = false;
            res->message = "Hay una misión activa; cancélala antes de pedir un nuevo despegue";
            return;
        }

        // El mismo servicio se mantiene por compatibilidad con control_manager_node,
        // pero su significado depende del estado del controlador:
        //   IDLE -> ARM + TAKEOFF
        //   HOLD -> LANDING
        // Esto evita interpretar el punto final de landing como un nuevo despegue.
        if (control_state_ == ControlState::HOLD) {
            landing_x_ = req->x;
            landing_y_ = req->y;
            landing_z_reference_ = req->z;
            landing_yaw_enu_ = req->yaw;
            landing_approach_z_ = current_z_;
            landing_command_sent_ = false;
            last_landing_command_time_s_ = -std::numeric_limits<double>::infinity();
            control_state_ = ControlState::LANDING;
            reset_pid_state();

            res->accepted = true;
            res->message = "Secuencia de aterrizaje iniciada";

            RCLCPP_INFO(
                this->get_logger(),
                "[%s] LANDING recibido: punto=(%.2f, %.2f, %.2f), approach_z=%.2f",
                namespace_path_.c_str(), landing_x_, landing_y_,
                landing_z_reference_, landing_approach_z_);
            return;
        }

        if (control_state_ != ControlState::IDLE) {
            res->accepted = false;
            res->message = "El controlador no está en IDLE ni HOLD";
            return;
        }

        hold_x_ = req->x;
        hold_y_ = req->y;
        hold_z_ = req->z;
        hold_yaw_enu_ = req->yaw;

        warmup_counter_ = 0;
        offboard_command_sent_ = false;
        arm_command_sent_ = false;
        control_state_ = ControlState::TAKEOFF;

        res->accepted = true;
        res->message = "Secuencia de armado y despegue iniciada";

        RCLCPP_INFO(this->get_logger(),
                    "[%s] ARM_TAKEOFF recibido: hold=(%.2f, %.2f, %.2f), yaw_enu=%.2f",
                    namespace_path_.c_str(), hold_x_, hold_y_, hold_z_, hold_yaw_enu_);
    }

    bool transform_mission_to_odom(
        const FollowWaypoints::Goal & goal,
        std::vector<std::array<double, 3>> & transformed_points,
        std::string & error_message)
    {
        geometry_msgs::msg::TransformStamped transform;

        try {
            // lookupTransform(target, source) devuelve T_target_source.
            // Necesitamos p_odom = T_odom_map * p_map.
            transform = tf_buffer_->lookupTransform(
                odom_frame_,
                mission_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(transform_timeout_s_));
        } catch (const tf2::TransformException & error) {
            error_message =
                "No se puede transformar la misión desde '" +
                mission_frame_ + "' hasta '" + odom_frame_ +
                "': " + error.what();
            return false;
        }

        transformed_points.clear();
        transformed_points.reserve(goal.x.size());

        for (std::size_t index = 0; index < goal.x.size(); ++index) {
            geometry_msgs::msg::PointStamped point_map;
            point_map.header.frame_id = mission_frame_;
            point_map.header.stamp = transform.header.stamp;
            point_map.point.x = goal.x[index];
            point_map.point.y = goal.y[index];
            point_map.point.z = goal.height;

            geometry_msgs::msg::PointStamped point_odom;

            try {
                tf2::doTransform(point_map, point_odom, transform);
            } catch (const tf2::TransformException & error) {
                error_message =
                    "Error transformando el waypoint " +
                    std::to_string(index) + ": " + error.what();
                transformed_points.clear();
                return false;
            }

            transformed_points.push_back(
                {
                    point_odom.point.x,
                    point_odom.point.y,
                    point_odom.point.z
                });
        }

        return true;
    }

    // ============================================================
    // ACTION SERVER
    // ============================================================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const FollowWaypoints::Goal> goal)
    {
        if (!has_odom_) {
            RCLCPP_WARN(this->get_logger(), "Rechazando goal: no hay odometría");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (control_state_ == ControlState::IDLE ||
            control_state_ == ControlState::TAKEOFF ||
            control_state_ == ControlState::LANDING) {
            RCLCPP_WARN(this->get_logger(),
                        "Rechazando goal: el dron no está disponible en HOLD para iniciar misión");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (active_goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "Rechazando goal: ya existe una misión activa");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->x.empty() || goal->x.size() != goal->y.size()) {
            RCLCPP_WARN(this->get_logger(),
                        "Rechazando goal: x e y deben tener la misma longitud y no estar vacíos");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->height <= 0.0 || goal->goal_tolerance <= 0.0 || goal->slowdown_radius <= 0.0) {
            RCLCPP_WARN(this->get_logger(),
                        "Rechazando goal: height, goal_tolerance y slowdown_radius deben ser > 0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        try {
            tf_buffer_->lookupTransform(
                odom_frame_,
                mission_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(transform_timeout_s_));
        } catch (const tf2::TransformException & error) {
            RCLCPP_WARN(
                this->get_logger(),
                "Rechazando goal: no existe TF %s -> %s: %s",
                mission_frame_.c_str(),
                odom_frame_.c_str(),
                error.what());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle)
    {
        if (goal_handle != active_goal_handle_) {
            return rclcpp_action::CancelResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Cancelación de misión solicitada");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFollowWaypoints> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        active_goal_handle_ = goal_handle;

        std::string transform_error;
        if (!transform_mission_to_odom(
                *goal,
                mission_points_,
                transform_error))
        {
            auto result = std::make_shared<FollowWaypoints::Result>();
            result->success = false;
            result->message = transform_error;
            result->waypoints_reached = 0;

            active_goal_handle_->abort(result);
            active_goal_handle_.reset();

            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] No se pudo preparar la misión: %s",
                namespace_path_.c_str(),
                transform_error.c_str());
            return;
        }

        mission_height_map_ = goal->height;
        mission_goal_tolerance_ = goal->goal_tolerance;
        mission_slowdown_radius_ = goal->slowdown_radius;
        mission_repetitions_ = std::max<uint32_t>(1, goal->repetitions);

        mission_waypoint_index_ = 0;
        mission_completed_repetitions_ = 0;
        mission_waypoints_reached_ = 0;
        reset_pid_state();
        control_state_ = ControlState::MISSION;

        RCLCPP_INFO(
            this->get_logger(),
            "[%s] Misión aceptada: %zu waypoints en %s, "
            "height_map=%.2f, transformados a %s, repetitions=%u",
            namespace_path_.c_str(),
            mission_points_.size(),
            mission_frame_.c_str(),
            mission_height_map_,
            odom_frame_.c_str(),
            mission_repetitions_);
    }

    void finish_active_goal(bool success, const std::string & message)
    {
        if (!active_goal_handle_) {
            return;
        }

        auto result = std::make_shared<FollowWaypoints::Result>();
        result->success = success;
        result->message = message;
        result->waypoints_reached = mission_waypoints_reached_;

        if (success) {
            active_goal_handle_->succeed(result);
        } else {
            active_goal_handle_->abort(result);
        }

        active_goal_handle_.reset();
        control_state_ = ControlState::HOLD;
        hold_x_ = current_x_;
        hold_y_ = current_y_;
        hold_z_ = current_z_;
        reset_pid_state();
    }

    void cancel_active_goal()
    {
        if (!active_goal_handle_) {
            return;
        }

        auto result = std::make_shared<FollowWaypoints::Result>();
        result->success = false;
        result->message = "Misión cancelada";
        result->waypoints_reached = mission_waypoints_reached_;
        active_goal_handle_->canceled(result);

        active_goal_handle_.reset();
        control_state_ = ControlState::HOLD;
        hold_x_ = current_x_;
        hold_y_ = current_y_;
        hold_z_ = current_z_;
        reset_pid_state();
    }

    // ============================================================
    // VEHICLE COMMAND
    // ============================================================
    void send_vehicle_command(
        uint16_t cmd,
        float p1 = 0.0f,
        float p2 = 0.0f,
        float p3 = 0.0f,
        float p4 = 0.0f,
        double p5 = 0.0,
        double p6 = 0.0,
        float p7 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = now_us();
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.param3 = p3;
        msg.param4 = p4;
        msg.param5 = p5;
        msg.param6 = p6;
        msg.param7 = p7;
        msg.target_system = target_system_;
        msg.target_component = 1;
        msg.source_system = target_system_;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    // ============================================================
    // PUBLICACIÓN PX4
    // ============================================================
    void publish_position_offboard_control_mode(uint64_t ts)
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = ts;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        ocm_pub_->publish(msg);
    }

    void publish_velocity_offboard_control_mode(uint64_t ts)
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = ts;
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        ocm_pub_->publish(msg);
    }

    void publish_position_setpoint_enu(uint64_t ts, double x_enu, double y_enu, double z_enu, double yaw_enu)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = ts;

        // ENU -> NED
        sp.position = {
            static_cast<float>(y_enu),
            static_cast<float>(x_enu),
            static_cast<float>(-z_enu)
        };

        sp.velocity = {
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64())
        };

        sp.acceleration = {
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64())
        };

        sp.yaw = static_cast<float>(yaw_enu_to_ned(yaw_enu));
        sp.yawspeed = static_cast<float>(nan_f64());
        setpoint_pub_->publish(sp);
    }

    void publish_velocity_setpoint_enu(uint64_t ts, double vx_enu, double vy_enu, double vz_enu)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = ts;

        sp.position = {
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64())
        };

        // ENU -> NED
        sp.velocity = {
            static_cast<float>(vy_enu),
            static_cast<float>(vx_enu),
            static_cast<float>(-vz_enu)
        };

        sp.acceleration = {
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64())
        };

        sp.yaw = static_cast<float>(nan_f64());
        sp.yawspeed = 0.0f;
        setpoint_pub_->publish(sp);
    }

    // ============================================================
    // PID
    // ============================================================
    void reset_pid_state()
    {
        ix_ = 0.0;
        iy_ = 0.0;
        iz_ = 0.0;

        prev_ex_ = 0.0;
        prev_ey_ = 0.0;
        prev_ez_ = 0.0;

        pid_initialized_ = false;
        first_derivative_tick_ = true;
        last_control_time_ = this->now().seconds();
    }

    std::array<double, 3> current_mission_goal() const
    {
        return mission_points_[mission_waypoint_index_];
    }

    void advance_mission_if_goal_reached()
    {
        const auto goal = current_mission_goal();
        const double ex = goal[0] - current_x_;
        const double ey = goal[1] - current_y_;
        const double ez = goal[2] - current_z_;
        const double dist = std::sqrt(
            ex * ex + ey * ey + ez * ez);

        if (dist > mission_goal_tolerance_) {
            return;
        }

        mission_waypoints_reached_++;
        RCLCPP_INFO(this->get_logger(),
                    "[%s] Waypoint alcanzado: %zu/%zu | "
                    "objetivo_odom=(%.2f, %.2f, %.2f)",
                    namespace_path_.c_str(),
                    mission_waypoint_index_ + 1,
                    mission_points_.size(),
                    goal[0], goal[1], goal[2]);

        mission_waypoint_index_++;
        reset_pid_state();

        if (mission_waypoint_index_ < mission_points_.size()) {
            return;
        }

        mission_completed_repetitions_++;

        if (mission_completed_repetitions_ >= mission_repetitions_) {
            finish_active_goal(true, "Misión completada");
            return;
        }

        mission_waypoint_index_ = 0;
        RCLCPP_INFO(this->get_logger(),
                    "[%s] Repetición completada: %u/%u",
                    namespace_path_.c_str(), mission_completed_repetitions_, mission_repetitions_);
    }

    void compute_and_publish_mission_control(uint64_t ts)
    {
        const auto goal = current_mission_goal();

        const double ex = goal[0] - current_x_;
        const double ey = goal[1] - current_y_;
        const double ez = goal[2] - current_z_;
        const double dist_xy = norm2d(ex, ey);

        const double now_s = this->now().seconds();
        double dt = now_s - last_control_time_;
        if (!pid_initialized_) {
            dt = control_period_ms_ / 1000.0;
            pid_initialized_ = true;
        }
        last_control_time_ = now_s;

        if (dt <= 1e-6) {
            dt = control_period_ms_ / 1000.0;
        }

        ix_ += ex * dt;
        iy_ += ey * dt;
        iz_ += ez * dt;

        ix_ = clamp(ix_, -integral_limit_x_, integral_limit_x_);
        iy_ = clamp(iy_, -integral_limit_y_, integral_limit_y_);
        iz_ = clamp(iz_, -integral_limit_z_, integral_limit_z_);

        double dex = 0.0;
        double dey = 0.0;
        double dez = 0.0;

        if (!first_derivative_tick_) {
            dex = (ex - prev_ex_) / dt;
            dey = (ey - prev_ey_) / dt;
            dez = (ez - prev_ez_) / dt;
        }
        first_derivative_tick_ = false;

        prev_ex_ = ex;
        prev_ey_ = ey;
        prev_ez_ = ez;

        double vx = kp_x_ * ex + ki_x_ * ix_ + kd_x_ * dex;
        double vy = kp_y_ * ey + ki_y_ * iy_ + kd_y_ * dey;
        double vz = kp_z_ * ez + ki_z_ * iz_ + kd_z_ * dez;

        double effective_max_xy = max_speed_xy_;
        if (dist_xy < mission_slowdown_radius_ && mission_slowdown_radius_ > 1e-6) {
            const double alpha = clamp(dist_xy / mission_slowdown_radius_, 0.15, 1.0);
            effective_max_xy *= alpha;
        }

        vx = clamp(vx, -max_vx_, max_vx_);
        vy = clamp(vy, -max_vy_, max_vy_);
        vz = clamp(vz, -max_vz_, max_vz_);

        const double vxy = norm2d(vx, vy);
        if (vxy > effective_max_xy && vxy > 1e-9) {
            const double scale = effective_max_xy / vxy;
            vx *= scale;
            vy *= scale;
        }

        publish_velocity_offboard_control_mode(ts);
        publish_velocity_setpoint_enu(ts, vx, vy, vz);
    }

    // ============================================================
    // FEEDBACK ACTION
    // ============================================================
    void publish_action_feedback()
    {
        if (!active_goal_handle_) {
            return;
        }

        auto feedback = std::make_shared<FollowWaypoints::Feedback>();
        feedback->pose = current_pose_;
        feedback->nav_state = nav_state_;
        feedback->arming_state = arming_state_;
        feedback->current_waypoint = static_cast<uint32_t>(mission_waypoint_index_);

        const auto goal = current_mission_goal();
        feedback->distance_to_waypoint = norm2d(goal[0] - current_x_, goal[1] - current_y_);
        active_goal_handle_->publish_feedback(feedback);
    }

    // ============================================================
    // MARKERS RVIZ2
    // ============================================================
    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray array;

        {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = marker_frame_;
            del.header.stamp = this->now();
            del.ns = "mission";
            del.id = 0;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            array.markers.push_back(del);
        }

        if (!mission_points_.empty()) {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = marker_frame_;
            line.header.stamp = this->now();
            line.ns = "mission_path";
            line.id = 1;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.06;
            line.color.r = 0.1f;
            line.color.g = 0.9f;
            line.color.b = 0.2f;
            line.color.a = 1.0f;

            for (const auto & p : mission_points_) {
                line.points.push_back(make_point(p[0], p[1], p[2]));
            }
            array.markers.push_back(line);

            visualization_msgs::msg::Marker pts;
            pts.header.frame_id = marker_frame_;
            pts.header.stamp = this->now();
            pts.ns = "mission_points";
            pts.id = 2;
            pts.type = visualization_msgs::msg::Marker::POINTS;
            pts.action = visualization_msgs::msg::Marker::ADD;
            pts.pose.orientation.w = 1.0;
            pts.scale.x = 0.20;
            pts.scale.y = 0.20;
            pts.color.r = 0.0f;
            pts.color.g = 0.6f;
            pts.color.b = 1.0f;
            pts.color.a = 1.0f;

            for (const auto & p : mission_points_) {
                pts.points.push_back(make_point(p[0], p[1], p[2]));
            }
            array.markers.push_back(pts);

            if (control_state_ == ControlState::MISSION) {
                const auto goal = current_mission_goal();

                visualization_msgs::msg::Marker goal_marker;
                goal_marker.header.frame_id = marker_frame_;
                goal_marker.header.stamp = this->now();
                goal_marker.ns = "mission_goal";
                goal_marker.id = 3;
                goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
                goal_marker.action = visualization_msgs::msg::Marker::ADD;
                goal_marker.pose.position.x = goal[0];
                goal_marker.pose.position.y = goal[1];
                goal_marker.pose.position.z = goal[2];
                goal_marker.pose.orientation.w = 1.0;
                goal_marker.scale.x = 0.35;
                goal_marker.scale.y = 0.35;
                goal_marker.scale.z = 0.35;
                goal_marker.color.r = 1.0f;
                goal_marker.color.g = 0.2f;
                goal_marker.color.b = 0.2f;
                goal_marker.color.a = 1.0f;
                array.markers.push_back(goal_marker);
            }
        }

        marker_pub_->publish(array);
    }

    // ============================================================
    // LOOP PRINCIPAL
    // ============================================================
    void tick()
    {
        const uint64_t ts = now_us();
        publish_markers();

        if (!has_odom_) {
            return;
        }

        switch (control_state_) {
        case ControlState::IDLE:
            // Antes de que llegue el servicio no se manda control activo.
            return;

        case ControlState::TAKEOFF:
            tick_takeoff(ts);
            return;

        case ControlState::HOLD:
            publish_position_offboard_control_mode(ts);
            publish_position_setpoint_enu(ts, hold_x_, hold_y_, hold_z_, hold_yaw_enu_);
            return;

        case ControlState::MISSION:
            tick_mission(ts);
            return;

        case ControlState::LANDING:
            tick_landing(ts);
            return;
        }
    }

    void tick_takeoff(uint64_t ts)
    {
        warmup_counter_++;
        publish_position_offboard_control_mode(ts);
        publish_position_setpoint_enu(ts, hold_x_, hold_y_, hold_z_, hold_yaw_enu_);

        if (warmup_counter_ == 50 && !offboard_command_sent_) {
            // VEHICLE_CMD_DO_SET_MODE = 176 | param1=1 custom mode | param2=6 offboard
            send_vehicle_command(176, 1.0f, 6.0f);
            offboard_command_sent_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Comando offboard enviado");
        }

        if (warmup_counter_ == 100 && !arm_command_sent_) {
            // VEHICLE_CMD_COMPONENT_ARM_DISARM = 400 | param1=1 arm
            RCLCPP_INFO(this->get_logger(),
                        "Comando de armado enviado");

            send_vehicle_command(400, 1.0f);
            arm_command_sent_ = true;
        }

        const double dist_xy = norm2d(hold_x_ - current_x_, hold_y_ - current_y_);
        const double err_z = std::abs(hold_z_ - current_z_);
        
        if (!arm_command_sent_) {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] Esperando comando de armado / takeoff hacia hold=(%.2f, %.2f, %.2f)",
                        namespace_path_.c_str(), hold_x_, hold_y_, hold_z_);
            return;
        }

        const bool xy_reached = dist_xy <= takeoff_tolerance_xy_;
        const bool z_reached  = err_z   <= takeoff_tolerance_z_;

        if (xy_reached && z_reached) {
            control_state_ = ControlState::HOLD;
        
            RCLCPP_INFO(this->get_logger(),
                        "[%s] Vuelo estable alcanzado en hold=(%.2f, %.2f, %.2f)",
                        namespace_path_.c_str(), hold_x_, hold_y_, hold_z_);
        }
        else if (!xy_reached && !z_reached) {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] En proceso hacia hold=(%.2f, %.2f, %.2f). dist_xy=%.2f, err_z=%.2f",
                        namespace_path_.c_str(), hold_x_, hold_y_, hold_z_, dist_xy, err_z);
        }
        else if (xy_reached && !z_reached) {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] Posición x,y alcanzada. Ajustando altura hacia z=%.2f. err_z=%.2f",
                        namespace_path_.c_str(), hold_z_, err_z);
        }
        else if (!xy_reached && z_reached) {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] Altura alcanzada. Ajustando posición x,y hacia hold=(%.2f, %.2f). dist_xy=%.2f",
                        namespace_path_.c_str(), hold_x_, hold_y_, dist_xy);
        }
        
    }

    void tick_landing(uint64_t ts)
    {
        // Finalización real: el aterrizaje no termina hasta que PX4 confirma
        // que el vehículo se encuentra desarmado.
        if (has_vehicle_status_ &&
            arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
        {
            control_state_ = ControlState::IDLE;
            landing_command_sent_ = false;
            RCLCPP_INFO(
                this->get_logger(),
                "[%s] Aterrizaje completado: PX4 confirma DISARMED",
                namespace_path_.c_str());
            return;
        }

        // Cuando PX4 ya ha entrado en AUTO_LAND dejamos de publicar OFFBOARD.
        // El autopiloto gestiona descenso, detección de suelo y auto-disarm.
        if (has_vehicle_status_ &&
            nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
        {
            return;
        }

        // Aproximación horizontal: antes de ceder el control a AUTO_LAND, el
        // UAS se coloca sobre el XY del punto de aterrizaje manteniendo la
        // altura con la que entró en esta fase.
        publish_position_offboard_control_mode(ts);
        publish_position_setpoint_enu(
            ts, landing_x_, landing_y_, landing_approach_z_, landing_yaw_enu_);

        const double dist_xy = norm2d(
            landing_x_ - current_x_, landing_y_ - current_y_);

        if (dist_xy > landing_tolerance_xy_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "[%s] Aproximación a landing XY=(%.2f, %.2f), dist_xy=%.2f",
                namespace_path_.c_str(), landing_x_, landing_y_, dist_xy);
            return;
        }

        // VEHICLE_CMD_NAV_LAND = 21. Param5/6/7 en NaN indican que PX4 debe
        // aterrizar desde la posición actual; el XY ya ha sido alineado por el
        // control OFFBOARD anterior. Se reintenta hasta observar AUTO_LAND.
        const double now_s = this->now().seconds();
        if (!landing_command_sent_ ||
            now_s - last_landing_command_time_s_ >= landing_command_retry_s_)
        {
            const float nan_f = std::numeric_limits<float>::quiet_NaN();
            const double nan_d = std::numeric_limits<double>::quiet_NaN();
            send_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
                0.0f, 0.0f, 0.0f, nan_f, nan_d, nan_d, nan_f);
            landing_command_sent_ = true;
            last_landing_command_time_s_ = now_s;
            RCLCPP_INFO(
                this->get_logger(),
                "[%s] Comando PX4 NAV_LAND enviado; esperando AUTO_LAND/DISARMED",
                namespace_path_.c_str());
        }
    }

    void tick_mission(uint64_t ts)
    {
        if (!active_goal_handle_) {
            control_state_ = ControlState::HOLD;
            return;
        }

        if (active_goal_handle_->is_canceling()) {
            cancel_active_goal();
            return;
        }

        advance_mission_if_goal_reached();

        // advance_mission_if_goal_reached() puede haber terminado la misión.
        if (control_state_ != ControlState::MISSION || !active_goal_handle_) {
            return;
        }

        compute_and_publish_mission_control(ts);
        publish_action_feedback();
    }

private:
    // ============================================================
    // PARÁMETROS Y ESTADO
    // ============================================================
    std::string namespace_path_;
    std::string vehicle_id_;

    std::string vehicle_command_topic_{
        "/test_ua_1/fmu/in/vehicle_command"};
    std::string offboard_control_mode_topic_{
        "/test_ua_1/fmu/in/offboard_control_mode"};
    std::string trajectory_setpoint_topic_{
        "/test_ua_1/fmu/in/trajectory_setpoint"};
    std::string vehicle_status_topic_{
        "/test_ua_1/fmu/out/vehicle_status"};
    std::string odom_topic_{"odom"};
    std::string marker_topic_{"mission_markers"};
    std::string arm_takeoff_service_{"arm_takeoff"};
    std::string follow_waypoints_action_{"follow_waypoints"};

    std::string marker_frame_suffix_{"odom"};
    std::string explicit_marker_frame_;
    std::string marker_frame_;

    std::string mission_frame_{"map"};
    std::string odom_frame_suffix_{"odom"};
    std::string odom_frame_;
    double transform_timeout_s_{0.20};

    int target_system_{1};

    ControlState control_state_{ControlState::IDLE};

    // HOLD / TAKEOFF
    double hold_x_{0.0};
    double hold_y_{0.0};
    double hold_z_{0.0};
    double hold_yaw_enu_{0.0};
    int warmup_counter_{0};
    bool offboard_command_sent_{false};
    bool arm_command_sent_{false};
    double takeoff_tolerance_xy_{0.30};
    double takeoff_tolerance_z_{0.20};

    // LANDING. El servicio arm_takeoff se reutiliza en HOLD por compatibilidad
    // con control_manager_node; la ejecución real usa PX4 NAV_LAND.
    double landing_x_{0.0};
    double landing_y_{0.0};
    double landing_z_reference_{0.0};
    double landing_yaw_enu_{0.0};
    double landing_approach_z_{0.0};
    double landing_tolerance_xy_{0.30};
    double landing_command_retry_s_{1.0};
    double last_landing_command_time_s_{-1.0};
    bool landing_command_sent_{false};

    // Misión
    std::vector<std::array<double, 3>> mission_points_;
    std::size_t mission_waypoint_index_{0};
    uint32_t mission_repetitions_{1};
    uint32_t mission_completed_repetitions_{0};
    uint32_t mission_waypoints_reached_{0};
    double mission_height_map_{5.0};
    double mission_goal_tolerance_{0.30};
    double mission_slowdown_radius_{1.5};

    // PID
    double kp_x_{0.9}, ki_x_{0.0}, kd_x_{0.15};
    double kp_y_{0.9}, ki_y_{0.0}, kd_y_{0.15};
    double kp_z_{0.8}, ki_z_{0.0}, kd_z_{0.10};

    double max_vx_{1.5};
    double max_vy_{1.5};
    double max_vz_{0.8};
    double max_speed_xy_{1.8};

    double integral_limit_x_{2.0};
    double integral_limit_y_{2.0};
    double integral_limit_z_{2.0};
    double control_period_ms_{50.0};

    double last_control_time_{0.0};
    bool pid_initialized_{false};
    bool first_derivative_tick_{true};
    double ix_{0.0}, iy_{0.0}, iz_{0.0};
    double prev_ex_{0.0}, prev_ey_{0.0}, prev_ez_{0.0};

    // Estado del vehículo
    geometry_msgs::msg::Pose current_pose_;
    double current_x_{0.0};
    double current_y_{0.0};
    double current_z_{0.0};
    bool has_odom_{false};

    uint8_t nav_state_{0};
    uint8_t arming_state_{0};
    bool has_vehicle_status_{false};

    // ============================================================
    // ROS
    // ============================================================
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    rclcpp::Service<controllers_pkg::srv::ArmTakeoff>::SharedPtr arm_takeoff_srv_;
    rclcpp_action::Server<FollowWaypoints>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleFollowWaypoints> active_goal_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlWaypointsNode>());
    rclcpp::shutdown();
    return 0;
}