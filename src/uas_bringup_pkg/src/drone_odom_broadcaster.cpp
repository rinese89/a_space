#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

// =============================================================================
// Estructura ROS 2 utilizada
//
// El nodo debe lanzarse dentro del namespace de cada aeronave:
//
//   namespace = ua_1
//   ns        = ua_1
//   id        = 1
//
// Los nombres de los tópicos y servicios son RELATIVOS. ROS 2 aplica el
// namespace del nodo automáticamente:
//
//   fmu/out/vehicle_odometry -> /ua_1/fmu/out/vehicle_odometry
//   odom                     -> /ua_1/odom
//   reset_odom_srv           -> /ua_1/reset_odom_srv
//
// Los frame_id no son namespaced automáticamente por ROS 2. Por ello se
// construyen explícitamente:
//
//   ua_1/odom -> ua_1/base_link
//
// No se añade ningún desplazamiento artificial basado en el identificador.
// La posición publicada procede directamente de PX4 y de la pose de aparición
// configurada para el modelo en Gazebo.
// =============================================================================

namespace
{

std::string strip_slashes(std::string value)
{
    while (!value.empty() && value.front() == '/') {
        value.erase(value.begin());
    }

    while (!value.empty() && value.back() == '/') {
        value.pop_back();
    }

    return value;
}

std::string make_namespaced_frame(
    const std::string & ns,
    const std::string & frame)
{
    const std::string clean_ns = strip_slashes(ns);
    const std::string clean_frame = strip_slashes(frame);

    if (clean_ns.empty()) {
        return clean_frame;
    }

    const std::string prefix = clean_ns + "/";
    if (clean_frame.rfind(prefix, 0) == 0) {
        return clean_frame;
    }

    return prefix + clean_frame;
}

inline bool is_finite3(const std::array<float, 3> & v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline bool is_finite4(const std::array<float, 4> & q)
{
    return std::isfinite(q[0]) && std::isfinite(q[1]) &&
           std::isfinite(q[2]) && std::isfinite(q[3]);
}

// NED -> ENU:
// [x_enu]   [0 1  0][x_ned]
// [y_enu] = [1 0  0][y_ned]
// [z_enu]   [0 0 -1][z_ned]
Eigen::Matrix3d ned_to_enu_rotation()
{
    Eigen::Matrix3d rotation;
    rotation << 0.0, 1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 0.0, -1.0;
    return rotation;
}

// FRD -> FLU:
// x forward se mantiene.
// y right   -> y left.
// z down    -> z up.
Eigen::Matrix3d frd_to_flu_rotation()
{
    Eigen::Matrix3d rotation;
    rotation << 1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0;
    return rotation;
}

Eigen::Vector3d ned_to_enu_vector(const Eigen::Vector3d & vector_ned)
{
    static const Eigen::Matrix3d rotation = ned_to_enu_rotation();
    return rotation * vector_ned;
}

Eigen::Vector3d frd_to_flu_vector(const Eigen::Vector3d & vector_frd)
{
    static const Eigen::Matrix3d rotation = frd_to_flu_rotation();
    return rotation * vector_frd;
}

Eigen::Quaterniond px4_q_to_ros_q(
    const std::array<float, 4> & q_px4,
    const uint8_t pose_frame)
{
    const Eigen::Quaterniond q_body_to_reference_px4(
        static_cast<double>(q_px4[0]),
        static_cast<double>(q_px4[1]),
        static_cast<double>(q_px4[2]),
        static_cast<double>(q_px4[3]));

    const Eigen::Matrix3d rotation_reference_body =
        q_body_to_reference_px4.normalized().toRotationMatrix();

    const Eigen::Matrix3d rotation_frd_to_flu = frd_to_flu_rotation();
    const Eigen::Matrix3d rotation_flu_to_frd = rotation_frd_to_flu.transpose();

    Eigen::Matrix3d rotation_ros_parent_child;

    if (pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
        rotation_ros_parent_child =
            ned_to_enu_rotation() *
            rotation_reference_body *
            rotation_flu_to_frd;
    } else if (pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD) {
        rotation_ros_parent_child =
            rotation_reference_body * rotation_flu_to_frd;
    } else {
        return Eigen::Quaterniond::Identity();
    }

    Eigen::Quaterniond q_ros(rotation_ros_parent_child);
    q_ros.normalize();
    return q_ros;
}

}  // namespace

class DroneOdomBroadcaster : public rclcpp::Node
{
public:
    DroneOdomBroadcaster()
    : Node("drone_odom_broadcaster")
    {
        // Identidad del UAS. Debe coincidir con el namespace usado en el launch.
        this->declare_parameter<std::string>("ns", "");
        this->declare_parameter<int>("id", 1);

        // Nombres relativos. El namespace del nodo se aplica automáticamente.
        this->declare_parameter<std::string>(
            "vehicle_odometry_topic", "fmu/out/vehicle_odometry");
        this->declare_parameter<std::string>("odom_topic", "odom");
        this->declare_parameter<std::string>("reset_service", "reset_odom_srv");

        // Nombres base de los frames. El namespace se añade explícitamente.
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");

        ns_ = strip_slashes(this->get_parameter("ns").as_string());
        id_ = static_cast<int>(this->get_parameter("id").as_int());

        vehicle_odometry_topic_ =
            this->get_parameter("vehicle_odometry_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        reset_service_name_ = this->get_parameter("reset_service").as_string();

        const std::string node_namespace = strip_slashes(this->get_namespace());

        if (ns_.empty()) {
            ns_ = node_namespace;
        } else if (!node_namespace.empty() && ns_ != node_namespace) {
            RCLCPP_WARN(
                this->get_logger(),
                "El parametro ns='%s' no coincide con el namespace ROS del nodo '%s'. "
                "Los topics se resolveran con el namespace ROS y los frames con ns.",
                ns_.c_str(),
                node_namespace.c_str());
        }

        if (ns_.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "El nodo no tiene namespace. Se publicaran los frames sin prefijo.");
        }

        const std::string odom_frame_parameter =
            this->get_parameter("odom_frame").as_string();
        const std::string base_frame_parameter =
            this->get_parameter("base_frame").as_string();

        odom_frame_ = make_namespaced_frame(ns_, odom_frame_parameter);
        base_frame_ = make_namespaced_frame(ns_, base_frame_parameter);

        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        rclcpp::QoS px4_qos(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        px4_qos.best_effort();

        odom_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            vehicle_odometry_topic_,
            px4_qos,
            std::bind(
                &DroneOdomBroadcaster::odom_callback,
                this,
                std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            odom_topic_,
            rclcpp::QoS(10));

        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            reset_service_name_,
            std::bind(
                &DroneOdomBroadcaster::reset_service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        RCLCPP_INFO(
            this->get_logger(),
            "UAS iniciado | ns='%s' | id=%d",
            ns_.c_str(),
            id_);

        RCLCPP_INFO(
            this->get_logger(),
            "Nombres relativos | entrada PX4='%s' | odom='%s' | reset='%s' | "
            "namespace ROS='%s'",
            vehicle_odometry_topic_.c_str(),
            odom_topic_.c_str(),
            reset_service_name_.c_str(),
            this->get_namespace());

        RCLCPP_INFO(
            this->get_logger(),
            "TF publicado: %s -> %s",
            odom_frame_.c_str(),
            base_frame_.c_str());
    }

private:
    void odom_callback(
        const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_odom_msg_ =
            std::make_shared<px4_msgs::msg::VehicleOdometry>(*msg);

        if (!is_finite3(msg->position) || !is_finite4(msg->q)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "VehicleOdometry contiene position/q no finitos. Se ignora el mensaje.");
            return;
        }

        Eigen::Vector3d position_ros = Eigen::Vector3d::Zero();

        double origin_x = 0.0;
        double origin_y = 0.0;
        double origin_z = 0.0;

        if (origin_is_set_) {
            if (origin_pose_frame_ == msg->pose_frame) {
                origin_x = origin_pose_x_;
                origin_y = origin_pose_y_;
                origin_z = origin_pose_z_;
            } else {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "El pose_frame ha cambiado despues del reset. "
                    "No se aplica el origen almacenado.");
            }
        }

        if (msg->pose_frame ==
            px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED)
        {
            const Eigen::Vector3d position_ned(
                static_cast<double>(msg->position[0]) - origin_x,
                static_cast<double>(msg->position[1]) - origin_y,
                static_cast<double>(msg->position[2]) - origin_z);

            position_ros = ned_to_enu_vector(position_ned);
        } else if (
            msg->pose_frame ==
            px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD)
        {
            position_ros.x() =
                static_cast<double>(msg->position[0]) - origin_x;
            position_ros.y() =
                static_cast<double>(msg->position[1]) - origin_y;
            position_ros.z() =
                static_cast<double>(msg->position[2]) - origin_z;
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "pose_frame desconocido: %u. Se ignora el mensaje.",
                static_cast<unsigned>(msg->pose_frame));
            return;
        }

        const Eigen::Quaterniond orientation_ros =
            px4_q_to_ros_q(msg->q, msg->pose_frame);

        Eigen::Vector3d linear_velocity_ros = Eigen::Vector3d::Zero();

        if (is_finite3(msg->velocity)) {
            const Eigen::Vector3d input_velocity(
                static_cast<double>(msg->velocity[0]),
                static_cast<double>(msg->velocity[1]),
                static_cast<double>(msg->velocity[2]));

            switch (msg->velocity_frame) {
                case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED:
                    linear_velocity_ros = ned_to_enu_vector(input_velocity);
                    break;

                case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD:
                case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD:
                    linear_velocity_ros = frd_to_flu_vector(input_velocity);
                    break;

                default:
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "velocity_frame desconocido: %u. "
                        "Se publica velocidad lineal cero.",
                        static_cast<unsigned>(msg->velocity_frame));
                    break;
            }
        }

        Eigen::Vector3d angular_velocity_ros = Eigen::Vector3d::Zero();

        if (is_finite3(msg->angular_velocity)) {
            const Eigen::Vector3d angular_velocity_frd(
                static_cast<double>(msg->angular_velocity[0]),
                static_cast<double>(msg->angular_velocity[1]),
                static_cast<double>(msg->angular_velocity[2]));

            angular_velocity_ros = frd_to_flu_vector(angular_velocity_frd);
        }

        const builtin_interfaces::msg::Time stamp = this->now();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_frame_;

        transform.transform.translation.x = position_ros.x();
        transform.transform.translation.y = position_ros.y();
        transform.transform.translation.z = position_ros.z();

        transform.transform.rotation.w = orientation_ros.w();
        transform.transform.rotation.x = orientation_ros.x();
        transform.transform.rotation.y = orientation_ros.y();
        transform.transform.rotation.z = orientation_ros.z();

        tf_broadcaster_->sendTransform(transform);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        odom_msg.pose.pose.position.x = position_ros.x();
        odom_msg.pose.pose.position.y = position_ros.y();
        odom_msg.pose.pose.position.z = position_ros.z();

        odom_msg.pose.pose.orientation.w = orientation_ros.w();
        odom_msg.pose.pose.orientation.x = orientation_ros.x();
        odom_msg.pose.pose.orientation.y = orientation_ros.y();
        odom_msg.pose.pose.orientation.z = orientation_ros.z();

        odom_msg.twist.twist.linear.x = linear_velocity_ros.x();
        odom_msg.twist.twist.linear.y = linear_velocity_ros.y();
        odom_msg.twist.twist.linear.z = linear_velocity_ros.z();

        odom_msg.twist.twist.angular.x = angular_velocity_ros.x();
        odom_msg.twist.twist.angular.y = angular_velocity_ros.y();
        odom_msg.twist.twist.angular.z = angular_velocity_ros.z();

        odom_pub_->publish(odom_msg);
    }

    void reset_service_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!current_odom_msg_) {
            response->success = false;
            response->message = "No hay odometria disponible todavia";

            RCLCPP_WARN(
                this->get_logger(),
                "[%s] Reset solicitado sin haber recibido odometria.",
                ns_.c_str());
            return;
        }

        if (!set_origin_from_msg(current_odom_msg_)) {
            response->success = false;
            response->message = "No se pudo fijar el nuevo origen";
            return;
        }

        response->success = true;
        response->message = "Odometria reseteada";

        RCLCPP_INFO(
            this->get_logger(),
            "[%s] Nuevo origen PX4: (%.3f, %.3f, %.3f), pose_frame=%u",
            ns_.c_str(),
            origin_pose_x_,
            origin_pose_y_,
            origin_pose_z_,
            static_cast<unsigned>(origin_pose_frame_));
    }

    bool set_origin_from_msg(
        const px4_msgs::msg::VehicleOdometry::SharedPtr & msg)
    {
        if (!msg || !is_finite3(msg->position)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Mensaje de odometria nulo o no finito al fijar el origen.");
            return false;
        }

        origin_pose_x_ = static_cast<double>(msg->position[0]);
        origin_pose_y_ = static_cast<double>(msg->position[1]);
        origin_pose_z_ = static_cast<double>(msg->position[2]);
        origin_pose_frame_ = msg->pose_frame;
        origin_is_set_ = true;
        return true;
    }

    std::string ns_;
    int id_{1};

    std::string vehicle_odometry_topic_;
    std::string odom_topic_;
    std::string reset_service_name_;

    std::string odom_frame_;
    std::string base_frame_;

    bool origin_is_set_{false};
    double origin_pose_x_{0.0};
    double origin_pose_y_{0.0};
    double origin_pose_z_{0.0};
    uint8_t origin_pose_frame_{
        px4_msgs::msg::VehicleOdometry::POSE_FRAME_UNKNOWN};

    px4_msgs::msg::VehicleOdometry::SharedPtr current_odom_msg_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneOdomBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
