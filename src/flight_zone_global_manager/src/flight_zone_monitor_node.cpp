#include <flight_zone_msgs/msg/flight_zone.hpp>
#include <flight_zone_msgs/msg/flight_zone_array.hpp>
#include <flight_zone_msgs/msg/vehicle_zone_status.hpp>
#include <flight_zone_msgs/msg/zone_containment.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace flight_zone_global_manager
{

using FlightZone = flight_zone_msgs::msg::FlightZone;
using FlightZoneArray = flight_zone_msgs::msg::FlightZoneArray;
using VehicleZoneStatus = flight_zone_msgs::msg::VehicleZoneStatus;
using ZoneContainment = flight_zone_msgs::msg::ZoneContainment;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

constexpr double kNumericalEpsilon = 1.0e-10;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Triangle
{
  Vec3 a;
  Vec3 b;
  Vec3 c;
};

struct PreparedZone
{
  FlightZone zone;
  std::vector<Triangle> triangles;
  bool valid{false};
  std::string error;
};

enum class PointRelation : uint8_t
{
  Outside,
  Inside,
  Boundary,
  Invalid
};

struct DroneContext
{
  std::string vehicle_id;
  std::string flight_zone_id;
  std::string namespace_path;
  std::string odom_topic;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
  rclcpp::Publisher<VehicleZoneStatus>::SharedPtr status_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inside_publisher;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher;

  uint8_t last_state{std::numeric_limits<uint8_t>::max()};
};

class FlightZoneMonitorNode : public rclcpp::Node
{
public:
  FlightZoneMonitorNode()
  : Node("flight_zone_monitor_node")
  {
    zone_topic_ = declare_parameter<std::string>("zone_topic", "/flight_zones");
    tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
    uas_prefix_ = declare_parameter<std::string>("uas_prefix", "ua_");
    odom_frame_suffix_ = declare_parameter<std::string>("odom_frame_suffix", "odom");
    base_frame_suffix_ = declare_parameter<std::string>("base_frame_suffix", "base_link");
    odom_topic_suffix_ = declare_parameter<std::string>("odom_topic_suffix", "odom");

    // Deliberately fixed across the A-space architecture.
    status_topic_suffix_ = "zone_status";

    inside_topic_suffix_ =
      declare_parameter<std::string>("inside_topic_suffix", "inside_flight_zone");
    marker_topic_suffix_ =
      declare_parameter<std::string>("marker_topic_suffix", "zone_status/markers");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.10);
    boundary_tolerance_m_ = declare_parameter<double>("boundary_tolerance_m", 0.03);
    boundary_is_inside_ = declare_parameter<bool>("boundary_is_inside", true);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    marker_sphere_diameter_m_ =
      declare_parameter<double>("marker_sphere_diameter_m", 0.45);
    marker_text_height_m_ = declare_parameter<double>("marker_text_height_m", 0.45);
    marker_text_offset_z_m_ = declare_parameter<double>("marker_text_offset_z_m", 0.75);

    normalise_name(odom_frame_suffix_);
    normalise_name(base_frame_suffix_);
    normalise_name(odom_topic_suffix_);
    normalise_name(inside_topic_suffix_);
    normalise_name(marker_topic_suffix_);

    if (target_frame_ != "map") {
      throw std::runtime_error("target_frame must be exactly 'map' in this implementation");
    }
    if (
      odom_frame_suffix_.empty() ||
      base_frame_suffix_.empty() ||
      odom_topic_suffix_.empty() ||
      inside_topic_suffix_.empty() ||
      marker_topic_suffix_.empty())
    {
      throw std::runtime_error("Frame/topic suffixes must not be empty");
    }
    if (tf_timeout_s_ < 0.0 || boundary_tolerance_m_ < 0.0 ||
      marker_sphere_diameter_m_ <= 0.0 || marker_text_height_m_ <= 0.0)
    {
      throw std::runtime_error("Invalid negative or zero monitor parameter");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto zone_qos = rclcpp::QoS(rclcpp::KeepLast(20));
    zone_qos.reliable();
    zone_qos.transient_local();

    zone_subscription_ = create_subscription<FlightZoneArray>(
      zone_topic_, zone_qos,
      std::bind(&FlightZoneMonitorNode::zone_callback, this, std::placeholders::_1));

    // /tf is used for global UAS discovery. Keep the standard dynamic-TF QoS.
    auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    tf_qos.best_effort();
    tf_qos.durability_volatile();

    tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      tf_topic_, tf_qos,
      std::bind(&FlightZoneMonitorNode::tf_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Global flight-zone monitor ready: zones='%s', discovery='%s', target='%s'",
      zone_topic_.c_str(), tf_topic_.c_str(), target_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Expected global TF pattern: <flight_zone>/<uas>/%s -> <flight_zone>/<uas>/%s",
      odom_frame_suffix_.c_str(), base_frame_suffix_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Per-UAS status output is fixed to /<flight_zone>/<uas>/%s",
      status_topic_suffix_.c_str());
  }

private:
  static void normalise_name(std::string & value)
  {
    while (!value.empty() && value.front() == '/') {
      value.erase(value.begin());
    }
    while (!value.empty() && value.back() == '/') {
      value.pop_back();
    }
  }

  static bool ends_with(const std::string & value, const std::string & suffix)
  {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  static std::string absolute_topic(
    const std::string & namespace_path,
    const std::string & suffix)
  {
    return "/" + namespace_path + "/" + suffix;
  }

  static Vec3 to_vec3(const geometry_msgs::msg::Point & point)
  {
    return Vec3{point.x, point.y, point.z};
  }

  static Vec3 add(const Vec3 & a, const Vec3 & b)
  {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
  }

  static Vec3 subtract(const Vec3 & a, const Vec3 & b)
  {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
  }

  static Vec3 multiply(const Vec3 & vector, const double scalar)
  {
    return Vec3{vector.x * scalar, vector.y * scalar, vector.z * scalar};
  }

  static double dot(const Vec3 & a, const Vec3 & b)
  {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  static Vec3 cross(const Vec3 & a, const Vec3 & b)
  {
    return Vec3{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x};
  }

  static double squared_norm(const Vec3 & vector)
  {
    return dot(vector, vector);
  }

  static Vec3 normalized(const Vec3 & vector)
  {
    const double norm = std::sqrt(squared_norm(vector));
    if (norm <= kNumericalEpsilon) {
      return Vec3{};
    }
    return multiply(vector, 1.0 / norm);
  }

  static double squared_distance(const Vec3 & a, const Vec3 & b)
  {
    return squared_norm(subtract(a, b));
  }

  // Squared point-to-triangle distance based on the Voronoi-region method
  // described in Real-Time Collision Detection (Christer Ericson).
  static double squared_distance_to_triangle(const Vec3 & p, const Triangle & triangle)
  {
    const Vec3 ab = subtract(triangle.b, triangle.a);
    const Vec3 ac = subtract(triangle.c, triangle.a);
    const Vec3 ap = subtract(p, triangle.a);
    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
      return squared_distance(p, triangle.a);
    }

    const Vec3 bp = subtract(p, triangle.b);
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
      return squared_distance(p, triangle.b);
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
      const double v = d1 / (d1 - d3);
      const Vec3 projection = add(triangle.a, multiply(ab, v));
      return squared_distance(p, projection);
    }

    const Vec3 cp = subtract(p, triangle.c);
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
      return squared_distance(p, triangle.c);
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
      const double w = d2 / (d2 - d6);
      const Vec3 projection = add(triangle.a, multiply(ac, w));
      return squared_distance(p, projection);
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
      const Vec3 bc = subtract(triangle.c, triangle.b);
      const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      const Vec3 projection = add(triangle.b, multiply(bc, w));
      return squared_distance(p, projection);
    }

    const double denominator = 1.0 / (va + vb + vc);
    const double v = vb * denominator;
    const double w = vc * denominator;
    const Vec3 projection = add(
      triangle.a,
      add(multiply(ab, v), multiply(ac, w)));
    return squared_distance(p, projection);
  }

  static bool ray_triangle_intersection(
    const Vec3 & origin,
    const Vec3 & direction,
    const Triangle & triangle,
    double & distance)
  {
    const Vec3 edge_1 = subtract(triangle.b, triangle.a);
    const Vec3 edge_2 = subtract(triangle.c, triangle.a);
    const Vec3 p_vector = cross(direction, edge_2);
    const double determinant = dot(edge_1, p_vector);

    if (std::abs(determinant) <= kNumericalEpsilon) {
      return false;
    }

    const double inverse_determinant = 1.0 / determinant;
    const Vec3 t_vector = subtract(origin, triangle.a);
    const double u = dot(t_vector, p_vector) * inverse_determinant;
    if (u < -kNumericalEpsilon || u > 1.0 + kNumericalEpsilon) {
      return false;
    }

    const Vec3 q_vector = cross(t_vector, edge_1);
    const double v = dot(direction, q_vector) * inverse_determinant;
    if (v < -kNumericalEpsilon || u + v > 1.0 + kNumericalEpsilon) {
      return false;
    }

    distance = dot(edge_2, q_vector) * inverse_determinant;
    return distance > kNumericalEpsilon;
  }

  static std::size_t unique_ray_intersection_count(
    const Vec3 & point,
    const Vec3 & ray_direction,
    const std::vector<Triangle> & triangles)
  {
    std::vector<double> distances;
    distances.reserve(triangles.size());

    for (const auto & triangle : triangles) {
      double distance = 0.0;
      if (ray_triangle_intersection(point, ray_direction, triangle, distance)) {
        distances.push_back(distance);
      }
    }

    if (distances.empty()) {
      return 0U;
    }

    std::sort(distances.begin(), distances.end());
    std::size_t unique_count = 1U;
    double previous = distances.front();

    for (std::size_t i = 1U; i < distances.size(); ++i) {
      const double tolerance = std::max(1.0e-8, std::abs(previous) * 1.0e-8);
      if (std::abs(distances[i] - previous) > tolerance) {
        ++unique_count;
        previous = distances[i];
      }
    }
    return unique_count;
  }

  PointRelation classify_point(
    const geometry_msgs::msg::Point & point_message,
    const PreparedZone & prepared_zone) const
  {
    if (!prepared_zone.valid || prepared_zone.triangles.empty()) {
      return PointRelation::Invalid;
    }

    const Vec3 point = to_vec3(point_message);
    const double boundary_tolerance_squared =
      boundary_tolerance_m_ * boundary_tolerance_m_;

    for (const auto & triangle : prepared_zone.triangles) {
      if (squared_distance_to_triangle(point, triangle) <= boundary_tolerance_squared) {
        return PointRelation::Boundary;
      }
    }

    const std::array<Vec3, 3> rays{
      normalized(Vec3{1.0, 0.371390676, 0.694245231}),
      normalized(Vec3{0.217391304, 1.0, 0.539682540}),
      normalized(Vec3{0.483870968, 0.290322581, 1.0})};

    std::size_t inside_votes = 0U;
    for (const auto & ray : rays) {
      const auto intersections =
        unique_ray_intersection_count(point, ray, prepared_zone.triangles);
      if ((intersections % 2U) == 1U) {
        ++inside_votes;
      }
    }

    return inside_votes >= 2U ? PointRelation::Inside : PointRelation::Outside;
  }

  static PreparedZone prepare_zone(const FlightZone & zone)
  {
    PreparedZone prepared;
    prepared.zone = zone;

    if (zone.header.frame_id != "map") {
      prepared.error = "zone frame is not map";
      return prepared;
    }
    if (zone.vertices.size() < 4U || zone.faces.size() < 4U) {
      prepared.error = "polyhedron requires at least four vertices and four faces";
      return prepared;
    }

    for (std::size_t face_index = 0U; face_index < zone.faces.size(); ++face_index) {
      const auto & face = zone.faces[face_index];
      if (face.vertex_indices.size() < 3U) {
        prepared.error = "face " + std::to_string(face_index) + " has fewer than three vertices";
        prepared.triangles.clear();
        return prepared;
      }

      const uint32_t first_index = face.vertex_indices.front();
      if (first_index >= zone.vertices.size()) {
        prepared.error = "face vertex index outside vertices array";
        prepared.triangles.clear();
        return prepared;
      }

      for (std::size_t i = 1U; i + 1U < face.vertex_indices.size(); ++i) {
        const uint32_t second_index = face.vertex_indices[i];
        const uint32_t third_index = face.vertex_indices[i + 1U];
        if (second_index >= zone.vertices.size() || third_index >= zone.vertices.size()) {
          prepared.error = "face vertex index outside vertices array";
          prepared.triangles.clear();
          return prepared;
        }

        Triangle triangle{
          to_vec3(zone.vertices[first_index]),
          to_vec3(zone.vertices[second_index]),
          to_vec3(zone.vertices[third_index])};

        const Vec3 normal = cross(
          subtract(triangle.b, triangle.a),
          subtract(triangle.c, triangle.a));
        if (squared_norm(normal) <= kNumericalEpsilon) {
          prepared.error = "degenerate triangle found while triangulating a face";
          prepared.triangles.clear();
          return prepared;
        }
        prepared.triangles.push_back(triangle);
      }
    }

    if (prepared.triangles.size() < 4U) {
      prepared.error = "polyhedron triangulation produced too few triangles";
      return prepared;
    }

    prepared.valid = true;
    return prepared;
  }


  void zone_callback(const FlightZoneArray::SharedPtr message)
  {
    std::map<std::string, PreparedZone> prepared_zones;
    std::size_t valid_count = 0U;
    std::size_t invalid_count = 0U;

    for (const auto & zone : message->zones) {
      if (zone.zone_id.empty()) {
        RCLCPP_ERROR(get_logger(), "Ignoring flight zone with empty zone_id");
        ++invalid_count;
        continue;
      }

      if (prepared_zones.find(zone.zone_id) != prepared_zones.end()) {
        RCLCPP_ERROR(
          get_logger(), "Ignoring duplicated flight zone '%s'", zone.zone_id.c_str());
        ++invalid_count;
        continue;
      }

      PreparedZone prepared = prepare_zone(zone);

      if (!zone.enabled) {
        prepared.valid = false;
        prepared.error = "assigned flight zone is disabled";
      } else if (zone.zone_type != FlightZone::INCLUSION) {
        // In this phase, the first namespace segment selects the allowed
        // inclusion volume. Exclusion/warning volumes can be evaluated by a
        // later global deconfliction layer.
        prepared.valid = false;
        prepared.error = "assigned flight zone must be of type inclusion";
      }

      if (prepared.valid) {
        ++valid_count;
      } else {
        ++invalid_count;
        RCLCPP_ERROR(
          get_logger(), "Flight zone '%s' is invalid for UAS assignment: %s",
          zone.zone_id.c_str(), prepared.error.c_str());
      }

      prepared_zones.emplace(zone.zone_id, std::move(prepared));
    }

    {
      std::lock_guard<std::mutex> lock(zone_mutex_);
      prepared_zones_ = std::move(prepared_zones);
      zones_received_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "Global flight-zone set updated: total=%zu valid=%zu invalid=%zu",
      message->zones.size(), valid_count, invalid_count);
  }

  static bool split_drone_namespace(
    const std::string & drone_namespace,
    std::string & flight_zone_id,
    std::string & vehicle_id)
  {
    const std::size_t slash = drone_namespace.find('/');

    if (
      slash == std::string::npos ||
      slash == 0U ||
      slash + 1U >= drone_namespace.size() ||
      drone_namespace.find('/', slash + 1U) != std::string::npos)
    {
      return false;
    }

    flight_zone_id = drone_namespace.substr(0U, slash);
    vehicle_id = drone_namespace.substr(slash + 1U);
    return !flight_zone_id.empty() && !vehicle_id.empty();
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr message)
  {
    const std::string odom_suffix = "/" + odom_frame_suffix_;

    for (const auto & transform : message->transforms) {
      std::string parent = transform.header.frame_id;
      std::string child = transform.child_frame_id;
      normalise_name(parent);
      normalise_name(child);

      if (!ends_with(parent, odom_suffix)) {
        continue;
      }

      const std::string drone_namespace =
        parent.substr(0U, parent.size() - odom_suffix.size());
      const std::string expected_child =
        drone_namespace + "/" + base_frame_suffix_;

      if (child != expected_child) {
        continue;
      }

      std::string flight_zone_id;
      std::string vehicle_id;
      if (!split_drone_namespace(drone_namespace, flight_zone_id, vehicle_id)) {
        continue;
      }

      if (!uas_prefix_.empty() && vehicle_id.rfind(uas_prefix_, 0U) != 0U) {
        continue;
      }

      discover_drone(flight_zone_id, vehicle_id, drone_namespace);
    }
  }

  void discover_drone(
    const std::string & flight_zone_id,
    const std::string & vehicle_id,
    const std::string & drone_namespace)
  {
    std::lock_guard<std::mutex> lock(drones_mutex_);

    if (drones_.find(drone_namespace) != drones_.end()) {
      return;
    }

    DroneContext context;
    context.vehicle_id = vehicle_id;
    context.flight_zone_id = flight_zone_id;
    context.namespace_path = drone_namespace;
    context.odom_topic = absolute_topic(drone_namespace, odom_topic_suffix_);

    context.status_publisher = create_publisher<VehicleZoneStatus>(
      absolute_topic(drone_namespace, status_topic_suffix_), 10);
    context.inside_publisher = create_publisher<std_msgs::msg::Bool>(
      absolute_topic(drone_namespace, inside_topic_suffix_), 10);
    context.marker_publisher = create_publisher<MarkerArray>(
      absolute_topic(drone_namespace, marker_topic_suffix_), 10);

    context.odom_subscription = create_subscription<nav_msgs::msg::Odometry>(
      context.odom_topic,
      rclcpp::SensorDataQoS(),
      [this, drone_namespace](const nav_msgs::msg::Odometry::SharedPtr odometry) {
        odom_callback(drone_namespace, odometry);
      });

    drones_.emplace(drone_namespace, std::move(context));

    RCLCPP_INFO(
      get_logger(),
      "Discovered active UAS '/%s' | assigned zone='%s' | odom='%s' | status='%s'",
      drone_namespace.c_str(),
      flight_zone_id.c_str(),
      absolute_topic(drone_namespace, odom_topic_suffix_).c_str(),
      absolute_topic(drone_namespace, status_topic_suffix_).c_str());
  }

  bool transform_odometry_position(
    const nav_msgs::msg::Odometry & odometry,
    const std::string & drone_namespace,
    geometry_msgs::msg::Point & map_position,
    std::string & source_frame,
    std::string & error)
  {
    source_frame = odometry.header.frame_id.empty() ?
      drone_namespace + "/" + odom_frame_suffix_ : odometry.header.frame_id;
    normalise_name(source_frame);

    geometry_msgs::msg::PointStamped source_point;
    source_point.header = odometry.header;
    source_point.header.frame_id = source_frame;
    source_point.point = odometry.pose.pose.position;

    if (source_frame == target_frame_) {
      map_position = source_point.point;
      return true;
    }

    try {
      const auto transformed = tf_buffer_->transform(
        source_point, target_frame_, tf2::durationFromSec(tf_timeout_s_));
      map_position = transformed.point;
      return true;
    } catch (const tf2::TransformException & exception) {
      error = exception.what();
      return false;
    }
  }

  static uint8_t containment_state(const PointRelation relation)
  {
    switch (relation) {
      case PointRelation::Inside:
        return ZoneContainment::INSIDE;
      case PointRelation::Boundary:
        return ZoneContainment::ON_BOUNDARY;
      case PointRelation::Invalid:
        return ZoneContainment::INVALID_GEOMETRY;
      case PointRelation::Outside:
      default:
        return ZoneContainment::OUTSIDE;
    }
  }

  static std::string relation_text(const PointRelation relation)
  {
    switch (relation) {
      case PointRelation::Inside:
        return "inside";
      case PointRelation::Boundary:
        return "on boundary";
      case PointRelation::Invalid:
        return "invalid geometry";
      case PointRelation::Outside:
      default:
        return "outside";
    }
  }

  static std::string overall_state_text(const uint8_t state)
  {
    switch (state) {
      case VehicleZoneStatus::INSIDE:
        return "INSIDE";
      case VehicleZoneStatus::OUTSIDE:
        return "OUTSIDE";
      case VehicleZoneStatus::ON_BOUNDARY:
        return "ON BOUNDARY";
      case VehicleZoneStatus::EXCLUSION_VIOLATION:
        return "EXCLUSION VIOLATION";
      case VehicleZoneStatus::TRANSFORM_ERROR:
        return "TF ERROR";
      case VehicleZoneStatus::INVALID_GEOMETRY:
        return "INVALID GEOMETRY";
      case VehicleZoneStatus::NO_ZONE:
      default:
        return "NO ZONE";
    }
  }

  static std_msgs::msg::ColorRGBA make_color(
    const float r, const float g, const float b, const float a)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  static std_msgs::msg::ColorRGBA state_color(const uint8_t state)
  {
    switch (state) {
      case VehicleZoneStatus::INSIDE:
        return make_color(0.05F, 0.90F, 0.10F, 1.0F);
      case VehicleZoneStatus::ON_BOUNDARY:
        return make_color(1.00F, 0.75F, 0.00F, 1.0F);
      case VehicleZoneStatus::OUTSIDE:
      case VehicleZoneStatus::EXCLUSION_VIOLATION:
        return make_color(1.00F, 0.05F, 0.05F, 1.0F);
      default:
        return make_color(0.55F, 0.55F, 0.55F, 1.0F);
    }
  }

  void publish_transform_error(
    const std::string & drone_namespace,
    const std::string & flight_zone_id,
    const std::string & vehicle_id,
    const nav_msgs::msg::Odometry & odometry,
    const std::string & source_frame,
    const std::string & error,
    const rclcpp::Publisher<VehicleZoneStatus>::SharedPtr & status_publisher,
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & inside_publisher)
  {
    VehicleZoneStatus status;
    status.header = odometry.header;
    status.header.frame_id = target_frame_;
    status.vehicle_id = vehicle_id;
    status.state = VehicleZoneStatus::TRANSFORM_ERROR;
    status.inside_allowed_volume = false;
    status.inside_exclusion_zone = false;
    status.source_frame = source_frame;
    status.detail =
      "Could not transform odometry position to map for flight zone '" +
      flight_zone_id + "': " + error;

    status_publisher->publish(status);

    std_msgs::msg::Bool inside;
    inside.data = false;
    inside_publisher->publish(inside);

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[%s] Cannot transform odometry from '%s' to '%s': %s",
      drone_namespace.c_str(), source_frame.c_str(), target_frame_.c_str(), error.c_str());
  }

  void odom_callback(
    const std::string & drone_namespace,
    const nav_msgs::msg::Odometry::SharedPtr odometry)
  {
    std::string vehicle_id;
    std::string flight_zone_id;
    rclcpp::Publisher<VehicleZoneStatus>::SharedPtr status_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inside_publisher;
    rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher;

    {
      std::lock_guard<std::mutex> lock(drones_mutex_);
      const auto iterator = drones_.find(drone_namespace);
      if (iterator == drones_.end()) {
        return;
      }
      vehicle_id = iterator->second.vehicle_id;
      flight_zone_id = iterator->second.flight_zone_id;
      status_publisher = iterator->second.status_publisher;
      inside_publisher = iterator->second.inside_publisher;
      marker_publisher = iterator->second.marker_publisher;
    }

    geometry_msgs::msg::Point map_position;
    std::string source_frame;
    std::string transform_error;

    if (!transform_odometry_position(
        *odometry, drone_namespace, map_position, source_frame, transform_error))
    {
      publish_transform_error(
        drone_namespace, flight_zone_id, vehicle_id, *odometry, source_frame,
        transform_error, status_publisher, inside_publisher);
      return;
    }

    PreparedZone zone_copy;
    bool zones_received = false;
    bool assigned_zone_found = false;
    {
      std::lock_guard<std::mutex> lock(zone_mutex_);
      zones_received = zones_received_;
      const auto zone_iterator = prepared_zones_.find(flight_zone_id);
      if (zone_iterator != prepared_zones_.end()) {
        zone_copy = zone_iterator->second;
        assigned_zone_found = true;
      }
    }

    VehicleZoneStatus status;
    status.header = odometry->header;
    status.header.frame_id = target_frame_;
    status.vehicle_id = vehicle_id;
    status.position = map_position;
    status.source_frame = source_frame;
    status.inside_exclusion_zone = false;

    if (!zones_received) {
      status.state = VehicleZoneStatus::NO_ZONE;
      status.inside_allowed_volume = false;
      status.detail = "Global flight-zone set has not been received yet";
    } else if (!assigned_zone_found) {
      status.state = VehicleZoneStatus::NO_ZONE;
      status.inside_allowed_volume = false;
      status.detail =
        "No flight zone with zone_id '" + flight_zone_id +
        "' exists for namespace '/" + drone_namespace + "'";
    } else if (!zone_copy.valid) {
      status.state = VehicleZoneStatus::INVALID_GEOMETRY;
      status.inside_allowed_volume = false;
      status.detail = zone_copy.error;

      ZoneContainment containment;
      containment.zone_id = flight_zone_id;
      containment.zone_type = zone_copy.zone.zone_type;
      containment.state = ZoneContainment::INVALID_GEOMETRY;
      containment.contains_position = false;
      containment.detail = zone_copy.error;
      status.zones.push_back(containment);
    } else {
      const auto relation = classify_point(map_position, zone_copy);

      ZoneContainment containment;
      containment.zone_id = zone_copy.zone.zone_id;
      containment.zone_type = zone_copy.zone.zone_type;
      containment.state = containment_state(relation);
      containment.contains_position =
        relation == PointRelation::Inside || relation == PointRelation::Boundary;
      containment.detail = relation_text(relation);
      status.zones.push_back(containment);

      if (relation == PointRelation::Inside) {
        status.state = VehicleZoneStatus::INSIDE;
        status.inside_allowed_volume = true;
        status.detail =
          "Vehicle is inside assigned flight zone '" + flight_zone_id + "'";
      } else if (relation == PointRelation::Boundary) {
        status.state = VehicleZoneStatus::ON_BOUNDARY;
        status.inside_allowed_volume = boundary_is_inside_;
        status.detail =
          "Vehicle is on the boundary of assigned flight zone '" +
          flight_zone_id + "'";
      } else if (relation == PointRelation::Invalid) {
        status.state = VehicleZoneStatus::INVALID_GEOMETRY;
        status.inside_allowed_volume = false;
        status.detail = zone_copy.error;
      } else {
        status.state = VehicleZoneStatus::OUTSIDE;
        status.inside_allowed_volume = false;
        status.detail =
          "Vehicle is outside assigned flight zone '" + flight_zone_id + "'";
      }
    }

    status_publisher->publish(status);

    std_msgs::msg::Bool inside_message;
    inside_message.data = status.inside_allowed_volume;
    inside_publisher->publish(inside_message);

    if (publish_markers_) {
      publish_status_markers(
        status, flight_zone_id, vehicle_id, marker_publisher);
    }

    bool state_changed = false;
    {
      std::lock_guard<std::mutex> lock(drones_mutex_);
      const auto iterator = drones_.find(drone_namespace);
      if (iterator != drones_.end() && iterator->second.last_state != status.state) {
        iterator->second.last_state = status.state;
        state_changed = true;
      }
    }

    if (state_changed) {
      RCLCPP_INFO(
        get_logger(), "%s/%s: %s at [%.3f, %.3f, %.3f] in map",
        flight_zone_id.c_str(), vehicle_id.c_str(),
        overall_state_text(status.state).c_str(),
        map_position.x, map_position.y, map_position.z);
    }
  }

  void publish_status_markers(
    const VehicleZoneStatus & status,
    const std::string & flight_zone_id,
    const std::string & vehicle_id,
    const rclcpp::Publisher<MarkerArray>::SharedPtr & marker_publisher)
  {
    MarkerArray markers;
    const auto color = state_color(status.state);
    const std::string marker_namespace =
      "zone_status/" + flight_zone_id + "/" + vehicle_id;

    Marker sphere;
    sphere.header = status.header;
    sphere.ns = marker_namespace;
    sphere.id = 0;
    sphere.type = Marker::SPHERE;
    sphere.action = Marker::ADD;
    sphere.pose.position = status.position;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = marker_sphere_diameter_m_;
    sphere.scale.y = marker_sphere_diameter_m_;
    sphere.scale.z = marker_sphere_diameter_m_;
    sphere.color = color;
    markers.markers.push_back(sphere);

    Marker text;
    text.header = status.header;
    text.ns = marker_namespace;
    text.id = 1;
    text.type = Marker::TEXT_VIEW_FACING;
    text.action = Marker::ADD;
    text.pose.position = status.position;
    text.pose.position.z += marker_text_offset_z_m_;
    text.pose.orientation.w = 1.0;
    text.scale.z = marker_text_height_m_;
    text.color = color;

    std::ostringstream stream;
    stream << flight_zone_id << "/" << vehicle_id << "\n"
           << overall_state_text(status.state);
    text.text = stream.str();
    markers.markers.push_back(text);

    marker_publisher->publish(markers);
  }

  std::string zone_topic_;
  std::string tf_topic_;
  std::string uas_prefix_;
  std::string odom_frame_suffix_;
  std::string base_frame_suffix_;
  std::string odom_topic_suffix_;
  std::string status_topic_suffix_{"zone_status"};
  std::string inside_topic_suffix_;
  std::string marker_topic_suffix_;
  std::string target_frame_;

  double tf_timeout_s_{0.10};
  double boundary_tolerance_m_{0.03};
  bool boundary_is_inside_{true};
  bool publish_markers_{true};
  double marker_sphere_diameter_m_{0.45};
  double marker_text_height_m_{0.45};
  double marker_text_offset_z_m_{0.75};

  std::mutex zone_mutex_;
  std::map<std::string, PreparedZone> prepared_zones_;
  bool zones_received_{false};

  std::mutex drones_mutex_;
  std::map<std::string, DroneContext> drones_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<FlightZoneArray>::SharedPtr zone_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;

};

}  // namespace flight_zone_global_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<flight_zone_global_manager::FlightZoneMonitorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("flight_zone_monitor_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}