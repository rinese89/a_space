#include <rclcpp/rclcpp.hpp>

#include <flight_zone_msgs/msg/flight_zone.hpp>
#include <flight_zone_msgs/msg/flight_zone_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace static_trajectory_manager
{

using FlightZone = flight_zone_msgs::msg::FlightZone;
using FlightZoneArray = flight_zone_msgs::msg::FlightZoneArray;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using StaticTrajectory = static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using TrajectorySegment = static_trajectory_manager::msg::TrajectorySegment;

constexpr double kEpsilon = 1.0e-10;
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

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

enum class PointRelation : uint8_t
{
  Outside,
  Inside,
  Boundary,
  Invalid
};

struct PreparedZone
{
  FlightZone zone;
  std::vector<Triangle> triangles;
  Vec3 min_corner;
  Vec3 max_corner;
  bool valid{false};
  std::string error;
};

struct SegmentConfig
{
  std::vector<Vec3> points;
};

struct TrajectoryConfig
{
  int32_t priority{0};
  std::string trajectory_id;
  uint32_t ua_id{0U};
  std::string uas_namespace;
  std::string flight_zone_id;
  std::string frame_id{"map"};
  std::string action_name;

  SegmentConfig takeoff;
  std::vector<SegmentConfig> missions;
  SegmentConfig landing;

  double goal_tolerance{0.12};
  double slowdown_radius{0.20};
  // Partial repetition: number of repetitions of the ordered mission
  // collection inside one complete takeoff -> mission[] -> landing operation.
  //
  // mission[] elements are independent polylines. No implicit geometric edge
  // exists between missions[i].back() and missions[i+1].front().
  uint32_t repetitions{1U};

  // Total repetition: period in seconds between complete-operation starts.
  // 0.0 disables total repetition.
  double operation_frequency{0.0};

  // Number of complete takeoff -> mission[] -> landing operations.
  // For periodic trajectories:
  //   0 => unlimited complete repetitions.
  //   N => exactly N complete operations.
  // For non-periodic trajectories this value is always 1.
  uint32_t total_repetitions{1U};

  // Runtime-only counters. They are deliberately not part of StaticTrajectory.msg.
  uint32_t completed_total_repetitions{0U};
  bool total_repetition_exhausted{false};

  double average_speed_mps{1.0};

  int64_t operation_start_ns{0};
  int64_t operation_end_ns{0};
  double estimated_distance_m{0.0};
  double estimated_duration_s{0.0};
  std::size_t input_index{0U};

  // Used to reject duplicated/stale transient-local temporal adjustments.
  std::set<int64_t> processed_adjustment_starts;

  // Runtime-only DDS publisher identity of the source that installed the
  // current cropped/supervision geometry through /adjusted_trajectories.
  // Re-publications from this same publisher are geometry-state heartbeats and
  // must NOT be interpreted as completed executions. A later adjustment from
  // another publisher (trajectory_endtime_adjustment_node) is processed with
  // the normal temporal/periodic logic, even though the geometry now matches.
  std::string geometry_update_publisher_gid;
};

static Vec3 subtract(const Vec3 & a, const Vec3 & b)
{
  return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 add(const Vec3 & a, const Vec3 & b)
{
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 multiply(const Vec3 & a, double scalar)
{
  return Vec3{a.x * scalar, a.y * scalar, a.z * scalar};
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

static double squared_norm(const Vec3 & a)
{
  return dot(a, a);
}

static double norm(const Vec3 & a)
{
  return std::sqrt(squared_norm(a));
}

static Vec3 normalized(const Vec3 & a)
{
  const double value = norm(a);
  if (value <= kEpsilon) {
    return Vec3{};
  }
  return multiply(a, 1.0 / value);
}

static Vec3 to_vec3(const geometry_msgs::msg::Point & point)
{
  return Vec3{point.x, point.y, point.z};
}

static bool finite(double value)
{
  return std::isfinite(value);
}

static builtin_interfaces::msg::Time ns_to_time(int64_t ns)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(ns / kNanosecondsPerSecond);
  int64_t remainder = ns % kNanosecondsPerSecond;
  if (remainder < 0) {
    --result.sec;
    remainder += kNanosecondsPerSecond;
  }
  result.nanosec = static_cast<uint32_t>(remainder);
  return result;
}

static int64_t time_to_ns(const builtin_interfaces::msg::Time & time)
{
  return static_cast<int64_t>(time.sec) * kNanosecondsPerSecond +
         static_cast<int64_t>(time.nanosec);
}

static int64_t seconds_to_ns(double seconds)
{
  if (!std::isfinite(seconds)) {
    throw std::runtime_error("Cannot convert non-finite seconds to nanoseconds");
  }

  const long double value =
    static_cast<long double>(seconds) *
    static_cast<long double>(kNanosecondsPerSecond);

  if (
    value > static_cast<long double>(std::numeric_limits<int64_t>::max()) ||
    value < static_cast<long double>(std::numeric_limits<int64_t>::min()))
  {
    throw std::runtime_error("Time value exceeds int64 nanosecond range");
  }

  return static_cast<int64_t>(std::llround(value));
}

static int64_t system_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

static int64_t parse_utc_iso8601_ns(const std::string & value)
{
  static const std::regex pattern(
    R"(^([0-9]{4})-([0-9]{2})-([0-9]{2})T([0-9]{2}):([0-9]{2}):([0-9]{2})(?:\.([0-9]{1,9}))?Z$)");

  std::smatch match;
  if (!std::regex_match(value, match, pattern)) {
    throw std::runtime_error(
      "UTC timestamp must use ISO-8601 form YYYY-MM-DDTHH:MM:SS[.nnnnnnnnn]Z: '" +
      value + "'");
  }

  std::tm utc{};
  utc.tm_year = std::stoi(match[1].str()) - 1900;
  utc.tm_mon = std::stoi(match[2].str()) - 1;
  utc.tm_mday = std::stoi(match[3].str());
  utc.tm_hour = std::stoi(match[4].str());
  utc.tm_min = std::stoi(match[5].str());
  utc.tm_sec = std::stoi(match[6].str());
  utc.tm_isdst = 0;

  if (utc.tm_mon < 0 || utc.tm_mon > 11 || utc.tm_mday < 1 || utc.tm_mday > 31 ||
    utc.tm_hour < 0 || utc.tm_hour > 23 || utc.tm_min < 0 || utc.tm_min > 59 ||
    utc.tm_sec < 0 || utc.tm_sec > 60)
  {
    throw std::runtime_error("UTC timestamp contains an out-of-range field: '" + value + "'");
  }

  const std::time_t seconds = timegm(&utc);
  if (seconds == static_cast<std::time_t>(-1)) {
    throw std::runtime_error("Could not convert UTC timestamp: '" + value + "'");
  }

  int64_t fractional_ns = 0;
  if (match[7].matched) {
    std::string fraction = match[7].str();
    while (fraction.size() < 9U) {
      fraction.push_back('0');
    }
    fractional_ns = std::stoll(fraction);
  }

  return static_cast<int64_t>(seconds) * kNanosecondsPerSecond + fractional_ns;
}

static std::string format_utc_iso8601(int64_t ns)
{
  std::time_t seconds = static_cast<std::time_t>(ns / kNanosecondsPerSecond);
  std::tm tm{};
  gmtime_r(&seconds, &tm);
  std::ostringstream stream;
  stream << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

static double polyline_length(const std::vector<Vec3> & points)
{
  double total = 0.0;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    total += norm(subtract(points[i], points[i - 1U]));
  }
  return total;
}

static double mission_collection_length(
  const std::vector<SegmentConfig> & missions)
{
  double total = 0.0;

  for (const auto & mission : missions) {
    total += polyline_length(mission.points);
  }

  return total;
}

static double operation_geometry_length(
  const TrajectoryConfig & trajectory)
{
  // Each phase/mission component is an independent polyline. In particular,
  // there is NO implicit edge:
  //   takeoff.back() -> mission[0].front()
  //   mission[i].back() -> mission[i+1].front()
  //   mission.back().back() -> landing.front()
  //
  // This is essential for supervised crops, where separate mission[] elements
  // explicitly represent disconnected collision-free pieces.
  return
    polyline_length(trajectory.takeoff.points) +
    static_cast<double>(trajectory.repetitions) *
    mission_collection_length(trajectory.missions) +
    polyline_length(trajectory.landing.points);
}

static const Vec3 * first_geometry_point(
  const TrajectoryConfig & trajectory)
{
  if (!trajectory.takeoff.points.empty()) {
    return &trajectory.takeoff.points.front();
  }

  for (const auto & mission : trajectory.missions) {
    if (!mission.points.empty()) {
      return &mission.points.front();
    }
  }

  if (!trajectory.landing.points.empty()) {
    return &trajectory.landing.points.front();
  }

  return nullptr;
}

class StaticTrajectoryManagerNode : public rclcpp::Node
{
public:
  StaticTrajectoryManagerNode()
  : Node("static_trajectory_manager_node")
  {
    config_file_ = declare_parameter<std::string>("config_file", "");
    flight_zones_topic_ = declare_parameter<std::string>("flight_zones_topic", "/flight_zones");
    requested_topic_ = declare_parameter<std::string>(
      "requested_static_trajectories_topic", "/requested_static_trajectories");
    unvalidated_topic_ = declare_parameter<std::string>(
      "unvalidated_trajectories_topic", "/unvalidated_trajectories");
    latest_topic_ = declare_parameter<std::string>(
      "latest_trajectories_topic", "/latest_trajectories");
    adjusted_topic_ = declare_parameter<std::string>(
      "adjusted_trajectories_topic", "/adjusted_trajectories");
    markers_topic_ = declare_parameter<std::string>(
      "requested_static_trajectories_markers_topic", "/requested_static_trajectories_markers");
    validation_period_ms_ = declare_parameter<int>("validation_period_ms", 1000);

    if (config_file_.empty()) {
      throw std::runtime_error("Parameter 'config_file' must contain the static trajectory YAML path");
    }
    if (validation_period_ms_ <= 0) {
      throw std::runtime_error("validation_period_ms must be greater than zero");
    }

    const std::array<std::pair<std::string, std::string>, 6> absolute_topics{{
      {"flight_zones_topic", flight_zones_topic_},
      {"requested_static_trajectories_topic", requested_topic_},
      {"unvalidated_trajectories_topic", unvalidated_topic_},
      {"latest_trajectories_topic", latest_topic_},
      {"adjusted_trajectories_topic", adjusted_topic_},
      {"requested_static_trajectories_markers_topic", markers_topic_}
    }};
    for (const auto & [name, topic] : absolute_topics) {
      if (topic.empty() || topic.front() != '/') {
        throw std::runtime_error(
                "Parameter '" + name + "' must contain an absolute ROS topic name");
      }
    }

    load_configuration();

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    state_qos.reliable();
    state_qos.transient_local();

    requested_publisher_ = create_publisher<StaticTrajectoryArray>(requested_topic_, state_qos);
    unvalidated_publisher_ = create_publisher<StaticTrajectoryArray>(unvalidated_topic_, state_qos);
    latest_publisher_ = create_publisher<StaticTrajectoryArray>(latest_topic_, state_qos);
    marker_publisher_ = create_publisher<MarkerArray>(markers_topic_, state_qos);

    auto zone_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    zone_qos.reliable();
    zone_qos.transient_local();

    flight_zones_subscription_ = create_subscription<FlightZoneArray>(
      flight_zones_topic_, zone_qos,
      std::bind(&StaticTrajectoryManagerNode::flight_zones_callback, this, std::placeholders::_1));

    adjusted_subscription_ = create_subscription<StaticTrajectoryArray>(
      adjusted_topic_, state_qos,
      std::bind(
        &StaticTrajectoryManagerNode::adjusted_trajectories_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    validation_timer_ = create_wall_timer(
      std::chrono::milliseconds(validation_period_ms_),
      std::bind(&StaticTrajectoryManagerNode::publish_valid_snapshot, this));

    RCLCPP_INFO(
      get_logger(),
      "Static trajectory manager ready | YAML='%s' | trajectories=%zu | FZ='%s' | "
      "requested='%s' | unvalidated='%s' | latest='%s' | adjusted='%s' | markers='%s'",
      config_file_.c_str(), trajectories_.size(), flight_zones_topic_.c_str(),
      requested_topic_.c_str(), unvalidated_topic_.c_str(), latest_topic_.c_str(),
      adjusted_topic_.c_str(), markers_topic_.c_str());
  }

private:
  template<typename T>
  static T required_value(const YAML::Node & node, const std::string & key, const std::string & context)
  {
    if (!node[key]) {
      throw std::runtime_error(context + ": missing field '" + key + "'");
    }
    return node[key].as<T>();
  }

  template<typename T>
  static T optional_value(
    const YAML::Node & node, const std::string & key, const T & fallback)
  {
    return node[key] ? node[key].as<T>() : fallback;
  }

  static std::vector<double> required_double_sequence(
    const YAML::Node & node, const std::string & key, const std::string & context)
  {
    if (!node[key] || !node[key].IsSequence() || node[key].size() == 0U) {
      throw std::runtime_error(context + ": field '" + key + "' must be a non-empty sequence");
    }

    std::vector<double> values;
    values.reserve(node[key].size());
    for (std::size_t i = 0U; i < node[key].size(); ++i) {
      const double value = node[key][i].as<double>();
      if (!finite(value)) {
        throw std::runtime_error(context + ": field '" + key + "' contains a non-finite value");
      }
      values.push_back(value);
    }
    return values;
  }

  static SegmentConfig parse_segment(
    const YAML::Node & node, const std::string & context, bool allow_height)
  {
    if (!node || !node.IsMap()) {
      throw std::runtime_error(context + " must be a YAML map");
    }

    const auto x = required_double_sequence(node, "x", context);
    const auto y = required_double_sequence(node, "y", context);
    if (x.size() != y.size() || x.size() < 2U) {
      throw std::runtime_error(context + ": x and y must have equal length and at least two points");
    }

    std::vector<double> z;
    if (node["z"]) {
      z = required_double_sequence(node, "z", context);
      if (z.size() != x.size()) {
        throw std::runtime_error(context + ": z must have the same length as x and y");
      }
    } else if (allow_height && node["height"]) {
      const double height = node["height"].as<double>();
      if (!finite(height)) {
        throw std::runtime_error(context + ": height must be finite");
      }
      z.assign(x.size(), height);
    } else {
      throw std::runtime_error(context + ": provide either z: [...] or height: <scalar>");
    }

    SegmentConfig segment;
    segment.points.reserve(x.size());
    for (std::size_t i = 0U; i < x.size(); ++i) {
      segment.points.push_back(Vec3{x[i], y[i], z[i]});
    }
    return segment;
  }

  static std::vector<SegmentConfig> parse_missions(
    const YAML::Node & node,
    const std::string & context)
  {
    if (!node) {
      throw std::runtime_error(context + " is required");
    }

    std::vector<SegmentConfig> missions;

    // Backward-compatible shorthand:
    //
    // mission:
    //   x: [...]
    //   y: [...]
    //   z: [...]
    //
    // is interpreted as exactly one mission component.
    if (node.IsMap()) {
      missions.push_back(
        parse_segment(
          node,
          context + "[0]",
          true));

      return missions;
    }

    // New representation:
    //
    // mission:
    //   - x: [...]
    //     y: [...]
    //     z: [...]
    //   - x: [...]
    //     y: [...]
    //     z: [...]
    if (!node.IsSequence() || node.size() == 0U) {
      throw std::runtime_error(
              context +
              " must be either one mission map or a non-empty sequence of mission maps");
    }

    missions.reserve(node.size());

    for (std::size_t i = 0U; i < node.size(); ++i) {
      const std::string mission_context =
        context + "[" + std::to_string(i) + "]";

      missions.push_back(
        parse_segment(
          node[i],
          mission_context,
          true));
    }

    return missions;
  }

  void load_configuration()
  {
    const YAML::Node document = YAML::LoadFile(config_file_);
    const YAML::Node root = document["static_trajectory_manager"];
    if (!root || !root.IsMap()) {
      throw std::runtime_error("YAML must contain a 'static_trajectory_manager' map");
    }

    default_frame_id_ = optional_value<std::string>(root, "frame_id", "map");
    flight_zone_validation_step_m_ = optional_value<double>(root, "flight_zone_validation_step_m", 0.25);
    boundary_tolerance_m_ = optional_value<double>(root, "flight_zone_boundary_tolerance_m", 0.03);
    boundary_is_inside_ = optional_value<bool>(root, "flight_zone_boundary_is_inside", true);
    marker_line_width_ = optional_value<double>(root, "marker_line_width", 0.12);
    marker_text_height_ = optional_value<double>(root, "marker_text_height", 0.32);

    if (default_frame_id_.empty()) {
      throw std::runtime_error("frame_id must not be empty");
    }
    if (!finite(flight_zone_validation_step_m_) || flight_zone_validation_step_m_ <= 0.0) {
      throw std::runtime_error("flight_zone_validation_step_m must be finite and positive");
    }
    if (!finite(boundary_tolerance_m_) || boundary_tolerance_m_ < 0.0) {
      throw std::runtime_error("flight_zone_boundary_tolerance_m must be finite and non-negative");
    }
    if (!finite(marker_line_width_) || marker_line_width_ <= 0.0 ||
      !finite(marker_text_height_) || marker_text_height_ <= 0.0)
    {
      throw std::runtime_error("marker sizes must be finite and positive");
    }

    const YAML::Node entries = root["trajectories"];
    if (!entries || !entries.IsSequence() || entries.size() == 0U) {
      throw std::runtime_error("'static_trajectory_manager.trajectories' must be a non-empty sequence");
    }

    std::set<std::string> ids;
    trajectories_.clear();
    trajectories_.reserve(entries.size());

    for (std::size_t i = 0U; i < entries.size(); ++i) {
      const YAML::Node entry = entries[i];
      const std::string context = "trajectories[" + std::to_string(i) + "]";
      if (!entry.IsMap()) {
        throw std::runtime_error(context + " must be a YAML map");
      }

      TrajectoryConfig trajectory;
      trajectory.input_index = i;
      trajectory.trajectory_id = required_value<std::string>(entry, "trajectory_id", context);
      trajectory.priority = required_value<int32_t>(entry, "priority", context);
      trajectory.ua_id = required_value<uint32_t>(entry, "ua_id", context);
      trajectory.uas_namespace = required_value<std::string>(entry, "uas_namespace", context);
      trajectory.flight_zone_id = required_value<std::string>(entry, "flight_zone_id", context);
      trajectory.frame_id = optional_value<std::string>(entry, "frame_id", default_frame_id_);
      trajectory.action_name = optional_value<std::string>(
        entry, "action_name",
        "/" + trajectory.flight_zone_id + "/" + trajectory.uas_namespace + "/follow_waypoints");
      trajectory.goal_tolerance = optional_value<double>(entry, "goal_tolerance", 0.12);
      trajectory.slowdown_radius = optional_value<double>(entry, "slowdown_radius", 0.20);
      trajectory.repetitions = optional_value<uint32_t>(entry, "repetitions", 1U);
      trajectory.operation_frequency = optional_value<double>(entry, "operation_frequency", 0.0);

      // Keep partial mission repetition and total operation repetition separate.
      // A periodic definition defaults to unlimited complete repetitions.
      // A non-periodic definition defaults to one complete operation.
      trajectory.total_repetitions = optional_value<uint32_t>(
        entry, "total_repetitions",
        trajectory.operation_frequency > 0.0 ? 0U : 1U);

      trajectory.average_speed_mps = required_value<double>(entry, "average_speed_mps", context);
      trajectory.operation_start_ns = parse_utc_iso8601_ns(
        required_value<std::string>(entry, "operation_start_utc", context));

      trajectory.takeoff = parse_segment(entry["takeoff"], context + ".takeoff", false);
      trajectory.missions = parse_missions(entry["mission"], context + ".mission");
      trajectory.landing = parse_segment(entry["landing"], context + ".landing", false);

      if (trajectory.trajectory_id.empty() || trajectory.uas_namespace.empty() ||
        trajectory.flight_zone_id.empty() || trajectory.frame_id.empty())
      {
        throw std::runtime_error(context + ": identifiers and frame_id must not be empty");
      }
      if (trajectory.ua_id == 0U) {
        throw std::runtime_error(context + ": ua_id must be greater than zero");
      }
      if (trajectory.action_name.empty() || trajectory.action_name.front() != '/') {
        throw std::runtime_error(context + ": action_name must be an absolute ROS name");
      }
      if (!finite(trajectory.goal_tolerance) || trajectory.goal_tolerance <= 0.0 ||
        !finite(trajectory.slowdown_radius) || trajectory.slowdown_radius <= 0.0)
      {
        throw std::runtime_error(context + ": goal_tolerance and slowdown_radius must be finite and positive");
      }
      if (trajectory.repetitions == 0U) {
        throw std::runtime_error(context + ": repetitions must be at least one");
      }
      if (!finite(trajectory.operation_frequency) || trajectory.operation_frequency < 0.0) {
        throw std::runtime_error(context + ": operation_frequency must be finite and >= 0 seconds");
      }

      if (trajectory.operation_frequency <= 0.0) {
        if (trajectory.total_repetitions != 1U) {
          throw std::runtime_error(
                  context +
                  ": total_repetitions must be 1 when operation_frequency is 0.0");
        }
      }

      if (!finite(trajectory.average_speed_mps) || trajectory.average_speed_mps <= 0.0) {
        throw std::runtime_error(context + ": average_speed_mps must be finite and positive");
      }
      if (!ids.insert(trajectory.trajectory_id).second) {
        throw std::runtime_error("Duplicate trajectory_id: '" + trajectory.trajectory_id + "'");
      }

      trajectory.estimated_distance_m =
        operation_geometry_length(trajectory);
      trajectory.estimated_duration_s = trajectory.estimated_distance_m / trajectory.average_speed_mps;
      const int64_t duration_ns = static_cast<int64_t>(
        std::ceil(trajectory.estimated_duration_s * static_cast<double>(kNanosecondsPerSecond)));
      trajectory.operation_end_ns = trajectory.operation_start_ns + duration_ns;

      if (trajectory.operation_frequency > 0.0 &&
        trajectory.operation_frequency + 1.0e-9 < trajectory.estimated_duration_s)
      {
        RCLCPP_WARN(
          get_logger(),
          "Trajectory '%s': operation_frequency=%.3f s is shorter than estimated duration=%.3f s; complete repetitions would overlap in time",
          trajectory.trajectory_id.c_str(), trajectory.operation_frequency, trajectory.estimated_duration_s);
      }

      trajectories_.push_back(std::move(trajectory));
    }
  }

  static PreparedZone prepare_zone(const FlightZone & zone)
  {
    PreparedZone prepared;
    prepared.zone = zone;

    if (zone.vertices.size() < 4U || zone.faces.size() < 4U) {
      prepared.error = "polyhedron requires at least four vertices and faces";
      return prepared;
    }

    prepared.min_corner = Vec3{
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
    prepared.max_corner = Vec3{
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};

    for (const auto & vertex : zone.vertices) {
      prepared.min_corner.x = std::min(prepared.min_corner.x, vertex.x);
      prepared.min_corner.y = std::min(prepared.min_corner.y, vertex.y);
      prepared.min_corner.z = std::min(prepared.min_corner.z, vertex.z);
      prepared.max_corner.x = std::max(prepared.max_corner.x, vertex.x);
      prepared.max_corner.y = std::max(prepared.max_corner.y, vertex.y);
      prepared.max_corner.z = std::max(prepared.max_corner.z, vertex.z);
    }

    for (const auto & face : zone.faces) {
      if (face.vertex_indices.size() < 3U) {
        prepared.error = "face has fewer than three vertices";
        prepared.triangles.clear();
        return prepared;
      }
      const uint32_t anchor = face.vertex_indices.front();
      if (anchor >= zone.vertices.size()) {
        prepared.error = "face index outside vertices array";
        prepared.triangles.clear();
        return prepared;
      }

      for (std::size_t i = 1U; i + 1U < face.vertex_indices.size(); ++i) {
        const uint32_t second = face.vertex_indices[i];
        const uint32_t third = face.vertex_indices[i + 1U];
        if (second >= zone.vertices.size() || third >= zone.vertices.size()) {
          prepared.error = "face index outside vertices array";
          prepared.triangles.clear();
          return prepared;
        }

        Triangle triangle{
          to_vec3(zone.vertices[anchor]),
          to_vec3(zone.vertices[second]),
          to_vec3(zone.vertices[third])};
        if (squared_norm(cross(
            subtract(triangle.b, triangle.a),
            subtract(triangle.c, triangle.a))) <= kEpsilon)
        {
          prepared.error = "degenerate triangle in zone";
          prepared.triangles.clear();
          return prepared;
        }
        prepared.triangles.push_back(triangle);
      }
    }

    if (prepared.triangles.size() < 4U) {
      prepared.error = "zone triangulation produced too few triangles";
      return prepared;
    }
    prepared.valid = true;
    return prepared;
  }

  static double squared_distance_to_triangle(const Vec3 & point, const Triangle & triangle)
  {
    const Vec3 ab = subtract(triangle.b, triangle.a);
    const Vec3 ac = subtract(triangle.c, triangle.a);
    const Vec3 ap = subtract(point, triangle.a);
    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
      return squared_norm(ap);
    }

    const Vec3 bp = subtract(point, triangle.b);
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
      return squared_norm(bp);
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
      const double v = d1 / (d1 - d3);
      return squared_norm(subtract(point, add(triangle.a, multiply(ab, v))));
    }

    const Vec3 cp = subtract(point, triangle.c);
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
      return squared_norm(cp);
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
      const double w = d2 / (d2 - d6);
      return squared_norm(subtract(point, add(triangle.a, multiply(ac, w))));
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
      const Vec3 bc = subtract(triangle.c, triangle.b);
      const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      return squared_norm(subtract(point, add(triangle.b, multiply(bc, w))));
    }

    const double denominator = 1.0 / (va + vb + vc);
    const double v = vb * denominator;
    const double w = vc * denominator;
    const Vec3 projection = add(triangle.a, add(multiply(ab, v), multiply(ac, w)));
    return squared_norm(subtract(point, projection));
  }

  static bool ray_triangle_intersection(
    const Vec3 & origin, const Vec3 & direction, const Triangle & triangle, double & distance)
  {
    const Vec3 edge_1 = subtract(triangle.b, triangle.a);
    const Vec3 edge_2 = subtract(triangle.c, triangle.a);
    const Vec3 p_vector = cross(direction, edge_2);
    const double determinant = dot(edge_1, p_vector);
    if (std::abs(determinant) <= kEpsilon) {
      return false;
    }

    const double inverse = 1.0 / determinant;
    const Vec3 t_vector = subtract(origin, triangle.a);
    const double u = dot(t_vector, p_vector) * inverse;
    if (u < -kEpsilon || u > 1.0 + kEpsilon) {
      return false;
    }

    const Vec3 q_vector = cross(t_vector, edge_1);
    const double v = dot(direction, q_vector) * inverse;
    if (v < -kEpsilon || u + v > 1.0 + kEpsilon) {
      return false;
    }

    distance = dot(edge_2, q_vector) * inverse;
    return distance > kEpsilon;
  }

  static std::size_t unique_ray_intersections(
    const Vec3 & point, const Vec3 & direction, const std::vector<Triangle> & triangles)
  {
    std::vector<double> distances;
    for (const auto & triangle : triangles) {
      double distance = 0.0;
      if (ray_triangle_intersection(point, direction, triangle, distance)) {
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

  PointRelation classify_point(const Vec3 & point, const PreparedZone & zone) const
  {
    if (!zone.valid || zone.triangles.empty()) {
      return PointRelation::Invalid;
    }

    if (point.x < zone.min_corner.x - boundary_tolerance_m_ ||
      point.x > zone.max_corner.x + boundary_tolerance_m_ ||
      point.y < zone.min_corner.y - boundary_tolerance_m_ ||
      point.y > zone.max_corner.y + boundary_tolerance_m_ ||
      point.z < zone.min_corner.z - boundary_tolerance_m_ ||
      point.z > zone.max_corner.z + boundary_tolerance_m_)
    {
      return PointRelation::Outside;
    }

    const double tolerance_squared = boundary_tolerance_m_ * boundary_tolerance_m_;
    for (const auto & triangle : zone.triangles) {
      if (squared_distance_to_triangle(point, triangle) <= tolerance_squared) {
        return PointRelation::Boundary;
      }
    }

    const std::array<Vec3, 3> rays{
      normalized(Vec3{1.0, 0.371390676, 0.694245231}),
      normalized(Vec3{0.217391304, 1.0, 0.539682540}),
      normalized(Vec3{0.483870968, 0.290322581, 1.0})};

    std::size_t inside_votes = 0U;
    for (const auto & ray : rays) {
      if (unique_ray_intersections(point, ray, zone.triangles) % 2U == 1U) {
        ++inside_votes;
      }
    }
    return inside_votes >= 2U ? PointRelation::Inside : PointRelation::Outside;
  }

  bool relation_is_inside(PointRelation relation) const
  {
    return relation == PointRelation::Inside ||
      (relation == PointRelation::Boundary && boundary_is_inside_);
  }

  bool segment_inside_zone(const Vec3 & a, const Vec3 & b, const PreparedZone & zone) const
  {
    const double distance = norm(subtract(b, a));
    const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(distance / flight_zone_validation_step_m_)));

    for (std::size_t i = 0U; i <= samples; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(samples);
      const Vec3 point = add(a, multiply(subtract(b, a), ratio));
      if (!relation_is_inside(classify_point(point, zone))) {
        return false;
      }
    }
    return true;
  }

  bool mission_inside_zone(const TrajectoryConfig & trajectory, const PreparedZone & zone) const
  {
    if (!zone.valid || !zone.zone.enabled || zone.zone.zone_type != FlightZone::INCLUSION) {
      return false;
    }

    if (zone.zone.header.frame_id != trajectory.frame_id) {
      return false;
    }

    // Every mission[] element is an independent polyline. Validate only the
    // explicitly represented internal edges. No connector is inferred between
    // two different mission components.
    for (const auto & mission : trajectory.missions) {
      const auto & points = mission.points;

      for (std::size_t i = 1U; i < points.size(); ++i) {
        if (!segment_inside_zone(points[i - 1U], points[i], zone)) {
          return false;
        }
      }
    }

    return true;
  }

  void flight_zones_callback(const FlightZoneArray::SharedPtr message)
  {
    prepared_zones_.clear();
    for (const auto & zone : message->zones) {
      PreparedZone prepared = prepare_zone(zone);
      if (!zone.enabled) {
        prepared.valid = false;
        prepared.error = "zone is disabled";
      }
      prepared_zones_[zone.zone_id] = std::move(prepared);
    }
    flight_zones_received_ = true;
    publish_valid_snapshot();
  }

  uint32_t remaining_total_repetitions(const TrajectoryConfig & trajectory) const
  {
    if (trajectory.operation_frequency <= 0.0) {
      return trajectory.completed_total_repetitions >= 1U ? 0U : 1U;
    }

    if (trajectory.total_repetitions == 0U) {
      return std::numeric_limits<uint32_t>::max();
    }

    if (trajectory.completed_total_repetitions >= trajectory.total_repetitions) {
      return 0U;
    }

    return trajectory.total_repetitions - trajectory.completed_total_repetitions;
  }

  std::string repetition_status(const TrajectoryConfig & trajectory) const
  {
    std::ostringstream stream;
    stream << "completed=" << trajectory.completed_total_repetitions << " | remaining=";

    if (trajectory.operation_frequency > 0.0 && trajectory.total_repetitions == 0U) {
      stream << "unlimited";
    } else {
      stream << remaining_total_repetitions(trajectory);
    }

    return stream.str();
  }

  static bool adjusted_segment_is_well_formed(const TrajectorySegment & segment)
  {
    return segment.x.size() == segment.y.size() &&
      segment.x.size() == segment.z.size();
  }

  static bool segment_geometry_matches(
    const SegmentConfig & stored,
    const TrajectorySegment & adjusted)
  {
    if (!adjusted_segment_is_well_formed(adjusted)) {
      return false;
    }

    if (stored.points.size() != adjusted.x.size()) {
      return false;
    }

    for (std::size_t i = 0U; i < stored.points.size(); ++i) {
      if (
        std::abs(stored.points[i].x - adjusted.x[i]) > kEpsilon ||
        std::abs(stored.points[i].y - adjusted.y[i]) > kEpsilon ||
        std::abs(stored.points[i].z - adjusted.z[i]) > kEpsilon)
      {
        return false;
      }
    }

    return true;
  }

  static bool mission_geometry_matches(
    const std::vector<SegmentConfig> & stored,
    const std::vector<TrajectorySegment> & adjusted)
  {
    if (stored.size() != adjusted.size()) {
      return false;
    }

    for (std::size_t i = 0U; i < stored.size(); ++i) {
      if (!segment_geometry_matches(stored[i], adjusted[i])) {
        return false;
      }
    }

    return true;
  }

  static bool mission_array_is_well_formed(
    const std::vector<TrajectorySegment> & missions)
  {
    return std::all_of(
      missions.begin(),
      missions.end(),
      [](const TrajectorySegment & mission) {
        return adjusted_segment_is_well_formed(mission);
      });
  }

  static SegmentConfig segment_config_from_message(const TrajectorySegment & segment)
  {
    SegmentConfig output;
    output.points.reserve(segment.x.size());

    for (std::size_t i = 0U; i < segment.x.size(); ++i) {
      output.points.push_back(
        Vec3{
          segment.x[i],
          segment.y[i],
          segment.z[i]});
    }

    return output;
  }

  static std::vector<SegmentConfig> mission_configs_from_message(
    const std::vector<TrajectorySegment> & missions)
  {
    std::vector<SegmentConfig> output;
    output.reserve(missions.size());

    for (const auto & mission : missions) {
      output.push_back(
        segment_config_from_message(mission));
    }

    return output;
  }

  bool replace_geometry_if_changed(
    TrajectoryConfig & stored,
    const StaticTrajectory & adjusted)
  {
    const bool takeoff_changed =
      !segment_geometry_matches(stored.takeoff, adjusted.takeoff);

    const bool mission_changed =
      !mission_geometry_matches(stored.missions, adjusted.mission);

    const bool landing_changed =
      !segment_geometry_matches(stored.landing, adjusted.landing);

    if (
      !takeoff_changed &&
      !mission_changed &&
      !landing_changed)
    {
      return false;
    }

    if (takeoff_changed) {
      stored.takeoff =
        segment_config_from_message(adjusted.takeoff);
    }

    if (mission_changed) {
      stored.missions =
        mission_configs_from_message(adjusted.mission);
    }

    if (landing_changed) {
      stored.landing =
        segment_config_from_message(adjusted.landing);
    }

    // Intentionally do NOT recalculate:
    //   - operation_frequency / total repetition state
    //   - operation_start_ns / operation_end_ns
    //   - estimated_distance_m / estimated_duration_s
    // The existing temporal-adjustment logic remains authoritative for timing
    // and periodic progression, but it is intentionally NOT executed for the
    // same callback when this function reports a geometry change.
    RCLCPP_INFO(
      get_logger(),
      "Replaced stored geometry for adjusted trajectory '%s' | "
      "takeoff=%s | mission_count=%zu | mission=%s | landing=%s",
      stored.trajectory_id.c_str(),
      takeoff_changed ? "changed" : "unchanged",
      stored.missions.size(),
      mission_changed ? "changed" : "unchanged",
      landing_changed ? "changed" : "unchanged");

    return true;
  }

  static std::string publisher_gid_key(const rclcpp::MessageInfo & message_info)
  {
    const auto & rmw_info = message_info.get_rmw_message_info();
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');

    for (std::size_t i = 0U; i < sizeof(rmw_info.publisher_gid.data); ++i) {
      stream << std::setw(2)
             << static_cast<unsigned int>(rmw_info.publisher_gid.data[i]);
    }

    return stream.str();
  }

  void adjusted_trajectories_callback(
    const StaticTrajectoryArray::SharedPtr message,
    const rclcpp::MessageInfo & message_info)
  {
    if (!message) {
      return;
    }

    const std::string publisher_gid =
      publisher_gid_key(message_info);

    bool changed = false;

    for (const auto & adjusted : message->trajectories) {
      if (adjusted.trajectory_id.empty()) {
        continue;
      }

      auto trajectory_it = std::find_if(
        trajectories_.begin(), trajectories_.end(),
        [&adjusted](const TrajectoryConfig & trajectory) {
          return trajectory.trajectory_id == adjusted.trajectory_id;
        });

      if (trajectory_it == trajectories_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring adjusted trajectory '%s': trajectory_id is not present in the static YAML.",
          adjusted.trajectory_id.c_str());
        continue;
      }

      auto & trajectory = *trajectory_it;

      if (
        !adjusted_segment_is_well_formed(adjusted.takeoff) ||
        !mission_array_is_well_formed(adjusted.mission) ||
        !adjusted_segment_is_well_formed(adjusted.landing))
      {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring adjusted trajectory '%s': one or more geometry segments "
          "have different x/y/z array lengths.",
          adjusted.trajectory_id.c_str());
        continue;
      }

      // Geometry update is independent from the existing temporal adjustment
      // semantics. If the supervision pipeline has cropped TAKEOFF, MISSION or
      // LANDING, replace only those stored point arrays. All timing and
      // periodicity handling below remains exactly as before.
      const bool geometry_changed =
        replace_geometry_if_changed(trajectory, adjusted);

      if (geometry_changed) {
        changed = true;

        // The publisher that first supplied this different geometry is the
        // geometry/supervision source for this trajectory. Its retained or
        // periodic re-publications are geometry-state updates, not completed
        // executions. Remember its DDS identity so a second identical message
        // cannot accidentally advance a periodic trajectory.
        trajectory.geometry_update_publisher_gid = publisher_gid;

        // A supervision crop is a geometry-only update. Do not interpret this
        // same message as a completed temporal occurrence.
        continue;
      }

      if (
        !trajectory.geometry_update_publisher_gid.empty() &&
        publisher_gid == trajectory.geometry_update_publisher_gid)
      {
        RCLCPP_DEBUG(
          get_logger(),
          "Ignoring geometry-only re-publication for trajectory '%s' from "
          "the supervision geometry publisher.",
          trajectory.trajectory_id.c_str());
        continue;
      }

      // From this point the geometry already matches what is stored and the
      // message comes from a different publisher. Therefore the trajectory is
      // treated exactly like any normal executed trajectory: end-time updates,
      // duplicate/stale checks and periodic repetition accounting all apply.
      const int64_t adjusted_start_ns = time_to_ns(adjusted.operation_start_utc);
      const int64_t adjusted_end_ns = time_to_ns(adjusted.operation_end_utc);

      if (adjusted_end_ns <= adjusted_start_ns) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring adjusted trajectory '%s': adjusted interval is invalid (%s -> %s).",
          adjusted.trajectory_id.c_str(),
          format_utc_iso8601(adjusted_start_ns).c_str(),
          format_utc_iso8601(adjusted_end_ns).c_str());
        continue;
      }

      // -------------------------------------------------------------------
      // Non-periodic operation: preserve the previous behaviour. The end-time
      // correction becomes the final stored UTC. There is no next occurrence.
      // -------------------------------------------------------------------
      if (trajectory.operation_frequency <= 0.0) {
        if (adjusted_end_ns == trajectory.operation_end_ns) {
          continue;
        }

        const int64_t previous_end_ns = trajectory.operation_end_ns;
        trajectory.operation_end_ns = adjusted_end_ns;
        changed = true;

        RCLCPP_INFO(
          get_logger(),
          "Applied final adjusted end for non-periodic trajectory '%s': %s -> %s",
          trajectory.trajectory_id.c_str(),
          format_utc_iso8601(previous_end_ns).c_str(),
          format_utc_iso8601(adjusted_end_ns).c_str());
        continue;
      }

      // -------------------------------------------------------------------
      // Periodic complete operation.
      //
      // /adjusted_trajectories represents a completed concrete occurrence.
      // The occurrence may have been delayed by trajectory_server_node, so
      // adjusted.operation_start_utc is authoritative for the occurrence that
      // actually ran.
      //
      // A duplicated retained message is ignored by its concrete start time.
      // A start older than the current stored occurrence is also stale.
      // -------------------------------------------------------------------
      if (
        trajectory.processed_adjustment_starts.find(adjusted_start_ns) !=
        trajectory.processed_adjustment_starts.end())
      {
        RCLCPP_DEBUG(
          get_logger(),
          "Ignoring duplicated adjustment for periodic trajectory '%s' at start=%s",
          trajectory.trajectory_id.c_str(),
          format_utc_iso8601(adjusted_start_ns).c_str());
        continue;
      }

      if (adjusted_start_ns < trajectory.operation_start_ns) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring stale adjustment for periodic trajectory '%s': adjusted start %s "
          "is older than current stored start %s",
          trajectory.trajectory_id.c_str(),
          format_utc_iso8601(adjusted_start_ns).c_str(),
          format_utc_iso8601(trajectory.operation_start_ns).c_str());
        continue;
      }

      trajectory.processed_adjustment_starts.insert(adjusted_start_ns);
      ++trajectory.completed_total_repetitions;

      const bool finite_total = trajectory.total_repetitions > 0U;
      const bool exhausted =
        finite_total &&
        trajectory.completed_total_repetitions >= trajectory.total_repetitions;

      if (exhausted) {
        // Keep the actual interval of the final completed occurrence for
        // /latest_trajectories.
        trajectory.operation_start_ns = adjusted_start_ns;
        trajectory.operation_end_ns = adjusted_end_ns;
        trajectory.total_repetition_exhausted = true;
        changed = true;

        RCLCPP_INFO(
          get_logger(),
          "Periodic trajectory '%s' completed its final total repetition | "
          "actual=%s -> %s | %s",
          trajectory.trajectory_id.c_str(),
          format_utc_iso8601(adjusted_start_ns).c_str(),
          format_utc_iso8601(adjusted_end_ns).c_str(),
          repetition_status(trajectory).c_str());
        continue;
      }

      const int64_t period_ns = seconds_to_ns(trajectory.operation_frequency);
      if (period_ns <= 0) {
        RCLCPP_ERROR(
          get_logger(),
          "Cannot advance periodic trajectory '%s': operation_frequency=%.9f s "
          "is not representable as a positive nanosecond period.",
          trajectory.trajectory_id.c_str(),
          trajectory.operation_frequency);
        continue;
      }

      // Advance one complete repetition.
      //
      // Both limits are shifted by the same period from the concrete occurrence
      // that actually executed. Therefore:
      //   next_start = actual_start + period
      //   next_end   = actual_end   + period
      //
      // This carries the measured positive/negative end-time correction into
      // the predicted duration of the next complete operation.
      const int64_t previous_stored_start = trajectory.operation_start_ns;
      const int64_t previous_stored_end = trajectory.operation_end_ns;

      if (
        adjusted_start_ns > std::numeric_limits<int64_t>::max() - period_ns ||
        adjusted_end_ns > std::numeric_limits<int64_t>::max() - period_ns)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Cannot advance periodic trajectory '%s': next repetition exceeds int64 UTC range.",
          trajectory.trajectory_id.c_str());
        continue;
      }

      trajectory.operation_start_ns = adjusted_start_ns + period_ns;
      trajectory.operation_end_ns = adjusted_end_ns + period_ns;
      trajectory.total_repetition_exhausted = false;
      changed = true;

      if (trajectory.operation_end_ns <= trajectory.operation_start_ns) {
        RCLCPP_ERROR(
          get_logger(),
          "Periodic trajectory '%s' produced an invalid next interval after adjustment.",
          trajectory.trajectory_id.c_str());
        trajectory.operation_start_ns = previous_stored_start;
        trajectory.operation_end_ns = previous_stored_end;
        --trajectory.completed_total_repetitions;
        trajectory.processed_adjustment_starts.erase(adjusted_start_ns);
        continue;
      }

      if (trajectory.operation_start_ns < adjusted_end_ns) {
        RCLCPP_WARN(
          get_logger(),
          "Periodic trajectory '%s': next start %s occurs before the previous real end %s. "
          "operation_frequency=%.3f s is shorter than the measured complete-operation duration.",
          trajectory.trajectory_id.c_str(),
          format_utc_iso8601(trajectory.operation_start_ns).c_str(),
          format_utc_iso8601(adjusted_end_ns).c_str(),
          trajectory.operation_frequency);
      }

      RCLCPP_INFO(
        get_logger(),
        "Advanced periodic trajectory '%s' to next complete repetition | "
        "previous stored=%s -> %s | actual completed=%s -> %s | "
        "next=%s -> %s | extra_time=%+.3f s | %s",
        trajectory.trajectory_id.c_str(),
        format_utc_iso8601(previous_stored_start).c_str(),
        format_utc_iso8601(previous_stored_end).c_str(),
        format_utc_iso8601(adjusted_start_ns).c_str(),
        format_utc_iso8601(adjusted_end_ns).c_str(),
        format_utc_iso8601(trajectory.operation_start_ns).c_str(),
        format_utc_iso8601(trajectory.operation_end_ns).c_str(),
        adjusted.extra_time,
        repetition_status(trajectory).c_str());
    }

    if (changed) {
      publish_valid_snapshot();
    }
  }

  static TrajectorySegment segment_to_message(const SegmentConfig & segment)
  {
    TrajectorySegment message;
    message.x.reserve(segment.points.size());
    message.y.reserve(segment.points.size());
    message.z.reserve(segment.points.size());
    for (const auto & point : segment.points) {
      message.x.push_back(point.x);
      message.y.push_back(point.y);
      message.z.push_back(point.z);
    }
    return message;
  }

  StaticTrajectory trajectory_to_message(const TrajectoryConfig & trajectory) const
  {
    StaticTrajectory message;
    message.priority = trajectory.priority;
    message.trajectory_id = trajectory.trajectory_id;
    message.ua_id = trajectory.ua_id;
    message.uas_namespace = trajectory.uas_namespace;
    message.flight_zone_id = trajectory.flight_zone_id;
    message.frame_id = trajectory.frame_id;
    message.action_name = trajectory.action_name;
    message.takeoff = segment_to_message(trajectory.takeoff);

    message.mission.reserve(
      trajectory.missions.size());

    for (const auto & mission : trajectory.missions) {
      message.mission.push_back(
        segment_to_message(mission));
    }

    message.landing = segment_to_message(trajectory.landing);
    message.goal_tolerance = trajectory.goal_tolerance;
    message.slowdown_radius = trajectory.slowdown_radius;
    message.repetitions = trajectory.repetitions;
    message.operation_frequency = trajectory.operation_frequency;
    message.average_speed_mps = trajectory.average_speed_mps;
    message.estimated_distance_m = trajectory.estimated_distance_m;
    message.estimated_duration_s = trajectory.estimated_duration_s;
    message.operation_start_utc = ns_to_time(trajectory.operation_start_ns);
    message.operation_end_utc = ns_to_time(trajectory.operation_end_ns);

    // Once an adjusted end has been absorbed by this manager it becomes the
    // new baseline end. Do not propagate the old overrun again downstream.
    message.extra_time = 0.0;

    message.spacial_conflict = false;
    message.spacial_conflicting_trajectory_id.clear();
    return message;
  }

  static std_msgs::msg::ColorRGBA color_from_index(std::size_t index)
  {
    const double h = std::fmod(static_cast<double>(index) * 0.618033988749895, 1.0);
    const double s = 0.72;
    const double v = 0.95;
    const double c = v * s;
    const double x = c * (1.0 - std::abs(std::fmod(h * 6.0, 2.0) - 1.0));
    const double m = v - c;

    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    const int sector = static_cast<int>(std::floor(h * 6.0)) % 6;
    switch (sector) {
      case 0: r = c; g = x; break;
      case 1: r = x; g = c; break;
      case 2: g = c; b = x; break;
      case 3: g = x; b = c; break;
      case 4: r = x; b = c; break;
      default: r = c; b = x; break;
    }

    std_msgs::msg::ColorRGBA color;
    color.r = static_cast<float>(r + m);
    color.g = static_cast<float>(g + m);
    color.b = static_cast<float>(b + m);
    color.a = 1.0F;
    return color;
  }

  static void append_line_strip_marker(
    const std::vector<Vec3> & points,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const std::string & marker_namespace,
    int marker_id,
    double line_width,
    const std_msgs::msg::ColorRGBA & marker_color,
    MarkerArray & marker_array)
  {
    if (points.size() < 2U) {
      return;
    }

    Marker line;
    line.header.frame_id = frame_id;
    line.header.stamp = stamp;
    line.ns = marker_namespace;
    line.id = marker_id;
    line.type = Marker::LINE_STRIP;
    line.action = Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = line_width;
    line.color = marker_color;
    line.points.reserve(points.size());

    for (const auto & point : points) {
      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.x;
      ros_point.y = point.y;
      ros_point.z = point.z;
      line.points.push_back(ros_point);
    }

    marker_array.markers.push_back(
      std::move(line));
  }

  void add_markers(
    const TrajectoryConfig & trajectory,
    const rclcpp::Time & stamp,
    MarkerArray & marker_array) const
  {
    const auto marker_color =
      color_from_index(
      trajectory.input_index);

    int marker_id = 0;

    append_line_strip_marker(
      trajectory.takeoff.points,
      trajectory.frame_id,
      stamp,
      "static_trajectory/" + trajectory.trajectory_id + "/takeoff",
      marker_id++,
      marker_line_width_,
      marker_color,
      marker_array);

    // Each mission is intentionally rendered as an independent LINE_STRIP.
    // RViz therefore never draws a fake connector between mission components.
    for (std::size_t i = 0U;
      i < trajectory.missions.size();
      ++i)
    {
      append_line_strip_marker(
        trajectory.missions[i].points,
        trajectory.frame_id,
        stamp,
        "static_trajectory/" + trajectory.trajectory_id +
        "/mission_" + std::to_string(i),
        marker_id++,
        marker_line_width_,
        marker_color,
        marker_array);
    }

    append_line_strip_marker(
      trajectory.landing.points,
      trajectory.frame_id,
      stamp,
      "static_trajectory/" + trajectory.trajectory_id + "/landing",
      marker_id++,
      marker_line_width_,
      marker_color,
      marker_array);

    Marker text;
    text.header.frame_id = trajectory.frame_id;
    text.header.stamp = stamp;
    text.ns =
      "static_trajectory/" +
      trajectory.trajectory_id +
      "/label";
    text.id = marker_id++;
    text.type = Marker::TEXT_VIEW_FACING;
    text.action = Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.scale.z = marker_text_height_;
    text.color = marker_color;
    text.color.a = 1.0F;

    const Vec3 * first =
      first_geometry_point(trajectory);

    if (first != nullptr) {
      text.pose.position.x = first->x;
      text.pose.position.y = first->y;
      text.pose.position.z =
        first->z +
        marker_text_height_ * 1.8;
    }

    std::ostringstream label;
    label
      << trajectory.trajectory_id
      << "  P=" << trajectory.priority << "\n"
      << "start: "
      << format_utc_iso8601(
      trajectory.operation_start_ns) << "\n"
      << "end:   "
      << format_utc_iso8601(
      trajectory.operation_end_ns) << "\n"
      << "missions="
      << trajectory.missions.size()
      << " | partial repetitions="
      << trajectory.repetitions;

    if (trajectory.operation_frequency > 0.0) {
      label
        << " | full period "
        << std::fixed
        << std::setprecision(1)
        << trajectory.operation_frequency
        << " s\n"
        << "total repetitions: done="
        << trajectory.completed_total_repetitions
        << " | remaining=";

      if (trajectory.total_repetitions == 0U) {
        label << "unlimited";
      } else {
        label <<
          remaining_total_repetitions(
          trajectory);
      }
    }

    text.text = label.str();

    marker_array.markers.push_back(
      std::move(text));
  }

  void publish_valid_snapshot()
  {
    StaticTrajectoryArray requested_output;
    StaticTrajectoryArray unvalidated_output;
    StaticTrajectoryArray latest_output;

    requested_output.header.stamp = now();
    requested_output.header.frame_id = default_frame_id_;
    unvalidated_output.header = requested_output.header;
    latest_output.header = requested_output.header;

    MarkerArray markers;
    Marker delete_all;
    delete_all.header.frame_id = default_frame_id_;
    delete_all.header.stamp = requested_output.header.stamp;
    delete_all.action = Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    const int64_t current_ns = system_now_ns();

    std::vector<const TrajectoryConfig *> requested;
    std::vector<const TrajectoryConfig *> unvalidated;
    std::vector<const TrajectoryConfig *> latest;

    requested.reserve(trajectories_.size());
    unvalidated.reserve(trajectories_.size());
    latest.reserve(trajectories_.size());

    for (const auto & trajectory : trajectories_) {
      // Geometry/flight-zone validation is independent from temporal validity.
      // Until /flight_zones has been received the request cannot be validated,
      // so it is exposed in /unvalidated_trajectories.
      if (!flight_zones_received_) {
        unvalidated.push_back(&trajectory);
        continue;
      }

      const auto zone_it = prepared_zones_.find(trajectory.flight_zone_id);
      if (zone_it == prepared_zones_.end()) {
        RCLCPP_DEBUG(
          get_logger(), "Unvalidated '%s': flight zone '%s' not found",
          trajectory.trajectory_id.c_str(), trajectory.flight_zone_id.c_str());
        unvalidated.push_back(&trajectory);
        continue;
      }

      if (!mission_inside_zone(trajectory, zone_it->second)) {
        RCLCPP_DEBUG(
          get_logger(),
          "Unvalidated '%s': one or more mission polylines are not fully inside enabled INCLUSION zone '%s'",
          trajectory.trajectory_id.c_str(), trajectory.flight_zone_id.c_str());
        unvalidated.push_back(&trajectory);
        continue;
      }

      // The temporal interval itself must be coherent. This should normally
      // be guaranteed by the computed duration, but it is re-checked here
      // because operation_end_utc can later be updated from /adjusted_trajectories.
      if (trajectory.operation_end_ns <= trajectory.operation_start_ns) {
        RCLCPP_DEBUG(
          get_logger(),
          "Unvalidated '%s': operation_end_utc is not later than operation_start_utc",
          trajectory.trajectory_id.c_str());
        unvalidated.push_back(&trajectory);
        continue;
      }

      // A periodic definition now represents ONE concrete complete occurrence
      // at a time. /adjusted_trajectories advances start/end to the following
      // occurrence after each takeoff -> mission[] -> landing execution.
      if (trajectory.operation_frequency > 0.0) {
        if (trajectory.total_repetition_exhausted) {
          latest.push_back(&trajectory);
        } else {
          requested.push_back(&trajectory);
        }
        continue;
      }

      // Non-periodic trajectories are current/future only while now is
      // strictly earlier than the (possibly adjusted) operation end.
      if (current_ns < trajectory.operation_end_ns) {
        requested.push_back(&trajectory);
      } else {
        latest.push_back(&trajectory);
      }
    }

    const auto ordering = [](const TrajectoryConfig * a, const TrajectoryConfig * b) {
      if (a->priority != b->priority) {
        return a->priority < b->priority;
      }
      return a->input_index < b->input_index;
    };

    std::stable_sort(requested.begin(), requested.end(), ordering);
    std::stable_sort(unvalidated.begin(), unvalidated.end(), ordering);
    std::stable_sort(latest.begin(), latest.end(), ordering);

    requested_output.trajectories.reserve(requested.size());
    for (const auto * trajectory : requested) {
      requested_output.trajectories.push_back(trajectory_to_message(*trajectory));

      // Markers are intentionally published only for requested trajectories.
      add_markers(*trajectory, requested_output.header.stamp, markers);
    }

    unvalidated_output.trajectories.reserve(unvalidated.size());
    for (const auto * trajectory : unvalidated) {
      unvalidated_output.trajectories.push_back(trajectory_to_message(*trajectory));
    }

    latest_output.trajectories.reserve(latest.size());
    for (const auto * trajectory : latest) {
      latest_output.trajectories.push_back(trajectory_to_message(*trajectory));
    }

    requested_publisher_->publish(requested_output);
    unvalidated_publisher_->publish(unvalidated_output);
    latest_publisher_->publish(latest_output);
    marker_publisher_->publish(markers);

    if (last_requested_count_ != requested.size() ||
      last_unvalidated_count_ != unvalidated.size() ||
      last_latest_count_ != latest.size())
    {
      RCLCPP_INFO(
        get_logger(),
        "Trajectory snapshot | requested=%zu | unvalidated=%zu | latest=%zu | total=%zu",
        requested.size(), unvalidated.size(), latest.size(), trajectories_.size());

      last_requested_count_ = requested.size();
      last_unvalidated_count_ = unvalidated.size();
      last_latest_count_ = latest.size();
    }
  }

  std::string config_file_;
  std::string flight_zones_topic_;
  std::string requested_topic_;
  std::string unvalidated_topic_;
  std::string latest_topic_;
  std::string adjusted_topic_;
  std::string markers_topic_;
  std::string default_frame_id_{"map"};

  int validation_period_ms_{1000};
  double flight_zone_validation_step_m_{0.25};
  double boundary_tolerance_m_{0.03};
  bool boundary_is_inside_{true};
  double marker_line_width_{0.12};
  double marker_text_height_{0.32};

  std::vector<TrajectoryConfig> trajectories_;
  std::map<std::string, PreparedZone> prepared_zones_;
  bool flight_zones_received_{false};

  std::size_t last_requested_count_{std::numeric_limits<std::size_t>::max()};
  std::size_t last_unvalidated_count_{std::numeric_limits<std::size_t>::max()};
  std::size_t last_latest_count_{std::numeric_limits<std::size_t>::max()};

  rclcpp::Publisher<StaticTrajectoryArray>::SharedPtr requested_publisher_;
  rclcpp::Publisher<StaticTrajectoryArray>::SharedPtr unvalidated_publisher_;
  rclcpp::Publisher<StaticTrajectoryArray>::SharedPtr latest_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher_;

  rclcpp::Subscription<FlightZoneArray>::SharedPtr flight_zones_subscription_;
  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr adjusted_subscription_;
  rclcpp::TimerBase::SharedPtr validation_timer_;
};

}  // namespace static_trajectory_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<static_trajectory_manager::StaticTrajectoryManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("static_trajectory_manager_node"),
      "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
