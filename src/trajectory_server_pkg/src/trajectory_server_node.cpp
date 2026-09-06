#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>
#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>

#include <static_trajectory_conflict_manager/msg/collision_static_trajectory.hpp>
#include <static_trajectory_conflict_manager/msg/collision_static_trajectory_array.hpp>
#include <static_trajectory_conflict_manager/msg/segment_collision.hpp>
#include <static_trajectory_conflict_manager/msg/trajectory_collision.hpp>
#include <trajectory_endtime_adjustment/srv/register_original_trajectory.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace trajectory_server_pkg
{

using VehicleStatus = px4_msgs::msg::VehicleStatus;
using DetectedCollisionTrajectory = collision_detection::msg::DetectedCollisionTrajectory;
using DetectedCollisionTrajectoryArray = collision_detection::msg::DetectedCollisionTrajectoryArray;
using StaticTrajectory = static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using TrajectorySegment = static_trajectory_manager::msg::TrajectorySegment;
using CollisionStaticTrajectory =
  static_trajectory_conflict_manager::msg::CollisionStaticTrajectory;
using CollisionStaticTrajectoryArray =
  static_trajectory_conflict_manager::msg::CollisionStaticTrajectoryArray;
using SegmentCollision = static_trajectory_conflict_manager::msg::SegmentCollision;
using TrajectoryCollision = static_trajectory_conflict_manager::msg::TrajectoryCollision;
using RegisterOriginalTrajectory =
  trajectory_endtime_adjustment::srv::RegisterOriginalTrajectory;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr double kGeometryEpsilon = 1.0e-12;

using SteadyTime = std::chrono::steady_clock::time_point;

enum class Phase : uint8_t
{
  TAKEOFF = SegmentCollision::TAKEOFF,
  MISSION = SegmentCollision::MISSION,
  LANDING = SegmentCollision::LANDING
};

enum class PendingMode : uint8_t
{
  STORED,
  WAITING_FOR_ACTIVE_CONFLICTS
};

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GeometrySegment
{
  Vec3 start;
  Vec3 end;
  Phase phase{Phase::MISSION};
  uint32_t phase_segment_index{0U};
};

struct ClosestPoints
{
  Vec3 first;
  Vec3 second;
  double distance_m{std::numeric_limits<double>::infinity()};
};

struct SpatialEvidence
{
  bool collision{false};
  TrajectoryCollision candidate_to_active;
};

struct VehicleStatusWatcher
{
  std::string topic;
  rclcpp::Subscription<VehicleStatus>::SharedPtr subscription;
  bool received{false};
  bool armed{false};
  SteadyTime received_at{};
};

struct PendingTrajectory
{
  std::string source_key;
  StaticTrajectory trajectory;
  int64_t original_start_ns{0};
  int64_t original_end_ns{0};
  PendingMode mode{PendingMode::STORED};
  std::set<std::string> blocker_active_keys;
  int64_t latest_blocker_completion_ns{0};

  // True only when this candidate is STRICTLY higher priority than at least
  // one active spatial blocker. Equal-priority waiting does not use the
  // special end-time-adjustment service.
  bool original_registration_required{false};
  bool original_registration_in_flight{false};
  bool original_registration_accepted{false};
};

struct ActiveTrajectory
{
  std::string source_key;
  StaticTrajectory trajectory;

  // Completion is accepted only for a sufficiently long ARM -> DISARM cycle.
  // A short cycle is treated as a failed/retried takeoff and is invalidated,
  // forcing a new ARM observation before DISARM can complete the trajectory.
  bool armed_seen{false};
  SteadyTime armed_since{};
  uint32_t ignored_short_disarms{0U};
};

static int64_t time_to_ns(const builtin_interfaces::msg::Time & time)
{
  return static_cast<int64_t>(time.sec) * kNanosecondsPerSecond +
         static_cast<int64_t>(time.nanosec);
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

static int64_t system_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

static std::string format_utc_iso8601(const builtin_interfaces::msg::Time & value)
{
  const std::time_t seconds = static_cast<std::time_t>(value.sec);
  std::tm utc{};
  if (gmtime_r(&seconds, &utc) == nullptr) {
    return std::to_string(value.sec) + "." + std::to_string(value.nanosec) + "Z";
  }

  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S");
  if (value.nanosec != 0U) {
    stream << "." << std::setw(9) << std::setfill('0') << value.nanosec;
  }
  stream << "Z";
  return stream.str();
}

static std::string trim_slashes(std::string value)
{
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

static std::string normalise_suffix(std::string value)
{
  return trim_slashes(std::move(value));
}

static std::string trajectory_source_key(const StaticTrajectory & trajectory)
{
  return trajectory.trajectory_id + "@" +
    std::to_string(time_to_ns(trajectory.operation_start_utc));
}

static std::string vehicle_status_topic(
  const StaticTrajectory & trajectory,
  const std::string & suffix)
{
  const std::string zone = trim_slashes(trajectory.flight_zone_id);
  const std::string uas = trim_slashes(trajectory.uas_namespace);
  if (zone.empty() || uas.empty()) {
    return {};
  }
  return "/" + zone + "_" + uas + "/" + normalise_suffix(suffix);
}

static Vec3 subtract(const Vec3 & a, const Vec3 & b)
{
  return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 add(const Vec3 & a, const Vec3 & b)
{
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 multiply(const Vec3 & value, double scalar)
{
  return Vec3{value.x * scalar, value.y * scalar, value.z * scalar};
}

static double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static double norm(const Vec3 & value)
{
  return std::sqrt(dot(value, value));
}

static double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

static Vec3 project_for_distance(Vec3 point, bool use_3d)
{
  if (!use_3d) {
    point.z = 0.0;
  }
  return point;
}

static geometry_msgs::msg::Point to_point(const Vec3 & value)
{
  geometry_msgs::msg::Point result;
  result.x = value.x;
  result.y = value.y;
  result.z = value.z;
  return result;
}

static std::vector<Vec3> segment_points(
  const TrajectorySegment & segment,
  const std::string & trajectory_id,
  const std::string & segment_name,
  bool allow_empty)
{
  if (
    segment.x.size() != segment.y.size() ||
    segment.x.size() != segment.z.size())
  {
    throw std::runtime_error(
      "Trajectory '" + trajectory_id + "': segment '" + segment_name +
      "' has different x/y/z lengths");
  }

  if (segment.x.empty()) {
    if (allow_empty) {
      return {};
    }
    throw std::runtime_error(
      "Trajectory '" + trajectory_id + "': segment '" + segment_name + "' is empty");
  }

  std::vector<Vec3> points;
  points.reserve(segment.x.size());
  for (std::size_t index = 0U; index < segment.x.size(); ++index) {
    const Vec3 point{segment.x[index], segment.y[index], segment.z[index]};
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::runtime_error(
        "Trajectory '" + trajectory_id + "': segment '" + segment_name +
        "' contains non-finite coordinates");
    }
    points.push_back(point);
  }
  return points;
}

static void append_polyline_segments(
  std::vector<GeometrySegment> & target,
  const std::vector<Vec3> & points,
  Phase phase,
  uint32_t & phase_segment_index)
{
  if (points.size() < 2U) {
    return;
  }

  target.reserve(
    target.size() +
    points.size() - 1U);

  for (
    std::size_t index = 1U;
    index < points.size();
    ++index)
  {
    target.push_back(
      GeometrySegment{
        points[index - 1U],
        points[index],
        phase,
        phase_segment_index++});
  }
}

static std::vector<GeometrySegment> build_geometry_segments(
  const StaticTrajectory & trajectory,
  bool allow_empty_phases)
{
  if (trajectory.repetitions == 0U) {
    throw std::runtime_error(
      "Trajectory '" + trajectory.trajectory_id + "': repetitions must be >= 1");
  }

  const auto takeoff =
    segment_points(
      trajectory.takeoff,
      trajectory.trajectory_id,
      "takeoff",
      allow_empty_phases);

  const auto landing =
    segment_points(
      trajectory.landing,
      trajectory.trajectory_id,
      "landing",
      allow_empty_phases);

  if (
    trajectory.mission.empty() &&
    !allow_empty_phases)
  {
    throw std::runtime_error(
      "Trajectory '" +
      trajectory.trajectory_id +
      "': mission array is empty");
  }

  std::vector<
    std::vector<Vec3>> missions;

  missions.reserve(
    trajectory.mission.size());

  for (
    std::size_t mission_index = 0U;
    mission_index < trajectory.mission.size();
    ++mission_index)
  {
    missions.push_back(
      segment_points(
        trajectory.mission[mission_index],
        trajectory.trajectory_id,
        "mission[" +
        std::to_string(mission_index) +
        "]",
        allow_empty_phases));
  }

  std::vector<GeometrySegment> segments;

  std::size_t geometry_segment_count = 0U;

  if (takeoff.size() >= 2U) {
    geometry_segment_count +=
      takeoff.size() - 1U;
  }

  for (const auto & mission : missions) {
    if (mission.size() >= 2U) {
      geometry_segment_count +=
        mission.size() - 1U;
    }
  }

  if (landing.size() >= 2U) {
    geometry_segment_count +=
      landing.size() - 1U;
  }

  segments.reserve(
    geometry_segment_count);

  uint32_t takeoff_segment_index = 0U;
  uint32_t mission_segment_index = 0U;
  uint32_t landing_segment_index = 0U;

  // Every TrajectorySegment is an independent continuous polyline.
  //
  // In particular, there is NO implicit spatial edge between:
  //   takeoff.back()        -> mission[0].front()
  //   mission[i].back()     -> mission[i + 1].front()
  //   mission.back().back() -> landing.front()
  //
  // This is required for supervised crops: separate collision-free mission
  // pieces must remain spatially disconnected.
  append_polyline_segments(
    segments,
    takeoff,
    Phase::TAKEOFF,
    takeoff_segment_index);

  for (const auto & mission : missions) {
    append_polyline_segments(
      segments,
      mission,
      Phase::MISSION,
      mission_segment_index);
  }

  append_polyline_segments(
    segments,
    landing,
    Phase::LANDING,
    landing_segment_index);

  // Partial repetitions repeat the same mission collection geometrically.
  // Runtime arbitration only needs unique spatial geometry. Duplicating the
  // same edges repetitions times would multiply identical collision evidence
  // without changing the collision decision.
  return segments;
}

static ClosestPoints closest_points_on_segments(
  const GeometrySegment & first,
  const GeometrySegment & second,
  bool use_3d)
{
  const Vec3 first_start_projected = project_for_distance(first.start, use_3d);
  const Vec3 first_end_projected = project_for_distance(first.end, use_3d);
  const Vec3 second_start_projected = project_for_distance(second.start, use_3d);
  const Vec3 second_end_projected = project_for_distance(second.end, use_3d);

  const Vec3 d1 = subtract(first_end_projected, first_start_projected);
  const Vec3 d2 = subtract(second_end_projected, second_start_projected);
  const Vec3 r = subtract(first_start_projected, second_start_projected);

  const double a = dot(d1, d1);
  const double e = dot(d2, d2);
  const double f = dot(d2, r);

  double s = 0.0;
  double t = 0.0;

  if (a <= kGeometryEpsilon && e <= kGeometryEpsilon) {
    s = 0.0;
    t = 0.0;
  } else if (a <= kGeometryEpsilon) {
    s = 0.0;
    t = clamp(f / e, 0.0, 1.0);
  } else {
    const double c = dot(d1, r);
    if (e <= kGeometryEpsilon) {
      t = 0.0;
      s = clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = dot(d1, d2);
      const double denominator = a * e - b * b;
      if (std::abs(denominator) > kGeometryEpsilon) {
        s = clamp((b * f - c * e) / denominator, 0.0, 1.0);
      }
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }

  const Vec3 first_actual = add(first.start, multiply(subtract(first.end, first.start), s));
  const Vec3 second_actual = add(second.start, multiply(subtract(second.end, second.start), t));

  const Vec3 first_distance = project_for_distance(first_actual, use_3d);
  const Vec3 second_distance = project_for_distance(second_actual, use_3d);

  ClosestPoints result;
  result.first = first_actual;
  result.second = second_actual;
  result.distance_m = norm(subtract(first_distance, second_distance));
  return result;
}

static uint8_t phase_value(Phase phase)
{
  return static_cast<uint8_t>(phase);
}


static std_msgs::msg::ColorRGBA color_from_trajectory_id(const std::string & trajectory_id)
{
  uint32_t hash = 2166136261U;
  for (const unsigned char character : trajectory_id) {
    hash ^= static_cast<uint32_t>(character);
    hash *= 16777619U;
  }

  const double h = static_cast<double>(hash % 10000U) / 10000.0;
  const double saturation = 0.72;
  const double value = 0.95;
  const double c = value * saturation;
  const double x = c * (1.0 - std::abs(std::fmod(h * 6.0, 2.0) - 1.0));
  const double m = value - c;

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

class TrajectoryServerNode : public rclcpp::Node
{
public:
  TrajectoryServerNode()
  : Node("trajectory_server_node")
  {
    available_topic_ = declare_parameter<std::string>(
      "available_static_trajectories_topic", "/available_static_trajectories");
    active_topic_ = declare_parameter<std::string>(
      "active_trajectories_topic", "/active_trajectories");
    supervised_topic_ = declare_parameter<std::string>(
      "supervised_trajectories_topic", "/supervised_trajectories");
    non_priority_adjustment_topic_ = declare_parameter<std::string>(
      "non_priority_adjustment_trajectories_topic",
      "/non_priority_adjustment_trajectories");
    active_markers_topic_ = declare_parameter<std::string>(
      "active_trajectories_markers_topic", "/active_trajectories_markers");

    original_trajectory_registration_service_ = declare_parameter<std::string>(
      "original_trajectory_registration_service",
      "/trajectory_endtime_adjustment/register_original_trajectory");

    storage_lead_time_s_ = declare_parameter<double>("storage_lead_time_s", 60.0);
    publication_lead_time_s_ = declare_parameter<double>("publication_lead_time_s", 30.0);
    reschedule_margin_s_ = declare_parameter<double>("reschedule_margin_s", 5.0);
    preserve_duration_on_reschedule_ = declare_parameter<bool>(
      "preserve_duration_on_reschedule", true);

    vehicle_status_suffix_ = declare_parameter<std::string>(
      "vehicle_status_suffix", "fmu/out/vehicle_status");
    vehicle_status_timeout_s_ = declare_parameter<double>(
      "vehicle_status_timeout_s", 2.0);

    // A DISARM shortly after ARM may correspond to a failed takeoff followed
    // by PX4 auto-disarm/failsafe recovery. Such a DISARM must not remove the
    // active trajectory because the UAS will retry the same occurrence.
    //
    // Set to 0.0 to recover the legacy "first ARM -> DISARM completes" behavior.
    minimum_armed_duration_for_completion_s_ =
      declare_parameter<double>(
      "minimum_armed_duration_for_completion_s",
      20.0);

    spatial_conflict_distance_m_ = declare_parameter<double>(
      "spatial_conflict_distance_m", 1.0);
    use_3d_ = declare_parameter<bool>("use_3d", true);
    max_segment_collisions_per_pair_ = declare_parameter<int>(
      "max_segment_collisions_per_pair", 64);

    supervision_period_ms_ = declare_parameter<int>("supervision_period_ms", 200);
    marker_line_width_ = declare_parameter<double>("marker_line_width", 0.14);
    marker_text_height_ = declare_parameter<double>("marker_text_height", 0.34);

    validate_parameters();
    vehicle_status_suffix_ = normalise_suffix(vehicle_status_suffix_);

    auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    retained_qos.reliable();
    retained_qos.transient_local();

    available_subscription_ = create_subscription<StaticTrajectoryArray>(
      available_topic_,
      retained_qos,
      std::bind(
        &TrajectoryServerNode::available_callback,
        this,
        std::placeholders::_1));

    supervised_subscription_ = create_subscription<DetectedCollisionTrajectoryArray>(
      supervised_topic_,
      retained_qos,
      std::bind(
        &TrajectoryServerNode::supervised_callback,
        this,
        std::placeholders::_1));

    active_publisher_ = create_publisher<StaticTrajectoryArray>(
      active_topic_, retained_qos);
    active_markers_publisher_ = create_publisher<MarkerArray>(
      active_markers_topic_, retained_qos);

    // This is an event stream produced by runtime priority arbitration.
    // It must NOT be published into /collision_static_trajectories, whose
    // ownership belongs to static_trajectory_conflict_manager.
    auto adjustment_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    adjustment_qos.reliable();
    non_priority_adjustment_publisher_ =
      create_publisher<CollisionStaticTrajectoryArray>(
        non_priority_adjustment_topic_, adjustment_qos);

    original_registration_client_ =
      create_client<RegisterOriginalTrajectory>(
        original_trajectory_registration_service_);

    supervision_timer_ = create_wall_timer(
      std::chrono::milliseconds(supervision_period_ms_),
      std::bind(&TrajectoryServerNode::supervision_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Trajectory server ready | input='%s' | supervised='%s' | active='%s' | "
      "non_priority_adjustment='%s' | registration_service='%s' | "
      "store=T-%.1fs | publish=T-%.1fs | "
      "reschedule margin=%.1fs | vehicle_status='%s' | "
      "min_ARM_duration_for_completion=%.1fs | spatial threshold=%.3fm | "
      "use_3d=%s | extra_time calculation=disabled",
      available_topic_.c_str(),
      supervised_topic_.c_str(),
      active_topic_.c_str(),
      non_priority_adjustment_topic_.c_str(),
      original_trajectory_registration_service_.c_str(),
      storage_lead_time_s_,
      publication_lead_time_s_,
      reschedule_margin_s_,
      vehicle_status_suffix_.c_str(),
      minimum_armed_duration_for_completion_s_,
      spatial_conflict_distance_m_,
      use_3d_ ? "true" : "false");
  }

private:
  void validate_parameters() const
  {
    const auto absolute = [](const std::string & value) {
        return !value.empty() && value.front() == '/';
      };

    if (!absolute(available_topic_) || !absolute(supervised_topic_) ||
      !absolute(active_topic_) || !absolute(non_priority_adjustment_topic_) ||
      !absolute(active_markers_topic_) ||
      !absolute(original_trajectory_registration_service_))
    {
      throw std::runtime_error(
        "Trajectory server topic/service names must be absolute");
    }

    if (!std::isfinite(storage_lead_time_s_) || storage_lead_time_s_ < 0.0 ||
      !std::isfinite(publication_lead_time_s_) || publication_lead_time_s_ < 0.0 ||
      storage_lead_time_s_ < publication_lead_time_s_)
    {
      throw std::runtime_error(
        "storage_lead_time_s must be >= publication_lead_time_s and both must be non-negative");
    }

    if (!std::isfinite(reschedule_margin_s_) || reschedule_margin_s_ < 0.0) {
      throw std::runtime_error("reschedule_margin_s must be finite and non-negative");
    }
    if (normalise_suffix(vehicle_status_suffix_).empty()) {
      throw std::runtime_error("vehicle_status_suffix must not be empty");
    }
    if (!std::isfinite(vehicle_status_timeout_s_) || vehicle_status_timeout_s_ <= 0.0) {
      throw std::runtime_error("vehicle_status_timeout_s must be finite and > 0");
    }
    if (
      !std::isfinite(minimum_armed_duration_for_completion_s_) ||
      minimum_armed_duration_for_completion_s_ < 0.0)
    {
      throw std::runtime_error(
              "minimum_armed_duration_for_completion_s must be finite and >= 0");
    }
    if (!std::isfinite(spatial_conflict_distance_m_) || spatial_conflict_distance_m_ < 0.0) {
      throw std::runtime_error("spatial_conflict_distance_m must be finite and >= 0");
    }
    if (max_segment_collisions_per_pair_ <= 0) {
      throw std::runtime_error("max_segment_collisions_per_pair must be > 0");
    }
    if (supervision_period_ms_ <= 0) {
      throw std::runtime_error("supervision_period_ms must be > 0");
    }
    if (!std::isfinite(marker_line_width_) || marker_line_width_ <= 0.0 ||
      !std::isfinite(marker_text_height_) || marker_text_height_ <= 0.0)
    {
      throw std::runtime_error("marker sizes must be finite and > 0");
    }
  }

  static bool valid_trajectory_identity_and_time(const StaticTrajectory & trajectory)
  {
    return !trajectory.trajectory_id.empty() &&
      !trajectory.flight_zone_id.empty() &&
      !trajectory.uas_namespace.empty() &&
      time_to_ns(trajectory.operation_end_utc) > time_to_ns(trajectory.operation_start_utc);
  }

  static bool segment_coordinates_valid(
    const TrajectorySegment & segment,
    bool allow_empty)
  {
    if (
      segment.x.size() != segment.y.size() ||
      segment.x.size() != segment.z.size())
    {
      return false;
    }

    if (!allow_empty && segment.x.empty()) {
      return false;
    }

    for (std::size_t index = 0U; index < segment.x.size(); ++index) {
      if (
        !std::isfinite(segment.x[index]) ||
        !std::isfinite(segment.y[index]) ||
        !std::isfinite(segment.z[index]))
      {
        return false;
      }
    }

    return true;
  }

  static bool mission_coordinates_valid(
    const std::vector<TrajectorySegment> & missions,
    bool allow_empty_phase)
  {
    if (missions.empty()) {
      return allow_empty_phase;
    }

    for (const auto & mission : missions) {
      // An empty mission component has no useful semantics. A fully removed
      // MISSION phase is represented by an empty mission array instead.
      if (!segment_coordinates_valid(mission, false)) {
        return false;
      }
    }

    return true;
  }

  bool is_supervised_id_locked(const std::string & trajectory_id) const
  {
    return latest_supervised_.find(trajectory_id) != latest_supervised_.end();
  }

  bool geometry_allowed_locked(const StaticTrajectory & trajectory) const
  {
    const bool supervised = is_supervised_id_locked(trajectory.trajectory_id);

    if (trajectory.repetitions == 0U) {
      return false;
    }

    // Normal trajectories keep the strict requirement: TAKEOFF, at least one
    // MISSION component and LANDING must contain valid geometry.
    //
    // A supervised trajectory is special because its AVAILABLE copy is the
    // deliberately cropped version. TAKEOFF or LANDING may be empty and the
    // complete MISSION phase may be represented by an empty mission array.
    // If mission components are present, each component must itself be valid.
    return
      segment_coordinates_valid(trajectory.takeoff, supervised) &&
      mission_coordinates_valid(trajectory.mission, supervised) &&
      segment_coordinates_valid(trajectory.landing, supervised);
  }

  void rebuild_available_locked()
  {
    if (!available_received_) {
      return;
    }

    std::map<std::string, StaticTrajectory> next;

    for (const auto & trajectory : raw_available_.trajectories) {
      if (!valid_trajectory_identity_and_time(trajectory)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Ignoring malformed trajectory in /available_static_trajectories");
        continue;
      }

      if (!geometry_allowed_locked(trajectory)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Ignoring trajectory '%s': incomplete/invalid geometry is only allowed "
          "while the trajectory is present in /supervised_trajectories",
          trajectory.trajectory_id.c_str());
        continue;
      }

      next[trajectory_source_key(trajectory)] = trajectory;
    }

    latest_available_ = std::move(next);

    // A completed/rejected occurrence may remain in the retained upstream
    // snapshot for a short time. Forget the guard only once that source
    // occurrence truly disappears from the accepted input set.
    for (auto it = completed_source_keys_.begin(); it != completed_source_keys_.end();) {
      if (latest_available_.find(*it) == latest_available_.end()) {
        it = completed_source_keys_.erase(it);
      } else {
        ++it;
      }
    }

    for (auto it = rejected_source_keys_.begin(); it != rejected_source_keys_.end();) {
      if (latest_available_.find(*it) == latest_available_.end()) {
        it = rejected_source_keys_.erase(it);
      } else {
        ++it;
      }
    }

    // Before a candidate becomes a waiting operation it remains authoritative
    // to the input snapshot. Therefore refresh it if its metadata changes and
    // remove it if upstream withdraws it. Waiting candidates are deliberately
    // retained because they may outlive their original planned interval.
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (it->second.mode == PendingMode::WAITING_FOR_ACTIVE_CONFLICTS) {
        ++it;
        continue;
      }

      const auto source = latest_available_.find(it->first);
      if (source == latest_available_.end()) {
        RCLCPP_INFO(
          get_logger(),
          "Removing stored trajectory '%s': no longer accepted from "
          "/available_static_trajectories",
          it->second.trajectory.trajectory_id.c_str());
        it = pending_.erase(it);
        continue;
      }

      it->second.trajectory = source->second;
      it->second.original_start_ns = time_to_ns(source->second.operation_start_utc);
      it->second.original_end_ns = time_to_ns(source->second.operation_end_utc);
      ++it;
    }
  }

  void available_callback(const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_header_ = message->header;
    raw_available_ = *message;
    available_received_ = true;
    rebuild_available_locked();
  }

  void supervised_callback(const DetectedCollisionTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    std::map<std::string, DetectedCollisionTrajectory> next;
    for (const auto & detected : message->trajectories) {
      const auto & id = detected.trajectory.trajectory_id;
      if (id.empty()) {
        continue;
      }
      next[id] = detected;
    }

    latest_supervised_ = std::move(next);

    // The retained inputs may arrive in either order at node startup. Rebuild
    // the AVAILABLE view whenever supervision state changes so an incomplete
    // supervised trajectory is not lost merely because AVAILABLE arrived first.
    rebuild_available_locked();
  }

  void ensure_vehicle_status_watcher_locked(const StaticTrajectory & trajectory)
  {
    const std::string topic = vehicle_status_topic(trajectory, vehicle_status_suffix_);
    if (topic.empty() || vehicle_watchers_.find(topic) != vehicle_watchers_.end()) {
      return;
    }

    VehicleStatusWatcher watcher;
    watcher.topic = topic;
    watcher.subscription = create_subscription<VehicleStatus>(
      topic,
      rclcpp::SensorDataQoS(),
      [this, topic](const VehicleStatus::SharedPtr message) {
        if (!message) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = vehicle_watchers_.find(topic);
        if (it == vehicle_watchers_.end()) {
          return;
        }
        it->second.received = true;
        it->second.armed =
          message->arming_state == VehicleStatus::ARMING_STATE_ARMED;
        it->second.received_at = std::chrono::steady_clock::now();
      });

    vehicle_watchers_.emplace(topic, std::move(watcher));
    RCLCPP_INFO(get_logger(), "Watching VehicleStatus '%s'", topic.c_str());
  }

  bool fresh_armed_state_locked(const StaticTrajectory & trajectory, bool & armed) const
  {
    const std::string topic = vehicle_status_topic(trajectory, vehicle_status_suffix_);
    const auto it = vehicle_watchers_.find(topic);
    if (it == vehicle_watchers_.end() || !it->second.received) {
      return false;
    }

    const double age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - it->second.received_at).count();
    if (age_s > vehicle_status_timeout_s_) {
      return false;
    }

    armed = it->second.armed;
    return true;
  }

  SpatialEvidence spatial_collision(
    const StaticTrajectory & candidate,
    const StaticTrajectory & active) const
  {
    SpatialEvidence result;
    result.candidate_to_active.conflicting_trajectory_id = active.trajectory_id;
    result.candidate_to_active.conflicting_priority = active.priority;

    if (candidate.frame_id != active.frame_id) {
      return result;
    }

    const auto candidate_segments = build_geometry_segments(
      candidate, is_supervised_id_locked(candidate.trajectory_id));
    const auto active_segments = build_geometry_segments(
      active, is_supervised_id_locked(active.trajectory_id));

    for (const auto & candidate_segment : candidate_segments) {
      for (const auto & active_segment : active_segments) {
        const ClosestPoints closest = closest_points_on_segments(
          candidate_segment, active_segment, use_3d_);

        if (closest.distance_m > spatial_conflict_distance_m_ + 1.0e-9) {
          continue;
        }

        result.collision = true;
        if (
          static_cast<int>(result.candidate_to_active.segment_collisions.size()) >=
          max_segment_collisions_per_pair_)
        {
          continue;
        }

        SegmentCollision evidence;
        evidence.own_phase = phase_value(candidate_segment.phase);
        evidence.own_segment_index = candidate_segment.phase_segment_index;
        evidence.own_closest_point = to_point(closest.first);
        evidence.conflicting_phase = phase_value(active_segment.phase);
        evidence.conflicting_segment_index = active_segment.phase_segment_index;
        evidence.conflicting_closest_point = to_point(closest.second);
        evidence.minimum_distance_m = closest.distance_m;
        result.candidate_to_active.segment_collisions.push_back(std::move(evidence));
      }
    }

    return result;
  }

  void mark_active_completions_locked(int64_t current_ns)
  {
    const SteadyTime steady_now =
      std::chrono::steady_clock::now();

    for (auto it = active_.begin(); it != active_.end();) {
      auto & state = it->second;
      ensure_vehicle_status_watcher_locked(state.trajectory);

      bool armed = false;
      const bool fresh =
        fresh_armed_state_locked(
        state.trajectory,
        armed);

      // --------------------------------------------------------------
      // ARM observation
      // --------------------------------------------------------------
      // Start the minimum-duration timer only when a NEW arm cycle is seen.
      // Repeated ARMED samples during the same cycle do not reset it.
      if (fresh && armed) {
        if (!state.armed_seen) {
          state.armed_seen = true;
          state.armed_since = steady_now;

          RCLCPP_INFO(
            get_logger(),
            "Active trajectory '%s': ARM observed; completion DISARM will be "
            "accepted after at least %.1f s of this arm cycle",
            state.trajectory.trajectory_id.c_str(),
            minimum_armed_duration_for_completion_s_);
        }

        ++it;
        continue;
      }

      // Initial DISARMED samples before the operation actually arms are not a
      // completion event. Likewise, after a short failed ARM -> DISARM cycle,
      // armed_seen is cleared and the server waits for the retry's NEW ARM.
      if (!(fresh && !armed && state.armed_seen)) {
        ++it;
        continue;
      }

      // --------------------------------------------------------------
      // DISARM after a previously observed ARM
      // --------------------------------------------------------------
      const double armed_duration_s =
        std::chrono::duration<double>(
        steady_now -
        state.armed_since).count();

      if (
        armed_duration_s <
        minimum_armed_duration_for_completion_s_)
      {
        ++state.ignored_short_disarms;

        RCLCPP_WARN(
          get_logger(),
          "Active trajectory '%s': ignoring DISARMED after only %.3f s ARMED "
          "(minimum %.3f s) | interpreted as failed/retried takeoff | "
          "ignored_short_disarms=%u | waiting for a NEW ARM cycle",
          state.trajectory.trajectory_id.c_str(),
          armed_duration_s,
          minimum_armed_duration_for_completion_s_,
          state.ignored_short_disarms);

        // CRITICAL: invalidate this arm cycle. Otherwise a UAS remaining
        // DISARMED on the ground could eventually be mistaken for completion
        // once wall-clock time exceeds the threshold. The retry must produce a
        // NEW ARMED observation, which starts a fresh timer.
        state.armed_seen = false;
        state.armed_since = SteadyTime{};

        ++it;
        continue;
      }

      // This is the first DISARM belonging to an arm cycle long enough to be
      // interpreted as the real end of the operation.
      const std::string completed_key =
        it->first;
      const std::string completed_id =
        state.trajectory.trajectory_id;

      completed_source_keys_.insert(
        state.source_key);

      for (auto & [_, pending] : pending_) {
        const auto blocker =
          pending.blocker_active_keys.find(
          completed_key);

        if (
          blocker ==
          pending.blocker_active_keys.end())
        {
          continue;
        }

        pending.blocker_active_keys.erase(
          blocker);

        pending.latest_blocker_completion_ns =
          std::max(
          pending.latest_blocker_completion_ns,
          current_ns);
      }

      RCLCPP_INFO(
        get_logger(),
        "Active trajectory '%s' completed: DISARMED observed after %.3f s "
        "of the latest ARM cycle (minimum %.3f s) | ignored_short_disarms=%u",
        completed_id.c_str(),
        armed_duration_s,
        minimum_armed_duration_for_completion_s_,
        state.ignored_short_disarms);

      it = active_.erase(it);
    }
  }

  void store_due_candidates_locked(int64_t current_ns)
  {
    const int64_t storage_lead_ns = static_cast<int64_t>(std::llround(
      storage_lead_time_s_ * static_cast<double>(kNanosecondsPerSecond)));

    for (const auto & [source_key, trajectory] : latest_available_) {
      if (pending_.find(source_key) != pending_.end() ||
        active_.find(source_key) != active_.end() ||
        completed_source_keys_.count(source_key) != 0U ||
        rejected_source_keys_.count(source_key) != 0U)
      {
        continue;
      }

      const int64_t start_ns = time_to_ns(trajectory.operation_start_utc);
      const int64_t end_ns = time_to_ns(trajectory.operation_end_utc);
      if (current_ns < start_ns - storage_lead_ns) {
        continue;
      }

      // Do not resurrect an occurrence that was never stored before its whole
      // planned interval had already elapsed. Waiting/active occurrences are
      // handled independently once they have entered server state.
      if (current_ns >= end_ns) {
        continue;
      }

      PendingTrajectory pending;
      pending.source_key = source_key;
      pending.trajectory = trajectory;
      pending.original_start_ns = start_ns;
      pending.original_end_ns = end_ns;
      pending.mode = PendingMode::STORED;
      pending_.emplace(source_key, std::move(pending));
      ensure_vehicle_status_watcher_locked(trajectory);

      RCLCPP_INFO(
        get_logger(),
        "Stored trajectory '%s' at T-%.1fs | start=%s",
        trajectory.trajectory_id.c_str(),
        storage_lead_time_s_,
        format_utc_iso8601(trajectory.operation_start_utc).c_str());
    }
  }

  void reject_candidate_locked(
    const std::string & source_key,
    const PendingTrajectory & pending,
    std::vector<TrajectoryCollision> collisions,
    std::vector<CollisionStaticTrajectory> & adjustment_events)
  {
    CollisionStaticTrajectory event;
    event.trajectory = pending.trajectory;
    event.collisions = std::move(collisions);
    adjustment_events.push_back(std::move(event));
    rejected_source_keys_.insert(source_key);

    RCLCPP_WARN(
      get_logger(),
      "Trajectory '%s' sent to /non_priority_adjustment_trajectories: "
      "an active spatially conflicting trajectory has strictly higher priority",
      pending.trajectory.trajectory_id.c_str());
  }

  void activate_candidate_locked(
    const std::string & source_key,
    PendingTrajectory & pending,
    int64_t current_ns)
  {
    if (pending.latest_blocker_completion_ns > 0) {
      const int64_t margin_ns = static_cast<int64_t>(std::llround(
        reschedule_margin_s_ * static_cast<double>(kNanosecondsPerSecond)));
      const int64_t requested_start_ns = pending.latest_blocker_completion_ns + margin_ns;
      const int64_t adjusted_start_ns = std::max(
        pending.original_start_ns,
        requested_start_ns);

      if (adjusted_start_ns != time_to_ns(pending.trajectory.operation_start_utc)) {
        const int64_t old_start_ns = time_to_ns(pending.trajectory.operation_start_utc);
        const int64_t old_end_ns = time_to_ns(pending.trajectory.operation_end_utc);
        const int64_t duration_ns = old_end_ns - old_start_ns;

        pending.trajectory.operation_start_utc = ns_to_time(adjusted_start_ns);
        if (preserve_duration_on_reschedule_ && duration_ns > 0) {
          pending.trajectory.operation_end_utc = ns_to_time(adjusted_start_ns + duration_ns);
        }

        RCLCPP_WARN(
          get_logger(),
          "Trajectory '%s' rescheduled after lower-priority active conflict | "
          "new_start=%s | margin=%.1fs | published immediately",
          pending.trajectory.trajectory_id.c_str(),
          format_utc_iso8601(pending.trajectory.operation_start_utc).c_str(),
          reschedule_margin_s_);
      }
    }

    // The active snapshot represents operations authorised by this server.
    // Any upstream spatial annotation is cleared because this candidate has
    // just passed runtime arbitration against all currently active trajectories.
    pending.trajectory.spacial_conflict = false;
    pending.trajectory.spacial_conflicting_trajectory_id.clear();

    ActiveTrajectory active;
    active.source_key = source_key;
    active.trajectory = pending.trajectory;

    bool armed = false;
    if (fresh_armed_state_locked(active.trajectory, armed) && armed) {
      // If the trajectory becomes ACTIVE while VehicleStatus is already ARMED,
      // conservatively start the minimum-duration clock now. We cannot infer
      // an earlier ARM transition from a single retained/current status sample.
      active.armed_seen = true;
      active.armed_since = std::chrono::steady_clock::now();
    }

    active_[source_key] = std::move(active);

    RCLCPP_INFO(
      get_logger(),
      "Trajectory '%s' entered /active_trajectories | scheduled_start=%s | now_offset=%.3fs",
      pending.trajectory.trajectory_id.c_str(),
      format_utc_iso8601(pending.trajectory.operation_start_utc).c_str(),
      static_cast<double>(current_ns - time_to_ns(pending.trajectory.operation_start_utc)) /
      static_cast<double>(kNanosecondsPerSecond));
  }

  void request_original_registration_locked(
    const std::string & source_key,
    PendingTrajectory & pending)
  {
    if (
      !pending.original_registration_required ||
      pending.original_registration_accepted ||
      pending.original_registration_in_flight)
    {
      return;
    }

    if (!original_registration_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot register original trajectory '%s': service '%s' is unavailable",
        pending.trajectory.trajectory_id.c_str(),
        original_trajectory_registration_service_.c_str());
      return;
    }

    auto request = std::make_shared<RegisterOriginalTrajectory::Request>();

    // IMPORTANT: pending.trajectory is still the ORIGINAL upstream trajectory
    // here. operation_start_utc/end_utc have not yet been rescheduled.
    request->trajectory = pending.trajectory;
    pending.original_registration_in_flight = true;

    RCLCPP_INFO(
      get_logger(),
      "Registering original trajectory '%s' before delayed high-priority activation | "
      "original_start=%s",
      pending.trajectory.trajectory_id.c_str(),
      format_utc_iso8601(pending.trajectory.operation_start_utc).c_str());

    try {
      original_registration_client_->async_send_request(
        request,
        [this, source_key](
          rclcpp::Client<RegisterOriginalTrajectory>::SharedFuture future)
        {
          bool accepted = false;
          std::string detail;

          try {
            const auto response = future.get();
            accepted = response && response->accepted;
            detail = response ? response->message : "null response";
          } catch (const std::exception & error) {
            detail = error.what();
          }

          std::lock_guard<std::mutex> lock(mutex_);
          const auto iterator = pending_.find(source_key);
          if (iterator == pending_.end()) {
            return;
          }

          iterator->second.original_registration_in_flight = false;
          if (accepted) {
            iterator->second.original_registration_accepted = true;
            RCLCPP_INFO(
              get_logger(),
              "Original trajectory '%s' registered successfully in "
              "trajectory_endtime_adjustment",
              iterator->second.trajectory.trajectory_id.c_str());
          } else {
            RCLCPP_WARN(
              get_logger(),
              "Original trajectory '%s' registration rejected/failed: %s",
              iterator->second.trajectory.trajectory_id.c_str(),
              detail.c_str());
          }
        });
    } catch (const std::exception & error) {
      pending.original_registration_in_flight = false;
      RCLCPP_ERROR(
        get_logger(),
        "Could not send original trajectory registration for '%s': %s",
        pending.trajectory.trajectory_id.c_str(),
        error.what());
    }
  }

  void process_pending_locked(
    int64_t current_ns,
    std::vector<CollisionStaticTrajectory> & adjustment_events)
  {
    const int64_t publication_lead_ns = static_cast<int64_t>(std::llround(
      publication_lead_time_s_ * static_cast<double>(kNanosecondsPerSecond)));

    std::vector<std::string> due_keys;
    due_keys.reserve(pending_.size());

    for (const auto & [source_key, pending] : pending_) {
      if (pending.mode == PendingMode::WAITING_FOR_ACTIVE_CONFLICTS) {
        due_keys.push_back(source_key);
        continue;
      }

      const int64_t start_ns = time_to_ns(pending.trajectory.operation_start_utc);
      if (current_ns >= start_ns - publication_lead_ns) {
        due_keys.push_back(source_key);
      }
    }

    std::stable_sort(
      due_keys.begin(), due_keys.end(),
      [this](const std::string & first_key, const std::string & second_key) {
        const auto & first = pending_.at(first_key).trajectory;
        const auto & second = pending_.at(second_key).trajectory;

        if (first.priority != second.priority) {
          return first.priority < second.priority;
        }
        if (first.trajectory_id != second.trajectory_id) {
          return first.trajectory_id < second.trajectory_id;
        }
        return time_to_ns(first.operation_start_utc) <
               time_to_ns(second.operation_start_utc);
      });

    for (const auto & source_key : due_keys) {
      auto pending_it = pending_.find(source_key);
      if (pending_it == pending_.end()) {
        continue;
      }

      auto & pending = pending_it->second;

      std::vector<TrajectoryCollision> rejecting_collisions;
      std::set<std::string> current_wait_blockers;
      bool has_strictly_lower_priority_blocker = false;

      try {
        for (const auto & [active_key, active_state] : active_) {
          if (active_key == source_key) {
            continue;
          }

          const SpatialEvidence evidence = spatial_collision(
            pending.trajectory,
            active_state.trajectory);

          if (!evidence.collision) {
            continue;
          }

          // Lower numerical value = higher priority.
          //
          // Active strictly higher priority -> candidate is rejected.
          if (active_state.trajectory.priority < pending.trajectory.priority) {
            rejecting_collisions.push_back(evidence.candidate_to_active);
            continue;
          }

          // Equal priority -> candidate waits. No special registration service.
          if (active_state.trajectory.priority == pending.trajectory.priority) {
            current_wait_blockers.insert(active_key);
            continue;
          }

          // Candidate strictly higher priority -> candidate waits for the
          // lower-priority active UAS to finish, and its ORIGINAL trajectory
          // must be registered in trajectory_endtime_adjustment before the
          // rescheduled copy can be published in /active_trajectories.
          current_wait_blockers.insert(active_key);
          has_strictly_lower_priority_blocker = true;
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(),
          "Cannot evaluate runtime spatial conflict for trajectory '%s': %s",
          pending.trajectory.trajectory_id.c_str(),
          error.what());
        continue;
      }

      if (!rejecting_collisions.empty()) {
        reject_candidate_locked(
          source_key,
          pending,
          std::move(rejecting_collisions),
          adjustment_events);
        pending_.erase(pending_it);
        continue;
      }

      if (!current_wait_blockers.empty()) {
        const bool first_wait =
          pending.mode != PendingMode::WAITING_FOR_ACTIVE_CONFLICTS;

        pending.mode = PendingMode::WAITING_FOR_ACTIVE_CONFLICTS;
        pending.blocker_active_keys.insert(
          current_wait_blockers.begin(),
          current_wait_blockers.end());

        if (has_strictly_lower_priority_blocker) {
          pending.original_registration_required = true;
          request_original_registration_locked(source_key, pending);
        }

        if (first_wait) {
          RCLCPP_WARN(
            get_logger(),
            "Trajectory '%s' waits for %zu active spatial conflict(s) | "
            "candidate priority=%d | original registration=%s",
            pending.trajectory.trajectory_id.c_str(),
            current_wait_blockers.size(),
            pending.trajectory.priority,
            pending.original_registration_required ? "required" : "not required");
        }

        continue;
      }

      // Previously registered blockers are cleared only by ARM -> DISARM.
      if (!pending.blocker_active_keys.empty()) {
        continue;
      }

      // If this candidate was strictly higher priority than a blocker, do not
      // publish the rescheduled copy until trajectory_endtime_adjustment has
      // positively acknowledged storage of the ORIGINAL trajectory. This
      // guarantees the service state exists before the first active snapshot. The adjustment node will later compare the real ACTIVE duration with the original duration; the calendar start delay itself is not extra_time.
      if (
        pending.original_registration_required &&
        !pending.original_registration_accepted)
      {
        request_original_registration_locked(source_key, pending);
        continue;
      }

      activate_candidate_locked(source_key, pending, current_ns);
      pending_.erase(pending_it);
    }
  }

  void prune_vehicle_watchers_locked()
  {
    std::set<std::string> required;
    for (const auto & [_, pending] : pending_) {
      required.insert(vehicle_status_topic(pending.trajectory, vehicle_status_suffix_));
    }
    for (const auto & [_, active] : active_) {
      required.insert(vehicle_status_topic(active.trajectory, vehicle_status_suffix_));
    }

    for (auto it = vehicle_watchers_.begin(); it != vehicle_watchers_.end();) {
      if (required.find(it->first) == required.end()) {
        it = vehicle_watchers_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void add_active_markers(
    const StaticTrajectory & trajectory,
    const rclcpp::Time & stamp,
    MarkerArray & marker_array) const
  {
    const bool supervised =
      is_supervised_id_locked(
      trajectory.trajectory_id);

    std::vector<GeometrySegment> geometry;

    try {
      geometry =
        build_geometry_segments(
        trajectory,
        supervised);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(),
        "Could not create ACTIVE marker for '%s': %s",
        trajectory.trajectory_id.c_str(),
        error.what());

      return;
    }

    if (geometry.empty()) {
      return;
    }

    const auto marker_color =
      color_from_trajectory_id(
      trajectory.trajectory_id);

    const std::string frame_id =
      trajectory.frame_id.empty() ?
      "map" :
      trajectory.frame_id;

    const std::string marker_namespace =
      "active_trajectory/" +
      trajectory.trajectory_id;

    // LINE_LIST is deliberate. A LINE_STRIP would reconnect separate mission
    // components and visually invent geometry across cropped gaps.
    Marker line;
    line.header.frame_id = frame_id;
    line.header.stamp = stamp;
    line.ns = marker_namespace;
    line.id = 0;
    line.type = Marker::LINE_LIST;
    line.action = Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = marker_line_width_;
    line.color = marker_color;

    for (const auto & segment : geometry) {
      line.points.push_back(
        to_point(segment.start));
      line.points.push_back(
        to_point(segment.end));
    }

    marker_array.markers.push_back(
      std::move(line));

    Marker text;
    text.header.frame_id = frame_id;
    text.header.stamp = stamp;
    text.ns = marker_namespace;
    text.id = 1;
    text.type = Marker::TEXT_VIEW_FACING;
    text.action = Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.pose.position =
      to_point(
      geometry.front().start);
    text.pose.position.z +=
      marker_text_height_ * 1.8;
    text.scale.z =
      marker_text_height_;
    text.color =
      marker_color;

    std::ostringstream label;
    label
      << "ACTIVE "
      << trajectory.trajectory_id
      << " | P="
      << trajectory.priority
      << " | M="
      << trajectory.mission.size()
      << "\n"
      << "start: "
      << format_utc_iso8601(
      trajectory.operation_start_utc)
      << "\n"
      << "end:   "
      << format_utc_iso8601(
      trajectory.operation_end_utc);

    text.text = label.str();

    marker_array.markers.push_back(
      std::move(text));
  }

  void publish_active_snapshot_locked(
    StaticTrajectoryArray & output,
    MarkerArray & markers)
  {
    output.header = latest_header_;
    output.header.stamp = now();
    if (output.header.frame_id.empty()) {
      output.header.frame_id = "map";
    }

    Marker delete_all;
    delete_all.header.frame_id = output.header.frame_id;
    delete_all.header.stamp = output.header.stamp;
    delete_all.action = Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    std::vector<const ActiveTrajectory *> ordered;
    ordered.reserve(active_.size());
    for (const auto & [_, active] : active_) {
      ordered.push_back(&active);
    }
    std::stable_sort(
      ordered.begin(), ordered.end(),
      [](const ActiveTrajectory * first, const ActiveTrajectory * second) {
        if (first->trajectory.priority != second->trajectory.priority) {
          return first->trajectory.priority < second->trajectory.priority;
        }
        if (first->trajectory.trajectory_id != second->trajectory.trajectory_id) {
          return first->trajectory.trajectory_id < second->trajectory.trajectory_id;
        }
        return time_to_ns(first->trajectory.operation_start_utc) <
          time_to_ns(second->trajectory.operation_start_utc);
      });

    output.trajectories.reserve(ordered.size());
    for (const auto * active : ordered) {
      output.trajectories.push_back(active->trajectory);
      add_active_markers(active->trajectory, output.header.stamp, markers);
    }
  }

  void supervision_tick()
  {
    const int64_t current_ns = system_now_ns();
    StaticTrajectoryArray active_output;
    MarkerArray active_markers;
    std::vector<CollisionStaticTrajectory> adjustment_events;

    std::size_t stored_count = 0U;
    std::size_t waiting_count = 0U;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      mark_active_completions_locked(current_ns);
      store_due_candidates_locked(current_ns);
      process_pending_locked(current_ns, adjustment_events);
      prune_vehicle_watchers_locked();
      publish_active_snapshot_locked(active_output, active_markers);

      stored_count = pending_.size();
      waiting_count = static_cast<std::size_t>(std::count_if(
        pending_.begin(), pending_.end(),
        [](const auto & item) {
          return item.second.mode == PendingMode::WAITING_FOR_ACTIVE_CONFLICTS;
        }));
    }

    active_publisher_->publish(active_output);
    active_markers_publisher_->publish(active_markers);

    if (!adjustment_events.empty()) {
      CollisionStaticTrajectoryArray adjustment_output;
      adjustment_output.header = active_output.header;
      adjustment_output.header.stamp = now();
      adjustment_output.trajectories = std::move(adjustment_events);
      non_priority_adjustment_publisher_->publish(adjustment_output);
    }

    if (last_active_count_ != active_output.trajectories.size()) {
      RCLCPP_INFO(
        get_logger(),
        "Active snapshot: stored=%zu waiting=%zu active=%zu",
        stored_count,
        waiting_count,
        active_output.trajectories.size());
      last_active_count_ = active_output.trajectories.size();
    }
  }

  std::string available_topic_;
  std::string supervised_topic_;
  std::string active_topic_;
  std::string non_priority_adjustment_topic_;
  std::string active_markers_topic_;
  std::string original_trajectory_registration_service_;

  double storage_lead_time_s_{60.0};
  double publication_lead_time_s_{30.0};
  double reschedule_margin_s_{5.0};
  bool preserve_duration_on_reschedule_{true};

  std::string vehicle_status_suffix_{"fmu/out/vehicle_status"};
  double vehicle_status_timeout_s_{2.0};
  double minimum_armed_duration_for_completion_s_{20.0};

  double spatial_conflict_distance_m_{1.0};
  bool use_3d_{true};
  int max_segment_collisions_per_pair_{64};

  int supervision_period_ms_{200};
  double marker_line_width_{0.14};
  double marker_text_height_{0.34};

  mutable std::mutex mutex_;
  std_msgs::msg::Header latest_header_;
  StaticTrajectoryArray raw_available_;
  bool available_received_{false};
  std::map<std::string, DetectedCollisionTrajectory> latest_supervised_;
  std::map<std::string, StaticTrajectory> latest_available_;
  std::map<std::string, PendingTrajectory> pending_;
  std::map<std::string, ActiveTrajectory> active_;
  std::map<std::string, VehicleStatusWatcher> vehicle_watchers_;
  std::set<std::string> completed_source_keys_;
  std::set<std::string> rejected_source_keys_;
  std::size_t last_active_count_{std::numeric_limits<std::size_t>::max()};

  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr available_subscription_;
  rclcpp::Subscription<DetectedCollisionTrajectoryArray>::SharedPtr supervised_subscription_;
  rclcpp::Publisher<StaticTrajectoryArray>::SharedPtr active_publisher_;
  rclcpp::Publisher<CollisionStaticTrajectoryArray>::SharedPtr
    non_priority_adjustment_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr active_markers_publisher_;
  rclcpp::Client<RegisterOriginalTrajectory>::SharedPtr original_registration_client_;
  rclcpp::TimerBase::SharedPtr supervision_timer_;
};

}  // namespace trajectory_server_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<trajectory_server_pkg::TrajectoryServerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("trajectory_server_node"),
      "Fatal error: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
