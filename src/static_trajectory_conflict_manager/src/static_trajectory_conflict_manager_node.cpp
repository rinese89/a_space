#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <static_trajectory_conflict_manager/msg/collision_static_trajectory.hpp>
#include <static_trajectory_conflict_manager/msg/collision_static_trajectory_array.hpp>
#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace static_trajectory_conflict_manager
{

using StaticTrajectory = static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using TrajectorySegment = static_trajectory_manager::msg::TrajectorySegment;

using CollisionStaticTrajectory =
  static_trajectory_conflict_manager::msg::CollisionStaticTrajectory;
using CollisionStaticTrajectoryArray =
  static_trajectory_conflict_manager::msg::CollisionStaticTrajectoryArray;

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr double kGeometryEpsilon = 1.0e-12;

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
};

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

static double squared_norm(const Vec3 & value)
{
  return dot(value, value);
}

static double norm(const Vec3 & value)
{
  return std::sqrt(squared_norm(value));
}

static double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

static Vec3 project_for_distance(const Vec3 & value, bool use_3d)
{
  if (use_3d) {
    return value;
  }

  return Vec3{value.x, value.y, 0.0};
}

static geometry_msgs::msg::Point to_point(const Vec3 & value)
{
  geometry_msgs::msg::Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

static int64_t time_to_ns(const builtin_interfaces::msg::Time & value)
{
  return
    static_cast<int64_t>(value.sec) * kNanosecondsPerSecond +
    static_cast<int64_t>(value.nanosec);
}

static int64_t seconds_to_ns(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0) {
    throw std::runtime_error(
            "period seconds must be finite and non-negative");
  }

  const long double value =
    static_cast<long double>(seconds) *
    static_cast<long double>(kNanosecondsPerSecond);

  if (
    value >
    static_cast<long double>(
      std::numeric_limits<int64_t>::max()))
  {
    throw std::runtime_error(
            "period exceeds int64 nanosecond range");
  }

  return static_cast<int64_t>(
    std::llround(value));
}

static int64_t floor_div(
  int64_t numerator,
  int64_t denominator)
{
  if (denominator <= 0) {
    throw std::runtime_error(
            "floor_div denominator must be positive");
  }

  int64_t quotient = numerator / denominator;
  const int64_t remainder = numerator % denominator;

  if (remainder < 0) {
    --quotient;
  }

  return quotient;
}

static int64_t ceil_div(
  int64_t numerator,
  int64_t denominator)
{
  if (denominator <= 0) {
    throw std::runtime_error(
            "ceil_div denominator must be positive");
  }

  int64_t quotient = numerator / denominator;
  const int64_t remainder = numerator % denominator;

  if (remainder > 0) {
    ++quotient;
  }

  return quotient;
}

static int64_t gcd_positive(
  int64_t first,
  int64_t second)
{
  first = std::llabs(first);
  second = std::llabs(second);

  while (second != 0) {
    const int64_t remainder = first % second;
    first = second;
    second = remainder;
  }

  return first;
}

static bool half_open_intervals_overlap(
  int64_t first_start,
  int64_t first_end,
  int64_t second_start,
  int64_t second_end)
{
  return
    first_start < second_end &&
    second_start < first_end;
}

static bool temporal_overlap_exists(
  const StaticTrajectory & first,
  const StaticTrajectory & second)
{
  const int64_t first_start =
    time_to_ns(first.operation_start_utc);

  const int64_t first_end =
    time_to_ns(first.operation_end_utc);

  const int64_t second_start =
    time_to_ns(second.operation_start_utc);

  const int64_t second_end =
    time_to_ns(second.operation_end_utc);

  if (
    first_end <= first_start ||
    second_end <= second_start)
  {
    return false;
  }

  const int64_t first_duration =
    first_end - first_start;

  const int64_t second_duration =
    second_end - second_start;

  const bool first_periodic =
    first.operation_frequency > 0.0;

  const bool second_periodic =
    second.operation_frequency > 0.0;

  if (!first_periodic && !second_periodic) {
    return half_open_intervals_overlap(
      first_start,
      first_end,
      second_start,
      second_end);
  }

  const int64_t first_period =
    first_periodic ?
    seconds_to_ns(first.operation_frequency) :
    0LL;

  const int64_t second_period =
    second_periodic ?
    seconds_to_ns(second.operation_frequency) :
    0LL;

  if (
    (first_periodic && first_period <= 0) ||
    (second_periodic && second_period <= 0))
  {
    return false;
  }

  if (first_periodic && !second_periodic) {
    const int64_t lower =
      second_start - first_duration + 1LL;

    const int64_t upper =
      second_end - 1LL;

    const int64_t first_sequence =
      std::max<int64_t>(
        0LL,
        ceil_div(
          lower - first_start,
          first_period));

    const int64_t occurrence_start =
      first_start +
      first_sequence * first_period;

    return occurrence_start <= upper;
  }

  if (!first_periodic && second_periodic) {
    const int64_t lower =
      first_start - second_duration + 1LL;

    const int64_t upper =
      first_end - 1LL;

    const int64_t second_sequence =
      std::max<int64_t>(
        0LL,
        ceil_div(
          lower - second_start,
          second_period));

    const int64_t occurrence_start =
      second_start +
      second_sequence * second_period;

    return occurrence_start <= upper;
  }

  // Infinite periodic definitions can be checked exactly from the lattice
  // generated by the two periods.
  const int64_t lattice_step =
    gcd_positive(
      first_period,
      second_period);

  if (lattice_step <= 0) {
    return false;
  }

  const int64_t delta =
    first_start - second_start;

  const int64_t z_min =
    ceil_div(
      -first_duration + 1LL - delta,
      lattice_step);

  const int64_t z_max =
    floor_div(
      second_duration - 1LL - delta,
      lattice_step);

  return z_min <= z_max;
}

static std::vector<Vec3> segment_points(
  const TrajectorySegment & segment,
  const std::string & trajectory_id,
  const std::string & segment_name)
{
  if (
    segment.x.size() != segment.y.size() ||
    segment.x.size() != segment.z.size())
  {
    throw std::runtime_error(
            "Trajectory '" + trajectory_id +
            "': segment '" + segment_name +
            "' has different x/y/z lengths");
  }

  std::vector<Vec3> points;
  points.reserve(segment.x.size());

  for (
    std::size_t index = 0U;
    index < segment.x.size();
    ++index)
  {
    const Vec3 point{
      segment.x[index],
      segment.y[index],
      segment.z[index]};

    if (
      !std::isfinite(point.x) ||
      !std::isfinite(point.y) ||
      !std::isfinite(point.z))
    {
      throw std::runtime_error(
              "Trajectory '" + trajectory_id +
              "': segment '" + segment_name +
              "' contains a non-finite coordinate");
    }

    points.push_back(point);
  }

  return points;
}

static void append_polyline_segments(
  std::vector<GeometrySegment> & target,
  const std::vector<Vec3> & points)
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
        points[index]});
  }
}

static std::vector<GeometrySegment> build_geometry_segments(
  const StaticTrajectory & trajectory)
{
  if (trajectory.repetitions == 0U) {
    throw std::runtime_error(
            "Trajectory '" +
            trajectory.trajectory_id +
            "': repetitions must be >= 1");
  }

  const auto takeoff =
    segment_points(
      trajectory.takeoff,
      trajectory.trajectory_id,
      "takeoff");

  const auto landing =
    segment_points(
      trajectory.landing,
      trajectory.trajectory_id,
      "landing");

  std::vector<
    std::vector<Vec3>> missions;

  missions.reserve(
    trajectory.mission.size());

  std::size_t geometry_segment_count = 0U;

  if (takeoff.size() >= 2U) {
    geometry_segment_count +=
      takeoff.size() - 1U;
  }

  for (
    std::size_t mission_index = 0U;
    mission_index < trajectory.mission.size();
    ++mission_index)
  {
    auto points =
      segment_points(
      trajectory.mission[mission_index],
      trajectory.trajectory_id,
      "mission[" +
      std::to_string(mission_index) +
      "]");

    if (points.size() >= 2U) {
      geometry_segment_count +=
        points.size() - 1U;
    }

    missions.push_back(
      std::move(points));
  }

  if (landing.size() >= 2U) {
    geometry_segment_count +=
      landing.size() - 1U;
  }

  std::vector<GeometrySegment> segments;
  segments.reserve(
    geometry_segment_count);

  // Every TrajectorySegment is an independent continuous polyline.
  //
  // There is NO implicit geometry between:
  //   takeoff.back()        -> mission[0].front()
  //   mission[i].back()     -> mission[i + 1].front()
  //   mission.back().back() -> landing.front()
  //
  // This preserves gaps introduced by supervised trajectory cropping.
  append_polyline_segments(
    segments,
    takeoff);

  for (const auto & mission : missions) {
    append_polyline_segments(
      segments,
      mission);
  }

  append_polyline_segments(
    segments,
    landing);

  // Partial repetitions repeat exactly the same mission collection. This node
  // performs a Boolean spatial-overlap test, so duplicating those same edges
  // repetitions times cannot change the collision result.
  return segments;
}

static double segment_distance(
  const GeometrySegment & first,
  const GeometrySegment & second,
  bool use_3d)
{
  const Vec3 first_start =
    project_for_distance(
      first.start,
      use_3d);

  const Vec3 first_end =
    project_for_distance(
      first.end,
      use_3d);

  const Vec3 second_start =
    project_for_distance(
      second.start,
      use_3d);

  const Vec3 second_end =
    project_for_distance(
      second.end,
      use_3d);

  const Vec3 d1 =
    subtract(first_end, first_start);

  const Vec3 d2 =
    subtract(second_end, second_start);

  const Vec3 r =
    subtract(first_start, second_start);

  const double a = dot(d1, d1);
  const double e = dot(d2, d2);
  const double f = dot(d2, r);

  double s = 0.0;
  double t = 0.0;

  if (
    a <= kGeometryEpsilon &&
    e <= kGeometryEpsilon)
  {
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
      const double denominator =
        a * e - b * b;

      if (
        std::abs(denominator) >
        kGeometryEpsilon)
      {
        s = clamp(
          (b * f - c * e) /
          denominator,
          0.0,
          1.0);
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

  const Vec3 first_closest =
    add(
      first_start,
      multiply(d1, s));

  const Vec3 second_closest =
    add(
      second_start,
      multiply(d2, t));

  return norm(
    subtract(
      first_closest,
      second_closest));
}

class Fnv1a64
{
public:
  void add_bytes(
    const void * data,
    std::size_t size)
  {
    const auto * bytes =
      static_cast<const unsigned char *>(data);

    for (
      std::size_t index = 0U;
      index < size;
      ++index)
    {
      value_ ^= static_cast<uint64_t>(
        bytes[index]);

      value_ *= 1099511628211ULL;
    }
  }

  template<typename T>
  void add_scalar(const T & value)
  {
    add_bytes(
      &value,
      sizeof(T));
  }

  void add_string(const std::string & value)
  {
    const uint64_t size =
      static_cast<uint64_t>(
        value.size());

    add_scalar(size);

    if (!value.empty()) {
      add_bytes(
        value.data(),
        value.size());
    }
  }

  void add_double(double value)
  {
    uint64_t bits = 0U;

    static_assert(
      sizeof(bits) == sizeof(value),
      "double must be 64 bit");

    std::memcpy(
      &bits,
      &value,
      sizeof(value));

    add_scalar(bits);
  }

  uint64_t value() const
  {
    return value_;
  }

private:
  uint64_t value_{1469598103934665603ULL};
};

static void hash_time(
  Fnv1a64 & hash,
  const builtin_interfaces::msg::Time & value)
{
  hash.add_scalar(value.sec);
  hash.add_scalar(value.nanosec);
}

static void hash_segment(
  Fnv1a64 & hash,
  const TrajectorySegment & segment)
{
  const uint64_t count =
    static_cast<uint64_t>(
      segment.x.size());

  hash.add_scalar(count);

  for (const double value : segment.x) {
    hash.add_double(value);
  }

  for (const double value : segment.y) {
    hash.add_double(value);
  }

  for (const double value : segment.z) {
    hash.add_double(value);
  }
}

static uint64_t trajectory_signature(
  const StaticTrajectory & trajectory)
{
  Fnv1a64 hash;

  hash.add_scalar(trajectory.priority);
  hash.add_string(trajectory.trajectory_id);
  hash.add_scalar(trajectory.ua_id);
  hash.add_string(trajectory.uas_namespace);
  hash.add_string(trajectory.flight_zone_id);
  hash.add_string(trajectory.frame_id);
  hash.add_string(trajectory.action_name);

  hash_segment(hash, trajectory.takeoff);

  const uint64_t mission_count =
    static_cast<uint64_t>(
    trajectory.mission.size());

  hash.add_scalar(
    mission_count);

  for (const auto & mission :
    trajectory.mission)
  {
    hash_segment(
      hash,
      mission);
  }

  hash_segment(hash, trajectory.landing);

  hash.add_double(trajectory.goal_tolerance);
  hash.add_double(trajectory.slowdown_radius);
  hash.add_scalar(trajectory.repetitions);
  hash.add_double(trajectory.operation_frequency);
  hash.add_double(trajectory.average_speed_mps);
  hash.add_double(trajectory.estimated_distance_m);
  hash.add_double(trajectory.estimated_duration_s);

  hash_time(
    hash,
    trajectory.operation_start_utc);

  hash_time(
    hash,
    trajectory.operation_end_utc);

  return hash.value();
}

static std_msgs::msg::ColorRGBA make_color(
  float r,
  float g,
  float b,
  float a = 1.0F)
{
  std_msgs::msg::ColorRGBA output;
  output.r = r;
  output.g = g;
  output.b = b;
  output.a = a;
  return output;
}

class StaticTrajectoryConflictManagerNode
  : public rclcpp::Node
{
public:
  StaticTrajectoryConflictManagerNode()
  : Node("static_trajectory_conflict_manager_node")
  {
    requested_topic_ =
      declare_parameter<std::string>(
        "requested_static_trajectories_topic",
        "/requested_static_trajectories");

    available_topic_ =
      declare_parameter<std::string>(
        "available_static_trajectories_topic",
        "/available_static_trajectories");

    collision_topic_ =
      declare_parameter<std::string>(
        "collision_static_trajectories_topic",
        "/collision_static_trajectories");

    available_markers_topic_ =
      declare_parameter<std::string>(
        "available_static_trajectories_markers_topic",
        "/available_static_trajectories_markers");

    collision_markers_topic_ =
      declare_parameter<std::string>(
        "collision_static_trajectories_markers_topic",
        "/collision_static_trajectories_markers");

    minimum_separation_m_ =
      declare_parameter<double>(
        "minimum_separation_m",
        1.0);

    use_3d_ =
      declare_parameter<bool>(
        "use_3d",
        true);

    publish_period_ms_ =
      declare_parameter<int>(
        "publish_period_ms",
        1000);

    marker_line_width_ =
      declare_parameter<double>(
        "marker_line_width",
        0.12);

    marker_text_height_ =
      declare_parameter<double>(
        "marker_text_height",
        0.32);

    validate_parameters();

    auto state_qos =
      rclcpp::QoS(
        rclcpp::KeepLast(1));

    state_qos.reliable();
    state_qos.transient_local();

    requested_subscription_ =
      create_subscription<StaticTrajectoryArray>(
        requested_topic_,
        state_qos,
        std::bind(
          &StaticTrajectoryConflictManagerNode::
          requested_trajectories_callback,
          this,
          std::placeholders::_1));

    available_publisher_ =
      create_publisher<StaticTrajectoryArray>(
        available_topic_,
        state_qos);

    collision_publisher_ =
      create_publisher<CollisionStaticTrajectoryArray>(
        collision_topic_,
        state_qos);

    available_marker_publisher_ =
      create_publisher<MarkerArray>(
        available_markers_topic_,
        state_qos);

    collision_marker_publisher_ =
      create_publisher<MarkerArray>(
        collision_markers_topic_,
        state_qos);

    publish_timer_ =
      create_wall_timer(
        std::chrono::milliseconds(
          publish_period_ms_),
        std::bind(
          &StaticTrajectoryConflictManagerNode::
          publish_current_state,
          this));

    RCLCPP_INFO(
      get_logger(),
      "Simplified static trajectory conflict manager ready | "
      "requested='%s' | available='%s' | collision='%s' | "
      "minimum_separation=%.3f m | geometry=%s | heartbeat=%d ms",
      requested_topic_.c_str(),
      available_topic_.c_str(),
      collision_topic_.c_str(),
      minimum_separation_m_,
      use_3d_ ? "3D" : "2D",
      publish_period_ms_);
  }

private:
  void validate_parameters() const
  {
    const auto absolute =
      [](const std::string & value)
      {
        return
          !value.empty() &&
          value.front() == '/';
      };

    if (
      !absolute(requested_topic_) ||
      !absolute(available_topic_) ||
      !absolute(collision_topic_) ||
      !absolute(available_markers_topic_) ||
      !absolute(collision_markers_topic_))
    {
      throw std::runtime_error(
              "All topic names must be absolute");
    }

    if (
      !std::isfinite(minimum_separation_m_) ||
      minimum_separation_m_ < 0.0)
    {
      throw std::runtime_error(
              "minimum_separation_m must be finite and >= 0");
    }

    if (publish_period_ms_ <= 0) {
      throw std::runtime_error(
              "publish_period_ms must be > 0");
    }

    if (
      !std::isfinite(marker_line_width_) ||
      marker_line_width_ <= 0.0 ||
      !std::isfinite(marker_text_height_) ||
      marker_text_height_ <= 0.0)
    {
      throw std::runtime_error(
              "marker sizes must be finite and > 0");
    }
  }

  bool pair_collides(
    const StaticTrajectory & candidate,
    const StaticTrajectory & accepted) const
  {
    if (
      !candidate.frame_id.empty() &&
      !accepted.frame_id.empty() &&
      candidate.frame_id != accepted.frame_id)
    {
      return false;
    }

    if (!temporal_overlap_exists(candidate, accepted)) {
      return false;
    }

    const auto candidate_segments =
      build_geometry_segments(candidate);

    const auto accepted_segments =
      build_geometry_segments(accepted);

    // Deliberately short-circuited. No collision evidence, closest points,
    // phases or segment indices are generated in this node anymore.
    for (const auto & candidate_segment : candidate_segments) {
      for (const auto & accepted_segment : accepted_segments) {
        const double distance =
          segment_distance(
            candidate_segment,
            accepted_segment,
            use_3d_);

        if (
          distance <=
          minimum_separation_m_ +
          kGeometryEpsilon)
        {
          return true;
        }
      }
    }

    return false;
  }

  bool requested_state_changed(
    const std::map<std::string, uint64_t> & signatures) const
  {
    if (
      signatures.size() !=
      requested_signatures_.size())
    {
      return true;
    }

    for (const auto & [trajectory_id, signature] :
      signatures)
    {
      const auto iterator =
        requested_signatures_.find(
          trajectory_id);

      if (
        iterator ==
        requested_signatures_.end() ||
        iterator->second != signature)
      {
        return true;
      }
    }

    return false;
  }

  void classify_snapshot(
    const std::map<std::string, StaticTrajectory> & current,
    const std::map<std::string, uint64_t> & signatures)
  {
    std::vector<const StaticTrajectory *> ordered;
    ordered.reserve(current.size());

    for (const auto & [_, trajectory] : current) {
      ordered.push_back(&trajectory);
    }

    // Priority only defines deterministic evaluation order.
    // Lower numerical value means earlier evaluation.
    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](const StaticTrajectory * first,
        const StaticTrajectory * second)
      {
        if (
          first->priority !=
          second->priority)
        {
          return
            first->priority <
            second->priority;
        }

        return
          first->trajectory_id <
          second->trajectory_id;
      });

    std::map<std::string, StaticTrajectory>
      next_available;

    std::map<std::string, StaticTrajectory>
      next_collision;

    for (const auto * candidate : ordered) {
      bool collision_found = false;

      for (const auto & [_, accepted] :
        next_available)
      {
        if (pair_collides(*candidate, accepted)) {
          collision_found = true;

          RCLCPP_INFO(
            get_logger(),
            "Trajectory '%s' classified COLLISION; "
            "analysis stopped at first collision",
            candidate->trajectory_id.c_str());

          break;
        }
      }

      if (collision_found) {
        next_collision.emplace(
          candidate->trajectory_id,
          *candidate);
      } else {
        next_available.emplace(
          candidate->trajectory_id,
          *candidate);
      }
    }

    available_state_ =
      std::move(next_available);

    collision_state_ =
      std::move(next_collision);

    requested_signatures_ =
      signatures;

    RCLCPP_INFO(
      get_logger(),
      "Classification updated | requested=%zu | available=%zu | collision=%zu",
      current.size(),
      available_state_.size(),
      collision_state_.size());

    publish_current_state_locked();
  }

  void requested_trajectories_callback(
    const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    input_received_ = true;
    latest_header_ = message->header;

    std::map<std::string, StaticTrajectory>
      current;

    std::map<std::string, uint64_t>
      signatures;

    for (const auto & trajectory :
      message->trajectories)
    {
      if (trajectory.trajectory_id.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring requested trajectory with empty trajectory_id");
        continue;
      }

      if (
        current.find(trajectory.trajectory_id) !=
        current.end())
      {
        RCLCPP_WARN(
          get_logger(),
          "Duplicate trajectory_id '%s' in requested snapshot; "
          "last occurrence wins",
          trajectory.trajectory_id.c_str());
      }

      current[trajectory.trajectory_id] =
        trajectory;

      signatures[trajectory.trajectory_id] =
        trajectory_signature(trajectory);
    }

    if (!requested_state_changed(signatures)) {
      // Nothing changed. The 1 Hz timer keeps both retained snapshots alive.
      return;
    }

    try {
      classify_snapshot(
        current,
        signatures);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot classify requested static trajectories: %s",
        error.what());
    }
  }

  MarkerArray make_markers(
    const std::map<std::string, StaticTrajectory> & trajectories,
    const std::string & marker_namespace,
    const std_msgs::msg::ColorRGBA & marker_color) const
  {
    MarkerArray output;

    Marker delete_all;
    delete_all.header = latest_header_;
    delete_all.header.stamp = now();

    if (delete_all.header.frame_id.empty()) {
      delete_all.header.frame_id = "map";
    }

    delete_all.action =
      Marker::DELETEALL;

    output.markers.push_back(
      delete_all);

    int marker_id = 0;

    for (const auto & [trajectory_id, trajectory] :
      trajectories)
    {
      std::vector<GeometrySegment> geometry;

      try {
        geometry =
          build_geometry_segments(
            trajectory);
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(),
          "Could not create marker for '%s': %s",
          trajectory_id.c_str(),
          error.what());

        continue;
      }

      const std::string frame_id =
        trajectory.frame_id.empty() ?
        delete_all.header.frame_id :
        trajectory.frame_id;

      Marker line;
      line.header.stamp = now();
      line.header.frame_id = frame_id;
      line.ns =
        marker_namespace + "/" +
        trajectory_id;
      line.id = marker_id++;
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

      output.markers.push_back(
        std::move(line));

      if (!geometry.empty()) {
        Marker text;
        text.header.stamp = now();
        text.header.frame_id = frame_id;
        text.ns =
          marker_namespace + "/" +
          trajectory_id;
        text.id = marker_id++;
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
        text.text =
          marker_namespace +
          " | " +
          trajectory_id +
          " | P=" +
          std::to_string(
            trajectory.priority) +
          " | M=" +
          std::to_string(
            trajectory.mission.size());

        output.markers.push_back(
          std::move(text));
      }
    }

    return output;
  }

  void publish_available_snapshot_locked()
  {
    StaticTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    output.trajectories.reserve(
      available_state_.size());

    for (const auto & [_, trajectory] :
      available_state_)
    {
      output.trajectories.push_back(
        trajectory);
    }

    available_publisher_->publish(
      output);

    available_marker_publisher_->publish(
      make_markers(
        available_state_,
        "AVAILABLE",
        make_color(
          0.10F,
          0.45F,
          1.00F,
          1.00F)));
  }

  void publish_collision_snapshot_locked()
  {
    CollisionStaticTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    output.trajectories.reserve(
      collision_state_.size());

    for (const auto & [_, trajectory] :
      collision_state_)
    {
      CollisionStaticTrajectory collision;
      collision.trajectory = trajectory;

      // Intentionally empty:
      // collision.collisions
      //
      // Presence in this array means COLLISION. Detailed collision evidence is
      // now the responsibility of the downstream classifier.
      output.trajectories.push_back(
        std::move(collision));
    }

    collision_publisher_->publish(
      output);

    collision_marker_publisher_->publish(
      make_markers(
        collision_state_,
        "COLLISION",
        make_color(
          1.00F,
          0.10F,
          0.10F,
          1.00F)));
  }

  void publish_current_state_locked()
  {
    publish_available_snapshot_locked();
    publish_collision_snapshot_locked();
  }

  void publish_current_state()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!input_received_) {
      return;
    }

    publish_current_state_locked();
  }

  std::string requested_topic_;
  std::string available_topic_;
  std::string collision_topic_;
  std::string available_markers_topic_;
  std::string collision_markers_topic_;

  double minimum_separation_m_{1.0};
  bool use_3d_{true};
  int publish_period_ms_{1000};

  double marker_line_width_{0.12};
  double marker_text_height_{0.32};

  mutable std::mutex mutex_;
  bool input_received_{false};

  std_msgs::msg::Header latest_header_;

  // Only collision-free trajectories form the comparison/reference set.
  std::map<std::string, StaticTrajectory>
    available_state_;

  // Output snapshot cache only. These trajectories are NEVER used as blockers
  // when classifying another trajectory.
  std::map<std::string, StaticTrajectory>
    collision_state_;

  // Lightweight input-state signatures only; no rejected geometry/evidence
  // cache is used for classification.
  std::map<std::string, uint64_t>
    requested_signatures_;

  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr
    requested_subscription_;

  rclcpp::Publisher<StaticTrajectoryArray>::SharedPtr
    available_publisher_;

  rclcpp::Publisher<CollisionStaticTrajectoryArray>::SharedPtr
    collision_publisher_;

  rclcpp::Publisher<MarkerArray>::SharedPtr
    available_marker_publisher_;

  rclcpp::Publisher<MarkerArray>::SharedPtr
    collision_marker_publisher_;

  rclcpp::TimerBase::SharedPtr
    publish_timer_;
};

}  // namespace static_trajectory_conflict_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        static_trajectory_conflict_manager::
        StaticTrajectoryConflictManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "static_trajectory_conflict_manager_node"),
      "Fatal error: %s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
