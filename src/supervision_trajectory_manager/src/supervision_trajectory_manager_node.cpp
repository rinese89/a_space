#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <a_space_virtual_net/msg/net_trajectory_segment.hpp>

#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>

#include <deconfliction_manager/msg/cropped_net_trajectory.hpp>
#include <deconfliction_manager/msg/requested_supervision_trajectory.hpp>
#include <deconfliction_manager/msg/requested_supervision_trajectory_array.hpp>

#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
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

namespace supervision_trajectory_manager
{

using NetTrajectorySegment =
  a_space_virtual_net::msg::NetTrajectorySegment;

using DetectedCollisionTrajectory =
  collision_detection::msg::DetectedCollisionTrajectory;
using DetectedCollisionTrajectoryArray =
  collision_detection::msg::DetectedCollisionTrajectoryArray;

using CroppedNetTrajectory =
  deconfliction_manager::msg::CroppedNetTrajectory;
using RequestedSupervisionTrajectory =
  deconfliction_manager::msg::RequestedSupervisionTrajectory;
using RequestedSupervisionTrajectoryArray =
  deconfliction_manager::msg::RequestedSupervisionTrajectoryArray;

using StaticTrajectory =
  static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray =
  static_trajectory_manager::msg::StaticTrajectoryArray;
using TrajectorySegment =
  static_trajectory_manager::msg::TrajectorySegment;

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

constexpr uint8_t kTakeoff = 0U;
constexpr uint8_t kMission = 1U;
constexpr uint8_t kLanding = 2U;

enum class StoredState : uint8_t
{
  PENDING = 0U,
  SUPERVISED = 1U
};

struct StoredEntry
{
  // Full ORIGINAL collision-detection payload. This is what is later
  // republished on /supervised_trajectories.
  DetectedCollisionTrajectory request;

  // Exact geometry-only crop calculated upstream by deconfliction_manager.
  CroppedNetTrajectory crop;
  bool crop_complete{false};

  // True only when the complete upstream net crop can be represented EXACTLY
  // by the legacy StaticTrajectory schema (one continuous polyline per phase).
  // No fragment is discarded and no gap is reconnected.
  bool adjusted_ready{false};

  // Exact legacy representation used by /adjusted_trajectories when
  // adjusted_ready=true.
  StaticTrajectory adjusted;

  StoredState state{StoredState::PENDING};

  // Periodic trajectories keep their ORIGINAL full geometry in request, but
  // their current total-operation window is synchronized from the cropped
  // occurrence seen in /available_static_trajectories.
  builtin_interfaces::msg::Time synchronized_start_utc;
  builtin_interfaces::msg::Time synchronized_end_utc;
  bool synchronized_time_received{false};
};

struct NetRun
{
  std::vector<geometry_msgs::msg::Point> points;
  double length_m{0.0};
};

static std_msgs::msg::ColorRGBA color(
  float r,
  float g,
  float b,
  float a = 1.0F)
{
  std_msgs::msg::ColorRGBA out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.a = a;
  return out;
}

static std::string join_ids(
  const std::vector<std::string> & ids)
{
  std::ostringstream stream;

  for (std::size_t i = 0U; i < ids.size(); ++i) {
    if (i != 0U) {
      stream << ",";
    }
    stream << ids[i];
  }

  return stream.str();
}

static double point_distance(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static bool same_node(
  uint64_t first,
  uint64_t second)
{
  return first == second;
}

static bool share_node(
  const NetTrajectorySegment & first,
  const NetTrajectorySegment & second)
{
  return
    same_node(first.start_node_id, second.start_node_id) ||
    same_node(first.start_node_id, second.end_node_id) ||
    same_node(first.end_node_id, second.start_node_id) ||
    same_node(first.end_node_id, second.end_node_id);
}

static const TrajectorySegment & original_phase(
  const StaticTrajectory & trajectory,
  uint8_t phase)
{
  if (phase == kTakeoff) {
    return trajectory.takeoff;
  }

  if (phase == kMission) {
    return trajectory.mission;
  }

  return trajectory.landing;
}

static std::size_t segment_count(
  const TrajectorySegment & segment)
{
  return
    std::min(
    segment.x.size(),
    std::min(
      segment.y.size(),
      segment.z.size()));
}

static geometry_msgs::msg::Point segment_point(
  const TrajectorySegment & segment,
  std::size_t index)
{
  geometry_msgs::msg::Point output;
  output.x = segment.x[index];
  output.y = segment.y[index];
  output.z = segment.z[index];
  return output;
}

static double point_to_polyline_parameter(
  const geometry_msgs::msg::Point & query,
  const TrajectorySegment & original)
{
  const std::size_t count =
    segment_count(original);

  if (count == 0U) {
    return 0.0;
  }

  if (count == 1U) {
    return point_distance(
      query,
      segment_point(original, 0U));
  }

  double best_distance_sq =
    std::numeric_limits<double>::infinity();
  double best_parameter = 0.0;
  double cumulative = 0.0;

  for (std::size_t i = 1U; i < count; ++i) {
    const auto start =
      segment_point(original, i - 1U);
    const auto end =
      segment_point(original, i);

    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double vz = end.z - start.z;

    const double wx = query.x - start.x;
    const double wy = query.y - start.y;
    const double wz = query.z - start.z;

    const double length_sq =
      vx * vx + vy * vy + vz * vz;
    const double length =
      std::sqrt(length_sq);

    double ratio = 0.0;

    if (length_sq > 1.0e-12) {
      ratio =
        std::clamp(
        (wx * vx + wy * vy + wz * vz) /
        length_sq,
        0.0,
        1.0);
    }

    const double cx =
      start.x + ratio * vx;
    const double cy =
      start.y + ratio * vy;
    const double cz =
      start.z + ratio * vz;

    const double dx =
      query.x - cx;
    const double dy =
      query.y - cy;
    const double dz =
      query.z - cz;

    const double distance_sq =
      dx * dx + dy * dy + dz * dz;

    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_parameter =
        cumulative + ratio * length;
    }

    cumulative += length;
  }

  return best_parameter;
}

static std::vector<
  const NetTrajectorySegment *>
phase_segments(
  const CroppedNetTrajectory & crop,
  uint8_t phase)
{
  std::vector<
    const NetTrajectorySegment *> output;

  for (const auto & segment :
    crop.retained_segments)
  {
    if (segment.phase == phase) {
      output.push_back(&segment);
    }
  }

  std::stable_sort(
    output.begin(),
    output.end(),
    [](
      const NetTrajectorySegment * first,
      const NetTrajectorySegment * second)
    {
      return
        first->net_segment_index <
        second->net_segment_index;
    });

  return output;
}

static std::vector<
  std::vector<const NetTrajectorySegment *>>
split_runs(
  const std::vector<
    const NetTrajectorySegment *> & ordered)
{
  std::vector<
    std::vector<const NetTrajectorySegment *>>
    runs;

  std::vector<
    const NetTrajectorySegment *> current;

  for (const auto * segment : ordered) {
    if (segment == nullptr) {
      continue;
    }

    if (current.empty()) {
      current.push_back(segment);
      continue;
    }

    const auto * previous =
      current.back();

    const bool consecutive_index =
      previous != nullptr &&
      segment->net_segment_index ==
      previous->net_segment_index + 1U;

    if (
      previous != nullptr &&
      consecutive_index &&
      share_node(*previous, *segment))
    {
      current.push_back(segment);
    } else {
      runs.push_back(current);
      current.clear();
      current.push_back(segment);
    }
  }

  if (!current.empty()) {
    runs.push_back(current);
  }

  return runs;
}

static NetRun materialize_run(
  const std::vector<
    const NetTrajectorySegment *> & edges,
  const TrajectorySegment & original)
{
  NetRun output;

  if (edges.empty() || edges.front() == nullptr) {
    return output;
  }

  const auto * first = edges.front();

  uint64_t current_node = 0U;
  geometry_msgs::msg::Point start_point;
  geometry_msgs::msg::Point current_point;

  if (
    edges.size() >= 2U &&
    edges[1U] != nullptr)
  {
    const auto * second = edges[1U];

    uint64_t shared = 0U;
    bool found_shared = false;

    const uint64_t candidates[] = {
      first->start_node_id,
      first->end_node_id};

    for (const uint64_t candidate :
      candidates)
    {
      if (
        candidate == second->start_node_id ||
        candidate == second->end_node_id)
      {
        shared = candidate;
        found_shared = true;
        break;
      }
    }

    if (found_shared) {
      if (first->start_node_id == shared) {
        start_point = first->end;
        current_point = first->start;
        current_node = first->start_node_id;
      } else {
        start_point = first->start;
        current_point = first->end;
        current_node = first->end_node_id;
      }
    } else {
      start_point = first->start;
      current_point = first->end;
      current_node = first->end_node_id;
    }
  } else {
    // A single surviving edge has no neighbouring net edge from which to infer
    // traversal direction. Use the ORIGINAL phase only to orient the already
    // selected edge; no collision/crop decision is made here.
    const double start_parameter =
      point_to_polyline_parameter(
      first->start,
      original);
    const double end_parameter =
      point_to_polyline_parameter(
      first->end,
      original);

    if (start_parameter <= end_parameter) {
      start_point = first->start;
      current_point = first->end;
      current_node = first->end_node_id;
    } else {
      start_point = first->end;
      current_point = first->start;
      current_node = first->start_node_id;
    }
  }

  output.points.push_back(
    start_point);
  output.points.push_back(
    current_point);
  output.length_m +=
    point_distance(
    start_point,
    current_point);

  for (std::size_t i = 1U;
    i < edges.size();
    ++i)
  {
    const auto * edge = edges[i];

    if (edge == nullptr) {
      break;
    }

    geometry_msgs::msg::Point next_point;
    uint64_t next_node = 0U;

    if (edge->start_node_id == current_node) {
      next_point = edge->end;
      next_node = edge->end_node_id;
    } else if (
      edge->end_node_id == current_node)
    {
      next_point = edge->start;
      next_node = edge->start_node_id;
    } else {
      // split_runs() should already have prevented this. Stop rather than
      // creating an artificial jump between disconnected pieces.
      break;
    }

    output.length_m +=
      point_distance(
      output.points.back(),
      next_point);

    output.points.push_back(
      next_point);

    current_node = next_node;
  }

  // The multi-edge route is normally oriented by net_segment_index. This final
  // check protects against a canonical edge orientation leaking into the first
  // edge in unusual cases.
  if (output.points.size() >= 2U) {
    const double first_parameter =
      point_to_polyline_parameter(
      output.points.front(),
      original);
    const double last_parameter =
      point_to_polyline_parameter(
      output.points.back(),
      original);

    if (first_parameter > last_parameter) {
      std::reverse(
        output.points.begin(),
        output.points.end());
    }
  }

  return output;
}

static TrajectorySegment materialize_phase_exact(
  const CroppedNetTrajectory & crop,
  const StaticTrajectory & original_trajectory,
  uint8_t phase)
{
  TrajectorySegment output;

  const auto ordered =
    phase_segments(crop, phase);

  if (ordered.empty()) {
    return output;
  }

  const auto runs =
    split_runs(ordered);

  std::vector<
    std::vector<const NetTrajectorySegment *>>
    non_empty_runs;

  for (const auto & run : runs) {
    if (!run.empty()) {
      non_empty_runs.push_back(run);
    }
  }

  if (non_empty_runs.empty()) {
    return output;
  }

  // CRITICAL:
  // StaticTrajectory/TrajectorySegment can encode only ONE continuous
  // polyline per phase. If the exact upstream crop contains two or more
  // disconnected retained pieces, selecting one of them (the old behavior)
  // would modify the trajectory. Joining them would be even worse because it
  // would create an artificial segment across the removed collision region.
  //
  // Therefore an exact conversion is possible only when the phase contains at
  // most one continuous retained run.
  if (non_empty_runs.size() != 1U) {
    std::ostringstream error;
    error
      << "phase=" << static_cast<unsigned int>(phase)
      << " contains " << non_empty_runs.size()
      << " disconnected retained net runs; legacy TrajectorySegment cannot "
      << "represent this crop exactly";
    throw std::runtime_error(error.str());
  }

  const TrajectorySegment & original =
    original_phase(
    original_trajectory,
    phase);

  const NetRun run =
    materialize_run(
    non_empty_runs.front(),
    original);

  if (run.points.size() < 2U) {
    return output;
  }

  output.x.reserve(run.points.size());
  output.y.reserve(run.points.size());
  output.z.reserve(run.points.size());

  for (const auto & point : run.points) {
    output.x.push_back(point.x);
    output.y.push_back(point.y);
    output.z.push_back(point.z);
  }

  return output;
}

static StaticTrajectory materialize_adjusted(
  const RequestedSupervisionTrajectory & request)
{
  const auto & detected =
    request.detected_collision;

  const auto & crop =
    request.cropped_trajectory;

  if (!crop.complete) {
    throw std::runtime_error(
            "cropped_trajectory.complete is false");
  }

  if (
    crop.trajectory_id !=
    detected.trajectory.trajectory_id)
  {
    throw std::runtime_error(
            "cropped_trajectory.trajectory_id does not match detected_collision.trajectory.trajectory_id");
  }

  if (
    !crop.frame_id.empty() &&
    !detected.trajectory.frame_id.empty() &&
    crop.frame_id !=
    detected.trajectory.frame_id)
  {
    throw std::runtime_error(
            "cropped_trajectory.frame_id does not match original trajectory frame_id");
  }

  StaticTrajectory output =
    detected.trajectory;

  // IMPORTANT:
  // deconfliction_manager has already decided which virtual-net nodes/edges
  // survive. This node only converts those retained edges into the legacy
  // StaticTrajectory representation required by /adjusted_trajectories.
  //
  // This conversion is allowed only if each phase is already one continuous
  // retained run. The node NEVER selects a subset and NEVER reconnects a gap.
  output.takeoff =
    materialize_phase_exact(
    crop,
    detected.trajectory,
    kTakeoff);

  output.mission =
    materialize_phase_exact(
    crop,
    detected.trajectory,
    kMission);

  output.landing =
    materialize_phase_exact(
    crop,
    detected.trajectory,
    kLanding);

  return output;
}

class SupervisionTrajectoryManagerNode :
  public rclcpp::Node
{
public:
  SupervisionTrajectoryManagerNode()
  : Node("supervision_trajectory_manager_node")
  {
    requested_topic_ =
      declare_parameter<std::string>(
      "requested_supervision_trajectories_topic",
      "/requested_supervision_trajectories");

    available_topic_ =
      declare_parameter<std::string>(
      "available_trajectories_topic",
      "/available_static_trajectories");

    adjusted_topic_ =
      declare_parameter<std::string>(
      "adjusted_trajectories_topic",
      "/adjusted_trajectories");

    supervised_topic_ =
      declare_parameter<std::string>(
      "supervised_trajectories_topic",
      "/supervised_trajectories");

    markers_topic_ =
      declare_parameter<std::string>(
      "supervision_markers_topic",
      "/supervision_trajectory_manager_markers");

    publish_period_ms_ =
      declare_parameter<int>(
      "publish_period_ms",
      1000);

    geometry_match_tolerance_m_ =
      declare_parameter<double>(
      "geometry_match_tolerance_m",
      1.0e-6);

    pending_line_width_ =
      declare_parameter<double>(
      "pending_line_width",
      0.12);

    supervised_node_scale_ =
      declare_parameter<double>(
      "supervised_node_scale",
      0.32);

    supervised_text_height_ =
      declare_parameter<double>(
      "supervised_text_height",
      0.28);

    validate_parameters();

    rclcpp::QoS qos(
      rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    requested_sub_ =
      create_subscription<
      RequestedSupervisionTrajectoryArray>(
      requested_topic_,
      qos,
      std::bind(
        &SupervisionTrajectoryManagerNode::
        requested_callback,
        this,
        std::placeholders::_1));

    available_sub_ =
      create_subscription<
      StaticTrajectoryArray>(
      available_topic_,
      qos,
      std::bind(
        &SupervisionTrajectoryManagerNode::
        available_callback,
        this,
        std::placeholders::_1));

    adjusted_pub_ =
      create_publisher<
      StaticTrajectoryArray>(
      adjusted_topic_,
      qos);

    supervised_pub_ =
      create_publisher<
      DetectedCollisionTrajectoryArray>(
      supervised_topic_,
      qos);

    markers_pub_ =
      create_publisher<MarkerArray>(
      markers_topic_,
      qos);

    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(
        publish_period_ms_),
      std::bind(
        &SupervisionTrajectoryManagerNode::
        publish,
        this));

    publish();

    RCLCPP_INFO(
      get_logger(),
      "Supervision trajectory manager | request='%s' "
      "[deconfliction_manager/RequestedSupervisionTrajectoryArray] | "
      "available='%s' | adjusted='%s' | supervised='%s' | %.3f Hz",
      requested_topic_.c_str(),
      available_topic_.c_str(),
      adjusted_topic_.c_str(),
      supervised_topic_.c_str(),
      1000.0 /
      static_cast<double>(
        publish_period_ms_));
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
      !absolute(adjusted_topic_) ||
      !absolute(supervised_topic_) ||
      !absolute(markers_topic_))
    {
      throw std::runtime_error(
              "All topics must be absolute");
    }

    if (publish_period_ms_ <= 0) {
      throw std::runtime_error(
              "publish_period_ms must be > 0");
    }

    if (
      !std::isfinite(
        geometry_match_tolerance_m_) ||
      geometry_match_tolerance_m_ < 0.0)
    {
      throw std::runtime_error(
              "geometry_match_tolerance_m must be finite and >= 0");
    }

    if (
      !std::isfinite(
        pending_line_width_) ||
      pending_line_width_ <= 0.0 ||
      !std::isfinite(
        supervised_node_scale_) ||
      supervised_node_scale_ <= 0.0 ||
      !std::isfinite(
        supervised_text_height_) ||
      supervised_text_height_ <= 0.0)
    {
      throw std::runtime_error(
              "Marker dimensions must be finite and > 0");
    }
  }

  static bool same_segment(
    const TrajectorySegment & first,
    const TrajectorySegment & second,
    double tolerance)
  {
    if (
      first.x.size() !=
      second.x.size() ||
      first.y.size() !=
      second.y.size() ||
      first.z.size() !=
      second.z.size())
    {
      return false;
    }

    for (std::size_t i = 0U;
      i < first.x.size();
      ++i)
    {
      if (
        std::abs(
          first.x[i] -
          second.x[i]) > tolerance ||
        std::abs(
          first.y[i] -
          second.y[i]) > tolerance ||
        std::abs(
          first.z[i] -
          second.z[i]) > tolerance)
      {
        return false;
      }
    }

    return true;
  }

  bool same_geometry(
    const StaticTrajectory & expected,
    const StaticTrajectory & observed) const
  {
    return
      expected.trajectory_id ==
      observed.trajectory_id &&
      expected.frame_id ==
      observed.frame_id &&
      same_segment(
        expected.takeoff,
        observed.takeoff,
        geometry_match_tolerance_m_) &&
      same_segment(
        expected.mission,
        observed.mission,
        geometry_match_tolerance_m_) &&
      same_segment(
        expected.landing,
        observed.landing,
        geometry_match_tolerance_m_);
  }

  static bool same_time(
    const builtin_interfaces::msg::Time & first,
    const builtin_interfaces::msg::Time & second)
  {
    return
      first.sec == second.sec &&
      first.nanosec == second.nanosec;
  }

  static bool valid_time_window(
    const StaticTrajectory & trajectory)
  {
    if (
      trajectory.operation_end_utc.sec >
      trajectory.operation_start_utc.sec)
    {
      return true;
    }

    if (
      trajectory.operation_end_utc.sec <
      trajectory.operation_start_utc.sec)
    {
      return false;
    }

    return
      trajectory.operation_end_utc.nanosec >
      trajectory.operation_start_utc.nanosec;
  }

  static bool periodic(
    const StaticTrajectory & trajectory)
  {
    return
      std::isfinite(
      trajectory.operation_frequency) &&
      trajectory.operation_frequency > 0.0;
  }

  void synchronize_periodic_times_from_available(
    StoredEntry & entry,
    const StaticTrajectory & available)
  {
    if (!periodic(entry.request.trajectory)) {
      return;
    }

    if (!valid_time_window(available)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Periodic supervised trajectory '%s' is AVAILABLE with an invalid "
        "execution window; keeping the previous supervised times",
        entry.request.trajectory.trajectory_id.c_str());

      return;
    }

    const bool changed =
      !entry.synchronized_time_received ||
      !same_time(
        entry.synchronized_start_utc,
        available.operation_start_utc) ||
      !same_time(
        entry.synchronized_end_utc,
        available.operation_end_utc);

    entry.synchronized_start_utc =
      available.operation_start_utc;
    entry.synchronized_end_utc =
      available.operation_end_utc;
    entry.synchronized_time_received = true;

    // If the trajectory ever becomes PENDING again, re-inject the latest
    // periodic occurrence rather than the original occurrence.
    entry.adjusted.operation_start_utc =
      available.operation_start_utc;
    entry.adjusted.operation_end_utc =
      available.operation_end_utc;

    if (changed) {
      RCLCPP_INFO(
        get_logger(),
        "Periodic supervised trajectory '%s' synchronized from AVAILABLE | "
        "next_start=%d.%09u | next_end=%d.%09u",
        entry.request.trajectory.trajectory_id.c_str(),
        available.operation_start_utc.sec,
        available.operation_start_utc.nanosec,
        available.operation_end_utc.sec,
        available.operation_end_utc.nanosec);
    }
  }

  void store_request(
    const RequestedSupervisionTrajectory & message)
  {
    const auto & request =
      message.detected_collision;

    const auto & crop =
      message.cropped_trajectory;

    const std::string & id =
      request.trajectory.trajectory_id;

    if (id.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring supervision request with empty trajectory_id");

      return;
    }

    if (
      !crop.trajectory_id.empty() &&
      crop.trajectory_id != id)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Ignoring supervision request '%s': crop trajectory_id='%s' differs",
        id.c_str(),
        crop.trajectory_id.c_str());

      return;
    }

    auto existing =
      stored_.find(id);

    if (!crop.complete) {
      if (existing == stored_.end()) {
        StoredEntry entry;
        entry.request = request;
        entry.crop = crop;
        entry.crop_complete = false;
        entry.adjusted_ready = false;
        entry.state = StoredState::PENDING;
        entry.synchronized_start_utc =
          request.trajectory.operation_start_utc;
        entry.synchronized_end_utc =
          request.trajectory.operation_end_utc;

        stored_[id] =
          std::move(entry);
      } else {
        // Do not regress a complete crop if a retained/incomplete snapshot is
        // received out of order. The collision evidence can still be refreshed.
        existing->second.request = request;

        if (!existing->second.crop_complete) {
          existing->second.crop = crop;
        }
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Supervision request '%s' has cropped_trajectory.complete=false; "
        "waiting for deconfliction_manager to publish the complete net crop",
        id.c_str());

      return;
    }

    StaticTrajectory new_adjusted;
    bool adjusted_ready = false;

    try {
      new_adjusted =
        materialize_adjusted(message);
      adjusted_ready = true;
    } catch (const std::exception & error) {
      // Preserve the exact upstream crop, but do NOT invent a different
      // StaticTrajectory. A disconnected crop requires a richer downstream
      // trajectory representation before it can be sent to /adjusted_trajectories.
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Supervision crop '%s' cannot be represented exactly as StaticTrajectory: %s. "
        "Nothing will be published on /adjusted_trajectories for this entry.",
        id.c_str(),
        error.what());
    }

    if (existing == stored_.end()) {
      StoredEntry entry;
      entry.request = request;
      entry.crop = crop;
      entry.crop_complete = true;
      entry.adjusted_ready =
        adjusted_ready;
      if (adjusted_ready) {
        entry.adjusted =
          std::move(new_adjusted);
      }
      entry.state =
        StoredState::PENDING;
      entry.synchronized_start_utc =
        request.trajectory.operation_start_utc;
      entry.synchronized_end_utc =
        request.trajectory.operation_end_utc;
      entry.synchronized_time_received = false;

      stored_[id] =
        std::move(entry);

      RCLCPP_INFO(
        get_logger(),
        "Stored supervision request '%s' from upstream crop | "
        "collision_nodes=%zu | collision_segments=%zu | "
        "retained_net_segments=%zu | adjusted_ready=%s",
        id.c_str(),
        request.collision_nodes.size(),
        request.collision_segments.size(),
        crop.retained_segments.size(),
        adjusted_ready ? "true" : "false");

      return;
    }

    const bool geometry_changed =
      existing->second.adjusted_ready != adjusted_ready ||
      (
        adjusted_ready &&
        (
          !existing->second.crop_complete ||
          !same_geometry(
            existing->second.adjusted,
            new_adjusted)
        )
      );

    const auto synchronized_start =
      existing->second.synchronized_start_utc;
    const auto synchronized_end =
      existing->second.synchronized_end_utc;
    const bool synchronized_received =
      existing->second.synchronized_time_received;

    existing->second.request = request;
    existing->second.crop = crop;
    existing->second.crop_complete = true;
    existing->second.adjusted_ready =
      adjusted_ready;

    if (geometry_changed) {
      if (adjusted_ready) {
        existing->second.adjusted =
          std::move(new_adjusted);
      } else {
        // Clear any previous materialization so an old geometry can never be
        // republished after the authoritative upstream crop became
        // non-representable.
        existing->second.adjusted =
          StaticTrajectory{};
      }
      existing->second.state =
        StoredState::PENDING;
      existing->second.synchronized_start_utc =
        request.trajectory.operation_start_utc;
      existing->second.synchronized_end_utc =
        request.trajectory.operation_end_utc;
      existing->second.synchronized_time_received =
        false;

      RCLCPP_INFO(
        get_logger(),
        "Upstream cropped geometry changed for '%s'; returned to PENDING",
        id.c_str());

      return;
    }

    // A retained/repeated deconfliction request may still carry the original
    // periodic time window. Preserve a newer window already learned from
    // /available_static_trajectories.
    if (
      adjusted_ready &&
      synchronized_received &&
      periodic(
        existing->second.request.trajectory))
    {
      existing->second.synchronized_start_utc =
        synchronized_start;
      existing->second.synchronized_end_utc =
        synchronized_end;
      existing->second.synchronized_time_received =
        true;
      existing->second.adjusted.operation_start_utc =
        synchronized_start;
      existing->second.adjusted.operation_end_utc =
        synchronized_end;
    }
  }

  void requested_callback(
    const RequestedSupervisionTrajectoryArray::
    SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    latest_header_ =
      message->header;

    // Do NOT clear stored_. Once the adjusted trajectory becomes available,
    // the collision may disappear upstream and therefore disappear from
    // /requested_supervision_trajectories. The supervision lifecycle must
    // persist until AVAILABLE no longer contains the expected adjusted
    // geometry.
    for (const auto & request :
      message->trajectories)
    {
      store_request(request);
    }

    verify_available();
    publish_locked();
  }

  void available_callback(
    const StaticTrajectoryArray::
    SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    available_ = *message;
    available_received_ = true;

    if (!message->header.frame_id.empty()) {
      latest_header_ =
        message->header;
    }

    verify_available();
    publish_locked();
  }

  void verify_available()
  {
    if (!available_received_) {
      return;
    }

    std::map<
      std::string,
      const StaticTrajectory *> by_id;

    for (const auto & trajectory :
      available_.trajectories)
    {
      if (!trajectory.trajectory_id.empty()) {
        by_id[
          trajectory.trajectory_id] =
          &trajectory;
      }
    }

    for (auto & [id, entry] : stored_) {
      if (
        !entry.crop_complete ||
        !entry.adjusted_ready)
      {
        continue;
      }

      const auto observed =
        by_id.find(id);

      const bool available_match =
        observed != by_id.end() &&
        observed->second != nullptr &&
        same_geometry(
        entry.adjusted,
        *observed->second);

      if (available_match) {
        // The AVAILABLE object is the adjusted/cropped StaticTrajectory.
        // Copy ONLY its current total-operation timestamps to the original
        // collision payload for periodic supervision.
        synchronize_periodic_times_from_available(
          entry,
          *observed->second);
      }

      if (entry.state ==
        StoredState::PENDING)
      {
        if (!available_match) {
          if (
            observed != by_id.end() &&
            observed->second != nullptr)
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              5000,
              "'%s' is AVAILABLE by id, but its geometry does not match the "
              "upstream cropped geometry materialized for /adjusted_trajectories",
              id.c_str());
          }

          continue;
        }

        entry.state =
          StoredState::SUPERVISED;

        RCLCPP_INFO(
          get_logger(),
          "'%s' confirmed AVAILABLE; supervision activated",
          id.c_str());

        continue;
      }

      // If the adjusted geometry disappears from AVAILABLE, requeue the same
      // upstream crop through /adjusted_trajectories.
      if (!available_match) {
        entry.state =
          StoredState::PENDING;

        RCLCPP_WARN(
          get_logger(),
          "Supervised trajectory '%s' is no longer AVAILABLE with the expected "
          "upstream crop; re-queued",
          id.c_str());
      }
    }
  }

  StaticTrajectoryArray adjusted_snapshot() const
  {
    StaticTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    std::vector<
      const StoredEntry *> ordered;

    for (const auto & [_, entry] :
      stored_)
    {
      if (
        entry.state ==
        StoredState::PENDING &&
        entry.crop_complete &&
        entry.adjusted_ready)
      {
        ordered.push_back(&entry);
      }
    }

    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](
        const StoredEntry * first,
        const StoredEntry * second)
      {
        if (
          first->adjusted.priority !=
          second->adjusted.priority)
        {
          return
            first->adjusted.priority <
            second->adjusted.priority;
        }

        if (
          first->adjusted.ua_id !=
          second->adjusted.ua_id)
        {
          return
            first->adjusted.ua_id <
            second->adjusted.ua_id;
        }

        return
          first->adjusted.trajectory_id <
          second->adjusted.trajectory_id;
      });

    output.trajectories.reserve(
      ordered.size());

    for (const auto * entry : ordered) {
      output.trajectories.push_back(
        entry->adjusted);
    }

    return output;
  }

  DetectedCollisionTrajectoryArray
  supervised_snapshot() const
  {
    DetectedCollisionTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    std::vector<
      const StoredEntry *> ordered;

    for (const auto & [_, entry] :
      stored_)
    {
      if (
        entry.state ==
        StoredState::SUPERVISED)
      {
        ordered.push_back(&entry);
      }
    }

    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](
        const StoredEntry * first,
        const StoredEntry * second)
      {
        if (
          first->request.trajectory.priority !=
          second->request.trajectory.priority)
        {
          return
            first->request.trajectory.priority <
            second->request.trajectory.priority;
        }

        if (
          first->request.trajectory.ua_id !=
          second->request.trajectory.ua_id)
        {
          return
            first->request.trajectory.ua_id <
            second->request.trajectory.ua_id;
        }

        return
          first->request.trajectory.trajectory_id <
          second->request.trajectory.trajectory_id;
      });

    output.trajectories.reserve(
      ordered.size());

    for (const auto * entry : ordered) {
      // Keep publishing the ORIGINAL full DetectedCollisionTrajectory. The new
      // upstream crop is an acquisition/planning artifact only.
      DetectedCollisionTrajectory supervised =
        entry->request;

      if (
        entry->synchronized_time_received &&
        periodic(supervised.trajectory))
      {
        supervised.trajectory.operation_start_utc =
          entry->synchronized_start_utc;

        supervised.trajectory.operation_end_utc =
          entry->synchronized_end_utc;
      }

      output.trajectories.push_back(
        std::move(supervised));
    }

    return output;
  }

  MarkerArray markers() const
  {
    MarkerArray output;

    Marker delete_all;
    delete_all.header.stamp = now();
    delete_all.header.frame_id =
      latest_header_.frame_id;
    delete_all.action = Marker::DELETEALL;

    output.markers.push_back(
      delete_all);

    int marker_id = 1;

    for (const auto & [id, entry] :
      stored_)
    {
      if (
        entry.state ==
        StoredState::PENDING)
      {
        if (
          !entry.crop_complete ||
          entry.crop.retained_segments.empty())
        {
          continue;
        }

        // Display the exact upstream crop as independent LINE_LIST edges. This
        // does not hide discarded disconnected components merely because the
        // legacy StaticTrajectory output can retain only one run per phase.
        Marker line;
        line.header.stamp = now();
        line.header.frame_id =
          entry.request.trajectory.frame_id;
        line.ns =
          "supervision/pending_upstream_crop/" +
          id;
        line.id = marker_id++;
        line.type = Marker::LINE_LIST;
        line.action = Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x =
          pending_line_width_;
        line.color =
          color(
          0.10F,
          0.75F,
          1.00F,
          1.00F);

        for (const auto & segment :
          entry.crop.retained_segments)
        {
          line.points.push_back(
            segment.start);
          line.points.push_back(
            segment.end);
        }

        output.markers.push_back(
          std::move(line));

        continue;
      }

      const auto collision_color =
        color(
        1.00F,
        0.64F,
        0.05F,
        1.00F);

      for (const auto & node :
        entry.request.collision_nodes)
      {
        Marker sphere;
        sphere.header.stamp = now();
        sphere.header.frame_id =
          entry.request.trajectory.frame_id;
        sphere.ns =
          "supervision/collision_nodes/" +
          id;
        sphere.id = marker_id++;
        sphere.type = Marker::SPHERE;
        sphere.action = Marker::ADD;
        sphere.pose.orientation.w = 1.0;
        sphere.pose.position =
          node.position;
        sphere.scale.x =
          supervised_node_scale_;
        sphere.scale.y =
          supervised_node_scale_;
        sphere.scale.z =
          supervised_node_scale_;
        sphere.color =
          collision_color;

        output.markers.push_back(
          std::move(sphere));

        Marker text;
        text.header.stamp = now();
        text.header.frame_id =
          entry.request.trajectory.frame_id;
        text.ns =
          "supervision/collision_nodes/" +
          id;
        text.id = marker_id++;
        text.type =
          Marker::TEXT_VIEW_FACING;
        text.action = Marker::ADD;
        text.pose.orientation.w = 1.0;
        text.pose.position =
          node.position;
        text.pose.position.z +=
          supervised_node_scale_ * 0.85;
        text.scale.z =
          supervised_text_height_;
        text.color =
          collision_color;
        text.text =
          id +
          " | node=" +
          std::to_string(node.node_id) +
          " | vs=" +
          join_ids(
          node.conflicting_trajectory_ids);

        output.markers.push_back(
          std::move(text));
      }
    }

    return output;
  }

  void publish_locked()
  {
    adjusted_pub_->publish(
      adjusted_snapshot());

    supervised_pub_->publish(
      supervised_snapshot());

    markers_pub_->publish(
      markers());
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(
      mutex_);

    verify_available();
    publish_locked();
  }

  std::string requested_topic_;
  std::string available_topic_;
  std::string adjusted_topic_;
  std::string supervised_topic_;
  std::string markers_topic_;

  int publish_period_ms_{1000};

  double geometry_match_tolerance_m_{1.0e-6};
  double pending_line_width_{0.12};
  double supervised_node_scale_{0.32};
  double supervised_text_height_{0.28};

  mutable std::mutex mutex_;

  std_msgs::msg::Header latest_header_;

  StaticTrajectoryArray available_;
  bool available_received_{false};

  std::map<
    std::string,
    StoredEntry> stored_;

  rclcpp::Subscription<
    RequestedSupervisionTrajectoryArray>::SharedPtr
    requested_sub_;

  rclcpp::Subscription<
    StaticTrajectoryArray>::SharedPtr
    available_sub_;

  rclcpp::Publisher<
    StaticTrajectoryArray>::SharedPtr
    adjusted_pub_;

  rclcpp::Publisher<
    DetectedCollisionTrajectoryArray>::SharedPtr
    supervised_pub_;

  rclcpp::Publisher<MarkerArray>::SharedPtr
    markers_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace supervision_trajectory_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        supervision_trajectory_manager::
        SupervisionTrajectoryManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "supervision_trajectory_manager_node"),
      "Fatal: %s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
