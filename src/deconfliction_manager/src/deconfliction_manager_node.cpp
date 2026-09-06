#include <rclcpp/rclcpp.hpp>

#include <a_space_virtual_net/msg/net_loaded_trajectory.hpp>
#include <a_space_virtual_net/msg/net_loaded_trajectory_array.hpp>
#include <a_space_virtual_net/msg/net_trajectory_segment.hpp>

#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>
#include <collision_detection/msg/grid_collision_node.hpp>

#include <deconfliction_manager/msg/cropped_net_node.hpp>
#include <deconfliction_manager/msg/cropped_net_segment.hpp>
#include <deconfliction_manager/msg/cropped_net_trajectory.hpp>
#include <deconfliction_manager/msg/requested_supervision_trajectory.hpp>
#include <deconfliction_manager/msg/requested_supervision_trajectory_array.hpp>

#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>

#include <geometry_msgs/msg/point.hpp>
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
#include <tuple>
#include <utility>
#include <vector>

namespace deconfliction_manager
{

using NetLoadedTrajectory =
  a_space_virtual_net::msg::NetLoadedTrajectory;
using NetLoadedTrajectoryArray =
  a_space_virtual_net::msg::NetLoadedTrajectoryArray;
using NetTrajectorySegment =
  a_space_virtual_net::msg::NetTrajectorySegment;

using DetectedCollisionTrajectory =
  collision_detection::msg::DetectedCollisionTrajectory;
using DetectedCollisionTrajectoryArray =
  collision_detection::msg::DetectedCollisionTrajectoryArray;
using GridCollisionNode =
  collision_detection::msg::GridCollisionNode;

using CroppedNetNode =
  deconfliction_manager::msg::CroppedNetNode;
using CroppedNetSegment =
  deconfliction_manager::msg::CroppedNetSegment;
using CroppedNetTrajectory =
  deconfliction_manager::msg::CroppedNetTrajectory;
using RequestedSupervisionTrajectory =
  deconfliction_manager::msg::RequestedSupervisionTrajectory;
using RequestedSupervisionTrajectoryArray =
  deconfliction_manager::msg::RequestedSupervisionTrajectoryArray;

using StaticTrajectory =
  static_trajectory_manager::msg::StaticTrajectory;
using TrajectorySegment =
  static_trajectory_manager::msg::TrajectorySegment;

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct SupervisionState
{
  DetectedCollisionTrajectory detected;
  CroppedNetTrajectory cropped;
};

constexpr uint8_t kTakeoffPhase = 0U;
constexpr uint8_t kMissionPhase = 1U;
constexpr uint8_t kLandingPhase = 2U;
constexpr uint32_t kNoMissionIndex =
  std::numeric_limits<uint32_t>::max();

struct GeometryEdgeKey
{
  uint8_t phase{0U};
  uint32_t mission_index{kNoMissionIndex};
  uint64_t edge_id{0U};

  bool operator<(const GeometryEdgeKey & other) const
  {
    return std::tie(
      phase,
      mission_index,
      edge_id) <
      std::tie(
      other.phase,
      other.mission_index,
      other.edge_id);
  }
};

struct RetainedNodeAccumulator
{
  geometry_msgs::msg::Point position;
  std::set<uint8_t> phases;
  std::set<uint32_t> mission_indices;
};

struct SegmentOrigin
{
  uint32_t mission_index{kNoMissionIndex};
  uint32_t source_repetition{0U};
  uint32_t mission_source_segment_index{0U};
};

struct OrientedRun
{
  std::vector<geometry_msgs::msg::Point> points;
  uint32_t first_net_segment_index{0U};
};

static std::string join_ids(
  const std::vector<std::string> & ids)
{
  std::ostringstream stream;

  for (std::size_t index = 0U;
    index < ids.size();
    ++index)
  {
    if (index != 0U) {
      stream << ",";
    }

    stream << ids[index];
  }

  return stream.str();
}

class DeconflictionManagerNode : public rclcpp::Node
{
public:
  DeconflictionManagerNode()
  : Node("deconfliction_manager_node")
  {
    detected_topic_ =
      declare_parameter<std::string>(
      "detected_collision_trajectories_topic",
      "/detected_collision_trajectories");

    net_loaded_topic_ =
      declare_parameter<std::string>(
      "net_loaded_trajectories_topic",
      "/net_loaded_trajectories");

    supervision_topic_ =
      declare_parameter<std::string>(
      "requested_supervision_trajectories_topic",
      "/requested_supervision_trajectories");

    manual_topic_ =
      declare_parameter<std::string>(
      "manual_adjustment_trajectories_topic",
      "/manual_adjustment_trajectories");

    markers_topic_ =
      declare_parameter<std::string>(
      "deconfliction_markers_topic",
      "/deconfliction_manager_markers");

    collision_node_threshold_ =
      declare_parameter<int>(
      "collision_node_threshold",
      5);

    publish_period_ms_ =
      declare_parameter<int>(
      "publish_period_ms",
      1000);

    supervision_node_scale_ =
      declare_parameter<double>(
      "supervision_node_scale",
      0.30);

    supervision_text_height_ =
      declare_parameter<double>(
      "supervision_text_height",
      0.28);

    supervision_crop_line_width_ =
      declare_parameter<double>(
      "supervision_crop_line_width",
      0.12);

    manual_line_width_ =
      declare_parameter<double>(
      "manual_line_width",
      0.14);

    manual_text_height_ =
      declare_parameter<double>(
      "manual_text_height",
      0.36);

    validate_parameters();

    rclcpp::QoS snapshot_qos(
      rclcpp::KeepLast(1));
    snapshot_qos.reliable();
    snapshot_qos.transient_local();

    detected_subscription_ =
      create_subscription<
      DetectedCollisionTrajectoryArray>(
      detected_topic_,
      snapshot_qos,
      std::bind(
        &DeconflictionManagerNode::
        detected_callback,
        this,
        std::placeholders::_1));

    net_loaded_subscription_ =
      create_subscription<
      NetLoadedTrajectoryArray>(
      net_loaded_topic_,
      snapshot_qos,
      std::bind(
        &DeconflictionManagerNode::
        net_loaded_callback,
        this,
        std::placeholders::_1));

    supervision_publisher_ =
      create_publisher<
      RequestedSupervisionTrajectoryArray>(
      supervision_topic_,
      snapshot_qos);

    manual_publisher_ =
      create_publisher<
      DetectedCollisionTrajectoryArray>(
      manual_topic_,
      snapshot_qos);

    marker_publisher_ =
      create_publisher<MarkerArray>(
      markers_topic_,
      snapshot_qos);

    publish_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(
        publish_period_ms_),
      std::bind(
        &DeconflictionManagerNode::
        publish_snapshots,
        this));

    publish_snapshots();

    RCLCPP_INFO(
      get_logger(),
      "Deconfliction manager ready | detected='%s' | net='%s' | "
      "supervision='%s' | manual='%s' | node_threshold=%d | "
      "publish_period=%d ms",
      detected_topic_.c_str(),
      net_loaded_topic_.c_str(),
      supervision_topic_.c_str(),
      manual_topic_.c_str(),
      collision_node_threshold_,
      publish_period_ms_);
  }

private:
  void validate_parameters() const
  {
    const auto absolute_topic =
      [](const std::string & topic)
      {
        return
          !topic.empty() &&
          topic.front() == '/';
      };

    if (
      !absolute_topic(detected_topic_) ||
      !absolute_topic(net_loaded_topic_) ||
      !absolute_topic(supervision_topic_) ||
      !absolute_topic(manual_topic_) ||
      !absolute_topic(markers_topic_))
    {
      throw std::runtime_error(
              "All configured topics must be absolute");
    }

    if (collision_node_threshold_ < 0) {
      throw std::runtime_error(
              "collision_node_threshold must be >= 0");
    }

    if (publish_period_ms_ <= 0) {
      throw std::runtime_error(
              "publish_period_ms must be > 0");
    }

    if (
      !std::isfinite(supervision_node_scale_) ||
      supervision_node_scale_ <= 0.0 ||
      !std::isfinite(
        supervision_text_height_) ||
      supervision_text_height_ <= 0.0 ||
      !std::isfinite(
        supervision_crop_line_width_) ||
      supervision_crop_line_width_ <= 0.0 ||
      !std::isfinite(manual_line_width_) ||
      manual_line_width_ <= 0.0 ||
      !std::isfinite(manual_text_height_) ||
      manual_text_height_ <= 0.0)
    {
      throw std::runtime_error(
              "Marker dimensions must be finite and > 0");
    }
  }

  static std::set<uint64_t>
  unique_collision_node_ids(
    const DetectedCollisionTrajectory & detected)
  {
    std::set<uint64_t> ids;

    for (const auto & node :
      detected.collision_nodes)
    {
      ids.insert(node.node_id);
    }

    return ids;
  }

  std::size_t collision_node_count(
    const DetectedCollisionTrajectory & detected) const
  {
    // The classification threshold is geometry-only:
    // one virtual-net node is counted once regardless of how many partial
    // mission repetitions traverse it or how many collision segments touch it.
    return
      unique_collision_node_ids(
      detected).size();
  }

  static std::vector<
    DetectedCollisionTrajectory>
  ordered_input(
    const DetectedCollisionTrajectoryArray & message)
  {
    std::map<
      std::string,
      DetectedCollisionTrajectory> by_id;

    for (const auto & detected :
      message.trajectories)
    {
      const auto & trajectory_id =
        detected.trajectory.trajectory_id;

      if (trajectory_id.empty()) {
        continue;
      }

      by_id[trajectory_id] = detected;
    }

    std::vector<
      DetectedCollisionTrajectory> ordered;
    ordered.reserve(by_id.size());

    for (const auto & [_, detected] : by_id) {
      ordered.push_back(detected);
    }

    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](
        const DetectedCollisionTrajectory & first,
        const DetectedCollisionTrajectory & second)
      {
        if (
          first.trajectory.priority !=
          second.trajectory.priority)
        {
          return
            first.trajectory.priority <
            second.trajectory.priority;
        }

        if (
          first.trajectory.ua_id !=
          second.trajectory.ua_id)
        {
          return
            first.trajectory.ua_id <
            second.trajectory.ua_id;
        }

        return
          first.trajectory.trajectory_id <
          second.trajectory.trajectory_id;
      });

    return ordered;
  }

  static std::vector<
    const NetTrajectorySegment *>
  ordered_net_segments(
    const NetLoadedTrajectory & loaded)
  {
    std::vector<
      const NetTrajectorySegment *> ordered;

    ordered.reserve(
      loaded.segments.size());

    for (const auto & segment :
      loaded.segments)
    {
      ordered.push_back(&segment);
    }

    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](
        const NetTrajectorySegment * first,
        const NetTrajectorySegment * second)
      {
        return
          first->net_segment_index <
          second->net_segment_index;
      });

    return ordered;
  }

  static std::size_t trajectory_segment_point_count(
    const TrajectorySegment & segment)
  {
    return
      std::min(
      segment.x.size(),
      std::min(
        segment.y.size(),
        segment.z.size()));
  }

  static uint32_t trajectory_segment_edge_count(
    const TrajectorySegment & segment)
  {
    const std::size_t count =
      trajectory_segment_point_count(segment);

    if (count < 2U) {
      return 0U;
    }

    return
      static_cast<uint32_t>(
      count - 1U);
  }

  static std::vector<uint32_t>
  mission_source_edge_counts(
    const StaticTrajectory & trajectory)
  {
    std::vector<uint32_t> counts;
    counts.reserve(
      trajectory.mission.size());

    for (const auto & mission :
      trajectory.mission)
    {
      counts.push_back(
        trajectory_segment_edge_count(
          mission));
    }

    return counts;
  }

  static uint32_t mission_cycle_source_edge_count(
    const std::vector<uint32_t> & counts)
  {
    uint64_t total = 0U;

    for (const uint32_t count : counts) {
      total += count;
    }

    if (
      total >
      static_cast<uint64_t>(
        std::numeric_limits<uint32_t>::max()))
    {
      throw std::runtime_error(
              "Mission source-edge count exceeds uint32 range");
    }

    return static_cast<uint32_t>(total);
  }

  static SegmentOrigin segment_origin(
    const StaticTrajectory & trajectory,
    const NetTrajectorySegment & segment)
  {
    SegmentOrigin output;

    if (segment.phase != kMissionPhase) {
      return output;
    }

    const auto counts =
      mission_source_edge_counts(
      trajectory);

    const uint32_t cycle_edge_count =
      mission_cycle_source_edge_count(
      counts);

    if (cycle_edge_count == 0U) {
      return output;
    }

    output.source_repetition =
      segment.original_phase_segment_index /
      cycle_edge_count;

    const uint32_t local_index =
      segment.original_phase_segment_index %
      cycle_edge_count;

    uint32_t offset = 0U;

    for (std::size_t mission_index = 0U;
      mission_index < counts.size();
      ++mission_index)
    {
      const uint32_t count =
        counts[mission_index];

      if (
        local_index >= offset &&
        local_index < offset + count)
      {
        output.mission_index =
          static_cast<uint32_t>(
          mission_index);

        output.mission_source_segment_index =
          local_index - offset;

        return output;
      }

      offset += count;
    }

    return output;
  }

  static void accumulate_retained_node(
    std::map<
      uint64_t,
      RetainedNodeAccumulator> & nodes,
    uint64_t node_id,
    const geometry_msgs::msg::Point & position,
    uint8_t phase,
    uint32_t mission_index,
    const std::set<uint64_t> & forbidden_nodes)
  {
    if (
      forbidden_nodes.count(
        node_id) != 0U)
    {
      return;
    }

    auto & node = nodes[node_id];
    node.position = position;
    node.phases.insert(phase);

    if (mission_index != kNoMissionIndex) {
      node.mission_indices.insert(
        mission_index);
    }
  }

  static bool share_node(
    const NetTrajectorySegment & first,
    const NetTrajectorySegment & second)
  {
    return
      first.start_node_id ==
      second.start_node_id ||
      first.start_node_id ==
      second.end_node_id ||
      first.end_node_id ==
      second.start_node_id ||
      first.end_node_id ==
      second.end_node_id;
  }

  static geometry_msgs::msg::Point
  trajectory_segment_point(
    const TrajectorySegment & segment,
    std::size_t index)
  {
    geometry_msgs::msg::Point point;
    point.x = segment.x[index];
    point.y = segment.y[index];
    point.z = segment.z[index];
    return point;
  }

  static double squared_distance(
    const geometry_msgs::msg::Point & first,
    const geometry_msgs::msg::Point & second)
  {
    const double dx =
      first.x - second.x;
    const double dy =
      first.y - second.y;
    const double dz =
      first.z - second.z;

    return
      dx * dx +
      dy * dy +
      dz * dz;
  }

  static double point_to_polyline_parameter(
    const geometry_msgs::msg::Point & query,
    const TrajectorySegment & original)
  {
    const std::size_t count =
      trajectory_segment_point_count(
      original);

    if (count == 0U) {
      return 0.0;
    }

    if (count == 1U) {
      return std::sqrt(
        squared_distance(
          query,
          trajectory_segment_point(
            original,
            0U)));
    }

    double best_distance_sq =
      std::numeric_limits<double>::infinity();
    double best_parameter = 0.0;
    double cumulative = 0.0;

    for (std::size_t index = 1U;
      index < count;
      ++index)
    {
      const auto start =
        trajectory_segment_point(
        original,
        index - 1U);

      const auto end =
        trajectory_segment_point(
        original,
        index);

      const double vx =
        end.x - start.x;
      const double vy =
        end.y - start.y;
      const double vz =
        end.z - start.z;

      const double wx =
        query.x - start.x;
      const double wy =
        query.y - start.y;
      const double wz =
        query.z - start.z;

      const double length_sq =
        vx * vx +
        vy * vy +
        vz * vz;

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

      geometry_msgs::msg::Point closest;
      closest.x =
        start.x + ratio * vx;
      closest.y =
        start.y + ratio * vy;
      closest.z =
        start.z + ratio * vz;

      const double distance_sq =
        squared_distance(
        query,
        closest);

      if (distance_sq < best_distance_sq) {
        best_distance_sq =
          distance_sq;
        best_parameter =
          cumulative +
          ratio * length;
      }

      cumulative += length;
    }

    return best_parameter;
  }

  static const TrajectorySegment &
  original_phase_segment(
    const StaticTrajectory & trajectory,
    uint8_t phase,
    uint32_t mission_index)
  {
    if (phase == kTakeoffPhase) {
      return trajectory.takeoff;
    }

    if (phase == kLandingPhase) {
      return trajectory.landing;
    }

    if (
      phase != kMissionPhase ||
      mission_index >=
      trajectory.mission.size())
    {
      throw std::runtime_error(
              "Invalid phase/mission_index while materializing cropped trajectory");
    }

    return
      trajectory.mission[
      mission_index];
  }

  static OrientedRun orient_run(
    const std::vector<
      const CroppedNetSegment *> & run,
    const TrajectorySegment & original)
  {
    OrientedRun output;

    if (
      run.empty() ||
      run.front() == nullptr)
    {
      return output;
    }

    output.first_net_segment_index =
      run.front()->segment.net_segment_index;

    const auto & first =
      run.front()->segment;

    geometry_msgs::msg::Point first_point;
    geometry_msgs::msg::Point second_point;
    uint64_t current_node = 0U;

    if (
      run.size() >= 2U &&
      run[1U] != nullptr)
    {
      const auto & next =
        run[1U]->segment;

      const bool start_is_shared =
        first.start_node_id ==
        next.start_node_id ||
        first.start_node_id ==
        next.end_node_id;

      const bool end_is_shared =
        first.end_node_id ==
        next.start_node_id ||
        first.end_node_id ==
        next.end_node_id;

      if (
        start_is_shared &&
        !end_is_shared)
      {
        first_point = first.end;
        second_point = first.start;
        current_node =
          first.start_node_id;
      } else if (
        end_is_shared &&
        !start_is_shared)
      {
        first_point = first.start;
        second_point = first.end;
        current_node =
          first.end_node_id;
      } else {
        const double start_parameter =
          point_to_polyline_parameter(
          first.start,
          original);

        const double end_parameter =
          point_to_polyline_parameter(
          first.end,
          original);

        if (start_parameter <= end_parameter) {
          first_point = first.start;
          second_point = first.end;
          current_node =
            first.end_node_id;
        } else {
          first_point = first.end;
          second_point = first.start;
          current_node =
            first.start_node_id;
        }
      }
    } else {
      const double start_parameter =
        point_to_polyline_parameter(
        first.start,
        original);

      const double end_parameter =
        point_to_polyline_parameter(
        first.end,
        original);

      if (start_parameter <= end_parameter) {
        first_point = first.start;
        second_point = first.end;
        current_node =
          first.end_node_id;
      } else {
        first_point = first.end;
        second_point = first.start;
        current_node =
          first.start_node_id;
      }
    }

    output.points.push_back(
      first_point);
    output.points.push_back(
      second_point);

    for (std::size_t index = 1U;
      index < run.size();
      ++index)
    {
      if (run[index] == nullptr) {
        break;
      }

      const auto & edge =
        run[index]->segment;

      geometry_msgs::msg::Point next_point;
      uint64_t next_node = 0U;

      if (
        edge.start_node_id ==
        current_node)
      {
        next_point =
          edge.end;
        next_node =
          edge.end_node_id;
      } else if (
        edge.end_node_id ==
        current_node)
      {
        next_point =
          edge.start;
        next_node =
          edge.start_node_id;
      } else {
        break;
      }

      output.points.push_back(
        next_point);
      current_node = next_node;
    }

    return output;
  }

  static TrajectorySegment trajectory_segment_from_points(
    const std::vector<
      geometry_msgs::msg::Point> & points)
  {
    TrajectorySegment output;

    output.x.reserve(points.size());
    output.y.reserve(points.size());
    output.z.reserve(points.size());

    for (const auto & point : points) {
      output.x.push_back(point.x);
      output.y.push_back(point.y);
      output.z.push_back(point.z);
    }

    return output;
  }

  static std::vector<
    std::vector<const CroppedNetSegment *>>
  continuous_runs(
    std::vector<
      const CroppedNetSegment *> ordered)
  {
    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](
        const CroppedNetSegment * first,
        const CroppedNetSegment * second)
      {
        return
          first->segment.net_segment_index <
          second->segment.net_segment_index;
      });

    std::vector<
      std::vector<const CroppedNetSegment *>>
      runs;

    std::vector<
      const CroppedNetSegment *> current;

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

      const bool consecutive =
        previous != nullptr &&
        segment->segment.net_segment_index ==
        previous->segment.net_segment_index +
        1U;

      const bool connected =
        previous != nullptr &&
        share_node(
          previous->segment,
          segment->segment);

      if (
        consecutive &&
        connected)
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

  static bool materialize_cropped_static_trajectory(
    CroppedNetTrajectory & crop,
    const StaticTrajectory & original)
  {
    StaticTrajectory adjusted =
      original;

    adjusted.takeoff =
      TrajectorySegment{};
    adjusted.mission.clear();
    adjusted.landing =
      TrajectorySegment{};

    std::map<
      uint32_t,
      std::vector<
        const CroppedNetSegment *>>
      mission_segments;

    std::vector<
      const CroppedNetSegment *>
      takeoff_segments;

    std::vector<
      const CroppedNetSegment *>
      landing_segments;

    for (const auto & wrapped :
      crop.retained_segments)
    {
      const auto & segment =
        wrapped.segment;

      if (segment.phase ==
        kTakeoffPhase)
      {
        takeoff_segments.push_back(
          &wrapped);
      } else if (
        segment.phase ==
        kLandingPhase)
      {
        landing_segments.push_back(
          &wrapped);
      } else if (
        segment.phase ==
        kMissionPhase &&
        wrapped.mission_index !=
        kNoMissionIndex)
      {
        mission_segments[
          wrapped.mission_index].
          push_back(&wrapped);
      }
    }

    const auto takeoff_runs =
      continuous_runs(
      takeoff_segments);

    if (takeoff_runs.size() > 1U) {
      return false;
    }

    if (!takeoff_runs.empty()) {
      const OrientedRun run =
        orient_run(
        takeoff_runs.front(),
        original.takeoff);

      adjusted.takeoff =
        trajectory_segment_from_points(
        run.points);
    }

    for (const auto & [mission_index, segments] :
      mission_segments)
    {
      if (
        mission_index >=
        original.mission.size())
      {
        return false;
      }

      const auto runs =
        continuous_runs(
        segments);

      for (const auto & run_segments :
        runs)
      {
        const OrientedRun run =
          orient_run(
          run_segments,
          original.mission[
            mission_index]);

        if (run.points.size() >= 2U) {
          adjusted.mission.push_back(
            trajectory_segment_from_points(
              run.points));
        }
      }
    }

    const auto landing_runs =
      continuous_runs(
      landing_segments);

    if (landing_runs.size() > 1U) {
      return false;
    }

    if (!landing_runs.empty()) {
      const OrientedRun run =
        orient_run(
        landing_runs.front(),
        original.landing);

      adjusted.landing =
        trajectory_segment_from_points(
        run.points);
    }

    crop.trajectory =
      std::move(adjusted);

    return true;
  }

  CroppedNetTrajectory build_crop(
    const DetectedCollisionTrajectory & detected)
  {
    CroppedNetTrajectory output;

    output.trajectory_id =
      detected.trajectory.trajectory_id;
    output.frame_id =
      detected.trajectory.frame_id;
    output.source_repetitions =
      detected.trajectory.repetitions;
    output.trajectory =
      detected.trajectory;

    const std::set<uint64_t> forbidden_nodes =
      unique_collision_node_ids(detected);

    output.unique_collision_node_count =
      static_cast<uint32_t>(
      forbidden_nodes.size());

    output.removed_node_ids.assign(
      forbidden_nodes.begin(),
      forbidden_nodes.end());

    std::set<uint64_t>
      explicitly_conflicting_edges;

    for (const auto & collision_segment :
      detected.collision_segments)
    {
      explicitly_conflicting_edges.insert(
        collision_segment.edge_id);
    }

    const auto loaded_it =
      loaded_by_id_.find(
      detected.trajectory.trajectory_id);

    if (loaded_it == loaded_by_id_.end()) {
      output.complete = false;
      output.trajectory_materialized = false;

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Cannot build net crop for '%s': trajectory is not present in '%s'",
        detected.trajectory.trajectory_id.c_str(),
        net_loaded_topic_.c_str());

      return output;
    }

    const NetLoadedTrajectory & loaded =
      loaded_it->second;

    output.complete = true;

    const auto ordered =
      ordered_net_segments(loaded);

    // Partial repetitions are collapsed, but different mission components are
    // deliberately NOT merged even when they traverse the same virtual-net
    // edge. This preserves the new mission[] semantics.
    std::set<GeometryEdgeKey>
      emitted_geometry;

    std::set<uint64_t>
      removed_edge_ids;

    std::map<
      uint64_t,
      RetainedNodeAccumulator>
      retained_nodes;

    for (const auto * segment : ordered) {
      if (segment == nullptr) {
        continue;
      }

      const SegmentOrigin origin =
        segment_origin(
        detected.trajectory,
        *segment);

      const uint32_t mission_index =
        segment->phase == kMissionPhase ?
        origin.mission_index :
        kNoMissionIndex;

      accumulate_retained_node(
        retained_nodes,
        segment->start_node_id,
        segment->start,
        segment->phase,
        mission_index,
        forbidden_nodes);

      accumulate_retained_node(
        retained_nodes,
        segment->end_node_id,
        segment->end,
        segment->phase,
        mission_index,
        forbidden_nodes);

      const bool endpoint_collision =
        forbidden_nodes.count(
        segment->start_node_id) != 0U ||
        forbidden_nodes.count(
        segment->end_node_id) != 0U;

      const bool edge_collision =
        explicitly_conflicting_edges.count(
        segment->edge_id) != 0U;

      const bool remove =
        endpoint_collision ||
        edge_collision;

      const GeometryEdgeKey key{
        segment->phase,
        mission_index,
        segment->edge_id};

      if (
        emitted_geometry.count(
          key) != 0U)
      {
        if (remove) {
          removed_edge_ids.insert(
            segment->edge_id);
        }

        continue;
      }

      emitted_geometry.insert(key);

      CroppedNetSegment wrapped;
      wrapped.mission_index =
        mission_index;
      wrapped.source_repetition =
        origin.source_repetition;
      wrapped.segment =
        *segment;

      if (remove) {
        output.removed_segments.push_back(
          std::move(wrapped));

        removed_edge_ids.insert(
          segment->edge_id);
      } else {
        output.retained_segments.push_back(
          std::move(wrapped));
      }
    }

    output.removed_edge_ids.assign(
      removed_edge_ids.begin(),
      removed_edge_ids.end());

    output.retained_nodes.reserve(
      retained_nodes.size());

    for (const auto & [node_id, accumulated] :
      retained_nodes)
    {
      CroppedNetNode node;
      node.node_id = node_id;
      node.position = accumulated.position;
      node.phases.assign(
        accumulated.phases.begin(),
        accumulated.phases.end());
      node.mission_indices.assign(
        accumulated.mission_indices.begin(),
        accumulated.mission_indices.end());

      output.retained_nodes.push_back(
        std::move(node));
    }

    output.trajectory_materialized =
      materialize_cropped_static_trajectory(
      output,
      detected.trajectory);

    if (!output.trajectory_materialized) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Crop for '%s' is valid on the virtual net but cannot be represented "
        "exactly by StaticTrajectory: TAKEOFF or LANDING contains more than "
        "one disconnected retained run",
        detected.trajectory.trajectory_id.c_str());
    }

    return output;
  }

  void recompute_locked()
  {
    requested_supervision_state_.clear();
    manual_adjustment_state_.clear();

    if (!detected_snapshot_received_) {
      return;
    }

    const auto trajectories =
      ordered_input(
      latest_detected_snapshot_);

    for (const auto & detected :
      trajectories)
    {
      const auto & trajectory =
        detected.trajectory;

      const std::size_t node_count =
        collision_node_count(detected);

      if (
        node_count <
        static_cast<std::size_t>(
          collision_node_threshold_))
      {
        SupervisionState state;
        state.detected = detected;
        state.cropped = build_crop(detected);

        RCLCPP_DEBUG(
          get_logger(),
          "Trajectory '%s' -> REQUESTED_SUPERVISION | "
          "missions=%zu | unique_collision_nodes=%zu < %d | "
          "cropped_missions=%zu | materialized=%s",
          trajectory.trajectory_id.c_str(),
          trajectory.mission.size(),
          node_count,
          collision_node_threshold_,
          state.cropped.trajectory.mission.size(),
          state.cropped.trajectory_materialized ? "true" : "false");

        requested_supervision_state_[
          trajectory.trajectory_id] =
          std::move(state);
      } else {
        manual_adjustment_state_[
          trajectory.trajectory_id] =
          detected;

        RCLCPP_DEBUG(
          get_logger(),
          "Trajectory '%s' -> MANUAL_ADJUSTMENT | "
          "unique_collision_nodes=%zu >= %d",
          trajectory.trajectory_id.c_str(),
          node_count,
          collision_node_threshold_);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Deconfliction snapshot classified | input=%zu | "
      "supervision=%zu | manual=%zu | threshold=unique net nodes",
      trajectories.size(),
      requested_supervision_state_.size(),
      manual_adjustment_state_.size());
  }

  void detected_callback(
    const DetectedCollisionTrajectoryArray::
    SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    latest_detected_snapshot_ = *message;
    latest_header_ = message->header;
    detected_snapshot_received_ = true;

    recompute_locked();
    publish_snapshots_locked();
  }

  void net_loaded_callback(
    const NetLoadedTrajectoryArray::
    SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    loaded_by_id_.clear();

    for (const auto & loaded :
      message->trajectories)
    {
      const std::string & id =
        loaded.trajectory.trajectory_id;

      if (!id.empty()) {
        loaded_by_id_[id] = loaded;
      }
    }

    net_snapshot_received_ = true;

    if (!message->header.frame_id.empty()) {
      latest_header_ = message->header;
    }

    // A retained /detected snapshot may have arrived before the retained net
    // snapshot. Rebuild crops immediately when the complete net geometry
    // becomes available.
    recompute_locked();
    publish_snapshots_locked();
  }

  RequestedSupervisionTrajectoryArray
  make_supervision_snapshot_locked() const
  {
    RequestedSupervisionTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    std::vector<
      const SupervisionState *> ordered;

    ordered.reserve(
      requested_supervision_state_.size());

    for (const auto & [_, state] :
      requested_supervision_state_)
    {
      ordered.push_back(&state);
    }

    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](
        const SupervisionState * first,
        const SupervisionState * second)
      {
        const auto & a =
          first->detected.trajectory;
        const auto & b =
          second->detected.trajectory;

        if (a.priority != b.priority) {
          return a.priority < b.priority;
        }

        if (a.ua_id != b.ua_id) {
          return a.ua_id < b.ua_id;
        }

        return
          a.trajectory_id <
          b.trajectory_id;
      });

    output.trajectories.reserve(
      ordered.size());

    for (const auto * state : ordered) {
      RequestedSupervisionTrajectory item;
      item.detected_collision =
        state->detected;
      item.cropped_trajectory =
        state->cropped;

      output.trajectories.push_back(
        std::move(item));
    }

    return output;
  }

  DetectedCollisionTrajectoryArray
  make_manual_snapshot_locked() const
  {
    DetectedCollisionTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    std::vector<
      const DetectedCollisionTrajectory *>
      ordered;

    ordered.reserve(
      manual_adjustment_state_.size());

    for (const auto & [_, detected] :
      manual_adjustment_state_)
    {
      ordered.push_back(&detected);
    }

    sort_detected_pointers(ordered);

    output.trajectories.reserve(
      ordered.size());

    for (const auto * detected : ordered) {
      output.trajectories.push_back(
        *detected);
    }

    return output;
  }

  static void sort_detected_pointers(
    std::vector<
      const DetectedCollisionTrajectory *> &
      ordered)
  {
    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](
        const DetectedCollisionTrajectory * first,
        const DetectedCollisionTrajectory * second)
      {
        if (
          first->trajectory.priority !=
          second->trajectory.priority)
        {
          return
            first->trajectory.priority <
            second->trajectory.priority;
        }

        if (
          first->trajectory.ua_id !=
          second->trajectory.ua_id)
        {
          return
            first->trajectory.ua_id <
            second->trajectory.ua_id;
        }

        return
          first->trajectory.trajectory_id <
          second->trajectory.trajectory_id;
      });
  }

  static void append_segment_line_list(
    Marker & marker,
    const TrajectorySegment & segment)
  {
    const std::size_t count =
      trajectory_segment_point_count(
      segment);

    if (count < 2U) {
      return;
    }

    for (std::size_t index = 1U;
      index < count;
      ++index)
    {
      marker.points.push_back(
        trajectory_segment_point(
          segment,
          index - 1U));

      marker.points.push_back(
        trajectory_segment_point(
          segment,
          index));
    }
  }

  static bool first_trajectory_point(
    const StaticTrajectory & trajectory,
    geometry_msgs::msg::Point & point)
  {
    if (
      trajectory_segment_point_count(
        trajectory.takeoff) > 0U)
    {
      point =
        trajectory_segment_point(
        trajectory.takeoff,
        0U);

      return true;
    }

    for (const auto & mission :
      trajectory.mission)
    {
      if (
        trajectory_segment_point_count(
          mission) > 0U)
      {
        point =
          trajectory_segment_point(
          mission,
          0U);

        return true;
      }
    }

    if (
      trajectory_segment_point_count(
        trajectory.landing) > 0U)
    {
      point =
        trajectory_segment_point(
        trajectory.landing,
        0U);

      return true;
    }

    return false;
  }

  void add_supervision_markers(
    const SupervisionState & state,
    int & marker_id,
    MarkerArray & markers) const
  {
    const auto & trajectory =
      state.detected.trajectory;

    if (
      !state.cropped.retained_segments.empty())
    {
      Marker crop;
      crop.header.stamp = now();
      crop.header.frame_id =
        trajectory.frame_id;
      crop.ns =
        "deconfliction/requested_supervision/crop/" +
        trajectory.trajectory_id;
      crop.id = marker_id++;
      crop.type = Marker::LINE_LIST;
      crop.action = Marker::ADD;
      crop.pose.orientation.w = 1.0;
      crop.scale.x =
        supervision_crop_line_width_;
      crop.color.r = 0.10F;
      crop.color.g = 0.85F;
      crop.color.b = 0.95F;
      crop.color.a = 1.00F;

      for (const auto & wrapped :
        state.cropped.retained_segments)
      {
        crop.points.push_back(
          wrapped.segment.start);
        crop.points.push_back(
          wrapped.segment.end);
      }

      markers.markers.push_back(
        std::move(crop));
    }

    for (const auto & node :
      state.detected.collision_nodes)
    {
      Marker sphere;
      sphere.header.stamp = now();
      sphere.header.frame_id =
        trajectory.frame_id;
      sphere.ns =
        "deconfliction/requested_supervision/nodes/" +
        trajectory.trajectory_id;
      sphere.id = marker_id++;
      sphere.type = Marker::SPHERE;
      sphere.action = Marker::ADD;
      sphere.pose.orientation.w = 1.0;
      sphere.pose.position = node.position;
      sphere.scale.x =
        supervision_node_scale_;
      sphere.scale.y =
        supervision_node_scale_;
      sphere.scale.z =
        supervision_node_scale_;
      sphere.color.r = 1.00F;
      sphere.color.g = 0.72F;
      sphere.color.b = 0.05F;
      sphere.color.a = 1.00F;

      markers.markers.push_back(
        std::move(sphere));

      Marker text;
      text.header.stamp = now();
      text.header.frame_id =
        trajectory.frame_id;
      text.ns =
        "deconfliction/requested_supervision/nodes/" +
        trajectory.trajectory_id;
      text.id = marker_id++;
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      text.pose.orientation.w = 1.0;
      text.pose.position = node.position;
      text.pose.position.z +=
        supervision_node_scale_ * 0.85;
      text.scale.z =
        supervision_text_height_;
      text.color.r = 1.00F;
      text.color.g = 0.72F;
      text.color.b = 0.05F;
      text.color.a = 1.00F;
      text.text =
        trajectory.trajectory_id +
        " | node=" +
        std::to_string(node.node_id) +
        " | vs=" +
        join_ids(
        node.conflicting_trajectory_ids);

      markers.markers.push_back(
        std::move(text));
    }
  }

  void add_manual_markers(
    const DetectedCollisionTrajectory & detected,
    int & marker_id,
    MarkerArray & markers) const
  {
    const auto & trajectory =
      detected.trajectory;

    Marker line;
    line.header.stamp = now();
    line.header.frame_id =
      trajectory.frame_id;
    line.ns =
      "deconfliction/manual_adjustment/" +
      trajectory.trajectory_id;
    line.id = marker_id++;
    line.type = Marker::LINE_LIST;
    line.action = Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x =
      manual_line_width_;
    line.color.r = 1.00F;
    line.color.g = 0.12F;
    line.color.b = 0.08F;
    line.color.a = 1.00F;

    append_segment_line_list(
      line,
      trajectory.takeoff);

    for (const auto & mission :
      trajectory.mission)
    {
      append_segment_line_list(
        line,
        mission);
    }

    append_segment_line_list(
      line,
      trajectory.landing);

    if (!line.points.empty()) {
      markers.markers.push_back(
        std::move(line));
    }

    geometry_msgs::msg::Point label_point;

    if (
      first_trajectory_point(
        trajectory,
        label_point))
    {
      Marker text;
      text.header.stamp = now();
      text.header.frame_id =
        trajectory.frame_id;
      text.ns =
        "deconfliction/manual_adjustment/" +
        trajectory.trajectory_id;
      text.id = marker_id++;
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      text.pose.orientation.w = 1.0;
      text.pose.position =
        label_point;
      text.pose.position.z +=
        manual_text_height_ * 1.5;
      text.scale.z =
        manual_text_height_;
      text.color.r = 1.00F;
      text.color.g = 0.12F;
      text.color.b = 0.08F;
      text.color.a = 1.00F;
      text.text =
        "MANUAL | " +
        trajectory.trajectory_id +
        " | missions=" +
        std::to_string(
        trajectory.mission.size());

      markers.markers.push_back(
        std::move(text));
    }
  }

  MarkerArray make_markers_locked() const
  {
    MarkerArray markers;

    Marker delete_all;
    delete_all.header.stamp = now();
    delete_all.header.frame_id =
      latest_header_.frame_id;
    delete_all.action = Marker::DELETEALL;

    markers.markers.push_back(
      delete_all);

    int marker_id = 1;

    for (const auto & [_, state] :
      requested_supervision_state_)
    {
      add_supervision_markers(
        state,
        marker_id,
        markers);
    }

    for (const auto & [_, detected] :
      manual_adjustment_state_)
    {
      add_manual_markers(
        detected,
        marker_id,
        markers);
    }

    return markers;
  }

  void publish_snapshots_locked()
  {
    supervision_publisher_->publish(
      make_supervision_snapshot_locked());

    manual_publisher_->publish(
      make_manual_snapshot_locked());

    marker_publisher_->publish(
      make_markers_locked());
  }

  void publish_snapshots()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Authoritative heartbeat. Empty snapshots and DELETEALL markers are
    // intentional so transient-local consumers remove disappeared state.
    publish_snapshots_locked();
  }

  std::string detected_topic_;
  std::string net_loaded_topic_;
  std::string supervision_topic_;
  std::string manual_topic_;
  std::string markers_topic_;

  int collision_node_threshold_{5};
  int publish_period_ms_{1000};

  double supervision_node_scale_{0.30};
  double supervision_text_height_{0.28};
  double supervision_crop_line_width_{0.12};
  double manual_line_width_{0.14};
  double manual_text_height_{0.36};

  mutable std::mutex mutex_;
  std_msgs::msg::Header latest_header_;

  bool detected_snapshot_received_{false};
  bool net_snapshot_received_{false};

  DetectedCollisionTrajectoryArray
    latest_detected_snapshot_;

  std::map<
    std::string,
    NetLoadedTrajectory> loaded_by_id_;

  std::map<
    std::string,
    SupervisionState>
    requested_supervision_state_;

  std::map<
    std::string,
    DetectedCollisionTrajectory>
    manual_adjustment_state_;

  rclcpp::Subscription<
    DetectedCollisionTrajectoryArray>::SharedPtr
    detected_subscription_;

  rclcpp::Subscription<
    NetLoadedTrajectoryArray>::SharedPtr
    net_loaded_subscription_;

  rclcpp::Publisher<
    RequestedSupervisionTrajectoryArray>::SharedPtr
    supervision_publisher_;

  rclcpp::Publisher<
    DetectedCollisionTrajectoryArray>::SharedPtr
    manual_publisher_;

  rclcpp::Publisher<MarkerArray>::SharedPtr
    marker_publisher_;

  rclcpp::TimerBase::SharedPtr
    publish_timer_;
};

}  // namespace deconfliction_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        deconfliction_manager::
        DeconflictionManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "deconfliction_manager_node"),
      "Fatal error: %s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
