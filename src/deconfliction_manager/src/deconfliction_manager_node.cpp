#include <rclcpp/rclcpp.hpp>

#include <a_space_virtual_net/msg/net_loaded_trajectory.hpp>
#include <a_space_virtual_net/msg/net_loaded_trajectory_array.hpp>
#include <a_space_virtual_net/msg/net_trajectory_segment.hpp>

#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>
#include <collision_detection/msg/grid_collision_node.hpp>

#include <deconfliction_manager/msg/cropped_net_node.hpp>
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

struct GeometryEdgeKey
{
  uint8_t phase{0U};
  uint64_t edge_id{0U};

  bool operator<(const GeometryEdgeKey & other) const
  {
    return std::tie(phase, edge_id) <
           std::tie(other.phase, other.edge_id);
  }
};

struct RetainedNodeAccumulator
{
  geometry_msgs::msg::Point position;
  std::set<uint8_t> phases;
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

  static void accumulate_retained_node(
    std::map<
      uint64_t,
      RetainedNodeAccumulator> & nodes,
    uint64_t node_id,
    const geometry_msgs::msg::Point & position,
    uint8_t phase,
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

    // Geometry is represented only once. Repeated partial mission operations
    // can generate the same virtual-net edge several times; (phase, edge_id)
    // is retained only on its first occurrence.
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

      accumulate_retained_node(
        retained_nodes,
        segment->start_node_id,
        segment->start,
        segment->phase,
        forbidden_nodes);

      accumulate_retained_node(
        retained_nodes,
        segment->end_node_id,
        segment->end,
        segment->phase,
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

      if (remove) {
        output.removed_segments.push_back(
          *segment);

        removed_edge_ids.insert(
          segment->edge_id);
      } else {
        output.retained_segments.push_back(
          *segment);
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

      output.retained_nodes.push_back(
        std::move(node));
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

        requested_supervision_state_[
          trajectory.trajectory_id] =
          std::move(state);

        RCLCPP_DEBUG(
          get_logger(),
          "Trajectory '%s' -> REQUESTED_SUPERVISION | "
          "unique_collision_nodes=%zu < %d",
          trajectory.trajectory_id.c_str(),
          node_count,
          collision_node_threshold_);
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

  static std::vector<
    geometry_msgs::msg::Point>
  expanded_points(
    const StaticTrajectory & trajectory)
  {
    std::vector<
      geometry_msgs::msg::Point> output;

    const auto append =
      [&output](
      const TrajectorySegment & segment)
      {
        const std::size_t count =
          std::min(
          segment.x.size(),
          std::min(
            segment.y.size(),
            segment.z.size()));

        for (std::size_t index = 0U;
          index < count;
          ++index)
        {
          geometry_msgs::msg::Point point;
          point.x = segment.x[index];
          point.y = segment.y[index];
          point.z = segment.z[index];
          output.push_back(point);
        }
      };

    append(trajectory.takeoff);

    const uint32_t repetitions =
      std::max<uint32_t>(
      1U,
      trajectory.repetitions);

    for (uint32_t repetition = 0U;
      repetition < repetitions;
      ++repetition)
    {
      append(trajectory.mission);
    }

    append(trajectory.landing);

    return output;
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

      for (const auto & segment :
        state.cropped.retained_segments)
      {
        crop.points.push_back(
          segment.start);
        crop.points.push_back(
          segment.end);
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

    const auto points =
      expanded_points(trajectory);

    if (points.size() >= 2U) {
      Marker line;
      line.header.stamp = now();
      line.header.frame_id =
        trajectory.frame_id;
      line.ns =
        "deconfliction/manual_adjustment/" +
        trajectory.trajectory_id;
      line.id = marker_id++;
      line.type = Marker::LINE_STRIP;
      line.action = Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x =
        manual_line_width_;
      line.color.r = 1.00F;
      line.color.g = 0.12F;
      line.color.b = 0.08F;
      line.color.a = 1.00F;
      line.points = points;

      markers.markers.push_back(
        std::move(line));
    }

    if (!points.empty()) {
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
      text.pose.position = points.front();
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
        trajectory.trajectory_id;

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
