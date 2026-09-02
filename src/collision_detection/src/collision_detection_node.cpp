#include <rclcpp/rclcpp.hpp>

#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>
#include <collision_detection/msg/grid_collision_node.hpp>
#include <collision_detection/msg/grid_collision_segment.hpp>

#include <a_space_virtual_net/msg/net_loaded_trajectory.hpp>
#include <a_space_virtual_net/msg/net_loaded_trajectory_array.hpp>
#include <a_space_virtual_net/msg/net_trajectory_segment.hpp>

#include <static_trajectory_conflict_manager/msg/collision_static_trajectory.hpp>
#include <static_trajectory_conflict_manager/msg/collision_static_trajectory_array.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace collision_detection
{

using DetectedCollisionTrajectory =
  collision_detection::msg::DetectedCollisionTrajectory;
using DetectedCollisionTrajectoryArray =
  collision_detection::msg::DetectedCollisionTrajectoryArray;
using GridCollisionNode = collision_detection::msg::GridCollisionNode;
using GridCollisionSegment = collision_detection::msg::GridCollisionSegment;

using NetLoadedTrajectory = a_space_virtual_net::msg::NetLoadedTrajectory;
using NetLoadedTrajectoryArray = a_space_virtual_net::msg::NetLoadedTrajectoryArray;
using NetTrajectorySegment = a_space_virtual_net::msg::NetTrajectorySegment;

using CollisionStaticTrajectory =
  static_trajectory_conflict_manager::msg::CollisionStaticTrajectory;
using CollisionStaticTrajectoryArray =
  static_trajectory_conflict_manager::msg::CollisionStaticTrajectoryArray;

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct NodeAccumulator
{
  geometry_msgs::msg::Point position;
  std::set<uint32_t> own_net_segment_indices;
  std::set<std::string> conflicting_trajectory_ids;
};

static std_msgs::msg::ColorRGBA make_color(
  float red,
  float green,
  float blue,
  float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

static std::set<std::string> other_users(
  const std::set<std::string> & users,
  const std::string & own_trajectory_id)
{
  std::set<std::string> output = users;
  output.erase(own_trajectory_id);
  return output;
}

class CollisionDetectionNode : public rclcpp::Node
{
public:
  CollisionDetectionNode()
  : Node("collision_detection_node")
  {
    collision_topic_ =
      declare_parameter<std::string>(
        "collision_static_trajectories_topic",
        "/collision_static_trajectories");

    net_loaded_topic_ =
      declare_parameter<std::string>(
        "net_loaded_trajectories_topic",
        "/net_loaded_trajectories");

    detected_topic_ =
      declare_parameter<std::string>(
        "detected_collision_trajectories_topic",
        "/detected_collision_trajectories");

    markers_topic_ =
      declare_parameter<std::string>(
        "detected_collision_markers_topic",
        "/detected_collision_trajectories_markers");

    publish_period_ms_ =
      declare_parameter<int>(
        "publish_period_ms",
        1000);

    collision_segment_width_ =
      declare_parameter<double>(
        "collision_segment_width",
        0.18);

    collision_node_scale_ =
      declare_parameter<double>(
        "collision_node_scale",
        0.32);

    label_height_ =
      declare_parameter<double>(
        "label_height",
        0.36);

    publish_labels_ =
      declare_parameter<bool>(
        "publish_labels",
        true);

    validate_parameters();

    auto snapshot_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    snapshot_qos.reliable();
    snapshot_qos.transient_local();

    collision_subscription_ =
      create_subscription<CollisionStaticTrajectoryArray>(
        collision_topic_,
        snapshot_qos,
        std::bind(
          &CollisionDetectionNode::collision_callback,
          this,
          std::placeholders::_1));

    net_loaded_subscription_ =
      create_subscription<NetLoadedTrajectoryArray>(
        net_loaded_topic_,
        snapshot_qos,
        std::bind(
          &CollisionDetectionNode::net_loaded_callback,
          this,
          std::placeholders::_1));

    detected_publisher_ =
      create_publisher<DetectedCollisionTrajectoryArray>(
        detected_topic_,
        snapshot_qos);

    marker_publisher_ =
      create_publisher<MarkerArray>(
        markers_topic_,
        snapshot_qos);

    publish_timer_ =
      create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(
          &CollisionDetectionNode::publish_snapshot,
          this));

    RCLCPP_INFO(
      get_logger(),
      "Collision detection ready | collision='%s' | net='%s' | output='%s' | markers='%s' | period=%d ms",
      collision_topic_.c_str(),
      net_loaded_topic_.c_str(),
      detected_topic_.c_str(),
      markers_topic_.c_str(),
      publish_period_ms_);
  }

private:
  void validate_parameters() const
  {
    const auto absolute_topic =
      [](const std::string & value)
      {
        return !value.empty() && value.front() == '/';
      };

    if (
      !absolute_topic(collision_topic_) ||
      !absolute_topic(net_loaded_topic_) ||
      !absolute_topic(detected_topic_) ||
      !absolute_topic(markers_topic_))
    {
      throw std::runtime_error(
              "All configured topic names must be absolute");
    }

    if (publish_period_ms_ <= 0) {
      throw std::runtime_error(
              "publish_period_ms must be > 0");
    }

    if (
      collision_segment_width_ <= 0.0 ||
      collision_node_scale_ <= 0.0 ||
      label_height_ <= 0.0)
    {
      throw std::runtime_error(
              "Marker sizes must be > 0");
    }
  }

  void collision_callback(
    const CollisionStaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    collision_header_ = message->header;
    collision_targets_.clear();

    for (const auto & collision : message->trajectories) {
      const std::string & trajectory_id =
        collision.trajectory.trajectory_id;

      if (trajectory_id.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring collision trajectory with empty trajectory_id");
        continue;
      }

      collision_targets_[trajectory_id] = collision;
    }

    collision_snapshot_received_ = true;
    recompute_locked();
  }

  void net_loaded_callback(
    const NetLoadedTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    net_header_ = message->header;
    loaded_by_id_.clear();

    for (const auto & loaded : message->trajectories) {
      const std::string & trajectory_id =
        loaded.trajectory.trajectory_id;

      if (trajectory_id.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring net-loaded trajectory with empty trajectory_id");
        continue;
      }

      loaded_by_id_[trajectory_id] = loaded;
    }

    net_snapshot_received_ = true;
    rebuild_usage_indices_locked();
    recompute_locked();
  }

  void rebuild_usage_indices_locked()
  {
    edge_users_.clear();
    node_users_.clear();

    for (const auto & [trajectory_id, loaded] : loaded_by_id_) {
      for (const auto & segment : loaded.segments) {
        edge_users_[segment.edge_id].insert(trajectory_id);
        node_users_[segment.start_node_id].insert(trajectory_id);
        node_users_[segment.end_node_id].insert(trajectory_id);
      }
    }
  }

  void accumulate_node(
    std::map<uint64_t, NodeAccumulator> & nodes,
    uint64_t node_id,
    const geometry_msgs::msg::Point & position,
    uint32_t own_net_segment_index,
    const std::set<std::string> & conflicts) const
  {
    if (conflicts.empty()) {
      return;
    }

    auto & node = nodes[node_id];
    node.position = position;
    node.own_net_segment_indices.insert(own_net_segment_index);
    node.conflicting_trajectory_ids.insert(
      conflicts.begin(),
      conflicts.end());
  }

  DetectedCollisionTrajectory detect_for_target_locked(
    const CollisionStaticTrajectory & source)
  {
    DetectedCollisionTrajectory output;
    output.trajectory = source.trajectory;

    const std::string & trajectory_id =
      source.trajectory.trajectory_id;

    const auto loaded_iterator =
      loaded_by_id_.find(trajectory_id);

    if (loaded_iterator == loaded_by_id_.end()) {
      output.loaded_on_net = false;
      output.has_shared_net_space = false;

      if (net_snapshot_received_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Collision trajectory '%s' is not present in /net_loaded_trajectories",
          trajectory_id.c_str());
      }

      return output;
    }

    output.loaded_on_net = true;

    const NetLoadedTrajectory & loaded =
      loaded_iterator->second;

    std::map<uint64_t, NodeAccumulator> collision_nodes;
    std::set<std::string> all_conflicting_ids;

    for (const auto & segment : loaded.segments) {
      const auto edge_iterator =
        edge_users_.find(segment.edge_id);

      std::set<std::string> edge_conflicts;

      if (edge_iterator != edge_users_.end()) {
        edge_conflicts =
          other_users(
          edge_iterator->second,
          trajectory_id);
      }

      if (!edge_conflicts.empty()) {
        GridCollisionSegment collision_segment;
        collision_segment.edge_id = segment.edge_id;
        collision_segment.axis = segment.axis;
        collision_segment.start_node_id = segment.start_node_id;
        collision_segment.end_node_id = segment.end_node_id;
        collision_segment.start = segment.start;
        collision_segment.end = segment.end;
        collision_segment.length_m = segment.length_m;
        collision_segment.phase = segment.phase;
        collision_segment.original_phase_segment_index =
          segment.original_phase_segment_index;
        collision_segment.net_segment_index = segment.net_segment_index;
        collision_segment.conflicting_trajectory_ids.assign(
          edge_conflicts.begin(),
          edge_conflicts.end());

        output.collision_segments.push_back(
          std::move(collision_segment));

        all_conflicting_ids.insert(
          edge_conflicts.begin(),
          edge_conflicts.end());
      }

      const auto start_users_iterator =
        node_users_.find(segment.start_node_id);

      if (start_users_iterator != node_users_.end()) {
        const auto start_conflicts =
          other_users(
          start_users_iterator->second,
          trajectory_id);

        accumulate_node(
          collision_nodes,
          segment.start_node_id,
          segment.start,
          segment.net_segment_index,
          start_conflicts);

        all_conflicting_ids.insert(
          start_conflicts.begin(),
          start_conflicts.end());
      }

      const auto end_users_iterator =
        node_users_.find(segment.end_node_id);

      if (end_users_iterator != node_users_.end()) {
        const auto end_conflicts =
          other_users(
          end_users_iterator->second,
          trajectory_id);

        accumulate_node(
          collision_nodes,
          segment.end_node_id,
          segment.end,
          segment.net_segment_index,
          end_conflicts);

        all_conflicting_ids.insert(
          end_conflicts.begin(),
          end_conflicts.end());
      }
    }

    output.collision_nodes.reserve(collision_nodes.size());

    for (const auto & [node_id, accumulator] : collision_nodes) {
      GridCollisionNode node;
      node.node_id = node_id;
      node.position = accumulator.position;
      node.own_net_segment_indices.assign(
        accumulator.own_net_segment_indices.begin(),
        accumulator.own_net_segment_indices.end());
      node.conflicting_trajectory_ids.assign(
        accumulator.conflicting_trajectory_ids.begin(),
        accumulator.conflicting_trajectory_ids.end());

      output.collision_nodes.push_back(std::move(node));
    }

    output.conflicting_trajectory_ids.assign(
      all_conflicting_ids.begin(),
      all_conflicting_ids.end());

    output.has_shared_net_space =
      !output.collision_nodes.empty() ||
      !output.collision_segments.empty();

    return output;
  }

  void recompute_locked()
  {
    detected_state_.clear();

    if (!collision_snapshot_received_) {
      return;
    }

    std::vector<const CollisionStaticTrajectory *> ordered;
    ordered.reserve(collision_targets_.size());

    for (const auto & [_, collision] : collision_targets_) {
      ordered.push_back(&collision);
    }

    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](const auto * first, const auto * second)
      {
        if (first->trajectory.priority != second->trajectory.priority) {
          return first->trajectory.priority < second->trajectory.priority;
        }

        return
          first->trajectory.trajectory_id <
          second->trajectory.trajectory_id;
      });

    detected_state_.reserve(ordered.size());

    for (const auto * source : ordered) {
      detected_state_.push_back(
        detect_for_target_locked(*source));
    }
  }

  DetectedCollisionTrajectoryArray make_output_locked() const
  {
    DetectedCollisionTrajectoryArray output;

    if (net_snapshot_received_) {
      output.header = net_header_;
    } else {
      output.header = collision_header_;
    }

    output.header.stamp = now();
    output.trajectories = detected_state_;
    return output;
  }

  MarkerArray make_markers_locked() const
  {
    MarkerArray markers;

    Marker delete_all;
    delete_all.header.frame_id =
      net_snapshot_received_ ?
      net_header_.frame_id :
      collision_header_.frame_id;
    delete_all.header.stamp = now();
    delete_all.action = Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    int marker_id = 0;

    for (const auto & detected : detected_state_) {
      const std::string & trajectory_id =
        detected.trajectory.trajectory_id;

      if (!detected.loaded_on_net) {
        continue;
      }

      Marker segment_marker;
      segment_marker.header = delete_all.header;
      segment_marker.ns =
        "collision_detection/segments/" + trajectory_id;
      segment_marker.id = marker_id++;
      segment_marker.type = Marker::LINE_LIST;
      segment_marker.action = Marker::ADD;
      segment_marker.pose.orientation.w = 1.0;
      segment_marker.scale.x = collision_segment_width_;
      segment_marker.color = make_color(1.0F, 0.08F, 0.08F, 1.0F);

      for (const auto & segment : detected.collision_segments) {
        segment_marker.points.push_back(segment.start);
        segment_marker.points.push_back(segment.end);
      }

      if (!segment_marker.points.empty()) {
        markers.markers.push_back(std::move(segment_marker));
      }

      Marker node_marker;
      node_marker.header = delete_all.header;
      node_marker.ns =
        "collision_detection/nodes/" + trajectory_id;
      node_marker.id = marker_id++;
      node_marker.type = Marker::SPHERE_LIST;
      node_marker.action = Marker::ADD;
      node_marker.pose.orientation.w = 1.0;
      node_marker.scale.x = collision_node_scale_;
      node_marker.scale.y = collision_node_scale_;
      node_marker.scale.z = collision_node_scale_;
      node_marker.color = make_color(1.0F, 0.72F, 0.05F, 1.0F);

      for (const auto & node : detected.collision_nodes) {
        node_marker.points.push_back(node.position);
      }

      if (!node_marker.points.empty()) {
        markers.markers.push_back(std::move(node_marker));
      }

      if (
        publish_labels_ &&
        !detected.collision_nodes.empty())
      {
        Marker label;
        label.header = delete_all.header;
        label.ns =
          "collision_detection/labels/" + trajectory_id;
        label.id = marker_id++;
        label.type = Marker::TEXT_VIEW_FACING;
        label.action = Marker::ADD;
        label.pose.orientation.w = 1.0;
        label.pose.position =
          detected.collision_nodes.front().position;
        label.pose.position.z += label_height_ * 1.5;
        label.scale.z = label_height_;
        label.color = make_color(1.0F, 1.0F, 1.0F, 1.0F);

        std::ostringstream text;
        text << trajectory_id << " | edges="
             << detected.collision_segments.size()
             << " nodes="
             << detected.collision_nodes.size();

        if (!detected.conflicting_trajectory_ids.empty()) {
          text << " | vs ";

          for (std::size_t index = 0U;
            index < detected.conflicting_trajectory_ids.size();
            ++index)
          {
            if (index > 0U) {
              text << ",";
            }

            text << detected.conflicting_trajectory_ids[index];
          }
        }

        label.text = text.str();
        markers.markers.push_back(std::move(label));
      }
    }

    return markers;
  }

  void publish_snapshot()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!collision_snapshot_received_) {
      return;
    }

    detected_publisher_->publish(
      make_output_locked());

    marker_publisher_->publish(
      make_markers_locked());
  }

  std::string collision_topic_;
  std::string net_loaded_topic_;
  std::string detected_topic_;
  std::string markers_topic_;

  int publish_period_ms_{1000};
  double collision_segment_width_{0.18};
  double collision_node_scale_{0.32};
  double label_height_{0.36};
  bool publish_labels_{true};

  mutable std::mutex mutex_;

  bool collision_snapshot_received_{false};
  bool net_snapshot_received_{false};

  std_msgs::msg::Header collision_header_;
  std_msgs::msg::Header net_header_;

  std::map<std::string, CollisionStaticTrajectory>
    collision_targets_;

  std::map<std::string, NetLoadedTrajectory>
    loaded_by_id_;

  std::map<uint64_t, std::set<std::string>> edge_users_;
  std::map<uint64_t, std::set<std::string>> node_users_;

  std::vector<DetectedCollisionTrajectory>
    detected_state_;

  rclcpp::Subscription<CollisionStaticTrajectoryArray>::SharedPtr
    collision_subscription_;

  rclcpp::Subscription<NetLoadedTrajectoryArray>::SharedPtr
    net_loaded_subscription_;

  rclcpp::Publisher<DetectedCollisionTrajectoryArray>::SharedPtr
    detected_publisher_;

  rclcpp::Publisher<MarkerArray>::SharedPtr
    marker_publisher_;

  rclcpp::TimerBase::SharedPtr
    publish_timer_;
};

}  // namespace collision_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        collision_detection::CollisionDetectionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("collision_detection_node"),
      "Fatal error: %s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
