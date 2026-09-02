#include <rclcpp/rclcpp.hpp>

#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>
#include <collision_detection/msg/grid_collision_node.hpp>

#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace deconfliction_manager
{

using DetectedCollisionTrajectory =
  collision_detection::msg::DetectedCollisionTrajectory;
using DetectedCollisionTrajectoryArray =
  collision_detection::msg::DetectedCollisionTrajectoryArray;
using GridCollisionNode =
  collision_detection::msg::GridCollisionNode;

using StaticTrajectory =
  static_trajectory_manager::msg::StaticTrajectory;
using TrajectorySegment =
  static_trajectory_manager::msg::TrajectorySegment;

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct SupervisionState
{
  DetectedCollisionTrajectory detected;
};

static std::string join_ids(const std::vector<std::string> & ids)
{
  std::ostringstream stream;

  for (std::size_t index = 0U; index < ids.size(); ++index) {
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
    detected_topic_ = declare_parameter<std::string>(
      "detected_collision_trajectories_topic",
      "/detected_collision_trajectories");

    supervision_topic_ = declare_parameter<std::string>(
      "requested_supervision_trajectories_topic",
      "/requested_supervision_trajectories");

    manual_topic_ = declare_parameter<std::string>(
      "manual_adjustment_trajectories_topic",
      "/manual_adjustment_trajectories");

    markers_topic_ = declare_parameter<std::string>(
      "deconfliction_markers_topic",
      "/deconfliction_manager_markers");

    collision_resource_threshold_ = declare_parameter<int>(
      "collision_resource_threshold",
      5);

    publish_period_ms_ = declare_parameter<int>(
      "publish_period_ms",
      1000);

    supervision_node_scale_ = declare_parameter<double>(
      "supervision_node_scale",
      0.30);

    supervision_text_height_ = declare_parameter<double>(
      "supervision_text_height",
      0.28);

    manual_line_width_ = declare_parameter<double>(
      "manual_line_width",
      0.14);

    manual_text_height_ = declare_parameter<double>(
      "manual_text_height",
      0.36);

    validate_parameters();

    rclcpp::QoS snapshot_qos(rclcpp::KeepLast(1));
    snapshot_qos.reliable();
    snapshot_qos.transient_local();

    detected_subscription_ =
      create_subscription<DetectedCollisionTrajectoryArray>(
        detected_topic_,
        snapshot_qos,
        std::bind(
          &DeconflictionManagerNode::detected_callback,
          this,
          std::placeholders::_1));

    supervision_publisher_ =
      create_publisher<DetectedCollisionTrajectoryArray>(
        supervision_topic_,
        snapshot_qos);

    manual_publisher_ =
      create_publisher<DetectedCollisionTrajectoryArray>(
        manual_topic_,
        snapshot_qos);

    marker_publisher_ =
      create_publisher<MarkerArray>(
        markers_topic_,
        snapshot_qos);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(
        &DeconflictionManagerNode::publish_snapshots,
        this));

    publish_snapshots();

    RCLCPP_INFO(
      get_logger(),
      "Deconfliction manager ready | input='%s' | supervision='%s' | "
      "manual='%s' | markers='%s' | threshold=%d | publish_period=%d ms",
      detected_topic_.c_str(),
      supervision_topic_.c_str(),
      manual_topic_.c_str(),
      markers_topic_.c_str(),
      collision_resource_threshold_,
      publish_period_ms_);
  }

private:
  void validate_parameters() const
  {
    const auto absolute_topic = [](const std::string & topic) {
        return !topic.empty() && topic.front() == '/';
      };

    if (
      !absolute_topic(detected_topic_) ||
      !absolute_topic(supervision_topic_) ||
      !absolute_topic(manual_topic_) ||
      !absolute_topic(markers_topic_))
    {
      throw std::runtime_error("All configured topics must be absolute");
    }

    if (collision_resource_threshold_ < 0) {
      throw std::runtime_error("collision_resource_threshold must be >= 0");
    }

    if (publish_period_ms_ <= 0) {
      throw std::runtime_error("publish_period_ms must be > 0");
    }

    if (
      !std::isfinite(supervision_node_scale_) ||
      supervision_node_scale_ <= 0.0 ||
      !std::isfinite(supervision_text_height_) ||
      supervision_text_height_ <= 0.0 ||
      !std::isfinite(manual_line_width_) ||
      manual_line_width_ <= 0.0 ||
      !std::isfinite(manual_text_height_) ||
      manual_text_height_ <= 0.0)
    {
      throw std::runtime_error("Marker dimensions must be finite and > 0");
    }
  }

  std::size_t collision_resource_count(
    const DetectedCollisionTrajectory & detected) const
  {
    return
      detected.collision_nodes.size() +
      detected.collision_segments.size();
  }

  static std::vector<DetectedCollisionTrajectory> ordered_input(
    const DetectedCollisionTrajectoryArray & message)
  {
    std::map<std::string, DetectedCollisionTrajectory> by_id;

    for (const auto & detected : message.trajectories) {
      const auto & trajectory_id = detected.trajectory.trajectory_id;

      if (trajectory_id.empty()) {
        continue;
      }

      by_id[trajectory_id] = detected;
    }

    std::vector<DetectedCollisionTrajectory> ordered;
    ordered.reserve(by_id.size());

    for (const auto & [_, detected] : by_id) {
      ordered.push_back(detected);
    }

    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](const DetectedCollisionTrajectory & first,
         const DetectedCollisionTrajectory & second)
      {
        if (first.trajectory.priority != second.trajectory.priority) {
          return first.trajectory.priority < second.trajectory.priority;
        }

        if (first.trajectory.ua_id != second.trajectory.ua_id) {
          return first.trajectory.ua_id < second.trajectory.ua_id;
        }

        return
          first.trajectory.trajectory_id <
          second.trajectory.trajectory_id;
      });

    return ordered;
  }

  void process_snapshot_locked(
    const DetectedCollisionTrajectoryArray & message)
  {
    requested_supervision_state_.clear();
    manual_adjustment_state_.clear();

    latest_header_ = message.header;

    const auto trajectories = ordered_input(message);

    for (const auto & detected : trajectories) {
      const auto & trajectory = detected.trajectory;
      const std::size_t resources = collision_resource_count(detected);

      if (
        resources <
        static_cast<std::size_t>(collision_resource_threshold_))
      {
        SupervisionState state;
        state.detected = detected;
        requested_supervision_state_[trajectory.trajectory_id] =
          std::move(state);

        RCLCPP_DEBUG(
          get_logger(),
          "Trajectory '%s' -> REQUESTED_SUPERVISION | resources=%zu < %d",
          trajectory.trajectory_id.c_str(),
          resources,
          collision_resource_threshold_);
      } else {
        manual_adjustment_state_[trajectory.trajectory_id] = detected;

        RCLCPP_DEBUG(
          get_logger(),
          "Trajectory '%s' -> MANUAL_ADJUSTMENT | resources=%zu >= %d",
          trajectory.trajectory_id.c_str(),
          resources,
          collision_resource_threshold_);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Deconfliction snapshot classified | input=%zu | supervision=%zu | manual=%zu",
      trajectories.size(),
      requested_supervision_state_.size(),
      manual_adjustment_state_.size());
  }

  void detected_callback(
    const DetectedCollisionTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    process_snapshot_locked(*message);
    publish_snapshots_locked();
  }

  DetectedCollisionTrajectoryArray make_supervision_snapshot_locked() const
  {
    DetectedCollisionTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    std::vector<const DetectedCollisionTrajectory *> ordered;
    ordered.reserve(requested_supervision_state_.size());

    for (const auto & [_, state] : requested_supervision_state_) {
      ordered.push_back(&state.detected);
    }

    sort_detected_pointers(ordered);

    output.trajectories.reserve(ordered.size());
    for (const auto * detected : ordered) {
      output.trajectories.push_back(*detected);
    }

    return output;
  }

  DetectedCollisionTrajectoryArray make_manual_snapshot_locked() const
  {
    DetectedCollisionTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    std::vector<const DetectedCollisionTrajectory *> ordered;
    ordered.reserve(manual_adjustment_state_.size());

    for (const auto & [_, detected] : manual_adjustment_state_) {
      ordered.push_back(&detected);
    }

    sort_detected_pointers(ordered);

    output.trajectories.reserve(ordered.size());
    for (const auto * detected : ordered) {
      output.trajectories.push_back(*detected);
    }

    return output;
  }

  static void sort_detected_pointers(
    std::vector<const DetectedCollisionTrajectory *> & ordered)
  {
    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](const DetectedCollisionTrajectory * first,
         const DetectedCollisionTrajectory * second)
      {
        if (first->trajectory.priority != second->trajectory.priority) {
          return first->trajectory.priority < second->trajectory.priority;
        }

        if (first->trajectory.ua_id != second->trajectory.ua_id) {
          return first->trajectory.ua_id < second->trajectory.ua_id;
        }

        return
          first->trajectory.trajectory_id <
          second->trajectory.trajectory_id;
      });
  }

  static std::vector<geometry_msgs::msg::Point> expanded_points(
    const StaticTrajectory & trajectory)
  {
    std::vector<geometry_msgs::msg::Point> output;

    const auto append = [&output](const TrajectorySegment & segment) {
        const std::size_t count = std::min(
          segment.x.size(),
          std::min(segment.y.size(), segment.z.size()));

        for (std::size_t index = 0U; index < count; ++index) {
          geometry_msgs::msg::Point point;
          point.x = segment.x[index];
          point.y = segment.y[index];
          point.z = segment.z[index];
          output.push_back(point);
        }
      };

    append(trajectory.takeoff);

    const uint32_t repetitions =
      std::max<uint32_t>(1U, trajectory.repetitions);

    for (uint32_t repetition = 0U; repetition < repetitions; ++repetition) {
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
    const auto & trajectory = state.detected.trajectory;

    for (const auto & node : state.detected.collision_nodes) {
      Marker sphere;
      sphere.header.stamp = now();
      sphere.header.frame_id = trajectory.frame_id;
      sphere.ns =
        "deconfliction/requested_supervision/nodes/" +
        trajectory.trajectory_id;
      sphere.id = marker_id++;
      sphere.type = Marker::SPHERE;
      sphere.action = Marker::ADD;
      sphere.pose.orientation.w = 1.0;
      sphere.pose.position = node.position;
      sphere.scale.x = supervision_node_scale_;
      sphere.scale.y = supervision_node_scale_;
      sphere.scale.z = supervision_node_scale_;
      sphere.color.r = 1.00F;
      sphere.color.g = 0.72F;
      sphere.color.b = 0.05F;
      sphere.color.a = 1.00F;
      markers.markers.push_back(std::move(sphere));

      Marker text;
      text.header.stamp = now();
      text.header.frame_id = trajectory.frame_id;
      text.ns =
        "deconfliction/requested_supervision/nodes/" +
        trajectory.trajectory_id;
      text.id = marker_id++;
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      text.pose.orientation.w = 1.0;
      text.pose.position = node.position;
      text.pose.position.z += supervision_node_scale_ * 0.85;
      text.scale.z = supervision_text_height_;
      text.color.r = 1.00F;
      text.color.g = 0.72F;
      text.color.b = 0.05F;
      text.color.a = 1.00F;
      text.text =
        trajectory.trajectory_id +
        " | node=" + std::to_string(node.node_id) +
        " | vs=" + join_ids(node.conflicting_trajectory_ids);
      markers.markers.push_back(std::move(text));
    }
  }

  void add_manual_markers(
    const DetectedCollisionTrajectory & detected,
    int & marker_id,
    MarkerArray & markers) const
  {
    const auto & trajectory = detected.trajectory;
    const auto points = expanded_points(trajectory);

    if (points.size() >= 2U) {
      Marker line;
      line.header.stamp = now();
      line.header.frame_id = trajectory.frame_id;
      line.ns =
        "deconfliction/manual_adjustment/" +
        trajectory.trajectory_id;
      line.id = marker_id++;
      line.type = Marker::LINE_STRIP;
      line.action = Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = manual_line_width_;
      line.color.r = 1.00F;
      line.color.g = 0.12F;
      line.color.b = 0.08F;
      line.color.a = 1.00F;
      line.points = points;
      markers.markers.push_back(std::move(line));
    }

    if (!points.empty()) {
      Marker text;
      text.header.stamp = now();
      text.header.frame_id = trajectory.frame_id;
      text.ns =
        "deconfliction/manual_adjustment/" +
        trajectory.trajectory_id;
      text.id = marker_id++;
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      text.pose.orientation.w = 1.0;
      text.pose.position = points.front();
      text.pose.position.z += manual_text_height_ * 1.5;
      text.scale.z = manual_text_height_;
      text.color.r = 1.00F;
      text.color.g = 0.12F;
      text.color.b = 0.08F;
      text.color.a = 1.00F;
      text.text =
        "MANUAL | " + trajectory.trajectory_id;
      markers.markers.push_back(std::move(text));
    }
  }

  MarkerArray make_markers_locked() const
  {
    MarkerArray markers;

    Marker delete_all;
    delete_all.header.stamp = now();
    delete_all.header.frame_id = latest_header_.frame_id;
    delete_all.action = Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    int marker_id = 1;

    for (const auto & [_, state] : requested_supervision_state_) {
      add_supervision_markers(state, marker_id, markers);
    }

    for (const auto & [_, detected] : manual_adjustment_state_) {
      add_manual_markers(detected, marker_id, markers);
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

    // Authoritative periodic heartbeat. Empty snapshots and DELETEALL markers
    // are intentional so consumers/RViz can remove disappeared trajectories.
    publish_snapshots_locked();
  }

  std::string detected_topic_;
  std::string supervision_topic_;
  std::string manual_topic_;
  std::string markers_topic_;

  int collision_resource_threshold_{5};
  int publish_period_ms_{1000};

  double supervision_node_scale_{0.30};
  double supervision_text_height_{0.28};
  double manual_line_width_{0.14};
  double manual_text_height_{0.36};

  mutable std::mutex mutex_;
  std_msgs::msg::Header latest_header_;

  std::map<std::string, SupervisionState>
    requested_supervision_state_;

  std::map<std::string, DetectedCollisionTrajectory>
    manual_adjustment_state_;

  rclcpp::Subscription<DetectedCollisionTrajectoryArray>::SharedPtr
    detected_subscription_;

  rclcpp::Publisher<DetectedCollisionTrajectoryArray>::SharedPtr
    supervision_publisher_;

  rclcpp::Publisher<DetectedCollisionTrajectoryArray>::SharedPtr
    manual_publisher_;

  rclcpp::Publisher<MarkerArray>::SharedPtr
    marker_publisher_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace deconfliction_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        deconfliction_manager::DeconflictionManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("deconfliction_manager_node"),
      "Fatal error: %s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
