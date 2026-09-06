#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>

#include <deconfliction_manager/msg/cropped_net_trajectory.hpp>
#include <deconfliction_manager/msg/requested_supervision_trajectory.hpp>
#include <deconfliction_manager/msg/requested_supervision_trajectory_array.hpp>

#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>

#include <std_msgs/msg/color_rgba.hpp>
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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace supervision_trajectory_manager
{

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

enum class StoredState : uint8_t
{
  PENDING = 0U,
  SUPERVISED = 1U
};

struct StoredEntry
{
  // ORIGINAL complete collision payload. This remains the source published on
  // /supervised_trajectories once the cropped copy has been accepted upstream.
  DetectedCollisionTrajectory request;

  // Exact crop produced by deconfliction_manager_node.
  CroppedNetTrajectory crop;
  bool crop_complete{false};
  bool adjusted_ready{false};

  // Direct copy of crop.trajectory. This node never rebuilds/crops/orients it.
  StaticTrajectory adjusted;

  StoredState state{StoredState::PENDING};

  // Periodic supervised trajectories publish the ORIGINAL geometry with the
  // current occurrence timestamps learned from the AVAILABLE cropped copy.
  builtin_interfaces::msg::Time synchronized_start_utc;
  builtin_interfaces::msg::Time synchronized_end_utc;
  bool synchronized_time_received{false};
};

static std_msgs::msg::ColorRGBA color(
  float red,
  float green,
  float blue,
  float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA output;
  output.r = red;
  output.g = green;
  output.b = blue;
  output.a = alpha;
  return output;
}

class SupervisionTrajectoryManagerNode : public rclcpp::Node
{
public:
  SupervisionTrajectoryManagerNode()
  : Node("supervision_trajectory_manager_node")
  {
    requested_topic_ = declare_parameter<std::string>(
      "requested_supervision_trajectories_topic",
      "/requested_supervision_trajectories");

    available_topic_ = declare_parameter<std::string>(
      "available_trajectories_topic",
      "/available_static_trajectories");

    adjusted_topic_ = declare_parameter<std::string>(
      "adjusted_trajectories_topic",
      "/adjusted_trajectories");

    supervised_topic_ = declare_parameter<std::string>(
      "supervised_trajectories_topic",
      "/supervised_trajectories");

    markers_topic_ = declare_parameter<std::string>(
      "supervision_markers_topic",
      "/supervision_trajectory_manager_markers");

    publish_period_ms_ = declare_parameter<int>(
      "publish_period_ms",
      1000);

    geometry_match_tolerance_m_ = declare_parameter<double>(
      "geometry_match_tolerance_m",
      1.0e-6);

    pending_line_width_ = declare_parameter<double>(
      "pending_line_width",
      0.12);

    supervised_node_scale_ = declare_parameter<double>(
      "supervised_node_scale",
      0.32);

    supervised_text_height_ = declare_parameter<double>(
      "supervised_text_height",
      0.28);

    validate_parameters();

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    requested_sub_ =
      create_subscription<RequestedSupervisionTrajectoryArray>(
      requested_topic_,
      qos,
      std::bind(
        &SupervisionTrajectoryManagerNode::requested_callback,
        this,
        std::placeholders::_1));

    available_sub_ =
      create_subscription<StaticTrajectoryArray>(
      available_topic_,
      qos,
      std::bind(
        &SupervisionTrajectoryManagerNode::available_callback,
        this,
        std::placeholders::_1));

    adjusted_pub_ =
      create_publisher<StaticTrajectoryArray>(
      adjusted_topic_,
      qos);

    supervised_pub_ =
      create_publisher<DetectedCollisionTrajectoryArray>(
      supervised_topic_,
      qos);

    markers_pub_ =
      create_publisher<MarkerArray>(
      markers_topic_,
      qos);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(
        &SupervisionTrajectoryManagerNode::publish,
        this));

    publish();

    RCLCPP_INFO(
      get_logger(),
      "Supervision trajectory manager | request='%s' | available='%s' | "
      "adjusted='%s' | supervised='%s' | crop source=deconfliction_manager | %.3f Hz",
      requested_topic_.c_str(),
      available_topic_.c_str(),
      adjusted_topic_.c_str(),
      supervised_topic_.c_str(),
      1000.0 / static_cast<double>(publish_period_ms_));
  }

private:
  void validate_parameters() const
  {
    const auto absolute = [](const std::string & value) {
        return !value.empty() && value.front() == '/';
      };

    if (
      !absolute(requested_topic_) ||
      !absolute(available_topic_) ||
      !absolute(adjusted_topic_) ||
      !absolute(supervised_topic_) ||
      !absolute(markers_topic_))
    {
      throw std::runtime_error("All topics must be absolute");
    }

    if (publish_period_ms_ <= 0) {
      throw std::runtime_error("publish_period_ms must be > 0");
    }

    if (
      !std::isfinite(geometry_match_tolerance_m_) ||
      geometry_match_tolerance_m_ < 0.0)
    {
      throw std::runtime_error(
              "geometry_match_tolerance_m must be finite and >= 0");
    }

    if (
      !std::isfinite(pending_line_width_) ||
      pending_line_width_ <= 0.0 ||
      !std::isfinite(supervised_node_scale_) ||
      supervised_node_scale_ <= 0.0 ||
      !std::isfinite(supervised_text_height_) ||
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
      first.x.size() != second.x.size() ||
      first.y.size() != second.y.size() ||
      first.z.size() != second.z.size())
    {
      return false;
    }

    for (std::size_t index = 0U;
      index < first.x.size();
      ++index)
    {
      if (
        std::abs(first.x[index] - second.x[index]) > tolerance ||
        std::abs(first.y[index] - second.y[index]) > tolerance ||
        std::abs(first.z[index] - second.z[index]) > tolerance)
      {
        return false;
      }
    }

    return true;
  }

  static bool same_missions(
    const std::vector<TrajectorySegment> & first,
    const std::vector<TrajectorySegment> & second,
    double tolerance)
  {
    if (first.size() != second.size()) {
      return false;
    }

    for (std::size_t index = 0U;
      index < first.size();
      ++index)
    {
      if (!same_segment(first[index], second[index], tolerance)) {
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
      expected.trajectory_id == observed.trajectory_id &&
      expected.frame_id == observed.frame_id &&
      expected.repetitions == observed.repetitions &&
      same_segment(
        expected.takeoff,
        observed.takeoff,
        geometry_match_tolerance_m_) &&
      same_missions(
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
      std::isfinite(trajectory.operation_frequency) &&
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
        "execution window; keeping previous supervised times",
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

    // If this occurrence must be re-injected, keep the latest periodic window.
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

  bool validate_materialized_crop(
    const DetectedCollisionTrajectory & request,
    const CroppedNetTrajectory & crop) const
  {
    const auto & original = request.trajectory;
    const auto & adjusted = crop.trajectory;

    if (adjusted.trajectory_id != original.trajectory_id) {
      RCLCPP_ERROR(
        get_logger(),
        "Ignoring supervision request '%s': materialized crop trajectory_id='%s' differs",
        original.trajectory_id.c_str(),
        adjusted.trajectory_id.c_str());
      return false;
    }

    if (
      !crop.frame_id.empty() &&
      !adjusted.frame_id.empty() &&
      crop.frame_id != adjusted.frame_id)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Ignoring supervision request '%s': crop frame='%s' but materialized trajectory frame='%s'",
        original.trajectory_id.c_str(),
        crop.frame_id.c_str(),
        adjusted.frame_id.c_str());
      return false;
    }

    return true;
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

    auto existing = stored_.find(id);

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
        stored_[id] = std::move(entry);
      } else {
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
        "waiting for complete virtual-net crop",
        id.c_str());
      return;
    }

    const bool adjusted_ready =
      crop.trajectory_materialized;

    if (
      adjusted_ready &&
      !validate_materialized_crop(request, crop))
    {
      return;
    }

    StaticTrajectory new_adjusted;
    if (adjusted_ready) {
      // CRITICAL: direct upstream copy. No local recrop, no run selection, no
      // edge orientation, no mission reconstruction.
      new_adjusted = crop.trajectory;
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Supervision crop '%s' is complete but trajectory_materialized=false. "
        "TAKEOFF or LANDING cannot be represented exactly; nothing is published "
        "to /adjusted_trajectories for this entry.",
        id.c_str());
    }

    if (existing == stored_.end()) {
      StoredEntry entry;
      entry.request = request;
      entry.crop = crop;
      entry.crop_complete = true;
      entry.adjusted_ready = adjusted_ready;
      if (adjusted_ready) {
        entry.adjusted = std::move(new_adjusted);
      }
      entry.state = StoredState::PENDING;
      entry.synchronized_start_utc =
        request.trajectory.operation_start_utc;
      entry.synchronized_end_utc =
        request.trajectory.operation_end_utc;
      entry.synchronized_time_received = false;

      stored_[id] = std::move(entry);

      RCLCPP_INFO(
        get_logger(),
        "Stored supervision request '%s' | original_missions=%zu | "
        "cropped_missions=%zu | collision_nodes=%zu | retained_net_segments=%zu | ready=%s",
        id.c_str(),
        request.trajectory.mission.size(),
        adjusted_ready ? crop.trajectory.mission.size() : 0U,
        request.collision_nodes.size(),
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
    existing->second.adjusted_ready = adjusted_ready;

    if (geometry_changed) {
      if (adjusted_ready) {
        existing->second.adjusted =
          std::move(new_adjusted);
      } else {
        existing->second.adjusted =
          StaticTrajectory{};
      }

      existing->second.state =
        StoredState::PENDING;
      existing->second.synchronized_start_utc =
        request.trajectory.operation_start_utc;
      existing->second.synchronized_end_utc =
        request.trajectory.operation_end_utc;
      existing->second.synchronized_time_received = false;

      RCLCPP_INFO(
        get_logger(),
        "Upstream materialized crop changed for '%s'; returned to PENDING | missions=%zu",
        id.c_str(),
        adjusted_ready ? existing->second.adjusted.mission.size() : 0U);
      return;
    }

    // A retained deconfliction snapshot can still carry the original periodic
    // window. Do not regress a newer occurrence learned from AVAILABLE.
    if (
      adjusted_ready &&
      synchronized_received &&
      periodic(existing->second.request.trajectory))
    {
      existing->second.synchronized_start_utc =
        synchronized_start;
      existing->second.synchronized_end_utc =
        synchronized_end;
      existing->second.synchronized_time_received = true;
      existing->second.adjusted.operation_start_utc =
        synchronized_start;
      existing->second.adjusted.operation_end_utc =
        synchronized_end;
    }
  }

  void requested_callback(
    const RequestedSupervisionTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    latest_header_ = message->header;

    // Intentionally persistent. Once the cropped trajectory reaches AVAILABLE,
    // it may disappear from the upstream collision snapshot. The supervision
    // lifecycle must survive that disappearance.
    for (const auto & request : message->trajectories) {
      store_request(request);
    }

    verify_available();
    publish_locked();
  }

  void available_callback(
    const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    available_ = *message;
    available_received_ = true;

    if (!message->header.frame_id.empty()) {
      latest_header_ = message->header;
    }

    verify_available();
    publish_locked();
  }

  void verify_available()
  {
    if (!available_received_) {
      return;
    }

    std::map<std::string, const StaticTrajectory *> by_id;

    for (const auto & trajectory : available_.trajectories) {
      if (!trajectory.trajectory_id.empty()) {
        by_id[trajectory.trajectory_id] = &trajectory;
      }
    }

    for (auto & [id, entry] : stored_) {
      if (
        !entry.crop_complete ||
        !entry.adjusted_ready)
      {
        continue;
      }

      const auto observed = by_id.find(id);

      const bool available_match =
        observed != by_id.end() &&
        observed->second != nullptr &&
        same_geometry(
          entry.adjusted,
          *observed->second);

      if (available_match) {
        synchronize_periodic_times_from_available(
          entry,
          *observed->second);
      }

      if (entry.state == StoredState::PENDING) {
        if (!available_match) {
          if (
            observed != by_id.end() &&
            observed->second != nullptr)
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              5000,
              "'%s' is AVAILABLE by id but its multi-mission geometry does not "
              "match cropped_trajectory.trajectory",
              id.c_str());
          }
          continue;
        }

        entry.state = StoredState::SUPERVISED;

        RCLCPP_INFO(
          get_logger(),
          "'%s' confirmed AVAILABLE; supervision activated | missions=%zu",
          id.c_str(),
          entry.adjusted.mission.size());
        continue;
      }

      if (!available_match) {
        entry.state = StoredState::PENDING;

        RCLCPP_WARN(
          get_logger(),
          "Supervised trajectory '%s' is no longer AVAILABLE with the expected "
          "multi-mission crop; re-queued",
          id.c_str());
      }
    }
  }

  StaticTrajectoryArray adjusted_snapshot() const
  {
    StaticTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    std::vector<const StoredEntry *> ordered;

    for (const auto & [_, entry] : stored_) {
      if (
        entry.state == StoredState::PENDING &&
        entry.crop_complete &&
        entry.adjusted_ready)
      {
        ordered.push_back(&entry);
      }
    }

    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](const StoredEntry * first, const StoredEntry * second) {
        if (first->adjusted.priority != second->adjusted.priority) {
          return first->adjusted.priority < second->adjusted.priority;
        }
        if (first->adjusted.ua_id != second->adjusted.ua_id) {
          return first->adjusted.ua_id < second->adjusted.ua_id;
        }
        return
          first->adjusted.trajectory_id <
          second->adjusted.trajectory_id;
      });

    output.trajectories.reserve(ordered.size());

    for (const auto * entry : ordered) {
      output.trajectories.push_back(entry->adjusted);
    }

    return output;
  }

  DetectedCollisionTrajectoryArray supervised_snapshot() const
  {
    DetectedCollisionTrajectoryArray output;
    output.header = latest_header_;
    output.header.stamp = now();

    std::vector<const StoredEntry *> ordered;

    for (const auto & [_, entry] : stored_) {
      if (entry.state == StoredState::SUPERVISED) {
        ordered.push_back(&entry);
      }
    }

    std::stable_sort(
      ordered.begin(),
      ordered.end(),
      [](const StoredEntry * first, const StoredEntry * second) {
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

    output.trajectories.reserve(ordered.size());

    for (const auto * entry : ordered) {
      // ORIGINAL collision payload and ORIGINAL multi-mission geometry.
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
    delete_all.header.frame_id = latest_header_.frame_id;
    delete_all.action = Marker::DELETEALL;
    output.markers.push_back(delete_all);

    int marker_id = 1;

    for (const auto & [id, entry] : stored_) {
      if (entry.state == StoredState::PENDING) {
        if (
          !entry.crop_complete ||
          entry.crop.retained_segments.empty())
        {
          continue;
        }

        Marker line;
        line.header.stamp = now();
        line.header.frame_id =
          entry.request.trajectory.frame_id;
        line.ns =
          "supervision/pending_upstream_crop/" + id;
        line.id = marker_id++;
        line.type = Marker::LINE_LIST;
        line.action = Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = pending_line_width_;
        line.color = color(0.10F, 0.75F, 1.00F, 1.00F);

        for (const auto & wrapped : entry.crop.retained_segments) {
          line.points.push_back(wrapped.segment.start);
          line.points.push_back(wrapped.segment.end);
        }

        output.markers.push_back(std::move(line));
        continue;
      }

      const auto collision_color =
        color(1.00F, 0.64F, 0.05F, 1.00F);

      for (const auto & node : entry.request.collision_nodes) {
        Marker sphere;
        sphere.header.stamp = now();
        sphere.header.frame_id =
          entry.request.trajectory.frame_id;
        sphere.ns =
          "supervision/collision_nodes/" + id;
        sphere.id = marker_id++;
        sphere.type = Marker::SPHERE;
        sphere.action = Marker::ADD;
        sphere.pose.orientation.w = 1.0;
        sphere.pose.position = node.position;
        sphere.scale.x = supervised_node_scale_;
        sphere.scale.y = supervised_node_scale_;
        sphere.scale.z = supervised_node_scale_;
        sphere.color = collision_color;
        output.markers.push_back(std::move(sphere));
      }
    }

    return output;
  }

  void publish_locked()
  {
    adjusted_pub_->publish(adjusted_snapshot());
    supervised_pub_->publish(supervised_snapshot());
    markers_pub_->publish(markers());
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
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

  std::map<std::string, StoredEntry> stored_;

  rclcpp::Subscription<RequestedSupervisionTrajectoryArray>::SharedPtr
    requested_sub_;
  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr
    available_sub_;

  rclcpp::Publisher<StaticTrajectoryArray>::SharedPtr
    adjusted_pub_;
  rclcpp::Publisher<DetectedCollisionTrajectoryArray>::SharedPtr
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
        supervision_trajectory_manager::SupervisionTrajectoryManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("supervision_trajectory_manager_node"),
      "Fatal: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
