#include <rclcpp/rclcpp.hpp>

#include <distance_control/msg/uas_pair_distance.hpp>
#include <distance_control/msg/uas_related_distance_array.hpp>
#include <flight_zone_msgs/msg/vehicle_zone_status.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace distance_control
{

using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using VehicleZoneStatus = flight_zone_msgs::msg::VehicleZoneStatus;
using UasPairDistance = distance_control::msg::UasPairDistance;
using UasRelatedDistanceArray = distance_control::msg::UasRelatedDistanceArray;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct UasIdentity
{
  uint32_t ua_id{0U};
  std::string uas_namespace;
  std::string flight_zone_id;
};

struct UasContext
{
  UasIdentity identity;
  std::string key;
  std::string status_topic;
  rclcpp::Subscription<VehicleZoneStatus>::SharedPtr status_subscription;
  bool status_received{false};
  VehicleZoneStatus last_status;
  std::chrono::steady_clock::time_point last_status_at{};
};

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

static std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F)
{
  std_msgs::msg::ColorRGBA out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.a = a;
  return out;
}

class DistanceControlNode : public rclcpp::Node
{
public:
  DistanceControlNode()
  : Node("distance_control_node")
  {
    active_topic_ = declare_parameter<std::string>(
      "active_trajectories_topic", "/active_trajectories");
    related_distance_topic_ = declare_parameter<std::string>(
      "related_distance_topic", "/uas_related_distance");
    zone_status_suffix_ = trim_slashes(
      declare_parameter<std::string>("zone_status_suffix", "zone_status"));
    zone_status_timeout_s_ = declare_parameter<double>(
      "zone_status_timeout_s", 2.0);
    evaluation_period_ms_ = declare_parameter<int>(
      "evaluation_period_ms", 100);
    markers_topic_ = declare_parameter<std::string>(
      "markers_topic", "/uas_related_distance/markers");
    drone_scale_ = declare_parameter<double>("drone_scale", 0.42);
    pair_line_width_ = declare_parameter<double>("pair_line_width", 0.06);
    text_height_ = declare_parameter<double>("text_height", 0.30);

    validate_parameters();

    auto snapshot_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    snapshot_qos.reliable();
    snapshot_qos.transient_local();

    active_subscription_ = create_subscription<StaticTrajectoryArray>(
      active_topic_, snapshot_qos,
      std::bind(&DistanceControlNode::active_callback, this, std::placeholders::_1));

    related_distance_publisher_ = create_publisher<UasRelatedDistanceArray>(
      related_distance_topic_, snapshot_qos);
    marker_publisher_ = create_publisher<MarkerArray>(
      markers_topic_, snapshot_qos);

    evaluation_timer_ = create_wall_timer(
      std::chrono::milliseconds(evaluation_period_ms_),
      std::bind(&DistanceControlNode::evaluate, this));

    RCLCPP_INFO(
      get_logger(),
      "distance_control_node ready | active='%s' | output='%s' | "
      "zone_status_suffix='%s' | timeout=%.3f s | period=%d ms | markers='%s'",
      active_topic_.c_str(), related_distance_topic_.c_str(), zone_status_suffix_.c_str(),
      zone_status_timeout_s_, evaluation_period_ms_, markers_topic_.c_str());
  }

private:
  void validate_parameters() const
  {
    const auto absolute = [](const std::string & value) {
        return !value.empty() && value.front() == '/';
      };

    if (!absolute(active_topic_) || !absolute(related_distance_topic_) || !absolute(markers_topic_)) {
      throw std::runtime_error("active/output/marker topics must be absolute ROS names");
    }
    if (zone_status_suffix_.empty() || zone_status_suffix_.find('/') != std::string::npos) {
      throw std::runtime_error("zone_status_suffix must be one non-empty relative ROS name segment");
    }
    if (!std::isfinite(zone_status_timeout_s_) || zone_status_timeout_s_ <= 0.0) {
      throw std::runtime_error("zone_status_timeout_s must be finite and > 0");
    }
    if (evaluation_period_ms_ <= 0) {
      throw std::runtime_error("evaluation_period_ms must be > 0");
    }
    if (!std::isfinite(drone_scale_) || drone_scale_ <= 0.0 ||
      !std::isfinite(pair_line_width_) || pair_line_width_ <= 0.0 ||
      !std::isfinite(text_height_) || text_height_ <= 0.0)
    {
      throw std::runtime_error("marker dimensions must be finite and > 0");
    }
  }

  static std::string uas_key(
    const std::string & flight_zone_id,
    const std::string & uas_namespace)
  {
    return trim_slashes(flight_zone_id) + "/" + trim_slashes(uas_namespace);
  }

  std::string status_topic_for(const UasIdentity & identity) const
  {
    return "/" + trim_slashes(identity.flight_zone_id) + "/" +
           trim_slashes(identity.uas_namespace) + "/" + zone_status_suffix_;
  }

  void ensure_uas_context_locked(const UasIdentity & identity)
  {
    const std::string key = uas_key(identity.flight_zone_id, identity.uas_namespace);
    auto existing = uas_contexts_.find(key);
    if (existing != uas_contexts_.end()) {
      existing->second.identity = identity;
      return;
    }

    UasContext context;
    context.identity = identity;
    context.key = key;
    context.status_topic = status_topic_for(identity);

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    status_qos.reliable();

    context.status_subscription = create_subscription<VehicleZoneStatus>(
      context.status_topic, status_qos,
      [this, key](const VehicleZoneStatus::SharedPtr message) {
        status_callback(key, message);
      });

    RCLCPP_INFO(
      get_logger(), "Tracking active UAS '%s' (ua_id=%u) | zone_status='%s'",
      key.c_str(), identity.ua_id, context.status_topic.c_str());

    uas_contexts_.emplace(key, std::move(context));
  }

  void status_callback(
    const std::string & key,
    const VehicleZoneStatus::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = uas_contexts_.find(key);
    if (it == uas_contexts_.end()) {
      return;
    }
    it->second.last_status = *message;
    it->second.last_status_at = std::chrono::steady_clock::now();
    it->second.status_received = true;
  }

  void active_callback(const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_active_header_ = message->header;

    std::map<std::string, UasIdentity> required;

    for (const auto & trajectory : message->trajectories) {
      const std::string flight_zone_id = trim_slashes(trajectory.flight_zone_id);
      const std::string uas_namespace = trim_slashes(trajectory.uas_namespace);

      if (flight_zone_id.empty() || uas_namespace.empty()) {
        RCLCPP_WARN(
          get_logger(), "Ignoring active trajectory '%s': empty flight_zone_id or uas_namespace",
          trajectory.trajectory_id.c_str());
        continue;
      }

      UasIdentity identity;
      identity.ua_id = trajectory.ua_id;
      identity.uas_namespace = uas_namespace;
      identity.flight_zone_id = flight_zone_id;

      const std::string key = uas_key(flight_zone_id, uas_namespace);
      const auto inserted = required.emplace(key, identity);
      if (!inserted.second && inserted.first->second.ua_id != identity.ua_id) {
        RCLCPP_WARN(
          get_logger(), "Inconsistent ua_id values for active UAS '%s': %u vs %u",
          key.c_str(), inserted.first->second.ua_id, identity.ua_id);
      }
    }

    for (const auto & [_, identity] : required) {
      ensure_uas_context_locked(identity);
    }

    for (auto it = uas_contexts_.begin(); it != uas_contexts_.end();) {
      if (required.count(it->first) == 0U) {
        RCLCPP_INFO(
          get_logger(), "UAS '%s' is no longer active; removing zone_status subscription",
          it->first.c_str());
        it = uas_contexts_.erase(it);
      } else {
        ++it;
      }
    }

    active_uas_keys_.clear();
    for (const auto & [key, _] : required) {
      active_uas_keys_.insert(key);
    }
  }

  bool fresh_status_locked(const UasContext & context) const
  {
    if (!context.status_received) {
      return false;
    }
    const double age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - context.last_status_at).count();
    return age_s <= zone_status_timeout_s_;
  }

  static double xy_distance(
    const VehicleZoneStatus & first,
    const VehicleZoneStatus & second)
  {
    const double dx = first.position.x - second.position.x;
    const double dy = first.position.y - second.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  UasRelatedDistanceArray make_distance_snapshot_locked()
  {
    UasRelatedDistanceArray out;
    out.header.stamp = now();
    out.header.frame_id = latest_active_header_.frame_id;
    out.active_uas_count = static_cast<uint32_t>(active_uas_keys_.size());

    std::vector<const UasContext *> valid;
    valid.reserve(uas_contexts_.size());

    for (const auto & key : active_uas_keys_) {
      const auto it = uas_contexts_.find(key);
      if (it == uas_contexts_.end() || !fresh_status_locked(it->second)) {
        continue;
      }
      valid.push_back(&it->second);
    }

    out.valid_uas_count = static_cast<uint32_t>(valid.size());

    for (std::size_t i = 0U; i < valid.size(); ++i) {
      for (std::size_t j = i + 1U; j < valid.size(); ++j) {
        const UasContext & first = *valid[i];
        const UasContext & second = *valid[j];

        const std::string first_frame = trim_slashes(first.last_status.header.frame_id);
        const std::string second_frame = trim_slashes(second.last_status.header.frame_id);

        if (first_frame.empty() || second_frame.empty() || first_frame != second_frame) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot calculate pair '%s' <-> '%s': zone_status frames differ ('%s' vs '%s')",
            first.identity.uas_namespace.c_str(), second.identity.uas_namespace.c_str(),
            first_frame.c_str(), second_frame.c_str());
          continue;
        }

        UasPairDistance pair;
        pair.header.stamp = out.header.stamp;
        pair.header.frame_id = first.last_status.header.frame_id;

        pair.first_ua_id = first.identity.ua_id;
        pair.first_uas_namespace = first.identity.uas_namespace;
        pair.first_flight_zone_id = first.identity.flight_zone_id;
        pair.first_position = first.last_status.position;

        pair.second_ua_id = second.identity.ua_id;
        pair.second_uas_namespace = second.identity.uas_namespace;
        pair.second_flight_zone_id = second.identity.flight_zone_id;
        pair.second_position = second.last_status.position;

        pair.distance_xy_m = xy_distance(first.last_status, second.last_status);
        out.pairs.push_back(std::move(pair));
      }
    }

    out.valid_pair_count = static_cast<uint32_t>(out.pairs.size());
    return out;
  }

  MarkerArray make_markers_locked(const UasRelatedDistanceArray & distances) const
  {
    MarkerArray out;

    Marker clear;
    clear.header.stamp = now();
    clear.header.frame_id = latest_active_header_.frame_id;
    clear.action = Marker::DELETEALL;
    out.markers.push_back(clear);

    int marker_id = 1;

    // Only one LINE_LIST marker is published for each valid UAS pair.
    //
    // The supervision trigger is strict:
    //   distance_xy < 1.5 m  -> red
    //   distance_xy >= 1.5 m -> green
    //
    // Therefore exactly 1.5 m is shown in green, consistently with the
    // supervisor_node condition that starts deconfliction only below 1.5 m.
    constexpr double kSafetyDistanceM = 1.5;

    for (const auto & pair : distances.pairs) {
      Marker line;
      line.header = pair.header;
      line.header.stamp = now();
      line.ns = "distance_control/pairs";
      line.id = marker_id++;
      line.type = Marker::LINE_LIST;
      line.action = Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = pair_line_width_;

      if (pair.distance_xy_m < kSafetyDistanceM) {
        line.color = color(1.0F, 0.0F, 0.0F, 1.0F);
      } else {
        line.color = color(0.0F, 1.0F, 0.0F, 1.0F);
      }

      line.points.push_back(pair.first_position);
      line.points.push_back(pair.second_position);

      out.markers.push_back(std::move(line));
    }

    return out;
  }

  void evaluate()
  {
    UasRelatedDistanceArray distance_snapshot;
    MarkerArray markers;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      distance_snapshot = make_distance_snapshot_locked();
      markers = make_markers_locked(distance_snapshot);
    }

    related_distance_publisher_->publish(distance_snapshot);
    marker_publisher_->publish(markers);
  }

  std::string active_topic_;
  std::string related_distance_topic_;
  std::string zone_status_suffix_;
  std::string markers_topic_;

  double zone_status_timeout_s_{2.0};
  double drone_scale_{0.42};
  double pair_line_width_{0.06};
  double text_height_{0.30};
  int evaluation_period_ms_{100};

  mutable std::mutex mutex_;
  std_msgs::msg::Header latest_active_header_;
  std::set<std::string> active_uas_keys_;
  std::map<std::string, UasContext> uas_contexts_;

  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr active_subscription_;
  rclcpp::Publisher<UasRelatedDistanceArray>::SharedPtr related_distance_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
};

}  // namespace distance_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<distance_control::DistanceControlNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("distance_control_node"),
      "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
