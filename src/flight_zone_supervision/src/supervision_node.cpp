#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <flight_zone_supervision/action/supervision_control.hpp>

#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>

#include <flight_zone_msgs/msg/vehicle_zone_status.hpp>

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

namespace flight_zone_supervision
{

using SupervisionControl = flight_zone_supervision::action::SupervisionControl;
using GoalHandleSupervisionControl = rclcpp_action::ClientGoalHandle<SupervisionControl>;
using SupervisionControlClient = rclcpp_action::Client<SupervisionControl>;

using DetectedCollisionTrajectory = collision_detection::msg::DetectedCollisionTrajectory;
using DetectedCollisionTrajectoryArray = collision_detection::msg::DetectedCollisionTrajectoryArray;
using VehicleZoneStatus = flight_zone_msgs::msg::VehicleZoneStatus;
using StaticTrajectory = static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using TrajectorySegment = static_trajectory_manager::msg::TrajectorySegment;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct DroneContext
{
  std::string uas_namespace;
  std::string status_topic;
  std::string action_name;

  rclcpp::Subscription<VehicleZoneStatus>::SharedPtr status_subscription;
  SupervisionControlClient::SharedPtr action_client;

  bool status_received{false};
  VehicleZoneStatus last_status;
  std::chrono::steady_clock::time_point last_status_at{};

  // Last PAUSE/RESUME command dispatched and last one positively
  // acknowledged by the SupervisionControl action server.
  int last_control_command{-1};
  int last_confirmed_control_command{-1};
  bool control_command_in_flight{false};
};

enum class PairMode : uint8_t
{
  PRIMARY_SUPERVISED_CONTROL = 0U,
  WAITING_COUNTERPART_PAUSE = 1U,
  COUNTERPART_HELD_FOR_CLEARANCE = 2U,
  CLEARED = 3U
};

struct PairState
{
  std::string supervised_trajectory_id;
  std::string counterpart_trajectory_id;
  std::string supervised_uas;
  std::string counterpart_uas;

  PairMode mode{PairMode::PRIMARY_SUPERVISED_CONTROL};

  bool initial_pause_cycle_done{false};

  // Used while the supervised UAS is the primary controlled vehicle.
  bool progress_window_active{false};
  double progress_reference_distance_m{std::numeric_limits<double>::infinity()};
  std::chrono::steady_clock::time_point progress_reference_at{};

  // Once escalation occurs, the non-supervised counterpart becomes a
  // controlled/paused vehicle until the supervised UAS clears the collision
  // geometry.
  bool counterpart_control_engaged{false};
  bool collision_region_seen{false};
  std::chrono::steady_clock::time_point clearance_started_at{};

  bool status_valid{false};
  double distance_m{std::numeric_limits<double>::infinity()};
  double collision_region_distance_m{std::numeric_limits<double>::infinity()};
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

static std::size_t point_count(const TrajectorySegment & segment)
{
  return std::min(segment.x.size(), std::min(segment.y.size(), segment.z.size()));
}

static geometry_msgs::msg::Point point_at(const TrajectorySegment & segment, std::size_t index)
{
  geometry_msgs::msg::Point point;
  point.x = segment.x[index];
  point.y = segment.y[index];
  point.z = segment.z[index];
  return point;
}

class SupervisionNode : public rclcpp::Node
{
public:
  SupervisionNode()
  : Node("supervision_node")
  {
    flight_zone_id_ = trim_slashes(
      declare_parameter<std::string>("flight_zone_id", ""));

    available_topic_ = declare_parameter<std::string>(
      "available_static_trajectories_topic", "/available_static_trajectories");
    supervised_topic_ = declare_parameter<std::string>(
      "supervised_trajectories_topic", "/supervised_trajectories");
    active_topic_ = declare_parameter<std::string>(
      "active_trajectories_topic", "/active_trajectories");

    zone_status_suffix_ = trim_slashes(
      declare_parameter<std::string>("zone_status_suffix", "zone_status"));
    control_action_suffix_ = trim_slashes(
      declare_parameter<std::string>("control_action_suffix", "supervision_control"));

    security_distance_m_ = declare_parameter<double>("security_distance_m", 1.0);
    use_3d_distance_ = declare_parameter<bool>("use_3d_distance", true);
    zone_status_timeout_s_ = declare_parameter<double>("zone_status_timeout_s", 2.0);

    // Escalation of a supervised pair:
    // If the separation does not increase by at least this amount within the
    // configured time window while the supervised UAS is paused, the
    // non-supervised counterpart is paused and the supervised UAS is resumed.
    distance_progress_timeout_s_ =
      declare_parameter<double>("distance_progress_timeout_s", 0.5);
    distance_progress_epsilon_m_ =
      declare_parameter<double>("distance_progress_epsilon_m", 0.05);

    // The supervised UAS is considered outside the original collision section
    // when its minimum distance to every stored collision node/segment exceeds
    // collision_exit_distance_m + collision_exit_hysteresis_m.
    collision_exit_distance_m_ =
      declare_parameter<double>("collision_exit_distance_m", 1.0);
    collision_exit_hysteresis_m_ =
      declare_parameter<double>("collision_exit_hysteresis_m", 0.20);

    evaluation_period_ms_ = declare_parameter<int>("evaluation_period_ms", 100);
    action_wait_timeout_ms_ = declare_parameter<int>("action_wait_timeout_ms", 20);

    markers_topic_ = declare_parameter<std::string>(
      "markers_topic", "supervision_markers");
    trajectory_line_width_ = declare_parameter<double>("trajectory_line_width", 0.10);
    collision_segment_width_ = declare_parameter<double>("collision_segment_width", 0.20);
    collision_node_scale_ = declare_parameter<double>("collision_node_scale", 0.34);
    drone_scale_ = declare_parameter<double>("drone_scale", 0.42);
    text_height_ = declare_parameter<double>("text_height", 0.30);

    validate_parameters();

    auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    retained_qos.reliable();
    retained_qos.transient_local();

    available_subscription_ = create_subscription<StaticTrajectoryArray>(
      available_topic_, retained_qos,
      std::bind(&SupervisionNode::available_callback, this, std::placeholders::_1));

    supervised_subscription_ = create_subscription<DetectedCollisionTrajectoryArray>(
      supervised_topic_, retained_qos,
      std::bind(&SupervisionNode::supervised_callback, this, std::placeholders::_1));

    active_subscription_ = create_subscription<StaticTrajectoryArray>(
      active_topic_, retained_qos,
      std::bind(&SupervisionNode::active_callback, this, std::placeholders::_1));

    marker_publisher_ = create_publisher<MarkerArray>(markers_topic_, retained_qos);

    evaluation_timer_ = create_wall_timer(
      std::chrono::milliseconds(evaluation_period_ms_),
      std::bind(&SupervisionNode::evaluate, this));

    RCLCPP_INFO(
      get_logger(),
      "[/%s] supervision_node ready | available='%s' | supervised='%s' | active='%s' | "
      "security_distance=%.3f m | progress_timeout=%.2f s | progress_epsilon=%.3f m | "
      "collision_exit=%.3f+%.3f m | action_suffix='%s'",
      flight_zone_id_.c_str(), available_topic_.c_str(), supervised_topic_.c_str(),
      active_topic_.c_str(), security_distance_m_, distance_progress_timeout_s_,
      distance_progress_epsilon_m_, collision_exit_distance_m_,
      collision_exit_hysteresis_m_, control_action_suffix_.c_str());
  }

private:
  void validate_parameters() const
  {
    if (flight_zone_id_.empty() || flight_zone_id_.find('/') != std::string::npos) {
      throw std::runtime_error("flight_zone_id must be one non-empty ROS namespace segment");
    }

    const auto absolute = [](const std::string & value) {
        return !value.empty() && value.front() == '/';
      };

    if (!absolute(available_topic_) || !absolute(supervised_topic_) || !absolute(active_topic_)) {
      throw std::runtime_error("Global trajectory input topics must be absolute");
    }

    if (zone_status_suffix_.empty() || control_action_suffix_.empty() || markers_topic_.empty()) {
      throw std::runtime_error("Topic/action suffixes and markers_topic must not be empty");
    }

    if (!std::isfinite(security_distance_m_) || security_distance_m_ <= 0.0) {
      throw std::runtime_error("security_distance_m must be finite and > 0");
    }

    if (!std::isfinite(zone_status_timeout_s_) || zone_status_timeout_s_ <= 0.0) {
      throw std::runtime_error("zone_status_timeout_s must be finite and > 0");
    }

    if (!std::isfinite(distance_progress_timeout_s_) || distance_progress_timeout_s_ <= 0.0 ||
      !std::isfinite(distance_progress_epsilon_m_) || distance_progress_epsilon_m_ < 0.0)
    {
      throw std::runtime_error(
              "distance_progress_timeout_s must be > 0 and distance_progress_epsilon_m must be >= 0");
    }

    if (!std::isfinite(collision_exit_distance_m_) || collision_exit_distance_m_ < 0.0 ||
      !std::isfinite(collision_exit_hysteresis_m_) || collision_exit_hysteresis_m_ < 0.0)
    {
      throw std::runtime_error(
              "collision exit distance/hysteresis must be finite and >= 0");
    }

    if (evaluation_period_ms_ <= 0 || action_wait_timeout_ms_ < 0) {
      throw std::runtime_error("evaluation_period_ms must be > 0 and action_wait_timeout_ms >= 0");
    }
  }

  std::string status_topic_for(const std::string & uas_namespace) const
  {
    return "/" + flight_zone_id_ + "/" + trim_slashes(uas_namespace) + "/" + zone_status_suffix_;
  }

  std::string action_name_for(const std::string & uas_namespace) const
  {
    return "/" + flight_zone_id_ + "/" + trim_slashes(uas_namespace) + "/" + control_action_suffix_;
  }

  void ensure_drone_context_locked(const std::string & raw_uas_namespace)
  {
    const std::string uas_namespace = trim_slashes(raw_uas_namespace);
    if (uas_namespace.empty() || drones_.find(uas_namespace) != drones_.end()) {
      return;
    }

    DroneContext context;
    context.uas_namespace = uas_namespace;
    context.status_topic = status_topic_for(uas_namespace);
    context.action_name = action_name_for(uas_namespace);

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    status_qos.reliable();

    context.status_subscription = create_subscription<VehicleZoneStatus>(
      context.status_topic, status_qos,
      [this, uas_namespace](const VehicleZoneStatus::SharedPtr message) {
        status_callback(uas_namespace, message);
      });

    context.action_client = rclcpp_action::create_client<SupervisionControl>(
      this, context.action_name);

    RCLCPP_INFO(
      get_logger(), "[/%s] tracking UAS '%s' | status='%s' | action='%s'",
      flight_zone_id_.c_str(), uas_namespace.c_str(), context.status_topic.c_str(),
      context.action_name.c_str());

    drones_.emplace(uas_namespace, std::move(context));
  }

  const StaticTrajectory * find_known_trajectory_locked(const std::string & trajectory_id) const
  {
    const auto available_it = available_.find(trajectory_id);
    if (available_it != available_.end()) {
      return &available_it->second;
    }

    const auto active_it = active_.find(trajectory_id);
    if (active_it != active_.end()) {
      return &active_it->second;
    }

    const auto supervised_it = supervised_.find(trajectory_id);
    if (supervised_it != supervised_.end()) {
      return &supervised_it->second.trajectory;
    }

    return nullptr;
  }

  void rebuild_required_drones_locked()
  {
    std::set<std::string> required;

    for (const auto & [_, detected] : supervised_) {
      if (detected.trajectory.flight_zone_id != flight_zone_id_) {
        continue;
      }

      const std::string owner = trim_slashes(detected.trajectory.uas_namespace);
      if (!owner.empty()) {
        required.insert(owner);
      }

      for (const auto & conflicting_id : detected.conflicting_trajectory_ids) {
        const StaticTrajectory * counterpart = find_known_trajectory_locked(conflicting_id);
        if (counterpart == nullptr || counterpart->flight_zone_id != flight_zone_id_) {
          continue;
        }
        const std::string uas = trim_slashes(counterpart->uas_namespace);
        if (!uas.empty()) {
          required.insert(uas);
        }
      }
    }

    for (const auto & uas : required) {
      ensure_drone_context_locked(uas);
    }

    for (auto it = drones_.begin(); it != drones_.end();) {
      if (required.count(it->first) == 0U) {
        it = drones_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void status_callback(const std::string & uas_namespace, const VehicleZoneStatus::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = drones_.find(uas_namespace);
    if (it == drones_.end()) {
      return;
    }

    it->second.last_status = *message;
    it->second.last_status_at = std::chrono::steady_clock::now();
    it->second.status_received = true;
  }

  void available_callback(const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    available_.clear();

    for (const auto & trajectory : message->trajectories) {
      if (trajectory.flight_zone_id == flight_zone_id_ && !trajectory.trajectory_id.empty()) {
        available_[trajectory.trajectory_id] = trajectory;
      }
    }

    latest_header_ = message->header;
    rebuild_required_drones_locked();
  }

  void supervised_callback(const DetectedCollisionTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    supervised_.clear();

    for (const auto & detected : message->trajectories) {
      if (detected.trajectory.flight_zone_id == flight_zone_id_ &&
        !detected.trajectory.trajectory_id.empty())
      {
        supervised_[detected.trajectory.trajectory_id] = detected;
      }
    }

    latest_header_ = message->header;
    rebuild_required_drones_locked();
  }

  void active_callback(const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    active_.clear();

    for (const auto & trajectory : message->trajectories) {
      if (trajectory.flight_zone_id == flight_zone_id_ && !trajectory.trajectory_id.empty()) {
        active_[trajectory.trajectory_id] = trajectory;
      }
    }

    latest_header_ = message->header;
    rebuild_required_drones_locked();

    std::set<std::string> live_occurrences;
    for (const auto & [_, trajectory] : active_) {
      live_occurrences.insert(occurrence_key(trajectory));
    }

    for (auto it = execute_dispatched_.begin(); it != execute_dispatched_.end();) {
      if (live_occurrences.count(*it) == 0U) {
        it = execute_dispatched_.erase(it);
      } else {
        ++it;
      }
    }
  }

  static std::string occurrence_key(const StaticTrajectory & trajectory)
  {
    std::ostringstream stream;
    stream << trajectory.trajectory_id << "@"
           << trajectory.operation_start_utc.sec << ":"
           << trajectory.operation_start_utc.nanosec;
    return stream.str();
  }

  void send_action_locked(
    const std::string & uas_namespace,
    uint8_t command,
    const StaticTrajectory * trajectory,
    const std::string & reason,
    const std::string & occurrence = "")
  {
    ensure_drone_context_locked(uas_namespace);
    auto context_it = drones_.find(trim_slashes(uas_namespace));
    if (context_it == drones_.end() || !context_it->second.action_client) {
      return;
    }

    auto client = context_it->second.action_client;
    if (!client->wait_for_action_server(std::chrono::milliseconds(action_wait_timeout_ms_))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[/%s/%s] supervision control action '%s' is not available",
        flight_zone_id_.c_str(), uas_namespace.c_str(), context_it->second.action_name.c_str());
      return;
    }

    SupervisionControl::Goal goal;
    goal.command = command;
    goal.reason = reason;

    if (trajectory != nullptr) {
      goal.trajectories.header = latest_header_;
      goal.trajectories.header.stamp = now();
      goal.trajectories.trajectories.push_back(*trajectory);
    }

    rclcpp_action::Client<SupervisionControl>::SendGoalOptions options;
    options.goal_response_callback =
      [this, uas_namespace, command, occurrence](const GoalHandleSupervisionControl::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(
            get_logger(), "[/%s/%s] supervision action command=%u rejected",
            flight_zone_id_.c_str(), uas_namespace.c_str(), static_cast<unsigned>(command));
          std::lock_guard<std::mutex> lock(mutex_);
          if (command == SupervisionControl::Goal::EXECUTE && !occurrence.empty()) {
            execute_dispatched_.erase(occurrence);
          } else if (
            command == SupervisionControl::Goal::PAUSE ||
            command == SupervisionControl::Goal::RESUME)
          {
            const auto it = drones_.find(trim_slashes(uas_namespace));
            if (it != drones_.end()) {
              if (it->second.last_control_command == static_cast<int>(command)) {
                it->second.last_control_command = -1;
              }
              it->second.control_command_in_flight = false;
            }
          }
          return;
        }
        RCLCPP_INFO(
          get_logger(), "[/%s/%s] supervision action command=%u accepted",
          flight_zone_id_.c_str(), uas_namespace.c_str(), static_cast<unsigned>(command));
      };

    options.result_callback =
      [this, uas_namespace, command, occurrence](const GoalHandleSupervisionControl::WrappedResult & result) {
        const bool success =
          result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result &&
          result.result->accepted;

        if (success) {
          if (
            command == SupervisionControl::Goal::PAUSE ||
            command == SupervisionControl::Goal::RESUME)
          {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = drones_.find(trim_slashes(uas_namespace));
            if (it != drones_.end()) {
              it->second.last_confirmed_control_command =
                static_cast<int>(command);
              it->second.control_command_in_flight = false;
            }
          }
          return;
        }

        RCLCPP_WARN(
          get_logger(), "[/%s/%s] supervision action command=%u did not complete successfully",
          flight_zone_id_.c_str(), uas_namespace.c_str(), static_cast<unsigned>(command));

        std::lock_guard<std::mutex> lock(mutex_);
        if (command == SupervisionControl::Goal::EXECUTE && !occurrence.empty()) {
          execute_dispatched_.erase(occurrence);
        } else if (command == SupervisionControl::Goal::PAUSE || command == SupervisionControl::Goal::RESUME) {
          const auto it = drones_.find(trim_slashes(uas_namespace));
          if (it != drones_.end()) {
            if (it->second.last_control_command == static_cast<int>(command)) {
              it->second.last_control_command = -1;
            }
            it->second.control_command_in_flight = false;
          }
        }
      };

    client->async_send_goal(goal, options);

    if (command == SupervisionControl::Goal::EXECUTE && !occurrence.empty()) {
      execute_dispatched_.insert(occurrence);
    } else if (command == SupervisionControl::Goal::PAUSE || command == SupervisionControl::Goal::RESUME) {
      context_it->second.last_control_command = static_cast<int>(command);
      context_it->second.control_command_in_flight = true;
    }
  }

  void ensure_control_state_locked(const std::string & uas_namespace, bool pause, const std::string & reason)
  {
    ensure_drone_context_locked(uas_namespace);
    const auto it = drones_.find(trim_slashes(uas_namespace));
    if (it == drones_.end()) {
      return;
    }

    const uint8_t command = pause ? SupervisionControl::Goal::PAUSE : SupervisionControl::Goal::RESUME;
    if (it->second.last_control_command == static_cast<int>(command)) {
      return;
    }

    send_action_locked(uas_namespace, command, nullptr, reason);
  }

  bool control_command_confirmed_locked(
    const std::string & uas_namespace,
    uint8_t command) const
  {
    const auto it = drones_.find(trim_slashes(uas_namespace));
    if (it == drones_.end()) {
      return false;
    }

    return
      it->second.last_confirmed_control_command ==
      static_cast<int>(command);
  }

  bool fresh_status_locked(const std::string & uas_namespace, const VehicleZoneStatus *& output) const
  {
    const auto it = drones_.find(trim_slashes(uas_namespace));
    if (it == drones_.end() || !it->second.status_received) {
      output = nullptr;
      return false;
    }

    const double age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - it->second.last_status_at).count();

    if (age_s > zone_status_timeout_s_) {
      output = nullptr;
      return false;
    }

    output = &it->second.last_status;
    return true;
  }

  double vehicle_distance(const VehicleZoneStatus & first, const VehicleZoneStatus & second) const
  {
    const double dx = first.position.x - second.position.x;
    const double dy = first.position.y - second.position.y;
    const double dz = use_3d_distance_ ? first.position.z - second.position.z : 0.0;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double point_distance(
    const geometry_msgs::msg::Point & first,
    const geometry_msgs::msg::Point & second) const
  {
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    const double dz = use_3d_distance_ ? first.z - second.z : 0.0;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double point_segment_distance(
    const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & end) const
  {
    const double sx = start.x;
    const double sy = start.y;
    const double sz = use_3d_distance_ ? start.z : 0.0;

    const double ex = end.x;
    const double ey = end.y;
    const double ez = use_3d_distance_ ? end.z : 0.0;

    const double px = point.x;
    const double py = point.y;
    const double pz = use_3d_distance_ ? point.z : 0.0;

    const double vx = ex - sx;
    const double vy = ey - sy;
    const double vz = ez - sz;

    const double wx = px - sx;
    const double wy = py - sy;
    const double wz = pz - sz;

    const double vv = vx * vx + vy * vy + vz * vz;
    if (vv <= 1.0e-12) {
      const double dx = px - sx;
      const double dy = py - sy;
      const double dz = pz - sz;
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    const double projection =
      std::clamp((wx * vx + wy * vy + wz * vz) / vv, 0.0, 1.0);

    const double cx = sx + projection * vx;
    const double cy = sy + projection * vy;
    const double cz = sz + projection * vz;

    const double dx = px - cx;
    const double dy = py - cy;
    const double dz = pz - cz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double distance_to_collision_region(
    const DetectedCollisionTrajectory & detected,
    const geometry_msgs::msg::Point & position) const
  {
    double best = std::numeric_limits<double>::infinity();

    for (const auto & node : detected.collision_nodes) {
      best = std::min(best, point_distance(position, node.position));
    }

    for (const auto & segment : detected.collision_segments) {
      best = std::min(
        best,
        point_segment_distance(position, segment.start, segment.end));
    }

    return best;
  }

  static const char * pair_mode_name(PairMode mode)
  {
    switch (mode) {
      case PairMode::PRIMARY_SUPERVISED_CONTROL:
        return "PRIMARY_SUPERVISED_CONTROL";
      case PairMode::WAITING_COUNTERPART_PAUSE:
        return "WAITING_COUNTERPART_PAUSE";
      case PairMode::COUNTERPART_HELD_FOR_CLEARANCE:
        return "COUNTERPART_HELD_FOR_CLEARANCE";
      case PairMode::CLEARED:
        return "CLEARED";
      default:
        return "UNKNOWN";
    }
  }

  void dispatch_active_trajectories_locked()
  {
    for (const auto & [trajectory_id, active] : active_) {
      const std::string key = occurrence_key(active);
      if (execute_dispatched_.count(key) != 0U) {
        continue;
      }

      const auto supervised_it = supervised_.find(trajectory_id);
      if (supervised_it == supervised_.end()) {
        send_action_locked(
          active.uas_namespace,
          SupervisionControl::Goal::EXECUTE,
          &active,
          "active non-supervised trajectory",
          key);
        continue;
      }

      // A supervised trajectory is executed using the ORIGINAL full trajectory
      // retained in /supervised_trajectories. The cropped version only exists in
      // the planning/available flow to reserve the non-conflicting resources.
      send_action_locked(
        supervised_it->second.trajectory.uas_namespace,
        SupervisionControl::Goal::EXECUTE,
        &supervised_it->second.trajectory,
        "active supervised trajectory: execute original full geometry",
        key);
    }
  }

  std::set<std::string> counterpart_ids_for(const DetectedCollisionTrajectory & detected) const
  {
    std::set<std::string> ids(
      detected.conflicting_trajectory_ids.begin(), detected.conflicting_trajectory_ids.end());

    for (const auto & node : detected.collision_nodes) {
      ids.insert(node.conflicting_trajectory_ids.begin(), node.conflicting_trajectory_ids.end());
    }
    for (const auto & segment : detected.collision_segments) {
      ids.insert(segment.conflicting_trajectory_ids.begin(), segment.conflicting_trajectory_ids.end());
    }

    ids.erase(detected.trajectory.trajectory_id);
    return ids;
  }

  void evaluate_supervised_pairs_locked()
  {
    std::map<std::string, PairState> next_pairs;

    // Only UAS that have actually entered the supervision-control sequence are
    // included here. PAUSE always dominates RESUME if one UAS participates in
    // several simultaneous pairs.
    std::set<std::string> controlled_uas_seen;
    std::set<std::string> uas_requiring_pause;

    const auto steady_now = std::chrono::steady_clock::now();

    for (const auto & [supervised_id, detected] : supervised_) {
      const auto supervised_active_it = active_.find(supervised_id);
      if (supervised_active_it == active_.end()) {
        continue;
      }

      for (const auto & counterpart_id : counterpart_ids_for(detected)) {
        const auto counterpart_active_it = active_.find(counterpart_id);
        if (counterpart_active_it == active_.end()) {
          continue;
        }

        // The counterpart in this pair must be non-supervised. Two supervised
        // trajectories are not arbitrated by this specific state machine.
        if (supervised_.find(counterpart_id) != supervised_.end()) {
          continue;
        }

        const std::string supervised_uas =
          trim_slashes(detected.trajectory.uas_namespace);
        const std::string counterpart_uas =
          trim_slashes(counterpart_active_it->second.uas_namespace);

        if (supervised_uas.empty() || counterpart_uas.empty() ||
          supervised_uas == counterpart_uas)
        {
          continue;
        }

        ensure_drone_context_locked(supervised_uas);
        ensure_drone_context_locked(counterpart_uas);

        const std::string pair_key = supervised_id + "|" + counterpart_id;
        PairState state;

        const auto previous_it = pair_states_.find(pair_key);
        if (previous_it != pair_states_.end()) {
          state = previous_it->second;
        }

        state.supervised_trajectory_id = supervised_id;
        state.counterpart_trajectory_id = counterpart_id;
        state.supervised_uas = supervised_uas;
        state.counterpart_uas = counterpart_uas;
        state.status_valid = false;
        state.distance_m = std::numeric_limits<double>::infinity();
        state.collision_region_distance_m =
          std::numeric_limits<double>::infinity();

        // The supervised UAS is always part of this control sequence.
        controlled_uas_seen.insert(supervised_uas);

        if (state.counterpart_control_engaged) {
          controlled_uas_seen.insert(counterpart_uas);
        }

        // Step 1. Always begin by pausing the UAS executing the supervised
        // trajectory for at least one evaluation cycle. This also preserves the
        // deterministic PAUSE-before-release behavior of the previous node.
        if (!state.initial_pause_cycle_done) {
          uas_requiring_pause.insert(supervised_uas);
          state.initial_pause_cycle_done = true;
          state.progress_window_active = false;

          next_pairs[pair_key] = state;
          continue;
        }

        const VehicleZoneStatus * supervised_status = nullptr;
        const VehicleZoneStatus * counterpart_status = nullptr;

        const bool supervised_fresh =
          fresh_status_locked(supervised_uas, supervised_status);
        const bool counterpart_fresh =
          fresh_status_locked(counterpart_uas, counterpart_status);

        const bool statuses_valid =
          supervised_fresh &&
          counterpart_fresh &&
          supervised_status != nullptr &&
          counterpart_status != nullptr &&
          supervised_status->header.frame_id ==
          counterpart_status->header.frame_id;

        if (statuses_valid) {
          state.status_valid = true;
          state.distance_m =
            vehicle_distance(*supervised_status, *counterpart_status);
        }

        // ------------------------------------------------------------------
        // PHASE 1: normal supervision of the supervised UAS.
        // ------------------------------------------------------------------
        if (state.mode == PairMode::PRIMARY_SUPERVISED_CONTROL) {
          if (!statuses_valid) {
            // Fail safe while the primary measure is active: no fresh pair
            // position means the supervised UAS cannot be released.
            uas_requiring_pause.insert(supervised_uas);
            state.progress_window_active = false;

            next_pairs[pair_key] = state;
            continue;
          }

          if (state.distance_m > security_distance_m_) {
            // Normal PAUSE/RESUME supervision: sufficient separation releases
            // the supervised UAS.
            state.progress_window_active = false;
            next_pairs[pair_key] = state;
            continue;
          }

          // At/below the security threshold the supervised UAS is paused.
          // We then observe whether the separation actually improves.
          bool escalate = false;

          if (!state.progress_window_active) {
            state.progress_window_active = true;
            state.progress_reference_distance_m = state.distance_m;
            state.progress_reference_at = steady_now;
          } else if (
            state.distance_m >=
            state.progress_reference_distance_m +
            distance_progress_epsilon_m_)
          {
            // Separation is growing: the primary measure is working. Start a
            // fresh progress window from the improved distance.
            state.progress_reference_distance_m = state.distance_m;
            state.progress_reference_at = steady_now;
          } else {
            const double elapsed_s = std::chrono::duration<double>(
              steady_now - state.progress_reference_at).count();

            if (elapsed_s >= distance_progress_timeout_s_) {
              escalate = true;
            }
          }

          if (!escalate) {
            uas_requiring_pause.insert(supervised_uas);
            next_pairs[pair_key] = state;
            continue;
          }

          // ----------------------------------------------------------------
          // Step 2 + 3:
          // The first measure is not creating increasing separation.
          // Pause the non-supervised counterpart and RESUME the supervised UAS.
          // ----------------------------------------------------------------
          state.mode = PairMode::WAITING_COUNTERPART_PAUSE;
          state.counterpart_control_engaged = true;
          state.progress_window_active = false;
          state.clearance_started_at = steady_now;

          controlled_uas_seen.insert(counterpart_uas);

          // Strict command ordering: keep S paused until N has positively
          // acknowledged the PAUSE action.
          uas_requiring_pause.insert(supervised_uas);
          uas_requiring_pause.insert(counterpart_uas);

          const std::string collision_frame =
            trim_slashes(detected.trajectory.frame_id);
          const std::string status_frame =
            trim_slashes(supervised_status->header.frame_id);

          if (!collision_frame.empty() && collision_frame == status_frame) {
            state.collision_region_distance_m =
              distance_to_collision_region(
              detected,
              supervised_status->position);

            if (
              std::isfinite(state.collision_region_distance_m) &&
              state.collision_region_distance_m <=
              collision_exit_distance_m_)
            {
              state.collision_region_seen = true;
            }
          }

          RCLCPP_WARN(
            get_logger(),
            "[/%s] Escalating supervised pair '%s' <-> '%s': "
            "separation did not improve by %.3f m within %.2f s. "
            "request PAUSE counterpart UAS '%s'; supervised UAS '%s' remains PAUSED until acknowledgement.",
            flight_zone_id_.c_str(),
            supervised_id.c_str(),
            counterpart_id.c_str(),
            distance_progress_epsilon_m_,
            distance_progress_timeout_s_,
            counterpart_uas.c_str(),
            supervised_uas.c_str());

          next_pairs[pair_key] = state;
          continue;
        }

        // ------------------------------------------------------------------
        // ESCALATION BARRIER:
        // Request PAUSE on the counterpart first. S remains PAUSED until the
        // counterpart's control_manager positively acknowledges that command.
        // ------------------------------------------------------------------
        if (state.mode == PairMode::WAITING_COUNTERPART_PAUSE) {
          state.counterpart_control_engaged = true;
          controlled_uas_seen.insert(counterpart_uas);

          const bool counterpart_pause_confirmed =
            control_command_confirmed_locked(
            counterpart_uas,
            SupervisionControl::Goal::PAUSE);

          // N must remain paused in either case.
          uas_requiring_pause.insert(counterpart_uas);

          if (!counterpart_pause_confirmed) {
            // Until PAUSE is acknowledged for N, S also remains paused.
            uas_requiring_pause.insert(supervised_uas);
            next_pairs[pair_key] = state;
            continue;
          }

          state.mode = PairMode::COUNTERPART_HELD_FOR_CLEARANCE;

          RCLCPP_WARN(
            get_logger(),
            "[/%s] Counterpart UAS '%s' accepted PAUSE for pair '%s' <-> '%s'. "
            "RESUME supervised UAS '%s' so it can clear the collision section.",
            flight_zone_id_.c_str(),
            counterpart_uas.c_str(),
            supervised_id.c_str(),
            counterpart_id.c_str(),
            supervised_uas.c_str());

          next_pairs[pair_key] = state;
          continue;
        }

        // ------------------------------------------------------------------
        // PHASE 2: counterpart held, supervised UAS is allowed to traverse the
        // original collision section.
        // ------------------------------------------------------------------
        if (state.mode == PairMode::COUNTERPART_HELD_FOR_CLEARANCE) {
          state.counterpart_control_engaged = true;
          controlled_uas_seen.insert(counterpart_uas);

          // The non-supervised UAS remains paused throughout this phase.
          uas_requiring_pause.insert(counterpart_uas);

          // No PAUSE is requested for supervised_uas here, therefore the
          // aggregation below sends/maintains RESUME for it.

          if (!statuses_valid) {
            // We cannot prove that the supervised UAS has cleared the original
            // collision section. Keep the counterpart frozen.
            next_pairs[pair_key] = state;
            continue;
          }

          const std::string collision_frame =
            trim_slashes(detected.trajectory.frame_id);
          const std::string status_frame =
            trim_slashes(supervised_status->header.frame_id);

          if (collision_frame.empty() || collision_frame != status_frame) {
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              2000,
              "[/%s] Cannot verify clearance of supervised trajectory '%s': "
              "collision frame='%s', zone_status frame='%s'. Counterpart remains PAUSED.",
              flight_zone_id_.c_str(),
              supervised_id.c_str(),
              collision_frame.c_str(),
              status_frame.c_str());

            next_pairs[pair_key] = state;
            continue;
          }

          state.collision_region_distance_m =
            distance_to_collision_region(
            detected,
            supervised_status->position);

          if (!std::isfinite(state.collision_region_distance_m)) {
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              2000,
              "[/%s] Supervised trajectory '%s' has no usable collision nodes/segments; "
              "counterpart '%s' remains PAUSED.",
              flight_zone_id_.c_str(),
              supervised_id.c_str(),
              counterpart_id.c_str());

            next_pairs[pair_key] = state;
            continue;
          }

          if (
            state.collision_region_distance_m <=
            collision_exit_distance_m_)
          {
            state.collision_region_seen = true;
          }

          const double release_distance =
            collision_exit_distance_m_ +
            collision_exit_hysteresis_m_;

          if (
            !state.collision_region_seen ||
            state.collision_region_distance_m <= release_distance)
          {
            // The counterpart is released only after S has actually been
            // observed in/near the collision region and subsequently moved
            // beyond the release margin.
            next_pairs[pair_key] = state;
            continue;
          }

          // Step 4 + 5:
          // The supervised UAS is now outside the complete set of original
          // collision nodes/segments by the requested margin. The counterpart
          // can be released.
          state.mode = PairMode::CLEARED;

          RCLCPP_INFO(
            get_logger(),
            "[/%s] Supervised trajectory '%s' cleared collision section | "
            "distance_to_collision_geometry=%.3f m > %.3f m. "
            "RESUME counterpart UAS '%s'.",
            flight_zone_id_.c_str(),
            supervised_id.c_str(),
            state.collision_region_distance_m,
            release_distance,
            counterpart_uas.c_str());

          next_pairs[pair_key] = state;
          continue;
        }

        // ------------------------------------------------------------------
        // PHASE 3: this pair has already been deconflicted. Both UAS remain
        // released until one trajectory leaves /active_trajectories.
        // ------------------------------------------------------------------
        if (state.mode == PairMode::CLEARED) {
          state.counterpart_control_engaged = true;
          controlled_uas_seen.insert(counterpart_uas);

          next_pairs[pair_key] = state;
          continue;
        }
      }
    }

    // Aggregate all active pairs. PAUSE has precedence if one UAS participates
    // in several pairs with different local decisions.
    // Dispatch all PAUSE requests first. Only after those requests have been
    // issued do we dispatch RESUME requests. The WAITING_COUNTERPART_PAUSE
    // barrier additionally requires positive action acknowledgement before S
    // can be resumed during an escalation.
    for (const auto & uas : controlled_uas_seen) {
      if (uas_requiring_pause.count(uas) == 0U) {
        continue;
      }

      ensure_control_state_locked(
        uas,
        true,
        "flight-zone supervised deconfliction requires PAUSE");
    }

    for (const auto & uas : controlled_uas_seen) {
      if (uas_requiring_pause.count(uas) != 0U) {
        continue;
      }

      ensure_control_state_locked(
        uas,
        false,
        "flight-zone supervised deconfliction permits RESUME");
    }

    // Release UAS that were controlled by a previous pair but are no longer
    // represented by any active supervised pair.
    std::set<std::string> previously_controlled_uas;

    for (const auto & [_, previous] : pair_states_) {
      if (!previous.supervised_uas.empty()) {
        previously_controlled_uas.insert(
          previous.supervised_uas);
      }

      if (
        previous.counterpart_control_engaged &&
        !previous.counterpart_uas.empty())
      {
        previously_controlled_uas.insert(
          previous.counterpart_uas);
      }
    }

    for (const auto & uas : previously_controlled_uas) {
      if (controlled_uas_seen.count(uas) == 0U) {
        ensure_control_state_locked(
          uas,
          false,
          "no active supervised conflict pair remains");
      }
    }

    pair_states_ = std::move(next_pairs);
  }

  static void append_segment_points(
    std::vector<geometry_msgs::msg::Point> & out, const TrajectorySegment & segment)
  {
    for (std::size_t i = 0U; i < point_count(segment); ++i) {
      out.push_back(point_at(segment, i));
    }
  }

  static std::vector<geometry_msgs::msg::Point> expanded_points(const StaticTrajectory & trajectory)
  {
    std::vector<geometry_msgs::msg::Point> out;
    append_segment_points(out, trajectory.takeoff);
    for (uint32_t r = 0U; r < std::max<uint32_t>(1U, trajectory.repetitions); ++r) {
      append_segment_points(out, trajectory.mission);
    }
    append_segment_points(out, trajectory.landing);
    return out;
  }

  MarkerArray make_markers_locked() const
  {
    MarkerArray out;
    Marker clear;
    clear.header.stamp = now();
    clear.header.frame_id = latest_header_.frame_id;
    clear.action = Marker::DELETEALL;
    out.markers.push_back(clear);

    int marker_id = 1;

    for (const auto & [trajectory_id, active] : active_) {
      const bool supervised_active = supervised_.find(trajectory_id) != supervised_.end();
      const StaticTrajectory * displayed = &active;
      if (supervised_active) {
        displayed = &supervised_.at(trajectory_id).trajectory;
      }

      const auto points = expanded_points(*displayed);
      if (points.size() >= 2U) {
        Marker line;
        line.header.stamp = now();
        line.header.frame_id = displayed->frame_id;
        line.ns = supervised_active ? "supervision/active_supervised" : "supervision/active_normal";
        line.id = marker_id++;
        line.type = Marker::LINE_STRIP;
        line.action = Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = trajectory_line_width_;
        line.color = supervised_active ? color(1.0F, 0.55F, 0.0F, 1.0F) : color(0.10F, 0.85F, 0.25F, 1.0F);
        line.points = points;
        out.markers.push_back(std::move(line));
      }
    }

    for (const auto & [trajectory_id, detected] : supervised_) {
      if (active_.find(trajectory_id) == active_.end()) {
        continue;
      }

      for (const auto & node : detected.collision_nodes) {
        Marker sphere;
        sphere.header.stamp = now();
        sphere.header.frame_id = detected.trajectory.frame_id;
        sphere.ns = "supervision/collision_nodes";
        sphere.id = marker_id++;
        sphere.type = Marker::SPHERE;
        sphere.action = Marker::ADD;
        sphere.pose.orientation.w = 1.0;
        sphere.pose.position = node.position;
        sphere.scale.x = collision_node_scale_;
        sphere.scale.y = collision_node_scale_;
        sphere.scale.z = collision_node_scale_;
        sphere.color = color(1.0F, 0.10F, 0.05F, 1.0F);
        out.markers.push_back(std::move(sphere));
      }

      for (const auto & segment : detected.collision_segments) {
        Marker line;
        line.header.stamp = now();
        line.header.frame_id = detected.trajectory.frame_id;
        line.ns = "supervision/collision_segments";
        line.id = marker_id++;
        line.type = Marker::LINE_LIST;
        line.action = Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = collision_segment_width_;
        line.color = color(1.0F, 0.05F, 0.05F, 1.0F);
        line.points.push_back(segment.start);
        line.points.push_back(segment.end);
        out.markers.push_back(std::move(line));
      }
    }

    for (const auto & [uas, drone] : drones_) {
      if (!drone.status_received) {
        continue;
      }
      Marker sphere;
      sphere.header = drone.last_status.header;
      sphere.header.stamp = now();
      sphere.ns = "supervision/drones";
      sphere.id = marker_id++;
      sphere.type = Marker::SPHERE;
      sphere.action = Marker::ADD;
      sphere.pose.orientation.w = 1.0;
      sphere.pose.position = drone.last_status.position;
      sphere.scale.x = drone_scale_;
      sphere.scale.y = drone_scale_;
      sphere.scale.z = drone_scale_;
      sphere.color = drone.last_control_command == static_cast<int>(SupervisionControl::Goal::PAUSE) ?
        color(0.95F, 0.10F, 0.80F, 1.0F) : color(0.20F, 0.65F, 1.0F, 1.0F);
      out.markers.push_back(std::move(sphere));

      Marker text;
      text.header = drone.last_status.header;
      text.header.stamp = now();
      text.ns = "supervision/drone_labels";
      text.id = marker_id++;
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      text.pose.orientation.w = 1.0;
      text.pose.position = drone.last_status.position;
      text.pose.position.z += drone_scale_;
      text.scale.z = text_height_;
      text.color = color(1.0F, 1.0F, 1.0F, 1.0F);
      text.text = uas + (drone.last_control_command == static_cast<int>(SupervisionControl::Goal::PAUSE) ? " | PAUSED" : "");
      out.markers.push_back(std::move(text));
    }

    for (const auto & [_, pair] : pair_states_) {
      const auto first_it = drones_.find(pair.supervised_uas);
      const auto second_it = drones_.find(pair.counterpart_uas);
      if (first_it == drones_.end() || second_it == drones_.end() ||
        !first_it->second.status_received || !second_it->second.status_received)
      {
        continue;
      }

      Marker link;
      link.header = first_it->second.last_status.header;
      link.header.stamp = now();
      link.ns = "supervision/security_links";
      link.id = marker_id++;
      link.type = Marker::LINE_LIST;
      link.action = Marker::ADD;
      link.pose.orientation.w = 1.0;
      link.scale.x = 0.06;
      if (pair.mode == PairMode::WAITING_COUNTERPART_PAUSE) {
        link.color = color(1.0F, 0.0F, 0.75F, 1.0F);
      } else if (pair.mode == PairMode::COUNTERPART_HELD_FOR_CLEARANCE) {
        link.color = color(1.0F, 0.55F, 0.0F, 1.0F);
      } else if (pair.mode == PairMode::CLEARED) {
        link.color = color(0.10F, 0.90F, 0.20F, 1.0F);
      } else {
        link.color = pair.status_valid && pair.distance_m > security_distance_m_ ?
          color(0.10F, 0.90F, 0.20F, 1.0F) :
          color(1.0F, 0.10F, 0.10F, 1.0F);
      }
      link.points.push_back(first_it->second.last_status.position);
      link.points.push_back(second_it->second.last_status.position);
      out.markers.push_back(std::move(link));

      Marker text;
      text.header = first_it->second.last_status.header;
      text.header.stamp = now();
      text.ns = "supervision/security_pair_labels";
      text.id = marker_id++;
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      text.pose.orientation.w = 1.0;
      text.pose.position.x =
        0.5 * (
        first_it->second.last_status.position.x +
        second_it->second.last_status.position.x);
      text.pose.position.y =
        0.5 * (
        first_it->second.last_status.position.y +
        second_it->second.last_status.position.y);
      text.pose.position.z =
        0.5 * (
        first_it->second.last_status.position.z +
        second_it->second.last_status.position.z) +
        text_height_;
      text.scale.z = text_height_;
      text.color = color(1.0F, 1.0F, 1.0F, 1.0F);

      std::ostringstream pair_label;
      pair_label << pair_mode_name(pair.mode);
      if (pair.status_valid) {
        pair_label << " | d=" << pair.distance_m << " m";
      }
      if (std::isfinite(pair.collision_region_distance_m)) {
        pair_label << " | collision_d="
                   << pair.collision_region_distance_m << " m";
      }
      text.text = pair_label.str();

      out.markers.push_back(std::move(text));
    }

    return out;
  }

  void evaluate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dispatch_active_trajectories_locked();
    evaluate_supervised_pairs_locked();
    marker_publisher_->publish(make_markers_locked());
  }

  std::string flight_zone_id_;
  std::string available_topic_;
  std::string supervised_topic_;
  std::string active_topic_;
  std::string zone_status_suffix_;
  std::string control_action_suffix_;
  std::string markers_topic_;

  double security_distance_m_{1.0};
  bool use_3d_distance_{true};
  double zone_status_timeout_s_{2.0};

  double distance_progress_timeout_s_{0.5};
  double distance_progress_epsilon_m_{0.05};
  double collision_exit_distance_m_{1.0};
  double collision_exit_hysteresis_m_{0.20};

  int evaluation_period_ms_{100};
  int action_wait_timeout_ms_{20};

  double trajectory_line_width_{0.10};
  double collision_segment_width_{0.20};
  double collision_node_scale_{0.34};
  double drone_scale_{0.42};
  double text_height_{0.30};

  mutable std::mutex mutex_;
  std_msgs::msg::Header latest_header_;

  std::map<std::string, StaticTrajectory> available_;
  std::map<std::string, DetectedCollisionTrajectory> supervised_;
  std::map<std::string, StaticTrajectory> active_;
  std::map<std::string, DroneContext> drones_;
  std::map<std::string, PairState> pair_states_;
  std::set<std::string> execute_dispatched_;

  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr available_subscription_;
  rclcpp::Subscription<DetectedCollisionTrajectoryArray>::SharedPtr supervised_subscription_;
  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr active_subscription_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
};

}  // namespace flight_zone_supervision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<flight_zone_supervision::SupervisionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("supervision_node"), "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
