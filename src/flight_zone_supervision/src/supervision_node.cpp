#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <flight_zone_supervision/action/supervision_control.hpp>

#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>

#include <distance_control/msg/uas_pair_distance.hpp>
#include <distance_control/msg/uas_related_distance_array.hpp>

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

using SupervisionControl =
  flight_zone_supervision::action::SupervisionControl;
using GoalHandleSupervisionControl =
  rclcpp_action::ClientGoalHandle<SupervisionControl>;
using SupervisionControlClient =
  rclcpp_action::Client<SupervisionControl>;

using DetectedCollisionTrajectory =
  collision_detection::msg::DetectedCollisionTrajectory;
using DetectedCollisionTrajectoryArray =
  collision_detection::msg::DetectedCollisionTrajectoryArray;

using UasPairDistance =
  distance_control::msg::UasPairDistance;
using UasRelatedDistanceArray =
  distance_control::msg::UasRelatedDistanceArray;

using StaticTrajectory =
  static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray =
  static_trajectory_manager::msg::StaticTrajectoryArray;
using TrajectorySegment =
  static_trajectory_manager::msg::TrajectorySegment;

using Marker =
  visualization_msgs::msg::Marker;
using MarkerArray =
  visualization_msgs::msg::MarkerArray;


struct DroneContext
{
  std::string uas_namespace;
  std::string action_name;

  SupervisionControlClient::SharedPtr action_client;

  // Last runtime command dispatched and last one positively acknowledged
  // by the SupervisionControl action server.
  int last_control_command{-1};
  int last_confirmed_control_command{-1};
  bool control_command_in_flight{false};
};


enum class PairMode : uint8_t
{
  MONITORING = 0U,
  WAITING_BOTH_PAUSE = 1U,
  WAITING_ELEVATE = 2U,
  WAITING_COUNTERPART_RESUME = 3U,
  COUNTERPART_RUNNING = 4U,
  WAITING_DESCEND = 5U,
  WAITING_SUPERVISED_RESUME = 6U,
  CLEARED = 7U
};


struct PairState
{
  std::string supervised_trajectory_id;
  std::string counterpart_trajectory_id;
  std::string supervised_uas;
  std::string counterpart_uas;

  PairMode mode{PairMode::MONITORING};

  bool distance_valid{false};
  double distance_xy_m{
    std::numeric_limits<double>::infinity()};

  // Captured from /uas_related_distance exactly when the pair enters
  // the deconfliction sequence. ELEVATE raises one grid step above this
  // altitude and DESCEND must restore it.
  bool reference_altitude_valid{false};
  double reference_z_m{0.0};
  std::string reference_frame;
};


struct RelatedPairView
{
  bool valid{false};
  std_msgs::msg::Header header;

  geometry_msgs::msg::Point supervised_position;
  geometry_msgs::msg::Point counterpart_position;

  double distance_xy_m{
    std::numeric_limits<double>::infinity()};
};


struct PositionSample
{
  std_msgs::msg::Header header;
  geometry_msgs::msg::Point position;
  std::chrono::steady_clock::time_point received_at{};
};


static std::string trim_slashes(
  std::string value)
{
  while (
    !value.empty() &&
    value.front() == '/')
  {
    value.erase(value.begin());
  }

  while (
    !value.empty() &&
    value.back() == '/')
  {
    value.pop_back();
  }

  return value;
}


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


static std::size_t point_count(
  const TrajectorySegment & segment)
{
  return std::min(
    segment.x.size(),
    std::min(
      segment.y.size(),
      segment.z.size()));
}


static geometry_msgs::msg::Point point_at(
  const TrajectorySegment & segment,
  std::size_t index)
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
    flight_zone_id_ =
      trim_slashes(
      declare_parameter<std::string>(
        "flight_zone_id",
        ""));

    available_topic_ =
      declare_parameter<std::string>(
      "available_static_trajectories_topic",
      "/available_static_trajectories");

    supervised_topic_ =
      declare_parameter<std::string>(
      "supervised_trajectories_topic",
      "/supervised_trajectories");

    active_topic_ =
      declare_parameter<std::string>(
      "active_trajectories_topic",
      "/active_trajectories");

    related_distance_topic_ =
      declare_parameter<std::string>(
      "related_distance_topic",
      "/uas_related_distance");

    control_action_suffix_ =
      trim_slashes(
      declare_parameter<std::string>(
        "control_action_suffix",
        "supervision_control"));

    // Runtime deconfliction sequence.
    //
    // 1) distance_xy < pause_distance_xy_m:
    //      PAUSE both UAS.
    // 2) ELEVATE supervised UAS one grid step.
    // 3) RESUME non-supervised counterpart.
    // 4) Keep reading distance_xy from /uas_related_distance.
    // 5) distance_xy > release_distance_xy_m:
    //      DESCEND supervised UAS to captured altitude.
    // 6) RESUME supervised trajectory.
    //
    // No distance is calculated by this node.
    pause_distance_xy_m_ =
      declare_parameter<double>(
      "pause_distance_xy_m",
      1.5);

    release_distance_xy_m_ =
      declare_parameter<double>(
      "release_distance_xy_m",
      2.0);

    vertical_grid_step_m_ =
      declare_parameter<double>(
      "vertical_grid_step_m",
      1.0);

    vertical_position_tolerance_m_ =
      declare_parameter<double>(
      "vertical_position_tolerance_m",
      0.15);

    related_distance_timeout_s_ =
      declare_parameter<double>(
      "related_distance_timeout_s",
      2.0);

    evaluation_period_ms_ =
      declare_parameter<int>(
      "evaluation_period_ms",
      100);

    action_wait_timeout_ms_ =
      declare_parameter<int>(
      "action_wait_timeout_ms",
      20);

    markers_topic_ =
      declare_parameter<std::string>(
      "markers_topic",
      "supervision_markers");

    trajectory_line_width_ =
      declare_parameter<double>(
      "trajectory_line_width",
      0.10);

    collision_segment_width_ =
      declare_parameter<double>(
      "collision_segment_width",
      0.20);

    collision_node_scale_ =
      declare_parameter<double>(
      "collision_node_scale",
      0.34);

    drone_scale_ =
      declare_parameter<double>(
      "drone_scale",
      0.42);

    pair_line_width_ =
      declare_parameter<double>(
      "pair_line_width",
      0.06);

    text_height_ =
      declare_parameter<double>(
      "text_height",
      0.30);

    validate_parameters();

    auto retained_qos =
      rclcpp::QoS(
      rclcpp::KeepLast(1));

    retained_qos.reliable();
    retained_qos.transient_local();

    available_subscription_ =
      create_subscription<StaticTrajectoryArray>(
      available_topic_,
      retained_qos,
      std::bind(
        &SupervisionNode::available_callback,
        this,
        std::placeholders::_1));

    supervised_subscription_ =
      create_subscription<
        DetectedCollisionTrajectoryArray>(
      supervised_topic_,
      retained_qos,
      std::bind(
        &SupervisionNode::supervised_callback,
        this,
        std::placeholders::_1));

    active_subscription_ =
      create_subscription<StaticTrajectoryArray>(
      active_topic_,
      retained_qos,
      std::bind(
        &SupervisionNode::active_callback,
        this,
        std::placeholders::_1));

    related_distance_subscription_ =
      create_subscription<UasRelatedDistanceArray>(
      related_distance_topic_,
      retained_qos,
      std::bind(
        &SupervisionNode::related_distance_callback,
        this,
        std::placeholders::_1));

    marker_publisher_ =
      create_publisher<MarkerArray>(
      markers_topic_,
      retained_qos);

    evaluation_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(
        evaluation_period_ms_),
      std::bind(
        &SupervisionNode::evaluate,
        this));

    RCLCPP_INFO(
      get_logger(),
      "[/%s] supervision_node ready | "
      "available='%s' | supervised='%s' | "
      "active='%s' | related_distance='%s' | "
      "pause_xy<%.3f m | release_xy>%.3f m | "
      "vertical_grid_step=%.3f m | "
      "vertical_tolerance=%.3f m | "
      "distance_timeout=%.3f s | action_suffix='%s'",
      flight_zone_id_.c_str(),
      available_topic_.c_str(),
      supervised_topic_.c_str(),
      active_topic_.c_str(),
      related_distance_topic_.c_str(),
      pause_distance_xy_m_,
      release_distance_xy_m_,
      vertical_grid_step_m_,
      vertical_position_tolerance_m_,
      related_distance_timeout_s_,
      control_action_suffix_.c_str());
  }

private:
  void validate_parameters() const
  {
    if (
      flight_zone_id_.empty() ||
      flight_zone_id_.find('/') !=
      std::string::npos)
    {
      throw std::runtime_error(
              "flight_zone_id must be one "
              "non-empty ROS namespace segment");
    }

    const auto absolute =
      [](const std::string & value) {
        return
          !value.empty() &&
          value.front() == '/';
      };

    if (
      !absolute(available_topic_) ||
      !absolute(supervised_topic_) ||
      !absolute(active_topic_) ||
      !absolute(related_distance_topic_))
    {
      throw std::runtime_error(
              "Global input topics must be absolute");
    }

    if (
      control_action_suffix_.empty() ||
      control_action_suffix_.find('/') !=
      std::string::npos ||
      markers_topic_.empty())
    {
      throw std::runtime_error(
              "control_action_suffix must be one "
              "non-empty relative ROS name segment "
              "and markers_topic must not be empty");
    }

    if (
      !std::isfinite(
        pause_distance_xy_m_) ||
      pause_distance_xy_m_ <= 0.0)
    {
      throw std::runtime_error(
              "pause_distance_xy_m "
              "must be finite and > 0");
    }

    if (
      !std::isfinite(
        release_distance_xy_m_) ||
      release_distance_xy_m_ <=
      pause_distance_xy_m_)
    {
      throw std::runtime_error(
              "release_distance_xy_m must be "
              "finite and > pause_distance_xy_m");
    }

    if (
      !std::isfinite(
        vertical_grid_step_m_) ||
      vertical_grid_step_m_ <= 0.0)
    {
      throw std::runtime_error(
              "vertical_grid_step_m "
              "must be finite and > 0");
    }

    if (
      !std::isfinite(
        vertical_position_tolerance_m_) ||
      vertical_position_tolerance_m_ <= 0.0 ||
      vertical_position_tolerance_m_ >=
      vertical_grid_step_m_)
    {
      throw std::runtime_error(
              "vertical_position_tolerance_m must "
              "be finite, > 0 and < "
              "vertical_grid_step_m");
    }

    if (
      !std::isfinite(
        related_distance_timeout_s_) ||
      related_distance_timeout_s_ <= 0.0)
    {
      throw std::runtime_error(
              "related_distance_timeout_s "
              "must be finite and > 0");
    }

    if (
      evaluation_period_ms_ <= 0 ||
      action_wait_timeout_ms_ < 0)
    {
      throw std::runtime_error(
              "evaluation_period_ms must be > 0 "
              "and action_wait_timeout_ms >= 0");
    }

    if (
      !std::isfinite(
        trajectory_line_width_) ||
      trajectory_line_width_ <= 0.0 ||
      !std::isfinite(
        collision_segment_width_) ||
      collision_segment_width_ <= 0.0 ||
      !std::isfinite(
        collision_node_scale_) ||
      collision_node_scale_ <= 0.0 ||
      !std::isfinite(
        drone_scale_) ||
      drone_scale_ <= 0.0 ||
      !std::isfinite(
        pair_line_width_) ||
      pair_line_width_ <= 0.0 ||
      !std::isfinite(
        text_height_) ||
      text_height_ <= 0.0)
    {
      throw std::runtime_error(
              "marker dimensions must be finite "
              "and > 0");
    }
  }


  std::string action_name_for(
    const std::string & uas_namespace) const
  {
    return
      "/" +
      flight_zone_id_ +
      "/" +
      trim_slashes(
        uas_namespace) +
      "/" +
      control_action_suffix_;
  }


  void ensure_drone_context_locked(
    const std::string & raw_uas_namespace)
  {
    const std::string uas_namespace =
      trim_slashes(
      raw_uas_namespace);

    if (uas_namespace.empty()) {
      return;
    }

    auto existing =
      drones_.find(
      uas_namespace);

    if (
      existing !=
      drones_.end())
    {
      return;
    }

    DroneContext context;
    context.uas_namespace =
      uas_namespace;
    context.action_name =
      action_name_for(
      uas_namespace);

    context.action_client =
      rclcpp_action::create_client<
        SupervisionControl>(
      this,
      context.action_name);

    RCLCPP_INFO(
      get_logger(),
      "[/%s] tracking UAS '%s' | action='%s' | "
      "position/distance source='%s'",
      flight_zone_id_.c_str(),
      uas_namespace.c_str(),
      context.action_name.c_str(),
      related_distance_topic_.c_str());

    drones_.emplace(
      uas_namespace,
      std::move(context));
  }


  const StaticTrajectory *
  find_known_trajectory_locked(
    const std::string & trajectory_id) const
  {
    const auto available_it =
      available_.find(
      trajectory_id);

    if (
      available_it !=
      available_.end())
    {
      return
        &available_it->second;
    }

    const auto active_it =
      active_.find(
      trajectory_id);

    if (
      active_it !=
      active_.end())
    {
      return
        &active_it->second;
    }

    const auto supervised_it =
      supervised_.find(
      trajectory_id);

    if (
      supervised_it !=
      supervised_.end())
    {
      return
        &supervised_it->second.trajectory;
    }

    return nullptr;
  }


  void rebuild_required_drones_locked()
  {
    std::set<std::string> required;

    // Every active trajectory in this flight-zone belongs to a UAS whose
    // action state must be visualised and may receive EXECUTE.
    for (
      const auto & [_, trajectory] :
      active_)
    {
      const std::string uas =
        trim_slashes(
        trajectory.uas_namespace);

      if (!uas.empty()) {
        required.insert(
          uas);
      }
    }

    // Keep supervised owners even if a retained supervised snapshot arrives
    // before their active occurrence.
    for (
      const auto & [_, detected] :
      supervised_)
    {
      if (
        trim_slashes(
          detected.trajectory.flight_zone_id) !=
        flight_zone_id_)
      {
        continue;
      }

      const std::string owner =
        trim_slashes(
        detected.trajectory.uas_namespace);

      if (!owner.empty()) {
        required.insert(
          owner);
      }

      for (
        const auto & conflicting_id :
        detected.conflicting_trajectory_ids)
      {
        const StaticTrajectory * counterpart =
          find_known_trajectory_locked(
          conflicting_id);

        if (
          counterpart == nullptr ||
          trim_slashes(
            counterpart->flight_zone_id) !=
          flight_zone_id_)
        {
          continue;
        }

        const std::string uas =
          trim_slashes(
          counterpart->uas_namespace);

        if (!uas.empty()) {
          required.insert(
            uas);
        }
      }
    }

    for (
      const auto & uas :
      required)
    {
      ensure_drone_context_locked(
        uas);
    }

    for (
      auto it =
        drones_.begin();
      it !=
      drones_.end();)
    {
      if (
        required.count(
          it->first) == 0U)
      {
        positions_.erase(
          it->first);

        it =
          drones_.erase(it);
      } else {
        ++it;
      }
    }
  }


  void available_callback(
    const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex>
      lock(mutex_);

    available_.clear();

    for (
      const auto & trajectory :
      message->trajectories)
    {
      if (
        trim_slashes(
          trajectory.flight_zone_id) ==
        flight_zone_id_ &&
        !trajectory.trajectory_id.empty())
      {
        available_[
          trajectory.trajectory_id] =
          trajectory;
      }
    }

    latest_header_ =
      message->header;

    rebuild_required_drones_locked();
  }


  void supervised_callback(
    const DetectedCollisionTrajectoryArray::SharedPtr
      message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex>
      lock(mutex_);

    supervised_.clear();

    for (
      const auto & detected :
      message->trajectories)
    {
      if (
        trim_slashes(
          detected.trajectory.flight_zone_id) ==
        flight_zone_id_ &&
        !detected.trajectory.trajectory_id.empty())
      {
        supervised_[
          detected.trajectory.trajectory_id] =
          detected;
      }
    }

    latest_header_ =
      message->header;

    rebuild_required_drones_locked();
  }


  void active_callback(
    const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex>
      lock(mutex_);

    active_.clear();

    for (
      const auto & trajectory :
      message->trajectories)
    {
      if (
        trim_slashes(
          trajectory.flight_zone_id) ==
        flight_zone_id_ &&
        !trajectory.trajectory_id.empty())
      {
        active_[
          trajectory.trajectory_id] =
          trajectory;
      }
    }

    latest_header_ =
      message->header;

    rebuild_required_drones_locked();

    std::set<std::string>
      live_occurrences;

    for (
      const auto & [_, trajectory] :
      active_)
    {
      live_occurrences.insert(
        occurrence_key(
          trajectory));
    }

    for (
      auto it =
        execute_dispatched_.begin();
      it !=
      execute_dispatched_.end();)
    {
      if (
        live_occurrences.count(
          *it) == 0U)
      {
        it =
          execute_dispatched_.erase(
          it);
      } else {
        ++it;
      }
    }
  }


  void related_distance_callback(
    const UasRelatedDistanceArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex>
      lock(mutex_);

    latest_related_distance_ =
      *message;

    latest_related_distance_received_ =
      true;

    latest_related_distance_at_ =
      std::chrono::steady_clock::now();

    // Refresh the latest known positions for UAS in this flight-zone.
    //
    // The distance_control_node already guarantees that the two positions
    // of a valid pair are expressed in the same frame.
    for (
      const auto & pair :
      message->pairs)
    {
      if (
        trim_slashes(
          pair.first_flight_zone_id) ==
        flight_zone_id_)
      {
        PositionSample sample;
        sample.header =
          pair.header;
        sample.position =
          pair.first_position;
        sample.received_at =
          latest_related_distance_at_;

        positions_[
          trim_slashes(
            pair.first_uas_namespace)] =
          sample;
      }

      if (
        trim_slashes(
          pair.second_flight_zone_id) ==
        flight_zone_id_)
      {
        PositionSample sample;
        sample.header =
          pair.header;
        sample.position =
          pair.second_position;
        sample.received_at =
          latest_related_distance_at_;

        positions_[
          trim_slashes(
            pair.second_uas_namespace)] =
          sample;
      }
    }
  }


  static std::string occurrence_key(
    const StaticTrajectory & trajectory)
  {
    std::ostringstream stream;

    stream
      << trajectory.trajectory_id
      << "@"
      << trajectory.operation_start_utc.sec
      << ":"
      << trajectory.operation_start_utc.nanosec;

    return stream.str();
  }


  static bool tracked_runtime_command(
    uint8_t command)
  {
    return
      command ==
      SupervisionControl::Goal::PAUSE ||
      command ==
      SupervisionControl::Goal::RESUME ||
      command ==
      SupervisionControl::Goal::ELEVATE ||
      command ==
      SupervisionControl::Goal::DESCEND ||
      command ==
      SupervisionControl::Goal::STOP;
  }


  static const char * command_name(
    int command)
  {
    if (
      command ==
      static_cast<int>(
        SupervisionControl::Goal::EXECUTE))
    {
      return "EXECUTE";
    }

    if (
      command ==
      static_cast<int>(
        SupervisionControl::Goal::PAUSE))
    {
      return "PAUSE";
    }

    if (
      command ==
      static_cast<int>(
        SupervisionControl::Goal::RESUME))
    {
      return "RESUME";
    }

    if (
      command ==
      static_cast<int>(
        SupervisionControl::Goal::ELEVATE))
    {
      return "ELEVATE";
    }

    if (
      command ==
      static_cast<int>(
        SupervisionControl::Goal::DESCEND))
    {
      return "DESCEND";
    }

    if (
      command ==
      static_cast<int>(
        SupervisionControl::Goal::STOP))
    {
      return "STOP";
    }

    return "NONE";
  }


  void send_action_locked(
    const std::string & uas_namespace,
    uint8_t command,
    const StaticTrajectory * trajectory,
    const std::string & reason,
    const std::string & occurrence = "")
  {
    ensure_drone_context_locked(
      uas_namespace);

    auto context_it =
      drones_.find(
      trim_slashes(
        uas_namespace));

    if (
      context_it ==
      drones_.end() ||
      !context_it->second.action_client)
    {
      return;
    }

    auto client =
      context_it->second.action_client;

    if (
      !client->wait_for_action_server(
        std::chrono::milliseconds(
          action_wait_timeout_ms_)))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[/%s/%s] supervision control action "
        "'%s' is not available",
        flight_zone_id_.c_str(),
        uas_namespace.c_str(),
        context_it->second.action_name.c_str());

      return;
    }

    SupervisionControl::Goal goal;
    goal.command =
      command;
    goal.reason =
      reason;

    if (trajectory != nullptr) {
      goal.trajectories.header =
        latest_header_;

      goal.trajectories.header.stamp =
        now();

      goal.trajectories.trajectories.push_back(
        *trajectory);
    }

    rclcpp_action::Client<
      SupervisionControl>::SendGoalOptions
      options;

    options.goal_response_callback =
      [this, uas_namespace, command, occurrence](
      const GoalHandleSupervisionControl::SharedPtr
        goal_handle)
      {
        if (goal_handle) {
          RCLCPP_INFO(
            get_logger(),
            "[/%s/%s] supervision action "
            "command=%u (%s) accepted by "
            "action server",
            flight_zone_id_.c_str(),
            uas_namespace.c_str(),
            static_cast<unsigned>(
              command),
            command_name(
              static_cast<int>(
                command)));

          return;
        }

        RCLCPP_ERROR(
          get_logger(),
          "[/%s/%s] supervision action "
          "command=%u (%s) rejected by "
          "action server",
          flight_zone_id_.c_str(),
          uas_namespace.c_str(),
          static_cast<unsigned>(
            command),
          command_name(
            static_cast<int>(
              command)));

        std::lock_guard<std::mutex>
          lock(mutex_);

        if (
          command ==
          SupervisionControl::Goal::EXECUTE &&
          !occurrence.empty())
        {
          execute_dispatched_.erase(
            occurrence);
          return;
        }

        if (
          tracked_runtime_command(
            command))
        {
          const auto it =
            drones_.find(
            trim_slashes(
              uas_namespace));

          if (
            it !=
            drones_.end())
          {
            if (
              it->second.last_control_command ==
              static_cast<int>(
                command))
            {
              it->second.last_control_command =
                -1;
            }

            it->second.control_command_in_flight =
              false;
          }
        }
      };

    options.result_callback =
      [this, uas_namespace, command, occurrence](
      const GoalHandleSupervisionControl::
        WrappedResult & result)
      {
        const bool success =
          result.code ==
          rclcpp_action::ResultCode::SUCCEEDED &&
          result.result &&
          result.result->accepted;

        std::lock_guard<std::mutex>
          lock(mutex_);

        if (success) {
          if (
            tracked_runtime_command(
              command))
          {
            const auto it =
              drones_.find(
              trim_slashes(
                uas_namespace));

            if (
              it !=
              drones_.end())
            {
              it->second.
                last_confirmed_control_command =
                static_cast<int>(
                command);

              it->second.
                control_command_in_flight =
                false;
            }
          }

          RCLCPP_INFO(
            get_logger(),
            "[/%s/%s] supervision command=%u "
            "(%s) positively acknowledged",
            flight_zone_id_.c_str(),
            uas_namespace.c_str(),
            static_cast<unsigned>(
              command),
            command_name(
              static_cast<int>(
                command)));

          return;
        }

        RCLCPP_WARN(
          get_logger(),
          "[/%s/%s] supervision action "
          "command=%u (%s) did not complete "
          "successfully",
          flight_zone_id_.c_str(),
          uas_namespace.c_str(),
          static_cast<unsigned>(
            command),
          command_name(
            static_cast<int>(
              command)));

        if (
          command ==
          SupervisionControl::Goal::EXECUTE &&
          !occurrence.empty())
        {
          execute_dispatched_.erase(
            occurrence);
          return;
        }

        if (
          tracked_runtime_command(
            command))
        {
          const auto it =
            drones_.find(
            trim_slashes(
              uas_namespace));

          if (
            it !=
            drones_.end())
          {
            if (
              it->second.last_control_command ==
              static_cast<int>(
                command))
            {
              it->second.last_control_command =
                -1;
            }

            it->second.
              control_command_in_flight =
              false;
          }
        }
      };

    client->async_send_goal(
      goal,
      options);

    if (
      command ==
      SupervisionControl::Goal::EXECUTE &&
      !occurrence.empty())
    {
      execute_dispatched_.insert(
        occurrence);
    } else if (
      tracked_runtime_command(
        command))
    {
      context_it->second.last_control_command =
        static_cast<int>(
        command);

      context_it->second.control_command_in_flight =
        true;
    }
  }


  void ensure_command_locked(
    const std::string & uas_namespace,
    uint8_t command,
    const std::string & reason)
  {
    ensure_drone_context_locked(
      uas_namespace);

    const auto it =
      drones_.find(
      trim_slashes(
        uas_namespace));

    if (
      it ==
      drones_.end())
    {
      return;
    }

    if (
      it->second.last_control_command ==
      static_cast<int>(
        command) &&
      (
        it->second.control_command_in_flight ||
        it->second.
          last_confirmed_control_command ==
        static_cast<int>(
          command)))
    {
      return;
    }

    send_action_locked(
      uas_namespace,
      command,
      nullptr,
      reason);
  }


  bool command_confirmed_locked(
    const std::string & uas_namespace,
    uint8_t command) const
  {
    const auto it =
      drones_.find(
      trim_slashes(
        uas_namespace));

    if (
      it ==
      drones_.end())
    {
      return false;
    }

    return
      it->second.
        last_confirmed_control_command ==
      static_cast<int>(
        command);
  }


  bool related_distance_snapshot_fresh_locked() const
  {
    if (
      !latest_related_distance_received_)
    {
      return false;
    }

    const double age_s =
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() -
      latest_related_distance_at_).count();

    return
      age_s <=
      related_distance_timeout_s_;
  }


  bool fresh_position_locked(
    const std::string & uas_namespace,
    PositionSample & output) const
  {
    const auto it =
      positions_.find(
      trim_slashes(
        uas_namespace));

    if (
      it ==
      positions_.end())
    {
      return false;
    }

    const double age_s =
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() -
      it->second.received_at).count();

    if (
      age_s >
      related_distance_timeout_s_)
    {
      return false;
    }

    output =
      it->second;

    return true;
  }


  bool find_related_pair_locked(
    const std::string & supervised_uas,
    const std::string & counterpart_uas,
    RelatedPairView & output) const
  {
    // Avoid aggregate braced initialization here. In ROS 2 Humble,
    // std_msgs::msg::Header and geometry_msgs::msg::Point expose explicit
    // default constructors, which makes `RelatedPairView{}` trigger
    // -Wpedantic/-Wextra conversion warnings. Default-initialize a local
    // object instead and assign it normally.
    RelatedPairView reset;
    output = reset;

    if (
      !related_distance_snapshot_fresh_locked())
    {
      return false;
    }

    const std::string supervised =
      trim_slashes(
      supervised_uas);

    const std::string counterpart =
      trim_slashes(
      counterpart_uas);

    for (
      const auto & pair :
      latest_related_distance_.pairs)
    {
      const std::string first_zone =
        trim_slashes(
        pair.first_flight_zone_id);

      const std::string second_zone =
        trim_slashes(
        pair.second_flight_zone_id);

      const std::string first_uas =
        trim_slashes(
        pair.first_uas_namespace);

      const std::string second_uas =
        trim_slashes(
        pair.second_uas_namespace);

      if (
        first_zone ==
        flight_zone_id_ &&
        second_zone ==
        flight_zone_id_ &&
        first_uas ==
        supervised &&
        second_uas ==
        counterpart)
      {
        output.valid =
          true;
        output.header =
          pair.header;
        output.supervised_position =
          pair.first_position;
        output.counterpart_position =
          pair.second_position;
        output.distance_xy_m =
          pair.distance_xy_m;

        return
          std::isfinite(
          output.distance_xy_m);
      }

      if (
        first_zone ==
        flight_zone_id_ &&
        second_zone ==
        flight_zone_id_ &&
        first_uas ==
        counterpart &&
        second_uas ==
        supervised)
      {
        output.valid =
          true;
        output.header =
          pair.header;
        output.supervised_position =
          pair.second_position;
        output.counterpart_position =
          pair.first_position;
        output.distance_xy_m =
          pair.distance_xy_m;

        return
          std::isfinite(
          output.distance_xy_m);
      }
    }

    return false;
  }


  static const char * pair_mode_name(
    PairMode mode)
  {
    switch (mode) {
      case PairMode::MONITORING:
        return "MONITORING";

      case PairMode::WAITING_BOTH_PAUSE:
        return "WAITING_BOTH_PAUSE";

      case PairMode::WAITING_ELEVATE:
        return "WAITING_ELEVATE";

      case PairMode::WAITING_COUNTERPART_RESUME:
        return "WAITING_COUNTERPART_RESUME";

      case PairMode::COUNTERPART_RUNNING:
        return "COUNTERPART_RUNNING";

      case PairMode::WAITING_DESCEND:
        return "WAITING_DESCEND";

      case PairMode::WAITING_SUPERVISED_RESUME:
        return "WAITING_SUPERVISED_RESUME";

      case PairMode::CLEARED:
        return "CLEARED";

      default:
        return "UNKNOWN";
    }
  }


  void dispatch_active_trajectories_locked()
  {
    for (
      const auto & [trajectory_id, active] :
      active_)
    {
      const std::string key =
        occurrence_key(
        active);

      if (
        execute_dispatched_.count(
          key) != 0U)
      {
        continue;
      }

      const auto supervised_it =
        supervised_.find(
        trajectory_id);

      if (
        supervised_it ==
        supervised_.end())
      {
        send_action_locked(
          active.uas_namespace,
          SupervisionControl::Goal::EXECUTE,
          &active,
          "active non-supervised trajectory",
          key);

        continue;
      }

      // A supervised trajectory is executed using the ORIGINAL full
      // trajectory retained in /supervised_trajectories. The cropped
      // version only exists in the planning/available flow.
      send_action_locked(
        supervised_it->second.
          trajectory.uas_namespace,
        SupervisionControl::Goal::EXECUTE,
        &supervised_it->second.trajectory,
        "active supervised trajectory: "
        "execute original full geometry",
        key);
    }
  }


  std::set<std::string>
  counterpart_ids_for(
    const DetectedCollisionTrajectory &
      detected) const
  {
    std::set<std::string> ids(
      detected.conflicting_trajectory_ids.begin(),
      detected.conflicting_trajectory_ids.end());

    for (
      const auto & node :
      detected.collision_nodes)
    {
      ids.insert(
        node.conflicting_trajectory_ids.begin(),
        node.conflicting_trajectory_ids.end());
    }

    for (
      const auto & segment :
      detected.collision_segments)
    {
      ids.insert(
        segment.conflicting_trajectory_ids.begin(),
        segment.conflicting_trajectory_ids.end());
    }

    ids.erase(
      detected.trajectory.trajectory_id);

    return ids;
  }


  void evaluate_supervised_pairs_locked()
  {
    std::map<std::string, PairState>
      next_pairs;

    for (
      const auto & [supervised_id, detected] :
      supervised_)
    {
      const auto supervised_active_it =
        active_.find(
        supervised_id);

      if (
        supervised_active_it ==
        active_.end())
      {
        continue;
      }

      for (
        const auto & counterpart_id :
        counterpart_ids_for(
          detected))
      {
        const auto counterpart_active_it =
          active_.find(
          counterpart_id);

        if (
          counterpart_active_it ==
          active_.end())
        {
          continue;
        }

        // Same semantic restriction as the previous implementation:
        // one supervised trajectory versus one non-supervised active
        // counterpart.
        if (
          supervised_.find(
            counterpart_id) !=
          supervised_.end())
        {
          continue;
        }

        const std::string supervised_uas =
          trim_slashes(
          detected.trajectory.uas_namespace);

        const std::string counterpart_uas =
          trim_slashes(
          counterpart_active_it->second.
            uas_namespace);

        if (
          supervised_uas.empty() ||
          counterpart_uas.empty() ||
          supervised_uas ==
          counterpart_uas)
        {
          continue;
        }

        ensure_drone_context_locked(
          supervised_uas);

        ensure_drone_context_locked(
          counterpart_uas);

        const std::string pair_key =
          supervised_id +
          "|" +
          counterpart_id;

        PairState state;

        const auto previous_it =
          pair_states_.find(
          pair_key);

        if (
          previous_it !=
          pair_states_.end())
        {
          state =
            previous_it->second;
        }

        state.supervised_trajectory_id =
          supervised_id;

        state.counterpart_trajectory_id =
          counterpart_id;

        state.supervised_uas =
          supervised_uas;

        state.counterpart_uas =
          counterpart_uas;

        state.distance_valid =
          false;

        state.distance_xy_m =
          std::numeric_limits<double>::
          infinity();

        RelatedPairView related;

        const bool distance_valid =
          find_related_pair_locked(
          supervised_uas,
          counterpart_uas,
          related);

        if (distance_valid) {
          state.distance_valid =
            true;

          state.distance_xy_m =
            related.distance_xy_m;
        }

        // ----------------------------------------------------------
        // 0. MONITORING
        // ----------------------------------------------------------
        if (
          state.mode ==
          PairMode::MONITORING)
        {
          if (
            !distance_valid ||
            state.distance_xy_m >=
            pause_distance_xy_m_)
          {
            next_pairs[
              pair_key] =
              state;

            continue;
          }

          state.reference_altitude_valid =
            true;

          state.reference_z_m =
            related.supervised_position.z;

          state.reference_frame =
            trim_slashes(
            related.header.frame_id);

          state.mode =
            PairMode::WAITING_BOTH_PAUSE;

          RCLCPP_WARN(
            get_logger(),
            "[/%s] XY deconfliction triggered "
            "for '%s' <-> '%s' | "
            "distance_xy=%.3f m < %.3f m | "
            "PAUSE supervised UAS '%s' and "
            "counterpart UAS '%s' | "
            "reference_z=%.3f m | "
            "source='%s'",
            flight_zone_id_.c_str(),
            supervised_id.c_str(),
            counterpart_id.c_str(),
            state.distance_xy_m,
            pause_distance_xy_m_,
            supervised_uas.c_str(),
            counterpart_uas.c_str(),
            state.reference_z_m,
            related_distance_topic_.c_str());

          next_pairs[
            pair_key] =
            state;

          continue;
        }

        // ----------------------------------------------------------
        // 1. PAUSE BOTH UAS
        // ----------------------------------------------------------
        if (
          state.mode ==
          PairMode::WAITING_BOTH_PAUSE)
        {
          ensure_command_locked(
            supervised_uas,
            SupervisionControl::Goal::PAUSE,
            "XY separation below pause threshold: "
            "pause supervised UAS before "
            "vertical deconfliction");

          ensure_command_locked(
            counterpart_uas,
            SupervisionControl::Goal::PAUSE,
            "XY separation below pause threshold: "
            "pause non-supervised counterpart "
            "before vertical deconfliction");

          const bool supervised_paused =
            command_confirmed_locked(
            supervised_uas,
            SupervisionControl::Goal::PAUSE);

          const bool counterpart_paused =
            command_confirmed_locked(
            counterpart_uas,
            SupervisionControl::Goal::PAUSE);

          if (
            !supervised_paused ||
            !counterpart_paused)
          {
            next_pairs[
              pair_key] =
              state;

            continue;
          }

          state.mode =
            PairMode::WAITING_ELEVATE;

          RCLCPP_WARN(
            get_logger(),
            "[/%s] Both UAS PAUSE commands "
            "acknowledged for pair '%s' <-> '%s'. "
            "Command ELEVATE=%u to supervised "
            "UAS '%s' (+%.3f m grid step).",
            flight_zone_id_.c_str(),
            supervised_id.c_str(),
            counterpart_id.c_str(),
            static_cast<unsigned>(
              SupervisionControl::Goal::ELEVATE),
            supervised_uas.c_str(),
            vertical_grid_step_m_);

          next_pairs[
            pair_key] =
            state;

          continue;
        }

        // ----------------------------------------------------------
        // 2. ELEVATE SUPERVISED UAS
        //
        // Action result acknowledges the command. Physical completion
        // is independently verified using the supervised endpoint
        // position already supplied in /uas_related_distance.
        // ----------------------------------------------------------
        if (
          state.mode ==
          PairMode::WAITING_ELEVATE)
        {
          ensure_command_locked(
            supervised_uas,
            SupervisionControl::Goal::ELEVATE,
            "vertical deconfliction: raise "
            "supervised UAS one grid step while "
            "mission remains paused");

          const bool elevate_ack =
            command_confirmed_locked(
            supervised_uas,
            SupervisionControl::Goal::ELEVATE);

          bool elevated =
            false;

          if (
            elevate_ack &&
            state.reference_altitude_valid &&
            distance_valid &&
            trim_slashes(
              related.header.frame_id) ==
            state.reference_frame)
          {
            const double target_z =
              state.reference_z_m +
              vertical_grid_step_m_;

            elevated =
              related.supervised_position.z >=
              target_z -
              vertical_position_tolerance_m_;
          }

          if (!elevated) {
            next_pairs[
              pair_key] =
              state;

            continue;
          }

          state.mode =
            PairMode::
            WAITING_COUNTERPART_RESUME;

          RCLCPP_INFO(
            get_logger(),
            "[/%s] Supervised UAS '%s' elevated "
            "| z=%.3f m | reference_z=%.3f m | "
            "step=%.3f m. RESUME counterpart "
            "UAS '%s'.",
            flight_zone_id_.c_str(),
            supervised_uas.c_str(),
            related.supervised_position.z,
            state.reference_z_m,
            vertical_grid_step_m_,
            counterpart_uas.c_str());

          next_pairs[
            pair_key] =
            state;

          continue;
        }

        // ----------------------------------------------------------
        // 3. RESUME NON-SUPERVISED COUNTERPART
        // ----------------------------------------------------------
        if (
          state.mode ==
          PairMode::
          WAITING_COUNTERPART_RESUME)
        {
          ensure_command_locked(
            counterpart_uas,
            SupervisionControl::Goal::RESUME,
            "supervised UAS is elevated: "
            "resume non-supervised counterpart");

          if (
            !command_confirmed_locked(
              counterpart_uas,
              SupervisionControl::Goal::RESUME))
          {
            next_pairs[
              pair_key] =
              state;

            continue;
          }

          state.mode =
            PairMode::COUNTERPART_RUNNING;

          RCLCPP_INFO(
            get_logger(),
            "[/%s] Counterpart UAS '%s' RESUME "
            "acknowledged for pair '%s' <-> '%s'. "
            "Monitor /uas_related_distance until "
            "distance_xy > %.3f m.",
            flight_zone_id_.c_str(),
            counterpart_uas.c_str(),
            supervised_id.c_str(),
            counterpart_id.c_str(),
            release_distance_xy_m_);

          next_pairs[
            pair_key] =
            state;

          continue;
        }

        // ----------------------------------------------------------
        // 4. MONITOR RUNNING COUNTERPART
        // ----------------------------------------------------------
        if (
          state.mode ==
          PairMode::COUNTERPART_RUNNING)
        {
          if (!distance_valid) {
            next_pairs[
              pair_key] =
              state;

            continue;
          }

          if (
            state.distance_xy_m <=
            release_distance_xy_m_)
          {
            next_pairs[
              pair_key] =
              state;

            continue;
          }

          state.mode =
            PairMode::WAITING_DESCEND;

          RCLCPP_INFO(
            get_logger(),
            "[/%s] Counterpart '%s' cleared "
            "pair '%s' | distance_xy=%.3f m > "
            "%.3f m. Command DESCEND=%u to "
            "supervised UAS '%s'.",
            flight_zone_id_.c_str(),
            counterpart_id.c_str(),
            pair_key.c_str(),
            state.distance_xy_m,
            release_distance_xy_m_,
            static_cast<unsigned>(
              SupervisionControl::Goal::DESCEND),
            supervised_uas.c_str());

          next_pairs[
            pair_key] =
            state;

          continue;
        }

        // ----------------------------------------------------------
        // 5. DESCEND SUPERVISED UAS
        // ----------------------------------------------------------
        if (
          state.mode ==
          PairMode::WAITING_DESCEND)
        {
          ensure_command_locked(
            supervised_uas,
            SupervisionControl::Goal::DESCEND,
            "counterpart XY separation is clear: "
            "return supervised UAS to its "
            "pre-elevation altitude");

          const bool descend_ack =
            command_confirmed_locked(
            supervised_uas,
            SupervisionControl::Goal::DESCEND);

          bool restored =
            false;

          if (
            descend_ack &&
            state.reference_altitude_valid &&
            distance_valid &&
            trim_slashes(
              related.header.frame_id) ==
            state.reference_frame)
          {
            restored =
              std::abs(
              related.supervised_position.z -
              state.reference_z_m) <=
              vertical_position_tolerance_m_;
          }

          if (!restored) {
            next_pairs[
              pair_key] =
              state;

            continue;
          }

          state.mode =
            PairMode::
            WAITING_SUPERVISED_RESUME;

          RCLCPP_INFO(
            get_logger(),
            "[/%s] Supervised UAS '%s' restored "
            "to previous altitude | z=%.3f m | "
            "reference_z=%.3f m. "
            "RESUME supervised trajectory.",
            flight_zone_id_.c_str(),
            supervised_uas.c_str(),
            related.supervised_position.z,
            state.reference_z_m);

          next_pairs[
            pair_key] =
            state;

          continue;
        }

        // ----------------------------------------------------------
        // 6. RESUME SUPERVISED TRAJECTORY
        // ----------------------------------------------------------
        if (
          state.mode ==
          PairMode::
          WAITING_SUPERVISED_RESUME)
        {
          ensure_command_locked(
            supervised_uas,
            SupervisionControl::Goal::RESUME,
            "vertical deconfliction complete: "
            "resume supervised trajectory");

          if (
            !command_confirmed_locked(
              supervised_uas,
              SupervisionControl::Goal::RESUME))
          {
            next_pairs[
              pair_key] =
              state;

            continue;
          }

          state.mode =
            PairMode::CLEARED;

          RCLCPP_INFO(
            get_logger(),
            "[/%s] Pair '%s' <-> '%s' "
            "deconflicted completely. "
            "Supervised UAS resumed; no further "
            "supervision is applied to this pair.",
            flight_zone_id_.c_str(),
            supervised_id.c_str(),
            counterpart_id.c_str());

          next_pairs[
            pair_key] =
            state;

          continue;
        }

        // ----------------------------------------------------------
        // 7. CLEARED
        // ----------------------------------------------------------
        if (
          state.mode ==
          PairMode::CLEARED)
        {
          next_pairs[
            pair_key] =
            state;

          continue;
        }
      }
    }

    pair_states_ =
      std::move(
      next_pairs);
  }


  static void append_segment_edges(
    std::vector<geometry_msgs::msg::Point> & out,
    const TrajectorySegment & segment)
  {
    const std::size_t count =
      point_count(
      segment);

    if (count < 2U) {
      return;
    }

    for (
      std::size_t i = 1U;
      i < count;
      ++i)
    {
      out.push_back(
        point_at(
          segment,
          i - 1U));

      out.push_back(
        point_at(
          segment,
          i));
    }
  }


  static std::vector<
    geometry_msgs::msg::Point>
  trajectory_line_list_points(
    const StaticTrajectory & trajectory)
  {
    std::vector<
      geometry_msgs::msg::Point>
      out;

    // Every trajectory component is an independent polyline.
    // Never invent connectors between takeoff, mission[i],
    // mission[i+1] or landing.
    append_segment_edges(
      out,
      trajectory.takeoff);

    for (
      const auto & mission :
      trajectory.mission)
    {
      append_segment_edges(
        out,
        mission);
    }

    append_segment_edges(
      out,
      trajectory.landing);

    return out;
  }


  bool uas_has_active_trajectory_locked(
    const std::string & uas_namespace,
    bool & supervised_active) const
  {
    const std::string uas =
      trim_slashes(
      uas_namespace);

    supervised_active =
      false;

    bool active_found =
      false;

    for (
      const auto & [trajectory_id, trajectory] :
      active_)
    {
      if (
        trim_slashes(
          trajectory.uas_namespace) !=
        uas)
      {
        continue;
      }

      active_found =
        true;

      if (
        supervised_.find(
          trajectory_id) !=
        supervised_.end())
      {
        supervised_active =
          true;
      }
    }

    return active_found;
  }


  std::string uas_runtime_state_locked(
    const std::string & uas_namespace) const
  {
    const std::string uas =
      trim_slashes(
      uas_namespace);

    bool supervised_active =
      false;

    const bool active =
      uas_has_active_trajectory_locked(
      uas,
      supervised_active);

    std::string state =
      active ?
      (
        supervised_active ?
        "ACTIVE_SUPERVISED" :
        "ACTIVE"
      ) :
      "SUPERVISED_WAITING";

    // Pair mode has priority because it expresses the deconfliction state
    // that the operator actually needs to see in RViz2.
    for (
      const auto & [_, pair] :
      pair_states_)
    {
      if (
        pair.supervised_uas ==
        uas)
      {
        switch (pair.mode) {
          case PairMode::MONITORING:
            state =
              "SUPERVISED_MONITORING";
            break;

          case PairMode::WAITING_BOTH_PAUSE:
            state =
              "SUPERVISED_PAUSE_PENDING";
            break;

          case PairMode::WAITING_ELEVATE:
            state =
              "SUPERVISED_PAUSED_WAIT_ELEVATE";
            break;

          case PairMode::WAITING_COUNTERPART_RESUME:
            state =
              "SUPERVISED_ELEVATED";
            break;

          case PairMode::COUNTERPART_RUNNING:
            state =
              "SUPERVISED_ELEVATED_HOLD";
            break;

          case PairMode::WAITING_DESCEND:
            state =
              "SUPERVISED_DESCEND_PENDING";
            break;

          case PairMode::WAITING_SUPERVISED_RESUME:
            state =
              "SUPERVISED_RESTORED_WAIT_RESUME";
            break;

          case PairMode::CLEARED:
            state =
              "SUPERVISED_RESUMED_CLEARED";
            break;
        }

        return state;
      }

      if (
        pair.counterpart_uas ==
        uas)
      {
        switch (pair.mode) {
          case PairMode::MONITORING:
            state =
              "COUNTERPART_MONITORING";
            break;

          case PairMode::WAITING_BOTH_PAUSE:
            state =
              "COUNTERPART_PAUSE_PENDING";
            break;

          case PairMode::WAITING_ELEVATE:
            state =
              "COUNTERPART_PAUSED";
            break;

          case PairMode::WAITING_COUNTERPART_RESUME:
            state =
              "COUNTERPART_RESUME_PENDING";
            break;

          case PairMode::COUNTERPART_RUNNING:
            state =
              "COUNTERPART_RUNNING";
            break;

          case PairMode::WAITING_DESCEND:
          case PairMode::WAITING_SUPERVISED_RESUME:
          case PairMode::CLEARED:
            state =
              "COUNTERPART_CLEAR_RUNNING";
            break;
        }

        return state;
      }
    }

    return state;
  }


  std_msgs::msg::ColorRGBA
  uas_state_color_locked(
    const std::string & uas_namespace) const
  {
    const std::string state =
      uas_runtime_state_locked(
      uas_namespace);

    if (
      state.find("PAUSE") !=
      std::string::npos)
    {
      return
        color(
        0.95F,
        0.10F,
        0.80F,
        1.0F);
    }

    if (
      state.find("ELEVATED") !=
      std::string::npos)
    {
      return
        color(
        1.0F,
        0.55F,
        0.0F,
        1.0F);
    }

    if (
      state.find("DESCEND") !=
      std::string::npos)
    {
      return
        color(
        0.75F,
        0.30F,
        1.0F,
        1.0F);
    }

    if (
      state.find("CLEARED") !=
      std::string::npos ||
      state.find("CLEAR_RUNNING") !=
      std::string::npos)
    {
      return
        color(
        0.10F,
        0.90F,
        0.20F,
        1.0F);
    }

    if (
      state.find("SUPERVISED") !=
      std::string::npos)
    {
      return
        color(
        1.0F,
        0.75F,
        0.10F,
        1.0F);
    }

    return
      color(
      0.20F,
      0.65F,
      1.0F,
      1.0F);
  }


  std_msgs::msg::ColorRGBA drone_marker_color_locked(
    const std::string & uas_namespace,
    const DroneContext & drone) const
  {
    // Runtime commands have priority over the generic ACTIVE/EXECUTE state.
    //
    // PAUSE   -> orange
    // ELEVATE -> purple
    // DESCEND -> purple
    // STOP    -> blue
    // RESUME  -> green
    if (
      drone.last_control_command ==
      static_cast<int>(
        SupervisionControl::Goal::PAUSE))
    {
      return color(
        1.0F,
        0.50F,
        0.0F,
        1.0F);
    }

    if (
      drone.last_control_command ==
        static_cast<int>(
          SupervisionControl::Goal::ELEVATE) ||
      drone.last_control_command ==
        static_cast<int>(
          SupervisionControl::Goal::DESCEND))
    {
      return color(
        0.65F,
        0.0F,
        0.85F,
        1.0F);
    }

    if (
      drone.last_control_command ==
      static_cast<int>(
        SupervisionControl::Goal::STOP))
    {
      return color(
        0.0F,
        0.25F,
        1.0F,
        1.0F);
    }

    if (
      drone.last_control_command ==
      static_cast<int>(
        SupervisionControl::Goal::RESUME))
    {
      return color(
        0.0F,
        1.0F,
        0.0F,
        1.0F);
    }

    // EXECUTE is intentionally not stored in last_control_command because it
    // is not one of the runtime commands tracked by the supervision state
    // machine. Infer it from the already existing execute_dispatched_ set.
    //
    // An ACTIVE trajectory discovered but not yet dispatched is BLUE.
    // Once EXECUTE has been dispatched for one of this UAS' active
    // occurrences, the sphere becomes GREEN.
    const std::string uas =
      trim_slashes(
      uas_namespace);

    for (
      const auto & [_, trajectory] :
      active_)
    {
      if (
        trim_slashes(
          trajectory.uas_namespace) !=
        uas)
      {
        continue;
      }

      if (
        execute_dispatched_.count(
          occurrence_key(
            trajectory)) != 0U)
      {
        return color(
          0.0F,
          1.0F,
          0.0F,
          1.0F);
      }
    }

    // Initial state after discovery in /active_trajectories, or any state
    // without an EXECUTE/RESUME/PAUSE/ELEVATE/DESCEND command yet.
    return color(
      0.0F,
      0.25F,
      1.0F,
      1.0F);
  }


  MarkerArray make_markers_locked() const
  {
    MarkerArray out;

    // Remove every marker from the previous snapshot so that UAS that are no
    // longer tracked disappear immediately from RViz2.
    Marker clear;
    clear.header.stamp =
      now();
    clear.header.frame_id =
      latest_header_.frame_id;
    clear.action =
      Marker::DELETEALL;

    out.markers.push_back(
      clear);

    int marker_id =
      1;

    // Publish exactly one sphere per tracked UAS.
    //
    // No trajectory lines, collision geometry, pair links or text labels are
    // published by supervision_node anymore.
    //
    // The current UAS position continues to come exclusively from
    // /uas_related_distance.
    for (
      const auto & [uas, drone] :
      drones_)
    {
      PositionSample position;

      if (
        !fresh_position_locked(
          uas,
          position))
      {
        continue;
      }

      Marker sphere;
      sphere.header =
        position.header;
      sphere.header.stamp =
        now();
      sphere.ns =
        "supervision/drone_states";
      sphere.id =
        marker_id++;
      sphere.type =
        Marker::SPHERE;
      sphere.action =
        Marker::ADD;
      sphere.pose.orientation.w =
        1.0;
      sphere.pose.position =
        position.position;
      sphere.scale.x =
        drone_scale_;
      sphere.scale.y =
        drone_scale_;
      sphere.scale.z =
        drone_scale_;
      sphere.color =
        drone_marker_color_locked(
        uas,
        drone);

      out.markers.push_back(
        std::move(
          sphere));
    }

    return out;
  }


  void evaluate()
  {
    std::lock_guard<std::mutex>
      lock(mutex_);

    dispatch_active_trajectories_locked();

    evaluate_supervised_pairs_locked();

    marker_publisher_->publish(
      make_markers_locked());
  }


  std::string flight_zone_id_;

  std::string available_topic_;
  std::string supervised_topic_;
  std::string active_topic_;
  std::string related_distance_topic_;

  std::string control_action_suffix_;
  std::string markers_topic_;

  double pause_distance_xy_m_{1.5};
  double release_distance_xy_m_{2.0};

  double vertical_grid_step_m_{1.0};
  double vertical_position_tolerance_m_{0.15};

  double related_distance_timeout_s_{2.0};

  int evaluation_period_ms_{100};
  int action_wait_timeout_ms_{20};

  double trajectory_line_width_{0.10};
  double collision_segment_width_{0.20};
  double collision_node_scale_{0.34};
  double drone_scale_{0.42};
  double pair_line_width_{0.06};
  double text_height_{0.30};

  mutable std::mutex mutex_;

  std_msgs::msg::Header latest_header_;

  std::map<
    std::string,
    StaticTrajectory>
    available_;

  std::map<
    std::string,
    DetectedCollisionTrajectory>
    supervised_;

  std::map<
    std::string,
    StaticTrajectory>
    active_;

  std::map<
    std::string,
    DroneContext>
    drones_;

  std::map<
    std::string,
    PairState>
    pair_states_;

  std::set<std::string>
    execute_dispatched_;

  bool latest_related_distance_received_{false};

  UasRelatedDistanceArray
    latest_related_distance_;

  std::chrono::steady_clock::time_point
    latest_related_distance_at_{};

  std::map<
    std::string,
    PositionSample>
    positions_;

  rclcpp::Subscription<
    StaticTrajectoryArray>::SharedPtr
    available_subscription_;

  rclcpp::Subscription<
    DetectedCollisionTrajectoryArray>::SharedPtr
    supervised_subscription_;

  rclcpp::Subscription<
    StaticTrajectoryArray>::SharedPtr
    active_subscription_;

  rclcpp::Subscription<
    UasRelatedDistanceArray>::SharedPtr
    related_distance_subscription_;

  rclcpp::Publisher<
    MarkerArray>::SharedPtr
    marker_publisher_;

  rclcpp::TimerBase::SharedPtr
    evaluation_timer_;
};

}  // namespace flight_zone_supervision


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);

  try {
    rclcpp::spin(
      std::make_shared<
        flight_zone_supervision::
        SupervisionNode>());
  } catch (
    const std::exception & error)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "supervision_node"),
      "Fatal error: %s",
      error.what());

    rclcpp::shutdown();

    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
