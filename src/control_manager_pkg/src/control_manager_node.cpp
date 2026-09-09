#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <controllers_pkg/action/follow_waypoints.hpp>
#include <controllers_pkg/srv/arm_takeoff.hpp>
#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>
#include <control_manager_pkg/srv/control_trajectory.hpp>
#include <flight_zone_supervision/action/supervision_control.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace control_manager_node
{

using StaticTrajectory = static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using TrajectorySegment = static_trajectory_manager::msg::TrajectorySegment;
using ArmTakeoff = controllers_pkg::srv::ArmTakeoff;
using FollowWaypoints = controllers_pkg::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
using FollowWaypointsClient = rclcpp_action::Client<FollowWaypoints>;
using ControlTrajectory = control_manager_pkg::srv::ControlTrajectory;
using SupervisionControl = flight_zone_supervision::action::SupervisionControl;
using GoalHandleSupervisionControl = rclcpp_action::ServerGoalHandle<SupervisionControl>;
using SupervisionControlServer = rclcpp_action::Server<SupervisionControl>;

constexpr int64_t kNsPerSecond = 1000000000LL;
constexpr double kEpsilon = 1.0e-9;

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class State : uint8_t
{
  WAITING_TRAJECTORY,
  WAITING_START,
  PAUSED_BEFORE_START,
  TAKEOFF_REQUEST,
  TAKEOFF_RESPONSE,
  MISSION_DISPATCH,
  MISSION_ACTIVE,
  PAUSE_CANCELING,
  PAUSED,
  VERTICAL_ELEVATE_DISPATCH,
  VERTICAL_ELEVATE_ACTIVE,
  VERTICAL_DESCEND_DISPATCH,
  VERTICAL_DESCEND_ACTIVE,
  STOP_CANCELING,
  RETURN_DISPATCH,
  RETURN_ACTIVE,
  LANDING_REQUEST,
  LANDING_RESPONSE,
  COMPLETED,
  FAILED
};

enum class ActionPurpose : uint8_t
{
  NONE,
  MISSION,
  VERTICAL_ELEVATE,
  VERTICAL_DESCEND,
  RETURN_TO_LANDING
};

enum class VerticalIntent : uint8_t
{
  NONE,
  ELEVATE,
  DESCEND
};

enum class CancelIntent : uint8_t
{
  NONE,
  PAUSE,
  STOP
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

static bool finite(double value)
{
  return std::isfinite(value);
}

static int64_t time_to_ns(const builtin_interfaces::msg::Time & value)
{
  return static_cast<int64_t>(value.sec) * kNsPerSecond +
         static_cast<int64_t>(value.nanosec);
}

static int64_t system_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

static double point_segment_distance_3d(
  const Point3 & p, const Point3 & a, const Point3 & b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double vz = b.z - a.z;
  const double wx = p.x - a.x;
  const double wy = p.y - a.y;
  const double wz = p.z - a.z;
  const double vv = vx * vx + vy * vy + vz * vz;

  if (vv <= kEpsilon) {
    const double dx = p.x - a.x;
    const double dy = p.y - a.y;
    const double dz = p.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  const double t = std::clamp((wx * vx + wy * vy + wz * vz) / vv, 0.0, 1.0);
  const double cx = a.x + t * vx;
  const double cy = a.y + t * vy;
  const double cz = a.z + t * vz;
  const double dx = p.x - cx;
  const double dy = p.y - cy;
  const double dz = p.z - cz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

class ControlManagerNode : public rclcpp::Node
{
public:
  ControlManagerNode()
  : Node("control_manager_node")
  {
    derive_identity_from_namespace();

    odom_frame_suffix_ = declare_parameter<std::string>("odom_frame_suffix", "odom");
    arm_takeoff_service_suffix_ = declare_parameter<std::string>(
      "arm_takeoff_service_suffix", "arm_takeoff");
    follow_waypoints_action_suffix_ = declare_parameter<std::string>(
      "follow_waypoints_action_suffix", "follow_waypoints");
    supervision_action_suffix_ = declare_parameter<std::string>(
      "supervision_action_suffix", "supervision_control");
    operator_service_suffix_ = declare_parameter<std::string>(
      "operator_service_suffix", "trajectory_control");

    tick_period_ms_ = declare_parameter<int>("tick_period_ms", 100);
    retry_period_ms_ = declare_parameter<int>("retry_period_ms", 500);
    service_wait_timeout_ms_ = declare_parameter<int>("service_wait_timeout_ms", 50);
    service_yaw_rad_ = declare_parameter<double>("service_yaw_rad", 0.0);
    action_wait_timeout_ms_ = declare_parameter<int>("action_wait_timeout_ms", 50);
    max_start_lateness_s_ = declare_parameter<double>("max_start_lateness_s", 5.0);
    transform_timeout_s_ = declare_parameter<double>("transform_timeout_s", 0.20);

    // Vertical grid step used by SupervisionControl::ELEVATE/DESCEND.
    // It must match supervision_node.vertical_grid_step_m.
    vertical_grid_step_m_ = declare_parameter<double>("vertical_grid_step_m", 1.0);

    max_path_deviation_m_ = declare_parameter<double>("max_path_deviation_m", 1.0);
    deviation_grace_period_s_ = declare_parameter<double>("deviation_grace_period_s", 1.0);
    deviation_hold_time_s_ = declare_parameter<double>("deviation_hold_time_s", 0.75);

    max_completed_keys_ = declare_parameter<int>("max_completed_keys", 128);

    validate_parameters();

    expected_action_name_ = "/" + drone_namespace_ + "/" + follow_waypoints_action_suffix_;
    supervision_action_name_ = "/" + drone_namespace_ + "/" + supervision_action_suffix_;
    odom_frame_ = drone_namespace_ + "/" + odom_frame_suffix_;

    arm_takeoff_client_ = create_client<ArmTakeoff>(arm_takeoff_service_suffix_);
    follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(
      this, follow_waypoints_action_suffix_);

    supervision_action_server_ = rclcpp_action::create_server<SupervisionControl>(
      this,
      supervision_action_suffix_,
      std::bind(
        &ControlManagerNode::handle_supervision_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &ControlManagerNode::handle_supervision_cancel,
        this,
        std::placeholders::_1),
      std::bind(
        &ControlManagerNode::handle_supervision_accepted,
        this,
        std::placeholders::_1));

    operator_service_ = create_service<ControlTrajectory>(
      operator_service_suffix_,
      std::bind(
        &ControlManagerNode::operator_control_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tick_timer_ = create_wall_timer(
      std::chrono::milliseconds(tick_period_ms_),
      std::bind(&ControlManagerNode::tick, this));

    next_attempt_at_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "Control manager ready | UAS='/%s' | supervision action='%s' | "
      "controller service='/%s/%s' | follow action='%s' | operator service='/%s/%s' | "
      "vertical_grid_step=%.3f m",
      drone_namespace_.c_str(),
      supervision_action_name_.c_str(),
      drone_namespace_.c_str(),
      arm_takeoff_service_suffix_.c_str(),
      expected_action_name_.c_str(),
      drone_namespace_.c_str(),
      operator_service_suffix_.c_str(),
      vertical_grid_step_m_);
  }

private:
  void derive_identity_from_namespace()
  {
    const std::string ns = trim_slashes(get_namespace());
    const auto slash = ns.find('/');
    if (slash == std::string::npos || slash == 0U || slash + 1U >= ns.size() ||
      ns.find('/', slash + 1U) != std::string::npos)
    {
      throw std::runtime_error(
        "control_manager_node must run in namespace '/<flight_zone>/<uas>', "
        "for example '/inspection_1/ua_ins_1'");
    }
    flight_zone_id_ = ns.substr(0U, slash);
    uas_namespace_ = ns.substr(slash + 1U);
    drone_namespace_ = flight_zone_id_ + "/" + uas_namespace_;
  }

  void validate_parameters() const
  {
    const auto valid_relative = [](const std::string & value) {
        return !value.empty() && value.front() != '/' && value.back() != '/' &&
               value.find("//") == std::string::npos;
      };

    if (!valid_relative(odom_frame_suffix_) ||
      !valid_relative(arm_takeoff_service_suffix_) ||
      !valid_relative(follow_waypoints_action_suffix_) ||
      !valid_relative(supervision_action_suffix_) ||
      !valid_relative(operator_service_suffix_))
    {
      throw std::runtime_error(
              "controller/action/service suffixes must be valid relative ROS names");
    }

    if (tick_period_ms_ <= 0 || retry_period_ms_ <= 0 || service_wait_timeout_ms_ < 0 ||
      action_wait_timeout_ms_ < 0 || max_start_lateness_s_ < 0.0 || transform_timeout_s_ < 0.0 ||
      !finite(service_yaw_rad_) || !finite(vertical_grid_step_m_) ||
      vertical_grid_step_m_ <= 0.0 ||
      max_path_deviation_m_ <= 0.0 || deviation_grace_period_s_ < 0.0 ||
      deviation_hold_time_s_ < 0.0 || max_completed_keys_ < 1)
    {
      throw std::runtime_error("invalid control_manager_node parameter");
    }
  }

  static bool segment_valid(const TrajectorySegment & segment, std::size_t minimum_points)
  {
    if (segment.x.size() < minimum_points || segment.x.size() != segment.y.size() ||
      segment.x.size() != segment.z.size())
    {
      return false;
    }
    for (std::size_t i = 0U; i < segment.x.size(); ++i) {
      if (!finite(segment.x[i]) || !finite(segment.y[i]) || !finite(segment.z[i])) {
        return false;
      }
    }
    return true;
  }

  bool trajectory_matches(const StaticTrajectory & trajectory) const
  {
    return trim_slashes(trajectory.flight_zone_id) == flight_zone_id_ &&
           trim_slashes(trajectory.uas_namespace) == uas_namespace_;
  }

  bool validate_trajectory(const StaticTrajectory & trajectory, std::string & reason) const
  {
    if (!trajectory_matches(trajectory)) {
      reason = "trajectory belongs to another UAS";
      return false;
    }
    if (trajectory.trajectory_id.empty() || trajectory.frame_id.empty()) {
      reason = "trajectory_id/frame_id is empty";
      return false;
    }
    if (trajectory.action_name != expected_action_name_) {
      reason = "action_name does not match this UAS: expected '" + expected_action_name_ + "'";
      return false;
    }
    if (!segment_valid(trajectory.takeoff, 2U) ||
      trajectory.mission.empty() ||
      !segment_valid(trajectory.landing, 2U))
    {
      reason = "takeoff/mission[]/landing geometry is invalid or mission[] is empty";
      return false;
    }

    for (std::size_t mission_index = 0U;
      mission_index < trajectory.mission.size();
      ++mission_index)
    {
      const auto & mission =
        trajectory.mission[mission_index];

      if (!segment_valid(mission, 2U)) {
        reason =
          "mission[" +
          std::to_string(mission_index) +
          "] geometry is invalid";
        return false;
      }

      // FollowWaypoints exposes one scalar height per action. Because each
      // mission[i] is dispatched as an independent action, different mission
      // components may use different heights, but each individual component
      // must be planar in Z.
      const double mission_height =
        mission.z.front();

      if (mission_height <= 0.0) {
        reason =
          "mission[" +
          std::to_string(mission_index) +
          "] height must be positive";
        return false;
      }

      for (const double z : mission.z) {
        if (std::abs(z - mission_height) > 1.0e-6) {
          reason =
            "FollowWaypoints supports one scalar height per action; mission[" +
            std::to_string(mission_index) +
            "].z must be constant";
          return false;
        }
      }
    }

    if (!finite(trajectory.goal_tolerance) || trajectory.goal_tolerance <= 0.0 ||
      !finite(trajectory.slowdown_radius) || trajectory.slowdown_radius <= 0.0 ||
      trajectory.repetitions < 1U)
    {
      reason = "goal_tolerance, slowdown_radius or repetitions is invalid";
      return false;
    }
    if (time_to_ns(trajectory.operation_end_utc) <= time_to_ns(trajectory.operation_start_utc)) {
      reason = "operation_end_utc must be later than operation_start_utc";
      return false;
    }
    return true;
  }

  static std::string occurrence_key(const StaticTrajectory & trajectory)
  {
    return trajectory.trajectory_id + "@" + std::to_string(time_to_ns(trajectory.operation_start_utc));
  }

  bool already_completed(const std::string & key) const
  {
    return completed_keys_set_.find(key) != completed_keys_set_.end();
  }

  void remember_completed(const std::string & key)
  {
    if (key.empty() || completed_keys_set_.find(key) != completed_keys_set_.end()) {
      return;
    }
    completed_keys_.push_back(key);
    completed_keys_set_.insert(key);
    while (static_cast<int>(completed_keys_.size()) > max_completed_keys_) {
      completed_keys_set_.erase(completed_keys_.front());
      completed_keys_.pop_front();
    }
  }

  rclcpp_action::GoalResponse handle_supervision_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const SupervisionControl::Goal> goal)
  {
    if (!goal) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    switch (goal->command) {
      case SupervisionControl::Goal::EXECUTE:
      case SupervisionControl::Goal::PAUSE:
      case SupervisionControl::Goal::RESUME:
      case SupervisionControl::Goal::ELEVATE:
      case SupervisionControl::Goal::DESCEND:
      case SupervisionControl::Goal::STOP:
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

      default:
        RCLCPP_WARN(
          get_logger(),
          "[/%s] Rejecting unknown supervision action command=%u",
          drone_namespace_.c_str(),
          static_cast<unsigned>(goal->command));
        return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handle_supervision_cancel(
    const std::shared_ptr<GoalHandleSupervisionControl>)
  {
    // SupervisionControl goals are short command transactions. Flight/action
    // cancellation is requested explicitly through PAUSE or STOP. ELEVATE and
    // DESCEND are acquired here and executed asynchronously by this node.
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_supervision_accepted(
    const std::shared_ptr<GoalHandleSupervisionControl> goal_handle)
  {
    if (!goal_handle) {
      return;
    }

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<SupervisionControl::Result>();
    auto feedback = std::make_shared<SupervisionControl::Feedback>();

    if (!goal) {
      result->accepted = false;
      result->message = "null supervision goal";
      result->state = state_name(state_);
      goal_handle->succeed(result);
      return;
    }

    std::string message;
    bool accepted = false;

    switch (goal->command) {
      case SupervisionControl::Goal::EXECUTE:
        accepted = accept_execute_command(goal->trajectories, message);
        break;

      case SupervisionControl::Goal::PAUSE:
        accepted = request_supervision_pause(message);
        break;

      case SupervisionControl::Goal::RESUME:
        accepted = request_supervision_resume(message);
        break;

      case SupervisionControl::Goal::ELEVATE:
        accepted = request_supervision_elevate(message);
        break;

      case SupervisionControl::Goal::DESCEND:
        accepted = request_supervision_descend(message);
        break;

      case SupervisionControl::Goal::STOP:
        accepted = request_supervision_stop(message);
        break;

      default:
        message = "unknown command";
        break;
    }

    feedback->state = state_name(state_);
    goal_handle->publish_feedback(feedback);

    result->accepted = accepted;
    result->message = message;
    result->state = state_name(state_);

    if (accepted) {
      RCLCPP_INFO(
        get_logger(),
        "[/%s] Supervision command=%u accepted | state=%s | reason='%s'",
        drone_namespace_.c_str(),
        static_cast<unsigned>(goal->command),
        state_name(state_),
        goal->reason.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "[/%s] Supervision command=%u rejected | state=%s | %s",
        drone_namespace_.c_str(),
        static_cast<unsigned>(goal->command),
        state_name(state_),
        message.c_str());
    }

    // The action acknowledges command acquisition. Mission execution continues
    // asynchronously in this node's state machine.
    goal_handle->succeed(result);
  }

  bool accept_execute_command(
    const StaticTrajectoryArray & trajectories,
    std::string & message)
  {
    if (trajectories.trajectories.size() != 1U) {
      message =
        "EXECUTE requires exactly one StaticTrajectory in trajectories[]";
      return false;
    }

    const auto & trajectory =
      trajectories.trajectories.front();

    std::string reason;
    if (!validate_trajectory(trajectory, reason)) {
      message =
        "invalid EXECUTE trajectory: " +
        reason;
      return false;
    }

    const std::string key =
      occurrence_key(trajectory);

    if (already_completed(key)) {
      message =
        "trajectory occurrence was already completed; command treated as idempotent";
      return true;
    }

    if (
      state_ == State::COMPLETED ||
      state_ == State::FAILED)
    {
      clear_plan();
    }

    if (active_trajectory_) {
      if (active_occurrence_key_ == key) {
        // While the occurrence has not started, allow an idempotent EXECUTE to
        // refresh the exact trajectory definition.
        if (
          state_ == State::WAITING_START ||
          state_ == State::PAUSED_BEFORE_START)
        {
          active_trajectory_ =
            trajectory;
          initialize_mission_execution();
        }

        message =
          "trajectory occurrence is already stored/executing";
        return true;
      }

      message =
        "control manager is busy with occurrence '" +
        active_occurrence_key_ +
        "'";
      return false;
    }

    if (state_ != State::WAITING_TRAJECTORY) {
      message =
        std::string(
        "cannot acquire a new trajectory while state=") +
        state_name(state_);
      return false;
    }

    active_trajectory_ =
      trajectory;

    active_occurrence_key_ =
      key;

    initialize_mission_execution();

    state_ =
      State::WAITING_START;

    if (preemptive_pause_latch_) {
      preemptive_pause_latch_ = false;
      supervision_start_delay_authorized_ = true;
      state_ =
        State::PAUSED_BEFORE_START;
    }

    next_attempt_at_ =
      std::chrono::steady_clock::now();

    const std::string conflict_detail =
      active_trajectory_->spacial_conflict ?
      " | conflict_with='" +
      active_trajectory_->
      spacial_conflicting_trajectory_id +
      "'" :
      "";

    RCLCPP_INFO(
      get_logger(),
      "[/%s] Acquired trajectory by supervision action '%s' | P%d | "
      "start_ns=%ld | missions=%zu | repetitions=%u | total_mission_points=%zu | "
      "spacial_conflict=%s%s",
      drone_namespace_.c_str(),
      active_trajectory_->trajectory_id.c_str(),
      active_trajectory_->priority,
      static_cast<long>(
        time_to_ns(
          active_trajectory_->
          operation_start_utc)),
      active_trajectory_->mission.size(),
      active_trajectory_->repetitions,
      total_mission_waypoints(*active_trajectory_),
      active_trajectory_->spacial_conflict ?
      "true" :
      "false",
      conflict_detail.c_str());

    message =
      "trajectory occurrence acquired";
    return true;
  }

  bool request_supervision_pause(
    std::string & message)
  {
    if (!active_trajectory_) {
      preemptive_pause_latch_ = true;
      message =
        "pause latched; the next EXECUTE occurrence will be acquired in PAUSED_BEFORE_START";
      return true;
    }

    switch (state_) {
      case State::WAITING_START:
      case State::TAKEOFF_REQUEST:
        pending_pause_request_ = false;
        supervision_start_delay_authorized_ = true;
        state_ =
          State::PAUSED_BEFORE_START;
        message =
          "paused before takeoff/start";
        return true;

      case State::TAKEOFF_RESPONSE:
        pending_pause_request_ = true;
        message =
          "pause queued; takeoff command is already in flight and mission will remain HOLD afterward";
        return true;

      case State::MISSION_DISPATCH:
        pending_pause_request_ = false;
        state_ =
          State::PAUSED;
        message =
          "paused before mission action dispatch; controller remains in HOLD";
        return true;

      case State::MISSION_ACTIVE:
        if (!active_goal_handle_) {
          message =
            "mission state is active but no FollowWaypoints goal handle is available";
          return false;
        }

        pending_pause_request_ = true;
        message =
          "pause requested; mission goal will be canceled and controller will HOLD";
        return true;

      case State::PAUSE_CANCELING:
      case State::PAUSED:
      case State::PAUSED_BEFORE_START:
        message =
          "trajectory is already paused or pause is in progress";
        return true;

      case State::VERTICAL_ELEVATE_DISPATCH:
      case State::VERTICAL_ELEVATE_ACTIVE:
      case State::VERTICAL_DESCEND_DISPATCH:
      case State::VERTICAL_DESCEND_ACTIVE:
        message =
          "mission is already paused; a supervision vertical maneuver is in progress";
        return true;

      default:
        message =
          std::string(
          "PAUSE is not valid while state=") +
          state_name(state_);
        return false;
    }
  }

  bool request_supervision_resume(
    std::string & message)
  {
    if (!active_trajectory_) {
      if (preemptive_pause_latch_) {
        preemptive_pause_latch_ = false;
        message =
          "preemptive pause latch cleared before trajectory acquisition";
      } else {
        message =
          "no stored trajectory; nothing is paused";
      }
      return true;
    }

    switch (state_) {
      case State::PAUSED_BEFORE_START:
        pending_pause_request_ = false;
        resume_after_pause_ = false;
        state_ =
          State::WAITING_START;
        next_attempt_at_ =
          std::chrono::steady_clock::now();
        message =
          "resume accepted; scheduled operation may continue toward takeoff";
        return true;

      case State::PAUSED:
        if (vertical_elevated_) {
          message =
            "RESUME rejected while the UAS is elevated; DESCEND must restore the pre-elevation altitude first";
          return false;
        }

        pending_pause_request_ = false;
        resume_after_pause_ = false;
        resume_after_vertical_ = false;
        state_ =
          State::MISSION_DISPATCH;
        next_attempt_at_ =
          std::chrono::steady_clock::now();
        message =
          "resume accepted; remaining mission will be sent from the paused waypoint";
        return true;

      case State::PAUSE_CANCELING:
        if (pending_vertical_intent_ != VerticalIntent::NONE) {
          message =
            "RESUME rejected because a supervision vertical maneuver is queued after PAUSE";
          return false;
        }

        resume_after_pause_ = true;
        pending_pause_request_ = false;
        message =
          "resume queued; mission will continue as soon as pause cancellation completes";
        return true;

      case State::TAKEOFF_RESPONSE:
        if (pending_pause_request_) {
          pending_pause_request_ = false;
          message =
            "queued post-takeoff pause canceled; mission will continue";
        } else {
          message =
            "trajectory is already running toward mission dispatch";
        }
        return true;

      case State::MISSION_ACTIVE:
        if (
          pending_pause_request_ &&
          !cancel_request_in_flight_)
        {
          pending_pause_request_ = false;
          message =
            "pending pause canceled; mission remains active";
        } else if (cancel_request_in_flight_) {
          resume_after_pause_ = true;
          message =
            "resume queued while mission cancellation is in flight";
        } else {
          message =
            "mission is already active";
        }
        return true;

      case State::VERTICAL_ELEVATE_DISPATCH:
      case State::VERTICAL_ELEVATE_ACTIVE:
        message =
          "RESUME rejected while the UAS is elevating/elevated; DESCEND is required before mission resume";
        return false;

      case State::VERTICAL_DESCEND_DISPATCH:
      case State::VERTICAL_DESCEND_ACTIVE:
        resume_after_vertical_ = true;
        message =
          "resume queued; mission will continue after DESCEND reaches the stored pre-elevation position";
        return true;

      case State::WAITING_START:
      case State::TAKEOFF_REQUEST:
      case State::MISSION_DISPATCH:
        pending_pause_request_ = false;
        message =
          "trajectory is already running/not paused";
        return true;

      default:
        message =
          std::string(
          "RESUME is not valid while state=") +
          state_name(state_);
        return false;
    }
  }

  bool capture_vertical_reference(
    std::string & message)
  {
    if (vertical_reference_map_) {
      return true;
    }

    if (!last_feedback_map_) {
      message =
        "cannot capture vertical reference: no valid mission feedback pose is available";
      return false;
    }

    vertical_reference_map_ =
      *last_feedback_map_;

    if (
      !finite(vertical_reference_map_->x) ||
      !finite(vertical_reference_map_->y) ||
      !finite(vertical_reference_map_->z))
    {
      vertical_reference_map_.reset();
      message =
        "cannot capture vertical reference: latest feedback pose is non-finite";
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "[/%s] Captured vertical reference in trajectory frame '%s': "
      "(%.3f, %.3f, %.3f)",
      drone_namespace_.c_str(),
      active_trajectory_ ?
      active_trajectory_->frame_id.c_str() :
      "",
      vertical_reference_map_->x,
      vertical_reference_map_->y,
      vertical_reference_map_->z);

    return true;
  }

  bool request_supervision_elevate(
    std::string & message)
  {
    if (!active_trajectory_) {
      message =
        "ELEVATE requires an acquired trajectory";
      return false;
    }

    if (vertical_elevated_) {
      message =
        "UAS is already elevated by supervision; ELEVATE treated as idempotent";
      return true;
    }

    switch (state_) {
      case State::MISSION_ACTIVE:
        if (!pending_pause_request_) {
          message =
            "ELEVATE requires PAUSE first; mission is still active";
          return false;
        }

        pending_vertical_intent_ =
          VerticalIntent::ELEVATE;
        message =
          "ELEVATE queued; it will start after the in-flight PAUSE cancellation reaches HOLD";
        return true;

      case State::PAUSE_CANCELING:
        pending_vertical_intent_ =
          VerticalIntent::ELEVATE;
        message =
          "ELEVATE queued; it will start immediately after PAUSE cancellation reaches HOLD";
        return true;

      case State::PAUSED:
      {
        if (!capture_vertical_reference(message)) {
          return false;
        }

        pending_vertical_intent_ =
          VerticalIntent::NONE;
        resume_after_vertical_ = false;
        state_ =
          State::VERTICAL_ELEVATE_DISPATCH;
        next_attempt_at_ =
          std::chrono::steady_clock::now();

        std::ostringstream stream;
        stream
          << "ELEVATE accepted; dispatching +"
          << vertical_grid_step_m_
          << " m from stored reference z="
          << vertical_reference_map_->z;
        message =
          stream.str();
        return true;
      }

      case State::VERTICAL_ELEVATE_DISPATCH:
      case State::VERTICAL_ELEVATE_ACTIVE:
        message =
          "ELEVATE is already pending/in progress";
        return true;

      case State::VERTICAL_DESCEND_DISPATCH:
      case State::VERTICAL_DESCEND_ACTIVE:
        message =
          "ELEVATE rejected while DESCEND is pending/in progress";
        return false;

      default:
        message =
          std::string(
          "ELEVATE is only valid after mission PAUSE; current state=") +
          state_name(state_);
        return false;
    }
  }

  bool request_supervision_descend(
    std::string & message)
  {
    if (!active_trajectory_) {
      message =
        "DESCEND requires an acquired trajectory";
      return false;
    }

    switch (state_) {
      case State::VERTICAL_ELEVATE_DISPATCH:
      case State::VERTICAL_ELEVATE_ACTIVE:
        pending_vertical_intent_ =
          VerticalIntent::DESCEND;
        message =
          "DESCEND queued; it will start after ELEVATE finishes";
        return true;

      case State::PAUSED:
        if (!vertical_elevated_) {
          message =
            "DESCEND rejected: there is no active supervision elevation to restore";
          return false;
        }

        if (!vertical_reference_map_) {
          message =
            "DESCEND rejected: stored pre-elevation reference is unavailable";
          return false;
        }

        pending_vertical_intent_ =
          VerticalIntent::NONE;
        state_ =
          State::VERTICAL_DESCEND_DISPATCH;
        next_attempt_at_ =
          std::chrono::steady_clock::now();

        message =
          "DESCEND accepted; returning to stored pre-elevation position";
        return true;

      case State::VERTICAL_DESCEND_DISPATCH:
      case State::VERTICAL_DESCEND_ACTIVE:
        message =
          "DESCEND is already pending/in progress";
        return true;

      default:
        message =
          std::string(
          "DESCEND is only valid while the supervised mission is paused and elevated; current state=") +
          state_name(state_);
        return false;
    }
  }

  bool request_supervision_stop(
    std::string & message)
  {
    if (
      state_ == State::RETURN_DISPATCH ||
      state_ == State::RETURN_ACTIVE ||
      state_ == State::LANDING_REQUEST ||
      state_ == State::LANDING_RESPONSE ||
      state_ == State::STOP_CANCELING)
    {
      message =
        "total stop sequence is already in progress";
      return true;
    }

    if (
      state_ == State::WAITING_TRAJECTORY ||
      state_ == State::COMPLETED ||
      state_ == State::FAILED ||
      !active_trajectory_)
    {
      message =
        "there is no running/stored operation to stop";
      return false;
    }

    resume_after_pause_ = false;
    resume_after_vertical_ = false;
    pending_vertical_intent_ =
      VerticalIntent::NONE;
    pending_stop_request_ = true;
    message =
      "total stop requested";
    return true;
  }

  static std::size_t total_mission_waypoints(
    const StaticTrajectory & trajectory)
  {
    std::size_t one_pass = 0U;

    for (const auto & mission : trajectory.mission) {
      one_pass += mission.x.size();
    }

    return
      one_pass *
      static_cast<std::size_t>(
      trajectory.repetitions);
  }

  const TrajectorySegment * current_mission_segment() const
  {
    if (
      !active_trajectory_ ||
      active_trajectory_->mission.empty() ||
      current_mission_index_ >=
      active_trajectory_->mission.size())
    {
      return nullptr;
    }

    return
      &active_trajectory_->
      mission[current_mission_index_];
  }

  void load_current_mission_path()
  {
    mission_path_map_.clear();

    const auto * mission =
      current_mission_segment();

    if (mission == nullptr) {
      return;
    }

    mission_path_map_.reserve(
      mission->x.size());

    for (std::size_t i = 0U;
      i < mission->x.size();
      ++i)
    {
      mission_path_map_.push_back(
        Point3{
          mission->x[i],
          mission->y[i],
          mission->z[i]});
    }
  }

  void initialize_mission_execution()
  {
    current_repetition_ = 0U;
    current_mission_index_ = 0U;
    current_mission_waypoint_ = 0U;
    action_base_index_ = 0U;
    action_mission_index_ = 0U;
    action_repetition_ = 0U;

    load_current_mission_path();
  }

  bool advance_to_next_mission_component()
  {
    if (
      !active_trajectory_ ||
      active_trajectory_->mission.empty())
    {
      return false;
    }

    if (
      current_mission_index_ + 1U <
      active_trajectory_->mission.size())
    {
      ++current_mission_index_;
    } else {
      if (
        current_repetition_ + 1U >=
        active_trajectory_->repetitions)
      {
        return false;
      }

      ++current_repetition_;
      current_mission_index_ = 0U;
    }

    current_mission_waypoint_ = 0U;
    action_base_index_ = 0U;
    load_current_mission_path();
    return true;
  }

  void operator_control_callback(
    const std::shared_ptr<ControlTrajectory::Request> request,
    std::shared_ptr<ControlTrajectory::Response> response)
  {
    response->accepted = false;
    response->state = state_name(state_);

    if (!request) {
      response->message = "null request";
      return;
    }

    std::string message;
    bool accepted = false;

    switch (request->command) {
      case ControlTrajectory::Request::PAUSE:
        accepted =
          request_supervision_pause(
          message);
        break;

      case ControlTrajectory::Request::RESUME:
        accepted =
          request_supervision_resume(
          message);
        break;

      case ControlTrajectory::Request::STOP:
        accepted =
          request_supervision_stop(
          message);
        break;

      default:
        message = "unknown command";
        break;
    }

    response->accepted = accepted;
    response->message = message;
    response->state = state_name(state_);

    if (accepted) {
      RCLCPP_WARN(
        get_logger(),
        "[/%s] Operator command=%u accepted in state=%s",
        drone_namespace_.c_str(),
        request->command,
        state_name(state_));
    }
  }

  bool transform_point_to_odom(const Point3 & source, const std::string & source_frame, Point3 & target)
  {
    try {
      geometry_msgs::msg::PointStamped input;
      input.header.frame_id = trim_slashes(source_frame);
      input.header.stamp = now();
      input.point.x = source.x;
      input.point.y = source.y;
      input.point.z = source.z;

      const auto transform = tf_buffer_->lookupTransform(
        odom_frame_, input.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_s_));
      geometry_msgs::msg::PointStamped output;
      tf2::doTransform(input, output, transform);
      target = Point3{output.point.x, output.point.y, output.point.z};
      return finite(target.x) && finite(target.y) && finite(target.z);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[/%s] Cannot transform point %s -> %s: %s",
        drone_namespace_.c_str(), source_frame.c_str(), odom_frame_.c_str(), error.what());
      return false;
    }
  }

  bool feedback_position_to_map(const Point3 & odom_point, Point3 & map_point)
  {
    if (!active_trajectory_) {
      return false;
    }
    try {
      geometry_msgs::msg::PointStamped input;
      input.header.frame_id = odom_frame_;
      input.header.stamp = now();
      input.point.x = odom_point.x;
      input.point.y = odom_point.y;
      input.point.z = odom_point.z;
      const auto transform = tf_buffer_->lookupTransform(
        trim_slashes(active_trajectory_->frame_id), odom_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_s_));
      geometry_msgs::msg::PointStamped output;
      tf2::doTransform(input, output, transform);
      map_point = Point3{output.point.x, output.point.y, output.point.z};
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[/%s] Cannot transform action feedback %s -> %s for path monitoring: %s",
        drone_namespace_.c_str(), odom_frame_.c_str(), active_trajectory_->frame_id.c_str(), error.what());
      return false;
    }
  }

  double distance_to_mission_path(const Point3 & point_map) const
  {
    if (mission_path_map_.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    if (mission_path_map_.size() == 1U) {
      const auto & p = mission_path_map_.front();
      const double dx = point_map.x - p.x;
      const double dy = point_map.y - p.y;
      const double dz = point_map.z - p.z;
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1U; i < mission_path_map_.size(); ++i) {
      best = std::min(best, point_segment_distance_3d(
        point_map, mission_path_map_[i - 1U], mission_path_map_[i]));
    }
    return best;
  }

  void process_mission_feedback(
    const FollowWaypoints::Feedback & feedback)
  {
    current_mission_waypoint_ =
      std::min<std::size_t>(
      action_base_index_ +
      static_cast<std::size_t>(
        feedback.current_waypoint),
      mission_path_map_.empty() ?
      0U :
      mission_path_map_.size() - 1U);

    const Point3 feedback_odom{
      feedback.pose.position.x,
      feedback.pose.position.y,
      feedback.pose.position.z};

    if (
      !finite(feedback_odom.x) ||
      !finite(feedback_odom.y) ||
      !finite(feedback_odom.z))
    {
      return;
    }

    // Keep the latest real mission pose independently from path-deviation
    // monitoring. Supervision ELEVATE captures this position after PAUSE and
    // later DESCEND returns to it without touching the mission cursor.
    last_feedback_odom_ =
      feedback_odom;

    Point3 feedback_map;
    if (!feedback_position_to_map(feedback_odom, feedback_map)) {
      return;
    }

    last_feedback_map_ =
      feedback_map;

    const auto now_steady =
      std::chrono::steady_clock::now();

    if (
      now_steady - mission_started_at_ <
      std::chrono::duration<double>(
        deviation_grace_period_s_))
    {
      return;
    }

    const double distance =
      distance_to_mission_path(
      feedback_map);

    last_path_deviation_m_ =
      distance;

    if (distance <= max_path_deviation_m_) {
      deviation_active_ = false;
      return;
    }

    if (!deviation_active_) {
      deviation_active_ = true;
      deviation_started_at_ =
        now_steady;

      RCLCPP_WARN(
        get_logger(),
        "[/%s] Path deviation detected | trajectory='%s' | repetition=%u | "
        "mission=%zu | distance=%.3f m | limit=%.3f m",
        drone_namespace_.c_str(),
        active_trajectory_->
        trajectory_id.c_str(),
        current_repetition_,
        current_mission_index_,
        distance,
        max_path_deviation_m_);

      return;
    }

    const double duration =
      std::chrono::duration<double>(
      now_steady -
      deviation_started_at_).count();

    if (
      duration >= deviation_hold_time_s_ &&
      !pending_stop_request_)
    {
      pending_stop_request_ = true;

      RCLCPP_ERROR(
        get_logger(),
        "[/%s] Sustained path deviation %.3f m for %.2f s -> TOTAL STOP",
        drone_namespace_.c_str(),
        distance,
        duration);
    }
  }

  void request_goal_cancellation(CancelIntent intent)
  {
    if (!active_goal_handle_ || cancel_request_in_flight_) {
      return;
    }
    cancel_intent_ = intent;
    cancel_request_in_flight_ = true;
    state_ = intent == CancelIntent::PAUSE ? State::PAUSE_CANCELING : State::STOP_CANCELING;

    follow_waypoints_client_->async_cancel_goal(
      active_goal_handle_,
      [this](const FollowWaypointsClient::CancelResponse::SharedPtr response) {
        cancel_request_in_flight_ = false;
        if (!response || response->return_code != FollowWaypointsClient::CancelResponse::ERROR_NONE) {
          RCLCPP_ERROR(
            get_logger(), "[/%s] Action cancellation request was rejected",
            drone_namespace_.c_str());
          // Do not dispatch another goal while the controller still owns the
          // goal whose cancellation was rejected. Restore the active state and
          // keep the operator request pending so the next supervision tick
          // retries the cancellation.
          switch (action_purpose_) {
            case ActionPurpose::VERTICAL_ELEVATE:
              state_ =
                State::VERTICAL_ELEVATE_ACTIVE;
              break;
            case ActionPurpose::VERTICAL_DESCEND:
              state_ =
                State::VERTICAL_DESCEND_ACTIVE;
              break;
            case ActionPurpose::RETURN_TO_LANDING:
              state_ =
                State::RETURN_ACTIVE;
              break;
            case ActionPurpose::MISSION:
            default:
              state_ =
                State::MISSION_ACTIVE;
              break;
          }
          cancel_intent_ = CancelIntent::NONE;

          if (resume_after_pause_) {
            resume_after_pause_ = false;
            pending_pause_request_ = false;
          }
        }
      });
  }

  void send_takeoff_service(bool landing)
  {
    if (!active_trajectory_) {
      fail("no active trajectory for service request");
      return;
    }
    if (!arm_takeoff_client_->wait_for_service(std::chrono::milliseconds(service_wait_timeout_ms_))) {
      next_attempt_at_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(retry_period_ms_);
      return;
    }

    const auto & segment = landing ? active_trajectory_->landing : active_trajectory_->takeoff;
    const std::size_t index = segment.x.size() - 1U;
    const Point3 target_map{segment.x[index], segment.y[index], segment.z[index]};
    Point3 target_odom;
    if (!transform_point_to_odom(target_map, active_trajectory_->frame_id, target_odom)) {
      next_attempt_at_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(retry_period_ms_);
      return;
    }

    auto request = std::make_shared<ArmTakeoff::Request>();
    request->x = target_odom.x;
    request->y = target_odom.y;
    request->z = target_odom.z;
    request->yaw = service_yaw_rad_;

    state_ = landing ? State::LANDING_RESPONSE : State::TAKEOFF_RESPONSE;

    RCLCPP_INFO(
      get_logger(), "[/%s] Sending %s service target=(%.3f, %.3f, %.3f)",
      drone_namespace_.c_str(), landing ? "LANDING" : "TAKEOFF",
      target_odom.x, target_odom.y, target_odom.z);

    arm_takeoff_client_->async_send_request(
      request,
      [this, landing](rclcpp::Client<ArmTakeoff>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response || !response->accepted) {
            RCLCPP_WARN(
              get_logger(), "[/%s] %s service rejected: %s",
              drone_namespace_.c_str(), landing ? "LANDING" : "TAKEOFF",
              response ? response->message.c_str() : "null response");
            state_ = landing ? State::LANDING_REQUEST : State::TAKEOFF_REQUEST;
            next_attempt_at_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(retry_period_ms_);
            return;
          }

          if (landing) {
            RCLCPP_INFO(
              get_logger(), "[/%s] LANDING command accepted by controller: %s",
              drone_namespace_.c_str(), response->message.c_str());
            complete_current_occurrence();
            return;
          }

          RCLCPP_INFO(
            get_logger(), "[/%s] TAKEOFF command accepted; mission action will be retried until controller reaches HOLD",
            drone_namespace_.c_str());
          if (pending_stop_request_) {
            state_ = State::RETURN_DISPATCH;
          } else if (pending_pause_request_) {
            pending_pause_request_ = false;
            state_ = State::PAUSED;
            RCLCPP_WARN(
              get_logger(),
              "[/%s] TAKEOFF completed while PAUSE was pending; controller remains in HOLD",
              drone_namespace_.c_str());
          } else {
            state_ = State::MISSION_DISPATCH;
          }
          next_attempt_at_ = std::chrono::steady_clock::now();
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Controller service exception: %s", error.what());
          state_ = landing ? State::LANDING_REQUEST : State::TAKEOFF_REQUEST;
          next_attempt_at_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(retry_period_ms_);
        }
      });
  }

  void send_action(
    ActionPurpose purpose)
  {
    if (!active_trajectory_) {
      fail(
        "no active trajectory for action request");
      return;
    }

    if (
      !follow_waypoints_client_->
      wait_for_action_server(
        std::chrono::milliseconds(
          action_wait_timeout_ms_)))
    {
      next_attempt_at_ =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(
        retry_period_ms_);
      return;
    }

    FollowWaypoints::Goal goal;
    std::size_t base_index = 0U;

    if (purpose == ActionPurpose::MISSION) {
      const auto * mission =
        current_mission_segment();

      if (
        mission == nullptr ||
        mission_path_map_.empty())
      {
        fail(
          "current mission component is unavailable");
        return;
      }

      base_index =
        current_mission_waypoint_;

      if (base_index >= mission_path_map_.size()) {
        if (advance_to_next_mission_component()) {
          state_ =
            State::MISSION_DISPATCH;
        } else {
          state_ =
            State::LANDING_REQUEST;
        }

        next_attempt_at_ =
          std::chrono::steady_clock::now();
        return;
      }

      goal.x.reserve(
        mission_path_map_.size() -
        base_index);

      goal.y.reserve(
        mission_path_map_.size() -
        base_index);

      for (
        std::size_t i = base_index;
        i < mission_path_map_.size();
        ++i)
      {
        goal.x.push_back(
          mission_path_map_[i].x);
        goal.y.push_back(
          mission_path_map_[i].y);
      }

      goal.height =
        mission->z.front();

      goal.goal_tolerance =
        active_trajectory_->
        goal_tolerance;

      goal.slowdown_radius =
        active_trajectory_->
        slowdown_radius;

      goal.repetitions =
        1U;
    } else if (
      purpose ==
      ActionPurpose::VERTICAL_ELEVATE ||
      purpose ==
      ActionPurpose::VERTICAL_DESCEND)
    {
      if (!vertical_reference_map_) {
        std::string reason;
        if (
          purpose ==
          ActionPurpose::VERTICAL_ELEVATE &&
          !capture_vertical_reference(
            reason))
        {
          RCLCPP_WARN(
            get_logger(),
            "[/%s] Cannot dispatch ELEVATE: %s; retrying",
            drone_namespace_.c_str(),
            reason.c_str());

          next_attempt_at_ =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(
            retry_period_ms_);
          return;
        }

        if (!vertical_reference_map_) {
          RCLCPP_ERROR(
            get_logger(),
            "[/%s] Cannot dispatch DESCEND: vertical reference is unavailable",
            drone_namespace_.c_str());

          state_ =
            State::PAUSED;
          return;
        }
      }

      const double target_z =
        purpose ==
        ActionPurpose::VERTICAL_ELEVATE ?
        vertical_reference_map_->z +
        vertical_grid_step_m_ :
        vertical_reference_map_->z;

      // FollowWaypoints goals use the same trajectory/map coordinate contract
      // as normal mission goals. A one-point goal is sufficient for a pure
      // vertical displacement while keeping X/Y fixed.
      goal.x = {
        vertical_reference_map_->x};
      goal.y = {
        vertical_reference_map_->y};
      goal.height =
        target_z;
      goal.goal_tolerance =
        active_trajectory_->
        goal_tolerance;
      goal.slowdown_radius =
        active_trajectory_->
        slowdown_radius;
      goal.repetitions =
        1U;
      base_index =
        0U;

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[/%s] Dispatching %s auxiliary goal | reference=(%.3f, %.3f, %.3f) | "
        "target_z=%.3f | mission cursor preserved repetition=%u mission=%zu waypoint=%zu",
        drone_namespace_.c_str(),
        purpose ==
        ActionPurpose::VERTICAL_ELEVATE ?
        "ELEVATE" :
        "DESCEND",
        vertical_reference_map_->x,
        vertical_reference_map_->y,
        vertical_reference_map_->z,
        target_z,
        current_repetition_,
        current_mission_index_,
        current_mission_waypoint_);
    } else {
      const auto & landing =
        active_trajectory_->landing;

      const Point3 landing_entry{
        landing.x.front(),
        landing.y.front(),
        landing.z.front()};

      if (landing_entry.z <= 0.0) {
        state_ =
          State::LANDING_REQUEST;
        next_attempt_at_ =
          std::chrono::steady_clock::now();
        return;
      }

      goal.x = {
        landing_entry.x};
      goal.y = {
        landing_entry.y};
      goal.height =
        landing_entry.z;
      goal.goal_tolerance =
        active_trajectory_->
        goal_tolerance;
      goal.slowdown_radius =
        active_trajectory_->
        slowdown_radius;
      goal.repetitions =
        1U;
      base_index =
        0U;
    }

    action_purpose_ =
      purpose;

    action_base_index_ =
      base_index;

    if (purpose == ActionPurpose::MISSION) {
      action_mission_index_ =
        current_mission_index_;
      action_repetition_ =
        current_repetition_;
    }

    const std::string trajectory_id =
      active_trajectory_->
      trajectory_id;

    auto dispatch_state =
      [purpose]() -> State {
        switch (purpose) {
          case ActionPurpose::MISSION:
            return State::MISSION_DISPATCH;
          case ActionPurpose::VERTICAL_ELEVATE:
            return State::VERTICAL_ELEVATE_DISPATCH;
          case ActionPurpose::VERTICAL_DESCEND:
            return State::VERTICAL_DESCEND_DISPATCH;
          case ActionPurpose::RETURN_TO_LANDING:
            return State::RETURN_DISPATCH;
          case ActionPurpose::NONE:
          default:
            return State::PAUSED;
        }
      };

    auto active_state =
      [purpose]() -> State {
        switch (purpose) {
          case ActionPurpose::MISSION:
            return State::MISSION_ACTIVE;
          case ActionPurpose::VERTICAL_ELEVATE:
            return State::VERTICAL_ELEVATE_ACTIVE;
          case ActionPurpose::VERTICAL_DESCEND:
            return State::VERTICAL_DESCEND_ACTIVE;
          case ActionPurpose::RETURN_TO_LANDING:
            return State::RETURN_ACTIVE;
          case ActionPurpose::NONE:
          default:
            return State::PAUSED;
        }
      };

    rclcpp_action::Client<
      FollowWaypoints>::
      SendGoalOptions options;

    options.goal_response_callback =
      [this,
        purpose,
        trajectory_id,
        dispatch_state,
        active_state](
        const GoalHandleFollowWaypoints::
        SharedPtr goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_DEBUG(
            get_logger(),
            "[/%s] %s action rejected; controller may not be in HOLD yet, retrying",
            drone_namespace_.c_str(),
            purpose ==
            ActionPurpose::MISSION ?
            "MISSION" :
            purpose ==
            ActionPurpose::VERTICAL_ELEVATE ?
            "ELEVATE" :
            purpose ==
            ActionPurpose::VERTICAL_DESCEND ?
            "DESCEND" :
            "RETURN");

          state_ =
            dispatch_state();

          action_purpose_ =
            ActionPurpose::NONE;

          next_attempt_at_ =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(
            retry_period_ms_);

          return;
        }

        active_goal_handle_ =
          goal_handle;

        deviation_active_ =
          false;

        mission_started_at_ =
          std::chrono::steady_clock::now();

        state_ =
          active_state();

        if (purpose == ActionPurpose::MISSION) {
          RCLCPP_INFO(
            get_logger(),
            "[/%s] MISSION action accepted | trajectory='%s' | repetition=%u | "
            "mission=%zu/%zu | base_waypoint=%zu",
            drone_namespace_.c_str(),
            trajectory_id.c_str(),
            action_repetition_,
            action_mission_index_,
            active_trajectory_ ?
            active_trajectory_->
            mission.size() :
            0U,
            action_base_index_);
        } else if (
          purpose ==
          ActionPurpose::VERTICAL_ELEVATE ||
          purpose ==
          ActionPurpose::VERTICAL_DESCEND)
        {
          RCLCPP_INFO(
            get_logger(),
            "[/%s] %s auxiliary action accepted | mission remains paused | "
            "repetition=%u mission=%zu waypoint=%zu",
            drone_namespace_.c_str(),
            purpose ==
            ActionPurpose::VERTICAL_ELEVATE ?
            "ELEVATE" :
            "DESCEND",
            current_repetition_,
            current_mission_index_,
            current_mission_waypoint_);
        } else {
          RCLCPP_INFO(
            get_logger(),
            "[/%s] RETURN action accepted | trajectory='%s'",
            drone_namespace_.c_str(),
            trajectory_id.c_str());
        }
      };

    options.feedback_callback =
      [this](
        const GoalHandleFollowWaypoints::
        SharedPtr,
        const std::shared_ptr<
        const FollowWaypoints::Feedback>
        feedback)
      {
        if (!feedback) {
          return;
        }

        if (
          action_purpose_ ==
          ActionPurpose::MISSION &&
          (
            state_ ==
            State::MISSION_ACTIVE ||
            state_ ==
            State::PAUSE_CANCELING ||
            state_ ==
            State::STOP_CANCELING))
        {
          process_mission_feedback(
            *feedback);
        }
      };

    options.result_callback =
      [this,
        purpose,
        trajectory_id](
        const GoalHandleFollowWaypoints::
        WrappedResult & result)
      {
        active_goal_handle_.reset();
        cancel_request_in_flight_ =
          false;

        if (
          result.code ==
          rclcpp_action::ResultCode::CANCELED)
        {
          const CancelIntent intent =
            cancel_intent_;

          cancel_intent_ =
            CancelIntent::NONE;

          action_purpose_ =
            ActionPurpose::NONE;

          deviation_active_ =
            false;

          if (
            intent ==
            CancelIntent::PAUSE &&
            purpose ==
            ActionPurpose::MISSION)
          {
            pending_pause_request_ =
              false;

            if (
              pending_vertical_intent_ ==
              VerticalIntent::ELEVATE)
            {
              pending_vertical_intent_ =
                VerticalIntent::NONE;

              state_ =
                State::VERTICAL_ELEVATE_DISPATCH;

              next_attempt_at_ =
                std::chrono::steady_clock::now();

              RCLCPP_INFO(
                get_logger(),
                "[/%s] Mission PAUSE completed; queued ELEVATE will now be dispatched | "
                "repetition=%u mission=%zu waypoint=%zu",
                drone_namespace_.c_str(),
                current_repetition_,
                current_mission_index_,
                current_mission_waypoint_);
            } else if (resume_after_pause_) {
              resume_after_pause_ =
                false;

              state_ =
                State::MISSION_DISPATCH;

              next_attempt_at_ =
                std::chrono::steady_clock::now();

              RCLCPP_INFO(
                get_logger(),
                "[/%s] PAUSE cancellation completed but RESUME was queued; "
                "remaining mission will be dispatched immediately",
                drone_namespace_.c_str());
            } else {
              state_ =
                State::PAUSED;

              RCLCPP_INFO(
                get_logger(),
                "[/%s] Mission PAUSED | repetition=%u | mission=%zu | waypoint=%zu; "
                "controller is in HOLD",
                drone_namespace_.c_str(),
                current_repetition_,
                current_mission_index_,
                current_mission_waypoint_);
            }

            return;
          }

          if (
            intent ==
            CancelIntent::STOP)
          {
            pending_vertical_intent_ =
              VerticalIntent::NONE;

            resume_after_vertical_ =
              false;

            state_ =
              State::RETURN_DISPATCH;

            pending_stop_request_ =
              false;

            next_attempt_at_ =
              std::chrono::steady_clock::now();

            RCLCPP_WARN(
              get_logger(),
              "[/%s] Active action canceled for TOTAL STOP; returning to landing point",
              drone_namespace_.c_str());

            return;
          }

          // Unexpected cancellation of a vertical auxiliary action: keep the
          // mission paused and retry the same vertical command.
          if (
            purpose ==
            ActionPurpose::VERTICAL_ELEVATE)
          {
            state_ =
              State::VERTICAL_ELEVATE_DISPATCH;

            next_attempt_at_ =
              std::chrono::steady_clock::now() +
              std::chrono::milliseconds(
              retry_period_ms_);

            return;
          }

          if (
            purpose ==
            ActionPurpose::VERTICAL_DESCEND)
          {
            state_ =
              State::VERTICAL_DESCEND_DISPATCH;

            next_attempt_at_ =
              std::chrono::steady_clock::now() +
              std::chrono::milliseconds(
              retry_period_ms_);

            return;
          }

          state_ =
            State::RETURN_DISPATCH;
          next_attempt_at_ =
            std::chrono::steady_clock::now();
          return;
        }

        const bool success =
          result.code ==
          rclcpp_action::ResultCode::SUCCEEDED &&
          result.result &&
          result.result->success;

        action_purpose_ =
          ActionPurpose::NONE;

        deviation_active_ =
          false;

        if (purpose == ActionPurpose::MISSION) {
          if (success) {
            current_mission_waypoint_ =
              mission_path_map_.size();

            const uint32_t completed_repetition =
              current_repetition_;

            const std::size_t completed_mission =
              current_mission_index_;

            if (advance_to_next_mission_component()) {
              state_ =
                State::MISSION_DISPATCH;

              next_attempt_at_ =
                std::chrono::steady_clock::now();

              RCLCPP_INFO(
                get_logger(),
                "[/%s] Mission component completed | trajectory='%s' | "
                "repetition=%u | mission=%zu -> next repetition=%u mission=%zu",
                drone_namespace_.c_str(),
                trajectory_id.c_str(),
                completed_repetition,
                completed_mission,
                current_repetition_,
                current_mission_index_);
            } else {
              state_ =
                State::LANDING_REQUEST;

              next_attempt_at_ =
                std::chrono::steady_clock::now();

              RCLCPP_INFO(
                get_logger(),
                "[/%s] Complete mission collection '%s' finished | "
                "repetitions=%u | missions_per_repetition=%zu; sending landing service",
                drone_namespace_.c_str(),
                trajectory_id.c_str(),
                active_trajectory_ ?
                active_trajectory_->
                repetitions :
                0U,
                active_trajectory_ ?
                active_trajectory_->
                mission.size() :
                0U);
            }
          } else {
            pending_stop_request_ =
              false;

            state_ =
              State::RETURN_DISPATCH;

            next_attempt_at_ =
              std::chrono::steady_clock::now();

            RCLCPP_ERROR(
              get_logger(),
              "[/%s] Mission action failed; executing return-and-land safety sequence",
              drone_namespace_.c_str());
          }

          return;
        }

        if (
          purpose ==
          ActionPurpose::VERTICAL_ELEVATE)
        {
          if (!success) {
            RCLCPP_WARN(
              get_logger(),
              "[/%s] ELEVATE auxiliary action did not complete successfully; retrying while mission remains PAUSED",
              drone_namespace_.c_str());

            state_ =
              State::VERTICAL_ELEVATE_DISPATCH;

            next_attempt_at_ =
              std::chrono::steady_clock::now() +
              std::chrono::milliseconds(
              retry_period_ms_);

            return;
          }

          vertical_elevated_ =
            true;

          RCLCPP_INFO(
            get_logger(),
            "[/%s] ELEVATE completed | reference_z=%.3f | elevated_z=%.3f | "
            "mission cursor preserved repetition=%u mission=%zu waypoint=%zu",
            drone_namespace_.c_str(),
            vertical_reference_map_ ?
            vertical_reference_map_->z :
            std::numeric_limits<double>::quiet_NaN(),
            vertical_reference_map_ ?
            vertical_reference_map_->z +
            vertical_grid_step_m_ :
            std::numeric_limits<double>::quiet_NaN(),
            current_repetition_,
            current_mission_index_,
            current_mission_waypoint_);

          if (
            pending_vertical_intent_ ==
            VerticalIntent::DESCEND)
          {
            pending_vertical_intent_ =
              VerticalIntent::NONE;

            state_ =
              State::VERTICAL_DESCEND_DISPATCH;

            next_attempt_at_ =
              std::chrono::steady_clock::now();
          } else {
            state_ =
              State::PAUSED;
          }

          return;
        }

        if (
          purpose ==
          ActionPurpose::VERTICAL_DESCEND)
        {
          if (!success) {
            RCLCPP_WARN(
              get_logger(),
              "[/%s] DESCEND auxiliary action did not complete successfully; retrying while mission remains PAUSED",
              drone_namespace_.c_str());

            state_ =
              State::VERTICAL_DESCEND_DISPATCH;

            next_attempt_at_ =
              std::chrono::steady_clock::now() +
              std::chrono::milliseconds(
              retry_period_ms_);

            return;
          }

          RCLCPP_INFO(
            get_logger(),
            "[/%s] DESCEND completed | restored_z=%.3f | "
            "mission cursor preserved repetition=%u mission=%zu waypoint=%zu",
            drone_namespace_.c_str(),
            vertical_reference_map_ ?
            vertical_reference_map_->z :
            std::numeric_limits<double>::quiet_NaN(),
            current_repetition_,
            current_mission_index_,
            current_mission_waypoint_);

          vertical_elevated_ =
            false;

          vertical_reference_map_.reset();

          pending_vertical_intent_ =
            VerticalIntent::NONE;

          if (resume_after_vertical_) {
            resume_after_vertical_ =
              false;

            state_ =
              State::MISSION_DISPATCH;

            next_attempt_at_ =
              std::chrono::steady_clock::now();

            RCLCPP_INFO(
              get_logger(),
              "[/%s] Queued RESUME released after DESCEND; remaining mission will be dispatched",
              drone_namespace_.c_str());
          } else {
            state_ =
              State::PAUSED;
          }

          return;
        }

        state_ =
          State::LANDING_REQUEST;

        next_attempt_at_ =
          std::chrono::steady_clock::now();

        if (!success) {
          RCLCPP_ERROR(
            get_logger(),
            "[/%s] Return-to-landing action failed; landing endpoint command will still be attempted",
            drone_namespace_.c_str());
        }
      };

    follow_waypoints_client_->
    async_send_goal(
      goal,
      options);
  }

  void process_pending_commands()
  {
    if (pending_stop_request_) {
      if (
        state_ == State::WAITING_START ||
        state_ == State::PAUSED_BEFORE_START ||
        state_ == State::TAKEOFF_REQUEST)
      {
        RCLCPP_WARN(
          get_logger(),
          "[/%s] TOTAL STOP before takeoff: occurrence canceled without flight commands",
          drone_namespace_.c_str());

        pending_stop_request_ = false;
        pending_pause_request_ = false;
        resume_after_pause_ = false;
        complete_current_occurrence();
        return;
      }

      if (
        active_goal_handle_ &&
        (
          state_ == State::MISSION_ACTIVE ||
          state_ == State::VERTICAL_ELEVATE_ACTIVE ||
          state_ == State::VERTICAL_DESCEND_ACTIVE ||
          state_ == State::RETURN_ACTIVE))
      {
        request_goal_cancellation(
          CancelIntent::STOP);
        return;
      }

      if (
        state_ == State::PAUSED ||
        state_ == State::MISSION_DISPATCH ||
        state_ == State::VERTICAL_ELEVATE_DISPATCH ||
        state_ == State::VERTICAL_DESCEND_DISPATCH)
      {
        pending_stop_request_ = false;
        pending_pause_request_ = false;
        pending_vertical_intent_ =
          VerticalIntent::NONE;
        resume_after_pause_ = false;
        resume_after_vertical_ = false;
        state_ =
          State::RETURN_DISPATCH;

        next_attempt_at_ =
          std::chrono::steady_clock::now();
        return;
      }
    }

    if (!pending_pause_request_) {
      return;
    }

    if (
      state_ == State::WAITING_START ||
      state_ == State::TAKEOFF_REQUEST)
    {
      pending_pause_request_ = false;
      supervision_start_delay_authorized_ = true;
      state_ =
        State::PAUSED_BEFORE_START;
      return;
    }

    if (state_ == State::MISSION_DISPATCH) {
      pending_pause_request_ = false;
      state_ =
        State::PAUSED;
      return;
    }

    if (
      state_ == State::MISSION_ACTIVE &&
      active_goal_handle_)
    {
      request_goal_cancellation(
        CancelIntent::PAUSE);
    }
  }

  void tick()
  {
    process_pending_commands();
    const auto steady_now = std::chrono::steady_clock::now();

    switch (state_) {
      case State::WAITING_TRAJECTORY:
      case State::PAUSED_BEFORE_START:
      case State::PAUSE_CANCELING:
      case State::PAUSED:
      case State::VERTICAL_ELEVATE_ACTIVE:
      case State::VERTICAL_DESCEND_ACTIVE:
      case State::STOP_CANCELING:
      case State::MISSION_ACTIVE:
      case State::RETURN_ACTIVE:
      case State::TAKEOFF_RESPONSE:
      case State::LANDING_RESPONSE:
        return;

      case State::WAITING_START:
      {
        if (!active_trajectory_) {
          clear_plan();
          return;
        }
        const int64_t now_ns = system_now_ns();
        const int64_t start_ns = time_to_ns(active_trajectory_->operation_start_utc);
        if (now_ns < start_ns) {
          return;
        }
        const double lateness_s = static_cast<double>(now_ns - start_ns) / 1.0e9;
        if (
          lateness_s > max_start_lateness_s_ &&
          !supervision_start_delay_authorized_)
        {
          remember_completed(active_occurrence_key_);
          fail("operation start was missed by " + std::to_string(lateness_s) + " s");
          return;
        }

        if (
          lateness_s > max_start_lateness_s_ &&
          supervision_start_delay_authorized_)
        {
          RCLCPP_WARN(
            get_logger(),
            "[/%s] Operation start lateness %.3f s accepted because supervision "
            "explicitly paused the trajectory before takeoff",
            drone_namespace_.c_str(),
            lateness_s);
        }

        supervision_start_delay_authorized_ = false;
        state_ = State::TAKEOFF_REQUEST;
        next_attempt_at_ = steady_now;
        RCLCPP_INFO(
          get_logger(), "[/%s] Operation '%s' is DUE | dispatch_lateness=%.3f s",
          drone_namespace_.c_str(), active_trajectory_->trajectory_id.c_str(), lateness_s);
        return;
      }

      case State::TAKEOFF_REQUEST:
        if (steady_now >= next_attempt_at_) {
          send_takeoff_service(false);
        }
        return;

      case State::MISSION_DISPATCH:
        if (steady_now >= next_attempt_at_) {
          send_action(ActionPurpose::MISSION);
        }
        return;

      case State::VERTICAL_ELEVATE_DISPATCH:
        if (steady_now >= next_attempt_at_) {
          send_action(
            ActionPurpose::VERTICAL_ELEVATE);
        }
        return;

      case State::VERTICAL_DESCEND_DISPATCH:
        if (steady_now >= next_attempt_at_) {
          send_action(
            ActionPurpose::VERTICAL_DESCEND);
        }
        return;

      case State::RETURN_DISPATCH:
        if (steady_now >= next_attempt_at_) {
          send_action(ActionPurpose::RETURN_TO_LANDING);
        }
        return;

      case State::LANDING_REQUEST:
        if (steady_now >= next_attempt_at_) {
          send_takeoff_service(true);
        }
        return;

      case State::COMPLETED:
        clear_plan();
        return;

      case State::FAILED:
        clear_plan();
        return;
    }
  }

  void complete_current_occurrence()
  {
    remember_completed(active_occurrence_key_);
    state_ = State::COMPLETED;
    RCLCPP_INFO(
      get_logger(), "[/%s] Operation occurrence completed | key='%s'",
      drone_namespace_.c_str(), active_occurrence_key_.c_str());
  }

  void fail(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "[/%s] Control sequence FAILED: %s", drone_namespace_.c_str(), reason.c_str());
    state_ = State::FAILED;
  }

  void clear_plan()
  {
    if (active_goal_handle_) {
      return;
    }
    active_trajectory_.reset();
    active_occurrence_key_.clear();
    mission_path_map_.clear();
    current_repetition_ = 0U;
    current_mission_index_ = 0U;
    current_mission_waypoint_ = 0U;
    action_base_index_ = 0U;
    action_mission_index_ = 0U;
    action_repetition_ = 0U;
    action_purpose_ = ActionPurpose::NONE;
    cancel_intent_ = CancelIntent::NONE;
    pending_vertical_intent_ =
      VerticalIntent::NONE;
    pending_pause_request_ = false;
    pending_stop_request_ = false;
    resume_after_pause_ = false;
    resume_after_vertical_ = false;
    supervision_start_delay_authorized_ = false;
    vertical_elevated_ = false;
    vertical_reference_map_.reset();
    last_feedback_odom_.reset();
    last_feedback_map_.reset();
    deviation_active_ = false;
    state_ = State::WAITING_TRAJECTORY;
  }

  static const char * state_name(State state)
  {
    switch (state) {
      case State::WAITING_TRAJECTORY: return "WAITING_TRAJECTORY";
      case State::WAITING_START: return "WAITING_START";
      case State::PAUSED_BEFORE_START: return "PAUSED_BEFORE_START";
      case State::TAKEOFF_REQUEST: return "TAKEOFF_REQUEST";
      case State::TAKEOFF_RESPONSE: return "TAKEOFF_RESPONSE";
      case State::MISSION_DISPATCH: return "MISSION_DISPATCH";
      case State::MISSION_ACTIVE: return "MISSION_ACTIVE";
      case State::PAUSE_CANCELING: return "PAUSE_CANCELING";
      case State::PAUSED: return "PAUSED";
      case State::VERTICAL_ELEVATE_DISPATCH: return "VERTICAL_ELEVATE_DISPATCH";
      case State::VERTICAL_ELEVATE_ACTIVE: return "VERTICAL_ELEVATE_ACTIVE";
      case State::VERTICAL_DESCEND_DISPATCH: return "VERTICAL_DESCEND_DISPATCH";
      case State::VERTICAL_DESCEND_ACTIVE: return "VERTICAL_DESCEND_ACTIVE";
      case State::STOP_CANCELING: return "STOP_CANCELING";
      case State::RETURN_DISPATCH: return "RETURN_DISPATCH";
      case State::RETURN_ACTIVE: return "RETURN_ACTIVE";
      case State::LANDING_REQUEST: return "LANDING_REQUEST";
      case State::LANDING_RESPONSE: return "LANDING_RESPONSE";
      case State::COMPLETED: return "COMPLETED";
      case State::FAILED: return "FAILED";
      default: return "UNKNOWN";
    }
  }

  std::string flight_zone_id_;
  std::string uas_namespace_;
  std::string drone_namespace_;
  std::string odom_frame_;
  std::string expected_action_name_;
  std::string supervision_action_name_;

  std::string odom_frame_suffix_;
  std::string arm_takeoff_service_suffix_;
  std::string follow_waypoints_action_suffix_;
  std::string supervision_action_suffix_;
  std::string operator_service_suffix_;

  int tick_period_ms_{100};
  int retry_period_ms_{500};
  int service_wait_timeout_ms_{50};
  int action_wait_timeout_ms_{50};
  double service_yaw_rad_{0.0};
  int max_completed_keys_{128};
  double max_start_lateness_s_{5.0};
  double transform_timeout_s_{0.20};
  double vertical_grid_step_m_{1.0};
  double max_path_deviation_m_{1.0};
  double deviation_grace_period_s_{1.0};
  double deviation_hold_time_s_{0.75};

  State state_{State::WAITING_TRAJECTORY};
  ActionPurpose action_purpose_{ActionPurpose::NONE};
  VerticalIntent pending_vertical_intent_{VerticalIntent::NONE};
  CancelIntent cancel_intent_{CancelIntent::NONE};

  std::optional<StaticTrajectory> active_trajectory_;
  std::string active_occurrence_key_;
  // Geometry of the single mission[i] currently being executed. Independent
  // mission components are never concatenated into this vector.
  std::vector<Point3> mission_path_map_;

  // Ordered execution cursor:
  //   repetition 0: mission[0] ... mission[N-1]
  //   repetition 1: mission[0] ... mission[N-1]
  //   ...
  uint32_t current_repetition_{0U};
  std::size_t current_mission_index_{0U};
  std::size_t current_mission_waypoint_{0U};

  // Snapshot of the unit/base waypoint represented by the in-flight
  // FollowWaypoints goal.
  std::size_t action_base_index_{0U};
  std::size_t action_mission_index_{0U};
  uint32_t action_repetition_{0U};

  bool pending_pause_request_{false};
  bool pending_stop_request_{false};
  bool resume_after_pause_{false};
  bool resume_after_vertical_{false};
  bool supervision_start_delay_authorized_{false};
  bool preemptive_pause_latch_{false};
  bool cancel_request_in_flight_{false};

  // Supervision vertical maneuver state. The reference is the real vehicle
  // pose captured from mission feedback in the trajectory/map frame before
  // ELEVATE. Mission indices are never modified by ELEVATE/DESCEND.
  bool vertical_elevated_{false};
  std::optional<Point3> vertical_reference_map_;
  std::optional<Point3> last_feedback_odom_;
  std::optional<Point3> last_feedback_map_;

  bool deviation_active_{false};
  double last_path_deviation_m_{0.0};
  std::chrono::steady_clock::time_point deviation_started_at_{};
  std::chrono::steady_clock::time_point mission_started_at_{};
  std::chrono::steady_clock::time_point next_attempt_at_{};

  std::deque<std::string> completed_keys_;
  std::set<std::string> completed_keys_set_;

  rclcpp::Client<ArmTakeoff>::SharedPtr arm_takeoff_client_;
  FollowWaypointsClient::SharedPtr follow_waypoints_client_;
  rclcpp::Service<ControlTrajectory>::SharedPtr operator_service_;
  SupervisionControlServer::SharedPtr supervision_action_server_;
  GoalHandleFollowWaypoints::SharedPtr active_goal_handle_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace control_manager_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<control_manager_node::ControlManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("control_manager_node"), "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
