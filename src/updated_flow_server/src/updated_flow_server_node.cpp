#include <rclcpp/rclcpp.hpp>

#include <updated_flow_server/msg/flow_trajectory_status.hpp>
#include <updated_flow_server/msg/updated_flow_status.hpp>

#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <static_trajectory_conflict_manager/msg/collision_static_trajectory_array.hpp>

#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace updated_flow_server
{

using FlowTrajectoryStatus = updated_flow_server::msg::FlowTrajectoryStatus;
using UpdatedFlowStatus = updated_flow_server::msg::UpdatedFlowStatus;
using StaticTrajectory = static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using CollisionStaticTrajectoryArray =
  static_trajectory_conflict_manager::msg::CollisionStaticTrajectoryArray;

struct VerificationSignature
{
  uint8_t expectation{FlowTrajectoryStatus::EXPECT_REMOVED};
  bool present_in_available{false};
  bool present_in_collision{false};
  bool verified{false};
  bool contradictory_input{false};

  bool operator==(const VerificationSignature & other) const
  {
    return
      expectation == other.expectation &&
      present_in_available == other.present_in_available &&
      present_in_collision == other.present_in_collision &&
      verified == other.verified &&
      contradictory_input == other.contradictory_input;
  }

  bool operator!=(const VerificationSignature & other) const
  {
    return !(*this == other);
  }
};

class UpdatedFlowServerNode : public rclcpp::Node
{
public:
  UpdatedFlowServerNode()
  : Node("updated_flow_server_node")
  {
    manual_topic_ = declare_parameter<std::string>(
      "manual_adjustment_trajectories_topic",
      "/manual_adjustment_trajectories");

    solved_topic_ = declare_parameter<std::string>(
      "solved_collision_trajectories_topic",
      "/solved_collision_trajectories");

    collision_topic_ = declare_parameter<std::string>(
      "collision_static_trajectories_topic",
      "/collision_static_trajectories");

    available_topic_ = declare_parameter<std::string>(
      "available_static_trajectories_topic",
      "/available_static_trajectories");

    removed_topic_ = declare_parameter<std::string>(
      "removed_trajectories_topic",
      "/removed_trajectories");

    status_topic_ = declare_parameter<std::string>(
      "updated_flow_status_topic",
      "/updated_flow_status");

    publish_period_ms_ = declare_parameter<int>(
      "publish_period_ms",
      1000);

    log_state_transitions_ = declare_parameter<bool>(
      "log_state_transitions",
      true);

    validate_parameters();

    auto snapshot_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    snapshot_qos.reliable();
    snapshot_qos.transient_local();

    manual_subscription_ = create_subscription<StaticTrajectoryArray>(
      manual_topic_, snapshot_qos,
      std::bind(&UpdatedFlowServerNode::manual_callback, this, std::placeholders::_1));

    solved_subscription_ = create_subscription<StaticTrajectoryArray>(
      solved_topic_, snapshot_qos,
      std::bind(&UpdatedFlowServerNode::solved_callback, this, std::placeholders::_1));

    collision_subscription_ = create_subscription<CollisionStaticTrajectoryArray>(
      collision_topic_, snapshot_qos,
      std::bind(&UpdatedFlowServerNode::collision_callback, this, std::placeholders::_1));

    available_subscription_ = create_subscription<StaticTrajectoryArray>(
      available_topic_, snapshot_qos,
      std::bind(&UpdatedFlowServerNode::available_callback, this, std::placeholders::_1));

    removed_publisher_ = create_publisher<StaticTrajectoryArray>(
      removed_topic_, snapshot_qos);

    status_publisher_ = create_publisher<UpdatedFlowStatus>(
      status_topic_, snapshot_qos);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(&UpdatedFlowServerNode::publish_snapshots, this));

    publish_snapshots();

    RCLCPP_INFO(
      get_logger(),
      "Updated flow server ready | manual='%s' | solved='%s' | collision='%s' | "
      "available='%s' | removed='%s' | status='%s' | period=%d ms",
      manual_topic_.c_str(), solved_topic_.c_str(), collision_topic_.c_str(),
      available_topic_.c_str(), removed_topic_.c_str(), status_topic_.c_str(),
      publish_period_ms_);
  }

private:
  void validate_parameters() const
  {
    const auto absolute_topic = [](const std::string & topic) {
      return !topic.empty() && topic.front() == '/';
    };

    if (
      !absolute_topic(manual_topic_) ||
      !absolute_topic(solved_topic_) ||
      !absolute_topic(collision_topic_) ||
      !absolute_topic(available_topic_) ||
      !absolute_topic(removed_topic_) ||
      !absolute_topic(status_topic_))
    {
      throw std::runtime_error("All configured topic names must be absolute");
    }

    if (publish_period_ms_ <= 0) {
      throw std::runtime_error("publish_period_ms must be > 0");
    }
  }

  static std::map<std::string, StaticTrajectory> trajectories_by_id(
    const StaticTrajectoryArray & snapshot)
  {
    std::map<std::string, StaticTrajectory> output;
    for (const auto & trajectory : snapshot.trajectories) {
      if (!trajectory.trajectory_id.empty()) {
        output[trajectory.trajectory_id] = trajectory;
      }
    }
    return output;
  }

  static std::set<std::string> available_ids(const StaticTrajectoryArray & snapshot)
  {
    std::set<std::string> output;
    for (const auto & trajectory : snapshot.trajectories) {
      if (!trajectory.trajectory_id.empty()) {
        output.insert(trajectory.trajectory_id);
      }
    }
    return output;
  }

  static std::set<std::string> collision_ids(
    const CollisionStaticTrajectoryArray & snapshot)
  {
    std::set<std::string> output;
    for (const auto & item : snapshot.trajectories) {
      if (!item.trajectory.trajectory_id.empty()) {
        output.insert(item.trajectory.trajectory_id);
      }
    }
    return output;
  }

  void manual_callback(const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    manual_snapshot_ = *message;
    update_header(message->header);
    rebuild_verification_locked();
    publish_snapshots_locked();
  }

  void solved_callback(const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    solved_snapshot_ = *message;
    update_header(message->header);
    rebuild_verification_locked();
    publish_snapshots_locked();
  }

  void collision_callback(const CollisionStaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    collision_snapshot_ = *message;
    update_header(message->header);
    rebuild_verification_locked();
    publish_snapshots_locked();
  }

  void available_callback(const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    available_snapshot_ = *message;
    update_header(message->header);
    rebuild_verification_locked();
    publish_snapshots_locked();
  }

  void update_header(const std_msgs::msg::Header & header)
  {
    if (!header.frame_id.empty()) {
      latest_header_ = header;
    }
  }

  void log_transition_if_needed_locked(const FlowTrajectoryStatus & status)
  {
    if (!log_state_transitions_) {
      return;
    }

    VerificationSignature current;
    current.expectation = status.expectation;
    current.present_in_available = status.present_in_available;
    current.present_in_collision = status.present_in_collision;
    current.verified = status.verified;
    current.contradictory_input = status.contradictory_input;

    const auto previous = previous_status_.find(status.trajectory_id);
    if (previous != previous_status_.end() && previous->second == current) {
      return;
    }
    previous_status_[status.trajectory_id] = current;

    if (status.contradictory_input) {
      RCLCPP_ERROR(
        get_logger(),
        "Trajectory '%s' is simultaneously present in manual and solved snapshots; removal wins",
        status.trajectory_id.c_str());
      return;
    }

    if (status.verified) {
      if (status.expectation == FlowTrajectoryStatus::EXPECT_REMOVED) {
        RCLCPP_INFO(
          get_logger(),
          "REMOVED verified for '%s': absent from available and collision",
          status.trajectory_id.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(),
          "SOLVED verified for '%s': available=yes collision=no",
          status.trajectory_id.c_str());
      }
      return;
    }

    if (status.expectation == FlowTrajectoryStatus::EXPECT_REMOVED) {
      RCLCPP_WARN(
        get_logger(),
        "Removal pending for '%s' | available=%s | collision=%s",
        status.trajectory_id.c_str(),
        status.present_in_available ? "yes" : "no",
        status.present_in_collision ? "yes" : "no");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Solved validation pending for '%s' | available=%s | collision=%s",
        status.trajectory_id.c_str(),
        status.present_in_available ? "yes" : "no",
        status.present_in_collision ? "yes" : "no");
    }
  }

  void rebuild_verification_locked()
  {
    const auto manual = trajectories_by_id(manual_snapshot_);
    const auto solved = trajectories_by_id(solved_snapshot_);
    const auto available = available_ids(available_snapshot_);
    const auto collisions = collision_ids(collision_snapshot_);

    status_state_.clear();
    std::set<std::string> expected_ids;

    for (const auto & [trajectory_id, _] : manual) {
      expected_ids.insert(trajectory_id);
    }
    for (const auto & [trajectory_id, _] : solved) {
      expected_ids.insert(trajectory_id);
    }

    for (const auto & trajectory_id : expected_ids) {
      const bool expected_removed = manual.count(trajectory_id) != 0U;
      const bool expected_available = solved.count(trajectory_id) != 0U;

      FlowTrajectoryStatus status;
      status.trajectory_id = trajectory_id;
      status.present_in_available = available.count(trajectory_id) != 0U;
      status.present_in_collision = collisions.count(trajectory_id) != 0U;
      status.contradictory_input = expected_removed && expected_available;

      // Manual/removal takes precedence if upstream state is contradictory.
      if (expected_removed) {
        status.expectation = FlowTrajectoryStatus::EXPECT_REMOVED;
        status.verified =
          !status.present_in_available &&
          !status.present_in_collision;

        if (status.contradictory_input) {
          status.detail =
            "Contradictory input: manual and solved simultaneously; removal takes precedence";
        } else if (status.verified) {
          status.detail =
            "Removed from flow: absent from available and collision outputs";
        } else {
          status.detail =
            "Waiting for trajectory to disappear from available/collision outputs";
        }
      } else {
        status.expectation = FlowTrajectoryStatus::EXPECT_AVAILABLE;
        status.verified =
          status.present_in_available &&
          !status.present_in_collision;

        if (status.verified) {
          status.detail =
            "Solved trajectory accepted: available=yes collision=no";
        } else if (status.present_in_collision) {
          status.detail =
            "Solved trajectory is still reported in collision output";
        } else {
          status.detail =
            "Waiting for solved trajectory to appear in available output";
        }
      }

      log_transition_if_needed_locked(status);
      status_state_[trajectory_id] = std::move(status);
    }

    for (auto iterator = previous_status_.begin(); iterator != previous_status_.end();) {
      if (expected_ids.count(iterator->first) == 0U) {
        iterator = previous_status_.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  StaticTrajectoryArray make_removed_snapshot_locked() const
  {
    StaticTrajectoryArray output;
    output.header = manual_snapshot_.header;
    if (output.header.frame_id.empty()) {
      output.header = latest_header_;
    }
    output.header.stamp = now();
    output.trajectories = manual_snapshot_.trajectories;

    std::stable_sort(
      output.trajectories.begin(), output.trajectories.end(),
      [](const StaticTrajectory & first, const StaticTrajectory & second) {
        if (first.priority != second.priority) {
          return first.priority < second.priority;
        }
        return first.trajectory_id < second.trajectory_id;
      });

    return output;
  }

  UpdatedFlowStatus make_status_snapshot_locked() const
  {
    UpdatedFlowStatus output;
    output.header = latest_header_;
    output.header.stamp = now();
    output.removed_expected_count =
      static_cast<uint32_t>(manual_snapshot_.trajectories.size());
    output.solved_expected_count =
      static_cast<uint32_t>(solved_snapshot_.trajectories.size());
    output.trajectories.reserve(status_state_.size());

    for (const auto & [_, status] : status_state_) {
      output.trajectories.push_back(status);
      if (status.contradictory_input) {
        ++output.contradictory_count;
      }
      if (status.verified) {
        ++output.verified_count;
      } else {
        ++output.pending_count;
      }
    }
    return output;
  }

  void publish_snapshots_locked()
  {
    removed_publisher_->publish(make_removed_snapshot_locked());
    status_publisher_->publish(make_status_snapshot_locked());
  }

  void publish_snapshots()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publish_snapshots_locked();
  }

  std::string manual_topic_;
  std::string solved_topic_;
  std::string collision_topic_;
  std::string available_topic_;
  std::string removed_topic_;
  std::string status_topic_;

  int publish_period_ms_{1000};
  bool log_state_transitions_{true};

  mutable std::mutex mutex_;
  std_msgs::msg::Header latest_header_;

  StaticTrajectoryArray manual_snapshot_;
  StaticTrajectoryArray solved_snapshot_;
  CollisionStaticTrajectoryArray collision_snapshot_;
  StaticTrajectoryArray available_snapshot_;

  std::map<std::string, FlowTrajectoryStatus> status_state_;
  std::map<std::string, VerificationSignature> previous_status_;

  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr manual_subscription_;
  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr solved_subscription_;
  rclcpp::Subscription<CollisionStaticTrajectoryArray>::SharedPtr collision_subscription_;
  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr available_subscription_;

  rclcpp::Publisher<StaticTrajectoryArray>::SharedPtr removed_publisher_;
  rclcpp::Publisher<UpdatedFlowStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace updated_flow_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<updated_flow_server::UpdatedFlowServerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("updated_flow_server_node"),
      "Fatal error: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
