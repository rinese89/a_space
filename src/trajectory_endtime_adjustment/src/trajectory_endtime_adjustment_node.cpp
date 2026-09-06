#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <trajectory_endtime_adjustment/srv/register_original_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace trajectory_endtime_adjustment
{

using StaticTrajectory = static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using RegisterOriginalTrajectory =
  trajectory_endtime_adjustment::srv::RegisterOriginalTrajectory;

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

struct TrackedOccurrence
{
  StaticTrajectory trajectory;
  int64_t first_seen_ns{0};
  int64_t last_seen_ns{0};
  uint64_t snapshots_seen{0U};
};

struct SpecialTrackedOccurrence
{
  // Original trajectory received through the service. Its start/end are never
  // replaced by the delayed values published by trajectory_server_node.
  StaticTrajectory original;

  // Copy observed in /active_trajectories after trajectory_server_node delayed
  // its start. operation_start_utc is the start used to measure real duration.
  StaticTrajectory active;

  int64_t first_seen_ns{0};
  int64_t last_seen_ns{0};
  uint64_t snapshots_seen{0U};
};

static int64_t time_to_ns(const builtin_interfaces::msg::Time & value)
{
  return static_cast<int64_t>(value.sec) * kNanosecondsPerSecond +
         static_cast<int64_t>(value.nanosec);
}

static builtin_interfaces::msg::Time ns_to_time(int64_t ns)
{
  builtin_interfaces::msg::Time result;

  int64_t seconds = ns / kNanosecondsPerSecond;
  int64_t remainder = ns % kNanosecondsPerSecond;

  if (remainder < 0) {
    --seconds;
    remainder += kNanosecondsPerSecond;
  }

  if (
    seconds > static_cast<int64_t>(std::numeric_limits<int32_t>::max()) ||
    seconds < static_cast<int64_t>(std::numeric_limits<int32_t>::min()))
  {
    throw std::runtime_error(
            "UTC timestamp cannot be represented by builtin_interfaces/msg/Time");
  }

  result.sec = static_cast<int32_t>(seconds);
  result.nanosec = static_cast<uint32_t>(remainder);
  return result;
}

static int64_t system_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

static std::string trajectory_instance_key(const StaticTrajectory & trajectory)
{
  return trajectory.trajectory_id + "@" +
         std::to_string(time_to_ns(trajectory.operation_start_utc));
}

static std::string trajectory_identity_key(const StaticTrajectory & trajectory)
{
  return trajectory.trajectory_id + "|" +
         trajectory.flight_zone_id + "|" +
         trajectory.uas_namespace + "|" +
         std::to_string(trajectory.ua_id);
}

static bool valid_trajectory_identity(const StaticTrajectory & trajectory)
{
  return !trajectory.trajectory_id.empty() &&
         !trajectory.flight_zone_id.empty() &&
         !trajectory.uas_namespace.empty();
}

static bool valid_trajectory_interval(const StaticTrajectory & trajectory)
{
  return time_to_ns(trajectory.operation_end_utc) >
         time_to_ns(trajectory.operation_start_utc);
}

static std::string signed_seconds_string(double value)
{
  std::ostringstream stream;
  stream << std::showpos << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

class TrajectoryEndtimeAdjustmentNode : public rclcpp::Node
{
public:
  TrajectoryEndtimeAdjustmentNode()
  : Node("trajectory_endtime_adjustment_node")
  {
    active_trajectories_topic_ = declare_parameter<std::string>(
      "active_trajectories_topic", "/active_trajectories");

    adjusted_trajectories_topic_ = declare_parameter<std::string>(
      "adjusted_trajectories_topic", "/adjusted_trajectories");

    original_trajectory_registration_service_ = declare_parameter<std::string>(
      "original_trajectory_registration_service",
      "/trajectory_endtime_adjustment/register_original_trajectory");

    validate_parameters();

    rclcpp::QoS state_qos(rclcpp::KeepLast(1));
    state_qos.reliable();
    state_qos.transient_local();

    active_subscription_ = create_subscription<StaticTrajectoryArray>(
      active_trajectories_topic_,
      state_qos,
      std::bind(
        &TrajectoryEndtimeAdjustmentNode::active_trajectories_callback,
        this,
        std::placeholders::_1));

    adjusted_publisher_ = create_publisher<StaticTrajectoryArray>(
      adjusted_trajectories_topic_,
      state_qos);

    registration_service_ = create_service<RegisterOriginalTrajectory>(
      original_trajectory_registration_service_,
      std::bind(
        &TrajectoryEndtimeAdjustmentNode::register_original_trajectory_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Trajectory end-time adjustment ready | active='%s' | adjusted='%s' | "
      "registration_service='%s'",
      active_trajectories_topic_.c_str(),
      adjusted_trajectories_topic_.c_str(),
      original_trajectory_registration_service_.c_str());
  }

private:
  static void validate_absolute_name(
    const std::string & value,
    const std::string & parameter_name)
  {
    if (
      value.empty() ||
      value.front() != '/' ||
      value.size() == 1U ||
      value.back() == '/' ||
      value.find("//") != std::string::npos)
    {
      throw std::runtime_error(
              "Parameter '" + parameter_name +
              "' must be a valid absolute ROS name");
    }
  }

  void validate_parameters() const
  {
    validate_absolute_name(
      active_trajectories_topic_, "active_trajectories_topic");
    validate_absolute_name(
      adjusted_trajectories_topic_, "adjusted_trajectories_topic");
    validate_absolute_name(
      original_trajectory_registration_service_,
      "original_trajectory_registration_service");

    if (active_trajectories_topic_ == adjusted_trajectories_topic_) {
      throw std::runtime_error(
              "active_trajectories_topic and adjusted_trajectories_topic "
              "must be different");
    }
  }

  void register_original_trajectory_callback(
    const std::shared_ptr<RegisterOriginalTrajectory::Request> request,
    std::shared_ptr<RegisterOriginalTrajectory::Response> response)
  {
    if (!request || !response) {
      return;
    }

    const auto & trajectory = request->trajectory;

    if (!valid_trajectory_identity(trajectory) || !valid_trajectory_interval(trajectory)) {
      response->accepted = false;
      response->message = "Trajectory identity or interval is invalid";
      return;
    }

    const std::string key = trajectory_instance_key(trajectory);

    {
      std::lock_guard<std::mutex> lock(mutex_);

      // Idempotent. A retry refreshes the original definition without creating
      // another pending special occurrence.
      registered_originals_[key] = trajectory;
    }

    response->accepted = true;
    response->message = "Original trajectory stored";

    RCLCPP_INFO(
      get_logger(),
      "Stored original priority trajectory '%s' | missions=%zu | "
      "original_start=%d.%09u | original_end=%d.%09u",
      trajectory.trajectory_id.c_str(),
      trajectory.mission.size(),
      trajectory.operation_start_utc.sec,
      trajectory.operation_start_utc.nanosec,
      trajectory.operation_end_utc.sec,
      trajectory.operation_end_utc.nanosec);
  }

  std::unordered_map<std::string, StaticTrajectory>::iterator
  find_registered_original_locked(const StaticTrajectory & active)
  {
    const std::string identity = trajectory_identity_key(active);
    const int64_t active_start_ns = time_to_ns(active.operation_start_utc);

    auto best = registered_originals_.end();
    int64_t best_original_start_ns = std::numeric_limits<int64_t>::min();

    for (auto iterator = registered_originals_.begin();
      iterator != registered_originals_.end();
      ++iterator)
    {
      const auto & original = iterator->second;

      if (trajectory_identity_key(original) != identity) {
        continue;
      }

      const int64_t original_start_ns =
        time_to_ns(original.operation_start_utc);

      // The server may delay this occurrence but must not move it to a time
      // earlier than its original start.
      if (original_start_ns > active_start_ns) {
        continue;
      }

      // If more than one original registration exists for the same identity,
      // choose the newest original occurrence that could have produced this
      // delayed active occurrence.
      if (original_start_ns > best_original_start_ns) {
        best = iterator;
        best_original_start_ns = original_start_ns;
      }
    }

    return best;
  }

  StaticTrajectory build_normal_adjustment(
    const TrackedOccurrence & tracked,
    int64_t actual_end_ns) const
  {
    const int64_t planned_end_ns =
      time_to_ns(tracked.trajectory.operation_end_utc);

    const double extra_time_s =
      static_cast<double>(actual_end_ns - planned_end_ns) /
      static_cast<double>(kNanosecondsPerSecond);

    // StaticTrajectory is copied as a whole. With the multi-mission interface,
    // this preserves takeoff, every mission[i], landing and all metadata
    // exactly. This node changes timing fields only.
    StaticTrajectory adjusted = tracked.trajectory;
    adjusted.extra_time = extra_time_s;
    adjusted.operation_end_utc = ns_to_time(actual_end_ns);
    return adjusted;
  }

  StaticTrajectory build_special_duration_adjustment(
    const SpecialTrackedOccurrence & tracked,
    int64_t actual_active_end_ns) const
  {
    const int64_t original_start_ns =
      time_to_ns(tracked.original.operation_start_utc);
    const int64_t original_end_ns =
      time_to_ns(tracked.original.operation_end_utc);

    const int64_t active_start_ns =
      time_to_ns(tracked.active.operation_start_utc);

    const int64_t original_duration_ns =
      original_end_ns - original_start_ns;

    // The real active duration starts at the delayed START published by
    // trajectory_server_node and finishes when the trajectory disappears from
    // the authoritative /active_trajectories snapshot.
    const int64_t active_duration_ns =
      actual_active_end_ns - active_start_ns;

    const int64_t extra_ns =
      active_duration_ns - original_duration_ns;

    if (
      (extra_ns > 0 &&
      original_end_ns > std::numeric_limits<int64_t>::max() - extra_ns) ||
      (extra_ns < 0 &&
      original_end_ns < std::numeric_limits<int64_t>::min() - extra_ns))
    {
      throw std::runtime_error(
              "Special duration adjustment exceeds int64 UTC range");
    }

    // Preserve the complete ORIGINAL geometry, including every independent
    // mission[i]. The special adjustment changes only timing metadata.
    StaticTrajectory adjusted = tracked.original;

    // IMPORTANT:
    // operation_start_utc remains the ORIGINAL value received by service.
    //
    // extra_time is independent of the calendar delay suffered before the
    // active occurrence started:
    //
    //   extra_time =
    //     (actual_active_end - active_start)
    //     -
    //     (original_end - original_start)
    //
    // Therefore:
    //   original 10:05:00 -> 10:06:00  = 60 s
    //   active   10:05:25 -> 10:06:25  = 60 s
    //   extra_time = 0 s
    //
    // while:
    //   active   10:05:25 -> 10:06:35  = 70 s
    //   extra_time = +10 s
    adjusted.extra_time =
      static_cast<double>(extra_ns) /
      static_cast<double>(kNanosecondsPerSecond);

    adjusted.operation_end_utc =
      ns_to_time(original_end_ns + extra_ns);

    return adjusted;
  }

  void active_trajectories_callback(
    const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }

    const int64_t observation_ns = system_now_ns();

    std::unordered_set<std::string> present_instances;
    present_instances.reserve(message->trajectories.size());

    std::vector<StaticTrajectory> adjustments_to_publish;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      latest_frame_id_ = message->header.frame_id;

      // ================================================================
      // A. Refresh/create state for trajectories PRESENT in ACTIVE.
      // ================================================================
      for (const auto & trajectory : message->trajectories) {
        if (!valid_trajectory_identity(trajectory)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring active trajectory with incomplete identity");
          continue;
        }

        if (!valid_trajectory_interval(trajectory)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring active trajectory '%s' because operation_end_utc <= "
            "operation_start_utc",
            trajectory.trajectory_id.c_str());
          continue;
        }

        const std::string active_key =
          trajectory_instance_key(trajectory);

        present_instances.insert(active_key);

        if (completed_instances_.find(active_key) != completed_instances_.end()) {
          continue;
        }

        // --------------------------------------------------------------
        // Special priority-delayed occurrence.
        //
        // The service registration does NOT generate extra_time.
        // The first ACTIVE snapshot only binds:
        //
        //   original trajectory <-> delayed active occurrence
        //
        // and stores active.operation_start_utc.
        // --------------------------------------------------------------
        auto special_it = special_tracked_.find(active_key);

        if (special_it != special_tracked_.end()) {
          special_it->second.active = trajectory;
          special_it->second.last_seen_ns = observation_ns;
          ++special_it->second.snapshots_seen;
          continue;
        }

        auto original_it = find_registered_original_locked(trajectory);

        if (original_it != registered_originals_.end()) {
          SpecialTrackedOccurrence tracked;
          tracked.original = original_it->second;
          tracked.active = trajectory;
          tracked.first_seen_ns = observation_ns;
          tracked.last_seen_ns = observation_ns;
          tracked.snapshots_seen = 1U;

          const int64_t original_start_ns =
            time_to_ns(tracked.original.operation_start_utc);
          const int64_t active_start_ns =
            time_to_ns(tracked.active.operation_start_utc);

          special_tracked_[active_key] = std::move(tracked);
          registered_originals_.erase(original_it);

          RCLCPP_INFO(
            get_logger(),
            "Bound delayed priority trajectory '%s' to its original schedule | "
            "calendar start delay=%s s | waiting for ACTIVE disappearance to "
            "measure actual duration",
            trajectory.trajectory_id.c_str(),
            signed_seconds_string(
              static_cast<double>(active_start_ns - original_start_ns) /
              static_cast<double>(kNanosecondsPerSecond)).c_str());

          continue;
        }

        // --------------------------------------------------------------
        // Normal trajectory.
        // --------------------------------------------------------------
        auto [iterator, inserted] =
          tracked_.try_emplace(active_key);

        auto & tracked = iterator->second;

        if (inserted) {
          tracked.trajectory = trajectory;
          tracked.first_seen_ns = observation_ns;
          tracked.last_seen_ns = observation_ns;
          tracked.snapshots_seen = 1U;

          RCLCPP_INFO(
            get_logger(),
            "Tracking normal active occurrence '%s' | missions=%zu | "
            "planned_end=%d.%09u",
            active_key.c_str(),
            trajectory.mission.size(),
            trajectory.operation_end_utc.sec,
            trajectory.operation_end_utc.nanosec);
        } else {
          tracked.trajectory = trajectory;
          tracked.last_seen_ns = observation_ns;
          ++tracked.snapshots_seen;
        }
      }

      // ================================================================
      // B. Special trajectories that DISAPPEARED from ACTIVE.
      // ================================================================
      for (auto iterator = special_tracked_.begin();
        iterator != special_tracked_.end();)
      {
        if (present_instances.find(iterator->first) != present_instances.end()) {
          ++iterator;
          continue;
        }

        try {
          StaticTrajectory adjusted =
            build_special_duration_adjustment(
              iterator->second,
              observation_ns);

          const int64_t original_duration_ns =
            time_to_ns(iterator->second.original.operation_end_utc) -
            time_to_ns(iterator->second.original.operation_start_utc);

          const int64_t active_duration_ns =
            observation_ns -
            time_to_ns(iterator->second.active.operation_start_utc);

          adjustments_to_publish.push_back(std::move(adjusted));
          completed_instances_.insert(iterator->first);

          RCLCPP_INFO(
            get_logger(),
            "Delayed priority occurrence '%s' completed | "
            "original_duration=%.3f s | active_duration=%.3f s | "
            "extra_time=%s s | ORIGINAL start preserved",
            iterator->second.original.trajectory_id.c_str(),
            static_cast<double>(original_duration_ns) /
            static_cast<double>(kNanosecondsPerSecond),
            static_cast<double>(active_duration_ns) /
            static_cast<double>(kNanosecondsPerSecond),
            signed_seconds_string(
              static_cast<double>(active_duration_ns - original_duration_ns) /
              static_cast<double>(kNanosecondsPerSecond)).c_str());
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            get_logger(),
            "Cannot build special duration adjustment for '%s': %s",
            iterator->second.original.trajectory_id.c_str(),
            error.what());
        }

        iterator = special_tracked_.erase(iterator);
      }

      // ================================================================
      // C. Normal trajectories that DISAPPEARED from ACTIVE.
      // ================================================================
      for (auto iterator = tracked_.begin();
        iterator != tracked_.end();)
      {
        if (present_instances.find(iterator->first) != present_instances.end()) {
          ++iterator;
          continue;
        }

        StaticTrajectory adjusted =
          build_normal_adjustment(
            iterator->second,
            observation_ns);

        adjustments_to_publish.push_back(std::move(adjusted));
        completed_instances_.insert(iterator->first);

        const int64_t planned_end_ns =
          time_to_ns(iterator->second.trajectory.operation_end_utc);

        const double extra_time_s =
          static_cast<double>(observation_ns - planned_end_ns) /
          static_cast<double>(kNanosecondsPerSecond);

        RCLCPP_INFO(
          get_logger(),
          "Normal occurrence '%s' disappeared from /active_trajectories | "
          "extra_time=%s s | snapshots=%llu",
          iterator->first.c_str(),
          signed_seconds_string(extra_time_s).c_str(),
          static_cast<unsigned long long>(
            iterator->second.snapshots_seen));

        iterator = tracked_.erase(iterator);
      }
    }

    if (adjustments_to_publish.empty()) {
      return;
    }

    StaticTrajectoryArray output;
    output.header = message->header;
    output.header.stamp = now();

    if (output.header.frame_id.empty()) {
      output.header.frame_id = latest_frame_id_;
    }

    output.trajectories =
      std::move(adjustments_to_publish);

    adjusted_publisher_->publish(output);

    RCLCPP_INFO(
      get_logger(),
      "Published %zu multi-mission-preserving adjustment(s) to '%s'",
      output.trajectories.size(),
      adjusted_trajectories_topic_.c_str());
  }

  std::string active_trajectories_topic_;
  std::string adjusted_trajectories_topic_;
  std::string original_trajectory_registration_service_;

  mutable std::mutex mutex_;

  // Normal active occurrences.
  std::unordered_map<std::string, TrackedOccurrence> tracked_;

  // Original trajectories registered by trajectory_server_node, keyed by
  // trajectory_id@ORIGINAL_START.
  std::unordered_map<std::string, StaticTrajectory> registered_originals_;

  // Delayed active occurrences linked to their corresponding original
  // trajectories, keyed by trajectory_id@ACTIVE_DELAYED_START.
  std::unordered_map<std::string, SpecialTrackedOccurrence> special_tracked_;

  // Protect against stale/replayed active snapshots.
  std::unordered_set<std::string> completed_instances_;

  std::string latest_frame_id_;

  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr active_subscription_;
  rclcpp::Publisher<StaticTrajectoryArray>::SharedPtr adjusted_publisher_;
  rclcpp::Service<RegisterOriginalTrajectory>::SharedPtr registration_service_;
};

}  // namespace trajectory_endtime_adjustment

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        trajectory_endtime_adjustment::TrajectoryEndtimeAdjustmentNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("trajectory_endtime_adjustment_node"),
      "Fatal error: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
