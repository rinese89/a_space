#include <rclcpp/rclcpp.hpp>

#include <collision_detection/msg/detected_collision_trajectory.hpp>
#include <collision_detection/msg/detected_collision_trajectory_array.hpp>
#include <collision_detection/msg/grid_collision_node.hpp>
#include <collision_detection/msg/grid_collision_segment.hpp>
#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
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

namespace supervision_trajectory_manager
{

using DetectedCollisionTrajectory = collision_detection::msg::DetectedCollisionTrajectory;
using DetectedCollisionTrajectoryArray = collision_detection::msg::DetectedCollisionTrajectoryArray;
using GridCollisionNode = collision_detection::msg::GridCollisionNode;
using GridCollisionSegment = collision_detection::msg::GridCollisionSegment;
using StaticTrajectory = static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using TrajectorySegment = static_trajectory_manager::msg::TrajectorySegment;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

constexpr uint8_t kTakeoff = 0U;
constexpr uint8_t kMission = 1U;
constexpr uint8_t kLanding = 2U;

struct PhaseIndex
{
  uint8_t phase{0U};
  uint32_t index{0U};
  bool operator<(const PhaseIndex & other) const
  {
    return std::tie(phase, index) < std::tie(other.phase, other.index);
  }
};

struct TaggedPointRef
{
  uint8_t phase{0U};
  uint32_t raw_index{0U};
  uint32_t repetition{0U};
};

struct CropMask
{
  std::array<std::set<uint32_t>, 3U> forbidden_segments;
  std::array<std::set<uint32_t>, 3U> forbidden_points;
};

enum class StoredState : uint8_t
{
  PENDING = 0U,
  SUPERVISED = 1U
};

struct StoredEntry
{
  DetectedCollisionTrajectory request;
  StaticTrajectory adjusted;
  StoredState state{StoredState::PENDING};
};

static std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F)
{
  std_msgs::msg::ColorRGBA out;
  out.r = r; out.g = g; out.b = b; out.a = a;
  return out;
}

static std::string join_ids(const std::vector<std::string> & ids)
{
  std::ostringstream ss;
  for (std::size_t i = 0U; i < ids.size(); ++i) {
    if (i != 0U) { ss << ","; }
    ss << ids[i];
  }
  return ss.str();
}

class SupervisionTrajectoryManagerNode : public rclcpp::Node
{
public:
  SupervisionTrajectoryManagerNode()
  : Node("supervision_trajectory_manager_node")
  {
    requested_topic_ = declare_parameter<std::string>(
      "requested_supervision_trajectories_topic", "/requested_supervision_trajectories");
    available_topic_ = declare_parameter<std::string>(
      "available_trajectories_topic", "/available_static_trajectories");
    adjusted_topic_ = declare_parameter<std::string>(
      "adjusted_trajectories_topic", "/adjusted_trajectories");
    supervised_topic_ = declare_parameter<std::string>(
      "supervised_trajectories_topic", "/supervised_trajectories");
    markers_topic_ = declare_parameter<std::string>(
      "supervision_markers_topic", "/supervision_trajectory_manager_markers");
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 1000);
    node_position_tolerance_m_ = declare_parameter<double>("node_position_tolerance_m", 0.75);
    geometry_match_tolerance_m_ = declare_parameter<double>("geometry_match_tolerance_m", 1.0e-6);
    pending_line_width_ = declare_parameter<double>("pending_line_width", 0.12);
    supervised_node_scale_ = declare_parameter<double>("supervised_node_scale", 0.32);
    supervised_text_height_ = declare_parameter<double>("supervised_text_height", 0.28);
    validate_parameters();

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    requested_sub_ = create_subscription<DetectedCollisionTrajectoryArray>(
      requested_topic_, qos,
      std::bind(&SupervisionTrajectoryManagerNode::requested_callback, this, std::placeholders::_1));
    available_sub_ = create_subscription<StaticTrajectoryArray>(
      available_topic_, qos,
      std::bind(&SupervisionTrajectoryManagerNode::available_callback, this, std::placeholders::_1));
    adjusted_pub_ = create_publisher<StaticTrajectoryArray>(adjusted_topic_, qos);
    supervised_pub_ = create_publisher<DetectedCollisionTrajectoryArray>(supervised_topic_, qos);
    markers_pub_ = create_publisher<MarkerArray>(markers_topic_, qos);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(&SupervisionTrajectoryManagerNode::publish, this));

    publish();
    RCLCPP_INFO(
      get_logger(),
      "Supervision trajectory manager | request='%s' available='%s' adjusted='%s' supervised='%s' @ %.3f Hz",
      requested_topic_.c_str(), available_topic_.c_str(), adjusted_topic_.c_str(),
      supervised_topic_.c_str(), 1000.0 / static_cast<double>(publish_period_ms_));
  }

private:
  void validate_parameters() const
  {
    const auto absolute = [](const std::string & s) { return !s.empty() && s.front() == '/'; };
    if (!absolute(requested_topic_) || !absolute(available_topic_) || !absolute(adjusted_topic_) ||
        !absolute(supervised_topic_) || !absolute(markers_topic_)) {
      throw std::runtime_error("All topics must be absolute");
    }
    if (publish_period_ms_ <= 0) { throw std::runtime_error("publish_period_ms must be > 0"); }
    if (!std::isfinite(node_position_tolerance_m_) || node_position_tolerance_m_ < 0.0 ||
        !std::isfinite(geometry_match_tolerance_m_) || geometry_match_tolerance_m_ < 0.0) {
      throw std::runtime_error("Geometry tolerances must be finite and >= 0");
    }
    if (!std::isfinite(pending_line_width_) || pending_line_width_ <= 0.0 ||
        !std::isfinite(supervised_node_scale_) || supervised_node_scale_ <= 0.0 ||
        !std::isfinite(supervised_text_height_) || supervised_text_height_ <= 0.0) {
      throw std::runtime_error("Marker dimensions must be finite and > 0");
    }
  }

  static std::size_t count(const TrajectorySegment & s)
  {
    return std::min(s.x.size(), std::min(s.y.size(), s.z.size()));
  }

  static geometry_msgs::msg::Point point(const TrajectorySegment & s, std::size_t i)
  {
    geometry_msgs::msg::Point p;
    p.x = s.x[i]; p.y = s.y[i]; p.z = s.z[i];
    return p;
  }

  static double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static const TrajectorySegment & phase(const StaticTrajectory & t, uint8_t p)
  {
    if (p == kTakeoff) { return t.takeoff; }
    if (p == kMission) { return t.mission; }
    return t.landing;
  }

  static void append_refs(
    std::vector<TaggedPointRef> & out, const TrajectorySegment & s,
    uint8_t p, uint32_t repetition)
  {
    for (uint32_t i = 0U; i < static_cast<uint32_t>(count(s)); ++i) {
      out.push_back(TaggedPointRef{p, i, repetition});
    }
  }

  CropMask build_crop_mask(const DetectedCollisionTrajectory & request) const
  {
    CropMask mask;
    std::set<PhaseIndex> collided;
    for (const auto & s : request.collision_segments) {
      if (s.phase <= kLanding) {
        collided.insert(PhaseIndex{s.phase, s.original_phase_segment_index});
      }
    }

    std::vector<TaggedPointRef> refs;
    append_refs(refs, request.trajectory.takeoff, kTakeoff, 0U);
    for (uint32_t r = 0U; r < std::max<uint32_t>(1U, request.trajectory.repetitions); ++r) {
      append_refs(refs, request.trajectory.mission, kMission, r);
    }
    append_refs(refs, request.trajectory.landing, kLanding, 0U);

    std::array<uint32_t, 3U> phase_index{0U, 0U, 0U};
    for (std::size_t i = 1U; i < refs.size(); ++i) {
      const auto & a = refs[i - 1U];
      const auto & b = refs[i];
      const uint8_t edge_phase = (a.phase == b.phase) ? b.phase : b.phase;
      if (edge_phase > kLanding) { continue; }
      const uint32_t index = phase_index.at(static_cast<std::size_t>(edge_phase))++;
      if (collided.count(PhaseIndex{edge_phase, index}) == 0U) { continue; }

      const bool same_instance =
        a.phase == b.phase && (edge_phase != kMission || a.repetition == b.repetition);
      const bool raw_edge = same_instance && b.raw_index == a.raw_index + 1U;

      if (raw_edge) {
        mask.forbidden_segments.at(static_cast<std::size_t>(edge_phase)).insert(a.raw_index);
      } else {
        if (a.phase <= kLanding) {
          mask.forbidden_points.at(static_cast<std::size_t>(a.phase)).insert(a.raw_index);
        }
        if (b.phase <= kLanding) {
          mask.forbidden_points.at(static_cast<std::size_t>(b.phase)).insert(b.raw_index);
        }
      }
    }

    // Node-only collision support: map a lattice collision node back to original
    // points by spatial proximity. This is conservative because lattice points
    // are snapped representations of the continuous trajectory.
    for (uint8_t p = kTakeoff; p <= kLanding; ++p) {
      const auto & s = phase(request.trajectory, p);
      for (uint32_t i = 0U; i < static_cast<uint32_t>(count(s)); ++i) {
        const auto original = point(s, i);
        for (const auto & collision_node : request.collision_nodes) {
          if (distance(original, collision_node.position) <= node_position_tolerance_m_) {
            mask.forbidden_points.at(static_cast<std::size_t>(p)).insert(i);
            break;
          }
        }
      }
    }

    return mask;
  }

  static TrajectorySegment crop_phase(
    const TrajectorySegment & source,
    const std::set<uint32_t> & forbidden_segments,
    const std::set<uint32_t> & forbidden_points)
  {
    TrajectorySegment out;
    const std::size_t n = count(source);
    if (n == 0U) { return out; }

    std::vector<std::vector<uint32_t>> runs;
    std::vector<uint32_t> current;

    for (uint32_t i = 0U; i < static_cast<uint32_t>(n); ++i) {
      if (forbidden_points.count(i) != 0U) {
        if (!current.empty()) { runs.push_back(current); current.clear(); }
        continue;
      }

      if (current.empty()) {
        current.push_back(i);
      } else {
        const uint32_t previous = current.back();
        const bool adjacent = i == previous + 1U;
        const bool edge_allowed = adjacent && forbidden_segments.count(previous) == 0U;
        if (!edge_allowed) { runs.push_back(current); current.clear(); }
        current.push_back(i);
      }

      if (i + 1U < n && forbidden_segments.count(i) != 0U) {
        runs.push_back(current);
        current.clear();
      }
    }
    if (!current.empty()) { runs.push_back(current); }

    const std::vector<uint32_t> * best = nullptr;
    for (const auto & run : runs) {
      if (best == nullptr || run.size() > best->size()) { best = &run; }
    }
    if (best == nullptr) { return out; }
    if (n >= 2U && best->size() < 2U) { return out; }

    out.x.reserve(best->size());
    out.y.reserve(best->size());
    out.z.reserve(best->size());
    for (const uint32_t i : *best) {
      out.x.push_back(source.x[i]);
      out.y.push_back(source.y[i]);
      out.z.push_back(source.z[i]);
    }
    return out;
  }

  StaticTrajectory adjusted(const DetectedCollisionTrajectory & request) const
  {
    StaticTrajectory out = request.trajectory;
    const CropMask mask = build_crop_mask(request);
    out.takeoff = crop_phase(
      request.trajectory.takeoff, mask.forbidden_segments[0], mask.forbidden_points[0]);
    out.mission = crop_phase(
      request.trajectory.mission, mask.forbidden_segments[1], mask.forbidden_points[1]);
    out.landing = crop_phase(
      request.trajectory.landing, mask.forbidden_segments[2], mask.forbidden_points[2]);
    return out;
  }

  static bool same_segment(const TrajectorySegment & a, const TrajectorySegment & b, double tol)
  {
    if (a.x.size() != b.x.size() || a.y.size() != b.y.size() || a.z.size() != b.z.size()) {
      return false;
    }
    for (std::size_t i = 0U; i < a.x.size(); ++i) {
      if (std::abs(a.x[i] - b.x[i]) > tol || std::abs(a.y[i] - b.y[i]) > tol ||
          std::abs(a.z[i] - b.z[i]) > tol) {
        return false;
      }
    }
    return true;
  }

  bool same_geometry(const StaticTrajectory & expected, const StaticTrajectory & observed) const
  {
    return expected.trajectory_id == observed.trajectory_id &&
      expected.frame_id == observed.frame_id &&
      same_segment(expected.takeoff, observed.takeoff, geometry_match_tolerance_m_) &&
      same_segment(expected.mission, observed.mission, geometry_match_tolerance_m_) &&
      same_segment(expected.landing, observed.landing, geometry_match_tolerance_m_);
  }

  void store_request(const DetectedCollisionTrajectory & request)
  {
    const std::string & id = request.trajectory.trajectory_id;
    if (id.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring supervision request with empty trajectory_id");
      return;
    }

    const StaticTrajectory new_adjusted = adjusted(request);
    auto it = stored_.find(id);
    if (it == stored_.end()) {
      StoredEntry entry;
      entry.request = request;
      entry.adjusted = new_adjusted;
      entry.state = StoredState::PENDING;
      stored_[id] = std::move(entry);
      RCLCPP_INFO(
        get_logger(), "Stored supervision request '%s' | nodes=%zu segments=%zu",
        id.c_str(), request.collision_nodes.size(), request.collision_segments.size());
      return;
    }

    const bool changed = !same_geometry(it->second.adjusted, new_adjusted);
    it->second.request = request;
    if (changed) {
      it->second.adjusted = new_adjusted;
      it->second.state = StoredState::PENDING;
      RCLCPP_INFO(get_logger(), "Adjusted geometry changed for '%s'; returned to PENDING", id.c_str());
    }
  }

  void requested_callback(const DetectedCollisionTrajectoryArray::SharedPtr msg)
  {
    if (!msg) { return; }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_header_ = msg->header;
    for (const auto & request : msg->trajectories) { store_request(request); }
    verify_available();
    publish_locked();
  }

  void available_callback(const StaticTrajectoryArray::SharedPtr msg)
  {
    if (!msg) { return; }
    std::lock_guard<std::mutex> lock(mutex_);
    available_ = *msg;
    available_received_ = true;
    if (!msg->header.frame_id.empty()) { latest_header_ = msg->header; }
    verify_available();
    publish_locked();
  }

  void verify_available()
  {
    if (!available_received_) { return; }
    std::map<std::string, const StaticTrajectory *> by_id;
    for (const auto & t : available_.trajectories) {
      if (!t.trajectory_id.empty()) { by_id[t.trajectory_id] = &t; }
    }

    for (auto & [id, entry] : stored_) {
      const auto it = by_id.find(id);
      const bool available_match =
        it != by_id.end() && it->second != nullptr && same_geometry(entry.adjusted, *it->second);

      if (entry.state == StoredState::PENDING) {
        if (!available_match) {
          if (it != by_id.end() && it->second != nullptr) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "'%s' is AVAILABLE by id, but its geometry does not yet match the adjusted version",
              id.c_str());
          }
          continue;
        }

        entry.state = StoredState::SUPERVISED;
        RCLCPP_INFO(get_logger(), "'%s' confirmed AVAILABLE; supervision activated", id.c_str());
        continue;
      }

      // A supervised trajectory is intentionally fixed in the available flow.
      // If it disappears or another geometry replaces it, queue the stored
      // adjusted version again through /adjusted_trajectories.
      if (!available_match) {
        entry.state = StoredState::PENDING;
        RCLCPP_WARN(
          get_logger(),
          "Supervised trajectory '%s' is no longer AVAILABLE with the expected geometry; re-queued",
          id.c_str());
      }
    }
  }

  StaticTrajectoryArray adjusted_snapshot() const
  {
    StaticTrajectoryArray out;
    out.header = latest_header_;
    out.header.stamp = now();
    std::vector<const StoredEntry *> ordered;
    for (const auto & [_, entry] : stored_) {
      if (entry.state == StoredState::PENDING) { ordered.push_back(&entry); }
    }
    std::stable_sort(ordered.begin(), ordered.end(), [](const StoredEntry * a, const StoredEntry * b) {
      if (a->adjusted.priority != b->adjusted.priority) { return a->adjusted.priority < b->adjusted.priority; }
      if (a->adjusted.ua_id != b->adjusted.ua_id) { return a->adjusted.ua_id < b->adjusted.ua_id; }
      return a->adjusted.trajectory_id < b->adjusted.trajectory_id;
    });
    for (const auto * entry : ordered) { out.trajectories.push_back(entry->adjusted); }
    return out;
  }

  DetectedCollisionTrajectoryArray supervised_snapshot() const
  {
    DetectedCollisionTrajectoryArray out;
    out.header = latest_header_;
    out.header.stamp = now();

    std::vector<const StoredEntry *> ordered;
    for (const auto & [_, entry] : stored_) {
      if (entry.state == StoredState::SUPERVISED) {
        ordered.push_back(&entry);
      }
    }

    std::stable_sort(
      ordered.begin(), ordered.end(),
      [](const StoredEntry * a, const StoredEntry * b) {
        if (a->request.trajectory.priority != b->request.trajectory.priority) {
          return a->request.trajectory.priority < b->request.trajectory.priority;
        }
        if (a->request.trajectory.ua_id != b->request.trajectory.ua_id) {
          return a->request.trajectory.ua_id < b->request.trajectory.ua_id;
        }
        return a->request.trajectory.trajectory_id < b->request.trajectory.trajectory_id;
      });

    out.trajectories.reserve(ordered.size());
    for (const auto * entry : ordered) {
      // Publish the ORIGINAL DetectedCollisionTrajectory received from
      // /requested_supervision_trajectories. The adjusted/cropped geometry is
      // used only for /adjusted_trajectories and AVAILABLE verification.
      out.trajectories.push_back(entry->request);
    }

    return out;
  }

  static std::vector<geometry_msgs::msg::Point> expanded_points(const StaticTrajectory & t)
  {
    std::vector<geometry_msgs::msg::Point> out;
    const auto append = [&out](const TrajectorySegment & s) {
      for (std::size_t i = 0U; i < count(s); ++i) { out.push_back(point(s, i)); }
    };
    append(t.takeoff);
    for (uint32_t r = 0U; r < std::max<uint32_t>(1U, t.repetitions); ++r) { append(t.mission); }
    append(t.landing);
    return out;
  }

  MarkerArray markers() const
  {
    MarkerArray out;
    Marker del;
    del.header.stamp = now();
    del.header.frame_id = latest_header_.frame_id;
    del.action = Marker::DELETEALL;
    out.markers.push_back(del);
    int marker_id = 1;

    for (const auto & [id, entry] : stored_) {
      if (entry.state == StoredState::PENDING) {
        const auto pts = expanded_points(entry.adjusted);
        if (pts.size() >= 2U) {
          Marker line;
          line.header.stamp = now();
          line.header.frame_id = entry.adjusted.frame_id;
          line.ns = "supervision/pending/" + id;
          line.id = marker_id++;
          line.type = Marker::LINE_STRIP;
          line.action = Marker::ADD;
          line.pose.orientation.w = 1.0;
          line.scale.x = pending_line_width_;
          line.color = color(0.10F, 0.75F, 1.00F, 1.00F);
          line.points = pts;
          out.markers.push_back(std::move(line));
        }
        continue;
      }

      const auto c = color(1.00F, 0.64F, 0.05F, 1.00F);
      for (const auto & n : entry.request.collision_nodes) {
        Marker sphere;
        sphere.header.stamp = now();
        sphere.header.frame_id = entry.request.trajectory.frame_id;
        sphere.ns = "supervision/collision_nodes/" + id;
        sphere.id = marker_id++;
        sphere.type = Marker::SPHERE;
        sphere.action = Marker::ADD;
        sphere.pose.orientation.w = 1.0;
        sphere.pose.position = n.position;
        sphere.scale.x = supervised_node_scale_;
        sphere.scale.y = supervised_node_scale_;
        sphere.scale.z = supervised_node_scale_;
        sphere.color = c;
        out.markers.push_back(std::move(sphere));

        Marker text;
        text.header.stamp = now();
        text.header.frame_id = entry.request.trajectory.frame_id;
        text.ns = "supervision/collision_nodes/" + id;
        text.id = marker_id++;
        text.type = Marker::TEXT_VIEW_FACING;
        text.action = Marker::ADD;
        text.pose.orientation.w = 1.0;
        text.pose.position = n.position;
        text.pose.position.z += supervised_node_scale_ * 0.85;
        text.scale.z = supervised_text_height_;
        text.color = c;
        text.text = id + " | node=" + std::to_string(n.node_id) + " | vs=" + join_ids(n.conflicting_trajectory_ids);
        out.markers.push_back(std::move(text));
      }
    }
    return out;
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

  std::string requested_topic_, available_topic_, adjusted_topic_, supervised_topic_, markers_topic_;
  int publish_period_ms_{1000};
  double node_position_tolerance_m_{0.75};
  double geometry_match_tolerance_m_{1.0e-6};
  double pending_line_width_{0.12};
  double supervised_node_scale_{0.32};
  double supervised_text_height_{0.28};

  mutable std::mutex mutex_;
  std_msgs::msg::Header latest_header_;
  StaticTrajectoryArray available_;
  bool available_received_{false};
  std::map<std::string, StoredEntry> stored_;

  rclcpp::Subscription<DetectedCollisionTrajectoryArray>::SharedPtr requested_sub_;
  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr available_sub_;
  rclcpp::Publisher<StaticTrajectoryArray>::SharedPtr adjusted_pub_;
  rclcpp::Publisher<DetectedCollisionTrajectoryArray>::SharedPtr supervised_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace supervision_trajectory_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<supervision_trajectory_manager::SupervisionTrajectoryManagerNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("supervision_trajectory_manager_node"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
