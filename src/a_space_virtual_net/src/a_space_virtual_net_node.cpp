#include <rclcpp/rclcpp.hpp>

#include <a_space_virtual_net/msg/net_loaded_trajectory.hpp>
#include <a_space_virtual_net/msg/net_loaded_trajectory_array.hpp>
#include <a_space_virtual_net/msg/net_trajectory_segment.hpp>
#include <a_space_virtual_net/msg/virtual_net_edge.hpp>
#include <a_space_virtual_net/msg/virtual_net_state.hpp>

#include <static_trajectory_manager/msg/static_trajectory.hpp>
#include <static_trajectory_manager/msg/static_trajectory_array.hpp>
#include <static_trajectory_manager/msg/trajectory_segment.hpp>

#include <static_trajectory_conflict_manager/msg/collision_static_trajectory_array.hpp>
#include <static_trajectory_conflict_manager/msg/segment_collision.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace a_space_virtual_net
{

using NetLoadedTrajectory = a_space_virtual_net::msg::NetLoadedTrajectory;
using NetLoadedTrajectoryArray = a_space_virtual_net::msg::NetLoadedTrajectoryArray;
using NetTrajectorySegment = a_space_virtual_net::msg::NetTrajectorySegment;
using VirtualNetEdge = a_space_virtual_net::msg::VirtualNetEdge;
using VirtualNetState = a_space_virtual_net::msg::VirtualNetState;

using StaticTrajectory = static_trajectory_manager::msg::StaticTrajectory;
using StaticTrajectoryArray = static_trajectory_manager::msg::StaticTrajectoryArray;
using TrajectorySegment = static_trajectory_manager::msg::TrajectorySegment;
using CollisionStaticTrajectoryArray =
  static_trajectory_conflict_manager::msg::CollisionStaticTrajectoryArray;
using SegmentCollision = static_trajectory_conflict_manager::msg::SegmentCollision;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

constexpr double kEpsilon = 1.0e-9;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GridIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct EdgeKey
{
  int x{0};
  int y{0};
  int z{0};
  uint8_t axis{NetTrajectorySegment::AXIS_X};

  bool operator<(const EdgeKey & other) const
  {
    return std::tie(x, y, z, axis) < std::tie(other.x, other.y, other.z, other.axis);
  }
};

struct GeometrySegment
{
  Vec3 start;
  Vec3 end;
  uint8_t phase{SegmentCollision::MISSION};
  uint32_t phase_segment_index{0U};
};

struct EdgeRuntime
{
  uint64_t edge_id{0U};
  EdgeKey key;
  Vec3 start;
  Vec3 end;
  std::set<std::string> available_ids;
  std::set<std::string> collision_ids;
};

static Vec3 add(const Vec3 & first, const Vec3 & second)
{
  return Vec3{first.x + second.x, first.y + second.y, first.z + second.z};
}

static Vec3 subtract(const Vec3 & first, const Vec3 & second)
{
  return Vec3{first.x - second.x, first.y - second.y, first.z - second.z};
}

static Vec3 multiply(const Vec3 & value, double scalar)
{
  return Vec3{value.x * scalar, value.y * scalar, value.z * scalar};
}

static double dot(const Vec3 & first, const Vec3 & second)
{
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

static double norm(const Vec3 & value)
{
  return std::sqrt(dot(value, value));
}

static geometry_msgs::msg::Point to_point(const Vec3 & value)
{
  geometry_msgs::msg::Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

static uint64_t fnv1a64(const std::string & value)
{
  uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char character : value) {
    hash ^= static_cast<uint64_t>(character);
    hash *= 1099511628211ULL;
  }
  return hash;
}

static std_msgs::msg::ColorRGBA hsv_color(
  double hue,
  double saturation,
  double value,
  float alpha)
{
  hue -= std::floor(hue);
  const double h = hue * 6.0;
  const int sector = static_cast<int>(std::floor(h)) % 6;
  const double f = h - std::floor(h);
  const double p = value * (1.0 - saturation);
  const double q = value * (1.0 - saturation * f);
  const double t = value * (1.0 - saturation * (1.0 - f));

  double r = value;
  double g = t;
  double b = p;

  switch (sector) {
    case 0: r = value; g = t; b = p; break;
    case 1: r = q; g = value; b = p; break;
    case 2: r = p; g = value; b = t; break;
    case 3: r = p; g = q; b = value; break;
    case 4: r = t; g = p; b = value; break;
    default: r = value; g = p; b = q; break;
  }

  std_msgs::msg::ColorRGBA output;
  output.r = static_cast<float>(r);
  output.g = static_cast<float>(g);
  output.b = static_cast<float>(b);
  output.a = alpha;
  return output;
}

static std_msgs::msg::ColorRGBA trajectory_color(
  const std::string & trajectory_id,
  float alpha)
{
  const uint64_t hash = fnv1a64(trajectory_id);
  const double hue = static_cast<double>(hash % 1000000ULL) / 1000000.0;
  return hsv_color(hue, 0.78, 0.95, alpha);
}

static std::vector<Vec3> segment_points(
  const TrajectorySegment & segment,
  const std::string & trajectory_id,
  const std::string & segment_name)
{
  if (segment.x.size() != segment.y.size() || segment.x.size() != segment.z.size()) {
    throw std::runtime_error(
      "Trajectory '" + trajectory_id + "', segment '" + segment_name +
      "': x/y/z sizes differ");
  }

  std::vector<Vec3> output;
  output.reserve(segment.x.size());

  for (std::size_t index = 0U; index < segment.x.size(); ++index) {
    const Vec3 point{segment.x[index], segment.y[index], segment.z[index]};
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::runtime_error(
        "Trajectory '" + trajectory_id + "', segment '" + segment_name +
        "': non-finite coordinate");
    }
    output.push_back(point);
  }

  return output;
}

static void append_polyline_geometry(
  std::vector<GeometrySegment> & output,
  const std::vector<Vec3> & points,
  uint8_t phase,
  uint32_t & phase_segment_index)
{
  if (points.size() < 2U) {
    return;
  }

  output.reserve(
    output.size() +
    points.size() - 1U);

  for (
    std::size_t index = 1U;
    index < points.size();
    ++index)
  {
    output.push_back(
      GeometrySegment{
        points[index - 1U],
        points[index],
        phase,
        phase_segment_index++});
  }
}

static std::vector<GeometrySegment> build_geometry_segments(
  const StaticTrajectory & trajectory)
{
  if (trajectory.repetitions == 0U) {
    throw std::runtime_error(
            "Trajectory '" +
            trajectory.trajectory_id +
            "': repetitions must be >= 1");
  }

  const auto takeoff =
    segment_points(
      trajectory.takeoff,
      trajectory.trajectory_id,
      "takeoff");

  const auto landing =
    segment_points(
      trajectory.landing,
      trajectory.trajectory_id,
      "landing");

  std::vector<std::vector<Vec3>> missions;
  missions.reserve(
    trajectory.mission.size());

  for (
    std::size_t mission_index = 0U;
    mission_index < trajectory.mission.size();
    ++mission_index)
  {
    missions.push_back(
      segment_points(
        trajectory.mission[mission_index],
        trajectory.trajectory_id,
        "mission[" +
        std::to_string(mission_index) +
        "]"));
  }

  std::size_t estimated_segments = 0U;

  if (takeoff.size() >= 2U) {
    estimated_segments +=
      takeoff.size() - 1U;
  }

  for (const auto & mission : missions) {
    if (mission.size() >= 2U) {
      estimated_segments +=
        (mission.size() - 1U) *
        static_cast<std::size_t>(
        trajectory.repetitions);
    }
  }

  if (landing.size() >= 2U) {
    estimated_segments +=
      landing.size() - 1U;
  }

  std::vector<GeometrySegment> output;
  output.reserve(
    estimated_segments);

  uint32_t takeoff_segment_index = 0U;
  uint32_t mission_segment_index = 0U;
  uint32_t landing_segment_index = 0U;

  // TAKEOFF is one independent polyline.
  append_polyline_geometry(
    output,
    takeoff,
    SegmentCollision::TAKEOFF,
    takeoff_segment_index);

  // Partial repetitions repeat the complete ordered mission collection.
  //
  // CRITICAL MULTI-MISSION SEMANTICS:
  // every mission[i] is an independent continuous polyline.
  // No geometric segment is inserted between:
  //
  //   mission[i].back() -> mission[i + 1].front()
  //
  // and no connector is inserted between repetitions either.
  //
  // This allows a cropped supervised trajectory to contain multiple
  // collision-free mission pieces without the virtual net reintroducing the
  // removed collision section as an artificial lattice route.
  for (
    uint32_t repetition = 0U;
    repetition < trajectory.repetitions;
    ++repetition)
  {
    for (const auto & mission : missions) {
      append_polyline_geometry(
        output,
        mission,
        SegmentCollision::MISSION,
        mission_segment_index);
    }
  }

  // LANDING is one independent polyline.
  append_polyline_geometry(
    output,
    landing,
    SegmentCollision::LANDING,
    landing_segment_index);

  return output;
}

class ASpaceVirtualNetNode : public rclcpp::Node
{
public:
  ASpaceVirtualNetNode()
  : Node("a_space_virtual_net_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", 0.0);
    origin_z_ = declare_parameter<double>("origin_z", 0.0);
    size_x_ = declare_parameter<double>("size_x", 20.0);
    size_y_ = declare_parameter<double>("size_y", 20.0);
    size_z_ = declare_parameter<double>("size_z", 10.0);
    density_net_ = declare_parameter<double>("density_net", 1.0);
    edge_capacity_ = declare_parameter<int>("edge_capacity", 1);

    available_topic_ = declare_parameter<std::string>(
      "available_static_trajectories_topic", "/available_static_trajectories");
    collision_topic_ = declare_parameter<std::string>(
      "collision_static_trajectories_topic", "/collision_static_trajectories");
    net_loaded_topic_ = declare_parameter<std::string>(
      "net_loaded_trajectories_topic", "/net_loaded_trajectories");
    state_topic_ = declare_parameter<std::string>(
      "virtual_net_state_topic", "/a_space_virtual_net/state");
    markers_topic_ = declare_parameter<std::string>(
      "virtual_net_markers_topic", "/a_space_virtual_net/markers");

    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 1000);
    free_line_width_ = declare_parameter<double>("free_line_width", 0.025);
    trajectory_line_width_ = declare_parameter<double>("trajectory_line_width", 0.10);
    marker_alpha_ = declare_parameter<double>("marker_alpha", 0.95);

    validate_parameters();
    build_net();

    auto snapshot_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    snapshot_qos.reliable();
    snapshot_qos.transient_local();

    available_subscription_ = create_subscription<StaticTrajectoryArray>(
      available_topic_, snapshot_qos,
      std::bind(&ASpaceVirtualNetNode::available_callback, this, std::placeholders::_1));

    collision_subscription_ = create_subscription<CollisionStaticTrajectoryArray>(
      collision_topic_, snapshot_qos,
      std::bind(&ASpaceVirtualNetNode::collision_callback, this, std::placeholders::_1));

    net_loaded_publisher_ = create_publisher<NetLoadedTrajectoryArray>(
      net_loaded_topic_, snapshot_qos);
    state_publisher_ = create_publisher<VirtualNetState>(state_topic_, snapshot_qos);
    marker_publisher_ = create_publisher<MarkerArray>(markers_topic_, snapshot_qos);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(&ASpaceVirtualNetNode::publish_all, this));

    {
      std::lock_guard<std::mutex> lock(mutex_);
      rebuild_loaded_state_locked();
    }
    publish_all();

    RCLCPP_INFO(
      get_logger(),
      "A-space virtual net ready | frame='%s' | origin=(%.2f,%.2f,%.2f) | "
      "size=(%.2f,%.2f,%.2f)m | density=%.3fm | nodes=(%u,%u,%u) | edges=%zu | "
      "loaded='%s' | publish=%d ms",
      frame_id_.c_str(), origin_x_, origin_y_, origin_z_, size_x_, size_y_, size_z_,
      density_net_, nodes_x_, nodes_y_, nodes_z_, edges_.size(),
      net_loaded_topic_.c_str(), publish_period_ms_);
  }

private:
  void validate_parameters()
  {
    const auto positive = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
    const auto absolute_topic = [](const std::string & value) {
      return !value.empty() && value.front() == '/';
    };

    if (frame_id_.empty()) {
      throw std::runtime_error("frame_id cannot be empty");
    }
    if (!positive(size_x_) || !positive(size_y_) || !positive(size_z_) || !positive(density_net_)) {
      throw std::runtime_error("size_x/size_y/size_z/density_net must be finite and > 0");
    }
    if (!std::isfinite(origin_x_) || !std::isfinite(origin_y_) || !std::isfinite(origin_z_)) {
      throw std::runtime_error("origin_x/origin_y/origin_z must be finite");
    }

    const auto integral_multiple = [this](double size) {
      const double q = size / density_net_;
      return std::abs(q - std::round(q)) <= 1.0e-6;
    };
    if (!integral_multiple(size_x_) || !integral_multiple(size_y_) || !integral_multiple(size_z_)) {
      throw std::runtime_error("Each grid dimension must be an integer multiple of density_net");
    }
    if (edge_capacity_ <= 0 || publish_period_ms_ <= 0) {
      throw std::runtime_error("edge_capacity and publish_period_ms must be > 0");
    }
    if (!positive(free_line_width_) || !positive(trajectory_line_width_) ||
      !std::isfinite(marker_alpha_) || marker_alpha_ <= 0.0 || marker_alpha_ > 1.0)
    {
      throw std::runtime_error("Invalid marker configuration");
    }
    if (!absolute_topic(available_topic_) || !absolute_topic(collision_topic_) ||
      !absolute_topic(net_loaded_topic_) || !absolute_topic(state_topic_) ||
      !absolute_topic(markers_topic_))
    {
      throw std::runtime_error("Configured topic names must be absolute");
    }

    nodes_x_ = static_cast<uint32_t>(std::llround(size_x_ / density_net_)) + 1U;
    nodes_y_ = static_cast<uint32_t>(std::llround(size_y_ / density_net_)) + 1U;
    nodes_z_ = static_cast<uint32_t>(std::llround(size_z_ / density_net_)) + 1U;
  }

  Vec3 node_position(const GridIndex & index) const
  {
    return Vec3{
      origin_x_ + static_cast<double>(index.x) * density_net_,
      origin_y_ + static_cast<double>(index.y) * density_net_,
      origin_z_ + static_cast<double>(index.z) * density_net_};
  }

  uint64_t node_id(const GridIndex & index) const
  {
    return static_cast<uint64_t>(index.z) * static_cast<uint64_t>(nodes_y_) *
           static_cast<uint64_t>(nodes_x_) +
           static_cast<uint64_t>(index.y) * static_cast<uint64_t>(nodes_x_) +
           static_cast<uint64_t>(index.x);
  }

  bool valid_node(const GridIndex & index) const
  {
    return index.x >= 0 && index.y >= 0 && index.z >= 0 &&
           index.x < static_cast<int>(nodes_x_) &&
           index.y < static_cast<int>(nodes_y_) &&
           index.z < static_cast<int>(nodes_z_);
  }

  EdgeKey make_edge_key(const GridIndex & first, const GridIndex & second) const
  {
    const int dx = second.x - first.x;
    const int dy = second.y - first.y;
    const int dz = second.z - first.z;

    if (std::abs(dx) + std::abs(dy) + std::abs(dz) != 1) {
      throw std::runtime_error("Virtual-net edge must join adjacent nodes");
    }
    if (dx != 0) {
      return EdgeKey{std::min(first.x, second.x), first.y, first.z, NetTrajectorySegment::AXIS_X};
    }
    if (dy != 0) {
      return EdgeKey{first.x, std::min(first.y, second.y), first.z, NetTrajectorySegment::AXIS_Y};
    }
    return EdgeKey{first.x, first.y, std::min(first.z, second.z), NetTrajectorySegment::AXIS_Z};
  }

  std::pair<GridIndex, GridIndex> edge_nodes(const EdgeRuntime & edge) const
  {
    GridIndex first{edge.key.x, edge.key.y, edge.key.z};
    GridIndex second = first;
    if (edge.key.axis == NetTrajectorySegment::AXIS_X) {
      ++second.x;
    } else if (edge.key.axis == NetTrajectorySegment::AXIS_Y) {
      ++second.y;
    } else {
      ++second.z;
    }
    return {first, second};
  }

  void add_edge(const GridIndex & first, const GridIndex & second)
  {
    EdgeRuntime edge;
    edge.edge_id = static_cast<uint64_t>(edges_.size());
    edge.key = make_edge_key(first, second);
    edge.start = node_position(first);
    edge.end = node_position(second);
    edge_lookup_[edge.key] = edges_.size();
    edges_.push_back(std::move(edge));
  }

  void build_net()
  {
    edges_.clear();
    edge_lookup_.clear();

    for (uint32_t z = 0U; z < nodes_z_; ++z) {
      for (uint32_t y = 0U; y < nodes_y_; ++y) {
        for (uint32_t x = 0U; x < nodes_x_; ++x) {
          const GridIndex current{static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)};
          if (x + 1U < nodes_x_) {
            add_edge(current, GridIndex{static_cast<int>(x + 1U), static_cast<int>(y), static_cast<int>(z)});
          }
          if (y + 1U < nodes_y_) {
            add_edge(current, GridIndex{static_cast<int>(x), static_cast<int>(y + 1U), static_cast<int>(z)});
          }
          if (z + 1U < nodes_z_) {
            add_edge(current, GridIndex{static_cast<int>(x), static_cast<int>(y), static_cast<int>(z + 1U)});
          }
        }
      }
    }
  }

  bool clip_axis(
    double start,
    double delta,
    double minimum,
    double maximum,
    double & t_min,
    double & t_max) const
  {
    if (std::abs(delta) <= kEpsilon) {
      return start >= minimum - kEpsilon && start <= maximum + kEpsilon;
    }

    double first = (minimum - start) / delta;
    double second = (maximum - start) / delta;
    if (first > second) {
      std::swap(first, second);
    }
    t_min = std::max(t_min, first);
    t_max = std::min(t_max, second);
    return t_min <= t_max + kEpsilon;
  }

  bool clip_segment(
    const Vec3 & original_start,
    const Vec3 & original_end,
    Vec3 & clipped_start,
    Vec3 & clipped_end) const
  {
    const Vec3 delta = subtract(original_end, original_start);
    double t_min = 0.0;
    double t_max = 1.0;

    if (!clip_axis(original_start.x, delta.x, origin_x_, origin_x_ + size_x_, t_min, t_max) ||
      !clip_axis(original_start.y, delta.y, origin_y_, origin_y_ + size_y_, t_min, t_max) ||
      !clip_axis(original_start.z, delta.z, origin_z_, origin_z_ + size_z_, t_min, t_max))
    {
      return false;
    }

    clipped_start = add(original_start, multiply(delta, t_min));
    clipped_end = add(original_start, multiply(delta, t_max));
    return true;
  }

  GridIndex nearest_node(const Vec3 & point) const
  {
    const auto coordinate = [this](double value, double origin, uint32_t count) {
      const int raw = static_cast<int>(std::llround((value - origin) / density_net_));
      return std::max(0, std::min(raw, static_cast<int>(count) - 1));
    };

    return GridIndex{
      coordinate(point.x, origin_x_, nodes_x_),
      coordinate(point.y, origin_y_, nodes_y_),
      coordinate(point.z, origin_z_, nodes_z_)};
  }

  std::vector<EdgeKey> lattice_path(GridIndex current, const GridIndex & target) const
  {
    const int dx = std::abs(target.x - current.x);
    const int dy = std::abs(target.y - current.y);
    const int dz = std::abs(target.z - current.z);
    const int sx = target.x > current.x ? 1 : (target.x < current.x ? -1 : 0);
    const int sy = target.y > current.y ? 1 : (target.y < current.y ? -1 : 0);
    const int sz = target.z > current.z ? 1 : (target.z < current.z ? -1 : 0);

    int done_x = 0;
    int done_y = 0;
    int done_z = 0;
    std::vector<EdgeKey> output;
    output.reserve(static_cast<std::size_t>(dx + dy + dz));

    while (!(current == target)) {
      const double tx = done_x < dx ?
        (static_cast<double>(done_x) + 0.5) / static_cast<double>(dx) :
        std::numeric_limits<double>::infinity();
      const double ty = done_y < dy ?
        (static_cast<double>(done_y) + 0.5) / static_cast<double>(dy) :
        std::numeric_limits<double>::infinity();
      const double tz = done_z < dz ?
        (static_cast<double>(done_z) + 0.5) / static_cast<double>(dz) :
        std::numeric_limits<double>::infinity();

      GridIndex next = current;
      if (tx <= ty && tx <= tz) {
        next.x += sx;
        ++done_x;
      } else if (ty <= tz) {
        next.y += sy;
        ++done_y;
      } else {
        next.z += sz;
        ++done_z;
      }

      if (!valid_node(next)) {
        break;
      }
      output.push_back(make_edge_key(current, next));
      current = next;
    }

    return output;
  }

  std::vector<EdgeKey> project_segment(const GeometrySegment & segment) const
  {
    Vec3 clipped_start;
    Vec3 clipped_end;
    if (!clip_segment(segment.start, segment.end, clipped_start, clipped_end)) {
      return {};
    }
    return lattice_path(nearest_node(clipped_start), nearest_node(clipped_end));
  }

  void clear_edge_occupancy_locked()
  {
    for (auto & edge : edges_) {
      edge.available_ids.clear();
      edge.collision_ids.clear();
    }
  }

  void register_edge_use_locked(
    const EdgeKey & key,
    const std::string & trajectory_id,
    uint8_t source_state)
  {
    const auto iterator = edge_lookup_.find(key);
    if (iterator == edge_lookup_.end()) {
      return;
    }
    auto & edge = edges_[iterator->second];
    if (source_state == NetLoadedTrajectory::COLLISION) {
      edge.collision_ids.insert(trajectory_id);
    } else {
      edge.available_ids.insert(trajectory_id);
    }
  }

  NetLoadedTrajectory discretize_trajectory_locked(
    const StaticTrajectory & trajectory,
    uint8_t source_state)
  {
    NetLoadedTrajectory output;
    output.source_state = source_state;
    output.trajectory = trajectory;

    if (!trajectory.frame_id.empty() && trajectory.frame_id != frame_id_) {
      RCLCPP_WARN(
        get_logger(),
        "Trajectory '%s' ignored: frame='%s' differs from net frame='%s'",
        trajectory.trajectory_id.c_str(), trajectory.frame_id.c_str(), frame_id_.c_str());
      return output;
    }

    const auto geometry = build_geometry_segments(trajectory);
    uint32_t net_segment_index = 0U;

    for (const auto & source_segment : geometry) {
      const auto projected_edges = project_segment(source_segment);
      for (const auto & key : projected_edges) {
        const auto edge_iterator = edge_lookup_.find(key);
        if (edge_iterator == edge_lookup_.end()) {
          continue;
        }

        const auto & edge = edges_[edge_iterator->second];
        const auto nodes = edge_nodes(edge);

        NetTrajectorySegment segment;
        segment.edge_id = edge.edge_id;
        segment.axis = edge.key.axis;
        segment.start_node_id = node_id(nodes.first);
        segment.end_node_id = node_id(nodes.second);
        segment.start = to_point(edge.start);
        segment.end = to_point(edge.end);
        segment.length_m = norm(subtract(edge.end, edge.start));
        segment.phase = source_segment.phase;
        segment.original_phase_segment_index = source_segment.phase_segment_index;
        segment.net_segment_index = net_segment_index++;
        output.segments.push_back(segment);

        register_edge_use_locked(key, trajectory.trajectory_id, source_state);
      }
    }

    return output;
  }

  void rebuild_loaded_state_locked()
  {
    clear_edge_occupancy_locked();
    loaded_trajectories_.clear();

    std::map<std::string, std::pair<StaticTrajectory, uint8_t>> merged;

    for (const auto & trajectory : available_snapshot_.trajectories) {
      if (!trajectory.trajectory_id.empty()) {
        merged[trajectory.trajectory_id] = {trajectory, NetLoadedTrajectory::AVAILABLE};
      }
    }

    for (const auto & collision : collision_snapshot_.trajectories) {
      if (!collision.trajectory.trajectory_id.empty()) {
        // Collision wins if an inconsistent upstream snapshot contains the ID in both states.
        merged[collision.trajectory.trajectory_id] =
          {collision.trajectory, NetLoadedTrajectory::COLLISION};
      }
    }

    std::vector<std::pair<StaticTrajectory, uint8_t>> ordered;
    ordered.reserve(merged.size());
    for (const auto & [_, item] : merged) {
      ordered.push_back(item);
    }

    std::stable_sort(
      ordered.begin(), ordered.end(),
      [](const auto & first, const auto & second) {
        if (first.first.priority != second.first.priority) {
          return first.first.priority < second.first.priority;
        }
        return first.first.trajectory_id < second.first.trajectory_id;
      });

    loaded_trajectories_.reserve(ordered.size());
    for (const auto & item : ordered) {
      try {
        loaded_trajectories_.push_back(discretize_trajectory_locked(item.first, item.second));
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Cannot load trajectory '%s' onto virtual net: %s",
          item.first.trajectory_id.c_str(), error.what());
      }
    }
  }

  void available_callback(const StaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    available_snapshot_ = *message;
    rebuild_loaded_state_locked();
  }

  void collision_callback(const CollisionStaticTrajectoryArray::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    collision_snapshot_ = *message;
    rebuild_loaded_state_locked();
  }

  uint32_t edge_occupancy(const EdgeRuntime & edge) const
  {
    std::set<std::string> ids = edge.available_ids;
    ids.insert(edge.collision_ids.begin(), edge.collision_ids.end());
    return static_cast<uint32_t>(ids.size());
  }

  uint8_t edge_state(const EdgeRuntime & edge) const
  {
    if (!edge.collision_ids.empty()) {
      return VirtualNetEdge::COLLISION;
    }
    if (!edge.available_ids.empty()) {
      return VirtualNetEdge::AVAILABLE;
    }
    return VirtualNetEdge::FREE;
  }

  VirtualNetState make_state_message_locked() const
  {
    VirtualNetState output;
    output.header.stamp = now();
    output.header.frame_id = frame_id_;
    output.origin.x = origin_x_;
    output.origin.y = origin_y_;
    output.origin.z = origin_z_;
    output.size_x = size_x_;
    output.size_y = size_y_;
    output.size_z = size_z_;
    output.density_net = density_net_;
    output.nodes_x = nodes_x_;
    output.nodes_y = nodes_y_;
    output.nodes_z = nodes_z_;
    output.total_edges = static_cast<uint64_t>(edges_.size());
    output.total_capacity_units = output.total_edges * static_cast<uint64_t>(edge_capacity_);
    output.edges.reserve(edges_.size());

    uint64_t occupied_capacity_units = 0U;

    for (const auto & edge : edges_) {
      VirtualNetEdge message;
      message.edge_id = edge.edge_id;
      message.axis = edge.key.axis;
      message.start = to_point(edge.start);
      message.end = to_point(edge.end);
      message.state = edge_state(edge);
      message.capacity = static_cast<uint32_t>(edge_capacity_);
      message.occupancy = edge_occupancy(edge);
      message.utilization =
        static_cast<double>(message.occupancy) / static_cast<double>(message.capacity);
      message.available_trajectory_ids.assign(edge.available_ids.begin(), edge.available_ids.end());
      message.collision_trajectory_ids.assign(edge.collision_ids.begin(), edge.collision_ids.end());

      if (message.state == VirtualNetEdge::FREE) {
        ++output.free_edges;
      } else if (message.state == VirtualNetEdge::AVAILABLE) {
        ++output.available_edges;
      } else if (message.state == VirtualNetEdge::COLLISION) {
        ++output.collision_edges;
      }

      occupied_capacity_units += std::min<uint64_t>(
        static_cast<uint64_t>(message.occupancy), static_cast<uint64_t>(message.capacity));
      output.edges.push_back(std::move(message));
    }

    output.occupied_capacity_units = occupied_capacity_units;
    if (output.total_capacity_units > 0U) {
      output.global_utilization =
        static_cast<double>(occupied_capacity_units) /
        static_cast<double>(output.total_capacity_units);
    }
    return output;
  }

  NetLoadedTrajectoryArray make_loaded_message_locked() const
  {
    NetLoadedTrajectoryArray output;
    output.header.stamp = now();
    output.header.frame_id = frame_id_;
    output.origin.x = origin_x_;
    output.origin.y = origin_y_;
    output.origin.z = origin_z_;
    output.size_x = size_x_;
    output.size_y = size_y_;
    output.size_z = size_z_;
    output.density_net = density_net_;
    output.nodes_x = nodes_x_;
    output.nodes_y = nodes_y_;
    output.nodes_z = nodes_z_;
    output.trajectories = loaded_trajectories_;
    return output;
  }

  MarkerArray make_markers_locked() const
  {
    MarkerArray output;

    Marker delete_all;
    delete_all.header.stamp = now();
    delete_all.header.frame_id = frame_id_;
    delete_all.action = Marker::DELETEALL;
    output.markers.push_back(delete_all);

    // Marker type 1: every unused edge in gray.
    Marker free;
    free.header = delete_all.header;
    free.ns = "a_space_virtual_net/free";
    free.id = 0;
    free.type = Marker::LINE_LIST;
    free.action = Marker::ADD;
    free.pose.orientation.w = 1.0;
    free.scale.x = free_line_width_;
    free.color.r = 0.55F;
    free.color.g = 0.55F;
    free.color.b = 0.55F;
    free.color.a = static_cast<float>(marker_alpha_);

    for (const auto & edge : edges_) {
      if (edge_occupancy(edge) == 0U) {
        free.points.push_back(to_point(edge.start));
        free.points.push_back(to_point(edge.end));
      }
    }
    output.markers.push_back(std::move(free));

    // Marker type 2: one colored LINE_LIST per loaded trajectory.
    int marker_id = 1;
    for (const auto & loaded : loaded_trajectories_) {
      Marker marker;
      marker.header = delete_all.header;
      marker.ns = "a_space_virtual_net/trajectory/" + loaded.trajectory.trajectory_id;
      marker.id = marker_id++;
      marker.type = Marker::LINE_LIST;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = trajectory_line_width_;
      marker.color = trajectory_color(
        loaded.trajectory.trajectory_id, static_cast<float>(marker_alpha_));
      marker.points.reserve(loaded.segments.size() * 2U);

      for (const auto & segment : loaded.segments) {
        marker.points.push_back(segment.start);
        marker.points.push_back(segment.end);
      }
      output.markers.push_back(std::move(marker));
    }

    return output;
  }

  void publish_all()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    net_loaded_publisher_->publish(make_loaded_message_locked());
    state_publisher_->publish(make_state_message_locked());
    marker_publisher_->publish(make_markers_locked());
  }

  std::string frame_id_;
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.0};
  double size_x_{20.0};
  double size_y_{20.0};
  double size_z_{10.0};
  double density_net_{1.0};
  int edge_capacity_{1};
  uint32_t nodes_x_{0U};
  uint32_t nodes_y_{0U};
  uint32_t nodes_z_{0U};

  std::string available_topic_;
  std::string collision_topic_;
  std::string net_loaded_topic_;
  std::string state_topic_;
  std::string markers_topic_;

  int publish_period_ms_{1000};
  double free_line_width_{0.025};
  double trajectory_line_width_{0.10};
  double marker_alpha_{0.95};

  mutable std::mutex mutex_;
  std::vector<EdgeRuntime> edges_;
  std::map<EdgeKey, std::size_t> edge_lookup_;
  StaticTrajectoryArray available_snapshot_;
  CollisionStaticTrajectoryArray collision_snapshot_;
  std::vector<NetLoadedTrajectory> loaded_trajectories_;

  rclcpp::Subscription<StaticTrajectoryArray>::SharedPtr available_subscription_;
  rclcpp::Subscription<CollisionStaticTrajectoryArray>::SharedPtr collision_subscription_;
  rclcpp::Publisher<NetLoadedTrajectoryArray>::SharedPtr net_loaded_publisher_;
  rclcpp::Publisher<VirtualNetState>::SharedPtr state_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace a_space_virtual_net

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<a_space_virtual_net::ASpaceVirtualNetNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("a_space_virtual_net_node"),
      "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
