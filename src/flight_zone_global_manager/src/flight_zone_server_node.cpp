#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <flight_zone_msgs/msg/flight_zone.hpp>
#include <flight_zone_msgs/msg/flight_zone_array.hpp>
#include <flight_zone_msgs/msg/polyhedron_face.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace flight_zone_global_manager
{

using FlightZone = flight_zone_msgs::msg::FlightZone;
using FlightZoneArray = flight_zone_msgs::msg::FlightZoneArray;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct ZoneVisualConfig
{
  std_msgs::msg::ColorRGBA surface_color;
  std_msgs::msg::ColorRGBA edge_color;
  std_msgs::msg::ColorRGBA text_color;
  double line_width{0.08};
  double text_height{0.65};
  bool show_surface{true};
};

struct LoadedZone
{
  FlightZone zone;
  ZoneVisualConfig visual;
};

class FlightZoneServerNode : public rclcpp::Node
{
public:
  FlightZoneServerNode()
  : Node("flight_zone_server_node")
  {
    const auto default_config =
      ament_index_cpp::get_package_share_directory("flight_zone_global_manager") +
      "/config/flight_zone_server.yaml";

    config_file_ = declare_parameter<std::string>("config_file", default_config);
    zone_topic_ = declare_parameter<std::string>("zone_topic", "/flight_zones");
    marker_topic_ =
      declare_parameter<std::string>("marker_topic", "/flight_zones/markers");
    publish_period_s_ = declare_parameter<double>("publish_period_s", 1.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    zone_publisher_ = create_publisher<FlightZoneArray>(zone_topic_, qos);
    marker_publisher_ = create_publisher<MarkerArray>(marker_topic_, qos);

    load_configuration(config_file_);
    publish_messages();

    if (publish_period_s_ > 0.0) {
      const auto period = std::chrono::duration<double>(publish_period_s_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FlightZoneServerNode::publish_messages, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "Flight-zone server ready: zones=%zu, frame='map'",
      loaded_zones_.size());
    RCLCPP_INFO(get_logger(), "Configuration: %s", config_file_.c_str());
    RCLCPP_INFO(get_logger(), "Zone topic: %s", zone_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Marker topic: %s", marker_topic_.c_str());
  }

private:
  static std_msgs::msg::ColorRGBA make_color(
    const float r, const float g, const float b, const float a)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  static std_msgs::msg::ColorRGBA parse_color(
    const YAML::Node & node,
    const std_msgs::msg::ColorRGBA & fallback,
    const std::string & field_name)
  {
    if (!node) {
      return fallback;
    }

    if (!node.IsSequence() || node.size() != 4U) {
      throw std::runtime_error(field_name + " must be [r, g, b, a]");
    }

    auto color = make_color(
      node[0].as<float>(),
      node[1].as<float>(),
      node[2].as<float>(),
      node[3].as<float>());

    color.r = std::clamp(color.r, 0.0F, 1.0F);
    color.g = std::clamp(color.g, 0.0F, 1.0F);
    color.b = std::clamp(color.b, 0.0F, 1.0F);
    color.a = std::clamp(color.a, 0.0F, 1.0F);
    return color;
  }

  static uint8_t parse_zone_type(const std::string & type)
  {
    if (type == "inclusion") {
      return FlightZone::INCLUSION;
    }

    if (type == "exclusion") {
      return FlightZone::EXCLUSION;
    }

    if (type == "warning") {
      return FlightZone::WARNING;
    }

    throw std::runtime_error(
            "Unknown zone_type '" + type +
            "'. Expected inclusion, exclusion or warning");
  }

  static geometry_msgs::msg::Point parse_vertex(
    const YAML::Node & node,
    const std::string & context)
  {
    if (!node.IsSequence() || node.size() != 3U) {
      throw std::runtime_error(context + " must be [x, y, z]");
    }

    geometry_msgs::msg::Point point;
    point.x = node[0].as<double>();
    point.y = node[1].as<double>();
    point.z = node[2].as<double>();
    return point;
  }

  void load_configuration(const std::string & file_path)
  {
    YAML::Node root;

    try {
      root = YAML::LoadFile(file_path);
    } catch (const YAML::Exception & exception) {
      throw std::runtime_error(
              "Unable to load YAML file '" + file_path + "': " +
              exception.what());
    }

    const std::string configured_frame =
      root["frame_id"] ? root["frame_id"].as<std::string>() : "map";

    if (configured_frame != "map") {
      throw std::runtime_error(
              "frame_id must be exactly 'map'; received '" +
              configured_frame + "'");
    }

    const auto zones = root["zones"];
    if (!zones || !zones.IsSequence() || zones.size() == 0U) {
      throw std::runtime_error(
              "YAML field zones must contain at least one zone");
    }

    loaded_zones_.clear();
    std::set<std::string> used_zone_ids;

    for (std::size_t zone_index = 0; zone_index < zones.size(); ++zone_index) {
      const auto zone_node = zones[zone_index];
      LoadedZone loaded;

      if (!zone_node["zone_id"]) {
        throw std::runtime_error(
                "zones[" + std::to_string(zone_index) +
                "].zone_id is required");
      }

      loaded.zone.header.frame_id = "map";
      loaded.zone.zone_id = zone_node["zone_id"].as<std::string>();

      // La geometría se publica sin asignación a vehículo. Estos campos se
      // conservan vacíos por compatibilidad con el mensaje FlightZone actual.
      // La relación UAS <-> flight zone se publicará en una capa independiente.
      loaded.zone.vehicle_id.clear();
      loaded.zone.display_name =
        zone_node["display_name"] ?
        zone_node["display_name"].as<std::string>() :
        loaded.zone.zone_id;

      loaded.zone.zone_type = parse_zone_type(
        zone_node["zone_type"] ?
        zone_node["zone_type"].as<std::string>() :
        "inclusion");

      loaded.zone.safety_margin =
        zone_node["safety_margin"] ?
        zone_node["safety_margin"].as<double>() :
        0.0;

      loaded.zone.enabled =
        zone_node["enabled"] ?
        zone_node["enabled"].as<bool>() :
        true;

      if (loaded.zone.zone_id.empty()) {
        throw std::runtime_error(
                "zones[" + std::to_string(zone_index) +
                "].zone_id must not be empty");
      }

      if (!used_zone_ids.insert(loaded.zone.zone_id).second) {
        throw std::runtime_error(
                "Duplicated zone_id '" + loaded.zone.zone_id + "'");
      }

      if (loaded.zone.safety_margin < 0.0) {
        throw std::runtime_error(
                "Zone '" + loaded.zone.zone_id +
                "' has a negative safety_margin");
      }

      const auto vertices = zone_node["vertices"];
      if (!vertices || !vertices.IsSequence() || vertices.size() < 4U) {
        throw std::runtime_error(
                "Zone '" + loaded.zone.zone_id +
                "' must contain at least four vertices");
      }

      for (
        std::size_t vertex_index = 0;
        vertex_index < vertices.size();
        ++vertex_index)
      {
        loaded.zone.vertices.push_back(
          parse_vertex(
            vertices[vertex_index],
            "zones[" + std::to_string(zone_index) +
            "].vertices[" + std::to_string(vertex_index) + "]"));
      }

      const auto faces = zone_node["faces"];
      if (!faces || !faces.IsSequence() || faces.size() < 4U) {
        throw std::runtime_error(
                "Zone '" + loaded.zone.zone_id +
                "' must contain at least four faces");
      }

      for (
        std::size_t face_index = 0;
        face_index < faces.size();
        ++face_index)
      {
        const auto face_node = faces[face_index];

        if (!face_node.IsSequence() || face_node.size() < 3U) {
          throw std::runtime_error(
                  "Each face of zone '" + loaded.zone.zone_id +
                  "' must contain at least three vertex indices");
        }

        flight_zone_msgs::msg::PolyhedronFace face;
        std::set<uint32_t> face_indices;

        for (std::size_t i = 0; i < face_node.size(); ++i) {
          const auto index = face_node[i].as<uint32_t>();

          if (index >= loaded.zone.vertices.size()) {
            throw std::runtime_error(
                    "Face index out of range in zone '" +
                    loaded.zone.zone_id + "'");
          }

          if (!face_indices.insert(index).second) {
            throw std::runtime_error(
                    "A face contains a repeated vertex index in zone '" +
                    loaded.zone.zone_id + "'");
          }

          face.vertex_indices.push_back(index);
        }

        loaded.zone.faces.push_back(face);
      }

      const auto visualization = zone_node["visualization"];

      loaded.visual.surface_color = parse_color(
        visualization ? visualization["surface_rgba"] : YAML::Node(),
        make_color(0.10F, 0.60F, 1.00F, 0.25F),
        "surface_rgba");

      loaded.visual.edge_color = parse_color(
        visualization ? visualization["edge_rgba"] : YAML::Node(),
        make_color(0.00F, 0.20F, 1.00F, 1.00F),
        "edge_rgba");

      loaded.visual.text_color = parse_color(
        visualization ? visualization["text_rgba"] : YAML::Node(),
        make_color(1.00F, 1.00F, 1.00F, 1.00F),
        "text_rgba");

      loaded.visual.line_width =
        visualization && visualization["line_width"] ?
        visualization["line_width"].as<double>() :
        0.08;

      loaded.visual.text_height =
        visualization && visualization["text_height"] ?
        visualization["text_height"].as<double>() :
        0.65;

      loaded.visual.show_surface =
        visualization && visualization["show_surface"] ?
        visualization["show_surface"].as<bool>() :
        true;

      if (
        loaded.visual.line_width <= 0.0 ||
        loaded.visual.text_height <= 0.0)
      {
        throw std::runtime_error(
                "line_width and text_height must be positive in zone '" +
                loaded.zone.zone_id + "'");
      }

      loaded_zones_.push_back(std::move(loaded));
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu unassigned flight zone(s) from: %s",
      loaded_zones_.size(),
      file_path.c_str());
  }

  static void initialise_marker(
    Marker & marker,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & marker_namespace,
    const int32_t id,
    const int32_t type)
  {
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
  }

  static Marker create_surface_marker(
    const LoadedZone & loaded,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & marker_namespace)
  {
    Marker marker;
    initialise_marker(
      marker,
      stamp,
      marker_namespace,
      0,
      Marker::TRIANGLE_LIST);

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color = loaded.visual.surface_color;

    // Triangulación mediante abanico. Las caras con más de tres vértices
    // deben ser convexas y estar ordenadas alrededor de su perímetro.
    for (const auto & face : loaded.zone.faces) {
      const auto anchor = face.vertex_indices.front();

      for (
        std::size_t i = 1;
        i + 1 < face.vertex_indices.size();
        ++i)
      {
        marker.points.push_back(
          loaded.zone.vertices[anchor]);
        marker.points.push_back(
          loaded.zone.vertices[face.vertex_indices[i]]);
        marker.points.push_back(
          loaded.zone.vertices[face.vertex_indices[i + 1]]);
      }
    }

    return marker;
  }

  static Marker create_edge_marker(
    const LoadedZone & loaded,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & marker_namespace)
  {
    Marker marker;
    initialise_marker(
      marker,
      stamp,
      marker_namespace,
      1,
      Marker::LINE_LIST);

    marker.scale.x = loaded.visual.line_width;
    marker.color = loaded.visual.edge_color;

    std::set<std::pair<uint32_t, uint32_t>> unique_edges;

    for (const auto & face : loaded.zone.faces) {
      for (
        std::size_t i = 0;
        i < face.vertex_indices.size();
        ++i)
      {
        const auto first = face.vertex_indices[i];
        const auto second =
          face.vertex_indices[
          (i + 1U) % face.vertex_indices.size()];

        unique_edges.emplace(
          std::min(first, second),
          std::max(first, second));
      }
    }

    for (const auto & edge : unique_edges) {
      marker.points.push_back(
        loaded.zone.vertices[edge.first]);
      marker.points.push_back(
        loaded.zone.vertices[edge.second]);
    }

    return marker;
  }

  static Marker create_text_marker(
    const LoadedZone & loaded,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & marker_namespace)
  {
    Marker marker;
    initialise_marker(
      marker,
      stamp,
      marker_namespace,
      2,
      Marker::TEXT_VIEW_FACING);

    marker.scale.z = loaded.visual.text_height;
    marker.color = loaded.visual.text_color;

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    double max_z = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();

    for (const auto & vertex : loaded.zone.vertices) {
      centroid_x += vertex.x;
      centroid_y += vertex.y;
      min_z = std::min(min_z, vertex.z);
      max_z = std::max(max_z, vertex.z);
    }

    centroid_x /= static_cast<double>(
      loaded.zone.vertices.size());
    centroid_y /= static_cast<double>(
      loaded.zone.vertices.size());

    marker.pose.position.x = centroid_x;
    marker.pose.position.y = centroid_y;
    marker.pose.position.z =
      max_z + loaded.visual.text_height;

    std::ostringstream label;
    label
      << loaded.zone.display_name << "\n"
      << "ID: " << loaded.zone.zone_id << "\n"
      << std::fixed << std::setprecision(1)
      << "Z: " << min_z << " - " << max_z << " m";

    marker.text = label.str();
    return marker;
  }

  void publish_messages()
  {
    const builtin_interfaces::msg::Time stamp = this->now();

    FlightZoneArray zone_array;
    zone_array.header.frame_id = "map";
    zone_array.header.stamp = stamp;

    MarkerArray marker_array;

    for (auto & loaded : loaded_zones_) {
      loaded.zone.header.frame_id = "map";
      loaded.zone.header.stamp = stamp;
      zone_array.zones.push_back(loaded.zone);

      if (!loaded.zone.enabled) {
        continue;
      }

      // zone_id debe ser único dentro de la infraestructura A-space.
      // Ya no se utiliza vehicle_id para construir el namespace del marcador.
      const std::string marker_namespace =
        "flight_zone/" + loaded.zone.zone_id;

      if (loaded.visual.show_surface) {
        marker_array.markers.push_back(
          create_surface_marker(
            loaded,
            stamp,
            marker_namespace));
      }

      marker_array.markers.push_back(
        create_edge_marker(
          loaded,
          stamp,
          marker_namespace));

      marker_array.markers.push_back(
        create_text_marker(
          loaded,
          stamp,
          marker_namespace));
    }

    zone_publisher_->publish(zone_array);
    marker_publisher_->publish(marker_array);
  }

  std::string config_file_;
  std::string zone_topic_;
  std::string marker_topic_;
  double publish_period_s_{1.0};

  std::vector<LoadedZone> loaded_zones_;
  rclcpp::Publisher<FlightZoneArray>::SharedPtr zone_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace flight_zone_global_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        flight_zone_global_manager::FlightZoneServerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("flight_zone_server_node"),
      "%s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
