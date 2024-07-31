// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "yabloc_common/ll2_decomposer/ll2_decomposer.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <yabloc_common/pub_sub.hpp>

#include <geometry_msgs/msg/polygon.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace yabloc::ll2_decomposer
{
Ll2Decomposer::Ll2Decomposer(const rclcpp::NodeOptions & options) : Node("ll2_to_image", options)
{
  using std::placeholders::_1;
  const rclcpp::QoS latch_qos = rclcpp::QoS(1).transient_local();
  const rclcpp::QoS map_qos = rclcpp::QoS(1).transient_local().reliable();

  // Publisher
  pub_road_marking_ = create_publisher<Cloud2>("~/output/ll2_road_marking", latch_qos);
  pub_sign_board_ = create_publisher<Cloud2>("~/output/ll2_sign_board", latch_qos);
  pub_bounding_box_ = create_publisher<Cloud2>("~/output/ll2_bounding_box", latch_qos);
  pub_marker_ = create_publisher<MarkerArray>("~/debug/sign_board_marker", latch_qos);

  // Subscriber
  auto cb_map = std::bind(&Ll2Decomposer::on_map, this, _1);
  sub_map_ = create_subscription<LaneletMapBin>("~/input/vector_map", map_qos, cb_map);

  auto load_lanelet2_labels =
    [this](const std::string & param_name, std::set<std::string> & labels) -> void {
    this->template declare_parameter<std::vector<std::string>>(param_name);
    auto label_array = get_parameter(param_name).as_string_array();
    for (const auto & l : label_array) labels.insert(l);
  };

  load_lanelet2_labels("road_marking_labels", road_marking_labels_);
  load_lanelet2_labels("sign_board_labels", sign_board_labels_);
  load_lanelet2_labels("bounding_box_labels", bounding_box_labels_);

  if (road_marking_labels_.empty()) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "There are no road marking labels. No LL2 elements will publish");
  }
}

void print_attr(const lanelet::LaneletMapPtr & lanelet_map, const rclcpp::Logger & logger)
{
  std::set<std::string> types;
  for (const lanelet::ConstLineString3d & line : lanelet_map->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    types.insert(attr.value());
  }
  for (const lanelet::ConstPolygon3d & polygon : lanelet_map->polygonLayer) {
    if (!polygon.hasAttribute(lanelet::AttributeName::Type)) {
      continue;
    }
    lanelet::Attribute attr = polygon.attribute(lanelet::AttributeName::Type);
    types.insert(attr.value());
  }

  for (const auto & type : types) {
    RCLCPP_INFO_STREAM(logger, "lanelet type: " << type);
  }
}

pcl::PointCloud<pcl::PointXYZL> Ll2Decomposer::load_bounding_boxes(
  const lanelet::PolygonLayer & polygons) const
{
  pcl::PointCloud<pcl::PointXYZL> cloud;
  int index = 0;

  for (const lanelet::ConstPolygon3d & polygon : polygons) {
    if (!polygon.hasAttribute(lanelet::AttributeName::Type)) continue;
    const lanelet::Attribute & attr = polygon.attribute(lanelet::AttributeName::Type);
    if (bounding_box_labels_.count(attr.value()) == 0) continue;

    for (const lanelet::ConstPoint3d & p : polygon) {
      pcl::PointXYZL xyzl;
      xyzl.x = static_cast<float>(p.x());
      xyzl.y = static_cast<float>(p.y());
      xyzl.z = static_cast<float>(p.z());
      xyzl.label = index;
      cloud.push_back(xyzl);
    }
    index++;
  }
  return cloud;
}

void Ll2Decomposer::on_map(const LaneletMapBin & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "subscribed binary vector map");
  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map);
  print_attr(lanelet_map, get_logger());

  const rclcpp::Time stamp = msg.header.stamp;

  const auto & ls_layer = lanelet_map->lineStringLayer;
  const auto & po_layer = lanelet_map->polygonLayer;
  auto tmp1 = extract_specified_line_string(ls_layer, sign_board_labels_);
  auto tmp2 = extract_specified_line_string(ls_layer, road_marking_labels_);
  pcl::PointCloud<pcl::PointNormal> ll2_sign_board = split_line_strings(tmp1);
  pcl::PointCloud<pcl::PointNormal> ll2_road_marking = split_line_strings(tmp2);
  pcl::PointCloud<pcl::PointXYZL> ll2_bounding_box = load_bounding_boxes(po_layer);

  publish_additional_marker(lanelet_map);

  common::publish_cloud(*pub_road_marking_, ll2_road_marking, stamp);
  common::publish_cloud(*pub_sign_board_, ll2_sign_board, stamp);
  common::publish_cloud(*pub_bounding_box_, ll2_bounding_box, stamp);

  RCLCPP_INFO_STREAM(get_logger(), "succeeded map decomposing");
}

pcl::PointCloud<pcl::PointNormal> Ll2Decomposer::split_line_strings(
  const lanelet::ConstLineStrings3d & line_strings)
{
  pcl::PointCloud<pcl::PointNormal> extracted;
  for (const lanelet::ConstLineString3d & line : line_strings) {
    lanelet::ConstPoint3d const * from = nullptr;
    for (const lanelet::ConstPoint3d & to : line) {
      if (from != nullptr) {
        pcl::PointNormal pn = to_point_normal(*from, to);
        extracted.push_back(pn);
      }
      from = &to;
    }
  }
  return extracted;
}

lanelet::ConstLineStrings3d Ll2Decomposer::extract_specified_line_string(
  const lanelet::LineStringLayer & line_string_layer, const std::set<std::string> & visible_labels)
{
  lanelet::ConstLineStrings3d line_strings;
  for (const lanelet::ConstLineString3d & line : line_string_layer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    const lanelet::Attribute & attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;
    line_strings.push_back(line);
  }
  return line_strings;
}

lanelet::ConstPolygons3d Ll2Decomposer::extract_specified_polygon(
  const lanelet::PolygonLayer & polygon_layer, const std::set<std::string> & visible_labels)
{
  lanelet::ConstPolygons3d polygons;
  for (const lanelet::ConstPolygon3d & polygon : polygon_layer) {
    if (!polygon.hasAttribute(lanelet::AttributeName::Type)) continue;
    const lanelet::Attribute & attr = polygon.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;
    polygons.push_back(polygon);
  }
  return polygons;
}

pcl::PointNormal Ll2Decomposer::to_point_normal(
  const lanelet::ConstPoint3d & from, const lanelet::ConstPoint3d & to)
{
  pcl::PointNormal pn;
  pn.x = static_cast<float>(from.x());
  pn.y = static_cast<float>(from.y());
  pn.z = static_cast<float>(from.z());
  pn.normal_x = static_cast<float>(to.x());
  pn.normal_y = static_cast<float>(to.y());
  pn.normal_z = static_cast<float>(to.z());
  return pn;
}

Ll2Decomposer::MarkerArray Ll2Decomposer::make_sign_marker_msg(
  const lanelet::LineStringLayer & line_string_layer, const std::set<std::string> & labels,
  const std::string & ns)
{
  lanelet::ConstLineStrings3d line_strings =
    extract_specified_line_string(line_string_layer, labels);

  MarkerArray marker_array;
  int id = 0;
  for (const lanelet::ConstLineString3d & line_string : line_strings) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    marker.type = Marker::LINE_STRIP;
    marker.color = autoware::universe_utils::createMarkerColor(0.6f, 0.6f, 0.6f, 0.999f);
    marker.scale.x = 0.1;
    marker.ns = ns;
    marker.id = id++;

    for (const lanelet::ConstPoint3d & p : line_string) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = p.z();
      marker.points.push_back(gp);
    }
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

Ll2Decomposer::MarkerArray Ll2Decomposer::make_polygon_marker_msg(
  const lanelet::PolygonLayer & polygon_layer, const std::set<std::string> & labels,
  const std::string & ns)
{
  lanelet::ConstPolygons3d polygons = extract_specified_polygon(polygon_layer, labels);

  MarkerArray marker_array;
  int id = 0;
  for (const lanelet::ConstPolygon3d & polygon : polygons) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    marker.type = Marker::LINE_STRIP;
    marker.color = autoware::universe_utils::createMarkerColor(0.4f, 0.4f, 0.8f, 0.999f);
    marker.scale.x = 0.2;
    marker.ns = ns;
    marker.id = id++;

    auto gen_point = [](const lanelet::ConstPoint3d & p) -> geometry_msgs::msg::Point {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = p.z();
      return gp;
    };

    for (const lanelet::ConstPoint3d & p : polygon) marker.points.push_back(gen_point(p));
    marker.points.push_back(gen_point(polygon.front()));

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

void Ll2Decomposer::publish_additional_marker(const lanelet::LaneletMapPtr & lanelet_map)
{
  auto marker1 =
    make_sign_marker_msg(lanelet_map->lineStringLayer, sign_board_labels_, "sign_board");
  auto marker2 = make_sign_marker_msg(lanelet_map->lineStringLayer, {"virtual"}, "virtual");
  auto marker3 =
    make_polygon_marker_msg(lanelet_map->polygonLayer, {"bounding_box"}, "bounding_box");

  std::copy(marker2.markers.begin(), marker2.markers.end(), std::back_inserter(marker1.markers));
  std::copy(marker3.markers.begin(), marker3.markers.end(), std::back_inserter(marker1.markers));
  pub_marker_->publish(marker1);
}

}  // namespace yabloc::ll2_decomposer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yabloc::ll2_decomposer::Ll2Decomposer)
