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

#ifndef YABLOC_COMMON__LL2_DECOMPOSER__LL2_DECOMPOSER_HPP_
#define YABLOC_COMMON__LL2_DECOMPOSER__LL2_DECOMPOSER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <set>
#include <string>

namespace yabloc::ll2_decomposer
{
class Ll2Decomposer : public rclcpp::Node
{
public:
  using Cloud2 = sensor_msgs::msg::PointCloud2;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  Ll2Decomposer();

private:
  rclcpp::Publisher<Cloud2>::SharedPtr pub_road_marking_;
  rclcpp::Publisher<Cloud2>::SharedPtr pub_sign_board_;
  rclcpp::Publisher<Cloud2>::SharedPtr pub_bounding_box_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  std::set<std::string> road_marking_labels_;
  std::set<std::string> sign_board_labels_;
  std::set<std::string> bounding_box_labels_;

  void on_map(const HADMapBin & msg);

  pcl::PointNormal to_point_normal(
    const lanelet::ConstPoint3d & from, const lanelet::ConstPoint3d & to) const;

  pcl::PointCloud<pcl::PointNormal> split_line_strings(
    const lanelet::ConstLineStrings3d & line_strings);

  pcl::PointCloud<pcl::PointXYZL> load_bounding_boxes(const lanelet::PolygonLayer & polygons) const;

  lanelet::ConstLineStrings3d extract_specified_line_string(
    const lanelet::LineStringLayer & line_strings, const std::set<std::string> & visible_labels);
  lanelet::ConstPolygons3d extract_specified_polygon(
    const lanelet::PolygonLayer & polygon_layer, const std::set<std::string> & visible_labels);

  MarkerArray make_sign_marker_msg(
    const lanelet::LineStringLayer & line_string_layer, const std::set<std::string> & labels,
    const std::string & ns);
  MarkerArray make_polygon_marker_msg(
    const lanelet::PolygonLayer & polygon_layer, const std::set<std::string> & labels,
    const std::string & ns);

  void publish_additional_marker(const lanelet::LaneletMapPtr & lanelet_map);
};
}  // namespace yabloc::ll2_decomposer

#endif  // YABLOC_COMMON__LL2_DECOMPOSER__LL2_DECOMPOSER_HPP_
