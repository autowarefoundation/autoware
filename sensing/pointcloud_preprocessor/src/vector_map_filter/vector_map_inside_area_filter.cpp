// Copyright 2022 Tier IV, Inc.
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

#include "pointcloud_preprocessor/vector_map_filter/vector_map_inside_area_filter.hpp"

namespace
{
tier4_autoware_utils::Box2d calcBoundingBox(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud)
{
  MultiPoint2d candidate_points;
  for (const auto & p : input_cloud->points) {
    candidate_points.emplace_back(p.x, p.y);
  }

  return boost::geometry::return_envelope<tier4_autoware_utils::Box2d>(candidate_points);
}

lanelet::ConstPolygons3d calcIntersectedPolygons(
  const tier4_autoware_utils::Box2d & bounding_box, const lanelet::ConstPolygons3d & polygons)
{
  lanelet::ConstPolygons3d intersected_polygons;
  for (const auto & polygon : polygons) {
    if (boost::geometry::intersects(bounding_box, lanelet::utils::to2D(polygon).basicPolygon())) {
      intersected_polygons.push_back(polygon);
    }
  }
  return intersected_polygons;
}

pcl::PointCloud<pcl::PointXYZ> removePointsWithinPolygons(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, const lanelet::ConstPolygons3d & polygons)
{
  std::vector<PolygonCgal> cgal_polys;

  for (const auto & polygon : polygons) {
    const auto lanelet_poly = lanelet::utils::to2D(polygon).basicPolygon();
    PolygonCgal cgal_poly;
    pointcloud_preprocessor::utils::to_cgal_polygon(lanelet_poly, cgal_poly);
    cgal_polys.emplace_back(cgal_poly);
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  pointcloud_preprocessor::utils::remove_polygon_cgal_from_cloud(
    *cloud_in, cgal_polys, filtered_cloud);

  return filtered_cloud;
}

}  // anonymous namespace

namespace pointcloud_preprocessor
{
VectorMapInsideAreaFilterComponent::VectorMapInsideAreaFilterComponent(
  const rclcpp::NodeOptions & node_options)
: Filter("VectorMapInsideAreaFilter", node_options)
{
  polygon_type_ =
    static_cast<std::string>(declare_parameter("polygon_type", "no_obstacle_segmentation_area"));

  using std::placeholders::_1;
  // Set subscriber
  map_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&VectorMapInsideAreaFilterComponent::mapCallback, this, _1));
}

void VectorMapInsideAreaFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }

  if (polygon_lanelets_.empty()) {
    output = *input;
    return;
  }

  // convert to PCL message
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*input, *pc_input);

  // calculate bounding box of points
  const auto bounding_box = calcBoundingBox(pc_input);

  // use only intersected lanelets to reduce calculation cost
  const auto intersected_lanelets = calcIntersectedPolygons(bounding_box, polygon_lanelets_);

  // filter pointcloud by lanelet
  const auto filtered_pc = removePointsWithinPolygons(pc_input, intersected_lanelets);

  // convert to ROS message
  pcl::toROSMsg(filtered_pc, output);
  output.header = input->header;
}

void VectorMapInsideAreaFilterComponent::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  tf_input_frame_ = map_msg->header.frame_id;

  const auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr);
  polygon_lanelets_ = lanelet::utils::query::getAllPolygonsByType(lanelet_map_ptr, polygon_type_);
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::VectorMapInsideAreaFilterComponent)
