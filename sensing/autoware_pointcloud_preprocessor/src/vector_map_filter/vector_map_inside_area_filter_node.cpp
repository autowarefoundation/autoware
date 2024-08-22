// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/vector_map_filter/vector_map_inside_area_filter_node.hpp"

namespace
{
autoware::universe_utils::Box2d calcBoundingBox(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud)
{
  MultiPoint2d candidate_points;
  for (const auto & p : input_cloud->points) {
    candidate_points.emplace_back(p.x, p.y);
  }

  return boost::geometry::return_envelope<autoware::universe_utils::Box2d>(candidate_points);
}

lanelet::ConstPolygons3d calcIntersectedPolygons(
  const autoware::universe_utils::Box2d & bounding_box, const lanelet::ConstPolygons3d & polygons)
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
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, const lanelet::ConstPolygons3d & polygons,
  const std::optional<float> & z_threshold_)
{
  std::vector<PolygonCgal> cgal_polys;

  for (const auto & polygon : polygons) {
    const auto lanelet_poly = lanelet::utils::to2D(polygon).basicPolygon();
    PolygonCgal cgal_poly;
    autoware::pointcloud_preprocessor::utils::to_cgal_polygon(lanelet_poly, cgal_poly);
    cgal_polys.emplace_back(cgal_poly);
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  autoware::pointcloud_preprocessor::utils::remove_polygon_cgal_from_cloud(
    *cloud_in, cgal_polys, filtered_cloud, z_threshold_);

  return filtered_cloud;
}

}  // anonymous namespace

namespace autoware::pointcloud_preprocessor
{
VectorMapInsideAreaFilterComponent::VectorMapInsideAreaFilterComponent(
  const rclcpp::NodeOptions & node_options)
: Filter("VectorMapInsideAreaFilter", node_options)
{
  polygon_type_ = declare_parameter<std::string>("polygon_type");

  using std::placeholders::_1;
  // Set subscriber
  map_sub_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&VectorMapInsideAreaFilterComponent::mapCallback, this, _1));

  // Set parameters
  use_z_filter_ = declare_parameter<bool>("use_z_filter");
  z_threshold_ = declare_parameter<float>("z_threshold");  // defined in the base_link frame

  // Set tf
  {
    rclcpp::Clock::SharedPtr ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_clock);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
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
  std::optional<float> z_threshold_in_base_link = std::nullopt;
  if (use_z_filter_) {
    // assume z_max is defined in the base_link frame
    const std::string base_link_frame = "base_link";
    z_threshold_in_base_link = z_threshold_;
    if (input->header.frame_id != base_link_frame) {
      try {
        // get z difference from baselink to input frame
        const auto transform =
          tf_buffer_->lookupTransform(input->header.frame_id, base_link_frame, input->header.stamp);
        *z_threshold_in_base_link += transform.transform.translation.z;
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "Failed to transform z_threshold to base_link frame");
        z_threshold_in_base_link = std::nullopt;
      }
    }
  }
  const auto filtered_pc =
    removePointsWithinPolygons(pc_input, intersected_lanelets, z_threshold_in_base_link);

  // convert to ROS message
  pcl::toROSMsg(filtered_pc, output);
  output.header = input->header;
}

void VectorMapInsideAreaFilterComponent::mapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg)
{
  tf_input_frame_ = map_msg->header.frame_id;

  const auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr);
  polygon_lanelets_ = lanelet::utils::query::getAllPolygonsByType(lanelet_map_ptr, polygon_type_);
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::pointcloud_preprocessor::VectorMapInsideAreaFilterComponent)
