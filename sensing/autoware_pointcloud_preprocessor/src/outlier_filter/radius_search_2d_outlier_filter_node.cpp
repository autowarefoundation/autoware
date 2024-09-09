// Copyright 2020 Tier IV, Inc.
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

#include "autoware/pointcloud_preprocessor/outlier_filter/radius_search_2d_outlier_filter_node.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <vector>

namespace autoware::pointcloud_preprocessor
{
RadiusSearch2DOutlierFilterComponent::RadiusSearch2DOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("RadiusSearch2DOutlierFilter", options)
{
  // set initial parameters
  {
    min_neighbors_ = static_cast<size_t>(declare_parameter<int64_t>("min_neighbors"));
    search_radius_ = declare_parameter<double>("search_radius");
  }

  kd_tree_ = pcl::make_shared<pcl::search::KdTree<pcl::PointXY>>(false);

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadiusSearch2DOutlierFilterComponent::paramCallback, this, _1));
}

void RadiusSearch2DOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *xyz_cloud);

  pcl::PointCloud<pcl::PointXY>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXY>);
  xy_cloud->points.resize(xyz_cloud->points.size());
  for (size_t i = 0; i < xyz_cloud->points.size(); ++i) {
    xy_cloud->points[i].x = xyz_cloud->points[i].x;
    xy_cloud->points[i].y = xyz_cloud->points[i].y;
  }

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_sqr_distances(xy_cloud->points.size());
  kd_tree_->setInputCloud(xy_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < xy_cloud->points.size(); ++i) {
    size_t k = kd_tree_->radiusSearch(i, search_radius_, k_indices, k_sqr_distances);
    if (k >= min_neighbors_) {
      pcl_output->points.push_back(xyz_cloud->points.at(i));
    }
  }
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult RadiusSearch2DOutlierFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "min_neighbors", min_neighbors_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new min neighbors to: %zu.", min_neighbors_);
  }
  if (get_param(p, "search_radius", search_radius_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new search radius to: %f.", search_radius_);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::pointcloud_preprocessor::RadiusSearch2DOutlierFilterComponent)
