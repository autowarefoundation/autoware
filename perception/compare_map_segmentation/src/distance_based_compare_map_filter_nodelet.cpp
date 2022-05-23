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

#include "compare_map_segmentation/distance_based_compare_map_filter_nodelet.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <vector>

namespace compare_map_segmentation
{
using pointcloud_preprocessor::get_param;

DistanceBasedCompareMapFilterComponent::DistanceBasedCompareMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("DistanceBasedCompareMapFilter", options)
{
  distance_threshold_ = static_cast<double>(declare_parameter("distance_threshold", 0.3));

  using std::placeholders::_1;
  sub_map_ = this->create_subscription<PointCloud2>(
    "map", rclcpp::QoS{1}.transient_local(),
    std::bind(&DistanceBasedCompareMapFilterComponent::input_target_callback, this, _1));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&DistanceBasedCompareMapFilterComponent::paramCallback, this, _1));
}

void DistanceBasedCompareMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (map_ptr_ == NULL || tree_ == NULL) {
    output = *input;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  pcl::getPointCloudDifference<pcl::PointXYZ>(
    *pcl_input, distance_threshold_ * distance_threshold_, tree_, *pcl_output);

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

void DistanceBasedCompareMapFilterComponent::input_target_callback(const PointCloud2ConstPtr map)
{
  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::fromROSMsg<pcl::PointXYZ>(*map, map_pcl);
  const auto map_pcl_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_pcl);

  std::scoped_lock lock(mutex_);
  map_ptr_ = map_pcl_ptr;
  tf_input_frame_ = map_ptr_->header.frame_id;
  if (!tree_) {
    if (map_ptr_->isOrganized()) {
      tree_.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
    } else {
      tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
    }
  }
  tree_->setInputCloud(map_ptr_);
}

rcl_interfaces::msg::SetParametersResult DistanceBasedCompareMapFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "distance_threshold", distance_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", distance_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace compare_map_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(compare_map_segmentation::DistanceBasedCompareMapFilterComponent)
