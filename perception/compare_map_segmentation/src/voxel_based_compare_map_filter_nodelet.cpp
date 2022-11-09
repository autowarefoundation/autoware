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

#include "compare_map_segmentation/voxel_based_compare_map_filter_nodelet.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <vector>

namespace compare_map_segmentation
{
using pointcloud_preprocessor::get_param;

VoxelBasedCompareMapFilterComponent::VoxelBasedCompareMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("VoxelBasedCompareMapFilter", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "voxel_based_compare_map_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  distance_threshold_ = static_cast<double>(declare_parameter("distance_threshold", 0.3));

  set_map_in_voxel_grid_ = false;

  using std::placeholders::_1;
  sub_map_ = this->create_subscription<PointCloud2>(
    "map", rclcpp::QoS{1}.transient_local(),
    std::bind(&VoxelBasedCompareMapFilterComponent::input_target_callback, this, _1));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&VoxelBasedCompareMapFilterComponent::paramCallback, this, _1));
}

void VoxelBasedCompareMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);
  if (voxel_map_ptr_ == NULL) {
    output = *input;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    const pcl::PointXYZ point = pcl_input->points.at(i);
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y, point.z), point, distance_threshold_, voxel_map_ptr_,
          voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y - distance_threshold_, point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y - distance_threshold_, point.z), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y - distance_threshold_, point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y, point.z - distance_threshold_), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y, point.z + distance_threshold_), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y + distance_threshold_, point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y + distance_threshold_, point.z), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x, point.y + distance_threshold_, point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }

    if (is_in_voxel(
          pcl::PointXYZ(
            point.x - distance_threshold_, point.y - distance_threshold_,
            point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y - distance_threshold_, point.z),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x - distance_threshold_, point.y - distance_threshold_,
            point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y, point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y, point.z), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y, point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x - distance_threshold_, point.y + distance_threshold_,
            point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x - distance_threshold_, point.y + distance_threshold_, point.z),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x - distance_threshold_, point.y + distance_threshold_,
            point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }

    if (is_in_voxel(
          pcl::PointXYZ(
            point.x + distance_threshold_, point.y - distance_threshold_,
            point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y - distance_threshold_, point.z),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x + distance_threshold_, point.y - distance_threshold_,
            point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y, point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y, point.z), point,
          distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y, point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x + distance_threshold_, point.y + distance_threshold_,
            point.z - distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(point.x + distance_threshold_, point.y + distance_threshold_, point.z),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }
    if (is_in_voxel(
          pcl::PointXYZ(
            point.x + distance_threshold_, point.y + distance_threshold_,
            point.z + distance_threshold_),
          point, distance_threshold_, voxel_map_ptr_, voxel_grid_)) {
      continue;
    }

    pcl_output->points.push_back(pcl_input->points.at(i));
  }
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

bool VoxelBasedCompareMapFilterComponent::is_in_voxel(
  const pcl::PointXYZ & src_point, const pcl::PointXYZ & target_point,
  const double distance_threshold, const PointCloudPtr & map,
  pcl::VoxelGrid<pcl::PointXYZ> & voxel) const
{
  int voxel_index =
    voxel.getCentroidIndexAt(voxel.getGridCoordinates(src_point.x, src_point.y, src_point.z));
  if (voxel_index != -1) {  // not empty voxel
    const double dist_x = map->points.at(voxel_index).x - target_point.x;
    const double dist_y = map->points.at(voxel_index).y - target_point.y;
    const double dist_z = map->points.at(voxel_index).z - target_point.z;
    const double sqr_distance = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
    if (sqr_distance < distance_threshold * distance_threshold) {
      return true;
    }
  }
  return false;
}

void VoxelBasedCompareMapFilterComponent::input_target_callback(const PointCloud2ConstPtr map)
{
  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::fromROSMsg<pcl::PointXYZ>(*map, map_pcl);
  const auto map_pcl_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_pcl);

  std::scoped_lock lock(mutex_);
  set_map_in_voxel_grid_ = true;
  tf_input_frame_ = map_pcl_ptr->header.frame_id;
  voxel_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(distance_threshold_, distance_threshold_, distance_threshold_);
  voxel_grid_.setInputCloud(map_pcl_ptr);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr_);
}

rcl_interfaces::msg::SetParametersResult VoxelBasedCompareMapFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "distance_threshold", distance_threshold_)) {
    voxel_grid_.setLeafSize(distance_threshold_, distance_threshold_, distance_threshold_);
    voxel_grid_.setSaveLeafLayout(true);
    if (set_map_in_voxel_grid_) {
      voxel_grid_.filter(*voxel_map_ptr_);
    }
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", distance_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace compare_map_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(compare_map_segmentation::VoxelBasedCompareMapFilterComponent)
