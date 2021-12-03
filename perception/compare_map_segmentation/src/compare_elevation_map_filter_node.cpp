// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "compare_map_segmentation/compare_elevation_map_filter_node.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <glob.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rcutils/filesystem.h>  // To be replaced by std::filesystem in C++17

#include <memory>
#include <string>
#include <utility>

namespace compare_map_segmentation
{
CompareElevationMapFilterComponent::CompareElevationMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("CompareElevationMapFilter", options)
{
  unsubscribe();
  layer_name_ = this->declare_parameter("map_layer_name", std::string("elevation"));
  height_diff_thresh_ = this->declare_parameter("height_diff_thresh", 0.15);
  map_frame_ = this->declare_parameter("map_frame", "map");

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  sub_map_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
    "input/elevation_map", durable_qos,
    std::bind(
      &CompareElevationMapFilterComponent::elevationMapCallback, this, std::placeholders::_1));
}

void CompareElevationMapFilterComponent::elevationMapCallback(
  const grid_map_msgs::msg::GridMap::ConstSharedPtr elevation_map)
{
  grid_map::GridMapRosConverter::fromMessage(*elevation_map, elevation_map_);
  elevation_map_data_ = elevation_map_.get(layer_name_);

  const float min_value = elevation_map_.get(layer_name_).minCoeffOfFinites();
  const float max_value = elevation_map_.get(layer_name_).maxCoeffOfFinites();
  grid_map::GridMapCvConverter::toImage<uint16_t, 1>(
    elevation_map_, layer_name_, CV_16UC1, min_value, max_value, elevation_image_);
  subscribe();
}

void CompareElevationMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  std::string output_frame = map_frame_;

  output_frame = elevation_map_.getFrameId();
  elevation_map_.setTimestamp(input->header.stamp.nanosec);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  for (const auto & point : pcl_input->points) {
    if (elevation_map_.isInside(grid_map::Position(point.x, point.y))) {
      float elevation_value = elevation_map_.atPosition(
        layer_name_, grid_map::Position(point.x, point.y),
        grid_map::InterpolationMethods::INTER_LINEAR);
      const float height_diff = point.z - elevation_value;
      if (height_diff > height_diff_thresh_) {
        pcl_output->points.push_back(point);
      }
    }
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header.stamp = input->header.stamp;
  output.header.frame_id = output_frame;
}
}  // namespace compare_map_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(compare_map_segmentation::CompareElevationMapFilterComponent)
