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

#ifndef COMPARE_MAP_SEGMENTATION__COMPARE_ELEVATION_MAP_FILTER_NODE_HPP_
#define COMPARE_MAP_SEGMENTATION__COMPARE_ELEVATION_MAP_FILTER_NODE_HPP_

#include "pointcloud_preprocessor/filter.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
namespace compare_map_segmentation
{
class CompareElevationMapFilterComponent : public pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

private:
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_map_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cloud_;
  grid_map::GridMap elevation_map_;
  cv::Mat elevation_image_;
  grid_map::Matrix elevation_map_data_;
  std::string layer_name_;
  std::string map_frame_;
  double height_diff_thresh_;

  void setVerbosityLevelToDebugIfFlagSet();
  void processPointcloud(grid_map::GridMapPclLoader * gridMapPclLoader);
  void elevationMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr elevation_map);

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit CompareElevationMapFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace compare_map_segmentation

#endif  // COMPARE_MAP_SEGMENTATION__COMPARE_ELEVATION_MAP_FILTER_NODE_HPP_
