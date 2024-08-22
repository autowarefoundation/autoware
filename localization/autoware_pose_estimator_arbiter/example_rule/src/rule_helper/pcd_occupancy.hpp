// Copyright 2023 Autoware Foundation
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

#ifndef RULE_HELPER__PCD_OCCUPANCY_HPP_
#define RULE_HELPER__PCD_OCCUPANCY_HPP_

#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace autoware::pose_estimator_arbiter::rule_helper
{
class PcdOccupancy
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
  explicit PcdOccupancy(int pcd_density_upper_threshold, int pcd_density_lower_threshold);

  [[nodiscard]] MarkerArray debug_marker_array() const;
  void init(PointCloud2::ConstSharedPtr msg);
  bool ndt_can_operate(
    const geometry_msgs::msg::Point & position, std::string * optional_message = nullptr) const;

private:
  const int pcd_density_upper_threshold_;
  const int pcd_density_lower_threshold_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_areas_{nullptr};
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};
};

}  // namespace autoware::pose_estimator_arbiter::rule_helper

#endif  // RULE_HELPER__PCD_OCCUPANCY_HPP_
