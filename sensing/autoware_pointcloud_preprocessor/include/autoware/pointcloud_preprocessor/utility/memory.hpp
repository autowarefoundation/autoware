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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__MEMORY_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__MEMORY_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware::pointcloud_preprocessor::utils
{
/** \brief Return whether the input PointCloud2 data has the same layout than PointXYZI. That is to
 * say whether you can memcpy from the PointCloud2 data buffer to a PointXYZI */
bool is_data_layout_compatible_with_point_xyzi(const sensor_msgs::msg::PointCloud2 & input);

/** \brief Return whether the input PointCloud2 data has the same layout than PointXYZIRC. That is
 * to say whether you can memcpy from the PointCloud2 data buffer to a PointXYZIRC */
bool is_data_layout_compatible_with_point_xyzirc(const sensor_msgs::msg::PointCloud2 & input);

/** \brief Return whether the input PointCloud2 data has the same layout than PointXYZIRADRT. That
 * is to say whether you can memcpy from the PointCloud2 data buffer to a PointXYZIRADRT */
bool is_data_layout_compatible_with_point_xyziradrt(const sensor_msgs::msg::PointCloud2 & input);

/** \brief Return whether the input PointCloud2 data has the same layout than PointXYZIRCAEDT. That
 * is to say whether you can memcpy from the PointCloud2 data buffer to a PointXYZIRCAEDT */
bool is_data_layout_compatible_with_point_xyzircaedt(const sensor_msgs::msg::PointCloud2 & input);

}  // namespace autoware::pointcloud_preprocessor::utils

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__MEMORY_HPP_
