// Copyright 2023 Tier IV, Inc.
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

#ifndef TIER4_AUTOWARE_UTILS__ROS__PCL_CONVERSION_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__PCL_CONVERSION_HPP_

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>

namespace tier4_autoware_utils
{
/**
 * @brief a faster implementation of converting sensor_msgs::msg::PointCloud2 to
 * pcl::PointCloud<pcl::PointXYZ> and transform the cloud
 *
 * @tparam Scalar
 * @param cloud      input PointCloud2 message
 * @param pcl_cloud  output transformed pcl cloud
 * @param transform  eigen transformation matrix
 */
template <typename Scalar>
void transformPointCloudFromROSMsg(
  const sensor_msgs::msg::PointCloud2 & cloud, pcl::PointCloud<pcl::PointXYZ> & pcl_cloud,
  const Eigen::Matrix<Scalar, 4, 4> & transform)
{
  // Copy info fields
  pcl_conversions::toPCL(cloud.header, pcl_cloud.header);
  pcl_cloud.width = cloud.width;
  pcl_cloud.height = cloud.height;
  pcl_cloud.is_dense = cloud.is_dense == 1;

  pcl::MsgFieldMap field_map;
  std::vector<pcl::PCLPointField> msg_fields;
  pcl_conversions::toPCL(cloud.fields, msg_fields);
  pcl::createMapping<pcl::PointXYZ>(msg_fields, field_map);

  // transform point data
  std::uint32_t num_points = cloud.width * cloud.height;
  pcl_cloud.points.resize(num_points);
  std::uint8_t * cloud_data = reinterpret_cast<std::uint8_t *>(&pcl_cloud.points[0]);
  pcl::detail::Transformer<Scalar> tf(transform);

  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const std::uint8_t * row_data = &cloud.data[row * cloud.row_step];
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::uint8_t * msg_data = row_data + col * cloud.point_step;
      for (const pcl::detail::FieldMapping & mapping : field_map) {
        const float * msg_ptr = reinterpret_cast<const float *>(msg_data);
        float * pcl_ptr = reinterpret_cast<float *>(cloud_data + mapping.struct_offset);
        tf.se3(msg_ptr, pcl_ptr);
      }
      cloud_data += sizeof(pcl::PointXYZ);
    }
  }
}

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__ROS__PCL_CONVERSION_HPP_
