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

#ifndef POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__RING_OUTLIER_FILTER_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__RING_OUTLIER_FILTER_NODELET_HPP_

#include "autoware_point_types/types.hpp"
#include "pointcloud_preprocessor/filter.hpp"

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <vector>

namespace pointcloud_preprocessor
{
using autoware_point_types::PointXYZI;
using point_cloud_msg_wrapper::PointCloud2Modifier;

class RingOutlierFilterComponent : public pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

private:
  double distance_ratio_;
  double object_length_threshold_;
  int num_points_threshold_;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

  bool isCluster(
    sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
    const std::vector<std::size_t> & tmp_indices)
  {
    PointXYZI * front_pt = reinterpret_cast<PointXYZI *>(&input_ptr->data[tmp_indices.front()]);
    PointXYZI * back_pt = reinterpret_cast<PointXYZI *>(&input_ptr->data[tmp_indices.back()]);

    const auto x_diff = front_pt->x - back_pt->x;
    const auto y_diff = front_pt->y - back_pt->y;
    const auto z_diff = front_pt->z - back_pt->z;
    return static_cast<int>(tmp_indices.size()) > num_points_threshold_ ||
           (x_diff * x_diff) + (y_diff * y_diff) + (z_diff * z_diff) >=
             object_length_threshold_ * object_length_threshold_;
  }

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit RingOutlierFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace pointcloud_preprocessor
#endif  // POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__RING_OUTLIER_FILTER_NODELET_HPP_
