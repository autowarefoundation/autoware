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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__POINTCLOUD_ACCUMULATOR__POINTCLOUD_ACCUMULATOR_NODE_HPP_  // NOLINT
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__POINTCLOUD_ACCUMULATOR__POINTCLOUD_ACCUMULATOR_NODE_HPP_  // NOLINT

#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <boost/circular_buffer.hpp>

#include <vector>

namespace autoware::pointcloud_preprocessor
{
class PointcloudAccumulatorComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

private:
  double accumulation_time_sec_;
  boost::circular_buffer<PointCloud2ConstPtr> pointcloud_buffer_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PointcloudAccumulatorComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::pointcloud_preprocessor

// clang-format off
#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__POINTCLOUD_ACCUMULATOR__POINTCLOUD_ACCUMULATOR_NODE_HPP_  // NOLINT
// clang-format on
