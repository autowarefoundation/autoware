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

#ifndef POINTCLOUD_PREPROCESSOR__PASSTHROUGH_FILTER__PASSTHROUGH_FILTER_UINT16_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__PASSTHROUGH_FILTER__PASSTHROUGH_FILTER_UINT16_NODELET_HPP_

#include "pointcloud_preprocessor/filter.hpp"
#include "pointcloud_preprocessor/passthrough_filter/passthrough_uint16.hpp"

#include <pcl/search/pcl_search.h>

#include <vector>

namespace pointcloud_preprocessor
{
class PassThroughFilterUInt16Component : public pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

private:
  pcl::PassThroughUInt16<pcl::PCLPointCloud2> impl_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PassThroughFilterUInt16Component(const rclcpp::NodeOptions & options);
};
}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__PASSTHROUGH_FILTER__PASSTHROUGH_FILTER_UINT16_NODELET_HPP_
