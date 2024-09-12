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

#include "autoware/pointcloud_preprocessor/pointcloud_accumulator/pointcloud_accumulator_node.hpp"

#include <vector>

namespace autoware::pointcloud_preprocessor
{
PointcloudAccumulatorComponent::PointcloudAccumulatorComponent(const rclcpp::NodeOptions & options)
: Filter("PointcloudAccumulator", options)
{
  // set initial parameters
  {
    accumulation_time_sec_ = declare_parameter<double>("accumulation_time_sec");
    pointcloud_buffer_.set_capacity(
      static_cast<size_t>(declare_parameter<int64_t>("pointcloud_buffer_size")));
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PointcloudAccumulatorComponent::paramCallback, this, _1));
}

void PointcloudAccumulatorComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  pointcloud_buffer_.push_front(input);
  rclcpp::Time last_time = input->header.stamp;
  pcl::PointCloud<pcl::PointXYZ> pcl_input;
  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  for (size_t i = 0; i < pointcloud_buffer_.size(); i++) {
    if (accumulation_time_sec_ < (last_time - pointcloud_buffer_.at(i)->header.stamp).seconds()) {
      break;
    }
    pcl::fromROSMsg(*pointcloud_buffer_.at(i), pcl_input);
    pcl_output += pcl_input;
  }
  pcl::toROSMsg(pcl_output, output);
  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult PointcloudAccumulatorComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "accumulation_time_sec", accumulation_time_sec_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new accumulation time to: %f.", accumulation_time_sec_);
  }
  int pointcloud_buffer_size;
  if (get_param(p, "pointcloud_buffer_size", pointcloud_buffer_size)) {
    pointcloud_buffer_.set_capacity((size_t)pointcloud_buffer_size);
    RCLCPP_DEBUG(get_logger(), "Setting new buffer size to: %d.", pointcloud_buffer_size);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PointcloudAccumulatorComponent)
