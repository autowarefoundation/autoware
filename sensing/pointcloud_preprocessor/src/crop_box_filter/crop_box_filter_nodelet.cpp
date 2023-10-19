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
/*
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: cropbox.cpp
 *
 */

#include "pointcloud_preprocessor/crop_box_filter/crop_box_filter_nodelet.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>

namespace pointcloud_preprocessor
{
CropBoxFilterComponent::CropBoxFilterComponent(const rclcpp::NodeOptions & options)
: Filter("CropBoxFilter", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "crop_box_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // set initial parameters
  {
    auto & p = param_;
    p.min_x = static_cast<float>(declare_parameter("min_x", -1.0));
    p.min_y = static_cast<float>(declare_parameter("min_y", -1.0));
    p.min_z = static_cast<float>(declare_parameter("min_z", -1.0));
    p.max_x = static_cast<float>(declare_parameter("max_x", 1.0));
    p.max_y = static_cast<float>(declare_parameter("max_y", 1.0));
    p.max_z = static_cast<float>(declare_parameter("max_z", 1.0));
    p.negative = static_cast<bool>(declare_parameter("negative", false));
    if (tf_input_frame_.empty()) {
      throw std::invalid_argument("Crop box requires non-empty input_frame");
    }
  }

  // set additional publishers
  {
    crop_box_polygon_pub_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>("~/crop_box_polygon", 10);
  }

  // set parameter service callback
  {
    using std::placeholders::_1;
    set_param_res_ = this->add_on_set_parameters_callback(
      std::bind(&CropBoxFilterComponent::paramCallback, this, _1));
  }
}

// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API.
void CropBoxFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)input;
  (void)indices;
  (void)output;
}

// TODO(sykwer): Temporary Implementation: Rename this function to `filter()` when all the filter
// nodes conform to new API. Then delete the old `filter()` defined above.
void CropBoxFilterComponent::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);

  if (indices) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Indices are not supported and will be ignored");
  }

  int x_offset = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  int y_offset = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  int z_offset = input->fields[pcl::getFieldIndex(*input, "z")].offset;

  output.data.resize(input->data.size());
  size_t output_size = 0;

  int skipped_count = 0;

  for (size_t global_offset = 0; global_offset + input->point_step <= input->data.size();
       global_offset += input->point_step) {
    Eigen::Vector4f point;
    std::memcpy(&point[0], &input->data[global_offset + x_offset], sizeof(float));
    std::memcpy(&point[1], &input->data[global_offset + y_offset], sizeof(float));
    std::memcpy(&point[2], &input->data[global_offset + z_offset], sizeof(float));
    point[3] = 1;

    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      skipped_count++;
      continue;
    }

    if (transform_info.need_transform) {
      point = transform_info.eigen_transform * point;
    }

    bool point_is_inside = point[2] > param_.min_z && point[2] < param_.max_z &&
                           point[1] > param_.min_y && point[1] < param_.max_y &&
                           point[0] > param_.min_x && point[0] < param_.max_x;
    if ((!param_.negative && point_is_inside) || (param_.negative && !point_is_inside)) {
      memcpy(&output.data[output_size], &input->data[global_offset], input->point_step);

      if (transform_info.need_transform) {
        std::memcpy(&output.data[output_size + x_offset], &point[0], sizeof(float));
        std::memcpy(&output.data[output_size + y_offset], &point[1], sizeof(float));
        std::memcpy(&output.data[output_size + z_offset], &point[2], sizeof(float));
      }

      output_size += input->point_step;
    }
  }

  if (skipped_count > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "%d points contained NaN values and have been ignored",
      skipped_count);
  }

  output.data.resize(output_size);

  // Note that tf_input_orig_frame_ is the input frame, while tf_input_frame_ is the frame of the
  // crop box
  output.header.frame_id = tf_input_frame_;

  output.height = 1;
  output.fields = input->fields;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.is_dense = input->is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  publishCropBoxPolygon();

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

void CropBoxFilterComponent::publishCropBoxPolygon()
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  const double x1 = param_.max_x;
  const double x2 = param_.min_x;
  const double x3 = param_.min_x;
  const double x4 = param_.max_x;

  const double y1 = param_.max_y;
  const double y2 = param_.max_y;
  const double y3 = param_.min_y;
  const double y4 = param_.min_y;

  const double z1 = param_.min_z;
  const double z2 = param_.max_z;

  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = tf_input_frame_;
  polygon_msg.header.stamp = get_clock()->now();
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  crop_box_polygon_pub_->publish(polygon_msg);
}

rcl_interfaces::msg::SetParametersResult CropBoxFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  CropBoxParam new_param{};

  if (
    get_param(p, "min_x", new_param.min_x) && get_param(p, "min_y", new_param.min_y) &&
    get_param(p, "min_z", new_param.min_z) && get_param(p, "max_x", new_param.max_x) &&
    get_param(p, "max_y", new_param.max_y) && get_param(p, "max_z", new_param.max_z) &&
    get_param(p, "negative", new_param.negative)) {
    if (
      param_.min_x != new_param.min_x || param_.max_x != new_param.max_x ||
      param_.min_y != new_param.min_y || param_.max_y != new_param.max_y ||
      param_.min_z != new_param.min_z || param_.max_z != new_param.max_z ||
      param_.negative != new_param.negative) {
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the minimum point to: %f %f %f.", get_name(),
        new_param.min_x, new_param.min_y, new_param.min_z);
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the minimum point to: %f %f %f.", get_name(),
        new_param.max_x, new_param.max_y, new_param.max_z);
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the filter negative flag to: %s.", get_name(),
        new_param.negative ? "true" : "false");
      param_ = new_param;
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::CropBoxFilterComponent)
