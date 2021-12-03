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

#include <vector>

namespace pointcloud_preprocessor
{
CropBoxFilterComponent::CropBoxFilterComponent(const rclcpp::NodeOptions & options)
: Filter("CropBoxFilter", options)
{
  // set initial parameters
  {
    Eigen::Vector4f new_min_point = Eigen::Vector4f::Zero();
    new_min_point(0) = static_cast<float>(declare_parameter("min_x", -1.0));
    new_min_point(1) = static_cast<float>(declare_parameter("min_y", -1.0));
    new_min_point(2) = static_cast<float>(declare_parameter("min_z", -1.0));
    impl_.setMin(new_min_point);

    Eigen::Vector4f new_max_point = Eigen::Vector4f::Zero();
    new_max_point(0) = static_cast<float>(declare_parameter("max_x", 1.0));
    new_max_point(1) = static_cast<float>(declare_parameter("max_y", 1.0));
    new_max_point(2) = static_cast<float>(declare_parameter("max_z", 1.0));
    impl_.setMax(new_max_point);

    impl_.setKeepOrganized(static_cast<bool>(declare_parameter("keep_organized", false)));
    impl_.setNegative(static_cast<bool>(declare_parameter("negative", false)));
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

void CropBoxFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*(input), *(pcl_input));
  impl_.setInputCloud(pcl_input);
  impl_.setIndices(indices);
  pcl::PCLPointCloud2 pcl_output;
  impl_.filter(pcl_output);
  pcl_conversions::moveFromPCL(pcl_output, output);

  publishCropBoxPolygon();
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

  const double x1 = impl_.getMax()(0);
  const double x2 = impl_.getMin()(0);
  const double x3 = impl_.getMin()(0);
  const double x4 = impl_.getMax()(0);

  const double y1 = impl_.getMax()(1);
  const double y2 = impl_.getMax()(1);
  const double y3 = impl_.getMin()(1);
  const double y4 = impl_.getMin()(1);

  const double z1 = impl_.getMin()(2);
  const double z2 = impl_.getMax()(2);

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
  boost::mutex::scoped_lock lock(mutex_);

  Eigen::Vector4f min_point, max_point;
  min_point = impl_.getMin();
  max_point = impl_.getMax();

  Eigen::Vector4f new_min_point = Eigen::Vector4f::Zero();
  Eigen::Vector4f new_max_point = Eigen::Vector4f::Zero();

  // Check the current values for minimum point
  if (
    get_param(p, "min_x", new_min_point(0)) && get_param(p, "min_y", new_min_point(1)) &&
    get_param(p, "min_z", new_min_point(2))) {
    if (min_point != new_min_point) {
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the minimum point to: %f %f %f.", get_name(),
        new_min_point(0), new_min_point(1), new_min_point(2));
      // Set the filter min point if different
      impl_.setMin(new_min_point);
    }
  }

  // Check the current values for the maximum point
  if (
    get_param(p, "max_x", new_max_point(0)) && get_param(p, "max_y", new_max_point(1)) &&
    get_param(p, "max_z", new_max_point(2))) {
    if (max_point != new_max_point) {
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the maximum point to: %f %f %f.", get_name(),
        new_max_point(0), new_max_point(1), new_max_point(2));
      // Set the filter max point if different
      impl_.setMax(new_max_point);
    }
  }

  // Check the current value for keep_organized
  bool keep_organized;
  if (get_param(p, "keep_organized", keep_organized)) {
    if (impl_.getKeepOrganized() != keep_organized) {
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the filter keep_organized value to: %s.",
        get_name(), keep_organized ? "true" : "false");
      // Call the virtual method in the child
      impl_.setKeepOrganized(keep_organized);
    }
  }

  // Check the current value for the negative flag
  bool negative;
  if (get_param(p, "negative", negative)) {
    if (impl_.getNegative() != negative) {
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the filter negative flag to: %s.", get_name(),
        negative ? "true" : "false");
      // Call the virtual method in the child
      impl_.setNegative(negative);
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
