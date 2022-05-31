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
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: filter.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include "pointcloud_preprocessor/filter.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl/io/io.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pointcloud_preprocessor::Filter::Filter(
  const std::string & filter_name, const rclcpp::NodeOptions & options)
: Node(filter_name, options), filter_field_name_(filter_name)
{
  // Set parameters (moved from NodeletLazy onInit)
  {
    tf_input_frame_ = static_cast<std::string>(declare_parameter("input_frame", ""));
    tf_output_frame_ = static_cast<std::string>(declare_parameter("output_frame", ""));
    max_queue_size_ = static_cast<std::size_t>(declare_parameter("max_queue_size", 5));

    // ---[ Optional parameters
    use_indices_ = static_cast<bool>(declare_parameter("use_indices", false));
    latched_indices_ = static_cast<bool>(declare_parameter("latched_indices", false));
    approximate_sync_ = static_cast<bool>(declare_parameter("approximate_sync", false));

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Filter (as Component) successfully created with the following parameters:"
        << std::endl
        << " - approximate_sync : " << (approximate_sync_ ? "true" : "false") << std::endl
        << " - use_indices      : " << (use_indices_ ? "true" : "false") << std::endl
        << " - latched_indices  : " << (latched_indices_ ? "true" : "false") << std::endl
        << " - max_queue_size   : " << max_queue_size_);
  }

  // Set publisher
  {
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_));
  }

  subscribe();

  // Set tf_listener, tf_buffer.
  setupTF();

  // Set parameter service callback
  set_param_res_filter_ = this->add_on_set_parameters_callback(
    std::bind(&Filter::filterParamCallback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::setupTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::subscribe()
{
  if (use_indices_) {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(
      this, "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());
    sub_indices_filter_.subscribe(
      this, "indices", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());

    if (approximate_sync_) {
      sync_input_indices_a_ = std::make_shared<ApproximateTimeSyncPolicy>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(std::bind(
        &Filter::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_input_indices_e_ = std::make_shared<ExactTimeSyncPolicy>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(std::bind(
        &Filter::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  } else {
    // Subscribe in an old fashion to input only (no filters)
    // CAN'T use auto-type here.
    std::function<void(const PointCloud2ConstPtr msg)> cb = std::bind(
      &Filter::input_indices_callback, this, std::placeholders::_1, PointIndicesConstPtr());
    sub_input_ = create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_), cb);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
    if (approximate_sync_) {
      sync_input_indices_a_.reset();
    } else {
      sync_input_indices_e_.reset();
    }
  } else {
    sub_input_.reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::computePublish(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices)
{
  auto output = std::make_unique<PointCloud2>();

  // Call the virtual method in the child
  filter(input, indices, *output);

  // Check whether the user has given a different output TF frame
  if (!tf_output_frame_.empty() && output->header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[computePublish] Transforming output dataset from %s to %s.",
      output->header.frame_id.c_str(), tf_output_frame_.c_str());
    // Convert the cloud into the different frame
    auto cloud_transformed = std::make_unique<PointCloud2>();
    if (!pcl_ros::transformPointCloud(tf_output_frame_, *output, *cloud_transformed, *tf_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(), "[computePublish] Error converting output dataset from %s to %s.",
        output->header.frame_id.c_str(), tf_output_frame_.c_str());
      return;
    }
    output = std::move(cloud_transformed);
  }
  if (tf_output_frame_.empty() && output->header.frame_id != tf_input_orig_frame_) {
    // no tf_output_frame given, transform the dataset to its original frame
    RCLCPP_DEBUG(
      this->get_logger(), "[computePublish] Transforming output dataset from %s back to %s.",
      output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());
    // Convert the cloud into the different frame
    auto cloud_transformed = std::make_unique<PointCloud2>();
    if (!pcl_ros::transformPointCloud(
          tf_input_orig_frame_, *output, *cloud_transformed, *tf_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(), "[computePublish] Error converting output dataset from %s back to %s.",
        output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());
      return;
    }
    output = std::move(cloud_transformed);
  }

  // Copy timestamp to keep it
  output->header.stamp = input->header.stamp;

  // Publish a boost shared ptr
  pub_output_->publish(std::move(output));
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult pointcloud_preprocessor::Filter::filterParamCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "input_frame", tf_input_frame_)) {
    RCLCPP_DEBUG(get_logger(), "Setting the input TF frame to: %s.", tf_input_frame_.c_str());
  }
  if (get_param(p, "output_frame", tf_output_frame_)) {
    RCLCPP_DEBUG(get_logger(), "Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::input_indices_callback(
  const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid input!");
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid indices!");
    return;
  }

  /// DEBUG
  if (indices) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback]\n"
      "   - PointCloud with %d data points (%s), stamp %f, and frame %s on input topic received.\n"
      "   - PointIndices with %zu values, stamp %f, and frame %s on indices topic received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str(),
      indices->indices.size(), rclcpp::Time(indices->header.stamp).seconds(),
      indices->header.frame_id.c_str());
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback] PointCloud with %d data points and frame %s on input topic "
      "received.",
      cloud->width * cloud->height, cloud->header.frame_id.c_str());
  }
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2ConstPtr cloud_tf;
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[input_indices_callback] Transforming input dataset from %s to %s.",
      cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;

    if (!tf_buffer_->canTransform(
          tf_input_frame_, cloud->header.frame_id, this->now(),
          rclcpp::Duration::from_seconds(1.0))) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "[input_indices_callback] timeout tf: " << cloud->header.frame_id
                                                                    << "->" << tf_input_frame_);
      return;
    }

    if (!pcl_ros::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, *tf_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[input_indices_callback] Error converting input dataset from %s to %s.",
        cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
      return;
    }
    cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
  } else {
    cloud_tf = cloud;
  }
  // Need setInputCloud () here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  computePublish(cloud_tf, vindices);
}
