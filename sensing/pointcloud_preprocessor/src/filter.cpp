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
#include <set>
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

  subscribe(filter_name);

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
  std::string filter_name = "";
  subscribe(filter_name);
}

void pointcloud_preprocessor::Filter::subscribe(const std::string & filter_name)
{
  // TODO(sykwer): Change the corresponding node to subscribe to `faster_input_indices_callback`
  // each time a child class supports the faster version.
  // When all the child classes support the faster version, this workaround is deleted.
  std::set<std::string> supported_nodes = {
    "CropBoxFilter", "RingOutlierFilter", "VoxelGridDownsampleFilter"};
  auto callback = supported_nodes.find(filter_name) != supported_nodes.end()
                    ? &Filter::faster_input_indices_callback
                    : &Filter::input_indices_callback;

  if (use_indices_) {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(
      this, "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());
    sub_indices_filter_.subscribe(
      this, "indices", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());

    if (approximate_sync_) {
      sync_input_indices_a_ = std::make_shared<ApproximateTimeSyncPolicy>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(
        std::bind(callback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_input_indices_e_ = std::make_shared<ExactTimeSyncPolicy>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(
        std::bind(callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  } else {
    // Subscribe in an old fashion to input only (no filters)
    // CAN'T use auto-type here.
    std::function<void(const PointCloud2ConstPtr msg)> cb =
      std::bind(callback, this, std::placeholders::_1, PointIndicesConstPtr());
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
// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API.
void pointcloud_preprocessor::Filter::computePublish(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices)
{
  auto output = std::make_unique<PointCloud2>();

  // Call the virtual method in the child
  filter(input, indices, *output);

  if (!convert_output_costly(output)) return;

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
// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API.
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

// For performance reason, we get only a transformation matrix here.
// The implementation is based on the one shown in the URL below.
// https://github.com/ros-perception/perception_pcl/blob/628aaec1dc73ef4adea01e9d28f11eb417b948fd/pcl_ros/src/transforms.cpp#L61-L94
bool pointcloud_preprocessor::Filter::_calculate_transform_matrix(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
  const tf2_ros::Buffer & tf_buffer, Eigen::Matrix4f & eigen_transform /*output*/)
{
  if (from.header.frame_id == target_frame) {
    eigen_transform = Eigen::Matrix4f::Identity(4, 4);
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer.lookupTransform(
      target_frame, from.header.frame_id, tf2_ros::fromMsg(from.header.stamp));
  } catch (tf2::LookupException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return false;
  } catch (tf2::ExtrapolationException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return false;
  }

  pcl_ros::transformAsMatrix(transform, eigen_transform);
  return true;
}

// Returns false in error cases
bool pointcloud_preprocessor::Filter::calculate_transform_matrix(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
  TransformInfo & transform_info /*output*/)
{
  transform_info.need_transform = false;

  if (target_frame.empty() || from.header.frame_id == target_frame) return true;

  RCLCPP_DEBUG(
    this->get_logger(), "[get_transform_matrix] Transforming input dataset from %s to %s.",
    from.header.frame_id.c_str(), target_frame.c_str());

  if (!tf_buffer_->canTransform(
        target_frame, from.header.frame_id, this->now(), rclcpp::Duration::from_seconds(1.0))) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "[get_transform_matrix] timeout tf: " << from.header.frame_id << "->" << target_frame);
    return false;
  }

  if (!_calculate_transform_matrix(
        target_frame, from, *tf_buffer_, transform_info.eigen_transform /*output*/)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[calculate_transform_matrix] Error converting input dataset from %s to %s.",
      from.header.frame_id.c_str(), target_frame.c_str());
    return false;
  }

  transform_info.need_transform = true;
  return true;
}

// Returns false in error cases
bool pointcloud_preprocessor::Filter::convert_output_costly(std::unique_ptr<PointCloud2> & output)
{
  // In terms of performance, we should avoid using pcl_ros library function,
  // but this code path isn't reached in the main use case of Autoware, so it's left as is for now.
  if (!tf_output_frame_.empty() && output->header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s to %s.",
      output->header.frame_id.c_str(), tf_output_frame_.c_str());

    // Convert the cloud into the different frame
    auto cloud_transformed = std::make_unique<PointCloud2>();
    if (!pcl_ros::transformPointCloud(tf_output_frame_, *output, *cloud_transformed, *tf_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[convert_output_costly] Error converting output dataset from %s to %s.",
        output->header.frame_id.c_str(), tf_output_frame_.c_str());
      return false;
    }

    output = std::move(cloud_transformed);
  }

  // Same as the comment above
  if (tf_output_frame_.empty() && output->header.frame_id != tf_input_orig_frame_) {
    // No tf_output_frame given, transform the dataset to its original frame
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s back to %s.",
      output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());

    auto cloud_transformed = std::make_unique<PointCloud2>();
    if (!pcl_ros::transformPointCloud(
          tf_input_orig_frame_, *output, *cloud_transformed, *tf_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[convert_output_costly] Error converting output dataset from %s back to %s.",
        output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());
      return false;
    }

    output = std::move(cloud_transformed);
  }

  return true;
}

// TODO(sykwer): Temporary Implementation: Rename this function to `input_indices_callback()` when
// all the filter nodes conform to new API. Then delete the old `input_indices_callback()` defined
// above.
void pointcloud_preprocessor::Filter::faster_input_indices_callback(
  const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices)
{
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid input!");
    return;
  }

  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid indices!");
    return;
  }

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

  tf_input_orig_frame_ = cloud->header.frame_id;

  // For performance reason, defer the transform computation.
  // Do not use pcl_ros::transformPointCloud(). It's too slow due to the unnecessary copy.
  TransformInfo transform_info;
  if (!calculate_transform_matrix(tf_input_frame_, *cloud, transform_info)) return;

  // Need setInputCloud() here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  auto output = std::make_unique<PointCloud2>();

  // TODO(sykwer): Change to `filter()` call after when the filter nodes conform to new API.
  faster_filter(cloud, vindices, *output, transform_info);

  if (!convert_output_costly(output)) return;

  output->header.stamp = cloud->header.stamp;
  pub_output_->publish(std::move(output));
}

// TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
// to new API. It's not a pure virtual function so that a child class does not have to implement
// this function.
void pointcloud_preprocessor::Filter::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  (void)input;
  (void)indices;
  (void)output;
  (void)transform_info;
}
