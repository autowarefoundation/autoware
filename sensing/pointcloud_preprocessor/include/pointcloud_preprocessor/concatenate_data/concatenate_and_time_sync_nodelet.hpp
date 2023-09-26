// Copyright 2023 TIER IV, Inc.
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
 * $Id: concatenate_data.cpp 35231 2011-01-14 05:33:20Z rusu $
 *
 */

#ifndef POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATE_AND_TIME_SYNC_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATE_AND_TIME_SYNC_NODELET_HPP_

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

// ROS includes
#include "autoware_point_types/types.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <tier4_debug_msgs/msg/string_stamped.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pointcloud_preprocessor
{
using autoware_point_types::PointXYZI;
using point_cloud_msg_wrapper::PointCloud2Modifier;

/** \brief @b PointCloudConcatenateDataSynchronizerComponent is a special form of data
 * synchronizer: it listens for a set of input PointCloud messages on the same topic,
 * checks their timestamps, and concatenates their fields together into a single
 * PointCloud output message.
 * \author Radu Bogdan Rusu
 */
class PointCloudConcatenateDataSynchronizerComponent : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;

  /** \brief constructor. */
  explicit PointCloudConcatenateDataSynchronizerComponent(const rclcpp::NodeOptions & node_options);

  /** \brief constructor.
   * \param queue_size the maximum queue size
   */
  PointCloudConcatenateDataSynchronizerComponent(
    const rclcpp::NodeOptions & node_options, int queue_size);

  /** \brief Empty destructor. */
  virtual ~PointCloudConcatenateDataSynchronizerComponent() {}

private:
  /** \brief The output PointCloud publisher. */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;
  /** \brief Delay Compensated PointCloud publisher*/
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
    transformed_raw_pc_publisher_map_;

  /** \brief The maximum number of messages that we can store in the queue. */
  int maximum_queue_size_ = 3;

  double timeout_sec_ = 0.1;

  bool publish_synchronized_pointcloud_;

  std::set<std::string> not_subscribed_topic_names_;

  /** \brief A vector of subscriber. */
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> filters_;

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_{this};

  const std::string input_twist_topic_type_;

  /** \brief Output TF frame the concatenated points should be transformed to. */
  std::string output_frame_;

  /** \brief Input point cloud topics. */
  // XmlRpc::XmlRpcValue input_topics_;
  std::vector<std::string> input_topics_;

  /** \brief TF listener object. */
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_ptr_queue_;

  std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_stdmap_;
  std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_stdmap_tmp_;
  std::mutex mutex_;

  std::vector<double> input_offset_;
  std::map<std::string, double> offset_map_;

  void transformPointCloud(const PointCloud2::ConstSharedPtr & in, PointCloud2::SharedPtr & out);
  Eigen::Matrix4f computeTransformToAdjustForOldTimestamp(
    const rclcpp::Time & old_stamp, const rclcpp::Time & new_stamp);
  std::map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> combineClouds(
    sensor_msgs::msg::PointCloud2::SharedPtr & concat_cloud_ptr);
  void publish();

  void convertToXYZICloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
    sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr);
  void setPeriod(const int64_t new_period);
  void cloud_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr,
    const std::string & topic_name);
  void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr input);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr input);
  void timer_callback();

  void checkConcatStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /** \brief processing time publisher. **/
  std::unique_ptr<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<tier4_autoware_utils::DebugPublisher> debug_publisher_;
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATE_AND_TIME_SYNC_NODELET_HPP_
