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

#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector_node.hpp"

#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

namespace autoware::pointcloud_preprocessor
{
/** @brief Constructor. */
DistortionCorrectorComponent::DistortionCorrectorComponent(const rclcpp::NodeOptions & options)
: Node("distortion_corrector_node", options)
{
  // initialize debug tool

  using autoware::universe_utils::DebugPublisher;
  using autoware::universe_utils::StopWatch;
  stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<DebugPublisher>(this, "distortion_corrector");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  // Parameter
  base_frame_ = declare_parameter<std::string>("base_frame");
  use_imu_ = declare_parameter<bool>("use_imu");
  use_3d_distortion_correction_ = declare_parameter<bool>("use_3d_distortion_correction");
  auto has_static_tf_only =
    declare_parameter<bool>("has_static_tf_only", false);  // TODO(amadeuszsz): remove default value

  // Publisher
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    // Publisher
    undistorted_pointcloud_pub_ = this->create_publisher<PointCloud2>(
      "~/output/pointcloud", rclcpp::SensorDataQoS(), pub_options);
  }

  // Subscriber
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(&DistortionCorrectorComponent::onTwist, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", 10,
    std::bind(&DistortionCorrectorComponent::onImu, this, std::placeholders::_1));
  pointcloud_sub_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DistortionCorrectorComponent::onPointCloud, this, std::placeholders::_1));

  // Setup the distortion corrector

  if (use_3d_distortion_correction_) {
    distortion_corrector_ = std::make_unique<DistortionCorrector3D>(this, has_static_tf_only);
  } else {
    distortion_corrector_ = std::make_unique<DistortionCorrector2D>(this, has_static_tf_only);
  }
}

void DistortionCorrectorComponent::onTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg)
{
  distortion_corrector_->processTwistMessage(twist_msg);
}

void DistortionCorrectorComponent::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  if (!use_imu_) {
    return;
  }

  distortion_corrector_->processIMUMessage(base_frame_, imu_msg);
}

void DistortionCorrectorComponent::onPointCloud(PointCloud2::UniquePtr pointcloud_msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  const auto points_sub_count = undistorted_pointcloud_pub_->get_subscription_count() +
                                undistorted_pointcloud_pub_->get_intra_process_subscription_count();

  if (points_sub_count < 1) {
    return;
  }

  distortion_corrector_->setPointCloudTransform(base_frame_, pointcloud_msg->header.frame_id);

  distortion_corrector_->initialize();
  distortion_corrector_->undistortPointCloud(use_imu_, *pointcloud_msg);

  if (debug_publisher_) {
    auto pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - pointcloud_msg->header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  undistorted_pointcloud_pub_->publish(std::move(pointcloud_msg));

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

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::DistortionCorrectorComponent)
