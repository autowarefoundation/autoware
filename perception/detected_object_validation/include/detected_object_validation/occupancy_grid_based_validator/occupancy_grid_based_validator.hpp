// Copyright 2022 Tier IV, Inc.
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

#ifndef DETECTED_OBJECT_VALIDATION__OCCUPANCY_GRID_BASED_VALIDATOR__OCCUPANCY_GRID_BASED_VALIDATOR_HPP_
#define DETECTED_OBJECT_VALIDATION__OCCUPANCY_GRID_BASED_VALIDATOR__OCCUPANCY_GRID_BASED_VALIDATOR_HPP_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace occupancy_grid_based_validator
{
class OccupancyGridBasedValidator : public rclcpp::Node
{
public:
  explicit OccupancyGridBasedValidator(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::DetectedObjects> objects_sub_;
  message_filters::Subscriber<nav_msgs::msg::OccupancyGrid> occ_grid_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  typedef message_filters::sync_policies::ApproximateTime<
    autoware_auto_perception_msgs::msg::DetectedObjects, nav_msgs::msg::OccupancyGrid>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
  float mean_threshold_;
  bool enable_debug_;

  void onObjectsAndOccGrid(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects,
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & input_occ_grid);

  cv::Mat fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid & occupancy_grid);
  std::optional<cv::Mat> getMask(
    const nav_msgs::msg::OccupancyGrid & occupancy_grid,
    const autoware_auto_perception_msgs::msg::DetectedObject & object);
  std::optional<cv::Mat> getMask(
    const nav_msgs::msg::OccupancyGrid & occupancy_grid,
    const autoware_auto_perception_msgs::msg::DetectedObject & object, cv::Mat mask);
  void showDebugImage(
    const nav_msgs::msg::OccupancyGrid & ros_occ_grid,
    const autoware_auto_perception_msgs::msg::DetectedObjects & objects, const cv::Mat & occ_grid);
};
}  // namespace occupancy_grid_based_validator

#endif  // DETECTED_OBJECT_VALIDATION__OCCUPANCY_GRID_BASED_VALIDATOR__OCCUPANCY_GRID_BASED_VALIDATOR_HPP_
