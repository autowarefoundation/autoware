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

#ifndef POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_
#define POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <string>

namespace pointcloud_preprocessor
{
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using rcl_interfaces::msg::SetParametersResult;
using sensor_msgs::msg::PointCloud2;

class DistortionCorrectorComponent : public rclcpp::Node
{
public:
  explicit DistortionCorrectorComponent(const rclcpp::NodeOptions & options);

private:
  void onPointCloud(PointCloud2::UniquePtr points_msg);
  void onVelocityReport(const VelocityReport::ConstSharedPtr velocity_report_msg);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    tf2::Transform * tf2_transform_ptr);

  bool undistortPointCloud(
    const std::deque<VelocityReport> & velocity_report_queue,
    const tf2::Transform & tf2_base_link_to_sensor, PointCloud2 & points);

  rclcpp::Subscription<PointCloud2>::SharedPtr input_points_sub_;
  rclcpp::Subscription<VelocityReport>::SharedPtr velocity_report_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr undistorted_points_pub_;

  tf2_ros::Buffer tf2_buffer_{get_clock()};
  tf2_ros::TransformListener tf2_listener_{tf2_buffer_};

  std::deque<autoware_auto_vehicle_msgs::msg::VelocityReport> velocity_report_queue_;

  std::string base_link_frame_ = "base_link";
  std::string time_stamp_field_name_;
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_
