// Copyright 2022 The Autoware Contributors
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

#ifndef POSE_INITIALIZER__GNSS_MODULE_HPP_
#define POSE_INITIALIZER__GNSS_MODULE_HPP_

#include <map_height_fitter/map_height_fitter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class GnssModule
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  explicit GnssModule(rclcpp::Node * node);
  PoseWithCovarianceStamped get_pose();

private:
  map_height_fitter::MapHeightFitter fitter_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_;
  PoseWithCovarianceStamped::ConstSharedPtr pose_;
  double timeout_;
};

#endif  // POSE_INITIALIZER__GNSS_MODULE_HPP_
