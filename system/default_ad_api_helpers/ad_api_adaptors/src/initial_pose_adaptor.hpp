// Copyright 2022 TIER IV, Inc.
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

#ifndef INITIAL_POSE_ADAPTOR_HPP_
#define INITIAL_POSE_ADAPTOR_HPP_

#include <autoware_ad_api_specs/localization.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <map_height_fitter/map_height_fitter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace ad_api_adaptors
{

class InitialPoseAdaptor : public rclcpp::Node
{
public:
  InitialPoseAdaptor();

private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Initialize = autoware_ad_api::localization::Initialize;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  component_interface_utils::Client<Initialize>::SharedPtr cli_initialize_;
  std::array<double, 36> rviz_particle_covariance_;
  map_height_fitter::MapHeightFitter fitter_;

  void on_initial_pose(const PoseWithCovarianceStamped::ConstSharedPtr msg);
};

}  // namespace ad_api_adaptors

#endif  // INITIAL_POSE_ADAPTOR_HPP_
