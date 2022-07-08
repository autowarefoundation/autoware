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

#ifndef LOCALIZATION_SCORE_HPP_
#define LOCALIZATION_SCORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "autoware_ad_api_msgs/msg/localization_score_array.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#include <memory>
#include <utility>

namespace default_ad_api
{
using autoware_ad_api_msgs::msg::LocalizationScoreArray;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using tier4_debug_msgs::msg::Float32Stamped;

class LocalizationScoreNode : public rclcpp::Node
{
public:
  explicit LocalizationScoreNode(const rclcpp::NodeOptions & options);

private:
  void callbackTpScore(const Float32Stamped::ConstSharedPtr msg_ptr);
  void callbackNvtlScore(const Float32Stamped::ConstSharedPtr msg_ptr);
  void callbackTimer();

  rclcpp::Publisher<LocalizationScoreArray>::SharedPtr pub_localization_scores_;
  rclcpp::Subscription<Float32Stamped>::SharedPtr sub_transform_probability_;
  rclcpp::Subscription<Float32Stamped>::SharedPtr sub_nearest_voxel_transformation_likelihood_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::TimerBase::SharedPtr timer_;
  double status_pub_hz_;

  autoware_ad_api_msgs::msg::LocalizationScore score_tp_;
  autoware_ad_api_msgs::msg::LocalizationScore score_nvtl_;
  bool is_tp_received_;
  bool is_nvtl_received_;
};

}  // namespace default_ad_api

#endif  // LOCALIZATION_SCORE_HPP_
