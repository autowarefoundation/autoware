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

#ifndef PACMOD_DYNAMIC_PARAMETER_CHANGER__PACMOD_DYNAMIC_PARAMETER_CHANGER_NODE_HPP_
#define PACMOD_DYNAMIC_PARAMETER_CHANGER__PACMOD_DYNAMIC_PARAMETER_CHANGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

struct ParamList
{
  double p_gain;
  double i_gain;
  double lugre_fc;
  double rtz_offset;
};

class PacmodDynamicParameterChangerNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<pacmod3_msgs::msg::SystemRptFloat>::SharedPtr steer_rpt_sub_;

  ParamList straight_course_param_;
  ParamList curve_course_param_;
  ParamList param_max_rate_;
  ParamList param_min_rate_;
  double min_steer_thresh_;
  double max_steer_thresh_;

  ParamList current_param_list_;
  rclcpp::Time current_param_time_;

public:
  PacmodDynamicParameterChangerNode();
  void subSteerRpt(const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr msg);
  ParamList calculateParam(
    const double current_steer_cmd, const double current_steer, const bool enabled);
  void sendCanMsg(const ParamList param_list);
  void sendDebugMsg(const ParamList param_list);

  ParamList interpolateParam(const ParamList p1, const ParamList p2, const double p1_rate);
  ParamList rateLimit(const ParamList new_param, const ParamList current_param);
  double rateLimit(
    const double new_value, const double current_value, const double delta_time,
    const double max_rate, const double min_rate);
};

#endif  // PACMOD_DYNAMIC_PARAMETER_CHANGER__PACMOD_DYNAMIC_PARAMETER_CHANGER_NODE_HPP_
