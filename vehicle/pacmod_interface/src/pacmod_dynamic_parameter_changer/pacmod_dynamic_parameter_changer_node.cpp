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

#include <pacmod_dynamic_parameter_changer/pacmod_dynamic_parameter_changer_node.hpp>

#include <algorithm>

PacmodDynamicParameterChangerNode::PacmodDynamicParameterChangerNode()
: Node("pacmod_dynamic_parameter_changer")
{
  straight_course_param_.p_gain = declare_parameter("straight_p_gain", 0.8);
  straight_course_param_.i_gain = declare_parameter("straight_i_gain", 0.002);
  straight_course_param_.lugre_fc = declare_parameter("straight_lugre_fc", 0.03);
  straight_course_param_.rtz_offset = declare_parameter("straight_rtz_offset", 0.085);
  curve_course_param_.p_gain = declare_parameter("curve_p_gain", 0.3);
  curve_course_param_.i_gain = declare_parameter("curve_i_gain", 0.002);
  curve_course_param_.lugre_fc = declare_parameter("curve_lugre_fc", 0.01);
  curve_course_param_.rtz_offset = declare_parameter("curve_rtz_offset", 0.085);
  min_steer_thresh_ = declare_parameter("min_steer_thresh", 0.2);
  max_steer_thresh_ = declare_parameter("max_steer_thresh", 0.5);
  param_max_rate_.p_gain = declare_parameter("p_gain_rate_max", 5.0);
  param_max_rate_.i_gain = declare_parameter("i_gain_rate_max", 0.05);
  param_max_rate_.lugre_fc = declare_parameter("lugre_fc_rate_max", 0.1);
  param_max_rate_.rtz_offset = declare_parameter("rtz_offset_rate_max", 0.5);
  param_min_rate_.p_gain = declare_parameter("p_gain_rate_min", -1.5);
  param_min_rate_.i_gain = declare_parameter("i_gain_rate_min", -0.05);
  param_min_rate_.lugre_fc = declare_parameter("lugre_fc_rate_min", -0.1);
  param_min_rate_.rtz_offset = declare_parameter("rtz_offset_rate_min", -0.5);

  current_param_list_ = curve_course_param_;

  can_pub_ = create_publisher<can_msgs::msg::Frame>("~/output/can", rclcpp::QoS{1});
  debug_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>("~/debug/parameter", rclcpp::QoS{1});

  using std::placeholders::_1;
  steer_rpt_sub_ = create_subscription<pacmod3_msgs::msg::SystemRptFloat>(
    "~/input/steer_rpt", rclcpp::QoS{1},
    std::bind(&PacmodDynamicParameterChangerNode::subSteerRpt, this, _1));
}

void PacmodDynamicParameterChangerNode::subSteerRpt(
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr msg)
{
  current_param_list_ = calculateParam(std::abs(msg->command), std::abs(msg->output), msg->enabled);
  current_param_time_ = now();
  sendCanMsg(current_param_list_);
  sendDebugMsg(current_param_list_);
}

ParamList PacmodDynamicParameterChangerNode::calculateParam(
  const double current_steer_cmd, const double current_steer, const bool enabled)
{
  if (!enabled) {
    // return curve (default) param
    return curve_course_param_;
  }

  const double steer = std::max(current_steer, current_steer_cmd);
  // run straight course (steer is low) -> return straight-course-param
  // run curve course (steer is high) -> return curve-course-param

  if (steer >= max_steer_thresh_) {
    return curve_course_param_;
  } else if (steer <= min_steer_thresh_) {
    return straight_course_param_;
  }

  const double straight_rate =
    (max_steer_thresh_ - steer) / (max_steer_thresh_ - min_steer_thresh_);

  const auto interpolate_param =
    interpolateParam(straight_course_param_, curve_course_param_, straight_rate);
  return rateLimit(interpolate_param, current_param_list_);
}

void PacmodDynamicParameterChangerNode::sendDebugMsg(const ParamList param_list)
{
  // publish debug msg
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(4);
  msg.data.at(0) = param_list.p_gain;
  msg.data.at(1) = param_list.i_gain;
  msg.data.at(2) = param_list.lugre_fc;
  msg.data.at(3) = param_list.rtz_offset;
  debug_pub_->publish(msg);
}

void PacmodDynamicParameterChangerNode::sendCanMsg(const ParamList param_list)
{
  // encoding
  can_msgs::msg::Frame frame;
  frame.header.stamp = now();
  frame.id = 0x13C;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = 8;
  uint16_t kp_u = static_cast<uint16_t>(1000.0 * param_list.p_gain);
  uint16_t ki_u = static_cast<uint16_t>(1000.0 * param_list.i_gain);
  uint16_t fc_u = static_cast<uint16_t>(1000.0 * param_list.lugre_fc);
  uint16_t rtz_u = static_cast<uint16_t>(1000.0 * param_list.rtz_offset);
  frame.data[0] = (kp_u & 0xFF00) >> 8;
  frame.data[1] = ki_u & 0x00FF;
  frame.data[2] = (ki_u & 0xFF00) >> 8;
  frame.data[3] = ki_u & 0x00FF;
  frame.data[4] = (fc_u & 0xFF00) >> 8;
  frame.data[5] = fc_u & 0x00FF;
  frame.data[6] = (rtz_u & 0xFF00) >> 8;
  frame.data[7] = rtz_u & 0x00FF;

  // publish can msg
  can_pub_->publish(frame);
}

ParamList PacmodDynamicParameterChangerNode::interpolateParam(
  const ParamList p1, const ParamList p2, const double p1_rate)
{
  const double p2_rate = 1.0 - p1_rate;
  ParamList p;
  p.p_gain = p1.p_gain * p1_rate + p2.p_gain * p2_rate;
  p.i_gain = p1.i_gain * p1_rate + p2.i_gain * p2_rate;
  p.lugre_fc = p1.lugre_fc * p1_rate + p2.lugre_fc * p2_rate;
  p.rtz_offset = p1.rtz_offset * p1_rate + p2.rtz_offset * p2_rate;
  return p;
}

ParamList PacmodDynamicParameterChangerNode::rateLimit(
  const ParamList new_param, const ParamList current_param)
{
  ParamList limited_param;
  const double dt = (now() - current_param_time_).seconds();

  // apply rate limit
  limited_param.p_gain = rateLimit(
    new_param.p_gain, current_param.p_gain, dt, param_max_rate_.p_gain, param_min_rate_.p_gain);
  limited_param.i_gain = rateLimit(
    new_param.i_gain, current_param.i_gain, dt, param_max_rate_.i_gain, param_min_rate_.i_gain);
  limited_param.lugre_fc = rateLimit(
    new_param.lugre_fc, current_param.lugre_fc, dt, param_max_rate_.lugre_fc,
    param_min_rate_.lugre_fc);
  limited_param.rtz_offset = rateLimit(
    new_param.rtz_offset, current_param.rtz_offset, dt, param_max_rate_.rtz_offset,
    param_min_rate_.rtz_offset);
  return limited_param;
}

double PacmodDynamicParameterChangerNode::rateLimit(
  const double new_value, const double current_value, const double delta_time,
  const double max_rate, const double min_rate)
{
  const double dval = new_value - current_value;
  const double limit_dval = std::min(delta_time * max_rate, std::max(dval, delta_time * min_rate));
  return current_value + limit_dval;
}
