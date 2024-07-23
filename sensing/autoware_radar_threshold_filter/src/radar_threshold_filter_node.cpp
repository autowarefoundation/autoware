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

#include "radar_threshold_filter_node.hpp"

#include <radar_msgs/msg/radar_scan.hpp>

#include <memory>
#include <string>
#include <vector>

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}

bool isWithin(double value, double max, double min)
{
  if (min < value && value < max) {
    return true;
  } else {
    return false;
  }
}
}  // namespace

namespace autoware::radar_threshold_filter
{
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;
using std::placeholders::_1;

RadarThresholdFilterNode::RadarThresholdFilterNode(const rclcpp::NodeOptions & node_options)
: Node("radar_threshold_filter", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarThresholdFilterNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.is_amplitude_filter = declare_parameter<bool>("node_params.is_amplitude_filter");
  node_param_.amplitude_min = declare_parameter<double>("node_params.amplitude_min");
  node_param_.amplitude_max = declare_parameter<double>("node_params.amplitude_max");
  node_param_.is_range_filter = declare_parameter<bool>("node_params.is_range_filter");
  node_param_.range_min = declare_parameter<double>("node_params.range_min");
  node_param_.range_max = declare_parameter<double>("node_params.range_max");
  node_param_.is_azimuth_filter = declare_parameter<bool>("node_params.is_azimuth_filter");
  node_param_.azimuth_min = declare_parameter<double>("node_params.azimuth_min");
  node_param_.azimuth_max = declare_parameter<double>("node_params.azimuth_max");
  node_param_.is_z_filter = declare_parameter<bool>("node_params.is_z_filter");
  node_param_.z_min = declare_parameter<double>("node_params.z_min");
  node_param_.z_max = declare_parameter<double>("node_params.z_max");

  // Subscriber
  sub_radar_ = create_subscription<RadarScan>(
    "~/input/radar", rclcpp::QoS{1}, std::bind(&RadarThresholdFilterNode::onData, this, _1));

  // Publisher
  pub_radar_ = create_publisher<RadarScan>("~/output/radar", 1);
}

rcl_interfaces::msg::SetParametersResult RadarThresholdFilterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  try {
    {
      auto & p = node_param_;
      update_param(params, "node_params.is_amplitude_filter", p.is_amplitude_filter);
      update_param(params, "node_params.amplitude_min", p.amplitude_min);
      update_param(params, "node_params.amplitude_max", p.amplitude_max);
      update_param(params, "node_params.is_range_filter", p.is_range_filter);
      update_param(params, "node_params.range_min", p.range_min);
      update_param(params, "node_params.range_max", p.range_max);
      update_param(params, "node_params.is_azimuth_filter", p.is_azimuth_filter);
      update_param(params, "node_params.azimuth_min", p.azimuth_min);
      update_param(params, "node_params.azimuth_max", p.azimuth_max);
      update_param(params, "node_params.is_z_filter", p.is_z_filter);
      update_param(params, "node_params.z_min", p.z_min);
      update_param(params, "node_params.z_max", p.z_max);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  result.successful = true;
  result.reason = "success";
  return result;
}

void RadarThresholdFilterNode::onData(const RadarScan::ConstSharedPtr radar_msg)
{
  RadarScan output;
  output.header = radar_msg->header;
  for (const auto & radar_return : radar_msg->returns) {
    if (isWithinThreshold(radar_return)) {
      output.returns.push_back(radar_return);
    }
  }
  pub_radar_->publish(output);
}

bool RadarThresholdFilterNode::isWithinThreshold(const RadarReturn & radar_return)
{
  if (
    node_param_.is_amplitude_filter &&
    !isWithin(radar_return.amplitude, node_param_.amplitude_max, node_param_.amplitude_min)) {
    return false;
  }

  if (
    node_param_.is_range_filter &&
    !isWithin(radar_return.range, node_param_.range_max, node_param_.range_min)) {
    return false;
  }

  if (
    node_param_.is_azimuth_filter &&
    !isWithin(radar_return.azimuth, node_param_.azimuth_max, node_param_.azimuth_min)) {
    return false;
  }

  if (node_param_.is_z_filter) {
    const auto z = radar_return.range * std::sin(radar_return.elevation);
    if (!isWithin(z, node_param_.z_max, node_param_.z_min)) {
      return false;
    }
  }
  return true;
}

}  // namespace autoware::radar_threshold_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::radar_threshold_filter::RadarThresholdFilterNode)
