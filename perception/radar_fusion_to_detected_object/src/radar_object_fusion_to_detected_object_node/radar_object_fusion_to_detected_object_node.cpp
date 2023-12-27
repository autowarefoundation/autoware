
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

#include "radar_fusion_to_detected_object/radar_fusion_to_detected_object_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::placeholders::_1;

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
}  // namespace

namespace radar_fusion_to_detected_object
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;

RadarObjectFusionToDetectedObjectNode::RadarObjectFusionToDetectedObjectNode(
  const rclcpp::NodeOptions & node_options)
: Node("radar_object_fusion_to_detected_object", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarObjectFusionToDetectedObjectNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("node_params.update_rate_hz");

  // Core Parameter
  core_param_.bounding_box_margin = declare_parameter<double>("core_params.bounding_box_margin");
  core_param_.split_threshold_velocity =
    declare_parameter<double>("core_params.split_threshold_velocity");
  core_param_.threshold_yaw_diff = declare_parameter<double>("core_params.threshold_yaw_diff");
  core_param_.velocity_weight_min_distance =
    declare_parameter<double>("core_params.velocity_weight_min_distance");
  core_param_.velocity_weight_average =
    declare_parameter<double>("core_params.velocity_weight_average");
  core_param_.velocity_weight_median =
    declare_parameter<double>("core_params.velocity_weight_median");
  core_param_.velocity_weight_target_value_average =
    declare_parameter<double>("core_params.velocity_weight_target_value_average");
  core_param_.velocity_weight_target_value_top =
    declare_parameter<double>("core_params.velocity_weight_target_value_top");
  core_param_.convert_doppler_to_twist =
    declare_parameter<bool>("core_params.convert_doppler_to_twist");
  core_param_.threshold_probability =
    declare_parameter<double>("core_params.threshold_probability");
  core_param_.compensate_probability =
    declare_parameter<bool>("core_params.compensate_probability");

  // Core
  radar_fusion_to_detected_object_ = std::make_unique<RadarFusionToDetectedObject>(get_logger());
  radar_fusion_to_detected_object_->setParam(core_param_);

  // Subscriber
  sub_object_.subscribe(this, "~/input/objects", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_radar_.subscribe(this, "~/input/radars", rclcpp::QoS{1}.get_rmw_qos_profile());

  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(20), sub_object_, sub_radar_);
  sync_ptr_->registerCallback(
    std::bind(&RadarObjectFusionToDetectedObjectNode::onData, this, _1, _2));

  // Publisher
  pub_objects_ = create_publisher<DetectedObjects>("~/output/objects", 1);
  pub_debug_low_confidence_objects_ =
    create_publisher<DetectedObjects>("~/debug/low_confidence_objects", 1);
}

rcl_interfaces::msg::SetParametersResult RadarObjectFusionToDetectedObjectNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "node_params.update_rate_hz", p.update_rate_hz);
    }

    // Core Parameter
    {
      // Copy to local variable
      auto & p = core_param_;

      // Update params
      update_param(params, "core_params.bounding_box_margin", p.bounding_box_margin);
      update_param(params, "core_params.split_threshold_velocity", p.split_threshold_velocity);
      update_param(params, "core_params.threshold_yaw_diff", p.threshold_yaw_diff);
      update_param(params, "core_params.velocity_weight_average", p.velocity_weight_average);
      update_param(params, "core_params.velocity_weight_median", p.velocity_weight_median);
      update_param(
        params, "core_params.velocity_weight_target_value_average",
        p.velocity_weight_target_value_average);
      update_param(
        params, "core_params.velocity_weight_target_value_top", p.velocity_weight_target_value_top);
      update_param(params, "core_params.convert_doppler_to_twist", p.convert_doppler_to_twist);
      update_param(params, "core_params.threshold_probability", p.threshold_probability);
      update_param(params, "core_params.compensate_probability", p.compensate_probability);

      // Set parameter to instance
      if (radar_fusion_to_detected_object_) {
        radar_fusion_to_detected_object_->setParam(core_param_);
      }
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

bool RadarObjectFusionToDetectedObjectNode::isDataReady()
{
  if (!detected_objects_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "waiting for detected objects data msg...");
    return false;
  }
  if (!radar_objects_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for radar objects data msg...");
    return false;
  }

  if (detected_objects_->header.frame_id != radar_objects_->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "The frame id between detected objects and radar objects is not same");
    return false;
  }

  return true;
}

void RadarObjectFusionToDetectedObjectNode::onData(
  const DetectedObjects::ConstSharedPtr object_msg, const DetectedObjects::ConstSharedPtr radar_msg)
{
  detected_objects_ = object_msg;
  radar_objects_ = radar_msg;
  if (!isDataReady()) {
    return;
  }

  if (radar_objects_->objects.empty()) {
    pub_objects_->publish(*detected_objects_);
    return;
  }

  // Set input data
  RadarFusionToDetectedObject::Input input{};
  std::vector<RadarFusionToDetectedObject::RadarInput> radars_{};
  for (const auto & radar_object_ : radar_objects_->objects) {
    auto radar_input = setRadarInput(radar_object_, radar_objects_->header);
    radars_.emplace_back(radar_input);
  }
  input.objects = detected_objects_;
  input.radars = std::make_shared<std::vector<RadarFusionToDetectedObject::RadarInput>>(radars_);

  // Update
  output_ = radar_fusion_to_detected_object_->update(input);
  pub_objects_->publish(output_.objects);
  pub_debug_low_confidence_objects_->publish(output_.debug_low_confidence_objects);
}

RadarFusionToDetectedObject::RadarInput RadarObjectFusionToDetectedObjectNode::setRadarInput(
  const DetectedObject & radar_object, const std_msgs::msg::Header & header_)
{
  RadarFusionToDetectedObject::RadarInput output{};
  output.pose_with_covariance = radar_object.kinematics.pose_with_covariance;
  output.twist_with_covariance = radar_object.kinematics.twist_with_covariance;
  output.target_value = radar_object.classification.at(0).probability;
  output.header = header_;
  return output;
}

}  // namespace radar_fusion_to_detected_object

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  radar_fusion_to_detected_object::RadarObjectFusionToDetectedObjectNode)
