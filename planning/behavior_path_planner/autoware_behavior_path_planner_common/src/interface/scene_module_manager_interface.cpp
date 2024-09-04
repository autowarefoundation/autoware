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

#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"

namespace autoware::behavior_path_planner
{
void SceneModuleManagerInterface::initInterface(
  rclcpp::Node * node, const std::vector<std::string> & rtc_types)
{
  using autoware::universe_utils::getOrDeclareParameter;

  // init manager configuration
  {
    std::string ns = name_ + ".";
    try {
      config_.enable_rtc = getOrDeclareParameter<bool>(*node, "enable_all_modules_auto_mode")
                             ? false
                             : getOrDeclareParameter<bool>(*node, ns + "enable_rtc");
    } catch (const std::exception & e) {
      config_.enable_rtc = getOrDeclareParameter<bool>(*node, ns + "enable_rtc");
    }

    config_.enable_simultaneous_execution_as_approved_module =
      getOrDeclareParameter<bool>(*node, ns + "enable_simultaneous_execution_as_approved_module");
    config_.enable_simultaneous_execution_as_candidate_module =
      getOrDeclareParameter<bool>(*node, ns + "enable_simultaneous_execution_as_candidate_module");
  }

  // init rtc configuration
  for (const auto & rtc_type : rtc_types) {
    const auto snake_case_name = utils::convertToSnakeCase(name_);
    const auto rtc_interface_name =
      rtc_type.empty() ? snake_case_name : snake_case_name + "_" + rtc_type;
    rtc_interface_ptr_map_.emplace(
      rtc_type, std::make_shared<RTCInterface>(node, rtc_interface_name, config_.enable_rtc));
    objects_of_interest_marker_interface_ptr_map_.emplace(
      rtc_type, std::make_shared<ObjectsOfInterestMarkerInterface>(node, rtc_interface_name));
  }

  // init publisher
  {
    pub_info_marker_ = node->create_publisher<MarkerArray>("~/info/" + name_, 20);
    pub_debug_marker_ = node->create_publisher<MarkerArray>("~/debug/" + name_, 20);
    pub_virtual_wall_ = node->create_publisher<MarkerArray>("~/virtual_wall/" + name_, 20);
    pub_drivable_lanes_ = node->create_publisher<MarkerArray>("~/drivable_lanes/" + name_, 20);
    pub_processing_time_ = node->create_publisher<universe_utils::ProcessingTimeDetail>(
      "~/processing_time/" + name_, 20);
  }

  // init steering factor
  {
    steering_factor_interface_ptr_ =
      std::make_shared<SteeringFactorInterface>(node, utils::convertToSnakeCase(name_));
  }

  // misc
  {
    node_ = node;
  }
}

void SceneModuleManagerInterface::updateIdleModuleInstance()
{
  if (idle_module_ptr_) {
    idle_module_ptr_->onEntry();
    return;
  }

  idle_module_ptr_ = createNewSceneModuleInstance();
}
}  // namespace autoware::behavior_path_planner
