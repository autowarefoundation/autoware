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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "scene.hpp"

#include <behavior_velocity_planner_common/plugin_interface.hpp>
#include <behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <functional>
#include <memory>
#include <optional>

namespace behavior_velocity_planner
{
class TrafficLightModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit TrafficLightModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "traffic_light"; }

private:
  TrafficLightModule::PlannerParam planner_param_;

  void launchNewModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;

  void modifyPathVelocity(autoware_auto_planning_msgs::msg::PathWithLaneId * path) override;

  bool isModuleRegisteredFromRegElement(const lanelet::Id & id, const size_t module_id) const;

  bool isModuleRegisteredFromExistingAssociatedModule(const lanelet::Id & id) const;

  bool hasSameTrafficLight(
    const lanelet::TrafficLightConstPtr element,
    const lanelet::TrafficLightConstPtr registered_element) const;

  // Debug
  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficSignal>::SharedPtr pub_tl_state_;

  std::optional<int> first_ref_stop_path_point_index_;
};

class TrafficLightModulePlugin : public PluginWrapper<TrafficLightModuleManager>
{
};

}  // namespace behavior_velocity_planner

#endif  // MANAGER_HPP_
