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

#include "scene_intersection.hpp"
#include "scene_merge_from_private_road.hpp"

#include <behavior_velocity_planner_common/plugin_interface.hpp>
#include <behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_api_msgs/msg/intersection_status.hpp>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace behavior_velocity_planner
{
class IntersectionModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit IntersectionModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "intersection"; }

private:
  IntersectionModule::PlannerParam intersection_param_;
  // additional for INTERSECTION_OCCLUSION
  RTCInterface occlusion_rtc_interface_;
  std::unordered_map<int64_t, UUID> occlusion_map_uuid_;

  void launchNewModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;

  bool hasSameParentLaneletAndTurnDirectionWithRegistered(const lanelet::ConstLanelet & lane) const;

  /* called from SceneModuleInterfaceWithRTC::plan */
  void sendRTC(const Time & stamp) override;
  void setActivation() override;
  /* called from SceneModuleInterface::updateSceneModuleInstances */
  void deleteExpiredModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;
};

class MergeFromPrivateModuleManager : public SceneModuleManagerInterface
{
public:
  explicit MergeFromPrivateModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "merge_from_private"; }

private:
  MergeFromPrivateRoadModule::PlannerParam merge_from_private_area_param_;

  void launchNewModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;

  bool hasSameParentLaneletAndTurnDirectionWithRegistered(const lanelet::ConstLanelet & lane) const;
};

class IntersectionModulePlugin : public PluginWrapper<IntersectionModuleManager>
{
};

class MergeFromPrivateModulePlugin : public PluginWrapper<MergeFromPrivateModuleManager>
{
};

}  // namespace behavior_velocity_planner

#endif  // MANAGER_HPP_
