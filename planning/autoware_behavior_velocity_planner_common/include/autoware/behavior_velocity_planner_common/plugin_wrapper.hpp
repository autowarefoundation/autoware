// Copyright 2023 The Autoware Contributors
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__PLUGIN_WRAPPER_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__PLUGIN_WRAPPER_HPP_

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>

#include <memory>
#include <optional>

namespace autoware::behavior_velocity_planner
{

template <class T>
class PluginWrapper : public PluginInterface
{
public:
  void init(rclcpp::Node & node) override { scene_manager_ = std::make_unique<T>(node); }
  void plan(tier4_planning_msgs::msg::PathWithLaneId * path) override
  {
    scene_manager_->plan(path);
  };
  void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & planner_data,
    const tier4_planning_msgs::msg::PathWithLaneId & path) override
  {
    scene_manager_->updateSceneModuleInstances(planner_data, path);
  }
  std::optional<int> getFirstStopPathPointIndex() override
  {
    return scene_manager_->getFirstStopPathPointIndex();
  }
  const char * getModuleName() override { return scene_manager_->getModuleName(); }

private:
  std::unique_ptr<T> scene_manager_;
};

}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__PLUGIN_WRAPPER_HPP_
