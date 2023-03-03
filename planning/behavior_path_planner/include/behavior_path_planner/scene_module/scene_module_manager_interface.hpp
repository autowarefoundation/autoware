
// Copyright 2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using tier4_autoware_utils::toHexString;
using unique_identifier_msgs::msg::UUID;
using SceneModulePtr = std::shared_ptr<SceneModuleInterface>;

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(
    rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config)
  : node_(node),
    clock_(*node->get_clock()),
    logger_(node->get_logger().get_child(name)),
    name_(name),
    max_module_num_(config.max_module_size),
    priority_(config.priority),
    enable_simultaneous_execution_(config.enable_simultaneous_execution)
  {
    pub_debug_marker_ = node->create_publisher<MarkerArray>("~/debug/" + name, 20);
  }

  virtual ~SceneModuleManagerInterface() = default;

  SceneModulePtr getNewModule()
  {
    if (idling_module_ != nullptr) {
      return idling_module_;
    }

    idling_module_ = createNewSceneModuleInstance();
    return idling_module_;
  }

  bool isExecutionRequested(
    const SceneModulePtr & module_ptr, const BehaviorModuleOutput & previous_module_output) const
  {
    module_ptr->setData(planner_data_);
    module_ptr->setPreviousModuleOutput(previous_module_output);
    module_ptr->updateData();

    return module_ptr->isExecutionRequested();
  }

  void registerNewModule(
    const SceneModulePtr & module_ptr, const BehaviorModuleOutput & previous_module_output)
  {
    module_ptr->setData(planner_data_);
    module_ptr->setPreviousModuleOutput(previous_module_output);
    module_ptr->onEntry();

    registered_modules_.push_back(module_ptr);
  }

  void deleteModules(const SceneModulePtr & module_ptr)
  {
    module_ptr->onExit();
    module_ptr->publishRTCStatus();

    const auto itr = std::find(registered_modules_.begin(), registered_modules_.end(), module_ptr);

    registered_modules_.erase(itr);

    pub_debug_marker_->publish(MarkerArray{});
  }

  void publishDebugMarker() const
  {
    using tier4_autoware_utils::appendMarkerArray;

    MarkerArray markers{};

    const auto marker_offset = std::numeric_limits<uint8_t>::max();

    uint32_t marker_id = marker_offset;
    for (const auto & m : registered_modules_) {
      for (auto & marker : m->getDebugMarkers().markers) {
        marker.id += marker_id;
        markers.markers.push_back(marker);
      }
      marker_id += marker_offset;
    }

    if (registered_modules_.empty() && idling_module_ != nullptr) {
      appendMarkerArray(idling_module_->getDebugMarkers(), &markers);
    }

    pub_debug_marker_->publish(markers);
  }

  bool exist(const SceneModulePtr & module_ptr) const
  {
    return std::find(registered_modules_.begin(), registered_modules_.end(), module_ptr) !=
           registered_modules_.end();
  }

  bool canLaunchNewModule() const { return registered_modules_.size() < max_module_num_; }

  bool isSimultaneousExecutable() const { return enable_simultaneous_execution_; }

  void setData(const std::shared_ptr<PlannerData> & planner_data) { planner_data_ = planner_data; }

  void reset()
  {
    std::for_each(registered_modules_.begin(), registered_modules_.end(), [](const auto & m) {
      m->onExit();
      m->publishRTCStatus();
    });
    registered_modules_.clear();

    idling_module_->onExit();
    idling_module_->publishRTCStatus();
    idling_module_.reset();

    pub_debug_marker_->publish(MarkerArray{});
  }

  size_t getPriority() const { return priority_; }

  std::string getModuleName() const { return name_; }

  std::vector<SceneModulePtr> getSceneModules() { return registered_modules_; }

  virtual void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) = 0;

protected:
  virtual std::shared_ptr<SceneModuleInterface> createNewSceneModuleInstance() = 0;

  rclcpp::Node * node_;

  rclcpp::Clock clock_;

  rclcpp::Logger logger_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;

  std::string name_;

  std::shared_ptr<PlannerData> planner_data_;

  std::vector<SceneModulePtr> registered_modules_;

  SceneModulePtr idling_module_;

private:
  size_t max_module_num_;

  size_t priority_;

  bool enable_simultaneous_execution_{false};
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
