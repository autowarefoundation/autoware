
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

using motion_utils::createDeadLineVirtualWallMarker;
using motion_utils::createSlowDownVirtualWallMarker;
using motion_utils::createStopVirtualWallMarker;
using tier4_autoware_utils::toHexString;
using unique_identifier_msgs::msg::UUID;
using SceneModulePtr = std::shared_ptr<SceneModuleInterface>;

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(
    rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
    const std::vector<std::string> & rtc_types)
  : node_(node),
    clock_(*node->get_clock()),
    logger_(node->get_logger().get_child(name)),
    name_(name),
    max_module_num_(config.max_module_size),
    priority_(config.priority),
    enable_simultaneous_execution_as_approved_module_(
      config.enable_simultaneous_execution_as_approved_module),
    enable_simultaneous_execution_as_candidate_module_(
      config.enable_simultaneous_execution_as_candidate_module)
  {
    for (const auto & rtc_type : rtc_types) {
      const auto snake_case_name = utils::convertToSnakeCase(name);
      const auto rtc_interface_name =
        rtc_type == "" ? snake_case_name : snake_case_name + "_" + rtc_type;
      rtc_interface_ptr_map_.emplace(
        rtc_type, std::make_shared<RTCInterface>(node, rtc_interface_name));
    }

    pub_debug_marker_ = node->create_publisher<MarkerArray>("~/debug/" + name, 20);
    pub_virtual_wall_ = node->create_publisher<MarkerArray>("~/virtual_wall/" + name, 20);
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

  void deleteModules(SceneModulePtr & module_ptr)
  {
    module_ptr->onExit();
    module_ptr->publishRTCStatus();

    const auto itr = std::find(registered_modules_.begin(), registered_modules_.end(), module_ptr);

    if (itr != registered_modules_.end()) {
      registered_modules_.erase(itr);
    }

    module_ptr.reset();

    pub_debug_marker_->publish(MarkerArray{});
  }

  void publishVirtualWall() const
  {
    using tier4_autoware_utils::appendMarkerArray;

    MarkerArray markers{};

    const auto marker_offset = std::numeric_limits<uint8_t>::max();

    uint32_t marker_id = marker_offset;
    for (const auto & m : registered_modules_) {
      const auto opt_stop_pose = m->getStopPose();
      if (!!opt_stop_pose) {
        const auto virtual_wall = createStopVirtualWallMarker(
          opt_stop_pose.get(), m->name(), rclcpp::Clock().now(), marker_id);
        appendMarkerArray(virtual_wall, &markers);
      }

      const auto opt_slow_pose = m->getSlowPose();
      if (!!opt_slow_pose) {
        const auto virtual_wall = createSlowDownVirtualWallMarker(
          opt_slow_pose.get(), m->name(), rclcpp::Clock().now(), marker_id);
        appendMarkerArray(virtual_wall, &markers);
      }

      const auto opt_dead_pose = m->getDeadPose();
      if (!!opt_dead_pose) {
        const auto virtual_wall = createDeadLineVirtualWallMarker(
          opt_dead_pose.get(), m->name(), rclcpp::Clock().now(), marker_id);
        appendMarkerArray(virtual_wall, &markers);
      }

      m->resetWallPoses();
    }

    pub_virtual_wall_->publish(markers);
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

  bool isSimultaneousExecutableAsApprovedModule() const
  {
    return enable_simultaneous_execution_as_approved_module_;
  }

  bool isSimultaneousExecutableAsCandidateModule() const
  {
    return enable_simultaneous_execution_as_candidate_module_;
  }

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

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_virtual_wall_;

  std::string name_;

  std::shared_ptr<PlannerData> planner_data_;

  std::vector<SceneModulePtr> registered_modules_;

  SceneModulePtr idling_module_;

  std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interface_ptr_map_;

private:
  size_t max_module_num_;

  size_t priority_;

  bool enable_simultaneous_execution_as_approved_module_{false};

  bool enable_simultaneous_execution_as_candidate_module_{false};
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
