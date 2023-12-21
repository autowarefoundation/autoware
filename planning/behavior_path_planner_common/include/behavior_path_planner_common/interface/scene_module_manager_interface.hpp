
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

#ifndef BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
#define BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_MANAGER_INTERFACE_HPP_

#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "tier4_autoware_utils/ros/parameter.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/publisher.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <cstddef>
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
using SceneModuleObserver = std::weak_ptr<SceneModuleInterface>;

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(const SceneModuleManagerInterface &) = delete;
  SceneModuleManagerInterface(SceneModuleManagerInterface &&) = delete;
  SceneModuleManagerInterface & operator=(const SceneModuleManagerInterface &) = delete;
  SceneModuleManagerInterface & operator=(SceneModuleManagerInterface &&) = delete;
  explicit SceneModuleManagerInterface(std::string name) : name_{std::move(name)} {}

  virtual ~SceneModuleManagerInterface() = default;

  virtual void init(rclcpp::Node * node) = 0;

  void updateIdleModuleInstance()
  {
    if (idle_module_ptr_) {
      idle_module_ptr_->onEntry();
      return;
    }

    idle_module_ptr_ = createNewSceneModuleInstance();
  }

  bool isExecutionRequested(const BehaviorModuleOutput & previous_module_output) const
  {
    idle_module_ptr_->setData(planner_data_);
    idle_module_ptr_->setPreviousModuleOutput(previous_module_output);
    idle_module_ptr_->updateData();

    return idle_module_ptr_->isExecutionRequested();
  }

  void registerNewModule(
    const SceneModuleObserver & observer, const BehaviorModuleOutput & previous_module_output)
  {
    if (observer.expired()) {
      return;
    }

    observer.lock()->setData(planner_data_);
    observer.lock()->setPreviousModuleOutput(previous_module_output);
    observer.lock()->onEntry();

    observers_.push_back(observer);
  }

  void updateObserver()
  {
    const auto itr = std::remove_if(
      observers_.begin(), observers_.end(),
      [](const auto & observer) { return observer.expired(); });

    observers_.erase(itr, observers_.end());
  }

  void publishVirtualWall() const
  {
    using tier4_autoware_utils::appendMarkerArray;

    MarkerArray markers{};

    const auto marker_offset = std::numeric_limits<uint8_t>::max();

    uint32_t marker_id = marker_offset;
    for (const auto & m : observers_) {
      if (m.expired()) {
        continue;
      }

      const auto opt_stop_pose = m.lock()->getStopPose();
      if (!!opt_stop_pose) {
        const auto virtual_wall = createStopVirtualWallMarker(
          opt_stop_pose.value(), m.lock()->name(), rclcpp::Clock().now(), marker_id);
        appendMarkerArray(virtual_wall, &markers);
      }

      const auto opt_slow_pose = m.lock()->getSlowPose();
      if (!!opt_slow_pose) {
        const auto virtual_wall = createSlowDownVirtualWallMarker(
          opt_slow_pose.value(), m.lock()->name(), rclcpp::Clock().now(), marker_id);
        appendMarkerArray(virtual_wall, &markers);
      }

      const auto opt_dead_pose = m.lock()->getDeadPose();
      if (!!opt_dead_pose) {
        const auto virtual_wall = createDeadLineVirtualWallMarker(
          opt_dead_pose.value(), m.lock()->name(), rclcpp::Clock().now(), marker_id);
        appendMarkerArray(virtual_wall, &markers);
      }

      const auto module_specific_wall = m.lock()->getModuleVirtualWall();
      appendMarkerArray(module_specific_wall, &markers);

      m.lock()->resetWallPoses();
    }

    pub_virtual_wall_->publish(markers);
  }

  void publishMarker() const
  {
    using tier4_autoware_utils::appendMarkerArray;

    MarkerArray info_markers{};
    MarkerArray debug_markers{};
    MarkerArray drivable_lanes_markers{};

    const auto marker_offset = std::numeric_limits<uint8_t>::max();

    uint32_t marker_id = marker_offset;
    for (const auto & m : observers_) {
      if (m.expired()) {
        continue;
      }

      for (auto & marker : m.lock()->getInfoMarkers().markers) {
        marker.id += marker_id;
        info_markers.markers.push_back(marker);
      }

      for (auto & marker : m.lock()->getDebugMarkers().markers) {
        marker.id += marker_id;
        debug_markers.markers.push_back(marker);
      }

      for (auto & marker : m.lock()->getDrivableLanesMarkers().markers) {
        marker.id += marker_id;
        drivable_lanes_markers.markers.push_back(marker);
      }

      marker_id += marker_offset;
    }

    if (observers_.empty() && idle_module_ptr_ != nullptr) {
      appendMarkerArray(idle_module_ptr_->getInfoMarkers(), &info_markers);
      appendMarkerArray(idle_module_ptr_->getDebugMarkers(), &debug_markers);
      appendMarkerArray(idle_module_ptr_->getDrivableLanesMarkers(), &drivable_lanes_markers);
    }

    pub_info_marker_->publish(info_markers);
    pub_debug_marker_->publish(debug_markers);
    pub_drivable_lanes_->publish(drivable_lanes_markers);
  }

  bool exist(const SceneModulePtr & module_ptr) const
  {
    return std::any_of(observers_.begin(), observers_.end(), [&](const auto & observer) {
      return !observer.expired() && observer.lock() == module_ptr;
    });
  }

  bool canLaunchNewModule() const { return observers_.size() < config_.max_module_size; }

  /**
   * Determine if the module is always executable, regardless of the state of other modules.
   *
   * When this returns true:
   * - The module can be executed even if other modules are not marked as 'simultaneously
   * executable'.
   * - Conversely, the presence of this module will not prevent other modules
   *   from executing, even if they are not marked as 'simultaneously executable'.
   */
  virtual bool isAlwaysExecutableModule() const { return false; }

  virtual bool isSimultaneousExecutableAsApprovedModule() const
  {
    if (isAlwaysExecutableModule()) {
      return true;
    }

    return config_.enable_simultaneous_execution_as_approved_module;
  }

  virtual bool isSimultaneousExecutableAsCandidateModule() const
  {
    if (isAlwaysExecutableModule()) {
      return true;
    }

    return config_.enable_simultaneous_execution_as_candidate_module;
  }

  bool isKeepLast() const { return config_.keep_last; }

  void setData(const std::shared_ptr<PlannerData> & planner_data) { planner_data_ = planner_data; }

  void reset()
  {
    std::for_each(observers_.begin(), observers_.end(), [](const auto & observer) {
      if (!observer.expired()) {
        observer.lock()->onExit();
        observer.lock()->publishRTCStatus();
      }
    });

    observers_.clear();

    if (idle_module_ptr_ != nullptr) {
      idle_module_ptr_->onExit();
      idle_module_ptr_->publishRTCStatus();
      idle_module_ptr_.reset();
    }

    pub_debug_marker_->publish(MarkerArray{});
  }

  size_t getPriority() const { return config_.priority; }

  std::string name() const { return name_; }

  std::vector<SceneModuleObserver> getSceneModuleObservers() { return observers_; }

  std::shared_ptr<SceneModuleInterface> getIdleModule() { return std::move(idle_module_ptr_); }

  virtual void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) = 0;

protected:
  void initInterface(rclcpp::Node * node, const std::vector<std::string> & rtc_types)
  {
    using tier4_autoware_utils::getOrDeclareParameter;

    // init manager configuration
    {
      std::string ns = name_ + ".";
      config_.enable_rtc = getOrDeclareParameter<bool>(*node, ns + "enable_rtc");
      config_.enable_simultaneous_execution_as_approved_module =
        getOrDeclareParameter<bool>(*node, ns + "enable_simultaneous_execution_as_approved_module");
      config_.enable_simultaneous_execution_as_candidate_module = getOrDeclareParameter<bool>(
        *node, ns + "enable_simultaneous_execution_as_candidate_module");
      config_.keep_last = getOrDeclareParameter<bool>(*node, ns + "keep_last");
      config_.priority = getOrDeclareParameter<int>(*node, ns + "priority");
      config_.max_module_size = getOrDeclareParameter<int>(*node, ns + "max_module_size");
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
    }

    // misc
    {
      node_ = node;
    }
  }

  virtual std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() = 0;

  rclcpp::Node * node_ = nullptr;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_info_marker_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_virtual_wall_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_drivable_lanes_;

  std::string name_;

  std::shared_ptr<PlannerData> planner_data_;

  std::vector<SceneModuleObserver> observers_;

  std::unique_ptr<SceneModuleInterface> idle_module_ptr_;

  std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interface_ptr_map_;

  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    objects_of_interest_marker_interface_ptr_map_;

  ModuleConfigParameters config_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
