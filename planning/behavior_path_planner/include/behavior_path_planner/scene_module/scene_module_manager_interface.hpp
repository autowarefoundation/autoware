
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
using SceneModuleObserver = std::weak_ptr<SceneModuleInterface>;

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
    enable_simultaneous_execution_as_approved_module_(
      config.enable_simultaneous_execution_as_approved_module),
    enable_simultaneous_execution_as_candidate_module_(
      config.enable_simultaneous_execution_as_candidate_module),
    enable_rtc_(config.enable_rtc),
    keep_last_(config.keep_last),
    max_module_num_(config.max_module_size),
    priority_(config.priority)
  {
    for (const auto & rtc_type : rtc_types) {
      const auto snake_case_name = utils::convertToSnakeCase(name);
      const auto rtc_interface_name =
        rtc_type == "" ? snake_case_name : snake_case_name + "_" + rtc_type;
      rtc_interface_ptr_map_.emplace(
        rtc_type, std::make_shared<RTCInterface>(node, rtc_interface_name, enable_rtc_));
    }

    pub_info_marker_ = node->create_publisher<MarkerArray>("~/info/" + name, 20);
    pub_debug_marker_ = node->create_publisher<MarkerArray>("~/debug/" + name, 20);
    pub_virtual_wall_ = node->create_publisher<MarkerArray>("~/virtual_wall/" + name, 20);
    pub_drivable_lanes_ = node->create_publisher<MarkerArray>("~/drivable_lanes/" + name, 20);
  }

  virtual ~SceneModuleManagerInterface() = default;

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
          opt_stop_pose.get(), m.lock()->name(), rclcpp::Clock().now(), marker_id);
        appendMarkerArray(virtual_wall, &markers);
      }

      const auto opt_slow_pose = m.lock()->getSlowPose();
      if (!!opt_slow_pose) {
        const auto virtual_wall = createSlowDownVirtualWallMarker(
          opt_slow_pose.get(), m.lock()->name(), rclcpp::Clock().now(), marker_id);
        appendMarkerArray(virtual_wall, &markers);
      }

      const auto opt_dead_pose = m.lock()->getDeadPose();
      if (!!opt_dead_pose) {
        const auto virtual_wall = createDeadLineVirtualWallMarker(
          opt_dead_pose.get(), m.lock()->name(), rclcpp::Clock().now(), marker_id);
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

  bool canLaunchNewModule() const { return observers_.size() < max_module_num_; }

  virtual bool isSimultaneousExecutableAsApprovedModule() const
  {
    return enable_simultaneous_execution_as_approved_module_;
  }

  virtual bool isSimultaneousExecutableAsCandidateModule() const
  {
    return enable_simultaneous_execution_as_candidate_module_;
  }

  bool isKeepLast() const { return keep_last_; }

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

  size_t getPriority() const { return priority_; }

  std::string name() const { return name_; }

  std::vector<SceneModuleObserver> getSceneModuleObservers() { return observers_; }

  std::shared_ptr<SceneModuleInterface> getIdleModule() { return std::move(idle_module_ptr_); }

  virtual void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) = 0;

protected:
  virtual std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() = 0;

  rclcpp::Node * node_;

  rclcpp::Clock clock_;

  rclcpp::Logger logger_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_info_marker_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_virtual_wall_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_drivable_lanes_;

  std::string name_;

  std::shared_ptr<PlannerData> planner_data_;

  std::vector<SceneModuleObserver> observers_;

  std::unique_ptr<SceneModuleInterface> idle_module_ptr_;

  std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interface_ptr_map_;

  bool enable_simultaneous_execution_as_approved_module_{false};

  bool enable_simultaneous_execution_as_candidate_module_{false};

private:
  bool enable_rtc_;

  bool keep_last_;

  size_t max_module_num_;

  size_t priority_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
