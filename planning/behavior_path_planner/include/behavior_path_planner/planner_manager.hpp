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

#ifndef BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_
#define BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_manager_interface.hpp"
#include "behavior_path_planner/util/lane_following/module_data.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using tier4_autoware_utils::StopWatch;
using SceneModulePtr = std::shared_ptr<SceneModuleInterface>;
using SceneModuleManagerPtr = std::shared_ptr<SceneModuleManagerInterface>;

struct SceneModuleStatus
{
  explicit SceneModuleStatus(const std::string & n) : module_name(n) {}

  std::string module_name;

  bool is_execution_ready{false};
  bool is_waiting_approval{false};

  ModuleStatus status{ModuleStatus::SUCCESS};
};

class PlannerManager
{
public:
  PlannerManager(
    rclcpp::Node & node, const std::shared_ptr<LaneFollowingParameters> & parameters,
    const bool verbose);

  BehaviorModuleOutput run(const std::shared_ptr<PlannerData> & data);

  void registerSceneModuleManager(const SceneModuleManagerPtr & manager_ptr)
  {
    RCLCPP_INFO(logger_, "register %s module", manager_ptr->getModuleName().c_str());
    manager_ptrs_.push_back(manager_ptr);
    processing_time_.emplace(manager_ptr->getModuleName(), 0.0);
  }

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
  {
    std::for_each(manager_ptrs_.begin(), manager_ptrs_.end(), [&parameters](const auto & m) {
      m->updateModuleParams(parameters);
    });
  }

  void reset()
  {
    approved_module_ptrs_.clear();
    candidate_module_opt_ = boost::none;
    root_lanelet_ = boost::none;
    std::for_each(manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->reset(); });
    resetProcessingTime();
  }

  void publishDebugMarker() const
  {
    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->publishDebugMarker(); });
  }

  std::vector<SceneModuleManagerPtr> getSceneModuleManagers() const { return manager_ptrs_; }

  std::vector<std::shared_ptr<SceneModuleStatus>> getSceneModuleStatus() const
  {
    std::vector<std::shared_ptr<SceneModuleStatus>> ret;

    const auto size = approved_module_ptrs_.size() + 1;

    ret.reserve(size);

    for (const auto & m : approved_module_ptrs_) {
      auto s = std::make_shared<SceneModuleStatus>(m->name());
      s->is_waiting_approval = m->isWaitingApproval();
      s->status = m->getCurrentStatus();
      ret.push_back(s);
    }

    if (!!candidate_module_opt_) {
      const auto m = candidate_module_opt_.get();
      auto s = std::make_shared<SceneModuleStatus>(m->name());
      s->is_waiting_approval = m->isWaitingApproval();
      s->status = m->getCurrentStatus();
      ret.push_back(s);
    }

    ret.shrink_to_fit();

    return ret;
  }

  void resetRootLanelet(const std::shared_ptr<PlannerData> & data);

  void print() const;

private:
  BehaviorModuleOutput run(
    const SceneModulePtr & module_ptr, const std::shared_ptr<PlannerData> & planner_data,
    const BehaviorModuleOutput & previous_module_output) const
  {
    module_ptr->setData(planner_data);
    module_ptr->setPreviousModuleOutput(previous_module_output);

    module_ptr->lockRTCCommand();
    const auto result = module_ptr->run();
    module_ptr->unlockRTCCommand();

    module_ptr->updateState();

    module_ptr->publishRTCStatus();

    return result;
  }

  void deleteExpiredModules(const SceneModulePtr & module_ptr) const
  {
    const auto itr = std::find_if(
      manager_ptrs_.begin(), manager_ptrs_.end(),
      [&module_ptr](const auto & m) { return m->getModuleName() == module_ptr->name(); });
    if (itr == manager_ptrs_.end()) {
      return;
    }

    (*itr)->deleteModules(module_ptr);
  }

  void addApprovedModule(const SceneModulePtr & module_ptr)
  {
    approved_module_ptrs_.push_back(module_ptr);
  }

  void resetProcessingTime()
  {
    for (auto & t : processing_time_) {
      t.second = 0.0;
    }
  }

  lanelet::ConstLanelet updateRootLanelet(const std::shared_ptr<PlannerData> & data) const
  {
    lanelet::ConstLanelet ret{};
    data->route_handler->getClosestLaneletWithinRoute(data->self_odometry->pose.pose, &ret);
    RCLCPP_DEBUG(logger_, "update start lanelet. id:%ld", ret.id());
    return ret;
  }

  BehaviorModuleOutput update(const std::shared_ptr<PlannerData> & data);

  BehaviorModuleOutput getReferencePath(const std::shared_ptr<PlannerData> & data) const;

  boost::optional<SceneModulePtr> getCandidateModule(
    const BehaviorModuleOutput & previous_module_output) const;

  boost::optional<std::pair<SceneModuleManagerPtr, SceneModulePtr>> selectHighestPriorityModule(
    std::vector<std::pair<SceneModuleManagerPtr, SceneModulePtr>> & request_modules) const;

  boost::optional<SceneModulePtr> candidate_module_opt_{boost::none};

  boost::optional<lanelet::ConstLanelet> root_lanelet_{boost::none};

  std::shared_ptr<LaneFollowingParameters> parameters_;

  std::vector<SceneModuleManagerPtr> manager_ptrs_;

  std::vector<SceneModulePtr> approved_module_ptrs_;

  rclcpp::Logger logger_;

  rclcpp::Clock clock_;

  mutable StopWatch<std::chrono::milliseconds> stop_watch_;

  mutable std::unordered_map<std::string, double> processing_time_;

  bool verbose_{false};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_
