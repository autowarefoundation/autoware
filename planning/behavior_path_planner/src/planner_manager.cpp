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

#include "behavior_path_planner/planner_manager.hpp"

#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"
#include "tier4_autoware_utils/ros/debug_publisher.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <magic_enum.hpp>

#include <boost/format.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{
PlannerManager::PlannerManager(
  rclcpp::Node & node, const size_t max_iteration_num, const bool verbose)
: plugin_loader_("behavior_path_planner", "behavior_path_planner::SceneModuleManagerInterface"),
  logger_(node.get_logger().get_child("planner_manager")),
  clock_(*node.get_clock()),
  max_iteration_num_{max_iteration_num},
  verbose_{verbose}
{
  processing_time_.emplace("total_time", 0.0);
  debug_publisher_ptr_ = std::make_unique<DebugPublisher>(&node, "~/debug");
}

void PlannerManager::launchScenePlugin(rclcpp::Node & node, const std::string & name)
{
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->init(&node);

    // Check if the plugin is already registered.
    for (const auto & running_plugin : manager_ptrs_) {
      if (plugin->name() == running_plugin->name()) {
        RCLCPP_WARN_STREAM(node.get_logger(), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    // register
    manager_ptrs_.push_back(plugin);
    processing_time_.emplace(plugin->name(), 0.0);
    RCLCPP_INFO_STREAM(node.get_logger(), "The scene plugin '" << name << "' is loaded.");
  } else {
    RCLCPP_ERROR_STREAM(node.get_logger(), "The scene plugin '" << name << "' is not available.");
  }
}

void PlannerManager::removeScenePlugin(rclcpp::Node & node, const std::string & name)
{
  auto it = std::remove_if(manager_ptrs_.begin(), manager_ptrs_.end(), [&](const auto plugin) {
    return plugin->name() == name;
  });

  if (it == manager_ptrs_.end()) {
    RCLCPP_WARN_STREAM(
      node.get_logger(),
      "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    manager_ptrs_.erase(it, manager_ptrs_.end());
    processing_time_.erase(name);
    RCLCPP_INFO_STREAM(node.get_logger(), "The scene plugin '" << name << "' is unloaded.");
  }
}

BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
  resetProcessingTime();
  stop_watch_.tic("total_time");
  debug_info_.clear();

  if (!root_lanelet_) {
    root_lanelet_ = updateRootLanelet(data);
  }

  std::for_each(
    manager_ptrs_.begin(), manager_ptrs_.end(), [&data](const auto & m) { m->setData(data); });

  auto result_output = [&]() {
    const bool is_any_approved_module_running =
      std::any_of(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [](const auto & m) {
        return m->getCurrentStatus() == ModuleStatus::RUNNING ||
               m->getCurrentStatus() == ModuleStatus::WAITING_APPROVAL;
      });

    // IDLE is a state in which an execution has been requested but not yet approved.
    // once approved, it basically turns to running.
    const bool is_any_candidate_module_running_or_idle =
      std::any_of(candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), [](const auto & m) {
        return m->getCurrentStatus() == ModuleStatus::RUNNING ||
               m->getCurrentStatus() == ModuleStatus::WAITING_APPROVAL ||
               m->getCurrentStatus() == ModuleStatus::IDLE;
      });

    const bool is_any_module_running =
      is_any_approved_module_running || is_any_candidate_module_running_or_idle;

    const bool is_out_of_route = utils::isEgoOutOfRoute(
      data->self_odometry->pose.pose, data->prev_modified_goal, data->route_handler);

    if (!is_any_module_running && is_out_of_route) {
      BehaviorModuleOutput output = utils::createGoalAroundPath(data);
      generateCombinedDrivableArea(output, data);
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 5000,
        "Ego is out of route, no module is running. Skip running scene modules.");
      return output;
    }

    for (size_t itr_num = 1;; ++itr_num) {
      /**
       * STEP1: get approved modules' output
       */
      auto approved_modules_output = runApprovedModules(data);

      /**
       * STEP2: check modules that need to be launched
       */
      const auto request_modules = getRequestModules(approved_modules_output);

      /**
       * STEP3: if there is no module that need to be launched, return approved modules' output
       */
      if (request_modules.empty()) {
        const auto output = runKeepLastModules(data, approved_modules_output);
        processing_time_.at("total_time") = stop_watch_.toc("total_time", true);
        return output;
      }

      /**
       * STEP4: if there is module that should be launched, execute the module
       */
      const auto [highest_priority_module, candidate_modules_output] =
        runRequestModules(request_modules, data, approved_modules_output);

      /**
       * STEP5: run keep last approved modules after running candidate modules.
       * NOTE: if no candidate module is launched, approved_modules_output used as input for keep
       * last modules and return the result immediately.
       */
      const auto output = runKeepLastModules(
        data, highest_priority_module ? candidate_modules_output : approved_modules_output);
      if (!highest_priority_module) {
        processing_time_.at("total_time") = stop_watch_.toc("total_time", true);
        return output;
      }

      /**
       * STEP6: if the candidate module's modification is NOT approved yet, return the result.
       * NOTE: the result is output of the candidate module, but the output path don't contains path
       * shape modification that needs approval. On the other hand, it could include velocity
       * profile modification.
       */
      if (highest_priority_module->isWaitingApproval()) {
        processing_time_.at("total_time") = stop_watch_.toc("total_time", true);
        return output;
      }

      /**
       * STEP7: if the candidate module is approved, push the module into approved_module_ptrs_
       */
      addApprovedModule(highest_priority_module);
      clearCandidateModules();
      debug_info_.emplace_back(highest_priority_module, Action::ADD, "To Approval");

      if (itr_num >= max_iteration_num_) {
        RCLCPP_WARN_THROTTLE(
          logger_, clock_, 1000, "Reach iteration limit (max: %ld). Output current result.",
          max_iteration_num_);
        processing_time_.at("total_time") = stop_watch_.toc("total_time", true);
        return output;
      }
    }

    return BehaviorModuleOutput{};  // something wrong.
  }();

  std::for_each(
    manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });

  generateCombinedDrivableArea(result_output, data);

  return result_output;
}

// NOTE: To deal with some policies about drivable area generation, currently DrivableAreaInfo is
// quite messy. Needs to be refactored.
void PlannerManager::generateCombinedDrivableArea(
  BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & data) const
{
  if (output.path.points.empty()) {
    RCLCPP_ERROR_STREAM(logger_, "[generateCombinedDrivableArea] Output path is empty!");
    return;
  }

  const auto & di = output.drivable_area_info;
  constexpr double epsilon = 1e-3;

  const auto is_driving_forward_opt = motion_utils::isDrivingForward(output.path.points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  if (epsilon < std::abs(di.drivable_margin)) {
    // for single free space pull over
    utils::generateDrivableArea(
      output.path, data->parameters.vehicle_length, di.drivable_margin, is_driving_forward);
  } else if (di.is_already_expanded) {
    // for single side shift
    utils::generateDrivableArea(
      output.path, di.drivable_lanes, false, false, data->parameters.vehicle_length, data,
      is_driving_forward);
  } else {
    const auto shorten_lanes = utils::cutOverlappedLanes(output.path, di.drivable_lanes);

    const auto & dp = data->drivable_area_expansion_parameters;
    const auto expanded_lanes = utils::expandLanelets(
      shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
      dp.drivable_area_types_to_skip);

    // for other modules where multiple modules may be launched
    utils::generateDrivableArea(
      output.path, expanded_lanes, di.enable_expanding_hatched_road_markings,
      di.enable_expanding_intersection_areas, data->parameters.vehicle_length, data,
      is_driving_forward);
  }

  // extract obstacles from drivable area
  utils::extractObstaclesFromDrivableArea(output.path, di.obstacles);
}

std::vector<SceneModulePtr> PlannerManager::getRequestModules(
  const BehaviorModuleOutput & previous_module_output) const
{
  if (previous_module_output.path.points.empty()) {
    RCLCPP_ERROR_STREAM(
      logger_, "Current module output is null. Skip candidate module check."
                 << "\n      - Approved  module list: " << getNames(approved_module_ptrs_)
                 << "\n      - Candidate module list: " << getNames(candidate_module_ptrs_));
    return {};
  }

  std::vector<SceneModulePtr> request_modules{};

  const auto toc = [this](const auto & name) {
    processing_time_.at(name) += stop_watch_.toc(name, true);
  };

  for (const auto & manager_ptr : manager_ptrs_) {
    stop_watch_.tic(manager_ptr->name());

    /**
     * determine the execution capability of modules based on existing approved modules.
     */
    // Condition 1: always executable module can be added regardless of the existence of other
    // modules, so skip checking the existence of other modules.
    // in other cases, need to check the existence of other modules and which module can be added.
    if (!manager_ptr->isAlwaysExecutableModule() && hasNonAlwaysExecutableApprovedModules()) {
      // pairs of find_block_module and is_executable
      std::vector<std::pair<std::function<bool(const SceneModulePtr &)>, std::function<bool()>>>
        conditions;

      // Condition 2: do not add modules that are neither always nor simultaneous executable
      // if there exists at least one approved module that is simultaneous but not always
      // executable. (only modules that are either always executable or simultaneous executable can
      // be added)
      conditions.emplace_back(
        [&](const SceneModulePtr & m) {
          return !getManager(m)->isAlwaysExecutableModule() &&
                 getManager(m)->isSimultaneousExecutableAsApprovedModule();
        },
        [&]() { return manager_ptr->isSimultaneousExecutableAsApprovedModule(); });

      // Condition 3: do not add modules that are not always executable if there exists
      // at least one approved module that is neither always nor simultaneous executable.
      // (only modules that are always executable can be added)
      conditions.emplace_back(
        [&](const SceneModulePtr & m) {
          return !getManager(m)->isAlwaysExecutableModule() &&
                 !getManager(m)->isSimultaneousExecutableAsApprovedModule();
        },
        [&]() { return false; });

      bool skip_module = false;
      for (const auto & condition : conditions) {
        const auto & find_block_module = condition.first;
        const auto & is_executable = condition.second;

        const auto itr = std::find_if(
          approved_module_ptrs_.begin(), approved_module_ptrs_.end(), find_block_module);

        if (itr != approved_module_ptrs_.end() && !is_executable()) {
          toc(manager_ptr->name());
          skip_module = true;
          continue;
        }
      }
      if (skip_module) {
        continue;
      }
    }
    // else{
    //   Condition 4: if none of the above conditions are met, any module can be added.
    //   (when the approved modules are either empty or consist only of always executable modules.)
    // }

    /**
     * launch new candidate module.
     */
    {
      const auto name = manager_ptr->name();
      const auto find_same_name_module = [&name](const auto & m) { return m->name() == name; };
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), find_same_name_module);

      if (itr == candidate_module_ptrs_.end()) {
        if (manager_ptr->canLaunchNewModule()) {
          manager_ptr->updateIdleModuleInstance();
          if (manager_ptr->isExecutionRequested(previous_module_output)) {
            request_modules.emplace_back(manager_ptr->getIdleModule());
          }
        }

        toc(manager_ptr->name());
        continue;
      }
    }

    /**
     * module already exist in candidate modules. check whether other modules can be launch as
     * candidate. if locked, break this loop.
     */
    {
      const auto name = manager_ptr->name();
      const auto find_block_module = [&name](const auto & m) {
        return m->name() == name && m->isLockedNewModuleLaunch();
      };
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), find_block_module);

      if (itr != candidate_module_ptrs_.end()) {
        request_modules.clear();
        request_modules.emplace_back(*itr);
        toc(manager_ptr->name());
        break;
      }
    }

    /**
     * module already exist. keep using it as candidate.
     */
    {
      const auto name = manager_ptr->name();
      const auto find_launched_module = [&name](const auto & m) { return m->name() == name; };
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), find_launched_module);

      if (itr != candidate_module_ptrs_.end()) {
        request_modules.emplace_back(*itr);
        toc(manager_ptr->name());
        continue;
      }
    }

    /**
     * same name module doesn't exist in candidate modules. launch new module.
     */
    {
      if (!manager_ptr->canLaunchNewModule()) {
        toc(manager_ptr->name());
        continue;
      }

      manager_ptr->updateIdleModuleInstance();
      if (!manager_ptr->isExecutionRequested(previous_module_output)) {
        toc(manager_ptr->name());
        continue;
      }

      request_modules.emplace_back(manager_ptr->getIdleModule());
    }

    toc(manager_ptr->name());
  }

  return request_modules;
}

BehaviorModuleOutput PlannerManager::runKeepLastModules(
  const std::shared_ptr<PlannerData> & data, const BehaviorModuleOutput & previous_output) const
{
  auto output = previous_output;
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
    if (getManager(m)->isKeepLast()) {
      output = run(m, data, output);
    }
  });

  return output;
}

BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  const auto & route_handler = data->route_handler;
  const auto & pose = data->self_odometry->pose.pose;
  const auto p = data->parameters;

  constexpr double extra_margin = 10.0;
  const auto backward_length =
    std::max(p.backward_path_length, p.backward_path_length + extra_margin);

  const auto lanelet_sequence = route_handler->getLaneletSequence(
    root_lanelet_.value(), pose, backward_length, std::numeric_limits<double>::max());

  lanelet::ConstLanelet closest_lane{};
  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        lanelet_sequence, pose, &closest_lane, p.ego_nearest_dist_threshold,
        p.ego_nearest_yaw_threshold)) {
    return utils::getReferencePath(closest_lane, data);
  }

  if (lanelet::utils::query::getClosestLanelet(lanelet_sequence, pose, &closest_lane)) {
    return utils::getReferencePath(closest_lane, data);
  }

  return {};  // something wrong.
}

SceneModulePtr PlannerManager::selectHighestPriorityModule(
  std::vector<SceneModulePtr> & request_modules) const
{
  if (request_modules.empty()) {
    return {};
  }

  sortByPriority(request_modules);

  return request_modules.front();
}

std::pair<SceneModulePtr, BehaviorModuleOutput> PlannerManager::runRequestModules(
  const std::vector<SceneModulePtr> & request_modules, const std::shared_ptr<PlannerData> & data,
  const BehaviorModuleOutput & previous_module_output)
{
  // modules that are filtered by simultaneous executable condition.
  std::vector<SceneModulePtr> executable_modules;

  // modules that are not approved after running yet.
  std::vector<SceneModulePtr> waiting_approved_modules;

  // modules that are already approved after running.
  std::vector<SceneModulePtr> already_approved_modules;

  // all request modules planning results.
  std::unordered_map<std::string, BehaviorModuleOutput> results;

  /**
   * sort by priority. sorted_request_modules.front() is the highest priority module.
   */
  auto sorted_request_modules = request_modules;
  sortByPriority(sorted_request_modules);

  /**
   * remove non-executable modules.
   */
  for (const auto & module_ptr : sorted_request_modules) {
    // Condition 1: always executable module can be added regardless of the existence of other
    // modules.
    if (getManager(module_ptr)->isAlwaysExecutableModule()) {
      executable_modules.push_back(module_ptr);
      continue;
    }

    // Condition 4: If the executable modules are either empty or consist only of always executable
    // modules, any module can be added.
    const bool has_non_always_executable_module = std::any_of(
      executable_modules.begin(), executable_modules.end(),
      [this](const auto & m) { return !getManager(m)->isAlwaysExecutableModule(); });
    if (!has_non_always_executable_module) {
      executable_modules.push_back(module_ptr);
      continue;
    }

    // pairs of find_block_module and is_executable
    std::vector<std::pair<std::function<bool(const SceneModulePtr &)>, std::function<bool()>>>
      conditions;

    // Condition 3: Only modules that are always executable can be added
    // if there exists at least one executable module that is neither always nor simultaneous
    // executable.
    conditions.emplace_back(
      [this](const SceneModulePtr & m) {
        return !getManager(m)->isAlwaysExecutableModule() &&
               !getManager(m)->isSimultaneousExecutableAsCandidateModule();
      },
      [&]() { return false; });

    // Condition 2: Only modules that are either always executable or simultaneous executable can be
    // added if there exists at least one executable module that is simultaneous but not always
    // executable.
    conditions.emplace_back(
      [this](const SceneModulePtr & m) {
        return !getManager(m)->isAlwaysExecutableModule() &&
               getManager(m)->isSimultaneousExecutableAsCandidateModule();
      },
      [&]() { return getManager(module_ptr)->isSimultaneousExecutableAsCandidateModule(); });

    for (const auto & condition : conditions) {
      const auto & find_block_module = condition.first;
      const auto & is_executable = condition.second;

      const auto itr =
        std::find_if(executable_modules.begin(), executable_modules.end(), find_block_module);

      if (itr != executable_modules.end() && is_executable()) {
        executable_modules.push_back(module_ptr);
        break;
      }
    }
  }

  /**
   * run executable modules.
   */
  for (const auto & module_ptr : executable_modules) {
    const auto & manager_ptr = getManager(module_ptr);

    if (!manager_ptr->exist(module_ptr)) {
      manager_ptr->registerNewModule(
        std::weak_ptr<SceneModuleInterface>(module_ptr), previous_module_output);
    }

    results.emplace(module_ptr->name(), run(module_ptr, data, previous_module_output));
  }

  /**
   * remove expired modules.
   */
  {
    const auto remove_expired_modules = [this](auto & m) {
      if (m->getCurrentStatus() == ModuleStatus::FAILURE) {
        deleteExpiredModules(m);
        return true;
      }

      if (m->getCurrentStatus() == ModuleStatus::SUCCESS) {
        deleteExpiredModules(m);
        return true;
      }

      return false;
    };

    executable_modules.erase(
      std::remove_if(executable_modules.begin(), executable_modules.end(), remove_expired_modules),
      executable_modules.end());

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  /**
   * return null data if valid candidate module doesn't exist.
   */
  if (executable_modules.empty()) {
    clearCandidateModules();
    return std::make_pair(nullptr, BehaviorModuleOutput{});
  }

  /**
   * separate by approve condition.
   */
  std::for_each(executable_modules.begin(), executable_modules.end(), [&](const auto & m) {
    if (m->isWaitingApproval()) {
      waiting_approved_modules.push_back(m);
    } else {
      already_approved_modules.push_back(m);
    }
  });

  /**
   * choice highest priority module.
   */
  const auto module_ptr = [&]() {
    if (!already_approved_modules.empty()) {
      return selectHighestPriorityModule(already_approved_modules);
    }

    if (!waiting_approved_modules.empty()) {
      return selectHighestPriorityModule(waiting_approved_modules);
    }

    return SceneModulePtr();
  }();

  /**
   * register candidate modules.
   */
  updateCandidateModules(executable_modules, module_ptr);

  return std::make_pair(module_ptr, results.at(module_ptr->name()));
}

BehaviorModuleOutput PlannerManager::runApprovedModules(const std::shared_ptr<PlannerData> & data)
{
  std::unordered_map<std::string, BehaviorModuleOutput> results;
  BehaviorModuleOutput output = getReferencePath(data);
  results.emplace("root", output);

  if (approved_module_ptrs_.empty()) {
    return output;
  }

  const auto move_to_end = [](auto & modules, const auto & cond) {
    auto itr = modules.begin();
    while (itr != modules.end()) {
      const auto satisfied_exit_cond =
        std::all_of(itr, modules.end(), [&cond](const auto & m) { return cond(m); });

      if (satisfied_exit_cond) {
        return;
      }

      if (cond(*itr)) {
        auto tmp = std::move(*itr);
        itr = modules.erase(itr);
        modules.insert(modules.end(), std::move(tmp));
      } else {
        itr++;
      }
    }
  };

  // move modules whose keep last flag is true to end of the approved_module_ptrs_.
  {
    const auto keep_last_module_cond = [this](const auto & m) {
      return getManager(m)->isKeepLast();
    };
    move_to_end(approved_module_ptrs_, keep_last_module_cond);
  }

  // lock approved modules besides last one
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
    m->lockOutputPath();
  });

  // unlock only last approved module except keep last module.
  {
    const auto not_keep_last_modules = std::find_if(
      approved_module_ptrs_.rbegin(), approved_module_ptrs_.rend(),
      [this](const auto & m) { return !getManager(m)->isKeepLast(); });

    if (not_keep_last_modules != approved_module_ptrs_.rend()) {
      (*not_keep_last_modules)->unlockOutputPath();
    }
  }

  /**
   * execute approved modules except keep last modules.
   */
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
    if (!getManager(m)->isKeepLast()) {
      output = run(m, data, output);
      results.emplace(m->name(), output);
    }
  });

  /**
   * pop waiting approve module. push it back candidate modules. if waiting approve module is found
   * in iteration std::find_if, not only the module but also the next one are removed from
   * approved_module_ptrs_ since the next module's input (= waiting approve module's output) maybe
   * change significantly. And, only the waiting approve module is pushed back to
   * candidate_module_ptrs_.
   */
  {
    const auto not_keep_last_module = std::find_if(
      approved_module_ptrs_.rbegin(), approved_module_ptrs_.rend(),
      [this](const auto & m) { return !getManager(m)->isKeepLast(); });

    // convert reverse iterator -> iterator
    const auto begin_itr = not_keep_last_module != approved_module_ptrs_.rend()
                             ? std::next(not_keep_last_module).base()
                             : approved_module_ptrs_.begin();

    const auto waiting_approval_modules_itr = std::find_if(
      begin_itr, approved_module_ptrs_.end(),
      [](const auto & m) { return m->isWaitingApproval(); });

    if (waiting_approval_modules_itr != approved_module_ptrs_.end()) {
      clearCandidateModules();
      candidate_module_ptrs_.push_back(*waiting_approval_modules_itr);

      debug_info_.emplace_back(
        *waiting_approval_modules_itr, Action::MOVE, "Back To Waiting Approval");

      std::for_each(
        waiting_approval_modules_itr, approved_module_ptrs_.end(),
        [&results](const auto & m) { results.erase(m->name()); });

      approved_module_ptrs_.erase(waiting_approval_modules_itr);

      std::for_each(
        manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
    }
  }

  /**
   * remove failure modules. these modules' outputs are discarded as invalid plan.
   */
  {
    const auto itr = std::find_if(
      approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
      [](const auto & m) { return m->getCurrentStatus() == ModuleStatus::FAILURE; });

    std::for_each(itr, approved_module_ptrs_.end(), [this](auto & m) {
      debug_info_.emplace_back(m, Action::DELETE, "From Approved");
      deleteExpiredModules(m);
    });

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });

    if (itr != approved_module_ptrs_.end()) {
      clearCandidateModules();
    }

    approved_module_ptrs_.erase(itr, approved_module_ptrs_.end());
  }

  if (approved_module_ptrs_.empty()) {
    return results.at("root");
  }

  /**
   * use the last module's output as approved modules planning result.
   */
  const auto approved_modules_output = [&results, this]() {
    const auto itr = std::find_if(
      approved_module_ptrs_.rbegin(), approved_module_ptrs_.rend(),
      [&results](const auto & m) { return results.count(m->name()) != 0; });

    if (itr != approved_module_ptrs_.rend()) {
      return results.at((*itr)->name());
    }
    return results.at("root");
  }();

  /**
   * remove success module immediately. if lane change module has succeeded, update root lanelet.
   */
  {
    const auto success_module_cond = [](const auto & m) {
      return m->getCurrentStatus() == ModuleStatus::SUCCESS;
    };
    move_to_end(approved_module_ptrs_, success_module_cond);

    const auto itr =
      std::find_if(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), success_module_cond);

    if (std::any_of(itr, approved_module_ptrs_.end(), [](const auto & m) {
          return m->isRootLaneletToBeUpdated();
        })) {
      root_lanelet_ = updateRootLanelet(data);
    }

    std::for_each(itr, approved_module_ptrs_.end(), [&](auto & m) {
      debug_info_.emplace_back(m, Action::DELETE, "From Approved");
      deleteExpiredModules(m);
    });

    approved_module_ptrs_.erase(itr, approved_module_ptrs_.end());

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  return approved_modules_output;
}

void PlannerManager::updateCandidateModules(
  const std::vector<SceneModulePtr> & request_modules,
  const SceneModulePtr & highest_priority_module)
{
  const auto exist = [](const auto & module_ptr, const auto & module_ptrs) {
    const auto itr = std::find_if(
      module_ptrs.begin(), module_ptrs.end(),
      [&module_ptr](const auto & m) { return m->name() == module_ptr->name(); });

    return itr != module_ptrs.end();
  };

  /**
   * unregister expired modules
   */
  {
    const auto candidate_to_remove = [&](auto & itr) {
      if (!exist(itr, request_modules)) {
        deleteExpiredModules(itr);
        return true;
      }
      return itr->name() == highest_priority_module->name() &&
             !highest_priority_module->isWaitingApproval();
    };

    candidate_module_ptrs_.erase(
      std::remove_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), candidate_to_remove),
      candidate_module_ptrs_.end());

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  /**
   * register running candidate modules
   */
  for (const auto & m : request_modules) {
    if (
      m->name() == highest_priority_module->name() &&
      !highest_priority_module->isWaitingApproval()) {
      continue;
    }

    if (!exist(m, candidate_module_ptrs_)) {
      candidate_module_ptrs_.push_back(m);
    }
  }

  /**
   * sort by priority. sorted_request_modules.front() is the highest priority module.
   */
  sortByPriority(candidate_module_ptrs_);
}

void PlannerManager::resetRootLanelet(const std::shared_ptr<PlannerData> & data)
{
  if (!root_lanelet_) {
    root_lanelet_ = updateRootLanelet(data);
    return;
  }

  // when lane change module is running, don't update root lanelet.
  const bool is_lane_change_running = std::invoke([&]() {
    const auto lane_change_itr =
      std::find_if(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [](const auto & m) {
        return m->name().find("lane_change") != std::string::npos ||
               m->name().find("avoidance_by_lc") != std::string::npos;
      });
    return lane_change_itr != approved_module_ptrs_.end();
  });
  if (is_lane_change_running) {
    return;
  }

  const auto root_lanelet = updateRootLanelet(data);

  // if root_lanelet is not route lanelets, reset root lanelet.
  // this can be caused by rerouting.
  const auto & route_handler = data->route_handler;
  if (!route_handler->isRouteLanelet(root_lanelet_.value())) {
    root_lanelet_ = root_lanelet;
    return;
  }

  // check ego is in same lane
  if (root_lanelet_.value().id() == root_lanelet.id()) {
    return;
  }

  // check ego is in next lane
  const auto next_lanelets = route_handler->getRoutingGraphPtr()->following(root_lanelet_.value());
  for (const auto & next : next_lanelets) {
    if (next.id() == root_lanelet.id()) {
      return;
    }
  }

  if (!approved_module_ptrs_.empty()) {
    return;
  }

  for (const auto & m : candidate_module_ptrs_) {
    if (m->isLockedNewModuleLaunch()) {
      return;
    }
  }

  root_lanelet_ = root_lanelet;

  RCLCPP_INFO_EXPRESSION(logger_, verbose_, "change ego's following lane. reset.");
}

void PlannerManager::print() const
{
  if (!verbose_) {
    return;
  }

  const auto get_status = [](const auto & m) {
    return magic_enum::enum_name(m->getCurrentStatus());
  };

  size_t max_string_num = 0;

  std::ostringstream string_stream;
  string_stream << "\n";
  string_stream << "***********************************************************\n";
  string_stream << "                  planner manager status\n";
  string_stream << "-----------------------------------------------------------\n";
  string_stream << "registered modules: ";
  for (const auto & m : manager_ptrs_) {
    string_stream << "[" << m->name() << "]";
    max_string_num = std::max(max_string_num, m->name().length());
  }

  string_stream << "\n";
  string_stream << "approved modules  : ";
  for (const auto & m : approved_module_ptrs_) {
    string_stream << "[" << m->name() << "(" << get_status(m) << ")"
                  << "]->";
  }

  string_stream << "\n";
  string_stream << "candidate module  : ";
  for (const auto & m : candidate_module_ptrs_) {
    string_stream << "[" << m->name() << "(" << get_status(m) << ")"
                  << "]->";
  }

  string_stream << "\n";
  string_stream << "update module info: ";
  for (const auto & i : debug_info_) {
    string_stream << "[Module:" << i.module_name << " Status:" << magic_enum::enum_name(i.status)
                  << " Action:" << magic_enum::enum_name(i.action)
                  << " Description:" << i.description << "]\n"
                  << std::setw(28);
  }

  string_stream << "\n" << std::fixed << std::setprecision(1);
  string_stream << "processing time   : ";
  for (const auto & t : processing_time_) {
    string_stream << std::right << "[" << std::setw(static_cast<int>(max_string_num) + 1)
                  << std::left << t.first << ":" << std::setw(4) << std::right << t.second
                  << "ms]\n"
                  << std::setw(21);
  }

  RCLCPP_INFO_STREAM(logger_, string_stream.str());
}

void PlannerManager::publishProcessingTime() const
{
  for (const auto & t : processing_time_) {
    std::string name = t.first + std::string("/processing_time_ms");
    debug_publisher_ptr_->publish<DebugDoubleMsg>(name, t.second);
  }
}

std::shared_ptr<SceneModuleVisitor> PlannerManager::getDebugMsg()
{
  debug_msg_ptr_ = std::make_shared<SceneModuleVisitor>();
  for (const auto & approved_module : approved_module_ptrs_) {
    approved_module->acceptVisitor(debug_msg_ptr_);
  }

  for (const auto & candidate_module : candidate_module_ptrs_) {
    candidate_module->acceptVisitor(debug_msg_ptr_);
  }
  return debug_msg_ptr_;
}

std::string PlannerManager::getNames(const std::vector<SceneModulePtr> & modules)
{
  std::stringstream ss;
  for (const auto & m : modules) {
    ss << "[" << m->name() << "], ";
  }
  return ss.str();
}

}  // namespace behavior_path_planner
