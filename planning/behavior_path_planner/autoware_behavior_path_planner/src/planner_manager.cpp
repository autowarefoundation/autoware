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

#include "autoware/behavior_path_planner/planner_manager.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/universe_utils/ros/debug_publisher.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <magic_enum.hpp>

#include <boost/scope_exit.hpp>

#include <memory>
#include <string>

namespace autoware::behavior_path_planner
{
PlannerManager::PlannerManager(rclcpp::Node & node)
: plugin_loader_(
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface"),
  logger_(node.get_logger().get_child("planner_manager")),
  clock_(*node.get_clock())
{
  current_route_lanelet_ = std::make_shared<std::optional<lanelet::ConstLanelet>>(std::nullopt);
  processing_time_.emplace("total_time", 0.0);
  debug_publisher_ptr_ = std::make_unique<DebugPublisher>(&node, "~/debug");
  state_publisher_ptr_ = std::make_unique<DebugPublisher>(&node, "~/debug");
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
    RCLCPP_DEBUG_STREAM(node.get_logger(), "The scene plugin '" << name << "' is loaded.");
  } else {
    RCLCPP_ERROR_STREAM(node.get_logger(), "The scene plugin '" << name << "' is not available.");
  }
}

void PlannerManager::configureModuleSlot(
  const std::vector<std::vector<std::string>> & slot_configuration)
{
  std::unordered_map<std::string, SceneModuleManagerPtr> registered_modules;
  for (const auto & manager_ptr : manager_ptrs_) {
    registered_modules[manager_ptr->name()] = manager_ptr;
  }

  for (const auto & slot : slot_configuration) {
    SubPlannerManager sub_manager(current_route_lanelet_, processing_time_, debug_info_);
    for (const auto & module_name : slot) {
      if (const auto it = registered_modules.find(module_name); it != registered_modules.end()) {
        sub_manager.addSceneModuleManager(it->second);
      } else {
        // TODO(Mamoru Sobue): use LOG
        std::cout << module_name << " registered in slot_configuration is not registered, skipping"
                  << std::endl;
      }
    }
    if (sub_manager.getSceneModuleManager().size() != 0) {
      planner_manager_slots_.push_back(sub_manager);
      // TODO(Mamoru Sobue): use LOG
      std::cout << "added a slot with " << sub_manager.getSceneModuleManager().size() << " modules"
                << std::endl;
    }
  }
}

BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
  resetProcessingTime();
  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("total_time");
  BOOST_SCOPE_EXIT((&processing_time_)(&stop_watch))
  {
    processing_time_.at("total_time") += stop_watch.toc("total_time", true);
  }
  BOOST_SCOPE_EXIT_END;

  debug_info_.scene_status.clear();
  debug_info_.slot_status.clear();

  if (!current_route_lanelet_->has_value()) resetCurrentRouteLanelet(data);

  std::for_each(
    manager_ptrs_.begin(), manager_ptrs_.end(), [&data](const auto & m) { m->setData(data); });

  const bool is_any_approved_module_running = std::any_of(
    planner_manager_slots_.begin(), planner_manager_slots_.end(), [&](const auto & slot) {
      return slot.isAnyApprovedPred([](const auto & m) {
        const auto status = m->getCurrentStatus();
        return status == ModuleStatus::RUNNING || status == ModuleStatus::WAITING_APPROVAL;
      });
    });

  // IDLE is a state in which an execution has been requested but not yet approved.
  // once approved, it basically turns to running.
  const bool is_any_candidate_module_running_or_idle = std::any_of(
    planner_manager_slots_.begin(), planner_manager_slots_.end(), [](const auto & slot) {
      return slot.isAnyCandidatePred([](const auto & m) {
        const auto status = m->getCurrentStatus();
        return status == ModuleStatus::RUNNING || status == ModuleStatus::WAITING_APPROVAL ||
               status == ModuleStatus::IDLE;
      });
    });

  const bool is_any_module_running =
    is_any_approved_module_running || is_any_candidate_module_running_or_idle;

  updateCurrentRouteLanelet(data);

  const bool is_out_of_route = utils::isEgoOutOfRoute(
    data->self_odometry->pose.pose, current_route_lanelet_->value(), data->prev_modified_goal,
    data->route_handler);

  if (!is_any_module_running && is_out_of_route) {
    BehaviorModuleOutput result_output = utils::createGoalAroundPath(data);
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 5000,
      "Ego is out of route, no module is running. Skip running scene modules.");
    generateCombinedDrivableArea(result_output, data);
    return result_output;
  }
  std::vector<SceneModulePtr>
    deleted_modules;  // store the scene modules deleted from approved modules

  SlotOutput result_output = SlotOutput{
    getReferencePath(data),
    false,
    false,
    false,
  };

  for (auto & planner_manager_slot : planner_manager_slots_) {
    if (result_output.is_upstream_failed_approved) {
      // clear all candidate/approved modules of all subsequent slots, and keep result_output as is
      planner_manager_slot.propagateWithFailedApproved();
      debug_info_.slot_status.push_back(SlotStatus::UPSTREAM_APPROVED_FAILED);
    } else if (result_output.is_upstream_waiting_approved) {
      result_output = planner_manager_slot.propagateWithWaitingApproved(data, result_output);
      debug_info_.slot_status.push_back(SlotStatus::UPSTREAM_WAITING_APPROVED);
    } else if (result_output.is_upstream_candidate_exclusive) {
      result_output = planner_manager_slot.propagateWithExclusiveCandidate(data, result_output);
      debug_info_.slot_status.push_back(SlotStatus::UPSTREAM_EXCLUSIVE_CANDIDATE);
    } else {
      result_output = planner_manager_slot.propagateFull(data, result_output);
      debug_info_.slot_status.push_back(SlotStatus::NORMAL);
    }
  }

  std::for_each(manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) {
    m->updateObserver();
    m->publishRTCStatus();
  });

  generateCombinedDrivableArea(result_output.valid_output, data);
  return result_output.valid_output;
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

  const auto is_driving_forward_opt = autoware::motion_utils::isDrivingForward(output.path.points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  if (epsilon < std::abs(di.drivable_margin)) {
    // for single free space pull over
    utils::generateDrivableArea(
      output.path, data->parameters.vehicle_length, di.drivable_margin, is_driving_forward);
  } else if (di.is_already_expanded) {
    // for single side shift
    utils::generateDrivableArea(
      output.path, di.drivable_lanes, false, false, false, data, is_driving_forward);
  } else {
    const auto shorten_lanes = utils::cutOverlappedLanes(output.path, di.drivable_lanes);

    const auto & dp = data->drivable_area_expansion_parameters;
    const auto expanded_lanes = utils::expandLanelets(
      shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
      dp.drivable_area_types_to_skip);

    // for other modules where multiple modules may be launched
    utils::generateDrivableArea(
      output.path, expanded_lanes, di.enable_expanding_hatched_road_markings,
      di.enable_expanding_intersection_areas, di.enable_expanding_freespace_areas, data,
      is_driving_forward);
  }

  // extract obstacles from drivable area
  utils::extractObstaclesFromDrivableArea(output.path, di.obstacles);
}

void PlannerManager::updateCurrentRouteLanelet(const std::shared_ptr<PlannerData> & data)
{
  const auto & route_handler = data->route_handler;
  const auto & pose = data->self_odometry->pose.pose;
  const auto p = data->parameters;

  constexpr double extra_margin = 10.0;
  const auto backward_length =
    std::max(p.backward_path_length, p.backward_path_length + extra_margin);

  lanelet::ConstLanelet closest_lane{};

  if (route_handler->getClosestRouteLaneletFromLanelet(
        pose, current_route_lanelet_->value(), &closest_lane, p.ego_nearest_dist_threshold,
        p.ego_nearest_yaw_threshold)) {
    *current_route_lanelet_ = closest_lane;
    return;
  }

  const auto lanelet_sequence = route_handler->getLaneletSequence(
    current_route_lanelet_->value(), pose, backward_length, p.forward_path_length);

  const auto could_calculate_closest_lanelet =
    lanelet::utils::query::getClosestLaneletWithConstrains(
      lanelet_sequence, pose, &closest_lane, p.ego_nearest_dist_threshold,
      p.ego_nearest_yaw_threshold) ||
    lanelet::utils::query::getClosestLanelet(lanelet_sequence, pose, &closest_lane);

  if (could_calculate_closest_lanelet)
    *current_route_lanelet_ = closest_lane;
  else
    resetCurrentRouteLanelet(data);
}

BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  const auto reference_path = utils::getReferencePath(current_route_lanelet_->value(), data);
  publishDebugRootReferencePath(reference_path);
  return reference_path;
}

void PlannerManager::publishDebugRootReferencePath(
  const BehaviorModuleOutput & reference_path) const
{
  using visualization_msgs::msg::Marker;
  MarkerArray array;
  Marker m = autoware::universe_utils::createDefaultMarker(
    "map", clock_.now(), "root_reference_path", 0UL, Marker::LINE_STRIP,
    autoware::universe_utils::createMarkerScale(1.0, 1.0, 1.0),
    autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 1.0));
  for (const auto & p : reference_path.path.points) m.points.push_back(p.point.pose.position);
  array.markers.push_back(m);
  m.points.clear();
  m.id = 1UL;
  for (const auto & p : current_route_lanelet_->value().polygon3d().basicPolygon())
    m.points.emplace_back().set__x(p.x()).set__y(p.y()).set__z(p.z());
  array.markers.push_back(m);
  debug_publisher_ptr_->publish<MarkerArray>("root_reference_path", array);
}

bool PlannerManager::hasPossibleRerouteApprovedModules(
  const std::shared_ptr<PlannerData> & data) const
{
  const auto & approved_module = approved_modules();
  const auto not_possible_reroute_module = [&](const SceneModulePtr m) {
    if (m->name() == "dynamic_avoidance") {
      return false;
    }
    if (m->name() == "goal_planner" && !utils::isAllowedGoalModification(data->route_handler)) {
      return false;
    }
    return true;
  };

  return std::any_of(approved_module.begin(), approved_module.end(), not_possible_reroute_module);
}

void PlannerManager::print() const
{
  const auto approved_module_ptrs = approved_modules();
  const auto candidate_module_ptrs = candidate_modules();

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
  std::string delimiter = "";
  for (const auto & planner_manager_slot : planner_manager_slots_) {
    string_stream << std::exchange(delimiter, " ==> ") << "[[ ";
    std::string delimiter_sub = "";
    for (const auto & m : planner_manager_slot.approved_modules()) {
      string_stream << std::exchange(delimiter_sub, "->") << "[" << m->name() << "("
                    << get_status(m) << ")"
                    << "]";
    }
    string_stream << " ]]";
  }

  string_stream << "\n";
  string_stream << "candidate modules : ";
  delimiter = "";
  for (const auto & planner_manager_slot : planner_manager_slots_) {
    string_stream << std::exchange(delimiter, " ==> ") << "[[ ";
    std::string delimiter_sub = "";
    for (const auto & m : planner_manager_slot.candidate_modules()) {
      string_stream << std::exchange(delimiter_sub, "->") << "[" << m->name() << "("
                    << get_status(m) << ")"
                    << "]";
    }
    string_stream << " ]]";
  }

  string_stream << "\n";
  string_stream << "slot_status       : ";
  delimiter = "";
  for (const auto & slot_status : debug_info_.slot_status) {
    string_stream << std::exchange(delimiter, "->") << "[" << magic_enum::enum_name(slot_status)
                  << "]";
  }

  string_stream << "\n";
  string_stream << "update module info: ";
  for (const auto & i : debug_info_.scene_status) {
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

  state_publisher_ptr_->publish<DebugStringMsg>("internal_state", string_stream.str());

  RCLCPP_DEBUG_STREAM(logger_, string_stream.str());
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
  const auto approved_module_ptrs = approved_modules();
  const auto candidate_module_ptrs = candidate_modules();

  for (const auto & approved_module : approved_module_ptrs) {
    approved_module->acceptVisitor(debug_msg_ptr_);
  }

  for (const auto & candidate_module : candidate_module_ptrs) {
    candidate_module->acceptVisitor(debug_msg_ptr_);
  }
  return debug_msg_ptr_;
}

std::vector<SceneModulePtr> SubPlannerManager::getRequestModules(
  const BehaviorModuleOutput & previous_module_output,
  const std::vector<SceneModulePtr> & deleted_modules) const
{
  std::vector<SceneModulePtr> request_modules{};
  StopWatch<std::chrono::milliseconds> stop_watch;

  for (const auto & manager_ptr : manager_ptrs_) {
    stop_watch.tic(manager_ptr->name());
    BOOST_SCOPE_EXIT((&manager_ptr)(&processing_time_)(&stop_watch))
    {
      processing_time_.at(manager_ptr->name()) += stop_watch.toc(manager_ptr->name(), true);
    }
    BOOST_SCOPE_EXIT_END;

    if (const auto deleted_it = std::find_if(
          deleted_modules.begin(), deleted_modules.end(),
          [&](const auto & m) { return m->name() == manager_ptr->name(); });
        deleted_it != deleted_modules.end()) {
      continue;
    }

    // Condition 1:
    // the approved module queue is either
    // - consists of multiple simultaneous_executable_as_approved modules only
    // - consists of only 1 "not simultaneous_executable_as_approved" module
    const bool exclusive_module_exist_in_approved_pool = std::any_of(
      approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const SceneModulePtr & m) {
        return !getManager(m)->isSimultaneousExecutableAsApprovedModule();
      });
    const bool is_this_exclusive = !manager_ptr->isSimultaneousExecutableAsApprovedModule();
    const bool approved_pool_is_not_empty = !approved_module_ptrs_.empty();

    const bool is_this_not_joinable =
      exclusive_module_exist_in_approved_pool || (is_this_exclusive && approved_pool_is_not_empty);
    if (is_this_not_joinable) {
      continue;
    }

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
        continue;
      }
    }

    /**
     * same name module doesn't exist in candidate modules. launch new module.
     */
    {
      if (!manager_ptr->canLaunchNewModule()) {
        continue;
      }

      manager_ptr->updateIdleModuleInstance();
      if (!manager_ptr->isExecutionRequested(previous_module_output)) {
        continue;
      }

      request_modules.emplace_back(manager_ptr->getIdleModule());
    }
  }

  return request_modules;
}

SceneModulePtr SubPlannerManager::selectHighestPriorityModule(
  std::vector<SceneModulePtr> & request_modules) const
{
  if (request_modules.empty()) {
    return {};
  }

  sortByPriority(request_modules);

  return request_modules.front();
}

void SubPlannerManager::clearApprovedModules()
{
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [this](auto & m) {
    debug_info_.scene_status.emplace_back(
      m, SceneModuleUpdateInfo::Action::DELETE, "From Approved");
    deleteExpiredModules(m);
  });
  approved_module_ptrs_.clear();
}

void SubPlannerManager::updateCandidateModules(
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

std::pair<SceneModulePtr, BehaviorModuleOutput> SubPlannerManager::runRequestModules(
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

  // the candidate module queue is either
  // - consists of multiple simultaneous_executable_as_candidate modules only
  // - consists of only 1 "not simultaneous_executable_as_candidate" module
  for (const auto & module_ptr : sorted_request_modules) {
    // any module can join if executable_modules is empty
    const bool is_executable_modules_empty = executable_modules.empty();
    if (is_executable_modules_empty) {
      executable_modules.push_back(module_ptr);
      continue;
    }

    // if executable_module is not empty, only SimultaneousExecutableAsCandidate is joinable
    const bool is_this_cooperative =
      getManager(module_ptr)->isSimultaneousExecutableAsCandidateModule();
    const bool any_other_cooperative = std::any_of(
      executable_modules.begin(), executable_modules.end(), [&](const SceneModulePtr & m) {
        return getManager(m)->isSimultaneousExecutableAsCandidateModule();
      });

    if (is_this_cooperative && any_other_cooperative) {
      executable_modules.push_back(module_ptr);
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
  const auto module_ptr = [&]() -> SceneModulePtr {
    if (!already_approved_modules.empty()) {
      return selectHighestPriorityModule(already_approved_modules);
    }

    if (!waiting_approved_modules.empty()) {
      return selectHighestPriorityModule(waiting_approved_modules);
    }

    return nullptr;
  }();
  if (module_ptr == nullptr) {
    return std::make_pair(nullptr, BehaviorModuleOutput{});
  }

  /**
   * register candidate modules.
   */
  updateCandidateModules(executable_modules, module_ptr);

  return std::make_pair(module_ptr, results.at(module_ptr->name()));
}

BehaviorModuleOutput SubPlannerManager::run(
  const SceneModulePtr & module_ptr, const std::shared_ptr<PlannerData> & planner_data,
  const BehaviorModuleOutput & previous_module_output) const
{
  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic(module_ptr->name());

  module_ptr->setData(planner_data);
  module_ptr->setPreviousModuleOutput(previous_module_output);

  module_ptr->lockRTCCommand();
  const auto result = module_ptr->run();
  module_ptr->unlockRTCCommand();

  module_ptr->postProcess();

  module_ptr->updateCurrentState();

  module_ptr->publishSteeringFactor();

  module_ptr->publishObjectsOfInterestMarker();

  processing_time_.at(module_ptr->name()) += stop_watch.toc(module_ptr->name(), true);
  return result;
}

SlotOutput SubPlannerManager::runApprovedModules(
  const std::shared_ptr<PlannerData> & data, const BehaviorModuleOutput & upstream_slot_output)
{
  std::unordered_map<std::string, BehaviorModuleOutput> results;
  BehaviorModuleOutput output = upstream_slot_output;
  results.emplace("root", output);

  const bool is_candidate_plan_applied = true /* NOTE: not used in this process */;
  bool is_this_failed = false;
  bool is_this_waiting_approval = false;

  if (approved_module_ptrs_.empty()) {
    return SlotOutput{output, is_candidate_plan_applied, is_this_failed, is_this_waiting_approval};
  }

  // unlock only last approved module
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
    m->lockOutputPath();
  });
  approved_module_ptrs_.back()->unlockOutputPath();

  /**
   * bootstrap approved module output
   */
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
    output = run(m, data, output);
    results.emplace(m->name(), output);
  });

  const auto waiting_approval_modules_itr = std::find_if(
    approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
    [](const auto & m) { return m->isWaitingApproval(); });

  if (waiting_approval_modules_itr != approved_module_ptrs_.end()) {
    is_this_waiting_approval = true;

    // only keep this module as candidate
    clearCandidateModules();
    candidate_module_ptrs_.push_back(*waiting_approval_modules_itr);

    // delete following result but keep the rest of the following modules
    std::for_each(
      waiting_approval_modules_itr, approved_module_ptrs_.end(),
      [&results](const auto & m) { results.erase(m->name()); });
    debug_info_.scene_status.emplace_back(
      *waiting_approval_modules_itr, SceneModuleUpdateInfo::Action::MOVE,
      "Back To Waiting Approval");
    approved_module_ptrs_.erase(waiting_approval_modules_itr);

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  /**
   * remove failure modules. these modules' outputs are discarded as invalid plan.
   */
  const auto failed_itr = std::find_if(
    approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
    [](const auto & m) { return m->getCurrentStatus() == ModuleStatus::FAILURE; });
  if (failed_itr != approved_module_ptrs_.end()) {
    is_this_failed = true;

    // clear all candidates
    clearCandidateModules();

    // delete both subsequent result and modules
    std::for_each(failed_itr, approved_module_ptrs_.end(), [&](auto & m) {
      results.erase(m->name());
      debug_info_.scene_status.emplace_back(
        m, SceneModuleUpdateInfo::Action::DELETE, "From Approved");
      deleteExpiredModules(m);
    });
    approved_module_ptrs_.erase(failed_itr, approved_module_ptrs_.end());

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  if (approved_module_ptrs_.empty()) {
    return SlotOutput{
      results.at("root"), is_candidate_plan_applied, is_this_failed, is_this_waiting_approval};
  }

  // use the last module's output as approved modules planning result.
  const auto approved_modules_output = [&results, this]() {
    const auto itr = std::find_if(
      approved_module_ptrs_.rbegin(), approved_module_ptrs_.rend(),
      [&results](const auto & m) { return results.count(m->name()) != 0; });

    if (itr != approved_module_ptrs_.rend()) {
      return results.at((*itr)->name());
    }
    return results.at("root");
  }();

  // if lane change module has succeeded, update current route lanelet.
  if (std::any_of(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [](const auto & m) {
        return m->getCurrentStatus() == ModuleStatus::SUCCESS &&
               m->isCurrentRouteLaneletToBeReset();
      }))
    resetCurrentRouteLanelet(data);

  // remove success module immediately.
  for (auto success_itr = std::find_if(
         approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
         [](const auto & m) { return m->getCurrentStatus() == ModuleStatus::SUCCESS; });
       success_itr != approved_module_ptrs_.end();
       /* success_itr++ */) {
    if ((*success_itr)->getCurrentStatus() == ModuleStatus::SUCCESS) {
      debug_info_.scene_status.emplace_back(
        *success_itr, SceneModuleUpdateInfo::Action::DELETE, "From Approved");
      deleteExpiredModules(*success_itr);
      success_itr = approved_module_ptrs_.erase(success_itr);
    } else {
      success_itr++;
    }
  }

  std::for_each(
    manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });

  return SlotOutput{
    approved_modules_output, is_candidate_plan_applied, is_this_failed, is_this_waiting_approval};
}

SlotOutput SubPlannerManager::propagateFull(
  const std::shared_ptr<PlannerData> & data, const SlotOutput & previous_slot_output)
{
  const size_t module_size = manager_ptrs_.size();
  const size_t max_iteration_num = static_cast<int>(module_size * (module_size + 1) / 2);

  bool is_waiting_approved_slot = previous_slot_output.is_upstream_waiting_approved;
  bool is_failed_approved_slot = false;
  auto output_path = previous_slot_output.valid_output;

  std::vector<SceneModulePtr> deleted_modules;
  for (size_t itr_num = 0; itr_num < max_iteration_num; ++itr_num) {
    const auto approved_module_result = runApprovedModules(data, previous_slot_output.valid_output);
    const auto & approved_module_output = approved_module_result.valid_output;

    // these status needs to be propagated to downstream slots
    // if any of the slots returned following statuses, keep it
    is_waiting_approved_slot =
      is_waiting_approved_slot || approved_module_result.is_upstream_waiting_approved;
    is_failed_approved_slot =
      is_failed_approved_slot || approved_module_result.is_upstream_failed_approved;

    const auto request_modules = getRequestModules(approved_module_output, deleted_modules);
    if (request_modules.empty()) {
      // there is no module that needs to be launched
      return SlotOutput{
        approved_module_output, isAnyCandidateExclusive(), is_failed_approved_slot,
        is_waiting_approved_slot};
    }

    const auto [highest_priority_module, candidate_module_output] =
      runRequestModules(request_modules, data, approved_module_output);

    if (!highest_priority_module) {
      // there is no need to launch new module
      return SlotOutput{
        approved_module_output, isAnyCandidateExclusive(), is_failed_approved_slot,
        is_waiting_approved_slot};
    }

    if (highest_priority_module->isWaitingApproval()) {
      // there is no need to launch new module
      return SlotOutput{
        candidate_module_output, isAnyCandidateExclusive(), is_failed_approved_slot,
        is_waiting_approved_slot};
    }

    output_path = candidate_module_output;
    addApprovedModule(highest_priority_module);
    clearCandidateModules();
  }

  return SlotOutput{
    output_path, isAnyCandidateExclusive(), is_failed_approved_slot, is_waiting_approved_slot};
}

SlotOutput SubPlannerManager::propagateWithExclusiveCandidate(
  const std::shared_ptr<PlannerData> & data, const SlotOutput & previous_slot_output)
{
  const auto approved_module_result = runApprovedModules(data, previous_slot_output.valid_output);
  const auto & approved_module_output = approved_module_result.valid_output;

  // these status needs to be propagated to downstream slots
  // if any of the slots returned following statuses, keep it
  const bool is_waiting_approved_slot = previous_slot_output.is_upstream_waiting_approved |
                                        approved_module_result.is_upstream_waiting_approved;
  const bool is_failed_approved_slot = previous_slot_output.is_upstream_failed_approved |
                                       approved_module_result.is_upstream_failed_approved;

  // there is no module that needs to be launched
  return SlotOutput{
    approved_module_output, true, is_failed_approved_slot, is_waiting_approved_slot};
}

void SubPlannerManager::propagateWithFailedApproved()
{
  clearCandidateModules();
  clearApprovedModules();
}

SlotOutput SubPlannerManager::propagateWithWaitingApproved(
  const std::shared_ptr<PlannerData> & data, const SlotOutput & previous_slot_output)
{
  clearCandidateModules();

  return previous_slot_output.is_upstream_candidate_exclusive
           ? propagateWithExclusiveCandidate(data, previous_slot_output)
           : propagateFull(data, previous_slot_output);
}

}  // namespace autoware::behavior_path_planner
