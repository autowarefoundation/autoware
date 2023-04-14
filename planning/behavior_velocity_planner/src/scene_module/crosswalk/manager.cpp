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

#include <scene_module/crosswalk/manager.hpp>
#include <utilization/util.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace
{
std::vector<lanelet::ConstLanelet> getCrosswalksOnPath(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::vector<lanelet::ConstLanelet> crosswalks;

  // Add current lane id
  const auto nearest_lane_id =
    behavior_velocity_planner::planning_utils::getNearestLaneId(path, lanelet_map, current_pose);

  std::vector<int64_t> unique_lane_ids;
  if (nearest_lane_id) {
    // Add subsequent lane_ids from nearest lane_id
    unique_lane_ids = behavior_velocity_planner::planning_utils::getSubsequentLaneIdsSetOnPath(
      path, *nearest_lane_id);
  } else {
    // Add all lane_ids in path
    unique_lane_ids = behavior_velocity_planner::planning_utils::getSortedLaneIdsFromPath(path);
  }

  for (const auto lane_id : unique_lane_ids) {
    const auto ll = lanelet_map->laneletLayer.get(lane_id);

    constexpr int PEDESTRIAN_GRAPH_ID = 1;
    const auto conflicting_crosswalks = overall_graphs->conflictingInGraph(ll, PEDESTRIAN_GRAPH_ID);
    for (const auto & crosswalk : conflicting_crosswalks) {
      crosswalks.push_back(crosswalk);
    }
  }

  return crosswalks;
}

std::set<int64_t> getCrosswalkIdSetOnPath(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::set<int64_t> crosswalk_id_set;

  for (const auto & crosswalk :
       getCrosswalksOnPath(current_pose, path, lanelet_map, overall_graphs)) {
    crosswalk_id_set.insert(crosswalk.id());
  }

  return crosswalk_id_set;
}
}  // namespace

namespace behavior_velocity_planner
{
CrosswalkModuleManager::CrosswalkModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(node, getModuleName())
{
  const std::string ns(getModuleName());

  // for crosswalk parameters
  auto & cp = crosswalk_planner_param_;
  cp.show_processing_time = node.declare_parameter<bool>(ns + ".show_processing_time");

  // param for stop position
  cp.stop_line_distance = node.declare_parameter<double>(ns + ".stop_line_distance");
  cp.stop_margin = node.declare_parameter<double>(ns + ".stop_margin");
  cp.stop_line_margin = node.declare_parameter<double>(ns + ".stop_line_margin");
  cp.stop_position_threshold = node.declare_parameter<double>(ns + ".stop_position_threshold");

  // param for ego velocity
  cp.min_slow_down_velocity = node.declare_parameter<double>(ns + ".min_slow_down_velocity");
  cp.max_slow_down_jerk = node.declare_parameter<double>(ns + ".max_slow_down_jerk");
  cp.max_slow_down_accel = node.declare_parameter<double>(ns + ".max_slow_down_accel");
  cp.no_relax_velocity = node.declare_parameter<double>(ns + ".no_relax_velocity");

  // param for stuck vehicle
  cp.stuck_vehicle_velocity = node.declare_parameter<double>(ns + ".stuck_vehicle_velocity");
  cp.max_lateral_offset = node.declare_parameter<double>(ns + ".max_lateral_offset");
  cp.stuck_vehicle_attention_range =
    node.declare_parameter<double>(ns + ".stuck_vehicle_attention_range");

  // param for pass judge logic
  cp.ego_pass_first_margin = node.declare_parameter<double>(ns + ".ego_pass_first_margin");
  cp.ego_pass_later_margin = node.declare_parameter<double>(ns + ".ego_pass_later_margin");
  cp.stop_object_velocity = node.declare_parameter<double>(ns + ".stop_object_velocity_threshold");
  cp.min_object_velocity = node.declare_parameter<double>(ns + ".min_object_velocity");
  cp.max_yield_timeout = node.declare_parameter<double>(ns + ".max_yield_timeout");
  cp.ego_yield_query_stop_duration =
    node.declare_parameter<double>(ns + ".ego_yield_query_stop_duration");

  // param for input data
  cp.tl_state_timeout = node.declare_parameter<double>(ns + ".tl_state_timeout");

  // param for target area & object
  cp.crosswalk_attention_range = node.declare_parameter<double>(ns + ".crosswalk_attention_range");
  cp.look_unknown = node.declare_parameter<bool>(ns + ".target_object.unknown");
  cp.look_bicycle = node.declare_parameter<bool>(ns + ".target_object.bicycle");
  cp.look_motorcycle = node.declare_parameter<bool>(ns + ".target_object.motorcycle");
  cp.look_pedestrian = node.declare_parameter<bool>(ns + ".target_object.pedestrian");
}

void CrosswalkModuleManager::launchNewModules(const PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;
  for (const auto & crosswalk : getCrosswalksOnPath(
         planner_data_->current_odometry->pose, path, rh->getLaneletMapPtr(),
         rh->getOverallGraphPtr())) {
    const auto module_id = crosswalk.id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<CrosswalkModule>(
        module_id, crosswalk, crosswalk_planner_param_, logger_.get_child("crosswalk_module"),
        clock_));
      generateUUID(module_id);
      updateRTCStatus(
        getUUID(module_id), true, std::numeric_limits<double>::lowest(), path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
CrosswalkModuleManager::getModuleExpiredFunction(const PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;
  const auto crosswalk_id_set = getCrosswalkIdSetOnPath(
    planner_data_->current_odometry->pose, path, rh->getLaneletMapPtr(), rh->getOverallGraphPtr());

  return [crosswalk_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return crosswalk_id_set.count(scene_module->getModuleId()) == 0;
  };
}

WalkwayModuleManager::WalkwayModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());

  // for walkway parameters
  auto & wp = walkway_planner_param_;
  wp.stop_line_distance = node.declare_parameter<double>(ns + ".stop_line_distance");
  wp.stop_duration_sec = node.declare_parameter<double>(ns + ".stop_duration_sec");
}

void WalkwayModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;
  for (const auto & crosswalk : getCrosswalksOnPath(
         planner_data_->current_odometry->pose, path, rh->getLaneletMapPtr(),
         rh->getOverallGraphPtr())) {
    const auto module_id = crosswalk.id();
    if (
      !isModuleRegistered(module_id) &&
      crosswalk.attributeOr(lanelet::AttributeNamesString::Subtype, std::string("")) ==
        lanelet::AttributeValueString::Walkway) {
      registerModule(std::make_shared<WalkwayModule>(
        module_id, crosswalk, walkway_planner_param_, logger_.get_child("walkway_module"), clock_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
WalkwayModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;
  const auto walkway_id_set = getCrosswalkIdSetOnPath(
    planner_data_->current_odometry->pose, path, rh->getLaneletMapPtr(), rh->getOverallGraphPtr());

  return [walkway_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return walkway_id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace behavior_velocity_planner
