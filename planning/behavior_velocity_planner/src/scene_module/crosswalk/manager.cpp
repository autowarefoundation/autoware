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

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace
{
std::vector<lanelet::ConstLanelet> getCrosswalksOnPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::vector<lanelet::ConstLanelet> crosswalks;

  std::set<int64_t> unique_lane_ids;
  for (const auto & p : path.points) {
    unique_lane_ids.insert(p.lane_ids.at(0));  // should we iterate ids? keep as it was.
  }

  for (const auto & lane_id : unique_lane_ids) {
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
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::set<int64_t> crosswalk_id_set;

  for (const auto & crosswalk : getCrosswalksOnPath(path, lanelet_map, overall_graphs)) {
    crosswalk_id_set.insert(crosswalk.id());
  }

  return crosswalk_id_set;
}

}  // namespace

CrosswalkModuleManager::CrosswalkModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());

  // for crosswalk parameters
  auto & cp = crosswalk_planner_param_;
  cp.stop_line_distance = node.declare_parameter(ns + ".crosswalk.stop_line_distance", 1.5);
  cp.stop_margin = node.declare_parameter(ns + ".crosswalk.stop_margin", 1.0);
  cp.slow_margin = node.declare_parameter(ns + ".crosswalk.slow_margin", 2.0);
  cp.slow_velocity = node.declare_parameter(ns + ".crosswalk.slow_velocity", 5.0 / 3.6);
  cp.stop_predicted_object_prediction_time_margin =
    node.declare_parameter(ns + ".crosswalk.stop_predicted_object_prediction_time_margin", 3.0);
  cp.external_input_timeout = node.declare_parameter(ns + ".crosswalk.external_input_timeout", 1.0);

  // for walkway parameters
  auto & wp = walkway_planner_param_;
  wp.stop_margin = node.declare_parameter(ns + ".walkway.stop_margin", 1.0);
  wp.stop_line_distance = node.declare_parameter(ns + ".walkway.stop_line_distance", 1.0);
  wp.stop_duration_sec = node.declare_parameter(ns + ".walkway.stop_duration_sec", 1.0);
  wp.external_input_timeout = node.declare_parameter(ns + ".walkway.external_input_timeout", 1.0);
}

void CrosswalkModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & crosswalk :
       getCrosswalksOnPath(path, planner_data_->lanelet_map, planner_data_->overall_graphs)) {
    const auto module_id = crosswalk.id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<CrosswalkModule>(
        module_id, crosswalk, crosswalk_planner_param_, logger_.get_child("crosswalk_module"),
        clock_));
      if (
        crosswalk.attributeOr(lanelet::AttributeNamesString::Subtype, std::string("")) ==
        lanelet::AttributeValueString::Walkway) {
        registerModule(std::make_shared<WalkwayModule>(
          module_id, crosswalk, walkway_planner_param_, logger_.get_child("walkway_module"),
          clock_));
      }
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
CrosswalkModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto crosswalk_id_set =
    getCrosswalkIdSetOnPath(path, planner_data_->lanelet_map, planner_data_->overall_graphs);

  return [crosswalk_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return crosswalk_id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace behavior_velocity_planner
