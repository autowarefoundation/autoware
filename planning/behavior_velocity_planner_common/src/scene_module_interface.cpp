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

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <algorithm>
#include <limits>

namespace behavior_velocity_planner
{

using tier4_autoware_utils::StopWatch;

SceneModuleInterface::SceneModuleInterface(
  const int64_t module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: module_id_(module_id),
  activated_(false),
  safe_(false),
  distance_(std::numeric_limits<double>::lowest()),
  logger_(logger),
  clock_(clock)
{
}

size_t SceneModuleInterface::findEgoSegmentIndex(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points) const
{
  const auto & p = planner_data_;
  return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, p->current_odometry->pose, p->ego_nearest_dist_threshold);
}

SceneModuleManagerInterface::SceneModuleManagerInterface(
  rclcpp::Node & node, [[maybe_unused]] const char * module_name)
: node_(node), clock_(node.get_clock()), logger_(node.get_logger())
{
  const auto ns = std::string("~/debug/") + module_name;
  pub_debug_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(ns, 1);
  if (!node.has_parameter("is_publish_debug_path")) {
    is_publish_debug_path_ = node.declare_parameter<bool>("is_publish_debug_path");
  } else {
    is_publish_debug_path_ = node.get_parameter("is_publish_debug_path").as_bool();
  }
  if (is_publish_debug_path_) {
    pub_debug_path_ = node.create_publisher<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      std::string("~/debug/path_with_lane_id/") + module_name, 1);
  }
  pub_virtual_wall_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(
    std::string("~/virtual_wall/") + module_name, 5);
  pub_velocity_factor_ = node.create_publisher<autoware_adapi_v1_msgs::msg::VelocityFactorArray>(
    std::string("/planning/velocity_factors/") + module_name, 1);
  pub_stop_reason_ =
    node.create_publisher<tier4_planning_msgs::msg::StopReasonArray>("~/output/stop_reasons", 1);
  pub_infrastructure_commands_ =
    node.create_publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
      "~/output/infrastructure_commands", 1);

  processing_time_publisher_ = std::make_shared<DebugPublisher>(&node, "~/debug");
}

size_t SceneModuleManagerInterface::findEgoSegmentIndex(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points) const
{
  const auto & p = planner_data_;
  return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, p->current_odometry->pose, p->ego_nearest_dist_threshold, p->ego_nearest_yaw_threshold);
}

void SceneModuleManagerInterface::updateSceneModuleInstances(
  const std::shared_ptr<const PlannerData> & planner_data,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  planner_data_ = planner_data;

  launchNewModules(path);
  deleteExpiredModules(path);
}

void SceneModuleManagerInterface::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path)
{
  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("Total");
  visualization_msgs::msg::MarkerArray debug_marker_array;
  tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
  autoware_adapi_v1_msgs::msg::VelocityFactorArray velocity_factor_array;
  stop_reason_array.header.frame_id = "map";
  stop_reason_array.header.stamp = clock_->now();
  velocity_factor_array.header.frame_id = "map";
  velocity_factor_array.header.stamp = clock_->now();

  tier4_v2x_msgs::msg::InfrastructureCommandArray infrastructure_command_array;
  infrastructure_command_array.stamp = clock_->now();

  first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
  for (const auto & scene_module : scene_modules_) {
    tier4_planning_msgs::msg::StopReason stop_reason;
    scene_module->resetVelocityFactor();
    scene_module->setPlannerData(planner_data_);
    scene_module->modifyPathVelocity(path, &stop_reason);

    // The velocity factor must be called after modifyPathVelocity.
    const auto velocity_factor = scene_module->getVelocityFactor();
    if (velocity_factor.behavior != PlanningBehavior::UNKNOWN) {
      velocity_factor_array.factors.emplace_back(velocity_factor);
    }
    if (stop_reason.reason != "") {
      stop_reason_array.stop_reasons.emplace_back(stop_reason);
    }

    if (const auto command = scene_module->getInfrastructureCommand()) {
      infrastructure_command_array.commands.push_back(*command);
    }

    if (scene_module->getFirstStopPathPointIndex() < first_stop_path_point_index_) {
      first_stop_path_point_index_ = scene_module->getFirstStopPathPointIndex();
    }

    for (const auto & marker : scene_module->createDebugMarkerArray().markers) {
      debug_marker_array.markers.push_back(marker);
    }

    virtual_wall_marker_creator_.add_virtual_walls(scene_module->createVirtualWalls());
  }

  if (!stop_reason_array.stop_reasons.empty()) {
    pub_stop_reason_->publish(stop_reason_array);
  }
  pub_velocity_factor_->publish(velocity_factor_array);
  pub_infrastructure_commands_->publish(infrastructure_command_array);
  pub_debug_->publish(debug_marker_array);
  if (is_publish_debug_path_) {
    autoware_auto_planning_msgs::msg::PathWithLaneId debug_path;
    debug_path.header = path->header;
    debug_path.points = path->points;
    pub_debug_path_->publish(debug_path);
  }
  pub_virtual_wall_->publish(virtual_wall_marker_creator_.create_markers(clock_->now()));
  processing_time_publisher_->publish<Float64Stamped>(
    std::string(getModuleName()) + "/processing_time_ms", stop_watch.toc("Total"));
}

void SceneModuleManagerInterface::deleteExpiredModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto isModuleExpired = getModuleExpiredFunction(path);

  // Copy container to avoid iterator corruption
  // due to scene_modules_.erase() in unregisterModule()
  const auto copied_scene_modules = scene_modules_;

  for (const auto & scene_module : copied_scene_modules) {
    if (isModuleExpired(scene_module)) {
      unregisterModule(scene_module);
    }
  }
}

void SceneModuleManagerInterface::registerModule(
  const std::shared_ptr<SceneModuleInterface> & scene_module)
{
  RCLCPP_DEBUG(
    logger_, "register task: module = %s, id = %lu", getModuleName(), scene_module->getModuleId());
  registered_module_id_set_.emplace(scene_module->getModuleId());
  scene_modules_.insert(scene_module);
}

void SceneModuleManagerInterface::unregisterModule(
  const std::shared_ptr<SceneModuleInterface> & scene_module)
{
  RCLCPP_DEBUG(
    logger_, "unregister task: module = %s, id = %lu", getModuleName(),
    scene_module->getModuleId());
  registered_module_id_set_.erase(scene_module->getModuleId());
  scene_modules_.erase(scene_module);
}

SceneModuleManagerInterfaceWithRTC::SceneModuleManagerInterfaceWithRTC(
  rclcpp::Node & node, const char * module_name, const bool enable_rtc)
: SceneModuleManagerInterface(node, module_name),
  rtc_interface_(&node, module_name, enable_rtc),
  objects_of_interest_marker_interface_(&node, module_name)
{
}

void SceneModuleManagerInterfaceWithRTC::plan(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path)
{
  setActivation();
  modifyPathVelocity(path);
  sendRTC(path->header.stamp);
  publishObjectsOfInterestMarker();
}

void SceneModuleManagerInterfaceWithRTC::sendRTC(const Time & stamp)
{
  for (const auto & scene_module : scene_modules_) {
    const UUID uuid = getUUID(scene_module->getModuleId());
    updateRTCStatus(uuid, scene_module->isSafe(), scene_module->getDistance(), stamp);
  }
  publishRTCStatus(stamp);
}

void SceneModuleManagerInterfaceWithRTC::setActivation()
{
  for (const auto & scene_module : scene_modules_) {
    const UUID uuid = getUUID(scene_module->getModuleId());
    scene_module->setActivation(rtc_interface_.isActivated(uuid));
    scene_module->setRTCEnabled(rtc_interface_.isRTCEnabled(uuid));
  }
}

UUID SceneModuleManagerInterfaceWithRTC::getUUID(const int64_t & module_id) const
{
  if (map_uuid_.count(module_id) == 0) {
    const UUID uuid;
    return uuid;
  }
  return map_uuid_.at(module_id);
}

void SceneModuleManagerInterfaceWithRTC::generateUUID(const int64_t & module_id)
{
  map_uuid_.insert({module_id, tier4_autoware_utils::generateUUID()});
}

void SceneModuleManagerInterfaceWithRTC::removeUUID(const int64_t & module_id)
{
  const auto result = map_uuid_.erase(module_id);
  if (result == 0) {
    RCLCPP_WARN_STREAM(logger_, "[removeUUID] module_id = " << module_id << " is not registered.");
  }
}

void SceneModuleManagerInterfaceWithRTC::publishObjectsOfInterestMarker()
{
  for (const auto & scene_module : scene_modules_) {
    const auto objects = scene_module->getObjectsOfInterestData();
    for (const auto & obj : objects) {
      objects_of_interest_marker_interface_.insertObjectData(obj.pose, obj.shape, obj.color);
    }
    scene_module->clearObjectsOfInterestData();
  }

  objects_of_interest_marker_interface_.publishMarkerArray();
}

void SceneModuleManagerInterfaceWithRTC::deleteExpiredModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto isModuleExpired = getModuleExpiredFunction(path);

  // Copy container to avoid iterator corruption
  // due to scene_modules_.erase() in unregisterModule()
  const auto copied_scene_modules = scene_modules_;

  for (const auto & scene_module : copied_scene_modules) {
    if (isModuleExpired(scene_module)) {
      removeRTCStatus(getUUID(scene_module->getModuleId()));
      removeUUID(scene_module->getModuleId());
      unregisterModule(scene_module);
    }
  }
}

}  // namespace behavior_velocity_planner
