// Copyright 2022 TIER IV, Inc.
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

#include "scene_module/run_out/manager.hpp"

namespace behavior_velocity_planner
{
namespace
{
}  // namespace

RunOutModuleManager::RunOutModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  // Vehicle Parameters
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
    auto & p = planner_param_.vehicle_param;
    p.base_to_front = vehicle_info.wheel_base_m + vehicle_info.front_overhang_m;
    p.base_to_rear = vehicle_info.rear_overhang_m;
    p.width = vehicle_info.vehicle_width_m;
    p.wheel_tread = vehicle_info.wheel_tread_m;
    p.right_overhang = vehicle_info.right_overhang_m;
    p.left_overhang = vehicle_info.left_overhang_m;
  }

  const std::string ns(getModuleName());

  {
    auto & p = planner_param_.smoother;
    // smoother parameters are already declared in behavior velocity node, so use get parameter
    p.start_jerk = node.get_parameter("backward.start_jerk").get_value<double>();
  }

  {
    auto & p = planner_param_.common;
    p.normal_min_jerk = node.declare_parameter(".normal.min_jerk", -0.3);
    p.normal_min_acc = node.declare_parameter(".normal.min_acc", -1.0);
    p.limit_min_jerk = node.declare_parameter(".limit.min_jerk", -1.5);
    p.limit_min_acc = node.declare_parameter(".limit.min_acc", -2.5);
  }

  {
    auto & p = planner_param_.run_out;
    p.detection_method = node.declare_parameter(ns + ".detection_method", "Object");
    p.use_partition_lanelet = node.declare_parameter(ns + ".use_partition_lanelet", true);
    p.specify_decel_jerk = node.declare_parameter(ns + ".specify_decel_jerk", false);
    p.stop_margin = node.declare_parameter(ns + ".stop_margin", 2.5);
    p.passing_margin = node.declare_parameter(ns + ".passing_margin", 1.0);
    p.deceleration_jerk = node.declare_parameter(ns + ".deceleration_jerk", -0.3);
    p.detection_distance = node.declare_parameter(ns + ".detection_distance", 45.0);
    p.detection_span = node.declare_parameter(ns + ".detection_span", 1.0);
    p.min_vel_ego_kmph = node.declare_parameter(ns + ".min_vel_ego_kmph", 5.0);
  }

  {
    auto & p = planner_param_.detection_area;
    const std::string ns_da = ns + ".detection_area";
    p.margin_ahead = node.declare_parameter(ns_da + ".margin_ahead", 1.0);
    p.margin_behind = node.declare_parameter(ns_da + ".margin_behind", 0.5);
  }

  {
    auto & p = planner_param_.dynamic_obstacle;
    const std::string ns_do = ns + ".dynamic_obstacle";
    p.min_vel_kmph = node.declare_parameter(ns_do + ".min_vel_kmph", 0.0);
    p.max_vel_kmph = node.declare_parameter(ns_do + ".max_vel_kmph", 5.0);
    p.diameter = node.declare_parameter(ns_do + ".diameter", 0.1);
    p.height = node.declare_parameter(ns_do + ".height", 2.0);
    p.max_prediction_time = node.declare_parameter(ns_do + ".max_prediction_time", 10.0);
    p.time_step = node.declare_parameter(ns_do + ".time_step", 0.5);
    p.points_interval = node.declare_parameter(ns_do + ".points_interval", 0.1);
  }

  {
    auto & p = planner_param_.approaching;
    const std::string ns_a = ns + ".approaching";
    p.enable = node.declare_parameter(ns_a + ".enable", false);
    p.margin = node.declare_parameter(ns_a + ".margin", 0.0);
    p.limit_vel_kmph = node.declare_parameter(ns_a + ".limit_vel_kmph", 5.0);
  }

  {
    auto & p = planner_param_.state_param;
    const std::string ns_s = ns + ".state";
    p.stop_thresh = node.declare_parameter(ns_s + ".stop_thresh", 0.01);
    p.stop_time_thresh = node.declare_parameter(ns_s + ".stop_time_thresh", 3.0);
    p.disable_approach_dist = node.declare_parameter(ns_s + ".disable_approach_dist", 4.0);
    p.keep_approach_duration = node.declare_parameter(ns_s + ".keep_approach_duration", 1.0);
  }

  {
    auto & p = planner_param_.slow_down_limit;
    const std::string ns_m = ns + ".slow_down_limit";
    p.enable = node.declare_parameter(ns_m + ".enable", true);
    p.max_jerk = node.declare_parameter(ns_m + ".max_jerk", -0.7);
    p.max_acc = node.declare_parameter(ns_m + ".max_acc", -2.0);
  }

  debug_ptr_ = std::make_shared<RunOutDebug>(node);
  setDynamicObstacleCreator(node);
}

void RunOutModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.empty()) {
    return;
  }

  constexpr int64_t module_id = 0;
  if (!isModuleRegistered(module_id)) {
    registerModule(std::make_shared<RunOutModule>(
      module_id, planner_data_, planner_param_, logger_.get_child("run_out_module"),
      std::move(dynamic_obstacle_creator_), debug_ptr_, clock_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
RunOutModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  return
    [&path]([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) -> bool {
      return false;
    };
}

void RunOutModuleManager::setDynamicObstacleCreator(rclcpp::Node & node)
{
  using run_out_utils::DetectionMethod;

  const auto detection_method_enum = run_out_utils::toEnum(planner_param_.run_out.detection_method);
  switch (detection_method_enum) {
    case DetectionMethod::Object:
      dynamic_obstacle_creator_ = std::make_unique<DynamicObstacleCreatorForObject>(node);
      break;

    case DetectionMethod::ObjectWithoutPath:
      dynamic_obstacle_creator_ =
        std::make_unique<DynamicObstacleCreatorForObjectWithoutPath>(node);
      break;

    case DetectionMethod::Points:
      dynamic_obstacle_creator_ = std::make_unique<DynamicObstacleCreatorForPoints>(node);
      break;

    default:
      RCLCPP_WARN_STREAM(logger_, "detection method is invalid. use default method (Object).");
      dynamic_obstacle_creator_ = std::make_unique<DynamicObstacleCreatorForObject>(node);
      break;
  }

  dynamic_obstacle_creator_->setParam(planner_param_.dynamic_obstacle);
}
}  // namespace behavior_velocity_planner
