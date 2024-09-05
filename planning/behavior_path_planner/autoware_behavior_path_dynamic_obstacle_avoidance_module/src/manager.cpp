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

#include "autoware/behavior_path_dynamic_obstacle_avoidance_module/manager.hpp"

#include "autoware/universe_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace
{
PolygonGenerationMethod convertToPolygonGenerationMethod(const std::string & str)
{
  if (str == "ego_path_base") {
    return PolygonGenerationMethod::EGO_PATH_BASE;
  } else if (str == "object_path_base") {
    return PolygonGenerationMethod::OBJECT_PATH_BASE;
  }
  throw std::logic_error("The polygon_generation_method's string is invalid.");
}
}  // namespace

void DynamicObstacleAvoidanceModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {""});

  DynamicAvoidanceParameters p{};

  {  // common
    const std::string ns = "dynamic_avoidance.common.";
    p.enable_debug_info = node->declare_parameter<bool>(ns + "enable_debug_info");
    p.use_hatched_road_markings = node->declare_parameter<bool>(ns + "use_hatched_road_markings");
  }

  {  // target object
    const std::string ns = "dynamic_avoidance.target_object.";
    p.avoid_car = node->declare_parameter<bool>(ns + "car");
    p.avoid_truck = node->declare_parameter<bool>(ns + "truck");
    p.avoid_bus = node->declare_parameter<bool>(ns + "bus");
    p.avoid_trailer = node->declare_parameter<bool>(ns + "trailer");
    p.avoid_unknown = node->declare_parameter<bool>(ns + "unknown");
    p.avoid_bicycle = node->declare_parameter<bool>(ns + "bicycle");
    p.avoid_motorcycle = node->declare_parameter<bool>(ns + "motorcycle");
    p.avoid_pedestrian = node->declare_parameter<bool>(ns + "pedestrian");
    p.max_obstacle_vel = node->declare_parameter<double>(ns + "max_obstacle_vel");
    p.min_obstacle_vel = node->declare_parameter<double>(ns + "min_obstacle_vel");
    p.successive_num_to_entry_dynamic_avoidance_condition =
      node->declare_parameter<int>(ns + "successive_num_to_entry_dynamic_avoidance_condition");
    p.successive_num_to_exit_dynamic_avoidance_condition =
      node->declare_parameter<int>(ns + "successive_num_to_exit_dynamic_avoidance_condition");

    p.min_obj_lat_offset_to_ego_path =
      node->declare_parameter<double>(ns + "min_obj_lat_offset_to_ego_path");
    p.max_obj_lat_offset_to_ego_path =
      node->declare_parameter<double>(ns + "max_obj_lat_offset_to_ego_path");

    p.min_time_to_start_cut_in =
      node->declare_parameter<double>(ns + "cut_in_object.min_time_to_start_cut_in");
    p.min_lon_offset_ego_to_cut_in_object =
      node->declare_parameter<double>(ns + "cut_in_object.min_lon_offset_ego_to_object");
    p.min_cut_in_object_vel = node->declare_parameter<double>(ns + "cut_in_object.min_object_vel");

    p.max_time_from_outside_ego_path_for_cut_out =
      node->declare_parameter<double>(ns + "cut_out_object.max_time_from_outside_ego_path");
    p.min_cut_out_object_lat_vel =
      node->declare_parameter<double>(ns + "cut_out_object.min_object_lat_vel");
    p.min_cut_out_object_vel =
      node->declare_parameter<double>(ns + "cut_out_object.min_object_vel");

    p.max_front_object_angle =
      node->declare_parameter<double>(ns + "front_object.max_object_angle");
    p.max_front_object_ego_path_lat_cover_ratio =
      node->declare_parameter<double>(ns + "front_object.max_ego_path_lat_cover_ratio");
    p.min_front_object_vel = node->declare_parameter<double>(ns + "front_object.min_object_vel");

    p.min_overtaking_crossing_object_vel =
      node->declare_parameter<double>(ns + "crossing_object.min_overtaking_object_vel");
    p.max_overtaking_crossing_object_angle =
      node->declare_parameter<double>(ns + "crossing_object.max_overtaking_object_angle");
    p.min_oncoming_crossing_object_vel =
      node->declare_parameter<double>(ns + "crossing_object.min_oncoming_object_vel");
    p.max_oncoming_crossing_object_angle =
      node->declare_parameter<double>(ns + "crossing_object.max_oncoming_object_angle");
    p.max_pedestrian_crossing_vel =
      node->declare_parameter<double>(ns + "crossing_object.max_pedestrian_crossing_vel");

    p.max_stopped_object_vel =
      node->declare_parameter<double>(ns + "stopped_object.max_object_vel");
  }

  {  // drivable_area_generation
    const std::string ns = "dynamic_avoidance.drivable_area_generation.";
    p.expand_drivable_area = node->declare_parameter<bool>(ns + "expand_drivable_area");
    p.polygon_generation_method = convertToPolygonGenerationMethod(
      node->declare_parameter<std::string>(ns + "polygon_generation_method"));
    p.min_obj_path_based_lon_polygon_margin =
      node->declare_parameter<double>(ns + "object_path_base.min_longitudinal_polygon_margin");
    p.lat_offset_from_obstacle = node->declare_parameter<double>(ns + "lat_offset_from_obstacle");
    p.margin_distance_around_pedestrian =
      node->declare_parameter<double>(ns + "margin_distance_around_pedestrian");
    p.end_time_to_consider =
      node->declare_parameter<double>(ns + "predicted_path.end_time_to_consider");
    p.threshold_confidence =
      node->declare_parameter<double>(ns + "predicted_path.threshold_confidence");
    p.max_lat_offset_to_avoid = node->declare_parameter<double>(ns + "max_lat_offset_to_avoid");
    p.max_time_for_lat_shift =
      node->declare_parameter<double>(ns + "max_time_for_object_lat_shift");
    p.lpf_gain_for_lat_avoid_to_offset =
      node->declare_parameter<double>(ns + "lpf_gain_for_lat_avoid_to_offset");

    p.max_ego_lat_acc = node->declare_parameter<double>(ns + "max_ego_lat_acc");
    p.max_ego_lat_jerk = node->declare_parameter<double>(ns + "max_ego_lat_jerk");
    p.delay_time_ego_shift = node->declare_parameter<double>(ns + "delay_time_ego_shift");

    p.max_time_to_collision_overtaking_object =
      node->declare_parameter<double>(ns + "overtaking_object.max_time_to_collision");
    p.start_duration_to_avoid_overtaking_object =
      node->declare_parameter<double>(ns + "overtaking_object.start_duration_to_avoid");
    p.end_duration_to_avoid_overtaking_object =
      node->declare_parameter<double>(ns + "overtaking_object.end_duration_to_avoid");
    p.duration_to_hold_avoidance_overtaking_object =
      node->declare_parameter<double>(ns + "overtaking_object.duration_to_hold_avoidance");

    p.max_time_to_collision_oncoming_object =
      node->declare_parameter<double>(ns + "oncoming_object.max_time_to_collision");
    p.start_duration_to_avoid_oncoming_object =
      node->declare_parameter<double>(ns + "oncoming_object.start_duration_to_avoid");
    p.end_duration_to_avoid_oncoming_object =
      node->declare_parameter<double>(ns + "oncoming_object.end_duration_to_avoid");
  }

  parameters_ = std::make_shared<DynamicAvoidanceParameters>(p);
}

void DynamicObstacleAvoidanceModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;
  auto & p = parameters_;

  {  // common
    const std::string ns = "dynamic_avoidance.common.";
    updateParam<bool>(parameters, ns + "enable_debug_info", p->enable_debug_info);
    updateParam<bool>(parameters, ns + "use_hatched_road_markings", p->use_hatched_road_markings);
  }

  {  // target object
    const std::string ns = "dynamic_avoidance.target_object.";

    updateParam<bool>(parameters, ns + "car", p->avoid_car);
    updateParam<bool>(parameters, ns + "truck", p->avoid_truck);
    updateParam<bool>(parameters, ns + "bus", p->avoid_bus);
    updateParam<bool>(parameters, ns + "trailer", p->avoid_trailer);
    updateParam<bool>(parameters, ns + "unknown", p->avoid_unknown);
    updateParam<bool>(parameters, ns + "bicycle", p->avoid_bicycle);
    updateParam<bool>(parameters, ns + "motorcycle", p->avoid_motorcycle);
    updateParam<bool>(parameters, ns + "pedestrian", p->avoid_pedestrian);

    updateParam<double>(parameters, ns + "max_obstacle_vel", p->max_obstacle_vel);
    updateParam<double>(parameters, ns + "min_obstacle_vel", p->min_obstacle_vel);

    updateParam<int>(
      parameters, ns + "successive_num_to_entry_dynamic_avoidance_condition",
      p->successive_num_to_entry_dynamic_avoidance_condition);
    updateParam<int>(
      parameters, ns + "successive_num_to_exit_dynamic_avoidance_condition",
      p->successive_num_to_exit_dynamic_avoidance_condition);

    updateParam<double>(
      parameters, ns + "min_obj_lat_offset_to_ego_path", p->min_obj_lat_offset_to_ego_path);
    updateParam<double>(
      parameters, ns + "max_obj_lat_offset_to_ego_path", p->max_obj_lat_offset_to_ego_path);

    updateParam<double>(
      parameters, ns + "cut_in_object.min_time_to_start_cut_in", p->min_time_to_start_cut_in);
    updateParam<double>(
      parameters, ns + "cut_in_object.min_lon_offset_ego_to_object",
      p->min_lon_offset_ego_to_cut_in_object);
    updateParam<double>(parameters, ns + "cut_in_object.min_object_vel", p->min_cut_in_object_vel);

    updateParam<double>(
      parameters, ns + "cut_out_object.max_time_from_outside_ego_path",
      p->max_time_from_outside_ego_path_for_cut_out);
    updateParam<double>(
      parameters, ns + "cut_out_object.min_object_lat_vel", p->min_cut_out_object_lat_vel);
    updateParam<double>(
      parameters, ns + "cut_out_object.min_object_vel", p->min_cut_out_object_vel);

    updateParam<double>(
      parameters, ns + "front_object.max_object_angle", p->max_front_object_angle);
    updateParam<double>(
      parameters, ns + "front_object.max_ego_path_lat_cover_ratio",
      p->max_front_object_ego_path_lat_cover_ratio);
    updateParam<double>(parameters, ns + "front_object.min_object_vel", p->min_front_object_vel);

    updateParam<double>(
      parameters, ns + "crossing_object.min_overtaking_object_vel",
      p->min_overtaking_crossing_object_vel);
    updateParam<double>(
      parameters, ns + "crossing_object.max_overtaking_object_angle",
      p->max_overtaking_crossing_object_angle);
    updateParam<double>(
      parameters, ns + "crossing_object.min_oncoming_object_vel",
      p->min_oncoming_crossing_object_vel);
    updateParam<double>(
      parameters, ns + "crossing_object.max_oncoming_object_angle",
      p->max_oncoming_crossing_object_angle);
    updateParam<double>(
      parameters, ns + "crossing_object.max_pedestrian_crossing_vel",
      p->max_pedestrian_crossing_vel);

    updateParam<double>(
      parameters, ns + "stopped_object.max_object_vel", p->max_stopped_object_vel);
  }

  {  // drivable_area_generation
    const std::string ns = "dynamic_avoidance.drivable_area_generation.";
    std::string polygon_generation_method_str;
    if (updateParam<std::string>(
          parameters, ns + "polygon_generation_method", polygon_generation_method_str)) {
      p->polygon_generation_method =
        convertToPolygonGenerationMethod(polygon_generation_method_str);
    }
    updateParam<bool>(parameters, ns + "expand_drivable_area", p->expand_drivable_area);
    updateParam<double>(
      parameters, ns + "object_path_base.min_longitudinal_polygon_margin",
      p->min_obj_path_based_lon_polygon_margin);
    updateParam<double>(parameters, ns + "lat_offset_from_obstacle", p->lat_offset_from_obstacle);
    updateParam<double>(
      parameters, ns + "margin_distance_around_pedestrian", p->margin_distance_around_pedestrian);
    updateParam<double>(
      parameters, ns + "predicted_path.end_time_to_consider", p->end_time_to_consider);
    updateParam<double>(
      parameters, ns + "predicted_path.threshold_confidence", p->threshold_confidence);
    updateParam<double>(parameters, ns + "max_lat_offset_to_avoid", p->max_lat_offset_to_avoid);
    updateParam<double>(
      parameters, ns + "max_time_for_object_lat_shift", p->max_time_for_lat_shift);
    updateParam<double>(
      parameters, ns + "lpf_gain_for_lat_avoid_to_offset", p->lpf_gain_for_lat_avoid_to_offset);

    updateParam<double>(parameters, ns + "max_ego_lat_acc", p->max_ego_lat_acc);
    updateParam<double>(parameters, ns + "max_ego_lat_jerk", p->max_ego_lat_jerk);
    updateParam<double>(parameters, ns + "delay_time_ego_shift", p->delay_time_ego_shift);

    updateParam<double>(
      parameters, ns + "overtaking_object.max_time_to_collision",
      p->max_time_to_collision_overtaking_object);
    updateParam<double>(
      parameters, ns + "overtaking_object.start_duration_to_avoid",
      p->start_duration_to_avoid_overtaking_object);
    updateParam<double>(
      parameters, ns + "overtaking_object.end_duration_to_avoid",
      p->end_duration_to_avoid_overtaking_object);
    updateParam<double>(
      parameters, ns + "overtaking_object.duration_to_hold_avoidance",
      p->duration_to_hold_avoidance_overtaking_object);

    updateParam<double>(
      parameters, ns + "oncoming_object.max_time_to_collision",
      p->max_time_to_collision_oncoming_object);
    updateParam<double>(
      parameters, ns + "oncoming_object.start_duration_to_avoid",
      p->start_duration_to_avoid_oncoming_object);
    updateParam<double>(
      parameters, ns + "oncoming_object.end_duration_to_avoid",
      p->end_duration_to_avoid_oncoming_object);
  }

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::DynamicObstacleAvoidanceModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
