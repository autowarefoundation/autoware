// Copyright 2021 Tier IV, Inc.
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

#include <scene_module/occlusion_spot/manager.hpp>
#include <scene_module/occlusion_spot/scene_occlusion_spot.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
using occlusion_spot_utils::DETECTION_METHOD;
using occlusion_spot_utils::PASS_JUDGE;

OcclusionSpotModuleManager::OcclusionSpotModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());
  auto & pp = planner_param_;
  // for detection type
  {
    const std::string method = node.declare_parameter<std::string>(ns + ".detection_method");
    if (method == "occupancy_grid") {  // module id 0
      pp.detection_method = DETECTION_METHOD::OCCUPANCY_GRID;
      module_id_ = DETECTION_METHOD::OCCUPANCY_GRID;
    } else if (method == "predicted_object") {  // module id 1
      pp.detection_method = DETECTION_METHOD::PREDICTED_OBJECT;
      module_id_ = DETECTION_METHOD::PREDICTED_OBJECT;
    } else {
      throw std::invalid_argument{
        "[behavior_velocity]: occlusion spot detection method has invalid argument"};
    }
  }
  // for passable judgement
  {
    const std::string pass_judge = node.declare_parameter<std::string>(ns + ".pass_judge");
    if (pass_judge == "current_velocity") {
      pp.pass_judge = PASS_JUDGE::CURRENT_VELOCITY;
    } else if (pass_judge == "smooth_velocity") {
      pp.pass_judge = PASS_JUDGE::SMOOTH_VELOCITY;
    } else {
      throw std::invalid_argument{
        "[behavior_velocity]: occlusion spot pass judge method has invalid argument"};
    }
  }
  pp.use_object_info = node.declare_parameter<bool>(ns + ".use_object_info");
  pp.use_moving_object_ray_cast = node.declare_parameter<bool>(ns + ".use_moving_object_ray_cast");
  pp.use_partition_lanelet = node.declare_parameter<bool>(ns + ".use_partition_lanelet");
  pp.pedestrian_vel = node.declare_parameter<double>(ns + ".pedestrian_vel");
  pp.pedestrian_radius = node.declare_parameter<double>(ns + ".pedestrian_radius");

  // debug
  pp.is_show_occlusion = node.declare_parameter<bool>(ns + ".debug.is_show_occlusion");
  pp.is_show_cv_window = node.declare_parameter<bool>(ns + ".debug.is_show_cv_window");
  pp.is_show_processing_time = node.declare_parameter<bool>(ns + ".debug.is_show_processing_time");

  // threshold
  pp.detection_area_offset =
    node.declare_parameter<double>(ns + ".threshold.detection_area_offset");
  pp.detection_area_length =
    node.declare_parameter<double>(ns + ".threshold.detection_area_length");
  pp.stuck_vehicle_vel = node.declare_parameter<double>(ns + ".threshold.stuck_vehicle_vel");
  pp.lateral_distance_thr = node.declare_parameter<double>(ns + ".threshold.lateral_distance");
  pp.dist_thr = node.declare_parameter<double>(ns + ".threshold.search_dist");
  pp.angle_thr = node.declare_parameter<double>(ns + ".threshold.search_angle");

  // ego additional velocity config
  pp.v.safety_ratio = node.declare_parameter<double>(ns + ".motion.safety_ratio");
  pp.v.safe_margin = node.declare_parameter<double>(ns + ".motion.safe_margin");
  pp.v.max_slow_down_jerk = node.declare_parameter<double>(ns + ".motion.max_slow_down_jerk");
  pp.v.max_slow_down_accel = node.declare_parameter<double>(ns + ".motion.max_slow_down_accel");
  pp.v.non_effective_jerk = node.declare_parameter<double>(ns + ".motion.non_effective_jerk");
  pp.v.non_effective_accel =
    node.declare_parameter<double>(ns + ".motion.non_effective_acceleration");
  pp.v.min_allowed_velocity = node.declare_parameter<double>(ns + ".motion.min_allowed_velocity");
  // detection_area param
  pp.detection_area.min_occlusion_spot_size =
    node.declare_parameter<double>(ns + ".detection_area.min_occlusion_spot_size");
  pp.detection_area.min_longitudinal_offset =
    node.declare_parameter<double>(ns + ".detection_area.min_longitudinal_offset");
  pp.detection_area.max_lateral_distance =
    node.declare_parameter<double>(ns + ".detection_area.max_lateral_distance");
  pp.detection_area.slice_length =
    node.declare_parameter<double>(ns + ".detection_area.slice_length");
  // occupancy grid param
  pp.grid.free_space_max = node.declare_parameter<int>(ns + ".grid.free_space_max");
  pp.grid.occupied_min = node.declare_parameter<int>(ns + ".grid.occupied_min");

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  pp.baselink_to_front = vehicle_info.max_longitudinal_offset_m;
  pp.wheel_tread = vehicle_info.wheel_tread_m;
  pp.right_overhang = vehicle_info.right_overhang_m;
  pp.left_overhang = vehicle_info.left_overhang_m;
}

void OcclusionSpotModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.empty()) return;
  // general
  if (!isModuleRegistered(module_id_)) {
    registerModule(std::make_shared<OcclusionSpotModule>(
      module_id_, planner_data_, planner_param_, logger_.get_child("occlusion_spot_module"),
      clock_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
OcclusionSpotModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  return [path]([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return false;
  };
}
}  // namespace behavior_velocity_planner
