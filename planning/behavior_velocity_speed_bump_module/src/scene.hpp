// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include "util.hpp"

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <lanelet2_extension/regulatory_elements/speed_bump.hpp>
#include <rclcpp/rclcpp.hpp>

#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;

class SpeedBumpModule : public SceneModuleInterface
{
public:
  struct DebugData
  {
    double base_link2front;
    PathPolygonIntersectionStatus path_polygon_intersection_status;
    std::vector<geometry_msgs::msg::Pose> slow_start_poses;
    std::vector<geometry_msgs::msg::Point> slow_end_points;
    std::vector<geometry_msgs::msg::Point> speed_bump_polygon;
  };

  struct PlannerParam
  {
    double slow_start_margin;
    double slow_end_margin;
    bool print_debug_info;
    float speed_calculation_min_height;
    float speed_calculation_max_height;
    float speed_calculation_min_speed;
    float speed_calculation_max_speed;
  };

  SpeedBumpModule(
    const int64_t module_id, const int64_t lane_id,
    const lanelet::autoware::SpeedBump & speed_bump_reg_elem, const PlannerParam & planner_param,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;

private:
  int64_t module_id_;
  int64_t lane_id_;

  // Speed Bump Regulatory Element
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  bool applySlowDownSpeed(
    PathWithLaneId & output, const float speed_bump_speed,
    const PathPolygonIntersectionStatus & path_polygon_intersection_status);

  float speed_bump_slow_down_speed_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_HPP_
