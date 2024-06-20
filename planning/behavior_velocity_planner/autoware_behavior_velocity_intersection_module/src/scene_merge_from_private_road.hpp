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

#ifndef SCENE_MERGE_FROM_PRIVATE_ROAD_HPP_
#define SCENE_MERGE_FROM_PRIVATE_ROAD_HPP_

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

/**
 * @brief This module makes sure that vehicle will stop before entering public road from private
 * road. This module is meant to be registered with intersection module, which looks at intersecting
 * lanes before entering intersection
 */

namespace autoware::behavior_velocity_planner
{
class MergeFromPrivateRoadModule : public SceneModuleInterface
{
public:
  struct DebugData
  {
    geometry_msgs::msg::Pose virtual_wall_pose;
    geometry_msgs::msg::Pose stop_point_pose;
  };

public:
  struct PlannerParam
  {
    double attention_area_length;
    double stopline_margin;
    double stop_duration_sec;
    double stop_distance_threshold;
    double path_interpolation_ds;
    double occlusion_attention_area_length;
    bool consider_wrong_direction_vehicle;
  };

  MergeFromPrivateRoadModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
    const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

  const std::set<lanelet::Id> & getAssociativeIds() const { return associative_ids_; }
  lanelet::ConstLanelets getAttentionLanelets() const;

private:
  const int64_t lane_id_;
  const std::set<lanelet::Id> associative_ids_;

  // Parameter
  PlannerParam planner_param_;
  std::optional<lanelet::ConstLanelet> first_conflicting_lanelet_;

  StateMachine state_machine_;  //! for state

  // Debug
  mutable DebugData debug_data_;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_MERGE_FROM_PRIVATE_ROAD_HPP_
