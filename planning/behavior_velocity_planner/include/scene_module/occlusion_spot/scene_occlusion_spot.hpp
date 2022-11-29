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

#ifndef SCENE_MODULE__OCCLUSION_SPOT__SCENE_OCCLUSION_SPOT_HPP_
#define SCENE_MODULE__OCCLUSION_SPOT__SCENE_OCCLUSION_SPOT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>
#include <utilization/boost_geometry_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
class OcclusionSpotModule : public SceneModuleInterface
{
  using PlannerParam = occlusion_spot_utils::PlannerParam;
  using DebugData = occlusion_spot_utils::DebugData;

public:
  OcclusionSpotModule(
    const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
    const PlannerParam & planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief plan occlusion spot velocity at unknown area in occupancy grid
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() override;

private:
  // Parameter
  PlannerParam param_;
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  std::vector<lanelet::BasicPolygon2d> partition_lanelets_;

protected:
  int64_t module_id_{};

  // Debug
  mutable DebugData debug_data_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__SCENE_OCCLUSION_SPOT_HPP_
