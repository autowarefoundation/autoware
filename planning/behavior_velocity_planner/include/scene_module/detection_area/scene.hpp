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

#ifndef SCENE_MODULE__DETECTION_AREA__SCENE_HPP_
#define SCENE_MODULE__DETECTION_AREA__SCENE_HPP_

#include <boost/optional.hpp>

#include <memory>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <utilization/boost_geometry_helper.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/LinearMath/Transform.h>

namespace behavior_velocity_planner
{
using PathIndexWithPose = std::pair<size_t, geometry_msgs::msg::Pose>;  // front index, pose
using PathIndexWithPoint2d = std::pair<size_t, Point2d>;                // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;                  // front index, offset
using autoware_auto_planning_msgs::msg::PathWithLaneId;

class DetectionAreaModule : public SceneModuleInterface
{
public:
  enum class State { GO, STOP };

  struct DebugData
  {
    double base_link2front;
    std::vector<geometry_msgs::msg::Pose> stop_poses;
    std::vector<geometry_msgs::msg::Pose> dead_line_poses;
    geometry_msgs::msg::Pose first_stop_pose;
    std::vector<geometry_msgs::msg::Point> obstacle_points;
  };

  struct PlannerParam
  {
    double stop_margin;
    bool use_dead_line;
    double dead_line_margin;
    bool use_pass_judge_line;
    double state_clear_time;
    double hold_stop_margin_distance;
  };

public:
  DetectionAreaModule(
    const int64_t module_id, const int64_t lane_id,
    const lanelet::autoware::DetectionArea & detection_area_reg_elem,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() override;

private:
  LineString2d getStopLineGeometry2d() const;

  std::vector<geometry_msgs::msg::Point> getObstaclePoints() const;

  bool canClearStopState() const;

  bool hasEnoughBrakingDistance(
    const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose) const;

  // Lane id
  int64_t lane_id_;

  // Key Feature
  const lanelet::autoware::DetectionArea & detection_area_reg_elem_;

  // State
  State state_;
  std::shared_ptr<const rclcpp::Time> last_obstacle_found_time_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__DETECTION_AREA__SCENE_HPP_
