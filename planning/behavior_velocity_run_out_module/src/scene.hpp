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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include "debug.hpp"
#include "dynamic_obstacle.hpp"
#include "state_machine.hpp"
#include "utils.hpp"

#include <behavior_velocity_planner_common/scene_module_interface.hpp>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using run_out_utils::PlannerParam;
using run_out_utils::PoseWithRange;
using tier4_debug_msgs::msg::Float32Stamped;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;

class RunOutModule : public SceneModuleInterface
{
public:
  RunOutModule(
    const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    std::unique_ptr<DynamicObstacleCreator> dynamic_obstacle_creator,
    const std::shared_ptr<RunOutDebug> & debug_ptr, const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;

  void setPlannerParam(const PlannerParam & planner_param);

private:
  // Parameter
  PlannerParam planner_param_;

  // Variable
  BasicPolygons2d partition_lanelets_;
  std::unique_ptr<DynamicObstacleCreator> dynamic_obstacle_creator_;
  std::shared_ptr<RunOutDebug> debug_ptr_;
  std::unique_ptr<run_out_utils::StateMachine> state_machine_;
  std::shared_ptr<rclcpp::Time> first_detected_time_;

  // Function
  Polygons2d createDetectionAreaPolygon(const PathWithLaneId & smoothed_path) const;

  std::optional<DynamicObstacle> detectCollision(
    const std::vector<DynamicObstacle> & dynamic_obstacles, const PathWithLaneId & path,
    const std::vector<std::pair<int64_t, lanelet::ConstLanelet>> & crosswalk_lanelets);

  float calcCollisionPositionOfVehicleSide(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & base_pose) const;

  std::vector<geometry_msgs::msg::Point> createVehiclePolygon(
    const geometry_msgs::msg::Pose & base_pose) const;

  std::vector<DynamicObstacle> checkCollisionWithObstacles(
    const std::vector<DynamicObstacle> & dynamic_obstacles,
    std::vector<geometry_msgs::msg::Point> poly, const float travel_time,
    const std::vector<std::pair<int64_t, lanelet::ConstLanelet>> & crosswalk_lanelets) const;

  std::optional<DynamicObstacle> findNearestCollisionObstacle(
    const PathWithLaneId & path, const geometry_msgs::msg::Pose & base_pose,
    std::vector<DynamicObstacle> & dynamic_obstacles) const;

  std::optional<geometry_msgs::msg::Pose> calcPredictedObstaclePose(
    const std::vector<PredictedPath> & predicted_paths, const float travel_time,
    const float velocity_mps) const;

  bool checkCollisionWithShape(
    const Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range, const Shape & shape,
    const std::vector<std::pair<int64_t, lanelet::ConstLanelet>> & crosswalk_lanelets,
    std::vector<geometry_msgs::msg::Point> & collision_points) const;

  bool checkCollisionWithCylinder(
    const Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range, const float radius,
    std::vector<geometry_msgs::msg::Point> & collision_points) const;

  bool checkCollisionWithBoundingBox(
    const Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range,
    const geometry_msgs::msg::Vector3 & dimension,
    std::vector<geometry_msgs::msg::Point> & collision_points) const;

  bool checkCollisionWithPolygon() const;

  std::vector<geometry_msgs::msg::Point> createBoundingBoxForRangedPoints(
    const PoseWithRange & pose_with_range, const float x_offset, const float y_offset) const;

  std::optional<geometry_msgs::msg::Pose> calcStopPoint(
    const std::optional<DynamicObstacle> & dynamic_obstacle, const PathWithLaneId & path,
    const geometry_msgs::msg::Pose & current_pose, const float current_vel,
    const float current_acc) const;

  void insertStopPoint(
    const std::optional<geometry_msgs::msg::Pose> stop_point,
    autoware_auto_planning_msgs::msg::PathWithLaneId & path);

  void insertVelocityForState(
    const std::optional<DynamicObstacle> & dynamic_obstacle, const PlannerData planner_data,
    const PlannerParam & planner_param, const PathWithLaneId & smoothed_path,
    PathWithLaneId & output_path);

  void insertStoppingVelocity(
    const std::optional<DynamicObstacle> & dynamic_obstacle,
    const geometry_msgs::msg::Pose & current_pose, const float current_vel, const float current_acc,
    PathWithLaneId & output_path);

  void insertApproachingVelocity(
    const DynamicObstacle & dynamic_obstacle, const geometry_msgs::msg::Pose & current_pose,
    const float approaching_vel, const float approach_margin, PathWithLaneId & output_path);

  void applyMaxJerkLimit(
    const geometry_msgs::msg::Pose & current_pose, const float current_vel, const float current_acc,
    PathWithLaneId & path) const;

  std::vector<DynamicObstacle> excludeObstaclesOutSideOfPartition(
    const std::vector<DynamicObstacle> & dynamic_obstacles, const PathWithLaneId & path,
    const geometry_msgs::msg::Pose & current_pose) const;

  void publishDebugValue(
    const PathWithLaneId & path, const std::vector<DynamicObstacle> extracted_obstacles,
    const std::optional<DynamicObstacle> & dynamic_obstacle,
    const geometry_msgs::msg::Pose & current_pose) const;

  bool isMomentaryDetection();
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_HPP_
