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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_MODULE_HPP_

#include "behavior_path_planner/scene_module/pull_over/geometric_pull_over.hpp"
#include "behavior_path_planner/scene_module/pull_over/goal_searcher.hpp"
#include "behavior_path_planner/scene_module/pull_over/pull_over_parameters.hpp"
#include "behavior_path_planner/scene_module/pull_over/shift_pull_over.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/utils/geometric_parallel_parking.hpp"
#include "behavior_path_planner/scene_module/utils/occupancy_grid_based_collision_detector.hpp"

#include <lane_departure_checker/lane_departure_checker.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>

#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using geometry_msgs::msg::PoseArray;
using lane_departure_checker::LaneDepartureChecker;
using nav_msgs::msg::OccupancyGrid;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

enum PathType {
  NONE = 0,
  SHIFT,
  ARC_FORWARD,
  ARC_BACKWARD,
};

struct PUllOverStatus
{
  std::shared_ptr<PullOverPlannerBase> planner{};
  PullOverPath pull_over_path{};
  size_t current_path_idx{0};
  std::shared_ptr<PathWithLaneId> prev_stop_path{nullptr};
  lanelet::ConstLanelets current_lanes{};
  lanelet::ConstLanelets pull_over_lanes{};
  std::vector<DrivableLanes> lanes{};  // current + pull_over
  bool has_decided_path{false};
  bool is_safe{false};
  bool prev_is_safe{false};
  bool has_decided_velocity{false};
  bool has_requested_approval{false};
};

class PullOverModule : public SceneModuleInterface
{
public:
  PullOverModule(
    const std::string & name, rclcpp::Node & node, const PullOverParameters & parameters);

  BehaviorModuleOutput run() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  void onTimer();
  bool planWithEfficientPath();
  bool planWithCloseGoal();
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const PullOverParameters & parameters);

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  PullOverParameters parameters_;

  std::vector<std::shared_ptr<PullOverPlannerBase>> pull_over_planners_;
  std::shared_ptr<GoalSearcherBase> goal_searcher_;

  PullOverPath shift_parking_path_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  const double pull_over_lane_length_{200.0};
  const double check_distance_{100.0};

  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pose_pub_;

  PUllOverStatus status_;
  std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map_;
  Pose modified_goal_pose_;
  Pose refined_goal_pose_;
  std::vector<GoalCandidate> goal_candidates_;
  GeometricParallelParking parallel_parking_planner_;
  ParallelParkingParameters parallel_parking_parameters_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_;
  tier4_autoware_utils::LinearRing2d vehicle_footprint_;
  std::unique_ptr<rclcpp::Time> last_received_time_;
  std::unique_ptr<rclcpp::Time> last_approved_time_;
  std::unique_ptr<Pose> last_approved_pose_;

  void incrementPathIndex();
  PathWithLaneId getCurrentPath() const;
  PathWithLaneId getFullPath() const;
  PathWithLaneId getReferencePath() const;
  PathWithLaneId generateStopPath() const;
  Pose calcRefinedGoal() const;
  ParallelParkingParameters getGeometricPullOverParameters() const;
  bool isLongEnoughToParkingStart(
    const PathWithLaneId & path, const Pose & parking_start_pose) const;
  bool isLongEnough(
    const lanelet::ConstLanelets & lanelets, const Pose & goal_pose, const double buffer = 0) const;
  double calcMinimumShiftPathDistance() const;
  std::pair<double, double> calcDistanceToPathChange() const;

  bool isStopped();
  bool hasFinishedCurrentPath();
  bool hasFinishedPullOver();
  void updateOccupancyGrid();
  void resetStatus();

  TurnSignalInfo calcTurnSignalInfo() const;

  // debug
  void setDebugData();
  void printParkingPositionError() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_MODULE_HPP_
