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

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/util/geometric_parallel_parking/geometric_parallel_parking.hpp"
#include "behavior_path_planner/util/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp"
#include "behavior_path_planner/util/pull_over/freespace_pull_over.hpp"
#include "behavior_path_planner/util/pull_over/geometric_pull_over.hpp"
#include "behavior_path_planner/util/pull_over/goal_searcher.hpp"
#include "behavior_path_planner/util/pull_over/pull_over_parameters.hpp"
#include "behavior_path_planner/util/pull_over/shift_pull_over.hpp"

#include <freespace_planning_algorithms/astar_search.hpp>
#include <freespace_planning_algorithms/rrtstar.hpp>
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

using freespace_planning_algorithms::AbstractPlanningAlgorithm;
using freespace_planning_algorithms::AstarParam;
using freespace_planning_algorithms::AstarSearch;
using freespace_planning_algorithms::PlannerCommonParam;
using freespace_planning_algorithms::RRTStar;
using freespace_planning_algorithms::RRTStarParam;

enum PathType {
  NONE = 0,
  SHIFT,
  ARC_FORWARD,
  ARC_BACKWARD,
  FREESPACE,
};

struct PUllOverStatus
{
  std::shared_ptr<PullOverPath> pull_over_path{};
  std::shared_ptr<PullOverPath> lane_parking_pull_over_path{};
  size_t current_path_idx{0};
  bool require_increment_{true};  // if false, keep current path idx.
  std::shared_ptr<PathWithLaneId> prev_stop_path{nullptr};
  lanelet::ConstLanelets current_lanes{};
  lanelet::ConstLanelets pull_over_lanes{};
  std::vector<DrivableLanes> lanes{};  // current + pull_over
  bool has_decided_path{false};
  bool is_safe{false};
  bool prev_is_safe{false};
  bool has_decided_velocity{false};
  bool has_requested_approval{false};
  std::optional<Pose> stop_pose{};
  bool is_ready{false};
  bool during_freespace_parking{false};
};

class PullOverModule : public SceneModuleInterface
{
public:
#ifdef USE_OLD_ARCHITECTURE
  PullOverModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<PullOverParameters> & parameters);
#else
  PullOverModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<PullOverParameters> & parameters,
    const std::shared_ptr<RTCInterface> & rtc_interface);

  void updateModuleParams(const std::shared_ptr<PullOverParameters> & parameters)
  {
    parameters_ = parameters;
  }
#endif

  BehaviorModuleOutput run() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  ModuleStatus updateState() override;
  ModuleStatus getNodeStatusWhileWaitingApproval() const override { return ModuleStatus::SUCCESS; }
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const std::shared_ptr<PullOverParameters> & parameters);

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  std::shared_ptr<PullOverParameters> parameters_;

  std::vector<std::shared_ptr<PullOverPlannerBase>> pull_over_planners_;
  std::unique_ptr<PullOverPlannerBase> freespace_planner_;
  std::shared_ptr<GoalSearcherBase> goal_searcher_;
  PullOverPath shift_parking_path_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;

  PUllOverStatus status_;
  std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map_;
  std::optional<GoalCandidate> modified_goal_pose_;
  std::optional<size_t> prev_goal_id_;
  Pose refined_goal_pose_;
  GoalCandidates goal_candidates_;
  std::vector<PullOverPath> pull_over_path_candidates_;
  std::optional<Pose> closest_start_pose_;
  GeometricParallelParking parallel_parking_planner_;
  ParallelParkingParameters parallel_parking_parameters_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_stopped_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_stuck_;

  tier4_autoware_utils::LinearRing2d vehicle_footprint_;
  std::unique_ptr<rclcpp::Time> last_received_time_;
  std::unique_ptr<rclcpp::Time> last_approved_time_;
  std::unique_ptr<rclcpp::Time> last_increment_time_;
  std::unique_ptr<Pose> last_approved_pose_;

  bool incrementPathIndex();
  PathWithLaneId getCurrentPath() const;
  Pose calcRefinedGoal(const Pose & goal_pose) const;
  ParallelParkingParameters getGeometricPullOverParameters() const;
  std::pair<double, double> calcDistanceToPathChange() const;
  PathWithLaneId generateStopPath();
  PathWithLaneId generateEmergencyStopPath();
  void keepStoppedWithCurrentPath(PathWithLaneId & path);

  bool isStopped();
  bool isStopped(
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> & odometry_buffer, const double time);
  bool hasFinishedCurrentPath();
  bool hasFinishedPullOver();
  void updateOccupancyGrid();
  void resetStatus();

  bool checkCollision(const PathWithLaneId & path) const;
  bool hasEnoughDistance(const PullOverPath & pull_over_path) const;

  TurnSignalInfo calcTurnSignalInfo() const;

  bool planFreespacePath();
  bool returnToLaneParking();
  bool isStuck();

  // timer for generating pull over path candidates
  void onTimer();
  rclcpp::TimerBase::SharedPtr lane_parking_timer_;
  rclcpp::CallbackGroup::SharedPtr lane_parking_timer_cb_group_;
  std::mutex mutex_;

  // debug
  void setDebugData();
  void printParkingPositionError() const;

  void onFreespaceParkingTimer();
  rclcpp::TimerBase::SharedPtr freespace_parking_timer_;
  rclcpp::CallbackGroup::SharedPtr freespace_parking_timer_cb_group_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_MODULE_HPP_
