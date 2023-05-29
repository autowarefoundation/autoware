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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__GOAL_PLANNER__GOAL_PLANNER_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__GOAL_PLANNER__GOAL_PLANNER_MODULE_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/utils/geometric_parallel_parking/geometric_parallel_parking.hpp"
#include "behavior_path_planner/utils/goal_planner/default_fixed_goal_planner.hpp"
#include "behavior_path_planner/utils/goal_planner/freespace_pull_over.hpp"
#include "behavior_path_planner/utils/goal_planner/geometric_pull_over.hpp"
#include "behavior_path_planner/utils/goal_planner/goal_planner_parameters.hpp"
#include "behavior_path_planner/utils/goal_planner/goal_searcher.hpp"
#include "behavior_path_planner/utils/goal_planner/shift_pull_over.hpp"
#include "behavior_path_planner/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp"

#include <freespace_planning_algorithms/astar_search.hpp>
#include <freespace_planning_algorithms/rrtstar.hpp>
#include <lane_departure_checker/lane_departure_checker.hpp>
#include <motion_utils/distance/distance.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>

#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
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

enum class PathType {
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
};

class GoalPlannerModule : public SceneModuleInterface
{
public:
#ifdef USE_OLD_ARCHITECTURE
  GoalPlannerModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<GoalPlannerParameters> & parameters);
#else
  GoalPlannerModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<GoalPlannerParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map);

  void updateModuleParams(const std::shared_ptr<GoalPlannerParameters> & parameters)
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
  void processOnEntry() override;
  void processOnExit() override;
  void setParameters(const std::shared_ptr<GoalPlannerParameters> & parameters);
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  // not used, but need to override
  CandidateOutput planCandidate() const override { return CandidateOutput{}; };

private:
  PUllOverStatus status_;

  std::shared_ptr<GoalPlannerParameters> parameters_;

  // planner
  std::vector<std::shared_ptr<PullOverPlannerBase>> pull_over_planners_;
  std::unique_ptr<PullOverPlannerBase> freespace_planner_;
  std::unique_ptr<FixedGoalPlannerBase> fixed_goal_planner_;

  // goal searcher
  std::shared_ptr<GoalSearcherBase> goal_searcher_;
  std::optional<GoalCandidate> modified_goal_pose_;
  std::optional<size_t> prev_goal_id_;
  Pose refined_goal_pose_;
  GoalCandidates goal_candidates_;

  // collision detector
  // need to be shared_ptr to be used in planner and goal searcher
  std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map_;

  //  pull over path
  std::vector<PullOverPath> pull_over_path_candidates_;
  std::optional<Pose> closest_start_pose_;

  // check stopped and stuck state
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_stopped_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_stuck_;

  tier4_autoware_utils::LinearRing2d vehicle_footprint_;

  // save last time and pose
  std::unique_ptr<rclcpp::Time> last_received_time_;
  std::unique_ptr<rclcpp::Time> last_approved_time_;
  std::unique_ptr<rclcpp::Time> last_increment_time_;
  std::unique_ptr<rclcpp::Time> last_path_update_time_;
  std::unique_ptr<Pose> last_approved_pose_;

  // approximate distance from the start point to the end point of pull_over.
  // this is used as an assumed value to decelerate, etc., before generating the actual path.
  const double approximate_pull_over_distance_{20.0};
  // ego may exceed the stop distance, so add a buffer
  const double stop_distance_buffer_{2.0};

  // for parking policy
  bool left_side_parking_{true};
  mutable bool allow_goal_modification_{false};  // need to be set in isExecutionRequested

  // pre-generate lane parking paths in a separate thread
  rclcpp::TimerBase::SharedPtr lane_parking_timer_;
  rclcpp::CallbackGroup::SharedPtr lane_parking_timer_cb_group_;
  std::mutex mutex_;

  // generate freespace parking paths in a separate thread
  rclcpp::TimerBase::SharedPtr freespace_parking_timer_;
  rclcpp::CallbackGroup::SharedPtr freespace_parking_timer_cb_group_;

  // collision check
  void initializeOccupancyGridMap();
  void updateOccupancyGrid();
  bool checkCollision(const PathWithLaneId & path) const;

  // goal seach
  Pose calcRefinedGoal(const Pose & goal_pose) const;
  void generateGoalCandidates();

  // stop or decelerate
  void decelerateForTurnSignal(const Pose & stop_pose, PathWithLaneId & path) const;
  void decelerateBeforeSearchStart(
    const Pose & search_start_offset_pose, PathWithLaneId & path) const;
  PathWithLaneId generateStopPath();
  PathWithLaneId generateFeasibleStopPath();
  boost::optional<double> calcFeasibleDecelDistance(const double target_velocity) const;
  void keepStoppedWithCurrentPath(PathWithLaneId & path);
  double calcSignedArcLengthFromEgo(const PathWithLaneId & path, const Pose & pose) const;

  // status
  bool isStopped();
  bool isStopped(
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> & odometry_buffer, const double time);
  bool hasFinishedCurrentPath();
  bool hasFinishedGoalPlanner();
  bool isOnGoal() const;
  double calcModuleRequestLength() const;
  void resetStatus();
  bool needPathUpdate(const double path_update_duration) const;
  bool isStuck();
  bool hasDecidedPath() const;
  void decideVelocity();

  // validation
  bool hasEnoughDistance(const PullOverPath & pull_over_path) const;
  bool isCrossingPossible(
    const lanelet::ConstLanelet & start_lane, const lanelet::ConstLanelet & end_lane) const;
  bool isCrossingPossible(
    const Pose & start_pose, const Pose & end_pose, const lanelet::ConstLanelets lanes) const;
  bool isCrossingPossible(const PullOverPath & pull_over_path) const;
  bool hasEnoughTimePassedSincePathUpdate(const double duration) const;

  // freespace parking
  bool planFreespacePath();
  void returnToLaneParking();

  // plan pull over path
  BehaviorModuleOutput planWithGoalModification();
  BehaviorModuleOutput planWaitingApprovalWithGoalModification();
  void selectSafePullOverPath();

  // deal with pull over partial paths
  PathWithLaneId getCurrentPath() const;
  bool incrementPathIndex();
  void transitionToNextPathIfFinishingCurrentPath();

  // lanes and drivable area
  void setLanes();
  void setDrivableAreaInfo(BehaviorModuleOutput & output) const;

  // output setter
  void setOutput(BehaviorModuleOutput & output);
  void setStopPath(BehaviorModuleOutput & output);
  void setModifiedGoal(BehaviorModuleOutput & output) const;
  void setTurnSignalInfo(BehaviorModuleOutput & output) const;

  // new turn signal
  TurnSignalInfo calcTurnSignalInfo() const;

  // timer for generating pull over path candidates in a separate thread
  void onTimer();
  void onFreespaceParkingTimer();

  // flag for the interface which do not support `allow_goal_modification`
  // when the goal is in `road_shoulder`, always allow goal modification.
  bool checkOriginalGoalIsInShoulder() const;

  // steering factor
  void updateSteeringFactor(
    const std::array<Pose, 2> & pose, const std::array<double, 2> distance, const uint16_t type);

  // rtc
  std::pair<double, double> calcDistanceToPathChange() const;

  // debug
  void setDebugData();
  void printParkingPositionError() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__GOAL_PLANNER__GOAL_PLANNER_MODULE_HPP_
