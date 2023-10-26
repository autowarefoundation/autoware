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
#include "behavior_path_planner/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "behavior_path_planner/utils/start_goal_planner_common/common_module_data.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <freespace_planning_algorithms/astar_search.hpp>
#include <freespace_planning_algorithms/rrtstar.hpp>
#include <lane_departure_checker/lane_departure_checker.hpp>
#include <motion_utils/distance/distance.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

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

using behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams;
using behavior_path_planner::utils::path_safety_checker::ObjectsFilteringParams;
using behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using behavior_path_planner::utils::path_safety_checker::SafetyCheckParams;
using behavior_path_planner::utils::path_safety_checker::TargetObjectsOnLane;
using tier4_autoware_utils::Polygon2d;

enum class PathType {
  NONE = 0,
  SHIFT,
  ARC_FORWARD,
  ARC_BACKWARD,
  FREESPACE,
};

#define DEFINE_SETTER_GETTER(TYPE, NAME)                      \
public:                                                       \
  void set_##NAME(const TYPE & value)                         \
  {                                                           \
    const std::lock_guard<std::recursive_mutex> lock(mutex_); \
    NAME##_ = value;                                          \
  }                                                           \
                                                              \
  TYPE get_##NAME() const                                     \
  {                                                           \
    const std::lock_guard<std::recursive_mutex> lock(mutex_); \
    return NAME##_;                                           \
  }

class PullOverStatus
{
public:
  explicit PullOverStatus(std::recursive_mutex & mutex) : mutex_(mutex) {}

  // Reset all data members to their initial states
  void reset()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    pull_over_path_ = nullptr;
    lane_parking_pull_over_path_ = nullptr;
    current_path_idx_ = 0;
    require_increment_ = true;
    prev_stop_path_ = nullptr;
    prev_stop_path_after_approval_ = nullptr;
    current_lanes_.clear();
    pull_over_lanes_.clear();
    lanes_.clear();
    has_decided_path_ = false;
    is_safe_static_objects_ = false;
    is_safe_dynamic_objects_ = false;
    prev_is_safe_ = false;
    prev_is_safe_dynamic_objects_ = false;
    has_decided_velocity_ = false;
    has_requested_approval_ = false;
    is_ready_ = false;
  }

  DEFINE_SETTER_GETTER(std::shared_ptr<PullOverPath>, pull_over_path)
  DEFINE_SETTER_GETTER(std::shared_ptr<PullOverPath>, lane_parking_pull_over_path)
  DEFINE_SETTER_GETTER(size_t, current_path_idx)
  DEFINE_SETTER_GETTER(bool, require_increment)
  DEFINE_SETTER_GETTER(std::shared_ptr<PathWithLaneId>, prev_stop_path)
  DEFINE_SETTER_GETTER(std::shared_ptr<PathWithLaneId>, prev_stop_path_after_approval)
  DEFINE_SETTER_GETTER(lanelet::ConstLanelets, current_lanes)
  DEFINE_SETTER_GETTER(lanelet::ConstLanelets, pull_over_lanes)
  DEFINE_SETTER_GETTER(std::vector<DrivableLanes>, lanes)
  DEFINE_SETTER_GETTER(bool, has_decided_path)
  DEFINE_SETTER_GETTER(bool, is_safe_static_objects)
  DEFINE_SETTER_GETTER(bool, is_safe_dynamic_objects)
  DEFINE_SETTER_GETTER(bool, prev_is_safe)
  DEFINE_SETTER_GETTER(bool, prev_is_safe_dynamic_objects)
  DEFINE_SETTER_GETTER(bool, has_decided_velocity)
  DEFINE_SETTER_GETTER(bool, has_requested_approval)
  DEFINE_SETTER_GETTER(bool, is_ready)
  DEFINE_SETTER_GETTER(std::shared_ptr<rclcpp::Time>, last_approved_time)
  DEFINE_SETTER_GETTER(std::shared_ptr<rclcpp::Time>, last_increment_time)
  DEFINE_SETTER_GETTER(std::shared_ptr<rclcpp::Time>, last_path_update_time)
  DEFINE_SETTER_GETTER(std::shared_ptr<Pose>, last_approved_pose)
  DEFINE_SETTER_GETTER(std::optional<GoalCandidate>, modified_goal_pose)
  DEFINE_SETTER_GETTER(Pose, refined_goal_pose)
  DEFINE_SETTER_GETTER(GoalCandidates, goal_candidates)
  DEFINE_SETTER_GETTER(std::vector<PullOverPath>, pull_over_path_candidates)
  DEFINE_SETTER_GETTER(std::optional<Pose>, closest_start_pose)

private:
  std::shared_ptr<PullOverPath> pull_over_path_{nullptr};
  std::shared_ptr<PullOverPath> lane_parking_pull_over_path_{nullptr};
  size_t current_path_idx_{0};
  bool require_increment_{true};
  std::shared_ptr<PathWithLaneId> prev_stop_path_{nullptr};
  std::shared_ptr<PathWithLaneId> prev_stop_path_after_approval_{nullptr};
  lanelet::ConstLanelets current_lanes_{};
  lanelet::ConstLanelets pull_over_lanes_{};
  std::vector<DrivableLanes> lanes_{};
  bool has_decided_path_{false};
  bool is_safe_static_objects_{false};
  bool is_safe_dynamic_objects_{false};
  bool prev_is_safe_{false};
  bool prev_is_safe_dynamic_objects_{false};
  bool has_decided_velocity_{false};
  bool has_requested_approval_{false};
  bool is_ready_{false};

  // save last time and pose
  std::shared_ptr<rclcpp::Time> last_approved_time_;
  std::shared_ptr<rclcpp::Time> last_increment_time_;
  std::shared_ptr<rclcpp::Time> last_path_update_time_;
  std::shared_ptr<Pose> last_approved_pose_;

  // goal modification
  std::optional<GoalCandidate> modified_goal_pose_;
  Pose refined_goal_pose_{};
  GoalCandidates goal_candidates_{};

  //  pull over path
  std::vector<PullOverPath> pull_over_path_candidates_;
  std::optional<Pose> closest_start_pose_;

  std::recursive_mutex & mutex_;
};

#undef DEFINE_SETTER_GETTER

struct FreespacePlannerDebugData
{
  bool is_planning{false};
  size_t current_goal_idx{0};
  size_t num_goal_candidates{0};
};

struct GoalPlannerDebugData
{
  FreespacePlannerDebugData freespace_planner{};
  std::vector<Polygon2d> ego_polygons_expanded{};
};

class GoalPlannerModule : public SceneModuleInterface
{
public:
  GoalPlannerModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<GoalPlannerParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map);

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<GoalPlannerParameters>>(parameters);
    if (parameters_->safety_check_params.enable_safety_check) {
      ego_predicted_path_params_ =
        std::make_shared<EgoPredictedPathParams>(parameters_->ego_predicted_path_params);
      objects_filtering_params_ =
        std::make_shared<ObjectsFilteringParams>(parameters_->objects_filtering_params);
      safety_check_params_ = std::make_shared<SafetyCheckParams>(parameters_->safety_check_params);
    }
  }

  // TODO(someone): remove this, and use base class function
  [[deprecated]] BehaviorModuleOutput run() override;
  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  // TODO(someone): remove this, and use base class function
  [[deprecated]] ModuleStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  void processOnExit() override;
  void updateData() override;
  void setParameters(const std::shared_ptr<GoalPlannerParameters> & parameters);
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  // not used, but need to override
  CandidateOutput planCandidate() const override { return CandidateOutput{}; };

private:
  bool canTransitSuccessState() override { return false; }

  bool canTransitFailureState() override { return false; }

  bool canTransitIdleToRunningState() override { return false; }

  mutable StartGoalPlannerData goal_planner_data_;

  std::shared_ptr<GoalPlannerParameters> parameters_;

  mutable std::shared_ptr<EgoPredictedPathParams> ego_predicted_path_params_;
  mutable std::shared_ptr<ObjectsFilteringParams> objects_filtering_params_;
  mutable std::shared_ptr<SafetyCheckParams> safety_check_params_;

  vehicle_info_util::VehicleInfo vehicle_info_{};

  // planner
  std::vector<std::shared_ptr<PullOverPlannerBase>> pull_over_planners_;
  std::unique_ptr<PullOverPlannerBase> freespace_planner_;
  std::unique_ptr<FixedGoalPlannerBase> fixed_goal_planner_;

  // goal searcher
  std::shared_ptr<GoalSearcherBase> goal_searcher_;

  // collision detector
  // need to be shared_ptr to be used in planner and goal searcher
  std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map_;

  // check stopped and stuck state
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_stopped_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_stuck_;

  tier4_autoware_utils::LinearRing2d vehicle_footprint_;

  std::recursive_mutex mutex_;
  PullOverStatus status_;

  // approximate distance from the start point to the end point of pull_over.
  // this is used as an assumed value to decelerate, etc., before generating the actual path.
  const double approximate_pull_over_distance_{20.0};
  // ego may exceed the stop distance, so add a buffer
  const double stop_distance_buffer_{2.0};

  // for parking policy
  bool left_side_parking_{true};

  // pre-generate lane parking paths in a separate thread
  rclcpp::TimerBase::SharedPtr lane_parking_timer_;
  rclcpp::CallbackGroup::SharedPtr lane_parking_timer_cb_group_;

  // generate freespace parking paths in a separate thread
  rclcpp::TimerBase::SharedPtr freespace_parking_timer_;
  rclcpp::CallbackGroup::SharedPtr freespace_parking_timer_cb_group_;

  // debug
  mutable GoalPlannerDebugData debug_data_;

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

  void keepStoppedWithCurrentPath(PathWithLaneId & path);
  double calcSignedArcLengthFromEgo(const PathWithLaneId & path, const Pose & pose) const;

  // status
  bool isStopped();
  bool isStopped(
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> & odometry_buffer, const double time);
  bool hasFinishedCurrentPath();
  bool isOnModifiedGoal() const;
  double calcModuleRequestLength() const;
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
  std::vector<PullOverPath> sortPullOverPathCandidatesByGoalPriority(
    const std::vector<PullOverPath> & pull_over_path_candidates,
    const GoalCandidates & goal_candidates) const;

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

  /**
   * @brief Sets a stop path in the current path based on safety conditions and previous paths.
   *
   * This function sets a stop path in the current path. Depending on whether the previous safety
   * judgement against dynamic objects were safe or if a previous stop path existed, it either
   * generates a new stop path or uses the previous stop path.
   *
   * @param output BehaviorModuleOutput
   */
  void setStopPathFromCurrentPath(BehaviorModuleOutput & output);
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

  // safety check
  void initializeSafetyCheckParameters();
  SafetyCheckParams createSafetyCheckParams() const;
  void updateSafetyCheckTargetObjectsData(
    const PredictedObjects & filtered_objects, const TargetObjectsOnLane & target_objects_on_lane,
    const std::vector<PoseWithVelocityStamped> & ego_predicted_path) const;
  bool isSafePath() const;

  // debug
  void setDebugData();
  void printParkingPositionError() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__GOAL_PLANNER__GOAL_PLANNER_MODULE_HPP_
