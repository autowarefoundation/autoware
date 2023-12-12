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

#ifndef BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_PLANNER_MODULE_HPP_
#define BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_PLANNER_MODULE_HPP_

#include "behavior_path_goal_planner_module/default_fixed_goal_planner.hpp"
#include "behavior_path_goal_planner_module/freespace_pull_over.hpp"
#include "behavior_path_goal_planner_module/geometric_pull_over.hpp"
#include "behavior_path_goal_planner_module/goal_planner_parameters.hpp"
#include "behavior_path_goal_planner_module/goal_searcher.hpp"
#include "behavior_path_goal_planner_module/shift_pull_over.hpp"
#include "behavior_path_planner/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp"
#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "behavior_path_planner_common/utils/parking_departure/common_module_data.hpp"
#include "behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"

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

#define DEFINE_SETTER_WITH_MUTEX(TYPE, NAME)                  \
public:                                                       \
  void set_##NAME(const TYPE & value)                         \
  {                                                           \
    const std::lock_guard<std::recursive_mutex> lock(mutex_); \
    NAME##_ = value;                                          \
  }

#define DEFINE_GETTER_WITH_MUTEX(TYPE, NAME)                  \
public:                                                       \
  TYPE get_##NAME() const                                     \
  {                                                           \
    const std::lock_guard<std::recursive_mutex> lock(mutex_); \
    return NAME##_;                                           \
  }

#define DEFINE_SETTER_GETTER_WITH_MUTEX(TYPE, NAME) \
  DEFINE_SETTER_WITH_MUTEX(TYPE, NAME)              \
  DEFINE_GETTER_WITH_MUTEX(TYPE, NAME)

class ThreadSafeData
{
public:
  ThreadSafeData(std::recursive_mutex & mutex, rclcpp::Clock::SharedPtr clock)
  : mutex_(mutex), clock_(clock)
  {
  }

  bool incrementPathIndex()
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!pull_over_path_) {
      return false;
    }

    if (pull_over_path_->incrementPathIndex()) {
      last_path_idx_increment_time_ = clock_->now();
      return true;
    }
    return false;
  }

  void set_pull_over_path(const PullOverPath & path)
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    pull_over_path_ = std::make_shared<PullOverPath>(path);
    if (path.type != PullOverPlannerType::NONE && path.type != PullOverPlannerType::FREESPACE) {
      lane_parking_pull_over_path_ = std::make_shared<PullOverPath>(path);
    }

    last_path_update_time_ = clock_->now();
  }

  void set_pull_over_path(const std::shared_ptr<PullOverPath> & path)
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    pull_over_path_ = path;
    if (path->type != PullOverPlannerType::NONE && path->type != PullOverPlannerType::FREESPACE) {
      lane_parking_pull_over_path_ = path;
    }
    last_path_update_time_ = clock_->now();
  }

  template <typename... Args>
  void set(Args... args)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    (..., set(args));
  }
  void set(const GoalCandidates & arg) { set_goal_candidates(arg); }
  void set(const std::vector<PullOverPath> & arg) { set_pull_over_path_candidates(arg); }
  void set(const std::shared_ptr<PullOverPath> & arg) { set_pull_over_path(arg); }
  void set(const PullOverPath & arg) { set_pull_over_path(arg); }
  void set(const GoalCandidate & arg) { set_modified_goal_pose(arg); }

  void clearPullOverPath()
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    pull_over_path_ = nullptr;
  }

  bool foundPullOverPath() const
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!pull_over_path_) {
      return false;
    }

    return pull_over_path_->isValidPath();
  }

  PullOverPlannerType getPullOverPlannerType() const
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!pull_over_path_) {
      return PullOverPlannerType::NONE;
    }

    return pull_over_path_->type;
  };

  void reset()
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    pull_over_path_ = nullptr;
    pull_over_path_candidates_.clear();
    goal_candidates_.clear();
    modified_goal_pose_ = std::nullopt;
    last_path_update_time_ = std::nullopt;
    last_path_idx_increment_time_ = std::nullopt;
    closest_start_pose_ = std::nullopt;
  }

  DEFINE_GETTER_WITH_MUTEX(std::shared_ptr<PullOverPath>, pull_over_path)
  DEFINE_GETTER_WITH_MUTEX(std::shared_ptr<PullOverPath>, lane_parking_pull_over_path)
  DEFINE_GETTER_WITH_MUTEX(std::optional<rclcpp::Time>, last_path_update_time)
  DEFINE_GETTER_WITH_MUTEX(std::optional<rclcpp::Time>, last_path_idx_increment_time)

  DEFINE_SETTER_GETTER_WITH_MUTEX(std::vector<PullOverPath>, pull_over_path_candidates)
  DEFINE_SETTER_GETTER_WITH_MUTEX(GoalCandidates, goal_candidates)
  DEFINE_SETTER_GETTER_WITH_MUTEX(std::optional<GoalCandidate>, modified_goal_pose)
  DEFINE_SETTER_GETTER_WITH_MUTEX(std::optional<Pose>, closest_start_pose)

private:
  std::shared_ptr<PullOverPath> pull_over_path_{nullptr};
  std::shared_ptr<PullOverPath> lane_parking_pull_over_path_{nullptr};
  std::vector<PullOverPath> pull_over_path_candidates_;
  GoalCandidates goal_candidates_{};
  std::optional<GoalCandidate> modified_goal_pose_;
  std::optional<rclcpp::Time> last_path_update_time_;
  std::optional<rclcpp::Time> last_path_idx_increment_time_;
  std::optional<Pose> closest_start_pose_{};

  std::recursive_mutex & mutex_;
  rclcpp::Clock::SharedPtr clock_;
};

#undef DEFINE_SETTER_WITH_MUTEX
#undef DEFINE_GETTER_WITH_MUTEX
#undef DEFINE_SETTER_GETTER_WITH_MUTEX

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

struct LastApprovalData
{
  LastApprovalData(rclcpp::Time time, Pose pose) : time(time), pose(pose) {}

  rclcpp::Time time{};
  Pose pose{};
};

struct PreviousPullOverData
{
  struct SafetyStatus
  {
    std::optional<rclcpp::Time> safe_start_time{};
    bool is_safe{false};
  };

  void reset()
  {
    stop_path = nullptr;
    stop_path_after_approval = nullptr;
    found_path = false;
    safety_status = SafetyStatus{};
    has_decided_path = false;
  }

  std::shared_ptr<PathWithLaneId> stop_path{nullptr};
  std::shared_ptr<PathWithLaneId> stop_path_after_approval{nullptr};
  bool found_path{false};
  SafetyStatus safety_status{};
  bool has_decided_path{false};
};

class GoalPlannerModule : public SceneModuleInterface
{
public:
  GoalPlannerModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<GoalPlannerParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map);

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

  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  void processOnExit() override;
  void updateData() override;

  void updateEgoPredictedPathParams(
    std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
    const std::shared_ptr<GoalPlannerParameters> & goal_planner_params);

  void updateSafetyCheckParams(
    std::shared_ptr<SafetyCheckParams> & safety_check_params,
    const std::shared_ptr<GoalPlannerParameters> & goal_planner_params);

  void updateObjectsFilteringParams(
    std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
    const std::shared_ptr<GoalPlannerParameters> & goal_planner_params);

  void postProcess() override;
  void setParameters(const std::shared_ptr<GoalPlannerParameters> & parameters);
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }
  CandidateOutput planCandidate() const override { return CandidateOutput{}; }

private:
  /*
   * state transitions and plan function used in each state
   *
   * +--------------------------+
   * | RUNNING                  |
   * | planPullOverAsCandidate()|
   * +------------+-------------+
   *              | hasDecidedPath()
   *  2           v
   * +--------------------------+
   * | WAITING_APPROVAL         |
   * | planPullOverAsCandidate()|
   * +------------+-------------+
   *              | isActivated()
   *  3           v
   * +--------------------------+
   * | RUNNING                  |
   * | planPullOverAsOutput()   |
   * +--------------------------+
   */

  // The start_planner activates when it receives a new route,
  // so there is no need to terminate the goal planner.
  // If terminating it, it may switch to lane following and could generate an inappropriate path.
  bool canTransitSuccessState() override { return false; }
  bool canTransitFailureState() override { return false; }
  bool canTransitIdleToRunningState() override { return true; }

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
  ThreadSafeData thread_safe_data_;

  std::unique_ptr<LastApprovalData> last_approval_data_{nullptr};
  PreviousPullOverData prev_data_{nullptr};

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
  bool checkOccupancyGridCollision(const PathWithLaneId & path) const;
  bool checkObjectsCollision(
    const PathWithLaneId & path, const double collision_check_margin,
    const bool update_debug_data = false) const;

  // goal seach
  Pose calcRefinedGoal(const Pose & goal_pose) const;
  GoalCandidates generateGoalCandidates() const;

  // stop or decelerate
  void deceleratePath(PullOverPath & pull_over_path) const;
  void decelerateForTurnSignal(const Pose & stop_pose, PathWithLaneId & path) const;
  void decelerateBeforeSearchStart(
    const Pose & search_start_offset_pose, PathWithLaneId & path) const;
  PathWithLaneId generateStopPath() const;
  PathWithLaneId generateFeasibleStopPath() const;

  void keepStoppedWithCurrentPath(PathWithLaneId & path) const;
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
  bool foundPullOverPath() const;
  void updateStatus(const BehaviorModuleOutput & output);

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
  bool canReturnToLaneParking();

  // plan pull over path
  BehaviorModuleOutput planPullOver();
  BehaviorModuleOutput planPullOverAsOutput();
  BehaviorModuleOutput planPullOverAsCandidate();
  std::optional<std::pair<PullOverPath, GoalCandidate>> selectPullOverPath(
    const std::vector<PullOverPath> & pull_over_path_candidates,
    const GoalCandidates & goal_candidates, const double collision_check_margin) const;
  std::vector<PullOverPath> sortPullOverPathCandidatesByGoalPriority(
    const std::vector<PullOverPath> & pull_over_path_candidates,
    const GoalCandidates & goal_candidates) const;

  // lanes and drivable area
  std::vector<DrivableLanes> generateDrivableLanes() const;
  void setDrivableAreaInfo(BehaviorModuleOutput & output) const;

  // output setter
  void setOutput(BehaviorModuleOutput & output) const;
  void setStopPath(BehaviorModuleOutput & output) const;
  void updatePreviousData(const BehaviorModuleOutput & output);

  /**
   * @brief Sets a stop path in the current path based on safety conditions and previous paths.
   *
   * This function sets a stop path in the current path. Depending on whether the previous safety
   * judgement against dynamic objects were safe or if a previous stop path existed, it either
   * generates a new stop path or uses the previous stop path.
   *
   * @param output BehaviorModuleOutput
   */
  void setStopPathFromCurrentPath(BehaviorModuleOutput & output) const;
  void setModifiedGoal(BehaviorModuleOutput & output) const;
  void setTurnSignalInfo(BehaviorModuleOutput & output) const;

  // new turn signal
  TurnSignalInfo calcTurnSignalInfo() const;

  // timer for generating pull over path candidates in a separate thread
  void onTimer();
  void onFreespaceParkingTimer();

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
  /**
   * @brief Checks if the current path is safe.
   * @return std::pair<bool, bool>
   *         first: If the path is safe for a certain period of time, true.
   *         second: If the path is safe in the current state, true.
   */
  std::pair<bool, bool> isSafePath() const;

  // debug
  void setDebugData();
  void printParkingPositionError() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_PLANNER_MODULE_HPP_
