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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_PLANNER_MODULE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_PLANNER_MODULE_HPP_

#include "autoware/behavior_path_goal_planner_module/default_fixed_goal_planner.hpp"
#include "autoware/behavior_path_goal_planner_module/freespace_pull_over.hpp"
#include "autoware/behavior_path_goal_planner_module/geometric_pull_over.hpp"
#include "autoware/behavior_path_goal_planner_module/goal_planner_parameters.hpp"
#include "autoware/behavior_path_goal_planner_module/goal_searcher.hpp"
#include "autoware/behavior_path_goal_planner_module/shift_pull_over.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp"
#include "autoware/behavior_path_planner_common/utils/parking_departure/common_module_data.hpp"
#include "autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <autoware/lane_departure_checker/lane_departure_checker.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <atomic>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::lane_departure_checker::LaneDepartureChecker;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using geometry_msgs::msg::PoseArray;
using nav_msgs::msg::OccupancyGrid;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using autoware::freespace_planning_algorithms::AstarParam;
using autoware::freespace_planning_algorithms::AstarSearch;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::RRTStar;
using autoware::freespace_planning_algorithms::RRTStarParam;

using autoware::behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams;
using autoware::behavior_path_planner::utils::path_safety_checker::ObjectsFilteringParams;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::SafetyCheckParams;
using autoware::behavior_path_planner::utils::path_safety_checker::TargetObjectsOnLane;
using autoware::universe_utils::Polygon2d;

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

enum class DecidingPathStatus {
  NOT_DECIDED,
  DECIDING,
  DECIDED,
};
using DecidingPathStatusWithStamp = std::pair<DecidingPathStatus, rclcpp::Time>;

struct PreviousPullOverData
{
  struct SafetyStatus
  {
    std::optional<rclcpp::Time> safe_start_time{};
    bool is_safe{false};
  };

  void reset()
  {
    found_path = false;
    safety_status = SafetyStatus{};
    deciding_path_status = DecidingPathStatusWithStamp{};
  }

  bool found_path{false};
  SafetyStatus safety_status{};
  DecidingPathStatusWithStamp deciding_path_status{};
};

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
    set_pull_over_path_no_lock(path);
  }

  void set_pull_over_path(const std::shared_ptr<PullOverPath> & path)
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    set_pull_over_path_no_lock(path);
  }

  template <typename... Args>
  void set(Args... args)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    (..., set_no_lock(args));
  }

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
    last_previous_module_output_ = std::nullopt;
    prev_data_.reset();
  }

  DEFINE_GETTER_WITH_MUTEX(std::shared_ptr<PullOverPath>, pull_over_path)
  DEFINE_GETTER_WITH_MUTEX(std::shared_ptr<PullOverPath>, lane_parking_pull_over_path)
  DEFINE_GETTER_WITH_MUTEX(std::optional<rclcpp::Time>, last_path_update_time)
  DEFINE_GETTER_WITH_MUTEX(std::optional<rclcpp::Time>, last_path_idx_increment_time)

  DEFINE_SETTER_GETTER_WITH_MUTEX(std::vector<PullOverPath>, pull_over_path_candidates)
  DEFINE_SETTER_GETTER_WITH_MUTEX(GoalCandidates, goal_candidates)
  DEFINE_SETTER_GETTER_WITH_MUTEX(std::optional<GoalCandidate>, modified_goal_pose)
  DEFINE_SETTER_GETTER_WITH_MUTEX(std::optional<Pose>, closest_start_pose)
  DEFINE_SETTER_GETTER_WITH_MUTEX(std::optional<BehaviorModuleOutput>, last_previous_module_output)
  DEFINE_SETTER_GETTER_WITH_MUTEX(PreviousPullOverData, prev_data)
  DEFINE_SETTER_GETTER_WITH_MUTEX(CollisionCheckDebugMap, collision_check)
  DEFINE_SETTER_GETTER_WITH_MUTEX(PredictedObjects, static_target_objects)
  DEFINE_SETTER_GETTER_WITH_MUTEX(PredictedObjects, dynamic_target_objects)

private:
  void set_pull_over_path_no_lock(const PullOverPath & path)
  {
    pull_over_path_ = std::make_shared<PullOverPath>(path);
    if (path.type != PullOverPlannerType::NONE && path.type != PullOverPlannerType::FREESPACE) {
      lane_parking_pull_over_path_ = std::make_shared<PullOverPath>(path);
    }

    last_path_update_time_ = clock_->now();
  }

  void set_pull_over_path_no_lock(const std::shared_ptr<PullOverPath> & path)
  {
    pull_over_path_ = path;
    if (path->type != PullOverPlannerType::NONE && path->type != PullOverPlannerType::FREESPACE) {
      lane_parking_pull_over_path_ = path;
    }
    last_path_update_time_ = clock_->now();
  }

  void set_no_lock(const GoalCandidates & arg) { goal_candidates_ = arg; }
  void set_no_lock(const std::vector<PullOverPath> & arg) { pull_over_path_candidates_ = arg; }
  void set_no_lock(const std::shared_ptr<PullOverPath> & arg) { set_pull_over_path_no_lock(arg); }
  void set_no_lock(const PullOverPath & arg) { set_pull_over_path_no_lock(arg); }
  void set_no_lock(const GoalCandidate & arg) { modified_goal_pose_ = arg; }
  void set_no_lock(const BehaviorModuleOutput & arg) { last_previous_module_output_ = arg; }
  void set_no_lock(const PreviousPullOverData & arg) { prev_data_ = arg; }
  void set_no_lock(const CollisionCheckDebugMap & arg) { collision_check_ = arg; }

  std::shared_ptr<PullOverPath> pull_over_path_{nullptr};
  std::shared_ptr<PullOverPath> lane_parking_pull_over_path_{nullptr};
  std::vector<PullOverPath> pull_over_path_candidates_;
  GoalCandidates goal_candidates_{};
  std::optional<GoalCandidate> modified_goal_pose_;
  std::optional<rclcpp::Time> last_path_update_time_;
  std::optional<rclcpp::Time> last_path_idx_increment_time_;
  std::optional<Pose> closest_start_pose_{};
  std::optional<BehaviorModuleOutput> last_previous_module_output_{};
  PreviousPullOverData prev_data_{};
  CollisionCheckDebugMap collision_check_{};
  PredictedObjects static_target_objects_{};
  PredictedObjects dynamic_target_objects_{};

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
  lanelet::ConstLanelet expanded_pull_over_lane_between_ego{};
};

struct LastApprovalData
{
  LastApprovalData(rclcpp::Time time, Pose pose) : time(time), pose(pose) {}

  rclcpp::Time time{};
  Pose pose{};
};

// store stop_pose_ pointer with reason string
struct PoseWithString
{
  std::optional<Pose> * pose;
  std::string string;

  explicit PoseWithString(std::optional<Pose> * shared_pose) : pose(shared_pose), string("") {}

  void set(const Pose & new_pose, const std::string & new_string)
  {
    *pose = new_pose;
    string = new_string;
  }

  void set(const std::string & new_string) { string = new_string; }

  void clear()
  {
    pose->reset();
    string = "";
  }
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

  ~GoalPlannerModule()
  {
    if (lane_parking_timer_) {
      lane_parking_timer_->cancel();
    }
    if (freespace_parking_timer_) {
      freespace_parking_timer_->cancel();
    }

    while (is_lane_parking_cb_running_.load() || is_freespace_parking_cb_running_.load()) {
      const std::string running_callbacks = std::invoke([&]() {
        if (is_lane_parking_cb_running_ && is_freespace_parking_cb_running_) {
          return "lane parking and freespace parking";
        }
        if (is_lane_parking_cb_running_) {
          return "lane parking";
        }
        return "freespace parking";
      });
      RCLCPP_INFO_THROTTLE(
        getLogger(), *clock_, 1000, "Waiting for %s callback to finish...",
        running_callbacks.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 1000, "lane parking and freespace parking callbacks finished");
  }

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
  /**
   * @brief shared data for onTimer(onTimer/onFreespaceParkingTimer just read this)
   */
  struct GoalPlannerData
  {
    GoalPlannerData(const PlannerData & planner_data, const GoalPlannerParameters & parameters)
    {
      initializeOccupancyGridMap(planner_data, parameters);
    };
    GoalPlannerParameters parameters;
    std::shared_ptr<EgoPredictedPathParams> ego_predicted_path_params;
    std::shared_ptr<ObjectsFilteringParams> objects_filtering_params;
    std::shared_ptr<SafetyCheckParams> safety_check_params;
    autoware::universe_utils::LinearRing2d vehicle_footprint;

    PlannerData planner_data;
    ModuleStatus current_status;
    BehaviorModuleOutput previous_module_output;
    // collision detector
    // need to be shared_ptr to be used in planner and goal searcher
    std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map;
    std::shared_ptr<GoalSearcherBase> goal_searcher;

    const BehaviorModuleOutput & getPreviousModuleOutput() const { return previous_module_output; }
    const ModuleStatus & getCurrentStatus() const { return current_status; }
    void updateOccupancyGrid();
    GoalPlannerData clone() const;
    void update(
      const GoalPlannerParameters & parameters,
      const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params_,
      const std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params_,
      const std::shared_ptr<SafetyCheckParams> & safety_check_params_,
      const PlannerData & planner_data, const ModuleStatus & current_status,
      const BehaviorModuleOutput & previous_module_output,
      const std::shared_ptr<GoalSearcherBase> goal_searcher_,
      const autoware::universe_utils::LinearRing2d & vehicle_footprint);

  private:
    void initializeOccupancyGridMap(
      const PlannerData & planner_data, const GoalPlannerParameters & parameters);
  };
  std::optional<GoalPlannerData> gp_planner_data_{std::nullopt};
  std::mutex gp_planner_data_mutex_;

  // Flag class for managing whether a certain callback is running in multi-threading
  class ScopedFlag
  {
  public:
    explicit ScopedFlag(std::atomic<bool> & flag) : flag_(flag) { flag_.store(true); }

    ~ScopedFlag() { flag_.store(false); }

  private:
    std::atomic<bool> & flag_;
  };

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

  mutable StartGoalPlannerData goal_planner_data_;

  std::shared_ptr<GoalPlannerParameters> parameters_;

  mutable std::shared_ptr<EgoPredictedPathParams> ego_predicted_path_params_;
  mutable std::shared_ptr<ObjectsFilteringParams> objects_filtering_params_;
  mutable std::shared_ptr<SafetyCheckParams> safety_check_params_;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_{};

  // planner
  std::vector<std::shared_ptr<PullOverPlannerBase>> pull_over_planners_;
  std::unique_ptr<PullOverPlannerBase> freespace_planner_;
  std::unique_ptr<FixedGoalPlannerBase> fixed_goal_planner_;

  // goal searcher
  std::shared_ptr<GoalSearcherBase> goal_searcher_;

  // NOTE: this is latest occupancy_grid_map pointer which the local planner_data on
  // onFreespaceParkingTimer thread storage may point to while calculation.
  // onTimer/onFreespaceParkingTimer and their callees MUST NOT use this
  std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map_{nullptr};

  // check stopped and stuck state
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_stopped_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_stuck_;

  autoware::universe_utils::LinearRing2d vehicle_footprint_;

  std::recursive_mutex mutex_;
  // TODO(Mamoru Sobue): isSafePath() modifies ThreadSafeData::check_collision, avoid this mutable
  mutable ThreadSafeData thread_safe_data_;

  std::unique_ptr<LastApprovalData> last_approval_data_{nullptr};

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
  std::atomic<bool> is_lane_parking_cb_running_;

  // generate freespace parking paths in a separate thread
  rclcpp::TimerBase::SharedPtr freespace_parking_timer_;
  rclcpp::CallbackGroup::SharedPtr freespace_parking_timer_cb_group_;
  std::atomic<bool> is_freespace_parking_cb_running_;

  // debug
  mutable GoalPlannerDebugData debug_data_;
  mutable PoseWithString debug_stop_pose_with_info_;

  // collision check
  bool checkOccupancyGridCollision(
    const PathWithLaneId & path,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map) const;
  bool checkObjectsCollision(
    const PathWithLaneId & path, const std::vector<double> & curvatures,
    const std::shared_ptr<const PlannerData> planner_data, const GoalPlannerParameters & parameters,
    const double collision_check_margin, const bool extract_static_objects,
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
  PathWithLaneId generateFeasibleStopPath(const PathWithLaneId & path) const;

  void keepStoppedWithCurrentPath(PathWithLaneId & path) const;
  double calcSignedArcLengthFromEgo(const PathWithLaneId & path, const Pose & pose) const;

  // status
  bool isStopped();
  bool isStopped(
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> & odometry_buffer, const double time);
  bool hasFinishedCurrentPath();
  bool isOnModifiedGoal(const Pose & current_pose, const GoalPlannerParameters & parameters) const;
  double calcModuleRequestLength() const;
  bool needPathUpdate(
    const Pose & current_pose, const double path_update_duration,
    const GoalPlannerParameters & parameters) const;
  bool isStuck(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const GoalPlannerParameters & parameters);
  bool hasDecidedPath(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const GoalPlannerParameters & parameters,
    const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
    const std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
    const std::shared_ptr<SafetyCheckParams> & safety_check_params,
    const std::shared_ptr<GoalSearcherBase> goal_searcher) const;
  bool hasNotDecidedPath(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const GoalPlannerParameters & parameters,
    const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
    const std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
    const std::shared_ptr<SafetyCheckParams> & safety_check_params,
    const std::shared_ptr<GoalSearcherBase> goal_searcher) const;
  DecidingPathStatusWithStamp checkDecidingPathStatus(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const GoalPlannerParameters & parameters,
    const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
    const std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
    const std::shared_ptr<SafetyCheckParams> & safety_check_params,
    const std::shared_ptr<GoalSearcherBase> goal_searcher) const;
  void decideVelocity();
  bool foundPullOverPath() const;
  void updateStatus(const BehaviorModuleOutput & output);

  // validation
  bool hasEnoughDistance(
    const PullOverPath & pull_over_path, const PathWithLaneId & long_tail_reference_path) const;
  bool isCrossingPossible(
    const lanelet::ConstLanelet & start_lane, const lanelet::ConstLanelet & end_lane) const;
  bool isCrossingPossible(
    const Pose & start_pose, const Pose & end_pose, const lanelet::ConstLanelets lanes) const;
  bool isCrossingPossible(const PullOverPath & pull_over_path) const;
  bool hasEnoughTimePassedSincePathUpdate(const double duration) const;

  // freespace parking
  bool planFreespacePath(
    std::shared_ptr<const PlannerData> planner_data,
    const std::shared_ptr<GoalSearcherBase> goal_searcher,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map);
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
  void setOutput(BehaviorModuleOutput & output);
  void updatePreviousData();

  void setModifiedGoal(BehaviorModuleOutput & output) const;
  void setTurnSignalInfo(BehaviorModuleOutput & output);

  // new turn signal
  TurnSignalInfo calcTurnSignalInfo();
  std::optional<lanelet::Id> ignore_signal_{std::nullopt};

  bool hasPreviousModulePathShapeChanged(const BehaviorModuleOutput & previous_module_output) const;
  bool hasDeviatedFromLastPreviousModulePath(
    const std::shared_ptr<const PlannerData> planner_data) const;
  bool hasDeviatedFromCurrentPreviousModulePath(
    const std::shared_ptr<const PlannerData> planner_data,
    const BehaviorModuleOutput & previous_module_output) const;

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
  /*
  void updateSafetyCheckTargetObjectsData(
    const PredictedObjects & filtered_objects, const TargetObjectsOnLane & target_objects_on_lane,
    const std::vector<PoseWithVelocityStamped> & ego_predicted_path) const;
  */
  /**
   * @brief Checks if the current path is safe.
   * @return std::pair<bool, bool>
   *         first: If the path is safe for a certain period of time, true.
   *         second: If the path is safe in the current state, true.
   */
  std::pair<bool, bool> isSafePath(
    const std::shared_ptr<const PlannerData> planner_data, const GoalPlannerParameters & parameters,
    const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
    const std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
    const std::shared_ptr<SafetyCheckParams> & safety_check_params) const;

  // debug
  void setDebugData();
  void printParkingPositionError() const;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_PLANNER_MODULE_HPP_
