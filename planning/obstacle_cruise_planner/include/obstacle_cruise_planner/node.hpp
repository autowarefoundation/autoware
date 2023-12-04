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

#ifndef OBSTACLE_CRUISE_PLANNER__NODE_HPP_
#define OBSTACLE_CRUISE_PLANNER__NODE_HPP_

#include "obstacle_cruise_planner/common_structs.hpp"
#include "obstacle_cruise_planner/optimization_based_planner/optimization_based_planner.hpp"
#include "obstacle_cruise_planner/pid_based_planner/pid_based_planner.hpp"
#include "obstacle_cruise_planner/type_alias.hpp"
#include "signal_processing/lowpass_filter_1d.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace motion_planning
{
class ObstacleCruisePlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleCruisePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // callback functions
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);
  void onTrajectory(const Trajectory::ConstSharedPtr msg);
  void onSmoothedTrajectory(const Trajectory::ConstSharedPtr msg);

  // main functions
  std::vector<Polygon2d> createOneStepPolygons(
    const std::vector<TrajectoryPoint> & traj_points,
    const vehicle_info_util::VehicleInfo & vehicle_info,
    const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin = 0.0) const;
  std::vector<Obstacle> convertToObstacles(const std::vector<TrajectoryPoint> & traj_points) const;
  std::tuple<std::vector<StopObstacle>, std::vector<CruiseObstacle>, std::vector<SlowDownObstacle>>
  determineEgoBehaviorAgainstObstacles(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Obstacle> & obstacles);
  std::vector<TrajectoryPoint> decimateTrajectoryPoints(
    const std::vector<TrajectoryPoint> & traj_points) const;
  std::optional<StopObstacle> createStopObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
    const Obstacle & obstacle, const double precise_lateral_dist) const;
  bool isStopObstacle(const uint8_t label) const;
  bool isInsideCruiseObstacle(const uint8_t label) const;
  bool isOutsideCruiseObstacle(const uint8_t label) const;
  bool isCruiseObstacle(const uint8_t label) const;
  bool isSlowDownObstacle(const uint8_t label) const;
  std::optional<geometry_msgs::msg::Point> createCollisionPointForStopObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
    const Obstacle & obstacle) const;
  std::optional<CruiseObstacle> createCruiseObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
    const Obstacle & obstacle, const double precise_lat_dist);
  std::optional<std::vector<PointWithStamp>> createCollisionPointsForInsideCruiseObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
    const Obstacle & obstacle) const;
  std::optional<std::vector<PointWithStamp>> createCollisionPointsForOutsideCruiseObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
    const Obstacle & obstacle) const;
  bool isObstacleCrossing(
    const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle) const;
  double calcCollisionTimeMargin(
    const std::vector<PointWithStamp> & collision_points,
    const std::vector<TrajectoryPoint> & traj_points, const bool is_driving_forward) const;
  std::optional<SlowDownObstacle> createSlowDownObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
    const double precise_lat_dist);
  PlannerData createPlannerData(const std::vector<TrajectoryPoint> & traj_points) const;

  void checkConsistency(
    const rclcpp::Time & current_time, const PredictedObjects & predicted_objects,
    std::vector<StopObstacle> & stop_obstacles);
  void publishVelocityLimit(
    const std::optional<VelocityLimit> & vel_limit, const std::string & module_name);
  void publishDebugMarker() const;
  void publishDebugInfo() const;
  void publishCalculationTime(const double calculation_time) const;

  bool isFrontCollideObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
    const size_t first_collision_idx) const;

  bool enable_debug_info_;
  bool enable_calculation_time_info_;
  double min_behavior_stop_margin_;
  bool enable_approaching_on_curve_;
  double additional_safe_distance_margin_on_curve_;
  double min_safe_distance_margin_on_curve_;
  bool suppress_sudden_obstacle_stop_;

  std::vector<int> stop_obstacle_types_;
  std::vector<int> inside_cruise_obstacle_types_;
  std::vector<int> outside_cruise_obstacle_types_;
  std::vector<int> slow_down_obstacle_types_;

  // parameter callback result
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr vel_limit_pub_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr clear_vel_limit_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_cruise_wall_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_stop_wall_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_slow_down_wall_marker_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_stop_planning_info_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_cruise_planning_info_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_slow_down_planning_info_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_calculation_time_pub_;

  // subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr acc_sub_;

  // data for callback functions
  PredictedObjects::ConstSharedPtr objects_ptr_{nullptr};
  Odometry::ConstSharedPtr ego_odom_ptr_{nullptr};
  AccelWithCovarianceStamped::ConstSharedPtr ego_accel_ptr_{nullptr};

  // Vehicle Parameters
  VehicleInfo vehicle_info_;

  // planning algorithm
  enum class PlanningAlgorithm { OPTIMIZATION_BASE, PID_BASE, INVALID };
  PlanningAlgorithm getPlanningAlgorithmType(const std::string & param) const;
  PlanningAlgorithm planning_algorithm_;

  // stop watch
  mutable tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_{nullptr};

  // planner
  std::unique_ptr<PlannerInterface> planner_ptr_{nullptr};

  // previous obstacles
  std::vector<StopObstacle> prev_stop_obstacles_;
  std::vector<CruiseObstacle> prev_cruise_obstacles_;
  std::vector<SlowDownObstacle> prev_slow_down_obstacles_;

  // behavior determination parameter
  struct BehaviorDeterminationParam
  {
    BehaviorDeterminationParam() = default;
    explicit BehaviorDeterminationParam(rclcpp::Node & node);
    void onParam(const std::vector<rclcpp::Parameter> & parameters);

    double decimate_trajectory_step_length;
    // hysteresis for stop and cruise
    double obstacle_velocity_threshold_from_cruise_to_stop;
    double obstacle_velocity_threshold_from_stop_to_cruise;
    // inside
    double crossing_obstacle_velocity_threshold;
    double collision_time_margin;
    // outside
    double outside_obstacle_min_velocity_threshold;
    double ego_obstacle_overlap_time_threshold;
    double max_prediction_time_for_collision_check;
    double crossing_obstacle_traj_angle_threshold;
    // obstacle hold
    double stop_obstacle_hold_time_threshold;
    // prediction resampling
    double prediction_resampling_time_interval;
    double prediction_resampling_time_horizon;
    // max lateral margin
    double max_lat_margin_for_stop;
    double max_lat_margin_for_cruise;
    double max_lat_margin_for_slow_down;
    double lat_hysteresis_margin_for_slow_down;
    int successive_num_to_entry_slow_down_condition;
    int successive_num_to_exit_slow_down_condition;
    // consideration for the current ego pose
    bool enable_to_consider_current_pose{false};
    double time_to_convergence{1.5};
  };
  BehaviorDeterminationParam behavior_determination_param_;

  std::unordered_map<std::string, bool> need_to_clear_vel_limit_{
    {"cruise", false}, {"slow_down", false}};

  struct SlowDownConditionCounter
  {
    void resetCurrentUuids() { current_uuids_.clear(); }
    void addCurrentUuid(const std::string & uuid) { current_uuids_.push_back(uuid); }
    void removeCounterUnlessUpdated()
    {
      std::vector<std::string> obsolete_uuids;
      for (const auto & key_and_value : counter_) {
        if (
          std::find(current_uuids_.begin(), current_uuids_.end(), key_and_value.first) ==
          current_uuids_.end()) {
          obsolete_uuids.push_back(key_and_value.first);
        }
      }

      for (const auto & obsolete_uuid : obsolete_uuids) {
        counter_.erase(obsolete_uuid);
      }
    }

    int increaseCounter(const std::string & uuid)
    {
      if (counter_.count(uuid) != 0) {
        counter_.at(uuid) = std::max(1, counter_.at(uuid) + 1);
      } else {
        counter_.emplace(uuid, 1);
      }
      return counter_.at(uuid);
    }
    int decreaseCounter(const std::string & uuid)
    {
      if (counter_.count(uuid) != 0) {
        counter_.at(uuid) = std::min(-1, counter_.at(uuid) - 1);
      } else {
        counter_.emplace(uuid, -1);
      }
      return counter_.at(uuid);
    }
    void reset(const std::string & uuid) { counter_.emplace(uuid, 0); }

    // NOTE: positive is for meeting entering condition, and negative is for exiting.
    std::unordered_map<std::string, int> counter_;
    std::vector<std::string> current_uuids_;
  };
  SlowDownConditionCounter slow_down_condition_counter_;

  EgoNearestParam ego_nearest_param_;

  bool is_driving_forward_{true};
  bool enable_slow_down_planning_{false};

  // previous closest obstacle
  std::shared_ptr<StopObstacle> prev_closest_stop_obstacle_ptr_{nullptr};

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace motion_planning

#endif  // OBSTACLE_CRUISE_PLANNER__NODE_HPP_
