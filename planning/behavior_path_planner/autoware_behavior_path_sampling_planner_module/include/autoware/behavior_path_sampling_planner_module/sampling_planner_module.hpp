// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__SAMPLING_PLANNER_MODULE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__SAMPLING_PLANNER_MODULE_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/drivable_area_expansion.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_sampling_planner_module/sampling_planner_parameters.hpp"
#include "autoware/behavior_path_sampling_planner_module/util.hpp"
#include "autoware/motion_utils/trajectory/path_with_lane_id.hpp"
#include "autoware/universe_utils/geometry/boost_geometry.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware/universe_utils/math/constants.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"
#include "autoware_bezier_sampler/bezier_sampling.hpp"
#include "autoware_frenet_planner/frenet_planner.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "autoware_lanelet2_extension/utility/utilities.hpp"
#include "autoware_sampler_common/constraints/footprint.hpp"
#include "autoware_sampler_common/constraints/hard_constraint.hpp"
#include "autoware_sampler_common/constraints/soft_constraint.hpp"
#include "autoware_sampler_common/structures.hpp"
#include "autoware_sampler_common/transform/spline_transform.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tier4_planning_msgs/msg/lateral_offset.hpp"
#include "tier4_planning_msgs/msg/path_with_lane_id.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <algorithm>
#include <any>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
namespace autoware::behavior_path_planner
{
using autoware_planning_msgs::msg::TrajectoryPoint;
struct SamplingPlannerData
{
  // input
  std::vector<TrajectoryPoint> traj_points;  // converted from the input path
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;

  // ego
  geometry_msgs::msg::Pose ego_pose;
  double ego_vel{};
};

struct SamplingPlannerDebugData
{
  std::vector<autoware::sampler_common::Path> sampled_candidates{};
  size_t previous_sampled_candidates_nb = 0UL;
  std::vector<autoware::universe_utils::Polygon2d> obstacles{};
  std::vector<autoware::universe_utils::MultiPoint2d> footprints{};
};
class SamplingPlannerModule : public SceneModuleInterface
{
public:
  SamplingPlannerModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<SamplingPlannerParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    std::shared_ptr<SteeringFactorInterface> & steering_factor_interface_ptr);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  void updateData() override;
  bool isCurrentRouteLaneletToBeReset() const override
  {
    return getCurrentStatus() == ModuleStatus::SUCCESS;
  }

  void updateModuleParams(const std::any & parameters) override
  {
    std::shared_ptr<SamplingPlannerParameters> user_params_ =
      std::any_cast<std::shared_ptr<SamplingPlannerParameters>>(parameters);

    // Constraints
    internal_params_->constraints.hard.max_curvature = user_params_->max_curvature;
    internal_params_->constraints.hard.min_curvature = user_params_->min_curvature;
    internal_params_->constraints.soft.lateral_deviation_weight =
      user_params_->lateral_deviation_weight;
    internal_params_->constraints.soft.length_weight = user_params_->length_weight;
    internal_params_->constraints.soft.curvature_weight = user_params_->curvature_weight;
    internal_params_->constraints.soft.weights = user_params_->weights;
    internal_params_->constraints.ego_footprint = vehicle_info_.createFootprint(0.25);
    internal_params_->constraints.ego_width = vehicle_info_.vehicle_width_m;
    internal_params_->constraints.ego_length = vehicle_info_.vehicle_length_m;
    // Sampling
    internal_params_->sampling.enable_frenet = user_params_->enable_frenet;
    internal_params_->sampling.enable_bezier = user_params_->enable_bezier;
    internal_params_->sampling.resolution = user_params_->resolution;
    internal_params_->sampling.previous_path_reuse_points_nb =
      user_params_->previous_path_reuse_points_nb;
    internal_params_->sampling.target_lengths = user_params_->target_lengths;
    internal_params_->sampling.target_lateral_positions = user_params_->target_lateral_positions;
    internal_params_->sampling.nb_target_lateral_positions =
      user_params_->nb_target_lateral_positions;

    internal_params_->sampling.frenet.target_lateral_velocities =
      user_params_->target_lateral_velocities;
    internal_params_->sampling.frenet.target_lateral_accelerations =
      user_params_->target_lateral_accelerations;

    // Preprocessing
    internal_params_->preprocessing.force_zero_deviation = user_params_->force_zero_deviation;
    internal_params_->preprocessing.force_zero_heading = user_params_->force_zero_heading;
    internal_params_->preprocessing.smooth_reference = user_params_->smooth_reference;
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  SamplingPlannerDebugData debug_data_;
  HardConstraintsFunctionVector hard_constraints_;
  SoftConstraintsFunctionVector soft_constraints_;

private:
  SamplingPlannerData createPlannerData(
    const PlanResult & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound) const;

  PlanResult generatePath();

  bool canTransitSuccessState() override
  {
    std::vector<DrivableLanes> drivable_lanes{};
    const auto & prev_module_path =
      std::make_shared<PathWithLaneId>(getPreviousModuleOutput().path);
    const auto prev_module_reference_path =
      std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

    const auto & p = planner_data_->parameters;
    const auto ego_pose = planner_data_->self_odometry->pose.pose;
    const auto goal_pose = planner_data_->route_handler->getGoalPose();

    lanelet::ConstLanelet current_lane;

    if (!planner_data_->route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lane)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("behavior_path_planner").get_child("utils"),
        "failed to find closest lanelet within route!!!");
      return false;
    }
    const auto current_lane_sequence = planner_data_->route_handler->getLaneletSequence(
      current_lane, ego_pose, p.backward_path_length, p.forward_path_length);
    // expand drivable lanes
    std::for_each(
      current_lane_sequence.begin(), current_lane_sequence.end(), [&](const auto & lanelet) {
        drivable_lanes.push_back(generateExpandDrivableLanes(lanelet, planner_data_));
      });

    lanelet::ConstLanelets current_lanes;

    for (auto & d : drivable_lanes) {
      current_lanes.push_back(d.right_lane);
      current_lanes.push_back(d.left_lane);
      current_lanes.insert(current_lanes.end(), d.middle_lanes.begin(), d.middle_lanes.end());
    }
    lanelet::ConstLanelet closest_lanelet_to_ego;
    lanelet::utils::query::getClosestLanelet(current_lanes, ego_pose, &closest_lanelet_to_ego);
    lanelet::ConstLanelet closest_lanelet_to_goal;
    lanelet::utils::query::getClosestLanelet(current_lanes, goal_pose, &closest_lanelet_to_goal);
    const bool ego_and_goal_on_same_lanelet =
      closest_lanelet_to_goal.id() == closest_lanelet_to_ego.id();

    if (!ego_and_goal_on_same_lanelet) return false;

    const auto ego_arc = lanelet::utils::getArcCoordinates(current_lanes, ego_pose);
    const auto goal_arc = lanelet::utils::getArcCoordinates(current_lanes, goal_pose);
    const double length_to_goal = std::abs(goal_arc.length - ego_arc.length);

    constexpr double epsilon = 1E-5;
    if (length_to_goal < epsilon) return isReferencePathSafe();

    const auto nearest_index =
      autoware::motion_utils::findNearestIndex(prev_module_reference_path->points, ego_pose);
    if (!nearest_index) return false;
    auto toYaw = [](const geometry_msgs::msg::Quaternion & quat) -> double {
      const auto rpy = autoware::universe_utils::getRPY(quat);
      return rpy.z;
    };
    const auto quat = prev_module_reference_path->points[*nearest_index].point.pose.orientation;
    const double ref_path_yaw = toYaw(quat);
    const double ego_yaw = toYaw(ego_pose.orientation);
    const double yaw_difference = std::abs(ego_yaw - ref_path_yaw);

    // TODO(Daniel) magic numbers
    constexpr double threshold_lat_distance_for_merging = 0.15;
    constexpr double threshold_yaw_difference_for_merging = M_PI / 72.0;  // 2.5 degrees
    const bool merged_back_to_path =
      (std::abs(ego_arc.distance) < threshold_lat_distance_for_merging) &&
      (yaw_difference < threshold_yaw_difference_for_merging);
    return isReferencePathSafe() && (merged_back_to_path);
  }

  bool canTransitFailureState() override { return false; }

  bool isReferencePathSafe() const;

  void updateDebugMarkers();

  void prepareConstraints(
    autoware::sampler_common::Constraints & constraints,
    const PredictedObjects::ConstSharedPtr & predicted_objects,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound) const;

  autoware::frenet_planner::SamplingParameters prepareSamplingParameters(
    const autoware::sampler_common::State & initial_state,
    const autoware::sampler_common::transform::Spline2D & path_spline,
    const SamplingPlannerInternalParameters & internal_params_);

  PathWithLaneId convertFrenetPathToPathWithLaneID(
    const autoware::frenet_planner::Path frenet_path, const lanelet::ConstLanelets & lanelets,
    const double path_z);

  // member
  // std::shared_ptr<SamplingPlannerParameters> params_;
  std::shared_ptr<SamplingPlannerInternalParameters> internal_params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_{};
  std::optional<autoware::frenet_planner::Path> prev_sampling_path_ = std::nullopt;
  // move to utils

  void extendOutputDrivableArea(
    BehaviorModuleOutput & output, std::vector<DrivableLanes> & drivable_lanes);
  bool isEndPointsConnected(
    const lanelet::ConstLanelet & left_lane, const lanelet::ConstLanelet & right_lane) const;
  DrivableLanes generateExpandDrivableLanes(
    const lanelet::ConstLanelet & lanelet,
    const std::shared_ptr<const PlannerData> & planner_data) const;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__SAMPLING_PLANNER_MODULE_HPP_
