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

#include "obstacle_avoidance_planner/node.hpp"

#include "interpolation/spline_interpolation_points_2d.hpp"
#include "obstacle_avoidance_planner/cv_utils.hpp"
#include "obstacle_avoidance_planner/debug_visualization.hpp"
#include "obstacle_avoidance_planner/utils.hpp"
#include "rclcpp/time.hpp"
#include "tf2/utils.h"
#include "tier4_autoware_utils/ros/update_param.hpp"
#include "tier4_autoware_utils/trajectory/tmp_conversion.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
template <typename T1, typename T2>
size_t searchExtendedZeroVelocityIndex(
  const std::vector<T1> & fine_points, const std::vector<T2> & vel_points)
{
  const auto opt_zero_vel_idx = tier4_autoware_utils::searchZeroVelocityIndex(vel_points);
  const size_t zero_vel_idx = opt_zero_vel_idx ? opt_zero_vel_idx.get() : vel_points.size() - 1;
  return tier4_autoware_utils::findNearestIndex(
    fine_points, vel_points.at(zero_vel_idx).pose.position);
}

bool isPathShapeChanged(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<std::vector<autoware_auto_planning_msgs::msg::PathPoint>> &
    prev_path_points,
  const double max_mpt_length, const double max_path_shape_change_dist,
  const double delta_yaw_threshold)
{
  if (!prev_path_points) {
    return false;
  }

  // truncate prev points from ego pose to fixed end points
  const auto opt_prev_begin_idx = tier4_autoware_utils::findNearestIndex(
    *prev_path_points, ego_pose, std::numeric_limits<double>::max(), delta_yaw_threshold);
  const size_t prev_begin_idx = opt_prev_begin_idx ? *opt_prev_begin_idx : 0;
  const auto truncated_prev_points =
    points_utils::clipForwardPoints(*prev_path_points, prev_begin_idx, max_mpt_length);

  // truncate points from ego pose to fixed end points
  const auto opt_begin_idx = tier4_autoware_utils::findNearestIndex(
    path_points, ego_pose, std::numeric_limits<double>::max(), delta_yaw_threshold);
  const size_t begin_idx = opt_begin_idx ? *opt_begin_idx : 0;
  const auto truncated_points =
    points_utils::clipForwardPoints(path_points, begin_idx, max_mpt_length);

  // guard for lateral offset
  if (truncated_prev_points.size() < 2 || truncated_points.size() < 2) {
    return false;
  }

  // calculate lateral deviations between truncated path_points and prev_path_points
  for (const auto & prev_point : truncated_prev_points) {
    const double dist =
      tier4_autoware_utils::calcLateralOffset(truncated_points, prev_point.pose.position);
    if (dist > max_path_shape_change_dist) {
      return true;
    }
  }

  return false;
}

bool isPathGoalChanged(
  const double current_vel,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<std::vector<autoware_auto_planning_msgs::msg::PathPoint>> &
    prev_path_points)
{
  if (!prev_path_points) {
    return false;
  }

  constexpr double min_vel = 1e-3;
  if (std::abs(current_vel) > min_vel) {
    return false;
  }

  // NOTE: Path may be cropped and does not contain the goal.
  // Therefore we set a large value to distance threshold.
  constexpr double max_goal_moving_dist = 1.0;
  const double goal_moving_dist =
    tier4_autoware_utils::calcDistance2d(path_points.back(), prev_path_points->back());
  if (goal_moving_dist < max_goal_moving_dist) {
    return false;
  }

  return true;
}

bool hasValidNearestPointFromEgo(
  const geometry_msgs::msg::Pose & ego_pose, const Trajectories & trajs,
  const TrajectoryParam & traj_param)
{
  const auto traj = trajs.model_predictive_trajectory;
  const auto interpolated_points =
    interpolation_utils::getInterpolatedPoints(traj, traj_param.delta_arc_length_for_trajectory);

  const auto interpolated_poses_with_yaw =
    points_utils::convertToPosesWithYawEstimation(interpolated_points);
  const auto opt_nearest_idx = tier4_autoware_utils::findNearestIndex(
    interpolated_poses_with_yaw, ego_pose, traj_param.delta_dist_threshold_for_closest_point,
    traj_param.delta_yaw_threshold_for_closest_point);

  if (!opt_nearest_idx) {
    return false;
  }
  return true;
}

std::tuple<double, std::vector<double>> calcVehicleCirclesInfo(
  const VehicleParam & vehicle_param, const size_t circle_num_for_constraints,
  const size_t circle_num_for_radius, const double radius_ratio)
{
  const double radius = std::hypot(
                          vehicle_param.length / static_cast<double>(circle_num_for_radius) / 2.0,
                          vehicle_param.width / 2.0) *
                        radius_ratio;

  std::vector<double> longitudinal_offsets;
  const double unit_lon_length = vehicle_param.length / static_cast<double>(circle_num_for_radius);
  for (size_t i = 0; i < circle_num_for_constraints; ++i) {
    longitudinal_offsets.push_back(
      unit_lon_length / 2.0 +
      (unit_lon_length * (circle_num_for_radius - 1)) /
        static_cast<double>(circle_num_for_constraints - 1) * i -
      vehicle_param.rear_overhang);
  }

  return {radius, longitudinal_offsets};
}

[[maybe_unused]] void fillYawInTrajectoryPoint(
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & traj_point : traj_points) {
    points.push_back(traj_point.pose.position);
  }
  const auto yaw_vec = interpolation::slerpYawFromPoints(points);

  for (size_t i = 0; i < traj_points.size(); ++i) {
    traj_points.at(i).pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(yaw_vec.at(i));
  }
}

}  // namespace

ObstacleAvoidancePlanner::ObstacleAvoidancePlanner(const rclcpp::NodeOptions & node_options)
: Node("obstacle_avoidance_planner", node_options),
  logger_ros_clock_(RCL_ROS_TIME),
  eb_solved_count_(0)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // qos
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  // publisher to other nodes
  traj_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/path", 1);

  // debug publisher
  debug_eb_traj_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/debug/eb_trajectory", durable_qos);
  debug_extended_fixed_traj_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/debug/extended_fixed_traj", 1);
  debug_extended_non_fixed_traj_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/debug/extended_non_fixed_traj", 1);
  debug_mpt_fixed_traj_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/debug/mpt_fixed_traj", 1);
  debug_mpt_ref_traj_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/debug/mpt_ref_traj", 1);
  debug_mpt_traj_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/debug/mpt_traj", 1);
  debug_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", durable_qos);
  debug_wall_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/wall_marker", durable_qos);
  debug_clearance_map_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/clearance_map", durable_qos);
  debug_object_clearance_map_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/object_clearance_map", durable_qos);
  debug_area_with_objects_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/area_with_objects", durable_qos);
  debug_msg_pub_ =
    create_publisher<tier4_debug_msgs::msg::StringStamped>("~/debug/calculation_time", 1);

  // subscriber
  path_sub_ = create_subscription<autoware_auto_planning_msgs::msg::Path>(
    "~/input/path", rclcpp::QoS{1},
    std::bind(&ObstacleAvoidancePlanner::pathCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&ObstacleAvoidancePlanner::odomCallback, this, std::placeholders::_1));
  objects_sub_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", rclcpp::QoS{10},
    std::bind(&ObstacleAvoidancePlanner::objectsCallback, this, std::placeholders::_1));
  is_avoidance_sub_ = create_subscription<tier4_planning_msgs::msg::EnableAvoidance>(
    "/planning/scenario_planning/lane_driving/obstacle_avoidance_approval", rclcpp::QoS{10},
    std::bind(&ObstacleAvoidancePlanner::enableAvoidanceCallback, this, std::placeholders::_1));

  {  // vehicle param
    vehicle_param_ = VehicleParam{};
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    vehicle_param_.width = vehicle_info.vehicle_width_m;
    vehicle_param_.length = vehicle_info.vehicle_length_m;
    vehicle_param_.wheelbase = vehicle_info.wheel_base_m;
    vehicle_param_.rear_overhang = vehicle_info.rear_overhang_m;
    vehicle_param_.front_overhang = vehicle_info.front_overhang_m;
  }

  {  // option parameter
    is_publishing_debug_visualization_marker_ =
      declare_parameter<bool>("option.is_publishing_debug_visualization_marker");
    is_publishing_clearance_map_ = declare_parameter<bool>("option.is_publishing_clearance_map");
    is_publishing_object_clearance_map_ =
      declare_parameter<bool>("option.is_publishing_object_clearance_map");
    is_publishing_area_with_objects_ =
      declare_parameter<bool>("option.is_publishing_area_with_objects");

    is_showing_debug_info_ = declare_parameter<bool>("option.is_showing_debug_info");
    is_showing_calculation_time_ = declare_parameter<bool>("option.is_showing_calculation_time");
    is_stopping_if_outside_drivable_area_ =
      declare_parameter<bool>("option.is_stopping_if_outside_drivable_area");

    // drivability check
    use_vehicle_circles_for_drivability_ =
      declare_parameter<bool>("advanced.option.drivability_check.use_vehicle_circles");
    if (use_vehicle_circles_for_drivability_) {
      // vehicle_circles
      // NOTE: Vehicle shape for drivability check is considered as a set of circles
      use_manual_vehicle_circles_for_drivability_ = declare_parameter<bool>(
        "advanced.option.drivability_check.vehicle_circles.use_manual_vehicle_circles");
      vehicle_circle_constraints_num_for_drivability_ = declare_parameter<int>(
        "advanced.option.drivability_check.vehicle_circles.num_for_constraints");
      if (use_manual_vehicle_circles_for_drivability_) {  // vehicle circles are designated manually
        vehicle_circle_longitudinal_offsets_for_drivability_ =
          declare_parameter<std::vector<double>>(
            "advanced.option.drivability_check.vehicle_circles.longitudinal_offsets");
        vehicle_circle_radius_for_drivability_ =
          declare_parameter<double>("advanced.option.drivability_check.vehicle_circles.radius");
      } else {  // vehicle circles are calculated automatically with designated ratio
        const int default_radius_num =
          std::round(vehicle_param_.length / vehicle_param_.width * 1.5);

        vehicle_circle_radius_num_for_drivability_ = declare_parameter<int>(
          "advanced.option.drivability_check.vehicle_circles.num_for_radius", default_radius_num);
        vehicle_circle_radius_ratio_for_drivability_ = declare_parameter<double>(
          "advanced.option.drivability_check.vehicle_circles.radius_ratio");

        std::tie(
          vehicle_circle_radius_for_drivability_,
          vehicle_circle_longitudinal_offsets_for_drivability_) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_constraints_num_for_drivability_,
            vehicle_circle_radius_num_for_drivability_,
            vehicle_circle_radius_ratio_for_drivability_);
      }
    }

    enable_avoidance_ = declare_parameter<bool>("option.enable_avoidance");
    enable_pre_smoothing_ = declare_parameter<bool>("option.enable_pre_smoothing");
    skip_optimization_ = declare_parameter<bool>("option.skip_optimization");
    reset_prev_optimization_ = declare_parameter<bool>("option.reset_prev_optimization");
  }

  {  // trajectory parameter
    traj_param_ = TrajectoryParam{};

    // common
    traj_param_.num_sampling_points = declare_parameter<int>("common.num_sampling_points");
    traj_param_.trajectory_length = declare_parameter<double>("common.trajectory_length");
    traj_param_.forward_fixing_min_distance =
      declare_parameter<double>("common.forward_fixing_min_distance");
    traj_param_.forward_fixing_min_time =
      declare_parameter<double>("common.forward_fixing_min_time");
    traj_param_.backward_fixing_distance =
      declare_parameter<double>("common.backward_fixing_distance");
    traj_param_.delta_arc_length_for_trajectory =
      declare_parameter<double>("common.delta_arc_length_for_trajectory");

    traj_param_.delta_dist_threshold_for_closest_point =
      declare_parameter<double>("common.delta_dist_threshold_for_closest_point");
    traj_param_.delta_yaw_threshold_for_closest_point =
      declare_parameter<double>("common.delta_yaw_threshold_for_closest_point");
    traj_param_.delta_yaw_threshold_for_straight =
      declare_parameter<double>("common.delta_yaw_threshold_for_straight");

    traj_param_.num_fix_points_for_extending =
      declare_parameter<int>("common.num_fix_points_for_extending");
    traj_param_.max_dist_for_extending_end_point =
      declare_parameter<double>("common.max_dist_for_extending_end_point");

    // object
    traj_param_.max_avoiding_ego_velocity_ms =
      declare_parameter<double>("object.max_avoiding_ego_velocity_ms");
    traj_param_.max_avoiding_objects_velocity_ms =
      declare_parameter<double>("object.max_avoiding_objects_velocity_ms");
    traj_param_.is_avoiding_unknown =
      declare_parameter<bool>("object.avoiding_object_type.unknown", true);
    traj_param_.is_avoiding_car = declare_parameter<bool>("object.avoiding_object_type.car", true);
    traj_param_.is_avoiding_truck =
      declare_parameter<bool>("object.avoiding_object_type.truck", true);
    traj_param_.is_avoiding_bus = declare_parameter<bool>("object.avoiding_object_type.bus", true);
    traj_param_.is_avoiding_bicycle =
      declare_parameter<bool>("object.avoiding_object_type.bicycle", true);
    traj_param_.is_avoiding_motorbike =
      declare_parameter<bool>("object.avoiding_object_type.motorbike", true);
    traj_param_.is_avoiding_pedestrian =
      declare_parameter<bool>("object.avoiding_object_type.pedestrian", true);
    traj_param_.is_avoiding_animal =
      declare_parameter<bool>("object.avoiding_object_type.animal", true);
  }

  {  // elastic band parameter
    eb_param_ = EBParam{};

    // common
    eb_param_.num_joint_buffer_points =
      declare_parameter<int>("advanced.eb.common.num_joint_buffer_points");
    eb_param_.num_offset_for_begin_idx =
      declare_parameter<int>("advanced.eb.common.num_offset_for_begin_idx");
    eb_param_.delta_arc_length_for_eb =
      declare_parameter<double>("advanced.eb.common.delta_arc_length_for_eb");
    eb_param_.num_sampling_points_for_eb =
      declare_parameter<int>("advanced.eb.common.num_sampling_points_for_eb");

    // clearance
    eb_param_.clearance_for_straight_line =
      declare_parameter<double>("advanced.eb.clearance.clearance_for_straight_line");
    eb_param_.clearance_for_joint =
      declare_parameter<double>("advanced.eb.clearance.clearance_for_joint");
    eb_param_.clearance_for_only_smoothing =
      declare_parameter<double>("advanced.eb.clearance.clearance_for_only_smoothing");

    // qp
    eb_param_.qp_param.max_iteration = declare_parameter<int>("advanced.eb.qp.max_iteration");
    eb_param_.qp_param.eps_abs = declare_parameter<double>("advanced.eb.qp.eps_abs");
    eb_param_.qp_param.eps_rel = declare_parameter<double>("advanced.eb.qp.eps_rel");

    // other
    eb_param_.clearance_for_fixing = 0.0;
  }

  {  // mpt param
    mpt_param_ = MPTParam{};

    // option
    // TODO(murooka) implement plan_from_ego
    mpt_param_.plan_from_ego = false;
    // mpt_param_.plan_from_ego = declare_parameter<bool>("mpt.option.plan_from_ego");
    mpt_param_.steer_limit_constraint =
      declare_parameter<bool>("mpt.option.steer_limit_constraint");
    mpt_param_.fix_points_around_ego = declare_parameter<bool>("mpt.option.fix_points_around_ego");
    mpt_param_.enable_warm_start = declare_parameter<bool>("mpt.option.enable_warm_start");
    mpt_param_.enable_manual_warm_start =
      declare_parameter<bool>("mpt.option.enable_manual_warm_start");
    mpt_visualize_sampling_num_ = declare_parameter<int>("mpt.option.visualize_sampling_num");

    // common
    mpt_param_.num_curvature_sampling_points =
      declare_parameter<int>("mpt.common.num_curvature_sampling_points");

    mpt_param_.delta_arc_length_for_mpt_points =
      declare_parameter<double>("mpt.common.delta_arc_length_for_mpt_points");

    // kinematics
    mpt_param_.max_steer_rad =
      declare_parameter<double>("mpt.kinematics.max_steer_deg") * M_PI / 180.0;

    // By default, optimization_center_offset will be vehicle_info.wheel_base * 0.8
    // The 0.8 scale is adopted as it performed the best.
    constexpr double default_wheelbase_ratio = 0.8;
    mpt_param_.optimization_center_offset = declare_parameter<double>(
      "mpt.kinematics.optimization_center_offset",
      vehicle_param_.wheelbase * default_wheelbase_ratio);

    // collision free constraints
    mpt_param_.l_inf_norm =
      declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.l_inf_norm");
    mpt_param_.soft_constraint =
      declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.soft_constraint");
    mpt_param_.hard_constraint =
      declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.hard_constraint");
    // TODO(murooka) implement two-step soft constraint
    mpt_param_.two_step_soft_constraint = false;
    // mpt_param_.two_step_soft_constraint =
    // declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.two_step_soft_constraint");

    // vehicle_circles
    // NOTE: Vehicle shape for collision free constraints is considered as a set of circles
    use_manual_vehicle_circles_for_mpt_ = declare_parameter<bool>(
      "advanced.mpt.collision_free_constraints.vehicle_circles.use_manual_vehicle_circles");
    vehicle_circle_constraints_num_for_mpt_ = declare_parameter<int>(
      "advanced.mpt.collision_free_constraints.vehicle_circles.num_for_constraints");
    if (use_manual_vehicle_circles_for_mpt_) {  // vehicle circles are designated manually
      mpt_param_.vehicle_circle_longitudinal_offsets = declare_parameter<std::vector<double>>(
        "advanced.mpt.collision_free_constraints.vehicle_circles.longitudinal_offsets");
      mpt_param_.vehicle_circle_radius =
        declare_parameter<double>("advanced.mpt.collision_free_constraints.vehicle_circles.radius");
    } else {  // vehicle circles are calculated automatically with designated ratio
      const int default_radius_num = std::round(vehicle_param_.length / vehicle_param_.width * 1.5);

      vehicle_circle_radius_num_for_mpt_ = declare_parameter<int>(
        "advanced.mpt.collision_free_constraints.vehicle_circles.num_for_radius",
        default_radius_num);
      vehicle_circle_radius_ratio_for_mpt_ = declare_parameter<double>(
        "advanced.mpt.collision_free_constraints.vehicle_circles.radius_ratio");

      std::tie(mpt_param_.vehicle_circle_radius, mpt_param_.vehicle_circle_longitudinal_offsets) =
        calcVehicleCirclesInfo(
          vehicle_param_, vehicle_circle_constraints_num_for_mpt_,
          vehicle_circle_radius_num_for_mpt_, vehicle_circle_radius_ratio_for_mpt_);
    }

    // clearance
    mpt_param_.hard_clearance_from_road =
      declare_parameter<double>("advanced.mpt.clearance.hard_clearance_from_road");
    mpt_param_.soft_clearance_from_road =
      declare_parameter<double>("advanced.mpt.clearance.soft_clearance_from_road");
    mpt_param_.soft_second_clearance_from_road =
      declare_parameter<double>("advanced.mpt.clearance.soft_second_clearance_from_road");
    mpt_param_.extra_desired_clearance_from_road =
      declare_parameter<double>("advanced.mpt.clearance.extra_desired_clearance_from_road");
    mpt_param_.clearance_from_object =
      declare_parameter<double>("advanced.mpt.clearance.clearance_from_object");

    // weight
    mpt_param_.soft_avoidance_weight =
      declare_parameter<double>("advanced.mpt.weight.soft_avoidance_weight");
    mpt_param_.soft_second_avoidance_weight =
      declare_parameter<double>("advanced.mpt.weight.soft_second_avoidance_weight");

    mpt_param_.lat_error_weight = declare_parameter<double>("advanced.mpt.weight.lat_error_weight");
    mpt_param_.yaw_error_weight = declare_parameter<double>("advanced.mpt.weight.yaw_error_weight");
    mpt_param_.yaw_error_rate_weight =
      declare_parameter<double>("advanced.mpt.weight.yaw_error_rate_weight");
    mpt_param_.steer_input_weight =
      declare_parameter<double>("advanced.mpt.weight.steer_input_weight");
    mpt_param_.steer_rate_weight =
      declare_parameter<double>("advanced.mpt.weight.steer_rate_weight");

    mpt_param_.obstacle_avoid_lat_error_weight =
      declare_parameter<double>("advanced.mpt.weight.obstacle_avoid_lat_error_weight");
    mpt_param_.obstacle_avoid_yaw_error_weight =
      declare_parameter<double>("advanced.mpt.weight.obstacle_avoid_yaw_error_weight");
    mpt_param_.obstacle_avoid_steer_input_weight =
      declare_parameter<double>("advanced.mpt.weight.obstacle_avoid_steer_input_weight");
    mpt_param_.near_objects_length =
      declare_parameter<double>("advanced.mpt.weight.near_objects_length");

    mpt_param_.terminal_lat_error_weight =
      declare_parameter<double>("advanced.mpt.weight.terminal_lat_error_weight");
    mpt_param_.terminal_yaw_error_weight =
      declare_parameter<double>("advanced.mpt.weight.terminal_yaw_error_weight");
    mpt_param_.terminal_path_lat_error_weight =
      declare_parameter<double>("advanced.mpt.weight.terminal_path_lat_error_weight");
    mpt_param_.terminal_path_yaw_error_weight =
      declare_parameter<double>("advanced.mpt.weight.terminal_path_yaw_error_weight");
  }

  {  // replan
    max_path_shape_change_dist_for_replan_ =
      declare_parameter<double>("replan.max_path_shape_change_dist");
    max_ego_moving_dist_for_replan_ =
      declare_parameter<double>("replan.max_ego_moving_dist_for_replan");
    max_delta_time_sec_for_replan_ =
      declare_parameter<double>("replan.max_delta_time_sec_for_replan");
  }

  // TODO(murooka) tune this param when avoiding with obstacle_avoidance_planner
  traj_param_.center_line_width = vehicle_param_.width;

  objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>();

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleAvoidancePlanner::paramCallback, this, std::placeholders::_1));

  resetPlanning();

  self_pose_listener_.waitForFirstPose();
}

rcl_interfaces::msg::SetParametersResult ObstacleAvoidancePlanner::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // option parameter
    updateParam<bool>(
      parameters, "option.is_publishing_debug_visualization_marker",
      is_publishing_debug_visualization_marker_);
    updateParam<bool>(
      parameters, "option.is_publishing_clearance_map", is_publishing_clearance_map_);
    updateParam<bool>(
      parameters, "option.is_publishing_object_clearance_map", is_publishing_object_clearance_map_);
    updateParam<bool>(
      parameters, "option.is_publishing_area_with_objects", is_publishing_area_with_objects_);

    updateParam<bool>(parameters, "option.is_showing_debug_info", is_showing_debug_info_);
    updateParam<bool>(
      parameters, "option.is_showing_calculation_time", is_showing_calculation_time_);
    updateParam<bool>(
      parameters, "option.is_stopping_if_outside_drivable_area",
      is_stopping_if_outside_drivable_area_);

    // drivability check
    updateParam<bool>(
      parameters, "advanced.option.drivability_check.use_vehicle_circles",
      use_vehicle_circles_for_drivability_);
    if (use_vehicle_circles_for_drivability_) {
      updateParam<bool>(
        parameters, "advanced.option.drivability_check.vehicle_circles.use_manual_vehicle_circles",
        use_manual_vehicle_circles_for_drivability_);
      updateParam<int>(
        parameters, "advanced.option.drivability_check.vehicle_circles.num_for_constraints",
        vehicle_circle_constraints_num_for_drivability_);
      if (use_manual_vehicle_circles_for_drivability_) {
        updateParam<std::vector<double>>(
          parameters, "advanced.option.drivability_check.vehicle_circles.longitudinal_offsets",
          vehicle_circle_longitudinal_offsets_for_drivability_);
        updateParam<double>(
          parameters, "advanced.option.drivability_check.vehicle_circles.radius",
          vehicle_circle_radius_for_drivability_);
      } else {
        updateParam<int>(
          parameters, "advanced.option.drivability_check.vehicle_circles.num_for_radius",
          vehicle_circle_radius_num_for_drivability_);
        updateParam<double>(
          parameters, "advanced.option.drivability_check.vehicle_circles.radius_ratio",
          vehicle_circle_radius_ratio_for_drivability_);

        std::tie(
          vehicle_circle_radius_for_drivability_,
          vehicle_circle_longitudinal_offsets_for_drivability_) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_constraints_num_for_drivability_,
            vehicle_circle_radius_num_for_drivability_,
            vehicle_circle_radius_ratio_for_drivability_);
      }
    }

    updateParam<bool>(parameters, "option.enable_avoidance", enable_avoidance_);
    updateParam<bool>(parameters, "option.enable_pre_smoothing", enable_pre_smoothing_);
    updateParam<bool>(parameters, "option.skip_optimization", skip_optimization_);
    updateParam<bool>(parameters, "option.reset_prev_optimization", reset_prev_optimization_);
  }

  {  // trajectory parameter
    // common
    updateParam<int>(parameters, "common.num_sampling_points", traj_param_.num_sampling_points);
    updateParam<double>(parameters, "common.trajectory_length", traj_param_.trajectory_length);
    updateParam<double>(
      parameters, "common.forward_fixing_min_distance", traj_param_.forward_fixing_min_distance);
    updateParam<double>(
      parameters, "common.forward_fixing_min_time", traj_param_.forward_fixing_min_time);
    updateParam<double>(
      parameters, "common.backward_fixing_distance", traj_param_.backward_fixing_distance);
    updateParam<double>(
      parameters, "common.delta_arc_length_for_trajectory",
      traj_param_.delta_arc_length_for_trajectory);

    updateParam<double>(
      parameters, "common.delta_dist_threshold_for_closest_point",
      traj_param_.delta_dist_threshold_for_closest_point);
    updateParam<double>(
      parameters, "common.delta_yaw_threshold_for_closest_point",
      traj_param_.delta_yaw_threshold_for_closest_point);
    updateParam<double>(
      parameters, "common.delta_yaw_threshold_for_straight",
      traj_param_.delta_yaw_threshold_for_straight);
    updateParam<int>(
      parameters, "common.num_fix_points_for_extending", traj_param_.num_fix_points_for_extending);
    updateParam<double>(
      parameters, "common.max_dist_for_extending_end_point",
      traj_param_.max_dist_for_extending_end_point);

    // object
    updateParam<double>(
      parameters, "object.max_avoiding_ego_velocity_ms", traj_param_.max_avoiding_ego_velocity_ms);
    updateParam<double>(
      parameters, "object.max_avoiding_objects_velocity_ms",
      traj_param_.max_avoiding_objects_velocity_ms);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.unknown", traj_param_.is_avoiding_unknown);
    updateParam<bool>(parameters, "object.avoiding_object_type.car", traj_param_.is_avoiding_car);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.truck", traj_param_.is_avoiding_truck);
    updateParam<bool>(parameters, "object.avoiding_object_type.bus", traj_param_.is_avoiding_bus);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.bicycle", traj_param_.is_avoiding_bicycle);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.motorbike", traj_param_.is_avoiding_motorbike);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.pedestrian", traj_param_.is_avoiding_pedestrian);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.animal", traj_param_.is_avoiding_animal);
  }

  {  // elastic band parameter
    // common
    updateParam<int>(
      parameters, "advanced.eb.common.num_joint_buffer_points", eb_param_.num_joint_buffer_points);
    updateParam<int>(
      parameters, "advanced.eb.common.num_offset_for_begin_idx",
      eb_param_.num_offset_for_begin_idx);
    updateParam<double>(
      parameters, "advanced.eb.common.delta_arc_length_for_eb", eb_param_.delta_arc_length_for_eb);
    updateParam<int>(
      parameters, "advanced.eb.common.num_sampling_points_for_eb",
      eb_param_.num_sampling_points_for_eb);

    // clearance
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_straight_line",
      eb_param_.clearance_for_straight_line);
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_joint", eb_param_.clearance_for_joint);
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_only_smoothing",
      eb_param_.clearance_for_only_smoothing);

    // qp
    updateParam<int>(parameters, "advanced.eb.qp.max_iteration", eb_param_.qp_param.max_iteration);
    updateParam<double>(parameters, "advanced.eb.qp.eps_abs", eb_param_.qp_param.eps_abs);
    updateParam<double>(parameters, "advanced.eb.qp.eps_rel", eb_param_.qp_param.eps_rel);
  }

  {  // mpt param
    // option
    // updateParam<bool>(parameters, "mpt.option.plan_from_ego", mpt_param_.plan_from_ego);
    updateParam<bool>(
      parameters, "mpt.option.steer_limit_constraint", mpt_param_.steer_limit_constraint);
    updateParam<bool>(
      parameters, "mpt.option.fix_points_around_ego", mpt_param_.fix_points_around_ego);
    updateParam<bool>(parameters, "mpt.option.enable_warm_start", mpt_param_.enable_warm_start);
    updateParam<bool>(
      parameters, "mpt.option.enable_manual_warm_start", mpt_param_.enable_manual_warm_start);
    updateParam<int>(parameters, "mpt.option.visualize_sampling_num", mpt_visualize_sampling_num_);

    // common
    updateParam<int>(
      parameters, "mpt.common.num_curvature_sampling_points",
      mpt_param_.num_curvature_sampling_points);

    updateParam<double>(
      parameters, "mpt.common.delta_arc_length_for_mpt_points",
      mpt_param_.delta_arc_length_for_mpt_points);

    // kinematics
    double max_steer_deg = mpt_param_.max_steer_rad * 180.0 / M_PI;
    updateParam<double>(parameters, "mpt.kinematics.max_steer_deg", max_steer_deg);
    mpt_param_.max_steer_rad = max_steer_deg * M_PI / 180.0;
    updateParam<double>(
      parameters, "mpt.kinematics.optimization_center_offset",
      mpt_param_.optimization_center_offset);

    // collision_free_constraints
    updateParam<bool>(
      parameters, "advanced.mpt.collision_free_constraints.option.l_inf_norm",
      mpt_param_.l_inf_norm);
    // updateParam<bool>(
    //   parameters, "advanced.mpt.collision_free_constraints.option.two_step_soft_constraint",
    //   mpt_param_.two_step_soft_constraint);
    updateParam<bool>(
      parameters, "advanced.mpt.collision_free_constraints.option.soft_constraint",
      mpt_param_.soft_constraint);
    updateParam<bool>(
      parameters, "advanced.mpt.collision_free_constraints.option.hard_constraint",
      mpt_param_.hard_constraint);

    // vehicle_circles
    updateParam<bool>(
      parameters,
      "advanced.mpt.collision_free_constraints.vehicle_circles.use_manual_vehicle_circles",
      use_manual_vehicle_circles_for_mpt_);
    updateParam<int>(
      parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.num_for_constraints",
      vehicle_circle_constraints_num_for_mpt_);
    if (use_manual_vehicle_circles_for_mpt_) {
      updateParam<std::vector<double>>(
        parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.longitudinal_offsets",
        mpt_param_.vehicle_circle_longitudinal_offsets);
      updateParam<double>(
        parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.radius",
        mpt_param_.vehicle_circle_radius);
    } else {  // vehicle circles are calculated automatically with designated ratio
      updateParam<int>(
        parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.num_for_radius",
        vehicle_circle_radius_num_for_mpt_);
      updateParam<double>(
        parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.radius_ratio",
        vehicle_circle_radius_ratio_for_mpt_);

      std::tie(mpt_param_.vehicle_circle_radius, mpt_param_.vehicle_circle_longitudinal_offsets) =
        calcVehicleCirclesInfo(
          vehicle_param_, vehicle_circle_constraints_num_for_mpt_,
          vehicle_circle_radius_num_for_mpt_, vehicle_circle_radius_ratio_for_mpt_);
    }

    // clearance
    updateParam<double>(
      parameters, "advanced.mpt.clearance.hard_clearance_from_road",
      mpt_param_.hard_clearance_from_road);
    updateParam<double>(
      parameters, "advanced.mpt.clearance.soft_clearance_from_road",
      mpt_param_.soft_clearance_from_road);
    updateParam<double>(
      parameters, "advanced.mpt.clearance.soft_second_clearance_from_road",
      mpt_param_.soft_second_clearance_from_road);
    updateParam<double>(
      parameters, "advanced.mpt.clearance.extra_desired_clearance_from_road",
      mpt_param_.extra_desired_clearance_from_road);
    updateParam<double>(
      parameters, "advanced.mpt.clearance.clearance_from_object", mpt_param_.clearance_from_object);

    // weight
    updateParam<double>(
      parameters, "advanced.mpt.weight.soft_avoidance_weight", mpt_param_.soft_avoidance_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.soft_second_avoidance_weight",
      mpt_param_.soft_second_avoidance_weight);

    updateParam<double>(
      parameters, "advanced.mpt.weight.lat_error_weight", mpt_param_.lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.yaw_error_weight", mpt_param_.yaw_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.yaw_error_rate_weight", mpt_param_.yaw_error_rate_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.steer_input_weight", mpt_param_.steer_input_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.steer_rate_weight", mpt_param_.steer_rate_weight);

    updateParam<double>(
      parameters, "advanced.mpt.weight.obstacle_avoid_lat_error_weight",
      mpt_param_.obstacle_avoid_lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.obstacle_avoid_yaw_error_weight",
      mpt_param_.obstacle_avoid_yaw_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.obstacle_avoid_steer_input_weight",
      mpt_param_.obstacle_avoid_steer_input_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.near_objects_length", mpt_param_.near_objects_length);

    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_lat_error_weight",
      mpt_param_.terminal_lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_yaw_error_weight",
      mpt_param_.terminal_yaw_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_path_lat_error_weight",
      mpt_param_.terminal_path_lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_path_yaw_error_weight",
      mpt_param_.terminal_path_yaw_error_weight);
  }

  {  // replan
    updateParam<double>(
      parameters, "replan.max_path_shape_change_dist", max_path_shape_change_dist_for_replan_);
    updateParam<double>(
      parameters, "replan.max_ego_moving_dist_for_replan", max_ego_moving_dist_for_replan_);
    updateParam<double>(
      parameters, "replan.max_delta_time_sec_for_replan", max_delta_time_sec_for_replan_);
  }

  resetPlanning();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void ObstacleAvoidancePlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_twist_ptr_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
  current_twist_ptr_->header = msg->header;
  current_twist_ptr_->twist = msg->twist.twist;
}

void ObstacleAvoidancePlanner::objectsCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>(*msg);
}

void ObstacleAvoidancePlanner::enableAvoidanceCallback(
  const tier4_planning_msgs::msg::EnableAvoidance::SharedPtr msg)
{
  enable_avoidance_ = msg->enable_avoidance;
}

void ObstacleAvoidancePlanner::resetPlanning()
{
  RCLCPP_WARN(get_logger(), "[ObstacleAvoidancePlanner] Reset planning");

  costmap_generator_ptr_ = std::make_unique<CostmapGenerator>();

  eb_path_optimizer_ptr_ = std::make_unique<EBPathOptimizer>(
    is_showing_debug_info_, traj_param_, eb_param_, vehicle_param_);

  mpt_optimizer_ptr_ =
    std::make_unique<MPTOptimizer>(is_showing_debug_info_, traj_param_, vehicle_param_, mpt_param_);

  prev_path_points_ptr_ = nullptr;
  resetPrevOptimization();
}

void ObstacleAvoidancePlanner::resetPrevOptimization()
{
  prev_optimal_trajs_ptr_ = nullptr;
  eb_solved_count_ = 0;
}

void ObstacleAvoidancePlanner::pathCallback(
  const autoware_auto_planning_msgs::msg::Path::SharedPtr path_ptr)
{
  stop_watch_.tic(__func__);

  if (path_ptr->points.empty() || path_ptr->drivable_area.data.empty() || !current_twist_ptr_) {
    return;
  }

  current_ego_pose_ = self_pose_listener_.getCurrentPose()->pose;
  debug_data_ptr_ = std::make_shared<DebugData>();
  debug_data_ptr_->init(
    is_showing_calculation_time_, mpt_visualize_sampling_num_, current_ego_pose_,
    mpt_param_.vehicle_circle_radius, mpt_param_.vehicle_circle_longitudinal_offsets);

  // generate optimized trajectory
  const auto optimized_traj_points = generateOptimizedTrajectory(*path_ptr);

  // generate post processed trajectory
  const auto post_processed_traj_points =
    generatePostProcessedTrajectory(path_ptr->points, optimized_traj_points);

  // convert to output msg type
  auto output_traj_msg = tier4_autoware_utils::convertToTrajectory(post_processed_traj_points);
  output_traj_msg.header = path_ptr->header;

  // publish debug data
  publishDebugDataInMain(*path_ptr);

  {  // print and publish debug msg
    debug_data_ptr_->msg_stream << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n"
                                << "========================================";
    tier4_debug_msgs::msg::StringStamped debug_msg_msg;
    debug_msg_msg.stamp = get_clock()->now();
    debug_msg_msg.data = debug_data_ptr_->msg_stream.getString();
    debug_msg_pub_->publish(debug_msg_msg);
  }

  // make previous variables
  prev_path_points_ptr_ =
    std::make_unique<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(path_ptr->points);
  prev_ego_pose_ptr_ = std::make_unique<geometry_msgs::msg::Pose>(current_ego_pose_);

  traj_pub_->publish(output_traj_msg);
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
ObstacleAvoidancePlanner::generateOptimizedTrajectory(
  const autoware_auto_planning_msgs::msg::Path & path)
{
  stop_watch_.tic(__func__);

  if (reset_prev_optimization_) {
    resetPrevOptimization();
  }

  // return prev trajectory if replan is not required
  if (!checkReplan(path.points)) {
    if (prev_optimal_trajs_ptr_) {
      return prev_optimal_trajs_ptr_->model_predictive_trajectory;
    }

    return points_utils::convertToTrajectoryPoints(path.points);
  }
  latest_replanned_time_ptr_ = std::make_unique<rclcpp::Time>(this->now());

  // create clearance maps
  const CVMaps cv_maps = costmap_generator_ptr_->getMaps(
    enable_avoidance_, path, objects_ptr_->objects, traj_param_, debug_data_ptr_);

  // calculate trajectory with EB and MPT
  auto optimal_trajs = optimizeTrajectory(path, cv_maps);

  // insert 0 velocity when trajectory is over drivable area
  if (is_stopping_if_outside_drivable_area_) {
    insertZeroVelocityOutsideDrivableArea(optimal_trajs.model_predictive_trajectory, cv_maps);
  }

  publishDebugDataInOptimization(path, optimal_trajs.model_predictive_trajectory);

  // make previous trajectories
  prev_optimal_trajs_ptr_ =
    std::make_unique<Trajectories>(makePrevTrajectories(path.points, optimal_trajs));

  debug_data_ptr_->msg_stream << "  " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return optimal_trajs.model_predictive_trajectory;
}

// check if optimization is required or not.
// NOTE: previous trajectories information will be reset as well in some cases.
bool ObstacleAvoidancePlanner::checkReplan(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points)
{
  if (
    !prev_ego_pose_ptr_ || !latest_replanned_time_ptr_ || !prev_path_points_ptr_ ||
    !prev_optimal_trajs_ptr_) {
    return true;
  }

  const double max_mpt_length =
    traj_param_.num_sampling_points * mpt_param_.delta_arc_length_for_mpt_points;
  if (isPathShapeChanged(
        current_ego_pose_, path_points, prev_path_points_ptr_, max_mpt_length,
        max_path_shape_change_dist_for_replan_,
        traj_param_.delta_yaw_threshold_for_closest_point)) {
    RCLCPP_INFO(get_logger(), "Replan since path shape was changed.");
    return true;
  }

  if (isPathGoalChanged(current_twist_ptr_->twist.linear.x, path_points, prev_path_points_ptr_)) {
    RCLCPP_INFO(get_logger(), "Replan with resetting optimization since path goal was changed.");
    resetPrevOptimization();
    return true;
  }

  // For when ego pose is lost or new ego pose is designated in simulation
  const double delta_dist =
    tier4_autoware_utils::calcDistance2d(current_ego_pose_.position, prev_ego_pose_ptr_->position);
  if (delta_dist > max_ego_moving_dist_for_replan_) {
    RCLCPP_INFO(
      get_logger(),
      "Replan with resetting optimization since current ego pose is far from previous ego pose.");
    resetPrevOptimization();
    return true;
  }

  // For when ego pose moves far from trajectory
  if (!hasValidNearestPointFromEgo(current_ego_pose_, *prev_optimal_trajs_ptr_, traj_param_)) {
    RCLCPP_INFO(
      get_logger(),
      "Replan with resetting optimization since valid nearest trajectory point from ego was not "
      "found.");
    resetPrevOptimization();
    return true;
  }

  const double delta_time_sec = (this->now() - *latest_replanned_time_ptr_).seconds();
  if (delta_time_sec > max_delta_time_sec_for_replan_) {
    return true;
  }
  return false;
}

Trajectories ObstacleAvoidancePlanner::optimizeTrajectory(
  const autoware_auto_planning_msgs::msg::Path & path, const CVMaps & cv_maps)
{
  stop_watch_.tic(__func__);

  if (skip_optimization_) {
    const auto traj = points_utils::convertToTrajectoryPoints(path.points);
    Trajectories trajs;
    trajs.smoothed_trajectory = traj;
    trajs.model_predictive_trajectory = traj;
    return trajs;
  }

  // EB: smooth trajectory if enable_pre_smoothing is true
  const auto eb_traj =
    [&]() -> boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>> {
    if (enable_pre_smoothing_) {
      return eb_path_optimizer_ptr_->getEBTrajectory(
        current_ego_pose_, path, prev_optimal_trajs_ptr_, current_twist_ptr_->twist.linear.x,
        debug_data_ptr_);
    }
    return points_utils::convertToTrajectoryPoints(path.points);
  }();
  if (!eb_traj) {
    return getPrevTrajs(path.points);
  }

  // EB has to be solved twice before solving MPT with fixed points
  // since the result of EB is likely to change with/without fixing (1st/2nd EB)
  // that makes MPT fixing points worse.
  if (eb_solved_count_ < 2) {
    eb_solved_count_++;

    if (prev_optimal_trajs_ptr_) {
      prev_optimal_trajs_ptr_->model_predictive_trajectory.clear();
      prev_optimal_trajs_ptr_->mpt_ref_points.clear();
    }
  }

  // MPT: optimize trajectory to be kinematically feasible and collision free
  const auto mpt_trajs = mpt_optimizer_ptr_->getModelPredictiveTrajectory(
    enable_avoidance_, eb_traj.get(), path.points, prev_optimal_trajs_ptr_, cv_maps,
    current_ego_pose_, current_twist_ptr_->twist.linear.x, debug_data_ptr_);
  if (!mpt_trajs) {
    return getPrevTrajs(path.points);
  }

  // make trajectories, which has all optimized trajectories information
  Trajectories trajs;
  trajs.smoothed_trajectory = eb_traj.get();
  trajs.mpt_ref_points = mpt_trajs.get().ref_points;
  trajs.model_predictive_trajectory = mpt_trajs.get().mpt;

  // debug data
  debug_data_ptr_->mpt_traj = mpt_trajs.get().mpt;
  debug_data_ptr_->mpt_ref_traj =
    points_utils::convertToTrajectoryPoints(mpt_trajs.get().ref_points);
  debug_data_ptr_->eb_traj = eb_traj.get();

  debug_data_ptr_->msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return trajs;
}

Trajectories ObstacleAvoidancePlanner::getPrevTrajs(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const
{
  if (prev_optimal_trajs_ptr_) {
    return *prev_optimal_trajs_ptr_;
  }

  const auto traj = points_utils::convertToTrajectoryPoints(path_points);
  Trajectories trajs;
  trajs.smoothed_trajectory = traj;
  trajs.model_predictive_trajectory = traj;
  return trajs;
}

void ObstacleAvoidancePlanner::insertZeroVelocityOutsideDrivableArea(
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const CVMaps & cv_maps)
{
  stop_watch_.tic(__func__);

  const auto & map_info = cv_maps.map_info;
  const auto & road_clearance_map = cv_maps.clearance_map;

  const size_t nearest_idx =
    tier4_autoware_utils::findNearestIndex(traj_points, current_ego_pose_.position);

  // NOTE: Some end trajectory points will be ignored to check if outside the drivable area
  //       since these points might be outside drivable area if only end reference points have high
  //       curvature.
  const size_t end_idx = [&]() {
    const size_t enough_long_points_num =
      static_cast<size_t>(traj_param_.num_sampling_points * 0.7);
    constexpr size_t end_ignored_points_num = 5;
    if (traj_points.size() > enough_long_points_num) {
      return std::max(static_cast<size_t>(0), traj_points.size() - end_ignored_points_num);
    }
    return traj_points.size();
  }();

  for (size_t i = nearest_idx; i < end_idx; ++i) {
    const auto & traj_point = traj_points.at(i);

    // calculate the first point being outside drivable area
    const bool is_outside = [&]() {
      if (use_vehicle_circles_for_drivability_) {
        return cv_drivable_area_utils::isOutsideDrivableAreaFromCirclesFootprint(
          traj_point, road_clearance_map, map_info,
          vehicle_circle_longitudinal_offsets_for_drivability_,
          vehicle_circle_radius_for_drivability_);
      }
      return cv_drivable_area_utils::isOutsideDrivableAreaFromRectangleFootprint(
        traj_point, road_clearance_map, map_info, vehicle_param_);
    }();

    // only insert zero velocity to the first point outside drivable area
    if (is_outside) {
      traj_points[i].longitudinal_velocity_mps = 0.0;
      debug_data_ptr_->stop_pose_by_drivable_area = traj_points[i].pose;
      break;
    }
  }

  debug_data_ptr_->msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
}

void ObstacleAvoidancePlanner::publishDebugDataInOptimization(
  const autoware_auto_planning_msgs::msg::Path & path,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points)
{
  stop_watch_.tic(__func__);

  {  // publish trajectories
    auto debug_eb_traj = tier4_autoware_utils::convertToTrajectory(debug_data_ptr_->eb_traj);
    debug_eb_traj.header = path.header;
    debug_eb_traj_pub_->publish(debug_eb_traj);

    auto debug_mpt_fixed_traj =
      tier4_autoware_utils::convertToTrajectory(debug_data_ptr_->mpt_fixed_traj);
    debug_mpt_fixed_traj.header = path.header;
    debug_mpt_fixed_traj_pub_->publish(debug_mpt_fixed_traj);

    auto debug_mpt_ref_traj =
      tier4_autoware_utils::convertToTrajectory(debug_data_ptr_->mpt_ref_traj);
    debug_mpt_ref_traj.header = path.header;
    debug_mpt_ref_traj_pub_->publish(debug_mpt_ref_traj);

    auto debug_mpt_traj = tier4_autoware_utils::convertToTrajectory(debug_data_ptr_->mpt_traj);
    debug_mpt_traj.header = path.header;
    debug_mpt_traj_pub_->publish(debug_mpt_traj);
  }

  {  // publish markers
    if (is_publishing_debug_visualization_marker_) {
      stop_watch_.tic("getDebugVisualizationMarker");
      const auto & debug_marker = debug_visualization::getDebugVisualizationMarker(
        debug_data_ptr_, traj_points, vehicle_param_, false);
      debug_data_ptr_->msg_stream << "      getDebugVisualizationMarker:= "
                                  << stop_watch_.toc("getDebugVisualizationMarker") << " [ms]\n";

      stop_watch_.tic("publishDebugVisualizationMarker");
      debug_markers_pub_->publish(debug_marker);
      debug_data_ptr_->msg_stream << "      publishDebugVisualizationMarker:= "
                                  << stop_watch_.toc("publishDebugVisualizationMarker")
                                  << " [ms]\n";

      stop_watch_.tic("getDebugVisualizationWallMarker");
      const auto & debug_wall_marker =
        debug_visualization::getDebugVisualizationWallMarker(debug_data_ptr_, vehicle_param_);
      debug_data_ptr_->msg_stream << "      getDebugVisualizationWallMarker:= "
                                  << stop_watch_.toc("getDebugVisualizationWallMarker")
                                  << " [ms]\n";

      stop_watch_.tic("publishDebugVisualizationWallMarker");
      debug_wall_markers_pub_->publish(debug_wall_marker);
      debug_data_ptr_->msg_stream << "      publishDebugVisualizationWallMarker:= "
                                  << stop_watch_.toc("publishDebugVisualizationWallMarker")
                                  << " [ms]\n";
    }
  }

  debug_data_ptr_->msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
}

Trajectories ObstacleAvoidancePlanner::makePrevTrajectories(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const Trajectories & trajs)
{
  stop_watch_.tic(__func__);

  const auto post_processed_smoothed_traj =
    generatePostProcessedTrajectory(path_points, trajs.smoothed_trajectory);

  // TODO(murooka) generatePoseProcessedTrajectory may be too large
  Trajectories trajectories;
  trajectories.smoothed_trajectory = post_processed_smoothed_traj;
  trajectories.mpt_ref_points = trajs.mpt_ref_points;
  trajectories.model_predictive_trajectory = trajs.model_predictive_trajectory;

  debug_data_ptr_->msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";

  return trajectories;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
ObstacleAvoidancePlanner::generatePostProcessedTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_traj_points)
{
  stop_watch_.tic(__func__);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> trajectory_points;
  if (path_points.empty()) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_point;
    tmp_point.pose = current_ego_pose_;
    tmp_point.longitudinal_velocity_mps = 0.0;
    trajectory_points.push_back(tmp_point);
    return trajectory_points;
  }
  if (optimized_traj_points.empty()) {
    trajectory_points = points_utils::convertToTrajectoryPoints(path_points);
    return trajectory_points;
  }

  // calculate extended trajectory that connects to optimized trajectory smoothly
  const auto extended_traj_points = getExtendedTrajectory(path_points, optimized_traj_points);

  // concat trajectories
  const auto full_traj_points =
    points_utils::concatTrajectory(optimized_traj_points, extended_traj_points);

  // NOTE: fine_traj_points has no velocity information
  const auto fine_traj_points = generateFineTrajectoryPoints(path_points, full_traj_points);

  const auto fine_traj_points_with_vel =
    alignVelocity(fine_traj_points, path_points, full_traj_points);

  debug_data_ptr_->msg_stream << "  " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return fine_traj_points_with_vel;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
ObstacleAvoidancePlanner::getExtendedTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points)
{
  stop_watch_.tic(__func__);

  assert(!path_points.empty());

  const double accum_arc_length = tier4_autoware_utils::calcArcLength(optimized_points);
  if (accum_arc_length > traj_param_.trajectory_length) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(),
      "[Avoidance] Not extend trajectory");
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }

  // calculate end idx of optimized points on path points
  const auto opt_end_path_idx = tier4_autoware_utils::findNearestIndex(
    path_points, optimized_points.back().pose, std::numeric_limits<double>::max(),
    traj_param_.delta_yaw_threshold_for_closest_point);
  if (!opt_end_path_idx) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(),
      "[Avoidance] Not extend trajectory since could not find nearest idx from last opt point");
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }

  auto extended_traj_points =
    [&]() -> std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> {
    constexpr double non_fixed_traj_length = 5.0;  // TODO(murooka) may be better to tune
    const size_t non_fixed_begin_path_idx = opt_end_path_idx.get();
    const size_t non_fixed_end_path_idx =
      points_utils::findForwardIndex(path_points, non_fixed_begin_path_idx, non_fixed_traj_length);

    if (
      non_fixed_begin_path_idx == path_points.size() - 1 ||
      non_fixed_begin_path_idx == non_fixed_end_path_idx) {
      if (points_utils::isNearLastPathPoint(
            optimized_points.back(), path_points, traj_param_.max_dist_for_extending_end_point,
            traj_param_.delta_yaw_threshold_for_closest_point)) {
        return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
      }
      const auto last_traj_point = points_utils::convertToTrajectoryPoint(path_points.back());
      return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{last_traj_point};
    } else if (non_fixed_end_path_idx == path_points.size() - 1) {
      // no need to connect smoothly since extended trajectory length is short enough
      const auto last_traj_point = points_utils::convertToTrajectoryPoint(path_points.back());
      return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{last_traj_point};
    }

    // define non_fixed/fixed_traj_points
    const auto begin_point = optimized_points.back();
    const auto end_point =
      points_utils::convertToTrajectoryPoint(path_points.at(non_fixed_end_path_idx));
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> non_fixed_traj_points{
      begin_point, end_point};
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> fixed_path_points{
      path_points.begin() + non_fixed_end_path_idx + 1, path_points.end()};
    const auto fixed_traj_points = points_utils::convertToTrajectoryPoints(fixed_path_points);

    // spline interpolation to two traj points with end diff constraints
    const double begin_yaw = tf2::getYaw(begin_point.pose.orientation);
    const double end_yaw = tf2::getYaw(end_point.pose.orientation);
    const auto interpolated_non_fixed_traj_points =
      interpolation_utils::getConnectedInterpolatedPoints(
        non_fixed_traj_points, eb_param_.delta_arc_length_for_eb, begin_yaw, end_yaw);

    // concat interpolated_non_fixed and fixed traj points
    auto extended_points = interpolated_non_fixed_traj_points;
    extended_points.insert(
      extended_points.end(), fixed_traj_points.begin(), fixed_traj_points.end());

    debug_data_ptr_->extended_non_fixed_traj = interpolated_non_fixed_traj_points;
    debug_data_ptr_->extended_fixed_traj = fixed_traj_points;

    return extended_points;
  }();

  // NOTE: Extended points will be concatenated with optimized points.
  //       Then, minimum velocity of each point is chosen from concatenated points or path points
  //       since optimized points has zero velocity outside drivable area
  constexpr double large_velocity = 1e4;
  for (auto & point : extended_traj_points) {
    point.longitudinal_velocity_mps = large_velocity;
  }

  debug_data_ptr_->msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return extended_traj_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
ObstacleAvoidancePlanner::generateFineTrajectoryPoints(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points) const
{
  stop_watch_.tic(__func__);

  // interpolate x and y
  auto interpolated_traj_points = interpolation_utils::getInterpolatedTrajectoryPoints(
    traj_points, traj_param_.delta_arc_length_for_trajectory);

  // calculate yaw from x and y
  // NOTE: We do not use spline interpolation to yaw in behavior path since the yaw is unstable.
  //       Currently this implementation is removed since this calculation is heavy (~20ms)
  // fillYawInTrajectoryPoint(interpolated_traj_points);

  // compensate last pose
  points_utils::compensateLastPose(
    path_points.back(), interpolated_traj_points, traj_param_.max_dist_for_extending_end_point,
    traj_param_.delta_yaw_threshold_for_closest_point);

  debug_data_ptr_->msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";

  return interpolated_traj_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
ObstacleAvoidancePlanner::alignVelocity(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & fine_traj_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points) const
{
  stop_watch_.tic(__func__);

  // search zero velocity index of fine_traj_points
  const size_t zero_vel_fine_traj_idx = [&]() {
    const size_t zero_vel_path_idx = searchExtendedZeroVelocityIndex(fine_traj_points, path_points);
    const size_t zero_vel_traj_idx =
      searchExtendedZeroVelocityIndex(fine_traj_points, traj_points);  // for outside drivable area

    return std::min(zero_vel_path_idx, zero_vel_traj_idx);
  }();

  auto fine_traj_points_with_vel = fine_traj_points;
  size_t prev_begin_idx = 0;
  for (size_t i = 0; i < fine_traj_points_with_vel.size(); ++i) {
    const auto truncated_points = points_utils::clipForwardPoints(path_points, prev_begin_idx, 5.0);

    const auto & target_pos = fine_traj_points_with_vel[i].pose.position;
    const size_t closest_seg_idx =
      tier4_autoware_utils::findNearestSegmentIndex(truncated_points, target_pos);

    // lerp z
    fine_traj_points_with_vel[i].pose.position.z =
      lerpPoseZ(truncated_points, target_pos, closest_seg_idx);

    // lerp vx
    const double target_vel = lerpTwistX(truncated_points, target_pos, closest_seg_idx);
    if (i >= zero_vel_fine_traj_idx) {
      fine_traj_points_with_vel[i].longitudinal_velocity_mps = 0.0;
    } else if (target_vel < 1e-6) {  // NOTE: velocity may be negative due to linear interpolation
      const auto prev_idx = std::max(static_cast<int>(i) - 1, 0);
      fine_traj_points_with_vel[i].longitudinal_velocity_mps =
        fine_traj_points_with_vel[prev_idx].longitudinal_velocity_mps;
    } else {
      fine_traj_points_with_vel[i].longitudinal_velocity_mps = target_vel;
    }

    // NOTE: closest_seg_idx is for the clipped trajectory. This operation must be "+=".
    prev_begin_idx += closest_seg_idx;
  }

  debug_data_ptr_->msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return fine_traj_points_with_vel;
}

void ObstacleAvoidancePlanner::publishDebugDataInMain(
  const autoware_auto_planning_msgs::msg::Path & path) const
{
  stop_watch_.tic(__func__);

  {  // publish trajectories
    auto debug_extended_fixed_traj =
      tier4_autoware_utils::convertToTrajectory(debug_data_ptr_->extended_fixed_traj);
    debug_extended_fixed_traj.header = path.header;
    debug_extended_fixed_traj_pub_->publish(debug_extended_fixed_traj);

    auto debug_extended_non_fixed_traj =
      tier4_autoware_utils::convertToTrajectory(debug_data_ptr_->extended_non_fixed_traj);
    debug_extended_non_fixed_traj.header = path.header;
    debug_extended_non_fixed_traj_pub_->publish(debug_extended_non_fixed_traj);
  }

  {  // publish clearance map
    stop_watch_.tic("publishClearanceMap");

    if (is_publishing_area_with_objects_) {  // false
      debug_area_with_objects_pub_->publish(debug_visualization::getDebugCostmap(
        debug_data_ptr_->area_with_objects_map, path.drivable_area));
    }
    if (is_publishing_object_clearance_map_) {  // false
      debug_object_clearance_map_pub_->publish(debug_visualization::getDebugCostmap(
        debug_data_ptr_->only_object_clearance_map, path.drivable_area));
    }
    if (is_publishing_clearance_map_) {  // true
      debug_clearance_map_pub_->publish(
        debug_visualization::getDebugCostmap(debug_data_ptr_->clearance_map, path.drivable_area));
    }
    debug_data_ptr_->msg_stream << "    getDebugCostMap * 3:= "
                                << stop_watch_.toc("publishClearanceMap") << " [ms]\n";
  }

  debug_data_ptr_->msg_stream << "  " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ObstacleAvoidancePlanner)
