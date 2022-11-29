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
#include "motion_utils/motion_utils.hpp"
#include "obstacle_avoidance_planner/utils/cv_utils.hpp"
#include "obstacle_avoidance_planner/utils/debug_utils.hpp"
#include "obstacle_avoidance_planner/utils/utils.hpp"
#include "rclcpp/time.hpp"
#include "tf2/utils.h"
#include "tier4_autoware_utils/ros/update_param.hpp"
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
std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesInfo(
  const VehicleParam & vehicle_param, const size_t circle_num, const double rear_radius_ratio,
  const double front_radius_ratio)
{
  std::vector<double> longitudinal_offsets;
  std::vector<double> radiuses;

  {  // 1st circle (rear)
    longitudinal_offsets.push_back(-vehicle_param.rear_overhang);
    radiuses.push_back(vehicle_param.width / 2.0 * rear_radius_ratio);
  }

  {  // 2nd circle (front)
    const double radius = std::hypot(
      vehicle_param.length / static_cast<double>(circle_num) / 2.0, vehicle_param.width / 2.0);

    const double unit_lon_length = vehicle_param.length / static_cast<double>(circle_num);
    const double longitudinal_offset =
      unit_lon_length / 2.0 + unit_lon_length * (circle_num - 1) - vehicle_param.rear_overhang;

    longitudinal_offsets.push_back(longitudinal_offset);
    radiuses.push_back(radius * front_radius_ratio);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesInfo(
  const VehicleParam & vehicle_param, const size_t circle_num, const double radius_ratio)
{
  std::vector<double> longitudinal_offsets;
  std::vector<double> radiuses;

  const double radius =
    std::hypot(
      vehicle_param.length / static_cast<double>(circle_num) / 2.0, vehicle_param.width / 2.0) *
    radius_ratio;
  const double unit_lon_length = vehicle_param.length / static_cast<double>(circle_num);

  for (size_t i = 0; i < circle_num; ++i) {
    longitudinal_offsets.push_back(
      unit_lon_length / 2.0 + unit_lon_length * i - vehicle_param.rear_overhang);
    radiuses.push_back(radius);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesInfoByBicycleModel(
  const VehicleParam & vehicle_param, const size_t circle_num, const double rear_radius_ratio,
  const double front_radius_ratio)
{
  std::vector<double> longitudinal_offsets;
  std::vector<double> radiuses;

  {  // 1st circle (rear wheel)
    longitudinal_offsets.push_back(0.0);
    radiuses.push_back(vehicle_param.width / 2.0 * rear_radius_ratio);
  }

  {  // 2nd circle (front wheel)
    const double radius = std::hypot(
      vehicle_param.length / static_cast<double>(circle_num) / 2.0, vehicle_param.width / 2.0);

    const double unit_lon_length = vehicle_param.length / static_cast<double>(circle_num);
    const double longitudinal_offset =
      unit_lon_length / 2.0 + unit_lon_length * (circle_num - 1) - vehicle_param.rear_overhang;

    longitudinal_offsets.push_back(longitudinal_offset);
    radiuses.push_back(radius * front_radius_ratio);
  }

  return {radiuses, longitudinal_offsets};
}

[[maybe_unused]] void fillYawInTrajectoryPoint(std::vector<TrajectoryPoint> & traj_points)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & traj_point : traj_points) {
    points.push_back(traj_point.pose.position);
  }
  const auto yaw_vec = interpolation::splineYawFromPoints(points);

  for (size_t i = 0; i < traj_points.size(); ++i) {
    traj_points.at(i).pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(yaw_vec.at(i));
  }
}

template <class T>
[[maybe_unused]] size_t findNearestIndexWithSoftYawConstraints(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx_optional =
    motion_utils::findNearestIndex(points, pose, dist_threshold, yaw_threshold);
  return nearest_idx_optional ? *nearest_idx_optional
                              : motion_utils::findNearestIndex(points, pose.position);
}

template <>
[[maybe_unused]] size_t findNearestIndexWithSoftYawConstraints(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold, const double yaw_threshold)
{
  const auto points_with_yaw = points_utils::convertToPosesWithYawEstimation(points);

  return findNearestIndexWithSoftYawConstraints(
    points_with_yaw, pose, dist_threshold, yaw_threshold);
}

template <class T>
[[maybe_unused]] size_t findNearestSegmentIndexWithSoftYawConstraints(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx_optional =
    motion_utils::findNearestSegmentIndex(points, pose, dist_threshold, yaw_threshold);
  return nearest_idx_optional ? *nearest_idx_optional
                              : motion_utils::findNearestSegmentIndex(points, pose.position);
}

template <>
[[maybe_unused]] size_t findNearestSegmentIndexWithSoftYawConstraints(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold, const double yaw_threshold)
{
  const auto points_with_yaw = points_utils::convertToPosesWithYawEstimation(points);

  return findNearestSegmentIndexWithSoftYawConstraints(
    points_with_yaw, pose, dist_threshold, yaw_threshold);
}

template <typename T1, typename T2>
size_t searchExtendedZeroVelocityIndex(
  const std::vector<T1> & fine_points, const std::vector<T2> & vel_points,
  const double yaw_threshold)
{
  const auto opt_zero_vel_idx = motion_utils::searchZeroVelocityIndex(vel_points);
  const size_t zero_vel_idx = opt_zero_vel_idx ? opt_zero_vel_idx.get() : vel_points.size() - 1;
  return findNearestIndexWithSoftYawConstraints(
    fine_points, vel_points.at(zero_vel_idx).pose, std::numeric_limits<double>::max(),
    yaw_threshold);
}

Trajectory createTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const std_msgs::msg::Header & header)
{
  auto traj = motion_utils::convertToTrajectory(traj_points);
  traj.header = header;

  return traj;
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = motion_utils::resampleTrajectory(traj, interval);

  // convert Trajectory to std::vector<TrajectoryPoint>
  std::vector<TrajectoryPoint> resampled_traj_points;
  for (const auto & point : resampled_traj.points) {
    resampled_traj_points.push_back(point);
  }

  return resampled_traj_points;
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
  traj_pub_ = create_publisher<Trajectory>("~/output/path", 1);

  // debug publisher
  debug_eb_traj_pub_ = create_publisher<Trajectory>("~/debug/eb_trajectory", durable_qos);
  debug_extended_fixed_traj_pub_ = create_publisher<Trajectory>("~/debug/extended_fixed_traj", 1);
  debug_extended_non_fixed_traj_pub_ =
    create_publisher<Trajectory>("~/debug/extended_non_fixed_traj", 1);
  debug_mpt_fixed_traj_pub_ = create_publisher<Trajectory>("~/debug/mpt_fixed_traj", 1);
  debug_mpt_ref_traj_pub_ = create_publisher<Trajectory>("~/debug/mpt_ref_traj", 1);
  debug_mpt_traj_pub_ = create_publisher<Trajectory>("~/debug/mpt_traj", 1);
  debug_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", durable_qos);
  debug_wall_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/virtual_wall", durable_qos);
  debug_clearance_map_pub_ = create_publisher<OccupancyGrid>("~/debug/clearance_map", durable_qos);
  debug_object_clearance_map_pub_ =
    create_publisher<OccupancyGrid>("~/debug/object_clearance_map", durable_qos);
  debug_area_with_objects_pub_ =
    create_publisher<OccupancyGrid>("~/debug/area_with_objects", durable_qos);
  debug_msg_pub_ =
    create_publisher<tier4_debug_msgs::msg::StringStamped>("~/debug/calculation_time", 1);

  // subscriber
  path_sub_ = create_subscription<Path>(
    "~/input/path", rclcpp::QoS{1},
    std::bind(&ObstacleAvoidancePlanner::onPath, this, std::placeholders::_1));
  odom_sub_ = create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&ObstacleAvoidancePlanner::onOdometry, this, std::placeholders::_1));
  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{10},
    std::bind(&ObstacleAvoidancePlanner::onObjects, this, std::placeholders::_1));
  is_avoidance_sub_ = create_subscription<tier4_planning_msgs::msg::EnableAvoidance>(
    "/planning/scenario_planning/lane_driving/obstacle_avoidance_approval", rclcpp::QoS{10},
    std::bind(&ObstacleAvoidancePlanner::onEnableAvoidance, this, std::placeholders::_1));

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  {  // vehicle param
    vehicle_param_ = VehicleParam{};
    vehicle_param_.width = vehicle_info.vehicle_width_m;
    vehicle_param_.length = vehicle_info.vehicle_length_m;
    vehicle_param_.wheelbase = vehicle_info.wheel_base_m;
    vehicle_param_.rear_overhang = vehicle_info.rear_overhang_m;
    vehicle_param_.front_overhang = vehicle_info.front_overhang_m;
    vehicle_param_.right_overhang = vehicle_info.right_overhang_m;
    vehicle_param_.left_overhang = vehicle_info.left_overhang_m;
    vehicle_param_.wheel_tread = vehicle_info.wheel_tread_m;
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
    traj_param_.non_fixed_trajectory_length =
      declare_parameter<double>("common.non_fixed_trajectory_length");

    traj_param_.enable_clipping_fixed_traj =
      declare_parameter<bool>("common.enable_clipping_fixed_traj");

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

    // ego nearest search
    traj_param_.ego_nearest_dist_threshold =
      declare_parameter<double>("ego_nearest_dist_threshold");
    traj_param_.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");
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
    mpt_param_.plan_from_ego = declare_parameter<bool>("mpt.option.plan_from_ego");
    mpt_param_.max_plan_from_ego_length =
      declare_parameter<double>("mpt.option.max_plan_from_ego_length");
    mpt_param_.steer_limit_constraint =
      declare_parameter<bool>("mpt.option.steer_limit_constraint");
    mpt_param_.fix_points_around_ego = declare_parameter<bool>("mpt.option.fix_points_around_ego");
    mpt_param_.enable_warm_start = declare_parameter<bool>("mpt.option.enable_warm_start");
    mpt_param_.enable_manual_warm_start =
      declare_parameter<bool>("mpt.option.enable_manual_warm_start");
    mpt_visualize_sampling_num_ = declare_parameter<int>("mpt.option.visualize_sampling_num");
    mpt_param_.is_fixed_point_single = declare_parameter<bool>("mpt.option.is_fixed_point_single");

    // common
    mpt_param_.num_curvature_sampling_points =
      declare_parameter<int>("mpt.common.num_curvature_sampling_points");

    mpt_param_.delta_arc_length_for_mpt_points =
      declare_parameter<double>("mpt.common.delta_arc_length_for_mpt_points");

    // kinematics
    mpt_param_.max_steer_rad = vehicle_info.max_steer_angle_rad;

    // By default, optimization_center_offset will be vehicle_info.wheel_base * 0.8
    // The 0.8 scale is adopted as it performed the best.
    constexpr double default_wheelbase_ratio = 0.8;
    mpt_param_.optimization_center_offset = declare_parameter<double>(
      "mpt.kinematics.optimization_center_offset",
      vehicle_param_.wheelbase * default_wheelbase_ratio);

    // bounds search
    mpt_param_.bounds_search_widths =
      declare_parameter<std::vector<double>>("advanced.mpt.bounds_search_widths");

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

    {  // vehicle_circles
       // NOTE: Vehicle shape for collision free constraints is considered as a set of circles
      vehicle_circle_method_ = declare_parameter<std::string>(
        "advanced.mpt.collision_free_constraints.vehicle_circles.method");

      if (vehicle_circle_method_ == "uniform_circle") {
        vehicle_circle_num_for_calculation_ = declare_parameter<int>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.num");
        vehicle_circle_radius_ratios_.push_back(declare_parameter<double>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio"));

        std::tie(
          mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_num_for_calculation_,
            vehicle_circle_radius_ratios_.front());
      } else if (vehicle_circle_method_ == "rear_drive") {
        vehicle_circle_num_for_calculation_ = declare_parameter<int>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.num_for_calculation");

        vehicle_circle_radius_ratios_.push_back(declare_parameter<double>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.rear_radius_ratio"));
        vehicle_circle_radius_ratios_.push_back(declare_parameter<double>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.front_radius_ratio"));

        std::tie(
          mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_num_for_calculation_,
            vehicle_circle_radius_ratios_.front(), vehicle_circle_radius_ratios_.back());
      } else if (vehicle_circle_method_ == "bicycle_model") {
        vehicle_circle_num_for_calculation_ = declare_parameter<int>(
          "advanced.mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_"
          "calculation");

        vehicle_circle_radius_ratios_.push_back(
          declare_parameter<double>("advanced.mpt.collision_free_constraints.vehicle_circles."
                                    "bicycle_model.rear_radius_ratio"));
        vehicle_circle_radius_ratios_.push_back(
          declare_parameter<double>("advanced.mpt.collision_free_constraints.vehicle_circles."
                                    "bicycle_model.front_radius_ratio"));

        std::tie(
          mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
          calcVehicleCirclesInfoByBicycleModel(
            vehicle_param_, vehicle_circle_num_for_calculation_,
            vehicle_circle_radius_ratios_.front(), vehicle_circle_radius_ratios_.back());
      } else {
        throw std::invalid_argument(
          "advanced.mpt.collision_free_constraints.vehicle_circles.num parameter is invalid.");
      }
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

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleAvoidancePlanner::onParam, this, std::placeholders::_1));

  resetPlanning();

  self_pose_listener_.waitForFirstPose();
}

rcl_interfaces::msg::SetParametersResult ObstacleAvoidancePlanner::onParam(
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
    updateParam<bool>(parameters, "mpt.option.plan_from_ego", mpt_param_.plan_from_ego);
    updateParam<double>(
      parameters, "mpt.option.max_plan_from_ego_length", mpt_param_.max_plan_from_ego_length);
    updateParam<bool>(
      parameters, "mpt.option.steer_limit_constraint", mpt_param_.steer_limit_constraint);
    updateParam<bool>(
      parameters, "mpt.option.fix_points_around_ego", mpt_param_.fix_points_around_ego);
    updateParam<bool>(parameters, "mpt.option.enable_warm_start", mpt_param_.enable_warm_start);
    updateParam<bool>(
      parameters, "mpt.option.enable_manual_warm_start", mpt_param_.enable_manual_warm_start);
    updateParam<int>(parameters, "mpt.option.visualize_sampling_num", mpt_visualize_sampling_num_);
    updateParam<bool>(
      parameters, "mpt.option.option.is_fixed_point_single", mpt_param_.is_fixed_point_single);

    // common
    updateParam<int>(
      parameters, "mpt.common.num_curvature_sampling_points",
      mpt_param_.num_curvature_sampling_points);

    updateParam<double>(
      parameters, "mpt.common.delta_arc_length_for_mpt_points",
      mpt_param_.delta_arc_length_for_mpt_points);

    // kinematics
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

    {  // vehicle_circles
      // NOTE: Changing method is not supported
      // updateParam<std::string>(
      //   parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.method",
      //   vehicle_circle_method_);

      if (vehicle_circle_method_ == "uniform_circle") {
        updateParam<int>(
          parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.num",
          vehicle_circle_num_for_calculation_);
        updateParam<double>(
          parameters,
          "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio",
          vehicle_circle_radius_ratios_.front());

        std::tie(
          mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_num_for_calculation_,
            vehicle_circle_radius_ratios_.front());
      } else if (vehicle_circle_method_ == "rear_drive") {
        updateParam<int>(
          parameters,
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.num_for_calculation",
          vehicle_circle_num_for_calculation_);

        updateParam<double>(
          parameters,
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.rear_radius_ratio",
          vehicle_circle_radius_ratios_.front());

        updateParam<double>(
          parameters,
          "advanced.mpt.collision_free_constraints.vehicle_circles.rear_drive.front_radius_ratio",
          vehicle_circle_radius_ratios_.back());

        std::tie(
          mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
          calcVehicleCirclesInfo(
            vehicle_param_, vehicle_circle_num_for_calculation_,
            vehicle_circle_radius_ratios_.front(), vehicle_circle_radius_ratios_.back());
      } else {
        throw std::invalid_argument(
          "advanced.mpt.collision_free_constraints.vehicle_circles.num parameter is invalid.");
      }
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

void ObstacleAvoidancePlanner::onOdometry(const Odometry::SharedPtr msg)
{
  current_twist_ptr_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
  current_twist_ptr_->header = msg->header;
  current_twist_ptr_->twist = msg->twist.twist;
}

void ObstacleAvoidancePlanner::onObjects(const PredictedObjects::SharedPtr msg)
{
  objects_ptr_ = std::make_unique<PredictedObjects>(*msg);
}

void ObstacleAvoidancePlanner::onEnableAvoidance(
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

void ObstacleAvoidancePlanner::onPath(const Path::SharedPtr path_ptr)
{
  stop_watch_.tic(__func__);

  if (
    path_ptr->points.empty() || path_ptr->drivable_area.data.empty() || !current_twist_ptr_ ||
    !objects_ptr_) {
    return;
  }

  // create planner data
  PlannerData planner_data;
  planner_data.path = *path_ptr;
  planner_data.ego_pose = self_pose_listener_.getCurrentPose()->pose;
  planner_data.ego_vel = current_twist_ptr_->twist.linear.x;
  planner_data.objects = objects_ptr_->objects;

  debug_data_ = DebugData();
  debug_data_.init(
    is_showing_calculation_time_, mpt_visualize_sampling_num_, planner_data.ego_pose,
    mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets);

  const auto output_traj_msg = generateTrajectory(planner_data);

  // publish debug data
  publishDebugDataInMain(*path_ptr);

  {  // print and publish debug msg
    debug_data_.msg_stream << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n"
                           << "========================================";
    tier4_debug_msgs::msg::StringStamped debug_msg_msg;
    debug_msg_msg.stamp = get_clock()->now();
    debug_msg_msg.data = debug_data_.msg_stream.getString();
    debug_msg_pub_->publish(debug_msg_msg);
  }

  // make previous variables
  prev_path_points_ptr_ = std::make_unique<std::vector<PathPoint>>(path_ptr->points);
  prev_ego_pose_ptr_ = std::make_unique<geometry_msgs::msg::Pose>(planner_data.ego_pose);

  traj_pub_->publish(output_traj_msg);
}

Trajectory ObstacleAvoidancePlanner::generateTrajectory(const PlannerData & planner_data)
{
  const auto & p = planner_data;

  // TODO(someone): support backward path
  const auto is_driving_forward = motion_utils::isDrivingForward(p.path.points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.get() : is_driving_forward_;
  if (!is_driving_forward_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[ObstacleAvoidancePlanner] Backward path is NOT supported. Just converting path to "
      "trajectory");

    const auto traj_points = points_utils::convertToTrajectoryPoints(p.path.points);
    return createTrajectory(traj_points, p.path.header);
  }

  // generate optimized trajectory
  const auto optimized_traj_points = generateOptimizedTrajectory(planner_data);
  // generate post processed trajectory
  const auto post_processed_traj_points =
    generatePostProcessedTrajectory(p.path.points, optimized_traj_points, planner_data);

  // convert to output msg type
  return createTrajectory(post_processed_traj_points, p.path.header);
}

std::vector<TrajectoryPoint> ObstacleAvoidancePlanner::generateOptimizedTrajectory(
  const PlannerData & planner_data)
{
  stop_watch_.tic(__func__);

  if (reset_prev_optimization_) {
    resetPrevOptimization();
  }

  const auto & path = planner_data.path;

  // return prev trajectory if replan is not required
  if (!checkReplan(planner_data)) {
    if (prev_optimal_trajs_ptr_) {
      return prev_optimal_trajs_ptr_->model_predictive_trajectory;
    }

    return points_utils::convertToTrajectoryPoints(path.points);
  }
  prev_replanned_time_ptr_ = std::make_unique<rclcpp::Time>(this->now());

  // create clearance maps
  const CVMaps cv_maps = costmap_generator_ptr_->getMaps(
    enable_avoidance_, path, planner_data.objects, traj_param_, debug_data_);

  // calculate trajectory with EB and MPT
  auto optimal_trajs = optimizeTrajectory(planner_data, cv_maps);

  // calculate velocity
  // NOTE: Velocity is not considered in optimization.
  calcVelocity(path.points, optimal_trajs.model_predictive_trajectory);

  // insert 0 velocity when trajectory is over drivable area
  if (is_stopping_if_outside_drivable_area_) {
    insertZeroVelocityOutsideDrivableArea(
      planner_data, optimal_trajs.model_predictive_trajectory, cv_maps);
  }

  publishDebugDataInOptimization(planner_data, optimal_trajs.model_predictive_trajectory);

  // make previous trajectories
  prev_optimal_trajs_ptr_ =
    std::make_unique<Trajectories>(makePrevTrajectories(path.points, optimal_trajs, planner_data));

  debug_data_.msg_stream << "  " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return optimal_trajs.model_predictive_trajectory;
}

// check if optimization is required or not.
// NOTE: previous trajectories information will be reset as well in some cases.
bool ObstacleAvoidancePlanner::checkReplan(const PlannerData & planner_data)
{
  const auto & p = planner_data;

  if (
    !prev_ego_pose_ptr_ || !prev_replanned_time_ptr_ || !prev_path_points_ptr_ ||
    !prev_optimal_trajs_ptr_) {
    return true;
  }

  if (prev_optimal_trajs_ptr_->model_predictive_trajectory.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "Replan with resetting optimization since previous optimized trajectory is empty.");
    resetPrevOptimization();
    return true;
  }

  if (isPathShapeChanged(p)) {
    RCLCPP_INFO(get_logger(), "Replan with resetting optimization since path shape was changed.");
    resetPrevOptimization();
    return true;
  }

  if (isPathGoalChanged(p)) {
    RCLCPP_INFO(get_logger(), "Replan with resetting optimization since path goal was changed.");
    resetPrevOptimization();
    return true;
  }

  // For when ego pose is lost or new ego pose is designated in simulation
  const double delta_dist =
    tier4_autoware_utils::calcDistance2d(p.ego_pose, prev_ego_pose_ptr_->position);
  if (delta_dist > max_ego_moving_dist_for_replan_) {
    RCLCPP_INFO(
      get_logger(),
      "Replan with resetting optimization since current ego pose is far from previous ego pose.");
    resetPrevOptimization();
    return true;
  }

  // For when ego pose moves far from trajectory
  if (!isEgoNearToPrevTrajectory(p.ego_pose)) {
    RCLCPP_INFO(
      get_logger(),
      "Replan with resetting optimization since valid nearest trajectory point from ego was not "
      "found.");
    resetPrevOptimization();
    return true;
  }

  const double delta_time_sec = (this->now() - *prev_replanned_time_ptr_).seconds();
  if (delta_time_sec > max_delta_time_sec_for_replan_) {
    return true;
  }
  return false;
}

bool ObstacleAvoidancePlanner::isPathShapeChanged(const PlannerData & planner_data)
{
  if (!prev_path_points_ptr_) {
    return false;
  }

  const auto & p = planner_data;

  const double max_mpt_length =
    traj_param_.num_sampling_points * mpt_param_.delta_arc_length_for_mpt_points;

  // truncate prev points from ego pose to fixed end points
  const auto prev_begin_idx = findEgoNearestIndex(*prev_path_points_ptr_, p.ego_pose);
  const auto truncated_prev_points =
    points_utils::clipForwardPoints(*prev_path_points_ptr_, prev_begin_idx, max_mpt_length);

  // truncate points from ego pose to fixed end points
  const auto begin_idx = findEgoNearestIndex(p.path.points, p.ego_pose);
  const auto truncated_points =
    points_utils::clipForwardPoints(p.path.points, begin_idx, max_mpt_length);

  // guard for lateral offset
  if (truncated_prev_points.size() < 2 || truncated_points.size() < 2) {
    return false;
  }

  // calculate lateral deviations between truncated path_points and prev_path_points
  for (const auto & prev_point : truncated_prev_points) {
    const double dist =
      std::abs(motion_utils::calcLateralOffset(truncated_points, prev_point.pose.position));
    if (dist > max_path_shape_change_dist_for_replan_) {
      return true;
    }
  }

  return false;
}

bool ObstacleAvoidancePlanner::isPathGoalChanged(const PlannerData & planner_data)
{
  const auto & p = planner_data;

  if (!prev_path_points_ptr_) {
    return false;
  }

  constexpr double min_vel = 1e-3;
  if (std::abs(p.ego_vel) > min_vel) {
    return false;
  }

  // NOTE: Path may be cropped and does not contain the goal.
  // Therefore we set a large value to distance threshold.
  constexpr double max_goal_moving_dist = 1.0;
  const double goal_moving_dist =
    tier4_autoware_utils::calcDistance2d(p.path.points.back(), prev_path_points_ptr_->back());
  if (goal_moving_dist < max_goal_moving_dist) {
    return false;
  }

  return true;
}

bool ObstacleAvoidancePlanner::isEgoNearToPrevTrajectory(const geometry_msgs::msg::Pose & ego_pose)
{
  const auto & traj_points = prev_optimal_trajs_ptr_->model_predictive_trajectory;

  const auto resampled_traj_points =
    resampleTrajectoryPoints(traj_points, traj_param_.delta_arc_length_for_trajectory);
  const auto opt_nearest_idx = motion_utils::findNearestIndex(
    resampled_traj_points, ego_pose, traj_param_.delta_dist_threshold_for_closest_point,
    traj_param_.delta_yaw_threshold_for_closest_point);

  if (!opt_nearest_idx) {
    return false;
  }
  return true;
}

Trajectories ObstacleAvoidancePlanner::optimizeTrajectory(
  const PlannerData & planner_data, const CVMaps & cv_maps)
{
  stop_watch_.tic(__func__);

  const auto & p = planner_data;

  if (skip_optimization_) {
    const auto traj = points_utils::convertToTrajectoryPoints(p.path.points);
    Trajectories trajs;
    trajs.smoothed_trajectory = traj;
    trajs.model_predictive_trajectory = traj;
    return trajs;
  }

  // EB: smooth trajectory if enable_pre_smoothing is true
  const auto eb_traj = [&]() -> boost::optional<std::vector<TrajectoryPoint>> {
    if (enable_pre_smoothing_) {
      return eb_path_optimizer_ptr_->getEBTrajectory(
        p.ego_pose, p.path, prev_optimal_trajs_ptr_, p.ego_vel, debug_data_);
    }
    return points_utils::convertToTrajectoryPoints(p.path.points);
  }();
  if (!eb_traj) {
    return getPrevTrajs(p.path.points);
  }

  // NOTE: Elastic band sometimes diverges with status = "OSQP_SOLVED".
  constexpr double max_path_change_diff = 1.0e4;
  for (size_t i = 0; i < eb_traj->size(); ++i) {
    const auto & eb_pos = eb_traj->at(i).pose.position;
    const auto & path_pos = p.path.points.at(std::min(i, p.path.points.size() - 1)).pose.position;

    const double diff_x = eb_pos.x - path_pos.x;
    const double diff_y = eb_pos.y - path_pos.y;
    if (max_path_change_diff < std::abs(diff_x) || max_path_change_diff < std::abs(diff_y)) {
      return getPrevTrajs(p.path.points);
    }
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
    enable_avoidance_, eb_traj.get(), p.path.points, prev_optimal_trajs_ptr_, cv_maps, p.ego_pose,
    p.ego_vel, debug_data_);
  if (!mpt_trajs) {
    return getPrevTrajs(p.path.points);
  }

  // make trajectories, which has all optimized trajectories information
  Trajectories trajs;
  trajs.smoothed_trajectory = eb_traj.get();
  trajs.mpt_ref_points = mpt_trajs.get().ref_points;
  trajs.model_predictive_trajectory = mpt_trajs.get().mpt;

  // debug data
  debug_data_.mpt_traj = mpt_trajs.get().mpt;
  debug_data_.mpt_ref_traj = points_utils::convertToTrajectoryPoints(mpt_trajs.get().ref_points);
  debug_data_.eb_traj = eb_traj.get();

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return trajs;
}

Trajectories ObstacleAvoidancePlanner::getPrevTrajs(
  const std::vector<PathPoint> & path_points) const
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

void ObstacleAvoidancePlanner::calcVelocity(
  const std::vector<PathPoint> & path_points, std::vector<TrajectoryPoint> & traj_points) const
{
  for (size_t i = 0; i < traj_points.size(); i++) {
    const size_t nearest_seg_idx = findNearestSegmentIndexWithSoftYawConstraints(
      path_points, traj_points.at(i).pose, traj_param_.delta_dist_threshold_for_closest_point,
      traj_param_.delta_yaw_threshold_for_closest_point);

    // add this line not to exceed max index size
    const size_t max_idx = std::min(nearest_seg_idx + 1, path_points.size() - 1);
    // NOTE: std::max, not std::min, is used here since traj_points' sampling width may be longer
    // than path_points' sampling width. A zero velocity point is guaranteed to be inserted in an
    // output trajectory in the alignVelocity function
    traj_points.at(i).longitudinal_velocity_mps = std::max(
      path_points.at(nearest_seg_idx).longitudinal_velocity_mps,
      path_points.at(max_idx).longitudinal_velocity_mps);
  }
}

void ObstacleAvoidancePlanner::insertZeroVelocityOutsideDrivableArea(
  const PlannerData & planner_data, std::vector<TrajectoryPoint> & traj_points,
  const CVMaps & cv_maps)
{
  if (traj_points.empty()) {
    return;
  }

  stop_watch_.tic(__func__);

  const auto & map_info = cv_maps.map_info;
  const auto & road_clearance_map = cv_maps.clearance_map;

  const size_t nearest_idx = findEgoNearestIndex(traj_points, planner_data.ego_pose);

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
    const bool is_outside = cv_drivable_area_utils::isOutsideDrivableAreaFromRectangleFootprint(
      traj_point, road_clearance_map, map_info, vehicle_param_);

    // only insert zero velocity to the first point outside drivable area
    if (is_outside) {
      traj_points[i].longitudinal_velocity_mps = 0.0;
      debug_data_.stop_pose_by_drivable_area = traj_points[i].pose;

      // NOTE: traj_points does not have valid z for efficient calculation of trajectory
      if (!planner_data.path.points.empty()) {
        const size_t path_idx =
          motion_utils::findNearestIndex(planner_data.path.points, traj_points[i].pose.position);
        debug_data_.stop_pose_by_drivable_area->position.z =
          planner_data.path.points.at(path_idx).pose.position.z;
      }
      break;
    }
  }

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
}

void ObstacleAvoidancePlanner::publishDebugDataInOptimization(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points)
{
  stop_watch_.tic(__func__);

  const auto & p = planner_data;

  {  // publish trajectories
    const auto debug_eb_traj = createTrajectory(debug_data_.eb_traj, p.path.header);
    debug_eb_traj_pub_->publish(debug_eb_traj);

    const auto debug_mpt_fixed_traj = createTrajectory(debug_data_.mpt_fixed_traj, p.path.header);
    debug_mpt_fixed_traj_pub_->publish(debug_mpt_fixed_traj);

    const auto debug_mpt_ref_traj = createTrajectory(debug_data_.mpt_ref_traj, p.path.header);
    debug_mpt_ref_traj_pub_->publish(debug_mpt_ref_traj);

    const auto debug_mpt_traj = createTrajectory(debug_data_.mpt_traj, p.path.header);
    debug_mpt_traj_pub_->publish(debug_mpt_traj);
  }

  {  // publish markers
    if (is_publishing_debug_visualization_marker_) {
      stop_watch_.tic("getDebugVisualizationMarker");
      const auto & debug_marker =
        debug_utils::getDebugVisualizationMarker(debug_data_, traj_points, vehicle_param_, false);
      debug_data_.msg_stream << "      getDebugVisualizationMarker:= "
                             << stop_watch_.toc("getDebugVisualizationMarker") << " [ms]\n";

      stop_watch_.tic("publishDebugVisualizationMarker");
      debug_markers_pub_->publish(debug_marker);
      debug_data_.msg_stream << "      publishDebugVisualizationMarker:= "
                             << stop_watch_.toc("publishDebugVisualizationMarker") << " [ms]\n";

      stop_watch_.tic("getDebugVisualizationWallMarker");
      const auto & debug_wall_marker =
        debug_utils::getDebugVisualizationWallMarker(debug_data_, vehicle_param_);
      debug_data_.msg_stream << "      getDebugVisualizationWallMarker:= "
                             << stop_watch_.toc("getDebugVisualizationWallMarker") << " [ms]\n";

      stop_watch_.tic("publishDebugVisualizationWallMarker");
      debug_wall_markers_pub_->publish(debug_wall_marker);
      debug_data_.msg_stream << "      publishDebugVisualizationWallMarker:= "
                             << stop_watch_.toc("publishDebugVisualizationWallMarker") << " [ms]\n";
    }
  }

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
}

Trajectories ObstacleAvoidancePlanner::makePrevTrajectories(
  const std::vector<PathPoint> & path_points, const Trajectories & trajs,
  const PlannerData & planner_data)
{
  stop_watch_.tic(__func__);

  const auto post_processed_smoothed_traj =
    generatePostProcessedTrajectory(path_points, trajs.smoothed_trajectory, planner_data);

  // TODO(murooka) generatePoseProcessedTrajectory may be too large
  Trajectories trajectories;
  trajectories.smoothed_trajectory = post_processed_smoothed_traj;
  trajectories.mpt_ref_points = trajs.mpt_ref_points;
  trajectories.model_predictive_trajectory = trajs.model_predictive_trajectory;

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";

  return trajectories;
}

std::vector<TrajectoryPoint> ObstacleAvoidancePlanner::generatePostProcessedTrajectory(
  const std::vector<PathPoint> & path_points,
  const std::vector<TrajectoryPoint> & optimized_traj_points, const PlannerData & planner_data)
{
  stop_watch_.tic(__func__);

  std::vector<TrajectoryPoint> trajectory_points;
  if (path_points.empty()) {
    TrajectoryPoint tmp_point;
    tmp_point.pose = planner_data.ego_pose;
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

  debug_data_.msg_stream << "  " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return fine_traj_points_with_vel;
}

std::vector<TrajectoryPoint> ObstacleAvoidancePlanner::getExtendedTrajectory(
  const std::vector<PathPoint> & path_points, const std::vector<TrajectoryPoint> & optimized_points)
{
  stop_watch_.tic(__func__);

  assert(!path_points.empty());

  const double accum_arc_length = motion_utils::calcArcLength(optimized_points);
  if (accum_arc_length > traj_param_.trajectory_length) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(),
      "[Avoidance] Not extend trajectory");
    return std::vector<TrajectoryPoint>{};
  }

  // calculate end idx of optimized points on path points
  const auto opt_end_path_idx = motion_utils::findNearestIndex(
    path_points, optimized_points.back().pose, std::numeric_limits<double>::max(),
    traj_param_.delta_yaw_threshold_for_closest_point);
  if (!opt_end_path_idx) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(),
      "[Avoidance] Not extend trajectory since could not find nearest idx from last opt point");
    return std::vector<TrajectoryPoint>{};
  }

  auto extended_traj_points = [&]() -> std::vector<TrajectoryPoint> {
    const size_t non_fixed_begin_path_idx = opt_end_path_idx.get();
    const size_t non_fixed_end_path_idx = points_utils::findForwardIndex(
      path_points, non_fixed_begin_path_idx, traj_param_.non_fixed_trajectory_length);

    if (
      non_fixed_begin_path_idx == path_points.size() - 1 ||
      non_fixed_begin_path_idx == non_fixed_end_path_idx) {
      if (points_utils::isNearLastPathPoint(
            optimized_points.back(), path_points, traj_param_.max_dist_for_extending_end_point,
            traj_param_.delta_yaw_threshold_for_closest_point)) {
        return std::vector<TrajectoryPoint>{};
      }
      const auto last_traj_point = points_utils::convertToTrajectoryPoint(path_points.back());
      return std::vector<TrajectoryPoint>{last_traj_point};
    } else if (non_fixed_end_path_idx == path_points.size() - 1) {
      // no need to connect smoothly since extended trajectory length is short enough
      const auto last_traj_point = points_utils::convertToTrajectoryPoint(path_points.back());
      return std::vector<TrajectoryPoint>{last_traj_point};
    }

    // define non_fixed/fixed_traj_points
    const auto begin_point = optimized_points.back();
    const auto end_point =
      points_utils::convertToTrajectoryPoint(path_points.at(non_fixed_end_path_idx));
    const std::vector<TrajectoryPoint> non_fixed_traj_points{begin_point, end_point};
    const std::vector<PathPoint> fixed_path_points{
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

    debug_data_.extended_non_fixed_traj = interpolated_non_fixed_traj_points;
    debug_data_.extended_fixed_traj = fixed_traj_points;

    return extended_points;
  }();

  // NOTE: Extended points will be concatenated with optimized points.
  //       Then, minimum velocity of each point is chosen from concatenated points or path points
  //       since optimized points has zero velocity outside drivable area
  constexpr double large_velocity = 1e4;
  for (auto & point : extended_traj_points) {
    point.longitudinal_velocity_mps = large_velocity;
  }

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return extended_traj_points;
}

std::vector<TrajectoryPoint> ObstacleAvoidancePlanner::generateFineTrajectoryPoints(
  const std::vector<PathPoint> & path_points,
  const std::vector<TrajectoryPoint> & traj_points) const
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

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";

  return interpolated_traj_points;
}

std::vector<TrajectoryPoint> ObstacleAvoidancePlanner::alignVelocity(
  const std::vector<TrajectoryPoint> & fine_traj_points, const std::vector<PathPoint> & path_points,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  stop_watch_.tic(__func__);

  // insert zero velocity path index, and get optional zero_vel_path_idx
  const auto path_zero_vel_info =
    [&]() -> std::pair<std::vector<TrajectoryPoint>, boost::optional<size_t>> {
    const auto opt_path_zero_vel_idx = motion_utils::searchZeroVelocityIndex(path_points);
    if (opt_path_zero_vel_idx) {
      const auto & zero_vel_path_point = path_points.at(opt_path_zero_vel_idx.get());
      const auto opt_traj_seg_idx = motion_utils::findNearestSegmentIndex(
        fine_traj_points, zero_vel_path_point.pose, std::numeric_limits<double>::max(),
        traj_param_.delta_yaw_threshold_for_closest_point);
      if (opt_traj_seg_idx) {
        const auto interpolated_pose =
          lerpPose(fine_traj_points, zero_vel_path_point.pose.position, opt_traj_seg_idx.get());
        if (interpolated_pose) {
          TrajectoryPoint zero_vel_traj_point;
          zero_vel_traj_point.pose = interpolated_pose.get();
          zero_vel_traj_point.longitudinal_velocity_mps =
            zero_vel_path_point.longitudinal_velocity_mps;

          if (
            tier4_autoware_utils::calcDistance2d(
              fine_traj_points.at(opt_traj_seg_idx.get()).pose, zero_vel_traj_point.pose) < 1e-3) {
            return {fine_traj_points, opt_traj_seg_idx.get()};
          } else if (
            tier4_autoware_utils::calcDistance2d(
              fine_traj_points.at(opt_traj_seg_idx.get() + 1).pose, zero_vel_traj_point.pose) <
            1e-3) {
            return {fine_traj_points, opt_traj_seg_idx.get() + 1};
          }

          auto fine_traj_points_with_zero_vel = fine_traj_points;
          fine_traj_points_with_zero_vel.insert(
            fine_traj_points_with_zero_vel.begin() + opt_traj_seg_idx.get() + 1,
            zero_vel_traj_point);
          return {fine_traj_points_with_zero_vel, opt_traj_seg_idx.get() + 1};
        }
      }
    }

    return {fine_traj_points, {}};
  }();
  const auto fine_traj_points_with_path_zero_vel = path_zero_vel_info.first;
  const auto opt_zero_vel_path_idx = path_zero_vel_info.second;

  // search zero velocity index of fine_traj_points
  const size_t zero_vel_fine_traj_idx = [&]() {
    // zero velocity for being outside drivable area
    const size_t zero_vel_traj_idx = searchExtendedZeroVelocityIndex(
      fine_traj_points_with_path_zero_vel, traj_points,
      traj_param_.delta_yaw_threshold_for_closest_point);

    // zero velocity in path points
    if (opt_zero_vel_path_idx) {
      return std::min(opt_zero_vel_path_idx.get(), zero_vel_traj_idx);
    }
    return zero_vel_traj_idx;
  }();

  // interpolate z and velocity
  auto fine_traj_points_with_vel = fine_traj_points_with_path_zero_vel;
  size_t prev_begin_idx = 0;
  for (size_t i = 0; i < fine_traj_points_with_vel.size(); ++i) {
    auto truncated_points = points_utils::clipForwardPoints(path_points, prev_begin_idx, 5.0);
    if (truncated_points.size() < 2) {
      // NOTE: At least, two points must be contained in truncated_points
      truncated_points = std::vector<PathPoint>(
        path_points.begin() + prev_begin_idx,
        path_points.begin() + std::min(path_points.size(), prev_begin_idx + 2));
    }

    const auto & target_pose = fine_traj_points_with_vel[i].pose;
    const size_t closest_seg_idx = findNearestSegmentIndexWithSoftYawConstraints(
      truncated_points, target_pose, traj_param_.delta_dist_threshold_for_closest_point,
      traj_param_.delta_yaw_threshold_for_closest_point);

    // lerp z
    fine_traj_points_with_vel[i].pose.position.z =
      lerpPoseZ(truncated_points, target_pose.position, closest_seg_idx);

    // lerp vx
    const double target_vel = lerpTwistX(truncated_points, target_pose.position, closest_seg_idx);

    if (i >= zero_vel_fine_traj_idx) {
      fine_traj_points_with_vel[i].longitudinal_velocity_mps = 0.0;
    } else {
      fine_traj_points_with_vel[i].longitudinal_velocity_mps = target_vel;
    }

    // NOTE: closest_seg_idx is for the clipped trajectory. This operation must be "+=".
    prev_begin_idx += closest_seg_idx;
  }

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return fine_traj_points_with_vel;
}

void ObstacleAvoidancePlanner::publishDebugDataInMain(const Path & path) const
{
  stop_watch_.tic(__func__);

  {  // publish trajectories
    const auto debug_extended_fixed_traj =
      createTrajectory(debug_data_.extended_fixed_traj, path.header);
    debug_extended_fixed_traj_pub_->publish(debug_extended_fixed_traj);

    const auto debug_extended_non_fixed_traj =
      createTrajectory(debug_data_.extended_non_fixed_traj, path.header);
    debug_extended_non_fixed_traj_pub_->publish(debug_extended_non_fixed_traj);
  }

  {  // publish clearance map
    stop_watch_.tic("publishClearanceMap");

    if (is_publishing_area_with_objects_) {  // false
      debug_area_with_objects_pub_->publish(
        debug_utils::getDebugCostmap(debug_data_.area_with_objects_map, path.drivable_area));
    }
    if (is_publishing_object_clearance_map_) {  // false
      debug_object_clearance_map_pub_->publish(
        debug_utils::getDebugCostmap(debug_data_.only_object_clearance_map, path.drivable_area));
    }
    if (is_publishing_clearance_map_) {  // true
      debug_clearance_map_pub_->publish(
        debug_utils::getDebugCostmap(debug_data_.clearance_map, path.drivable_area));
    }
    debug_data_.msg_stream << "    getDebugCostMap * 3:= " << stop_watch_.toc("publishClearanceMap")
                           << " [ms]\n";
  }

  debug_data_.msg_stream << "  " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ObstacleAvoidancePlanner)
