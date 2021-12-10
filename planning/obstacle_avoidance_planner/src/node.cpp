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

#include "obstacle_avoidance_planner/debug.hpp"
#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"
#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "obstacle_avoidance_planner/process_cv.hpp"
#include "obstacle_avoidance_planner/util.hpp"

#include <autoware_utils/trajectory/tmp_conversion.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

ObstacleAvoidancePlanner::ObstacleAvoidancePlanner(const rclcpp::NodeOptions & node_options)
: Node("obstacle_avoidance_planner", node_options), min_num_points_for_getting_yaw_(2)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>(clock);
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  trajectory_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/path", 1);
  avoiding_traj_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/scenario_planning/lane_driving/obstacle_avoidance_candidate_trajectory",
    durable_qos);
  debug_smoothed_points_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/debug/smoothed_points", durable_qos);
  is_avoidance_possible_pub_ = create_publisher<tier4_planning_msgs::msg::IsAvoidancePossible>(
    "/planning/scenario_planning/lane_driving/obstacle_avoidance_ready", durable_qos);
  debug_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", durable_qos);
  debug_clearance_map_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/clearance_map", durable_qos);
  debug_object_clearance_map_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/object_clearance_map", durable_qos);
  debug_area_with_objects_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/area_with_objects", durable_qos);

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

  is_publishing_area_with_objects_ = declare_parameter("is_publishing_area_with_objects", false);
  is_publishing_clearance_map_ = declare_parameter("is_publishing_clearance_map", false);
  is_showing_debug_info_ = declare_parameter("is_showing_debug_info", false);
  is_using_vehicle_config_ = declare_parameter("is_using_vehicle_config", false);
  is_stopping_if_outside_drivable_area_ =
    declare_parameter("is_stopping_if_outside_drivable_area", true);
  enable_avoidance_ = declare_parameter("enable_avoidance", true);

  qp_param_ = std::make_unique<QPParam>();
  traj_param_ = std::make_unique<TrajectoryParam>();
  constrain_param_ = std::make_unique<ConstrainParam>();
  vehicle_param_ = std::make_unique<VehicleParam>();
  mpt_param_ = std::make_unique<MPTParam>();
  qp_param_->max_iteration = declare_parameter("qp_max_iteration", 10000);
  qp_param_->eps_abs = declare_parameter("qp_eps_abs", 1.0e-8);
  qp_param_->eps_rel = declare_parameter("qp_eps_rel", 1.0e-11);
  qp_param_->eps_abs_for_extending = declare_parameter("qp_eps_abs_for_extending", 1.0e-6);
  qp_param_->eps_rel_for_extending = declare_parameter("qp_eps_rel_for_extending", 1.0e-8);
  qp_param_->eps_abs_for_visualizing = declare_parameter("qp_eps_abs_for_visualizing", 1.0e-6);
  qp_param_->eps_rel_for_visualizing = declare_parameter("qp_eps_rel_for_visualizing", 1.0e-8);

  traj_param_->num_sampling_points = declare_parameter("num_sampling_points", 100);
  traj_param_->num_joint_buffer_points = declare_parameter("num_joint_buffer_points", 2);
  traj_param_->num_joint_buffer_points_for_extending =
    declare_parameter("num_joint_buffer_points_for_extending", 4);
  traj_param_->num_offset_for_begin_idx = declare_parameter("num_offset_for_begin_idx", 2);
  traj_param_->num_fix_points_for_extending = declare_parameter("num_fix_points_for_extending", 2);
  traj_param_->forward_fixing_mpt_distance = declare_parameter("forward_fixing_mpt_distance", 10);
  traj_param_->delta_arc_length_for_optimization =
    declare_parameter("delta_arc_length_for_optimization", 1.0);
  traj_param_->delta_arc_length_for_mpt_points =
    declare_parameter("delta_arc_length_for_mpt_points", 1.0);
  traj_param_->delta_arc_length_for_trajectory =
    declare_parameter("delta_arc_length_for_trajectory", 0.1);
  traj_param_->delta_dist_threshold_for_closest_point =
    declare_parameter("delta_dist_threshold_for_closest_point", 3.0);
  traj_param_->delta_yaw_threshold_for_closest_point =
    declare_parameter("delta_yaw_threshold_for_closest_point", 1.0);
  traj_param_->delta_yaw_threshold_for_straight =
    declare_parameter("delta_yaw_threshold_for_straight", 0.02);
  traj_param_->trajectory_length = declare_parameter("trajectory_length", 200);
  traj_param_->forward_fixing_distance = declare_parameter("forward_fixing_distance", 10.0);
  traj_param_->backward_fixing_distance = declare_parameter("backward_fixing_distance", 5.0);
  traj_param_->max_avoiding_ego_velocity_ms =
    declare_parameter("max_avoiding_ego_velocity_ms", 6.0);
  traj_param_->max_avoiding_objects_velocity_ms =
    declare_parameter("max_avoiding_objects_velocity_ms", 0.1);
  traj_param_->center_line_width = declare_parameter("center_line_width", 1.7);
  traj_param_->acceleration_for_non_deceleration_range =
    declare_parameter("acceleration_for_non_deceleration_range", 1.0);
  traj_param_->max_dist_for_extending_end_point =
    declare_parameter("max_dist_for_extending_end_point", 5.0);
  traj_param_->is_avoiding_unknown = declare_parameter("avoiding_object_type.unknown", true);
  traj_param_->is_avoiding_car = declare_parameter("avoiding_object_type.car", true);
  traj_param_->is_avoiding_truck = declare_parameter("avoiding_object_type.truck", true);
  traj_param_->is_avoiding_bus = declare_parameter("avoiding_object_type.bus", true);
  traj_param_->is_avoiding_bicycle = declare_parameter("avoiding_object_type.bicycle", true);
  traj_param_->is_avoiding_motorbike = declare_parameter("avoiding_object_type.motorbike", true);
  traj_param_->is_avoiding_pedestrian = declare_parameter("avoiding_object_type.pedestrian", true);
  traj_param_->is_avoiding_animal = declare_parameter("avoiding_object_type.animal", true);

  constrain_param_->is_getting_constraints_close2path_points =
    declare_parameter("is_getting_constraints_close2path_points", false);
  constrain_param_->clearance_for_straight_line =
    declare_parameter("clearance_for_straight_line", 0.05);
  constrain_param_->clearance_for_joint = declare_parameter("clearance_for_joint", 0.1);
  constrain_param_->range_for_extend_joint = declare_parameter("range_for_extend_joint", 1.6);
  constrain_param_->clearance_for_only_smoothing =
    declare_parameter("clearance_for_only_smoothing", 0.1);
  constrain_param_->clearance_from_object_for_straight =
    declare_parameter("clearance_from_object_for_straight", 10.0);
  constrain_param_->clearance_from_road = declare_parameter("clearance_from_road", 0.1);
  constrain_param_->clearance_from_object = declare_parameter("clearance_from_object", 0.6);
  constrain_param_->extra_desired_clearance_from_road =
    declare_parameter("extra_desired_clearance_from_road", 0.2);
  constrain_param_->min_object_clearance_for_joint =
    declare_parameter("min_object_clearance_for_joint", 3.2);
  constrain_param_->max_x_constrain_search_range =
    declare_parameter("max_x_constrain_search_range", 0.4);
  constrain_param_->coef_x_constrain_search_resolution =
    declare_parameter("coef_x_constrain_search_resolution", 1.0);
  constrain_param_->coef_y_constrain_search_resolution =
    declare_parameter("coef_y_constrain_search_resolution", 0.5);
  constrain_param_->keep_space_shape_x = declare_parameter("keep_space_shape_x", 3.0);
  constrain_param_->keep_space_shape_y = declare_parameter("keep_space_shape_y", 2.0);
  constrain_param_->max_lon_space_for_driveable_constraint =
    declare_parameter("max_lon_space_for_driveable_constraint", 0.5);
  constrain_param_->clearance_for_fixing = 0.0;

  min_delta_dist_for_replan_ = declare_parameter("min_delta_dist_for_replan", 5.0);
  min_delta_time_sec_for_replan_ = declare_parameter("min_delta_time_sec_for_replan", 1.0);
  distance_for_path_shape_change_detection_ =
    declare_parameter("distance_for_path_shape_change_detection", 2.0);

  // vehicle param
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  vehicle_param_->width = vehicle_info.vehicle_width_m;
  vehicle_param_->length = vehicle_info.vehicle_length_m;
  vehicle_param_->wheelbase = vehicle_info.wheel_base_m;
  vehicle_param_->rear_overhang = vehicle_info.rear_overhang_m;
  vehicle_param_->front_overhang = vehicle_info.front_overhang_m;

  if (is_using_vehicle_config_) {
    double vehicle_width = vehicle_info.vehicle_width_m;
    traj_param_->center_line_width = vehicle_width;
    constrain_param_->keep_space_shape_y = vehicle_width;
  }
  constrain_param_->min_object_clearance_for_deceleration =
    constrain_param_->clearance_from_object + constrain_param_->keep_space_shape_y * 0.5;

  double max_steer_deg = 0;
  max_steer_deg = declare_parameter("max_steer_deg", 30.0);
  vehicle_param_->max_steer_rad = max_steer_deg * M_PI / 180.0;
  vehicle_param_->steer_tau = declare_parameter("steer_tau", 0.1);

  // mpt param
  mpt_param_->is_hard_fixing_terminal_point =
    declare_parameter("is_hard_fixing_terminal_point", true);
  mpt_param_->num_curvature_sampling_points = declare_parameter("num_curvature_sampling_points", 5);
  mpt_param_->base_point_weight = declare_parameter("base_point_weight", 2000.0);
  mpt_param_->top_point_weight = declare_parameter("top_point_weight", 1000.0);
  mpt_param_->mid_point_weight = declare_parameter("mid_point_weight", 1000.0);
  mpt_param_->lat_error_weight = declare_parameter("lat_error_weight", 10.0);
  mpt_param_->yaw_error_weight = declare_parameter("yaw_error_weight", 0.0);
  mpt_param_->steer_input_weight = declare_parameter("steer_input_weight", 0.1);
  mpt_param_->steer_rate_weight = declare_parameter("steer_rate_weight", 100.0);
  mpt_param_->steer_acc_weight = declare_parameter("steer_acc_weight", 0.000001);
  mpt_param_->terminal_lat_error_weight = declare_parameter("terminal_lat_error_weight", 0.0);
  mpt_param_->terminal_yaw_error_weight = declare_parameter("terminal_yaw_error_weight", 100.0);
  mpt_param_->terminal_path_lat_error_weight =
    declare_parameter("terminal_path_lat_error_weight", 1000.0);
  mpt_param_->terminal_path_yaw_error_weight =
    declare_parameter("terminal_path_yaw_error_weight", 1000.0);
  mpt_param_->zero_ff_steer_angle = declare_parameter("zero_ff_steer_angle", 0.5);

  mpt_param_->clearance_from_road = vehicle_param_->width * 0.5 +
                                    constrain_param_->clearance_from_road +
                                    constrain_param_->extra_desired_clearance_from_road;
  mpt_param_->clearance_from_object =
    vehicle_param_->width * 0.5 + constrain_param_->clearance_from_object;
  mpt_param_->base_point_dist_from_base_link = 0;
  mpt_param_->top_point_dist_from_base_link =
    (vehicle_param_->length - vehicle_param_->rear_overhang);
  mpt_param_->mid_point_dist_from_base_link =
    (mpt_param_->base_point_dist_from_base_link + mpt_param_->top_point_dist_from_base_link) * 0.5;

  in_objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>();

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleAvoidancePlanner::paramCallback, this, std::placeholders::_1));

  initialize();
}

ObstacleAvoidancePlanner::~ObstacleAvoidancePlanner() {}

rcl_interfaces::msg::SetParametersResult ObstacleAvoidancePlanner::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto update_param = [&](const std::string & name, double & v) {
    auto it = std::find_if(
      parameters.cbegin(), parameters.cend(),
      [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
    if (it != parameters.cend()) {
      v = it->as_double();
      return true;
    }
    return false;
  };

  // trajectory total/fixing length
  update_param("trajectory_length", traj_param_->trajectory_length);
  update_param("forward_fixing_distance", traj_param_->forward_fixing_distance);
  update_param("backward_fixing_distance", traj_param_->backward_fixing_distance);

  // clearance for unique points
  update_param("clearance_for_straight_line", constrain_param_->clearance_for_straight_line);
  update_param("clearance_for_joint", constrain_param_->clearance_for_joint);
  update_param("clearance_for_only_smoothing", constrain_param_->clearance_for_only_smoothing);
  update_param(
    "clearance_from_object_for_straight", constrain_param_->clearance_from_object_for_straight);

  // clearance(distance) when generating trajectory
  update_param("clearance_from_road", constrain_param_->clearance_from_road);
  update_param("clearance_from_object", constrain_param_->clearance_from_object);
  update_param("min_object_clearance_for_joint", constrain_param_->min_object_clearance_for_joint);
  update_param(
    "extra_desired_clearance_from_road", constrain_param_->extra_desired_clearance_from_road);

  // avoiding param
  update_param("max_avoiding_objects_velocity_ms", traj_param_->max_avoiding_objects_velocity_ms);
  update_param("max_avoiding_ego_velocity_ms", traj_param_->max_avoiding_ego_velocity_ms);
  update_param("center_line_width", traj_param_->center_line_width);
  update_param(
    "acceleration_for_non_deceleration_range",
    traj_param_->acceleration_for_non_deceleration_range);

  // mpt param
  update_param("base_point_weight", mpt_param_->base_point_weight);
  update_param("top_point_weight", mpt_param_->top_point_weight);
  update_param("mid_point_weight", mpt_param_->mid_point_weight);
  update_param("lat_error_weight", mpt_param_->lat_error_weight);
  update_param("yaw_error_weight", mpt_param_->yaw_error_weight);
  update_param("steer_input_weight", mpt_param_->steer_input_weight);
  update_param("steer_rate_weight", mpt_param_->steer_rate_weight);
  update_param("steer_acc_weight", mpt_param_->steer_acc_weight);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

// ROS callback functions
void ObstacleAvoidancePlanner::pathCallback(
  const autoware_auto_planning_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_ego_pose_ptr_ = getCurrentEgoPose();
  if (
    msg->points.empty() || msg->drivable_area.data.empty() || !current_ego_pose_ptr_ ||
    !current_twist_ptr_) {
    return;
  }
  autoware_auto_planning_msgs::msg::Trajectory output_trajectory_msg = generateTrajectory(*msg);
  trajectory_pub_->publish(output_trajectory_msg);
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
  in_objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>(*msg);
}

void ObstacleAvoidancePlanner::enableAvoidanceCallback(
  const tier4_planning_msgs::msg::EnableAvoidance::SharedPtr msg)
{
  enable_avoidance_ = msg->enable_avoidance;
}
// End ROS callback functions

autoware_auto_planning_msgs::msg::Trajectory ObstacleAvoidancePlanner::generateTrajectory(
  const autoware_auto_planning_msgs::msg::Path & path)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  const auto traj_points = generateOptimizedTrajectory(*current_ego_pose_ptr_, path);

  const auto post_processed_traj =
    generatePostProcessedTrajectory(*current_ego_pose_ptr_, path.points, traj_points);

  auto output = autoware_utils::convertToTrajectory(post_processed_traj);
  output.header = path.header;

  prev_path_points_ptr_ =
    std::make_unique<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(path.points);

  auto t_end = std::chrono::high_resolution_clock::now();
  float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  RCLCPP_INFO_EXPRESSION(
    get_logger(), is_showing_debug_info_,
    "Total time: = %f [ms]\n==========================", elapsed_ms);
  return output;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
ObstacleAvoidancePlanner::generateOptimizedTrajectory(
  const geometry_msgs::msg::Pose & ego_pose, const autoware_auto_planning_msgs::msg::Path & path)
{
  if (!needReplan(
        ego_pose, prev_ego_pose_ptr_, path.points, prev_replanned_time_ptr_, prev_path_points_ptr_,
        prev_trajectories_ptr_)) {
    return getPrevTrajectory(path.points);
  }
  prev_ego_pose_ptr_ = std::make_unique<geometry_msgs::msg::Pose>(ego_pose);
  prev_replanned_time_ptr_ = std::make_unique<rclcpp::Time>(this->now());

  DebugData debug_data;
  const auto optional_trajs = eb_path_optimizer_ptr_->generateOptimizedTrajectory(
    enable_avoidance_, ego_pose, path, prev_trajectories_ptr_, in_objects_ptr_->objects,
    &debug_data);
  if (!optional_trajs) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "[Avoidance] Optimization failed, passing previous trajectory");
    const bool is_prev_traj = true;
    const auto prev_trajs_inside_area = calcTrajectoryInsideArea(
      getPrevTrajs(path.points), path.points, debug_data.clearance_map, path.drivable_area.info,
      &debug_data, is_prev_traj);
    prev_trajectories_ptr_ = std::make_unique<Trajectories>(
      makePrevTrajectories(*current_ego_pose_ptr_, path.points, prev_trajs_inside_area.get()));

    const auto prev_traj = util::concatTraj(prev_trajs_inside_area.get());
    publishingDebugData(debug_data, path, prev_traj, *vehicle_param_);
    return prev_traj;
  }

  const auto trajs_inside_area = getTrajectoryInsideArea(
    optional_trajs.get(), path.points, debug_data.clearance_map, path.drivable_area.info,
    &debug_data);

  prev_trajectories_ptr_ = std::make_unique<Trajectories>(
    makePrevTrajectories(*current_ego_pose_ptr_, path.points, trajs_inside_area));
  const auto optimized_trajectory = util::concatTraj(trajs_inside_area);
  publishingDebugData(debug_data, path, optimized_trajectory, *vehicle_param_);
  return optimized_trajectory;
}

void ObstacleAvoidancePlanner::initialize()
{
  RCLCPP_WARN(get_logger(), "[ObstacleAvoidancePlanner] Resetting");
  eb_path_optimizer_ptr_ = std::make_unique<EBPathOptimizer>(
    is_showing_debug_info_, *qp_param_, *traj_param_, *constrain_param_, *vehicle_param_,
    *mpt_param_);
  prev_path_points_ptr_ = nullptr;
  prev_trajectories_ptr_ = nullptr;
}

std::unique_ptr<geometry_msgs::msg::Pose> ObstacleAvoidancePlanner::getCurrentEgoPose()
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer_ptr_->lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "[ObstacleAvoidancePlanner] %s", ex.what());
    return nullptr;
  }

  geometry_msgs::msg::Pose p;
  p.orientation = tf_current_pose.transform.rotation;
  p.position.x = tf_current_pose.transform.translation.x;
  p.position.y = tf_current_pose.transform.translation.y;
  p.position.z = tf_current_pose.transform.translation.z;
  std::unique_ptr<geometry_msgs::msg::Pose> p_ptr = std::make_unique<geometry_msgs::msg::Pose>(p);
  return p_ptr;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
ObstacleAvoidancePlanner::generatePostProcessedTrajectory(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points) const
{
  auto t_start = std::chrono::high_resolution_clock::now();

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> trajectory_points;
  if (path_points.empty()) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_point;
    tmp_point.pose = ego_pose;
    tmp_point.longitudinal_velocity_mps = 0;
    trajectory_points.push_back(tmp_point);
    return trajectory_points;
  }
  if (optimized_points.empty()) {
    trajectory_points = util::convertPathToTrajectory(path_points);
    return trajectory_points;
  }
  trajectory_points = convertPointsToTrajectory(path_points, optimized_points);

  auto t_end = std::chrono::high_resolution_clock::now();
  float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  RCLCPP_INFO_EXPRESSION(
    get_logger(), is_showing_debug_info_, "Post processing time: = %f [ms]", elapsed_ms);

  return trajectory_points;
}

bool ObstacleAvoidancePlanner::needReplan(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::unique_ptr<geometry_msgs::msg::Pose> & prev_ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<rclcpp::Time> & prev_replanned_time,
  const std::unique_ptr<std::vector<autoware_auto_planning_msgs::msg::PathPoint>> &
    prev_path_points,
  std::unique_ptr<Trajectories> & prev_trajs)
{
  if (!prev_ego_pose || !prev_replanned_time || !prev_path_points || !prev_trajs) {
    return true;
  }

  if (isPathShapeChanged(ego_pose, path_points, prev_path_points)) {
    RCLCPP_INFO(get_logger(), "[Avoidance] Path shape is changed, reset prev trajs");
    prev_trajs = nullptr;
    return true;
  }

  if (!util::hasValidNearestPointFromEgo(
        *current_ego_pose_ptr_, *prev_trajectories_ptr_, *traj_param_)) {
    RCLCPP_INFO(
      get_logger(), "[Avoidance] Could not find valid nearest point from ego, reset prev trajs");
    prev_trajs = nullptr;
    return true;
  }

  const double delta_dist = util::calculate2DDistance(ego_pose.position, prev_ego_pose->position);
  if (delta_dist > min_delta_dist_for_replan_) {
    return true;
  }

  rclcpp::Duration delta_time = this->now() - *prev_replanned_time;
  const double delta_time_sec = delta_time.seconds();
  if (delta_time_sec > min_delta_time_sec_for_replan_) {
    return true;
  }
  return false;
}

bool ObstacleAvoidancePlanner::isPathShapeChanged(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<std::vector<autoware_auto_planning_msgs::msg::PathPoint>> &
    prev_path_points)
{
  if (!prev_path_points) {
    return true;
  }
  const int default_nearest_prev_path_idx = 0;
  const int nearest_prev_path_idx = util::getNearestIdx(
    *prev_path_points, ego_pose, default_nearest_prev_path_idx,
    traj_param_->delta_yaw_threshold_for_closest_point);
  const int default_nearest_path_idx = 0;
  const int nearest_path_idx = util::getNearestIdx(
    path_points, ego_pose, default_nearest_path_idx,
    traj_param_->delta_yaw_threshold_for_closest_point);

  const auto prev_first = prev_path_points->begin() + nearest_prev_path_idx;
  const auto prev_last = prev_path_points->end();
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> truncated_prev_points(
    prev_first, prev_last);

  const auto first = path_points.begin() + nearest_path_idx;
  const auto last = path_points.end();
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> truncated_points(first, last);

  for (const auto & prev_point : truncated_prev_points) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto & point : truncated_points) {
      const double dist = util::calculate2DDistance(point.pose.position, prev_point.pose.position);
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
    if (min_dist > distance_for_path_shape_change_detection_) {
      return true;
    }
  }
  return false;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
ObstacleAvoidancePlanner::convertPointsToTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & trajectory_points) const
{
  std::vector<geometry_msgs::msg::Point> interpolated_points =
    util::getInterpolatedPoints(trajectory_points, traj_param_->delta_arc_length_for_trajectory);
  // add discarded point in the process of interpolation
  interpolated_points.push_back(trajectory_points.back().pose.position);
  if (static_cast<int>(interpolated_points.size()) < min_num_points_for_getting_yaw_) {
    return util::convertPathToTrajectory(path_points);
  }
  std::vector<geometry_msgs::msg::Point> candidate_points = interpolated_points;
  geometry_msgs::msg::Pose last_pose;
  last_pose.position = candidate_points.back();
  last_pose.orientation = util::getQuaternionFromPoints(
    candidate_points.back(), candidate_points[candidate_points.size() - 2]);
  const auto extended_point_opt = util::getLastExtendedPoint(
    path_points.back(), last_pose, traj_param_->delta_yaw_threshold_for_closest_point,
    traj_param_->max_dist_for_extending_end_point);
  if (extended_point_opt) {
    candidate_points.push_back(extended_point_opt.get());
  }

  const int zero_velocity_idx = util::getZeroVelocityIdx(
    is_showing_debug_info_, candidate_points, path_points, prev_trajectories_ptr_, *traj_param_);

  auto traj_points = util::convertPointsToTrajectoryPointsWithYaw(candidate_points);
  traj_points = util::fillTrajectoryWithVelocity(traj_points, 1e4);
  if (prev_trajectories_ptr_) {
    const int max_skip_comparison_velocity_idx_for_optimized_points =
      calculateNonDecelerationRange(traj_points, *current_ego_pose_ptr_, current_twist_ptr_->twist);
    const auto optimized_trajectory = util::concatTraj(*prev_trajectories_ptr_);
    traj_points = util::alignVelocityWithPoints(
      traj_points, optimized_trajectory, zero_velocity_idx,
      max_skip_comparison_velocity_idx_for_optimized_points);
  }
  const int max_skip_comparison_idx_for_path_points = -1;
  traj_points = util::alignVelocityWithPoints(
    traj_points, path_points, zero_velocity_idx, max_skip_comparison_idx_for_path_points);

  return traj_points;
}

void ObstacleAvoidancePlanner::publishingDebugData(
  const DebugData & debug_data, const autoware_auto_planning_msgs::msg::Path & path,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const VehicleParam & vehicle_param)
{
  auto traj = autoware_utils::convertToTrajectory(debug_data.foa_data.avoiding_traj_points);
  traj.header = path.header;
  avoiding_traj_pub_->publish(traj);

  auto debug_smoothed_points = autoware_utils::convertToTrajectory(debug_data.smoothed_points);
  debug_smoothed_points.header = path.header;
  debug_smoothed_points_pub_->publish(debug_smoothed_points);

  tier4_planning_msgs::msg::IsAvoidancePossible is_avoidance_possible;
  is_avoidance_possible.is_avoidance_possible = debug_data.foa_data.is_avoidance_possible;
  is_avoidance_possible_pub_->publish(is_avoidance_possible);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj_points_debug = traj_points;
  // Add z information for virtual wall
  if (!traj_points_debug.empty()) {
    const int idx = util::getNearestIdx(
      path.points, traj_points.back().pose, 0, traj_param_->delta_yaw_threshold_for_closest_point);
    traj_points_debug.back().pose.position.z = path.points.at(idx).pose.position.z + 1.0;
  }

  debug_markers_pub_->publish(
    getDebugVisualizationMarker(debug_data, traj_points_debug, vehicle_param));
  if (is_publishing_area_with_objects_) {
    debug_area_with_objects_pub_->publish(
      getDebugCostmap(debug_data.area_with_objects_map, path.drivable_area));
  }
  if (is_publishing_clearance_map_) {
    debug_clearance_map_pub_->publish(
      getDebugCostmap(debug_data.clearance_map, path.drivable_area));
    debug_object_clearance_map_pub_->publish(
      getDebugCostmap(debug_data.only_object_clearance_map, path.drivable_area));
  }
}

int ObstacleAvoidancePlanner::calculateNonDecelerationRange(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Twist & ego_twist) const
{
  const int default_idx = 0;
  const int nearest_idx = util::getNearestIdx(
    traj_points, ego_pose, default_idx, traj_param_->delta_yaw_threshold_for_closest_point);
  const double non_decelerating_arc_length =
    (ego_twist.linear.x - traj_param_->max_avoiding_ego_velocity_ms) /
    traj_param_->acceleration_for_non_deceleration_range;
  if (non_decelerating_arc_length < 0 || traj_points.size() < 2) {
    return 0;
  }

  double accum_arc_length = 0;
  for (std::size_t i = nearest_idx + 1; i < traj_points.size(); i++) {
    accum_arc_length +=
      util::calculate2DDistance(traj_points[i].pose.position, traj_points[i - 1].pose.position);
    if (accum_arc_length > non_decelerating_arc_length) {
      return i;
    }
  }
  return 0;
}

Trajectories ObstacleAvoidancePlanner::getTrajectoryInsideArea(
  const Trajectories & trajs,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const cv::Mat & road_clearance_map, const nav_msgs::msg::MapMetaData & map_info,
  DebugData * debug_data) const
{
  debug_data->current_vehicle_footprints =
    util::getVehicleFootprints(trajs.model_predictive_trajectory, *vehicle_param_);
  const auto current_trajs_inside_area =
    calcTrajectoryInsideArea(trajs, path_points, road_clearance_map, map_info, debug_data);
  if (!current_trajs_inside_area) {
    RCLCPP_WARN(
      get_logger(),
      "[Avoidance] Current trajectory is not inside drivable area, passing previous one. "
      "Might stop at the end of drivable trajectory.");
    auto prev_trajs = getPrevTrajs(path_points);
    const bool is_prev_traj = true;
    const auto prev_trajs_inside_area = calcTrajectoryInsideArea(
      prev_trajs, path_points, road_clearance_map, map_info, debug_data, is_prev_traj);
    return prev_trajs_inside_area.get();
  }
  return current_trajs_inside_area.get();
}

boost::optional<Trajectories> ObstacleAvoidancePlanner::calcTrajectoryInsideArea(
  const Trajectories & trajs,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const cv::Mat & road_clearance_map, const nav_msgs::msg::MapMetaData & map_info,
  DebugData * debug_data, const bool is_prev_traj) const
{
  if (!is_stopping_if_outside_drivable_area_) {
    const auto stop_idx = getStopIdx(path_points, trajs, map_info, road_clearance_map, debug_data);
    if (stop_idx) {
      auto clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(
        get_logger(), clock, std::chrono::milliseconds(1000).count(),
        "[Avoidance] Expecting over drivable area");
    }
    return getBaseTrajectory(path_points, trajs);
  }
  const auto optional_stop_idx =
    getStopIdx(path_points, trajs, map_info, road_clearance_map, debug_data);
  if (!is_prev_traj && optional_stop_idx) {
    return boost::none;
  }

  auto tmp_trajs = getBaseTrajectory(path_points, trajs);
  if (is_prev_traj) {
    tmp_trajs.extended_trajectory =
      std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
    debug_data->is_expected_to_over_drivable_area = true;
  }

  if (optional_stop_idx && !prev_trajectories_ptr_) {
    if (optional_stop_idx.get() < static_cast<int>(trajs.model_predictive_trajectory.size())) {
      tmp_trajs.model_predictive_trajectory =
        std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{
          trajs.model_predictive_trajectory.begin(),
          trajs.model_predictive_trajectory.begin() + optional_stop_idx.get()};
      tmp_trajs.extended_trajectory =
        std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
      debug_data->is_expected_to_over_drivable_area = true;
    }
  }
  return tmp_trajs;
}

Trajectories ObstacleAvoidancePlanner::getPrevTrajs(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const
{
  if (!prev_trajectories_ptr_) {
    const auto traj = util::convertPathToTrajectory(path_points);
    Trajectories trajs;
    trajs.smoothed_trajectory = traj;
    trajs.model_predictive_trajectory = traj;
    return trajs;
  } else {
    return *prev_trajectories_ptr_;
  }
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
ObstacleAvoidancePlanner::getPrevTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj;
  const auto & trajs = getPrevTrajs(path_points);
  traj.insert(
    traj.end(), trajs.model_predictive_trajectory.begin(), trajs.model_predictive_trajectory.end());
  traj.insert(traj.end(), trajs.extended_trajectory.begin(), trajs.extended_trajectory.end());
  return traj;
}

Trajectories ObstacleAvoidancePlanner::makePrevTrajectories(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const Trajectories & trajs) const
{
  const auto post_processed_smoothed_traj =
    generatePostProcessedTrajectory(ego_pose, path_points, trajs.smoothed_trajectory);
  Trajectories trajectories;
  trajectories.smoothed_trajectory = post_processed_smoothed_traj;
  trajectories.mpt_ref_points = trajs.mpt_ref_points;
  trajectories.model_predictive_trajectory = trajs.model_predictive_trajectory;
  trajectories.extended_trajectory = trajs.extended_trajectory;
  return trajectories;
}

Trajectories ObstacleAvoidancePlanner::getBaseTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const Trajectories & trajs) const
{
  auto base_trajs = trajs;
  if (!trajs.extended_trajectory.empty()) {
    const auto extended_point_opt = util::getLastExtendedTrajPoint(
      path_points.back(), trajs.extended_trajectory.back().pose,
      traj_param_->delta_yaw_threshold_for_closest_point,
      traj_param_->max_dist_for_extending_end_point);
    if (extended_point_opt) {
      base_trajs.extended_trajectory.push_back(extended_point_opt.get());
    }
  } else if (!trajs.model_predictive_trajectory.empty()) {
    const auto extended_point_opt = util::getLastExtendedTrajPoint(
      path_points.back(), trajs.model_predictive_trajectory.back().pose,
      traj_param_->delta_yaw_threshold_for_closest_point,
      traj_param_->max_dist_for_extending_end_point);
    if (extended_point_opt) {
      base_trajs.extended_trajectory.push_back(extended_point_opt.get());
    }
  }
  double prev_velocity = 1e4;
  for (auto & p : base_trajs.model_predictive_trajectory) {
    if (p.longitudinal_velocity_mps < 1e-6) {
      p.longitudinal_velocity_mps = prev_velocity;
    } else {
      prev_velocity = p.longitudinal_velocity_mps;
    }
  }
  for (auto & p : base_trajs.extended_trajectory) {
    if (p.longitudinal_velocity_mps < 1e-6) {
      p.longitudinal_velocity_mps = prev_velocity;
    } else {
      prev_velocity = p.longitudinal_velocity_mps;
    }
  }
  return base_trajs;
}

boost::optional<int> ObstacleAvoidancePlanner::getStopIdx(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const Trajectories & trajs, const nav_msgs::msg::MapMetaData & map_info,
  const cv::Mat & road_clearance_map, DebugData * debug_data) const
{
  const int nearest_idx = util::getNearestIdx(
    path_points, *current_ego_pose_ptr_, 0, traj_param_->delta_yaw_threshold_for_closest_point);
  const double accum_ds = util::getArcLength(path_points, nearest_idx);

  auto target_points = trajs.model_predictive_trajectory;
  if (accum_ds < traj_param_->num_sampling_points * traj_param_->delta_arc_length_for_mpt_points) {
    target_points.insert(
      target_points.end(), trajs.extended_trajectory.begin(), trajs.extended_trajectory.end());
    const auto extended_mpt_point_opt = util::getLastExtendedTrajPoint(
      path_points.back(), trajs.model_predictive_trajectory.back().pose,
      traj_param_->delta_yaw_threshold_for_closest_point,
      traj_param_->max_dist_for_extending_end_point);
    if (extended_mpt_point_opt) {
      target_points.push_back(extended_mpt_point_opt.get());
    }
  }

  const auto footprints = util::getVehicleFootprints(target_points, *vehicle_param_);
  const int debug_nearest_footprint_idx =
    util::getNearestIdx(target_points, current_ego_pose_ptr_->position);
  std::vector<util::Footprint> debug_truncated_footprints(
    footprints.begin() + debug_nearest_footprint_idx, footprints.end());
  debug_data->vehicle_footprints = debug_truncated_footprints;

  const auto optional_idx =
    process_cv::getStopIdx(footprints, *current_ego_pose_ptr_, road_clearance_map, map_info);
  return optional_idx;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ObstacleAvoidancePlanner)
