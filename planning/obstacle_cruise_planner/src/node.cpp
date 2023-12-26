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

#include "obstacle_cruise_planner/node.hpp"

#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/conversion.hpp"
#include "object_recognition_utils/predicted_path_utils.hpp"
#include "obstacle_cruise_planner/polygon_utils.hpp"
#include "obstacle_cruise_planner/utils.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"
#include "tier4_autoware_utils/ros/marker_helper.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"

#include <boost/format.hpp>

#include <algorithm>
#include <chrono>

namespace
{
VelocityLimitClearCommand createVelocityLimitClearCommandMessage(
  const rclcpp::Time & current_time, const std::string & module_name)
{
  VelocityLimitClearCommand msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_cruise_planner." + module_name;
  msg.command = true;
  return msg;
}

template <typename T>
std::optional<T> getObstacleFromUuid(
  const std::vector<T> & obstacles, const std::string & target_uuid)
{
  const auto itr = std::find_if(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid == target_uuid;
  });

  if (itr == obstacles.end()) {
    return std::nullopt;
  }
  return *itr;
}

bool isFrontObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_idx,
  const geometry_msgs::msg::Point & obstacle_pos)
{
  const size_t obstacle_idx = motion_utils::findNearestIndex(traj_points, obstacle_pos);

  const double ego_to_obstacle_distance =
    motion_utils::calcSignedArcLength(traj_points, ego_idx, obstacle_idx);

  if (ego_to_obstacle_distance < 0) {
    return false;
  }

  return true;
}

PredictedPath resampleHighestConfidencePredictedPath(
  const std::vector<PredictedPath> & predicted_paths, const double time_interval,
  const double time_horizon)
{
  // get highest confidence path
  const auto reliable_path = std::max_element(
    predicted_paths.begin(), predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  // resample
  return object_recognition_utils::resamplePredictedPath(
    *reliable_path, time_interval, time_horizon);
}

double calcDiffAngleAgainstTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & target_pose)
{
  const size_t nearest_idx = motion_utils::findNearestIndex(traj_points, target_pose.position);
  const double traj_yaw = tf2::getYaw(traj_points.at(nearest_idx).pose.orientation);

  const double target_yaw = tf2::getYaw(target_pose.orientation);

  const double diff_yaw = tier4_autoware_utils::normalizeRadian(target_yaw - traj_yaw);
  return diff_yaw;
}

std::pair<double, double> projectObstacleVelocityToTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle)
{
  const size_t object_idx = motion_utils::findNearestIndex(traj_points, obstacle.pose.position);
  const double traj_yaw = tf2::getYaw(traj_points.at(object_idx).pose.orientation);

  const double obstacle_yaw = tf2::getYaw(obstacle.pose.orientation);

  const Eigen::Rotation2Dd R_ego_to_obstacle(obstacle_yaw - traj_yaw);
  const Eigen::Vector2d obstacle_velocity(obstacle.twist.linear.x, obstacle.twist.linear.y);
  const Eigen::Vector2d projected_velocity = R_ego_to_obstacle * obstacle_velocity;

  return std::make_pair(projected_velocity[0], projected_velocity[1]);
}

double calcObstacleMaxLength(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in obstacle_cruise_planner.");
}

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point, const bool is_driving_forward)
{
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = tier4_autoware_utils::calcOffsetPose(
    goal_point.pose, extend_distance * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
  extend_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extend_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extend_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extend_trajectory_point;
}

std::vector<TrajectoryPoint> extendTrajectoryPoints(
  const std::vector<TrajectoryPoint> & input_points, const double extend_distance,
  const double step_length)
{
  auto output_points = input_points;
  const auto is_driving_forward_opt = motion_utils::isDrivingForwardWithTwist(input_points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  if (extend_distance < std::numeric_limits<double>::epsilon()) {
    return output_points;
  }

  const auto goal_point = input_points.back();

  double extend_sum = 0.0;
  while (extend_sum <= (extend_distance - step_length)) {
    const auto extend_trajectory_point =
      getExtendTrajectoryPoint(extend_sum, goal_point, is_driving_forward);
    output_points.push_back(extend_trajectory_point);
    extend_sum += step_length;
  }
  const auto extend_trajectory_point =
    getExtendTrajectoryPoint(extend_distance, goal_point, is_driving_forward);
  output_points.push_back(extend_trajectory_point);

  return output_points;
}

Trajectory createTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const std_msgs::msg::Header & header)
{
  auto traj = motion_utils::convertToTrajectory(traj_points);
  traj.header = header;

  return traj;
}

std::vector<int> getTargetObjectType(rclcpp::Node & node, const std::string & param_prefix)
{
  std::unordered_map<std::string, int> types_map{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK},     {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN}};

  std::vector<int> types;
  for (const auto & type : types_map) {
    if (node.declare_parameter<bool>(param_prefix + type.first)) {
      types.push_back(type.second);
    }
  }
  return types;
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = motion_utils::resampleTrajectory(traj, interval);
  return motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

geometry_msgs::msg::Point toGeomPoint(const tier4_autoware_utils::Point2d & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  return geom_point;
}

bool isLowerConsideringHysteresis(
  const double current_val, const bool was_low, const double high_val, const double low_val)
{
  if (was_low) {
    if (high_val < current_val) {
      return false;
    }
    return true;
  }
  if (current_val < low_val) {
    return true;
  }
  return false;
}
}  // namespace

namespace motion_planning
{
ObstacleCruisePlannerNode::BehaviorDeterminationParam::BehaviorDeterminationParam(
  rclcpp::Node & node)
{  // behavior determination
  decimate_trajectory_step_length =
    node.declare_parameter<double>("behavior_determination.decimate_trajectory_step_length");
  obstacle_velocity_threshold_from_cruise_to_stop = node.declare_parameter<double>(
    "behavior_determination.obstacle_velocity_threshold_from_cruise_to_stop");
  obstacle_velocity_threshold_from_stop_to_cruise = node.declare_parameter<double>(
    "behavior_determination.obstacle_velocity_threshold_from_stop_to_cruise");
  crossing_obstacle_velocity_threshold = node.declare_parameter<double>(
    "behavior_determination.crossing_obstacle.obstacle_velocity_threshold");
  crossing_obstacle_traj_angle_threshold = node.declare_parameter<double>(
    "behavior_determination.crossing_obstacle.obstacle_traj_angle_threshold");
  collision_time_margin = node.declare_parameter<double>(
    "behavior_determination.stop.crossing_obstacle.collision_time_margin");
  outside_obstacle_min_velocity_threshold = node.declare_parameter<double>(
    "behavior_determination.cruise.outside_obstacle.obstacle_velocity_threshold");
  ego_obstacle_overlap_time_threshold = node.declare_parameter<double>(
    "behavior_determination.cruise.outside_obstacle.ego_obstacle_overlap_time_threshold");
  max_prediction_time_for_collision_check = node.declare_parameter<double>(
    "behavior_determination.cruise.outside_obstacle.max_prediction_time_for_collision_check");
  stop_obstacle_hold_time_threshold =
    node.declare_parameter<double>("behavior_determination.stop_obstacle_hold_time_threshold");
  prediction_resampling_time_interval =
    node.declare_parameter<double>("behavior_determination.prediction_resampling_time_interval");
  prediction_resampling_time_horizon =
    node.declare_parameter<double>("behavior_determination.prediction_resampling_time_horizon");

  max_lat_margin_for_stop =
    node.declare_parameter<double>("behavior_determination.stop.max_lat_margin");
  max_lat_margin_for_cruise =
    node.declare_parameter<double>("behavior_determination.cruise.max_lat_margin");
  max_lat_margin_for_slow_down =
    node.declare_parameter<double>("behavior_determination.slow_down.max_lat_margin");
  lat_hysteresis_margin_for_slow_down =
    node.declare_parameter<double>("behavior_determination.slow_down.lat_hysteresis_margin");
  successive_num_to_entry_slow_down_condition = node.declare_parameter<int>(
    "behavior_determination.slow_down.successive_num_to_entry_slow_down_condition");
  successive_num_to_exit_slow_down_condition = node.declare_parameter<int>(
    "behavior_determination.slow_down.successive_num_to_exit_slow_down_condition");
  enable_to_consider_current_pose = node.declare_parameter<bool>(
    "behavior_determination.consider_current_pose.enable_to_consider_current_pose");
  time_to_convergence = node.declare_parameter<double>(
    "behavior_determination.consider_current_pose.time_to_convergence");
}

void ObstacleCruisePlannerNode::BehaviorDeterminationParam::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // behavior determination
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.decimate_trajectory_step_length",
    decimate_trajectory_step_length);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.crossing_obstacle.obstacle_velocity_threshold",
    crossing_obstacle_velocity_threshold);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.crossing_obstacle.obstacle_traj_angle_threshold",
    crossing_obstacle_traj_angle_threshold);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.stop.crossing_obstacle.collision_time_margin",
    collision_time_margin);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.cruise.outside_obstacle.obstacle_velocity_threshold",
    outside_obstacle_min_velocity_threshold);
  tier4_autoware_utils::updateParam<double>(
    parameters,
    "behavior_determination.cruise.outside_obstacle.ego_obstacle_overlap_time_threshold",
    ego_obstacle_overlap_time_threshold);
  tier4_autoware_utils::updateParam<double>(
    parameters,
    "behavior_determination.cruise.outside_obstacle.max_prediction_time_for_collision_check",
    max_prediction_time_for_collision_check);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.stop_obstacle_hold_time_threshold",
    stop_obstacle_hold_time_threshold);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.prediction_resampling_time_interval",
    prediction_resampling_time_interval);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.prediction_resampling_time_horizon",
    prediction_resampling_time_horizon);

  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.stop.max_lat_margin", max_lat_margin_for_stop);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.cruise.max_lat_margin", max_lat_margin_for_cruise);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.slow_down.max_lat_margin", max_lat_margin_for_slow_down);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.slow_down.lat_hysteresis_margin",
    lat_hysteresis_margin_for_slow_down);
  tier4_autoware_utils::updateParam<int>(
    parameters, "behavior_determination.slow_down.successive_num_to_entry_slow_down_condition",
    successive_num_to_entry_slow_down_condition);
  tier4_autoware_utils::updateParam<int>(
    parameters, "behavior_determination.slow_down.successive_num_to_exit_slow_down_condition",
    successive_num_to_exit_slow_down_condition);
  tier4_autoware_utils::updateParam<bool>(
    parameters, "behavior_determination.consider_current_pose.enable_to_consider_current_pose",
    enable_to_consider_current_pose);
  tier4_autoware_utils::updateParam<double>(
    parameters, "behavior_determination.consider_current_pose.time_to_convergence",
    time_to_convergence);
}

ObstacleCruisePlannerNode::ObstacleCruisePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_cruise_planner", node_options),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo()),
  debug_data_ptr_(std::make_shared<DebugData>())
{
  using std::placeholders::_1;

  // subscriber
  traj_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleCruisePlannerNode::onTrajectory, this, _1));
  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    [this](const PredictedObjects::ConstSharedPtr msg) { objects_ptr_ = msg; });
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS{1},
    [this](const Odometry::ConstSharedPtr msg) { ego_odom_ptr_ = msg; });
  acc_sub_ = create_subscription<AccelWithCovarianceStamped>(
    "~/input/acceleration", rclcpp::QoS{1},
    [this](const AccelWithCovarianceStamped::ConstSharedPtr msg) { ego_accel_ptr_ = msg; });

  // publisher
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  vel_limit_pub_ =
    create_publisher<VelocityLimit>("~/output/velocity_limit", rclcpp::QoS{1}.transient_local());
  clear_vel_limit_pub_ = create_publisher<VelocityLimitClearCommand>(
    "~/output/clear_velocity_limit", rclcpp::QoS{1}.transient_local());

  // debug publisher
  debug_calculation_time_pub_ = create_publisher<Float32Stamped>("~/debug/processing_time_ms", 1);
  debug_cruise_wall_marker_pub_ = create_publisher<MarkerArray>("~/debug/cruise/virtual_wall", 1);
  debug_stop_wall_marker_pub_ = create_publisher<MarkerArray>("~/virtual_wall", 1);
  debug_slow_down_wall_marker_pub_ =
    create_publisher<MarkerArray>("~/debug/slow_down/virtual_wall", 1);
  debug_marker_pub_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  debug_stop_planning_info_pub_ =
    create_publisher<Float32MultiArrayStamped>("~/debug/stop_planning_info", 1);
  debug_cruise_planning_info_pub_ =
    create_publisher<Float32MultiArrayStamped>("~/debug/cruise_planning_info", 1);
  debug_slow_down_planning_info_pub_ =
    create_publisher<Float32MultiArrayStamped>("~/debug/slow_down_planning_info", 1);

  const auto longitudinal_info = LongitudinalInfo(*this);

  ego_nearest_param_ = EgoNearestParam(*this);

  enable_debug_info_ = declare_parameter<bool>("common.enable_debug_info");
  enable_calculation_time_info_ = declare_parameter<bool>("common.enable_calculation_time_info");
  enable_slow_down_planning_ = declare_parameter<bool>("common.enable_slow_down_planning");

  behavior_determination_param_ = BehaviorDeterminationParam(*this);

  {  // planning algorithm
    const std::string planning_algorithm_param =
      declare_parameter<std::string>("common.planning_algorithm");
    planning_algorithm_ = getPlanningAlgorithmType(planning_algorithm_param);

    if (planning_algorithm_ == PlanningAlgorithm::OPTIMIZATION_BASE) {
      planner_ptr_ = std::make_unique<OptimizationBasedPlanner>(
        *this, longitudinal_info, vehicle_info_, ego_nearest_param_, debug_data_ptr_);
    } else if (planning_algorithm_ == PlanningAlgorithm::PID_BASE) {
      planner_ptr_ = std::make_unique<PIDBasedPlanner>(
        *this, longitudinal_info, vehicle_info_, ego_nearest_param_, debug_data_ptr_);
    } else {
      std::logic_error("Designated algorithm is not supported.");
    }

    min_behavior_stop_margin_ = declare_parameter<double>("common.min_behavior_stop_margin");
    additional_safe_distance_margin_on_curve_ =
      declare_parameter<double>("common.stop_on_curve.additional_safe_distance_margin");
    enable_approaching_on_curve_ =
      declare_parameter<bool>("common.stop_on_curve.enable_approaching");
    min_safe_distance_margin_on_curve_ =
      declare_parameter<double>("common.stop_on_curve.min_safe_distance_margin");
    suppress_sudden_obstacle_stop_ =
      declare_parameter<bool>("common.suppress_sudden_obstacle_stop");
    planner_ptr_->setParam(
      enable_debug_info_, enable_calculation_time_info_, min_behavior_stop_margin_,
      enable_approaching_on_curve_, additional_safe_distance_margin_on_curve_,
      min_safe_distance_margin_on_curve_, suppress_sudden_obstacle_stop_);
  }

  {  // stop/cruise/slow down obstacle type
    stop_obstacle_types_ = getTargetObjectType(*this, "common.stop_obstacle_type.");
    inside_cruise_obstacle_types_ =
      getTargetObjectType(*this, "common.cruise_obstacle_type.inside.");
    outside_cruise_obstacle_types_ =
      getTargetObjectType(*this, "common.cruise_obstacle_type.outside.");
    slow_down_obstacle_types_ = getTargetObjectType(*this, "common.slow_down_obstacle_type.");
  }

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleCruisePlannerNode::onParam, this, std::placeholders::_1));

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}

ObstacleCruisePlannerNode::PlanningAlgorithm ObstacleCruisePlannerNode::getPlanningAlgorithmType(
  const std::string & param) const
{
  if (param == "pid_base") {
    return PlanningAlgorithm::PID_BASE;
  } else if (param == "optimization_base") {
    return PlanningAlgorithm::OPTIMIZATION_BASE;
  }
  return PlanningAlgorithm::INVALID;
}

rcl_interfaces::msg::SetParametersResult ObstacleCruisePlannerNode::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  planner_ptr_->onParam(parameters);

  tier4_autoware_utils::updateParam<bool>(
    parameters, "common.enable_debug_info", enable_debug_info_);
  tier4_autoware_utils::updateParam<bool>(
    parameters, "common.enable_calculation_time_info", enable_calculation_time_info_);

  tier4_autoware_utils::updateParam<bool>(
    parameters, "common.stop_on_curve.enable_approaching", enable_approaching_on_curve_);
  tier4_autoware_utils::updateParam<double>(
    parameters, "common.stop_on_curve.additional_safe_distance_margin",
    additional_safe_distance_margin_on_curve_);
  tier4_autoware_utils::updateParam<double>(
    parameters, "common.stop_on_curve.min_safe_distance_margin",
    min_safe_distance_margin_on_curve_);

  planner_ptr_->setParam(
    enable_debug_info_, enable_calculation_time_info_, min_behavior_stop_margin_,
    enable_approaching_on_curve_, additional_safe_distance_margin_on_curve_,
    min_safe_distance_margin_on_curve_, suppress_sudden_obstacle_stop_);

  tier4_autoware_utils::updateParam<bool>(
    parameters, "common.enable_slow_down_planning", enable_slow_down_planning_);

  behavior_determination_param_.onParam(parameters);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void ObstacleCruisePlannerNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  const auto traj_points = motion_utils::convertToTrajectoryPointArray(*msg);

  // check if subscribed variables are ready
  if (traj_points.empty() || !ego_odom_ptr_ || !ego_accel_ptr_ || !objects_ptr_) {
    return;
  }

  stop_watch_.tic(__func__);
  *debug_data_ptr_ = DebugData();

  const auto is_driving_forward = motion_utils::isDrivingForwardWithTwist(traj_points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.value() : is_driving_forward_;

  // 1. Convert predicted objects to obstacles which are
  //    (1) with a proper label
  //    (2) in front of ego
  //    (3) not too far from trajectory
  const auto target_obstacles = convertToObstacles(traj_points);

  // 2. Determine ego's behavior against each obstacle from stop, cruise and slow down.
  const auto & [stop_obstacles, cruise_obstacles, slow_down_obstacles] =
    determineEgoBehaviorAgainstObstacles(traj_points, target_obstacles);

  // 3. Create data for planning
  const auto planner_data = createPlannerData(traj_points);

  // 4. Stop planning
  const auto stop_traj_points = planner_ptr_->generateStopTrajectory(planner_data, stop_obstacles);

  // 5. Cruise planning
  std::optional<VelocityLimit> cruise_vel_limit;
  const auto cruise_traj_points = planner_ptr_->generateCruiseTrajectory(
    planner_data, stop_traj_points, cruise_obstacles, cruise_vel_limit);
  publishVelocityLimit(cruise_vel_limit, "cruise");

  // 6. Slow down planning
  std::optional<VelocityLimit> slow_down_vel_limit;
  const auto slow_down_traj_points = planner_ptr_->generateSlowDownTrajectory(
    planner_data, cruise_traj_points, slow_down_obstacles, slow_down_vel_limit);
  publishVelocityLimit(slow_down_vel_limit, "slow_down");

  // 7. Publish trajectory
  const auto output_traj = createTrajectory(slow_down_traj_points, msg->header);
  trajectory_pub_->publish(output_traj);

  // 8. Publish debug data
  publishDebugMarker();
  publishDebugInfo();

  // 9. Publish and print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  publishCalculationTime(calculation_time);
  RCLCPP_INFO_EXPRESSION(
    get_logger(), enable_calculation_time_info_, "%s := %f [ms]", __func__, calculation_time);
}

std::vector<Polygon2d> ObstacleCruisePlannerNode::createOneStepPolygons(
  const std::vector<TrajectoryPoint> & traj_points,
  const vehicle_info_util::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin) const
{
  const auto & p = behavior_determination_param_;
  const bool is_enable_current_pose_consideration = p.enable_to_consider_current_pose;
  const double step_length = p.decimate_trajectory_step_length;
  const double time_to_convergence = p.time_to_convergence;

  const size_t nearest_idx =
    motion_utils::findNearestSegmentIndex(traj_points, current_ego_pose.position);
  const auto nearest_pose = traj_points.at(nearest_idx).pose;
  const auto current_ego_pose_error =
    tier4_autoware_utils::inverseTransformPose(current_ego_pose, nearest_pose);
  const double current_ego_lat_error = current_ego_pose_error.position.y;
  const double current_ego_yaw_error = tf2::getYaw(current_ego_pose_error.orientation);
  double time_elapsed = 0.0;

  std::vector<Polygon2d> polygons;
  std::vector<geometry_msgs::msg::Pose> last_poses = {traj_points.at(0).pose};
  if (is_enable_current_pose_consideration) {
    last_poses.push_back(current_ego_pose);
  }

  for (size_t i = 0; i < traj_points.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> current_poses = {traj_points.at(i).pose};

    // estimate the future ego pose with assuming that the pose error against the reference path
    // will decrease to zero by the time_to_convergence
    if (is_enable_current_pose_consideration && time_elapsed < time_to_convergence) {
      const double rem_ratio = (time_to_convergence - time_elapsed) / time_to_convergence;
      geometry_msgs::msg::Pose indexed_pose_err;
      indexed_pose_err.set__orientation(
        tier4_autoware_utils::createQuaternionFromYaw(current_ego_yaw_error * rem_ratio));
      indexed_pose_err.set__position(
        tier4_autoware_utils::createPoint(0.0, current_ego_lat_error * rem_ratio, 0.0));

      current_poses.push_back(
        tier4_autoware_utils::transformPose(indexed_pose_err, traj_points.at(i).pose));

      if (traj_points.at(i).longitudinal_velocity_mps != 0.0) {
        time_elapsed += step_length / std::abs(traj_points.at(i).longitudinal_velocity_mps);
      } else {
        time_elapsed = std::numeric_limits<double>::max();
      }
    }
    polygons.push_back(
      polygon_utils::createOneStepPolygon(last_poses, current_poses, vehicle_info, lat_margin));
    last_poses = current_poses;
  }
  return polygons;
}

std::vector<Obstacle> ObstacleCruisePlannerNode::convertToObstacles(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  stop_watch_.tic(__func__);

  const auto obj_stamp = rclcpp::Time(objects_ptr_->header.stamp);

  std::vector<Obstacle> target_obstacles;
  for (const auto & predicted_object : objects_ptr_->objects) {
    const auto & object_id =
      tier4_autoware_utils::toHexString(predicted_object.object_id).substr(0, 4);
    const auto & current_obstacle_pose =
      obstacle_cruise_utils::getCurrentObjectPose(predicted_object, obj_stamp, now(), true);

    // 1. Check if the obstacle's label is target
    const uint8_t label = predicted_object.classification.front().label;
    const bool is_target_obstacle =
      isStopObstacle(label) || isCruiseObstacle(label) || isSlowDownObstacle(label);
    if (!is_target_obstacle) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_, "Ignore obstacle (%s) since its label is not target.",
        object_id.c_str());
      continue;
    }

    // 2. Check if the obstacle is in front of the ego.
    const size_t ego_idx = ego_nearest_param_.findIndex(traj_points, ego_odom_ptr_->pose.pose);
    const bool is_front_obstacle =
      isFrontObstacle(traj_points, ego_idx, current_obstacle_pose.pose.position);
    if (!is_front_obstacle) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_, "Ignore obstacle (%s) since it is not front obstacle.",
        object_id.c_str());
      continue;
    }

    // 3. Check if rough lateral distance is smaller than the threshold
    const double min_lat_dist_to_traj_poly = [&]() {
      const double lat_dist_from_obstacle_to_traj =
        motion_utils::calcLateralOffset(traj_points, current_obstacle_pose.pose.position);
      const double obstacle_max_length = calcObstacleMaxLength(predicted_object.shape);
      return std::abs(lat_dist_from_obstacle_to_traj) - vehicle_info_.vehicle_width_m -
             obstacle_max_length;
    }();
    const auto & p = behavior_determination_param_;
    const double max_lat_margin = std::max(
      std::max(p.max_lat_margin_for_stop, p.max_lat_margin_for_cruise),
      p.max_lat_margin_for_slow_down);
    if (max_lat_margin < min_lat_dist_to_traj_poly) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_,
        "Ignore obstacle (%s) since it is too far from the trajectory.", object_id.c_str());
      continue;
    }

    const auto target_obstacle = Obstacle(obj_stamp, predicted_object, current_obstacle_pose.pose);
    target_obstacles.push_back(target_obstacle);
  }

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_debug_info_, "  %s := %f [ms]", __func__,
    calculation_time);

  return target_obstacles;
}

bool ObstacleCruisePlannerNode::isStopObstacle(const uint8_t label) const
{
  const auto & types = stop_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isInsideCruiseObstacle(const uint8_t label) const
{
  const auto & types = inside_cruise_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isOutsideCruiseObstacle(const uint8_t label) const
{
  const auto & types = outside_cruise_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isCruiseObstacle(const uint8_t label) const
{
  return isInsideCruiseObstacle(label) || isOutsideCruiseObstacle(label);
}

bool ObstacleCruisePlannerNode::isSlowDownObstacle(const uint8_t label) const
{
  const auto & types = slow_down_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isFrontCollideObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
  const size_t first_collision_idx) const
{
  const auto obstacle_idx = motion_utils::findNearestIndex(traj_points, obstacle.pose.position);

  const double obstacle_to_col_points_distance =
    motion_utils::calcSignedArcLength(traj_points, obstacle_idx, first_collision_idx);
  const double obstacle_max_length = calcObstacleMaxLength(obstacle.shape);

  // If the obstacle is far in front of the collision point, the obstacle is behind the ego.
  return obstacle_to_col_points_distance > -obstacle_max_length;
}

std::tuple<std::vector<StopObstacle>, std::vector<CruiseObstacle>, std::vector<SlowDownObstacle>>
ObstacleCruisePlannerNode::determineEgoBehaviorAgainstObstacles(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Obstacle> & obstacles)
{
  stop_watch_.tic(__func__);

  // calculated decimated trajectory points and trajectory polygon
  const auto decimated_traj_points = decimateTrajectoryPoints(traj_points);
  const auto decimated_traj_polys =
    createOneStepPolygons(decimated_traj_points, vehicle_info_, ego_odom_ptr_->pose.pose);
  debug_data_ptr_->detection_polygons = decimated_traj_polys;

  // determine ego's behavior from stop, cruise and slow down
  std::vector<StopObstacle> stop_obstacles;
  std::vector<CruiseObstacle> cruise_obstacles;
  std::vector<SlowDownObstacle> slow_down_obstacles;
  slow_down_condition_counter_.resetCurrentUuids();
  for (const auto & obstacle : obstacles) {
    const auto obstacle_poly = obstacle.toPolygon();

    // Calculate distance between trajectory and obstacle first
    double precise_lat_dist = std::numeric_limits<double>::max();
    for (const auto & traj_poly : decimated_traj_polys) {
      const double current_precise_lat_dist = bg::distance(traj_poly, obstacle_poly);
      precise_lat_dist = std::min(precise_lat_dist, current_precise_lat_dist);
    }

    // Filter obstacles for cruise, stop and slow down
    const auto cruise_obstacle =
      createCruiseObstacle(decimated_traj_points, decimated_traj_polys, obstacle, precise_lat_dist);
    if (cruise_obstacle) {
      cruise_obstacles.push_back(*cruise_obstacle);
      continue;
    }
    const auto stop_obstacle =
      createStopObstacle(decimated_traj_points, decimated_traj_polys, obstacle, precise_lat_dist);
    if (stop_obstacle) {
      stop_obstacles.push_back(*stop_obstacle);
      continue;
    }
    const auto slow_down_obstacle =
      createSlowDownObstacle(decimated_traj_points, obstacle, precise_lat_dist);
    if (slow_down_obstacle) {
      slow_down_obstacles.push_back(*slow_down_obstacle);
      continue;
    }
  }
  slow_down_condition_counter_.removeCounterUnlessUpdated();

  // Check target obstacles' consistency
  checkConsistency(objects_ptr_->header.stamp, *objects_ptr_, stop_obstacles);

  // update previous obstacles
  prev_stop_obstacles_ = stop_obstacles;
  prev_cruise_obstacles_ = cruise_obstacles;
  prev_slow_down_obstacles_ = slow_down_obstacles;

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_calculation_time_info_, "  %s := %f [ms]",
    __func__, calculation_time);

  return {stop_obstacles, cruise_obstacles, slow_down_obstacles};
}

std::vector<TrajectoryPoint> ObstacleCruisePlannerNode::decimateTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  const auto & p = behavior_determination_param_;

  // trim trajectory
  const size_t ego_seg_idx =
    ego_nearest_param_.findSegmentIndex(traj_points, ego_odom_ptr_->pose.pose);
  const size_t traj_start_point_idx = ego_seg_idx;
  const auto trimmed_traj_points =
    std::vector<TrajectoryPoint>(traj_points.begin() + traj_start_point_idx, traj_points.end());

  // decimate trajectory
  const auto decimated_traj_points =
    resampleTrajectoryPoints(trimmed_traj_points, p.decimate_trajectory_step_length);

  // extend trajectory
  const auto extended_traj_points = extendTrajectoryPoints(
    decimated_traj_points, planner_ptr_->getSafeDistanceMargin(),
    p.decimate_trajectory_step_length);
  if (extended_traj_points.size() < 2) {
    return traj_points;
  }
  return extended_traj_points;
}

std::optional<CruiseObstacle> ObstacleCruisePlannerNode::createCruiseObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle, const double precise_lat_dist)
{
  const auto & object_id = obstacle.uuid.substr(0, 4);
  const auto & p = behavior_determination_param_;

  // NOTE: When driving backward, Stop will be planned instead of cruise.
  //       When the obstacle is crossing the ego's trajectory, cruise can be ignored.
  if (!isCruiseObstacle(obstacle.classification.label) || !is_driving_forward_) {
    return std::nullopt;
  }
  if (p.max_lat_margin_for_cruise < precise_lat_dist) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore obstacle (%s) since it's far from trajectory.", object_id.c_str());
    return std::nullopt;
  }
  if (isObstacleCrossing(traj_points, obstacle)) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore obstacle (%s) since it's crossing the ego's trajectory..",
      object_id.c_str());
    return std::nullopt;
  }

  const auto collision_points = [&]() -> std::optional<std::vector<PointWithStamp>> {
    constexpr double epsilon = 1e-6;
    if (precise_lat_dist < epsilon) {
      // obstacle is inside the trajectory
      return createCollisionPointsForInsideCruiseObstacle(traj_points, traj_polys, obstacle);
    }
    // obstacle is outside the trajectory
    return createCollisionPointsForOutsideCruiseObstacle(traj_points, traj_polys, obstacle);
  }();
  if (!collision_points) {
    return std::nullopt;
  }

  const auto [tangent_vel, normal_vel] = projectObstacleVelocityToTrajectory(traj_points, obstacle);
  return CruiseObstacle{obstacle.uuid, obstacle.stamp, obstacle.pose,
                        tangent_vel,   normal_vel,     *collision_points};
}

std::optional<std::vector<PointWithStamp>>
ObstacleCruisePlannerNode::createCollisionPointsForInsideCruiseObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle) const
{
  const auto & object_id = obstacle.uuid.substr(0, 4);
  const auto & p = behavior_determination_param_;

  // check label
  if (!isInsideCruiseObstacle(obstacle.classification.label)) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore inside obstacle (%s) since its type is not designated.", object_id.c_str());
    return std::nullopt;
  }

  {  // consider hysteresis
    const auto [obstacle_tangent_vel, obstacle_normal_vel] =
      projectObstacleVelocityToTrajectory(traj_points, obstacle);
    // const bool is_prev_obstacle_stop = getObstacleFromUuid(prev_stop_obstacles_,
    // obstacle.uuid).has_value();
    const bool is_prev_obstacle_cruise =
      getObstacleFromUuid(prev_cruise_obstacles_, obstacle.uuid).has_value();

    if (is_prev_obstacle_cruise) {
      if (obstacle_tangent_vel < p.obstacle_velocity_threshold_from_cruise_to_stop) {
        return std::nullopt;
      }
      // NOTE: else is keeping cruise
    } else {  // if (is_prev_obstacle_stop) {
      // TODO(murooka) consider hysteresis for slow down
      // If previous obstacle is stop or does not exist.
      if (obstacle_tangent_vel < p.obstacle_velocity_threshold_from_stop_to_cruise) {
        return std::nullopt;
      }
      // NOTE: else is cruise from stop
    }
  }

  // Get highest confidence predicted path
  const auto resampled_predicted_path = resampleHighestConfidencePredictedPath(
    obstacle.predicted_paths, p.prediction_resampling_time_interval,
    p.prediction_resampling_time_horizon);

  // calculate nearest collision point
  std::vector<size_t> collision_index;
  const auto collision_points = polygon_utils::getCollisionPoints(
    traj_points, traj_polys, obstacle.stamp, resampled_predicted_path, obstacle.shape, now(),
    is_driving_forward_, collision_index);
  return collision_points;
}

std::optional<std::vector<PointWithStamp>>
ObstacleCruisePlannerNode::createCollisionPointsForOutsideCruiseObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle) const
{
  const auto & p = behavior_determination_param_;
  const auto & object_id = obstacle.uuid.substr(0, 4);

  // check label
  if (!isOutsideCruiseObstacle(obstacle.classification.label)) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore outside obstacle (%s) since its type is not designated.", object_id.c_str());
    return std::nullopt;
  }

  if (
    std::hypot(obstacle.twist.linear.x, obstacle.twist.linear.y) <
    p.outside_obstacle_min_velocity_threshold) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore outside obstacle (%s) since the obstacle velocity is low.",
      object_id.c_str());
    return std::nullopt;
  }

  // Get highest confidence predicted path
  const auto resampled_predicted_path = resampleHighestConfidencePredictedPath(
    obstacle.predicted_paths, p.prediction_resampling_time_interval,
    p.prediction_resampling_time_horizon);

  // calculate collision condition for cruise
  std::vector<size_t> collision_index;
  const auto collision_points = polygon_utils::getCollisionPoints(
    traj_points, traj_polys, obstacle.stamp, resampled_predicted_path, obstacle.shape, now(),
    is_driving_forward_, collision_index,
    vehicle_info_.vehicle_width_m + p.max_lat_margin_for_cruise,
    p.max_prediction_time_for_collision_check);
  if (collision_points.empty()) {
    // Ignore vehicle obstacles outside the trajectory without collision
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore outside obstacle (%s) since there are no collision points.",
      object_id.c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
    return std::nullopt;
  }

  const double overlap_time =
    (rclcpp::Time(collision_points.back().stamp) - rclcpp::Time(collision_points.front().stamp))
      .seconds();
  if (overlap_time < p.ego_obstacle_overlap_time_threshold) {
    // Ignore vehicle obstacles outside the trajectory, whose predicted path
    // overlaps the ego trajectory in a certain time.
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore outside obstacle (%s) since it will not collide with the ego.",
      object_id.c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
    return std::nullopt;
  }

  // Ignore obstacles behind the ego vehicle.
  // Note: Only using isFrontObstacle(), behind obstacles cannot be filtered
  // properly when the trajectory is crossing or overlapping.
  const size_t first_collision_index = collision_index.front();
  if (!isFrontCollideObstacle(traj_points, obstacle, first_collision_index)) {
    return std::nullopt;
  }
  return collision_points;
}

std::optional<StopObstacle> ObstacleCruisePlannerNode::createStopObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle, const double precise_lat_dist) const
{
  const auto & p = behavior_determination_param_;
  const auto & object_id = obstacle.uuid.substr(0, 4);

  // NOTE: consider all target obstacles when driving backward
  if (!isStopObstacle(obstacle.classification.label)) {
    return std::nullopt;
  }
  if (p.max_lat_margin_for_stop < precise_lat_dist) {
    return std::nullopt;
  }

  // check obstacle velocity
  // NOTE: If precise_lat_dist is 0, always plan stop
  constexpr double epsilon = 1e-6;
  if (epsilon < precise_lat_dist) {
    const auto [obstacle_tangent_vel, obstacle_normal_vel] =
      projectObstacleVelocityToTrajectory(traj_points, obstacle);
    if (p.obstacle_velocity_threshold_from_stop_to_cruise <= obstacle_tangent_vel) {
      return std::nullopt;
    }
  }

  // NOTE: Dynamic obstacles for stop are crossing ego's trajectory with high speed,
  //       and the collision between ego and obstacles are within the margin threshold.
  const bool is_obstacle_crossing = isObstacleCrossing(traj_points, obstacle);
  const double has_high_speed = p.crossing_obstacle_velocity_threshold <
                                std::hypot(obstacle.twist.linear.x, obstacle.twist.linear.y);
  if (is_obstacle_crossing && has_high_speed) {
    // Get highest confidence predicted path
    const auto resampled_predicted_path = resampleHighestConfidencePredictedPath(
      obstacle.predicted_paths, p.prediction_resampling_time_interval,
      p.prediction_resampling_time_horizon);

    std::vector<size_t> collision_index;
    const auto collision_points = polygon_utils::getCollisionPoints(
      traj_points, traj_polys, obstacle.stamp, resampled_predicted_path, obstacle.shape, now(),
      is_driving_forward_, collision_index);
    if (collision_points.empty()) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_,
        "[Stop] Ignore inside obstacle (%s) since there is no collision point between the "
        "predicted path "
        "and the ego.",
        object_id.c_str());
      debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }

    const double collision_time_margin =
      calcCollisionTimeMargin(collision_points, traj_points, is_driving_forward_);
    if (p.collision_time_margin < collision_time_margin) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_,
        "[Stop] Ignore inside obstacle (%s) since it will not collide with the ego.",
        object_id.c_str());
      debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }
  }

  // calculate collision points with trajectory with lateral stop margin
  const auto traj_polys_with_lat_margin = createOneStepPolygons(
    traj_points, vehicle_info_, ego_odom_ptr_->pose.pose, p.max_lat_margin_for_stop);

  const auto collision_point = polygon_utils::getCollisionPoint(
    traj_points, traj_polys_with_lat_margin, obstacle, is_driving_forward_, vehicle_info_);
  if (!collision_point) {
    return std::nullopt;
  }

  const auto [tangent_vel, normal_vel] = projectObstacleVelocityToTrajectory(traj_points, obstacle);
  return StopObstacle{
    obstacle.uuid, obstacle.stamp, obstacle.pose,          obstacle.shape,
    tangent_vel,   normal_vel,     collision_point->first, collision_point->second};
}

std::optional<SlowDownObstacle> ObstacleCruisePlannerNode::createSlowDownObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
  const double precise_lat_dist)
{
  const auto & object_id = obstacle.uuid.substr(0, 4);
  const auto & p = behavior_determination_param_;
  slow_down_condition_counter_.addCurrentUuid(obstacle.uuid);

  const bool is_prev_obstacle_slow_down =
    getObstacleFromUuid(prev_slow_down_obstacles_, obstacle.uuid).has_value();

  if (!enable_slow_down_planning_ || !isSlowDownObstacle(obstacle.classification.label)) {
    return std::nullopt;
  }

  // check lateral distance considering hysteresis
  const bool is_lat_dist_low = isLowerConsideringHysteresis(
    precise_lat_dist, is_prev_obstacle_slow_down,
    p.max_lat_margin_for_slow_down + p.lat_hysteresis_margin_for_slow_down / 2.0,
    p.max_lat_margin_for_slow_down - p.lat_hysteresis_margin_for_slow_down / 2.0);

  const bool is_slow_down_required = [&]() {
    if (is_prev_obstacle_slow_down) {
      // check if exiting slow down
      if (!is_lat_dist_low) {
        const int count = slow_down_condition_counter_.decreaseCounter(obstacle.uuid);
        if (count <= -p.successive_num_to_exit_slow_down_condition) {
          slow_down_condition_counter_.reset(obstacle.uuid);
          return false;
        }
      }
      return true;
    }
    // check if entering slow down
    if (is_lat_dist_low) {
      const int count = slow_down_condition_counter_.increaseCounter(obstacle.uuid);
      if (p.successive_num_to_entry_slow_down_condition <= count) {
        slow_down_condition_counter_.reset(obstacle.uuid);
        return true;
      }
    }
    return false;
  }();
  if (!is_slow_down_required) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[SlowDown] Ignore obstacle (%s) since it's far from trajectory. (%f [m])", object_id.c_str(),
      precise_lat_dist);
    return std::nullopt;
  }

  const auto obstacle_poly = obstacle.toPolygon();

  // calculate collision points with trajectory with lateral stop margin
  // NOTE: For additional margin, hysteresis is not divided by two.
  const auto traj_polys_with_lat_margin = createOneStepPolygons(
    traj_points, vehicle_info_, ego_odom_ptr_->pose.pose,
    p.max_lat_margin_for_slow_down + p.lat_hysteresis_margin_for_slow_down);

  std::vector<Polygon2d> front_collision_polygons;
  size_t front_seg_idx = 0;
  std::vector<Polygon2d> back_collision_polygons;
  size_t back_seg_idx = 0;
  for (size_t i = 0; i < traj_polys_with_lat_margin.size(); ++i) {
    std::vector<Polygon2d> collision_polygons;
    bg::intersection(traj_polys_with_lat_margin.at(i), obstacle_poly, collision_polygons);

    if (!collision_polygons.empty()) {
      if (front_collision_polygons.empty()) {
        front_collision_polygons = collision_polygons;
        front_seg_idx = i == 0 ? i : i - 1;
      }
      back_collision_polygons = collision_polygons;
      back_seg_idx = i == 0 ? i : i - 1;
    } else {
      if (!back_collision_polygons.empty()) {
        break;  // for efficient calculation
      }
    }
  }

  if (front_collision_polygons.empty() || back_collision_polygons.empty()) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[SlowDown] Ignore obstacle (%s) since there is no collision point", object_id.c_str());
    return std::nullopt;
  }

  // calculate front collision point
  double front_min_dist = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point front_collision_point;
  for (const auto & collision_poly : front_collision_polygons) {
    for (const auto & collision_point : collision_poly.outer()) {
      const auto collision_geom_point = toGeomPoint(collision_point);
      const double dist = motion_utils::calcLongitudinalOffsetToSegment(
        traj_points, front_seg_idx, collision_geom_point);
      if (dist < front_min_dist) {
        front_min_dist = dist;
        front_collision_point = collision_geom_point;
      }
    }
  }

  // calculate back collision point
  double back_max_dist = -std::numeric_limits<double>::max();
  geometry_msgs::msg::Point back_collision_point = front_collision_point;
  for (const auto & collision_poly : back_collision_polygons) {
    for (const auto & collision_point : collision_poly.outer()) {
      const auto collision_geom_point = toGeomPoint(collision_point);
      const double dist = motion_utils::calcLongitudinalOffsetToSegment(
        traj_points, back_seg_idx, collision_geom_point);
      if (back_max_dist < dist) {
        back_max_dist = dist;
        back_collision_point = collision_geom_point;
      }
    }
  }

  const auto [tangent_vel, normal_vel] = projectObstacleVelocityToTrajectory(traj_points, obstacle);
  return SlowDownObstacle{obstacle.uuid,    obstacle.stamp,        obstacle.classification,
                          obstacle.pose,    tangent_vel,           normal_vel,
                          precise_lat_dist, front_collision_point, back_collision_point};
}

void ObstacleCruisePlannerNode::checkConsistency(
  const rclcpp::Time & current_time, const PredictedObjects & predicted_objects,
  std::vector<StopObstacle> & stop_obstacles)
{
  const auto current_closest_stop_obstacle =
    obstacle_cruise_utils::getClosestStopObstacle(stop_obstacles);

  // If previous closest obstacle ptr is not set
  if (!prev_closest_stop_obstacle_ptr_) {
    if (current_closest_stop_obstacle) {
      prev_closest_stop_obstacle_ptr_ =
        std::make_shared<StopObstacle>(*current_closest_stop_obstacle);
    }
    return;
  }

  // Put previous closest target obstacle if necessary
  const auto predicted_object_itr = std::find_if(
    predicted_objects.objects.begin(), predicted_objects.objects.end(),
    [&](PredictedObject predicted_object) {
      return tier4_autoware_utils::toHexString(predicted_object.object_id) ==
             prev_closest_stop_obstacle_ptr_->uuid;
    });

  // If previous closest obstacle is not in the current perception lists
  // just return the current target obstacles
  if (predicted_object_itr == predicted_objects.objects.end()) {
    return;
  }

  // Previous closest obstacle is in the perception lists
  const auto obstacle_itr = std::find_if(
    stop_obstacles.begin(), stop_obstacles.end(),
    [&](const auto & obstacle) { return obstacle.uuid == prev_closest_stop_obstacle_ptr_->uuid; });

  // Previous closest obstacle is both in the perception lists and target obstacles
  if (obstacle_itr != stop_obstacles.end()) {
    if (current_closest_stop_obstacle) {
      if ((current_closest_stop_obstacle->uuid == prev_closest_stop_obstacle_ptr_->uuid)) {
        // prev_closest_obstacle is current_closest_stop_obstacle just return the target
        // obstacles(in target obstacles)
        prev_closest_stop_obstacle_ptr_ =
          std::make_shared<StopObstacle>(*current_closest_stop_obstacle);
      } else {
        // New obstacle becomes new stop obstacle
        prev_closest_stop_obstacle_ptr_ =
          std::make_shared<StopObstacle>(*current_closest_stop_obstacle);
      }
    } else {
      // Previous closest stop obstacle becomes cruise obstacle
      prev_closest_stop_obstacle_ptr_ = nullptr;
    }
  } else {
    // prev obstacle is not in the target obstacles, but in the perception list
    const double elapsed_time = (current_time - prev_closest_stop_obstacle_ptr_->stamp).seconds();
    if (
      predicted_object_itr->kinematics.initial_twist_with_covariance.twist.linear.x <
        behavior_determination_param_.obstacle_velocity_threshold_from_stop_to_cruise &&
      elapsed_time < behavior_determination_param_.stop_obstacle_hold_time_threshold) {
      stop_obstacles.push_back(*prev_closest_stop_obstacle_ptr_);
      return;
    }

    if (current_closest_stop_obstacle) {
      prev_closest_stop_obstacle_ptr_ =
        std::make_shared<StopObstacle>(*current_closest_stop_obstacle);
    } else {
      prev_closest_stop_obstacle_ptr_ = nullptr;
    }
  }
}

bool ObstacleCruisePlannerNode::isObstacleCrossing(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle) const
{
  const double diff_angle = calcDiffAngleAgainstTrajectory(traj_points, obstacle.pose);

  // NOTE: Currently predicted objects does not have orientation availability even
  // though sometimes orientation is not available.
  const bool is_obstacle_crossing_trajectory =
    behavior_determination_param_.crossing_obstacle_traj_angle_threshold < std::abs(diff_angle) &&
    behavior_determination_param_.crossing_obstacle_traj_angle_threshold <
      M_PI - std::abs(diff_angle);
  if (!is_obstacle_crossing_trajectory) {
    return false;
  }

  // Only obstacles crossing the ego's trajectory with high speed are considered.
  return true;
}

double ObstacleCruisePlannerNode::calcCollisionTimeMargin(
  const std::vector<PointWithStamp> & collision_points,
  const std::vector<TrajectoryPoint> & traj_points, const bool is_driving_forward) const
{
  const auto & ego_pose = ego_odom_ptr_->pose.pose;
  const double ego_vel = ego_odom_ptr_->twist.twist.linear.x;

  const double time_to_reach_collision_point = [&]() {
    const double abs_ego_offset = is_driving_forward
                                    ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                    : std::abs(vehicle_info_.min_longitudinal_offset_m);
    const double dist_from_ego_to_obstacle =
      std::abs(motion_utils::calcSignedArcLength(
        traj_points, ego_pose.position, collision_points.front().point)) -
      abs_ego_offset;
    return dist_from_ego_to_obstacle / std::max(1e-6, std::abs(ego_vel));
  }();

  const double time_to_start_cross =
    (rclcpp::Time(collision_points.front().stamp) - now()).seconds();
  const double time_to_end_cross = (rclcpp::Time(collision_points.back().stamp) - now()).seconds();

  if (time_to_reach_collision_point < time_to_start_cross) {  // Ego goes first.
    return time_to_start_cross - time_to_reach_collision_point;
  }
  if (time_to_end_cross < time_to_reach_collision_point) {  // Obstacle goes first.
    return time_to_reach_collision_point - time_to_end_cross;
  }
  return 0.0;  // Ego and obstacle will collide.
}

PlannerData ObstacleCruisePlannerNode::createPlannerData(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  PlannerData planner_data;
  planner_data.current_time = now();
  planner_data.traj_points = traj_points;
  planner_data.ego_pose = ego_odom_ptr_->pose.pose;
  planner_data.ego_vel = ego_odom_ptr_->twist.twist.linear.x;
  planner_data.ego_acc = ego_accel_ptr_->accel.accel.linear.x;
  planner_data.is_driving_forward = is_driving_forward_;
  return planner_data;
}

void ObstacleCruisePlannerNode::publishVelocityLimit(
  const std::optional<VelocityLimit> & vel_limit, const std::string & module_name)
{
  if (vel_limit) {
    vel_limit_pub_->publish(*vel_limit);
    need_to_clear_vel_limit_.at(module_name) = true;
    return;
  }

  if (!need_to_clear_vel_limit_.at(module_name)) {
    return;
  }

  // clear velocity limit
  const auto clear_vel_limit_msg = createVelocityLimitClearCommandMessage(now(), module_name);
  clear_vel_limit_pub_->publish(clear_vel_limit_msg);
  need_to_clear_vel_limit_.at(module_name) = false;
}

void ObstacleCruisePlannerNode::publishDebugMarker() const
{
  stop_watch_.tic(__func__);

  // 1. publish debug marker
  MarkerArray debug_marker;

  // obstacles to cruise
  std::vector<geometry_msgs::msg::Point> stop_collision_points;
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_cruise.size(); ++i) {
    // obstacle
    const auto obstacle_marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->obstacles_to_cruise.at(i).pose, i, "obstacles_to_cruise", 1.0, 0.6, 0.1);
    debug_marker.markers.push_back(obstacle_marker);

    // collision points
    for (size_t j = 0; j < debug_data_ptr_->obstacles_to_cruise.at(i).collision_points.size();
         ++j) {
      stop_collision_points.push_back(
        debug_data_ptr_->obstacles_to_cruise.at(i).collision_points.at(j).point);
    }
  }
  for (size_t i = 0; i < stop_collision_points.size(); ++i) {
    auto collision_point_marker = tier4_autoware_utils::createDefaultMarker(
      "map", now(), "cruise_collision_points", i, Marker::SPHERE,
      tier4_autoware_utils::createMarkerScale(0.25, 0.25, 0.25),
      tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
    collision_point_marker.pose.position = stop_collision_points.at(i);
    debug_marker.markers.push_back(collision_point_marker);
  }

  // obstacles to stop
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_stop.size(); ++i) {
    // obstacle
    const auto obstacle_marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->obstacles_to_stop.at(i).pose, i, "obstacles_to_stop", 1.0, 0.0, 0.0);
    debug_marker.markers.push_back(obstacle_marker);

    // collision point
    auto collision_point_marker = tier4_autoware_utils::createDefaultMarker(
      "map", now(), "stop_collision_points", 0, Marker::SPHERE,
      tier4_autoware_utils::createMarkerScale(0.25, 0.25, 0.25),
      tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
    collision_point_marker.pose.position = debug_data_ptr_->obstacles_to_stop.at(i).collision_point;
    debug_marker.markers.push_back(collision_point_marker);
  }

  // obstacles to slow down
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_slow_down.size(); ++i) {
    // obstacle
    const auto obstacle_marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->obstacles_to_slow_down.at(i).pose, i, "obstacles_to_slow_down", 0.7, 0.7,
      0.0);
    debug_marker.markers.push_back(obstacle_marker);

    // collision points
    auto front_collision_point_marker = tier4_autoware_utils::createDefaultMarker(
      "map", now(), "slow_down_collision_points", i * 2 + 0, Marker::SPHERE,
      tier4_autoware_utils::createMarkerScale(0.25, 0.25, 0.25),
      tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
    front_collision_point_marker.pose.position =
      debug_data_ptr_->obstacles_to_slow_down.at(i).front_collision_point;
    auto back_collision_point_marker = tier4_autoware_utils::createDefaultMarker(
      "map", now(), "slow_down_collision_points", i * 2 + 1, Marker::SPHERE,
      tier4_autoware_utils::createMarkerScale(0.25, 0.25, 0.25),
      tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
    back_collision_point_marker.pose.position =
      debug_data_ptr_->obstacles_to_slow_down.at(i).back_collision_point;

    debug_marker.markers.push_back(front_collision_point_marker);
    debug_marker.markers.push_back(back_collision_point_marker);
  }

  // intentionally ignored obstacles to cruise or stop
  for (size_t i = 0; i < debug_data_ptr_->intentionally_ignored_obstacles.size(); ++i) {
    const auto marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->intentionally_ignored_obstacles.at(i).pose, i,
      "intentionally_ignored_obstacles", 0.0, 1.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  {  // footprint polygons
    auto marker = tier4_autoware_utils::createDefaultMarker(
      "map", now(), "detection_polygons", 0, Marker::LINE_LIST,
      tier4_autoware_utils::createMarkerScale(0.01, 0.0, 0.0),
      tier4_autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (const auto & detection_polygon : debug_data_ptr_->detection_polygons) {
      for (size_t dp_idx = 0; dp_idx < detection_polygon.outer().size(); ++dp_idx) {
        const auto & current_point = detection_polygon.outer().at(dp_idx);
        const auto & next_point =
          detection_polygon.outer().at((dp_idx + 1) % detection_polygon.outer().size());

        marker.points.push_back(
          tier4_autoware_utils::createPoint(current_point.x(), current_point.y(), 0.0));
        marker.points.push_back(
          tier4_autoware_utils::createPoint(next_point.x(), next_point.y(), 0.0));
      }
    }
    debug_marker.markers.push_back(marker);
  }

  // slow down debug wall marker
  tier4_autoware_utils::appendMarkerArray(
    debug_data_ptr_->slow_down_debug_wall_marker, &debug_marker);

  debug_marker_pub_->publish(debug_marker);

  // 2. publish virtual wall for cruise and stop
  debug_cruise_wall_marker_pub_->publish(debug_data_ptr_->cruise_wall_marker);
  debug_stop_wall_marker_pub_->publish(debug_data_ptr_->stop_wall_marker);
  debug_slow_down_wall_marker_pub_->publish(debug_data_ptr_->slow_down_wall_marker);

  // 3. print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_calculation_time_info_, "  %s := %f [ms]",
    __func__, calculation_time);
}

void ObstacleCruisePlannerNode::publishDebugInfo() const
{
  // stop
  const auto stop_debug_msg = planner_ptr_->getStopPlanningDebugMessage(now());
  debug_stop_planning_info_pub_->publish(stop_debug_msg);

  // cruise
  const auto cruise_debug_msg = planner_ptr_->getCruisePlanningDebugMessage(now());
  debug_cruise_planning_info_pub_->publish(cruise_debug_msg);

  // slow_down
  const auto slow_down_debug_msg = planner_ptr_->getSlowDownPlanningDebugMessage(now());
  debug_slow_down_planning_info_pub_->publish(slow_down_debug_msg);
}

void ObstacleCruisePlannerNode::publishCalculationTime(const double calculation_time) const
{
  Float32Stamped calculation_time_msg;
  calculation_time_msg.stamp = now();
  calculation_time_msg.data = calculation_time;
  debug_calculation_time_pub_->publish(calculation_time_msg);
}
}  // namespace motion_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::ObstacleCruisePlannerNode)
