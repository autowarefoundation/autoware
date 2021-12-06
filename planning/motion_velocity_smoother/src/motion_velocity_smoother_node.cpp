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

#include "motion_velocity_smoother/motion_velocity_smoother_node.hpp"

#include "autoware_utils/ros/update_param.hpp"
#include "motion_velocity_smoother/smoother/jerk_filtered_smoother.hpp"
#include "motion_velocity_smoother/smoother/l2_pseudo_jerk_smoother.hpp"
#include "motion_velocity_smoother/smoother/linf_pseudo_jerk_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

// clang-format on
namespace motion_velocity_smoother
{
MotionVelocitySmootherNode::MotionVelocitySmootherNode(const rclcpp::NodeOptions & node_options)
: Node("motion_velocity_smoother", node_options)
{
  using std::placeholders::_1;

  // set common params
  initCommonParam();
  over_stop_velocity_warn_thr_ =
    declare_parameter("over_stop_velocity_warn_thr", autoware_utils::kmph2mps(5.0));

  // create smoother
  initSmootherBaseParam();
  switch (node_param_.algorithm_type) {
    case AlgorithmType::JERK_FILTERED: {
      initJerkFilteredSmootherParam();
      smoother_ = std::make_shared<JerkFilteredSmoother>(jerk_filtered_smoother_param_);
      smoother_->setParam(base_param_);

      // Set Publisher for jerk filtered algorithm
      pub_forward_filtered_trajectory_ =
        create_publisher<Trajectory>("~/debug/forward_filtered_trajectory", 1);
      pub_backward_filtered_trajectory_ =
        create_publisher<Trajectory>("~/debug/backward_filtered_trajectory", 1);
      pub_merged_filtered_trajectory_ =
        create_publisher<Trajectory>("~/debug/merged_filtered_trajectory", 1);
      pub_closest_merged_velocity_ =
        create_publisher<Float32Stamped>("~/closest_merged_velocity", 1);
      break;
    }
    case AlgorithmType::L2: {
      initL2PseudoJerkSmootherParam();
      smoother_ = std::make_shared<L2PseudoJerkSmoother>(l2_pseudo_jerk_smoother_param_);
      smoother_->setParam(base_param_);
      break;
    }
    case AlgorithmType::LINF: {
      initLinfPseudoJerkSmootherParam();
      smoother_ = std::make_shared<LinfPseudoJerkSmoother>(linf_pseudo_jerk_smoother_param_);
      smoother_->setParam(base_param_);
      break;
    }
    case AlgorithmType::ANALYTICAL: {
      initAnalyticalJerkConstrainedSmootherParam();
      smoother_ = std::make_shared<AnalyticalJerkConstrainedSmoother>(
        analytical_jerk_constrained_smoother_param_);
      smoother_->setParam(base_param_);
      break;
    }
    default:
      throw std::domain_error("[MotionVelocitySmootherNode] invalid algorithm");
  }

  // publishers, subscribers
  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_velocity_limit_ = create_publisher<VelocityLimit>(
    "~/output/current_velocity_limit_mps", rclcpp::QoS{1}.transient_local());
  pub_dist_to_stopline_ = create_publisher<Float32Stamped>("~/distance_to_stopline", 1);
  pub_over_stop_velocity_ = create_publisher<StopSpeedExceeded>("~/stop_speed_exceeded", 1);
  sub_current_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&MotionVelocitySmootherNode::onCurrentTrajectory, this, _1));
  sub_current_odometry_ = create_subscription<Odometry>(
    "/localization/kinematic_state", 1,
    std::bind(&MotionVelocitySmootherNode::onCurrentOdometry, this, _1));
  sub_external_velocity_limit_ = create_subscription<VelocityLimit>(
    "~/input/external_velocity_limit_mps", 1,
    std::bind(&MotionVelocitySmootherNode::onExternalVelocityLimit, this, _1));

  // parameter update
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MotionVelocitySmootherNode::onParameter, this, _1));

  // debug
  publish_debug_trajs_ = declare_parameter("publish_debug_trajs", false);
  debug_closest_velocity_ = create_publisher<Float32Stamped>("~/closest_velocity", 1);
  debug_closest_acc_ = create_publisher<Float32Stamped>("~/closest_acceleration", 1);
  debug_closest_jerk_ = create_publisher<Float32Stamped>("~/closest_jerk", 1);
  debug_closest_max_velocity_ = create_publisher<Float32Stamped>("~/closest_max_velocity", 1);
  debug_calculation_time_ = create_publisher<Float32Stamped>("~/calculation_time", 1);
  pub_trajectory_raw_ = create_publisher<Trajectory>("~/debug/trajectory_raw", 1);
  pub_trajectory_vel_lim_ =
    create_publisher<Trajectory>("~/debug/trajectory_external_velocity_limited", 1);
  pub_trajectory_latacc_filtered_ =
    create_publisher<Trajectory>("~/debug/trajectory_lateral_acc_filtered", 1);
  pub_trajectory_resampled_ = create_publisher<Trajectory>("~/debug/trajectory_time_resampled", 1);

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  external_velocity_limit_ = node_param_.max_velocity;
  max_velocity_with_deceleration_ = node_param_.max_velocity;

  // publish default max velocity
  VelocityLimit max_vel_msg{};
  max_vel_msg.stamp = this->now();
  max_vel_msg.max_velocity = node_param_.max_velocity;
  pub_velocity_limit_->publish(max_vel_msg);
}

rcl_interfaces::msg::SetParametersResult MotionVelocitySmootherNode::onParameter(
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
  {
    auto & p = node_param_;
    update_param("max_velocity", p.max_velocity);
    update_param(
      "margin_to_insert_external_velocity_limit", p.margin_to_insert_external_velocity_limit);
    update_param("replan_vel_deviation", p.replan_vel_deviation);
    update_param("engage_velocity", p.engage_velocity);
    update_param("engage_acceleration", p.engage_acceleration);
    update_param("engage_exit_ratio", p.engage_exit_ratio);
    update_param("stopping_velocity", p.stopping_velocity);
    update_param("stopping_distance", p.stopping_distance);
    update_param("extract_ahead_dist", p.extract_ahead_dist);
    update_param("extract_behind_dist", p.extract_behind_dist);
    update_param("stop_dist_to_prohibit_engage", p.stop_dist_to_prohibit_engage);
    update_param("delta_yaw_threshold", p.delta_yaw_threshold);
  }

  {
    auto & p = base_param_;
    update_param("normal.max_acc", p.max_accel);
    update_param("normal.min_acc", p.min_decel);
    update_param("stop_decel", p.stop_decel);
    update_param("normal.max_jerk", p.max_jerk);
    update_param("normal.min_jerk", p.min_jerk);
    update_param("max_lateral_accel", p.max_lateral_accel);
    update_param("min_curve_velocity", p.min_curve_velocity);
    update_param("decel_distance_before_curve", p.decel_distance_before_curve);
    update_param("decel_distance_after_curve", p.decel_distance_after_curve);
    update_param("max_trajectory_length", p.resample_param.max_trajectory_length);
    update_param("min_trajectory_length", p.resample_param.min_trajectory_length);
    update_param("resample_time", p.resample_param.resample_time);
    update_param("dense_resample_dt", p.resample_param.dense_resample_dt);
    update_param("min_interval_distance", p.resample_param.dense_min_interval_distance);
    update_param("sparse_resample_dt", p.resample_param.sparse_resample_dt);
    update_param("sparse_min_interval_distance", p.resample_param.sparse_min_interval_distance);
    smoother_->setParam(p);
  }

  switch (node_param_.algorithm_type) {
    case AlgorithmType::JERK_FILTERED: {
      auto & p = jerk_filtered_smoother_param_;
      update_param("jerk_weight", p.jerk_weight);
      update_param("over_v_weight", p.over_v_weight);
      update_param("over_a_weight", p.over_a_weight);
      update_param("over_j_weight", p.over_j_weight);
      update_param("jerk_filter_ds", p.jerk_filter_ds);
      std::dynamic_pointer_cast<JerkFilteredSmoother>(smoother_)->setParam(p);
      break;
    }
    case AlgorithmType::L2: {
      auto & p = l2_pseudo_jerk_smoother_param_;
      update_param("pseudo_jerk_weight", p.pseudo_jerk_weight);
      update_param("over_v_weight", p.over_v_weight);
      update_param("over_a_weight", p.over_a_weight);
      std::dynamic_pointer_cast<L2PseudoJerkSmoother>(smoother_)->setParam(p);
      break;
    }
    case AlgorithmType::LINF: {
      auto & p = linf_pseudo_jerk_smoother_param_;
      update_param("pseudo_jerk_weight", p.pseudo_jerk_weight);
      update_param("over_v_weight", p.over_v_weight);
      update_param("over_a_weight", p.over_a_weight);
      std::dynamic_pointer_cast<LinfPseudoJerkSmoother>(smoother_)->setParam(p);
      break;
    }
    case AlgorithmType::ANALYTICAL: {
      auto & p = analytical_jerk_constrained_smoother_param_;
      update_param("resample.ds_resample", p.resample.ds_resample);
      update_param("resample.num_resample", p.resample.num_resample);
      update_param("resample.delta_yaw_threshold", p.resample.delta_yaw_threshold);
      update_param(
        "latacc.constant_velocity_dist_threshold", p.latacc.constant_velocity_dist_threshold);
      update_param("forward.max_acc", p.forward.max_acc);
      update_param("forward.min_acc", p.forward.min_acc);
      update_param("forward.max_jerk", p.forward.max_jerk);
      update_param("forward.min_jerk", p.forward.min_jerk);
      update_param("forward.kp", p.forward.kp);
      update_param("backward.start_jerk", p.backward.start_jerk);
      update_param("backward.min_jerk_mild_stop", p.backward.min_jerk_mild_stop);
      update_param("backward.min_jerk", p.backward.min_jerk);
      update_param("backward.min_acc_mild_stop", p.backward.min_acc_mild_stop);
      update_param("backward.min_acc", p.backward.min_acc);
      update_param("backward.span_jerk", p.backward.span_jerk);
      std::dynamic_pointer_cast<AnalyticalJerkConstrainedSmoother>(smoother_)->setParam(p);
      break;
    }
    default:
      throw std::domain_error("[MotionVelocitySmootherNode] invalid algorithm");
  }

  rcl_interfaces::msg::SetParametersResult result{};
  result.successful = true;
  result.reason = "success";
  return result;
}

void MotionVelocitySmootherNode::initCommonParam()
{
  auto & p = node_param_;
  p.max_velocity = declare_parameter("max_velocity", 20.0);  // 72.0 kmph
  p.margin_to_insert_external_velocity_limit =
    declare_parameter("margin_to_insert_external_velocity_limit", 0.3);
  p.replan_vel_deviation = declare_parameter("replan_vel_deviation", 3.0);
  p.engage_velocity = declare_parameter("engage_velocity", 0.3);
  p.engage_acceleration = declare_parameter("engage_acceleration", 0.1);
  p.engage_exit_ratio = declare_parameter("engage_exit_ratio", 0.5);
  p.engage_exit_ratio = std::min(std::max(p.engage_exit_ratio, 0.0), 1.0);
  p.stopping_velocity = declare_parameter("stopping_velocity", autoware_utils::kmph2mps(10.0));
  p.stopping_distance = declare_parameter("stopping_distance", 0.0);
  p.extract_ahead_dist = declare_parameter("extract_ahead_dist", 200.0);
  p.extract_behind_dist = declare_parameter("extract_behind_dist", 3.0);
  p.stop_dist_to_prohibit_engage = declare_parameter("stop_dist_to_prohibit_engage", 1.5);
  p.delta_yaw_threshold = declare_parameter("delta_yaw_threshold", M_PI / 3.0);
  p.post_resample_param.max_trajectory_length =
    declare_parameter("post_max_trajectory_length", 300.0);
  p.post_resample_param.min_trajectory_length =
    declare_parameter("post_min_trajectory_length", 30.0);
  p.post_resample_param.resample_time = declare_parameter("post_resample_time", 10.0);
  p.post_resample_param.dense_resample_dt = declare_parameter("post_dense_resample_dt", 0.1);
  p.post_resample_param.dense_min_interval_distance =
    declare_parameter("post_dense_min_interval_distance", 0.1);
  p.post_resample_param.sparse_resample_dt = declare_parameter("post_sparse_resample_dt", 0.1);
  p.post_resample_param.sparse_min_interval_distance =
    declare_parameter("post_sparse_min_interval_distance", 1.0);
  p.algorithm_type = getAlgorithmType(declare_parameter("algorithm_type", "JerkFiltered"));
}

void MotionVelocitySmootherNode::initSmootherBaseParam()
{
  auto & p = base_param_;
  p.max_accel = declare_parameter("normal.max_acc", 2.0);   // 0.11G
  p.min_decel = declare_parameter("normal.min_acc", -3.0);  // -0.2G
  p.stop_decel = declare_parameter("stop_decel", 0.0);
  p.max_jerk = declare_parameter("normal.max_jerk", 0.3);
  p.min_jerk = declare_parameter("normal.min_jerk", -0.1);
  p.max_lateral_accel = declare_parameter("max_lateral_accel", 0.2);
  p.decel_distance_before_curve = declare_parameter("decel_distance_before_curve", 3.5);
  p.decel_distance_after_curve = declare_parameter("decel_distance_after_curve", 0.0);
  p.min_curve_velocity = declare_parameter("min_curve_velocity", 1.38);
  p.resample_param.max_trajectory_length = declare_parameter("max_trajectory_length", 200.0);
  p.resample_param.min_trajectory_length = declare_parameter("min_trajectory_length", 30.0);
  p.resample_param.resample_time = declare_parameter("resample_time", 10.0);
  p.resample_param.dense_resample_dt = declare_parameter("dense_resample_dt", 0.1);
  p.resample_param.dense_min_interval_distance =
    declare_parameter("dense_min_interval_distance", 0.1);
  p.resample_param.sparse_resample_dt = declare_parameter("sparse_resample_dt", 0.5);
  p.resample_param.sparse_min_interval_distance =
    declare_parameter("sparse_min_interval_distance", 4.0);
}

void MotionVelocitySmootherNode::initJerkFilteredSmootherParam()
{
  auto & p = jerk_filtered_smoother_param_;
  p.jerk_weight = declare_parameter("jerk_weight", 10.0);
  p.over_v_weight = declare_parameter("over_v_weight", 100000.0);
  p.over_a_weight = declare_parameter("over_a_weight", 5000.0);
  p.over_j_weight = declare_parameter("over_j_weight", 2000.0);
  p.jerk_filter_ds = declare_parameter("jerk_filter_ds", 0.1);
}

void MotionVelocitySmootherNode::initL2PseudoJerkSmootherParam()
{
  auto & p = l2_pseudo_jerk_smoother_param_;
  p.pseudo_jerk_weight = declare_parameter("pseudo_jerk_weight", 100.0);
  p.over_v_weight = declare_parameter("over_v_weight", 100000.0);
  p.over_a_weight = declare_parameter("over_a_weight", 1000.0);
}

void MotionVelocitySmootherNode::initLinfPseudoJerkSmootherParam()
{
  auto & p = linf_pseudo_jerk_smoother_param_;
  p.pseudo_jerk_weight = declare_parameter("pseudo_jerk_weight", 200.0);
  p.over_v_weight = declare_parameter("over_v_weight", 100000.0);
  p.over_a_weight = declare_parameter("over_a_weight", 5000.0);
}

void MotionVelocitySmootherNode::initAnalyticalJerkConstrainedSmootherParam()
{
  auto & p = analytical_jerk_constrained_smoother_param_;
  p.resample.ds_resample = declare_parameter("resample.ds_resample", 0.1);
  p.resample.num_resample = declare_parameter("resample.num_resample", 1);
  p.resample.delta_yaw_threshold = declare_parameter("resample.delta_yaw_threshold", 0.785);

  p.latacc.enable_constant_velocity_while_turning =
    declare_parameter("latacc.enable_constant_velocity_while_turning", false);
  p.latacc.constant_velocity_dist_threshold =
    declare_parameter("latacc.constant_velocity_dist_threshold", 2.0);

  p.forward.max_acc = declare_parameter("forward.max_acc", 1.0);
  p.forward.min_acc = declare_parameter("forward.min_acc", -1.0);
  p.forward.max_jerk = declare_parameter("forward.max_jerk", 0.3);
  p.forward.min_jerk = declare_parameter("forward.min_jerk", -0.3);
  p.forward.kp = declare_parameter("forward.kp", 0.3);

  p.backward.start_jerk = declare_parameter("backward.start_jerk", -0.1);
  p.backward.min_jerk_mild_stop = declare_parameter("backward.min_jerk_mild_stop", -0.3);
  p.backward.min_jerk = declare_parameter("backward.min_jerk", -1.5);
  p.backward.min_acc_mild_stop = declare_parameter("backward.min_acc_mild_stop", -1.0);
  p.backward.min_acc = declare_parameter("backward.min_acc", -2.5);
  p.backward.span_jerk = declare_parameter("backward.span_jerk", -0.01);
}

void MotionVelocitySmootherNode::publishTrajectory(const TrajectoryPoints & trajectory) const
{
  Trajectory publishing_trajectory = autoware_utils::convertToTrajectory(trajectory);
  publishing_trajectory.header = base_traj_raw_ptr_->header;
  pub_trajectory_->publish(publishing_trajectory);
}

void MotionVelocitySmootherNode::onCurrentOdometry(const Odometry::ConstSharedPtr msg)
{
  current_odometry_ptr_ = msg;
}

void MotionVelocitySmootherNode::onExternalVelocityLimit(const VelocityLimit::ConstSharedPtr msg)
{
  constexpr double eps = 1.0E-04;
  const double margin = node_param_.margin_to_insert_external_velocity_limit;

  // calculate distance and maximum velocity
  // to decelerate to external velocity limit with jerk and acceleration
  // constraints
  if (!prev_output_.empty()) {
    // if external velocity limit decreases
    if ((external_velocity_limit_ - msg->max_velocity) > eps) {
      if (prev_closest_point_) {
        const double v0 = prev_closest_point_->longitudinal_velocity_mps;
        const double a0 = prev_closest_point_->acceleration_mps2;

        if (isEngageStatus(v0)) {
          max_velocity_with_deceleration_ = external_velocity_limit_;
          external_velocity_limit_dist_ = 0.0;
        } else {
          const double a_min =
            msg->use_constraints ? msg->constraints.min_acceleration : smoother_->getMinDecel();
          const double j_max =
            msg->use_constraints ? msg->constraints.max_jerk : smoother_->getMaxJerk();
          const double j_min =
            msg->use_constraints ? msg->constraints.min_jerk : smoother_->getMinJerk();
          double stop_dist;
          std::map<double, double> jerk_profile;
          if (!trajectory_utils::calcStopDistWithJerkConstraints(
                v0, a0, j_max, j_min, a_min, msg->max_velocity, jerk_profile, stop_dist)) {
            RCLCPP_WARN(get_logger(), "Stop distance calculation is failed!");
          }
          external_velocity_limit_dist_ = stop_dist + margin;
          // If the closest acceleration is positive, velocity will increase
          // until the acceleration becomes zero
          // So we set the maximum increased velocity as the velocity limit
          if (a0 > 0) {
            max_velocity_with_deceleration_ = v0 - 0.5 * a0 * a0 / j_min;
          } else {
            max_velocity_with_deceleration_ = v0;
          }

          if (max_velocity_with_deceleration_ < msg->max_velocity) {
            max_velocity_with_deceleration_ = msg->max_velocity;
            external_velocity_limit_dist_ = 0.0;
          }
        }
      }
      // if external velocity limit increases
    } else if ((msg->max_velocity - external_velocity_limit_) > eps) {
      max_velocity_with_deceleration_ = msg->max_velocity;
    }
  }

  external_velocity_limit_ = msg->max_velocity;
  pub_velocity_limit_->publish(*msg);
}

bool MotionVelocitySmootherNode::checkData() const
{
  if (!current_pose_ptr_ || !current_odometry_ptr_ || !base_traj_raw_ptr_) {
    RCLCPP_DEBUG(
      get_logger(), "wait topics : current_pose = %d, current_vel = %d, base_traj = %d",
      (bool)current_pose_ptr_, (bool)current_odometry_ptr_, (bool)base_traj_raw_ptr_);
    return false;
  }
  if (base_traj_raw_ptr_->points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "input trajectory size must > 1. Skip computation.");
    return false;
  }

  return true;
}

void MotionVelocitySmootherNode::onCurrentTrajectory(const Trajectory::ConstSharedPtr msg)
{
  base_traj_raw_ptr_ = msg;

  stop_watch_.tic();
  RCLCPP_DEBUG(get_logger(), "========================= run start =========================");

  current_pose_ptr_ = self_pose_listener_.getCurrentPose();

  // guard
  if (!checkData()) {
    return;
  }

  // calculate distance to insert external velocity limit
  if (!prev_output_.empty()) {
    const double travel_dist = calcTravelDistance();
    external_velocity_limit_dist_ -= travel_dist;
    external_velocity_limit_dist_ = std::max(external_velocity_limit_dist_, 0.0);
    RCLCPP_DEBUG(
      get_logger(), "run: travel_dist = %f, external_velocity_limit_dist_ = %f", travel_dist,
      external_velocity_limit_dist_);
  }

  // calculate trajectory velocity
  TrajectoryPoints output =
    calcTrajectoryVelocity(autoware_utils::convertToTrajectoryPointArray(*base_traj_raw_ptr_));
  if (output.empty()) {
    RCLCPP_WARN(get_logger(), "Output Point is empty");
    return;
  }

  // Get the nearest point
  const auto output_closest_idx = autoware_utils::findNearestIndex(
    output, current_pose_ptr_->pose, std::numeric_limits<double>::max(),
    node_param_.delta_yaw_threshold);
  const auto output_closest_point =
    trajectory_utils::calcInterpolatedTrajectoryPoint(output, current_pose_ptr_->pose);
  if (!output_closest_idx) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Cannot find closest waypoint for output trajectory");
    return;
  }

  // Resample the optimized trajectory
  auto output_resampled = resampling::resampleTrajectory(
    output, current_odometry_ptr_->twist.twist.linear.x, *output_closest_idx,
    node_param_.post_resample_param);
  if (!output_resampled) {
    RCLCPP_WARN(get_logger(), "Failed to get the resampled output trajectory");
    return;
  }

  // Set 0 at the end of the trajectory
  if (!output_resampled->empty()) {
    output_resampled->back().longitudinal_velocity_mps = 0.0;
  }

  // publish message
  publishTrajectory(*output_resampled);

  // publish debug message
  publishStopDistance(output, *output_closest_idx);
  publishClosestState(output_closest_point);

  prev_output_ = output;
  prev_closest_point_ = output_closest_point;

  // Publish Calculation Time
  Float32Stamped calculation_time_data{};
  calculation_time_data.stamp = this->now();
  calculation_time_data.data = stop_watch_.toc();
  debug_calculation_time_->publish(calculation_time_data);
  RCLCPP_DEBUG(get_logger(), "run: calculation time = %f [ms]", calculation_time_data.data);
  RCLCPP_DEBUG(get_logger(), "========================== run() end ==========================\n\n");
}

TrajectoryPoints MotionVelocitySmootherNode::calcTrajectoryVelocity(
  const TrajectoryPoints & traj_input) const
{
  TrajectoryPoints output{};  // velocity is optimized by qp solver

  // Extract trajectory around self-position with desired forward-backward length
  const auto input_closest = autoware_utils::findNearestIndex(
    traj_input, current_pose_ptr_->pose, std::numeric_limits<double>::max(),
    node_param_.delta_yaw_threshold);
  if (!input_closest) {
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    RCLCPP_WARN_THROTTLE(
      get_logger(), clock, 5000, "Cannot find the closest point from input trajectory");
    return prev_output_;
  }

  auto traj_extracted = trajectory_utils::extractPathAroundIndex(
    traj_input, *input_closest, node_param_.extract_ahead_dist, node_param_.extract_behind_dist);
  if (!traj_extracted) {
    RCLCPP_WARN(get_logger(), "Fail to extract the path from the input trajectory");
    return prev_output_;
  }

  // Smoother can not handle negative velocity,
  // so multiple -1 to velocity if any trajectory points have reverse
  // velocity
  const bool is_reverse = std::any_of(
    traj_extracted->begin(), traj_extracted->end(),
    [](auto & pt) { return pt.longitudinal_velocity_mps < 0; });
  if (is_reverse) {
    for (auto & pt : *traj_extracted) {
      pt.longitudinal_velocity_mps *= -1.0;
    }
  }

  // Debug
  if (publish_debug_trajs_) {
    pub_trajectory_raw_->publish(toTrajectoryMsg(*traj_extracted, base_traj_raw_ptr_->header));
  }

  // Apply external velocity limit
  applyExternalVelocityLimit(*traj_extracted);

  // Change trajectory velocity to zero when current_velocity == 0 & stop_dist is close
  const auto traj_extracted_closest = autoware_utils::findNearestIndex(
    *traj_extracted, current_pose_ptr_->pose, std::numeric_limits<double>::max(),
    node_param_.delta_yaw_threshold);
  if (!traj_extracted_closest) {
    RCLCPP_WARN(get_logger(), "Cannot find the closest point from extracted trajectory");
    return prev_output_;
  }

  // Apply velocity to approach stop point
  applyStopApproachingVelocity(*traj_extracted);

  // Debug
  if (publish_debug_trajs_) {
    pub_trajectory_vel_lim_->publish(toTrajectoryMsg(*traj_extracted, base_traj_raw_ptr_->header));
  }

  // Smoothing velocity
  if (!smoothVelocity(*traj_extracted, output)) {
    return prev_output_;
  }

  // for reverse velocity
  if (is_reverse) {
    for (auto & pt : output) {
      pt.longitudinal_velocity_mps *= -1.0;
    }
  }

  return output;
}

bool MotionVelocitySmootherNode::smoothVelocity(
  const TrajectoryPoints & input, TrajectoryPoints & traj_smoothed) const
{
  // Lateral acceleration limit
  const auto traj_lateral_acc_filtered = smoother_->applyLateralAccelerationFilter(input);
  if (!traj_lateral_acc_filtered) {
    return false;
  }

  // Resample trajectory with ego-velocity based interval distance
  const auto traj_pre_resampled_closest = autoware_utils::findNearestIndex(
    *traj_lateral_acc_filtered, current_pose_ptr_->pose, std::numeric_limits<double>::max(),
    node_param_.delta_yaw_threshold);
  auto traj_resampled = smoother_->resampleTrajectory(
    *traj_lateral_acc_filtered, current_odometry_ptr_->twist.twist.linear.x,
    *traj_pre_resampled_closest);
  if (!traj_resampled) {
    RCLCPP_WARN(get_logger(), "Fail to do resampling before the optimization");
    return false;
  }

  // Set 0[m/s] in the terminal point
  if (!traj_resampled->empty()) {
    traj_resampled->back().longitudinal_velocity_mps = 0.0;
  }

  // Publish Closest Resample Trajectory Velocity
  publishClosestVelocity(*traj_resampled, current_pose_ptr_->pose, debug_closest_max_velocity_);

  // Calculate initial motion for smoothing
  double initial_vel{};
  double initial_acc{};
  InitializeType type{};
  const auto traj_resampled_closest = autoware_utils::findNearestIndex(
    *traj_resampled, current_pose_ptr_->pose, std::numeric_limits<double>::max(),
    node_param_.delta_yaw_threshold);
  if (!traj_resampled_closest) {
    RCLCPP_WARN(get_logger(), "Cannot find closest waypoint for resampled trajectory");
    return false;
  }
  std::tie(initial_vel, initial_acc, type) =
    calcInitialMotion(*traj_resampled, *traj_resampled_closest, prev_output_);

  // Clip trajectory from closest point
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(), traj_resampled->begin() + *traj_resampled_closest, traj_resampled->end());

  std::vector<TrajectoryPoints> debug_trajectories;
  if (!smoother_->apply(initial_vel, initial_acc, clipped, traj_smoothed, debug_trajectories)) {
    RCLCPP_WARN(get_logger(), "Fail to solve optimization.");
  }

  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled->begin(),
    traj_resampled->begin() + *traj_resampled_closest);

  // Set 0 velocity after input-stop-point
  overwriteStopPoint(*traj_resampled, traj_smoothed);

  // For the endpoint of the trajectory
  if (!traj_smoothed.empty()) {
    traj_smoothed.back().longitudinal_velocity_mps = 0.0;
  }

  // Max velocity filter for safety
  trajectory_utils::applyMaximumVelocityLimit(
    *traj_resampled_closest, traj_smoothed.size(), node_param_.max_velocity, traj_smoothed);

  // Insert behind velocity for output's consistency
  insertBehindVelocity(*traj_resampled_closest, type, traj_smoothed);

  RCLCPP_DEBUG(get_logger(), "smoothVelocity : traj_smoothed.size() = %lu", traj_smoothed.size());
  if (publish_debug_trajs_) {
    pub_trajectory_latacc_filtered_->publish(
      toTrajectoryMsg(*traj_lateral_acc_filtered, base_traj_raw_ptr_->header));
    pub_trajectory_resampled_->publish(
      toTrajectoryMsg(*traj_resampled, base_traj_raw_ptr_->header));

    if (!debug_trajectories.empty()) {
      for (auto & debug_trajectory : debug_trajectories) {
        debug_trajectory.insert(
          debug_trajectory.begin(), traj_resampled->begin(),
          traj_resampled->begin() + *traj_resampled_closest);
        for (size_t i = 0; i < *traj_resampled_closest; ++i) {
          debug_trajectory.at(i).longitudinal_velocity_mps =
            debug_trajectory.at(*traj_resampled_closest).longitudinal_velocity_mps;
        }
      }
    }
    publishDebugTrajectories(debug_trajectories);
  }

  return true;
}

void MotionVelocitySmootherNode::insertBehindVelocity(
  const size_t output_closest, const InitializeType type, TrajectoryPoints & output) const
{
  const bool keep_closest_vel_for_behind =
    (type == InitializeType::INIT || type == InitializeType::LARGE_DEVIATION_REPLAN ||
     type == InitializeType::ENGAGING);

  for (size_t i = output_closest - 1; i < output.size(); --i) {
    if (keep_closest_vel_for_behind) {
      output.at(i).longitudinal_velocity_mps = output.at(output_closest).longitudinal_velocity_mps;
      output.at(i).acceleration_mps2 = output.at(output_closest).acceleration_mps2;
    } else {
      const auto prev_output_point =
        trajectory_utils::calcInterpolatedTrajectoryPoint(prev_output_, output.at(i).pose);

      // output should be always positive: TODO(Horibe) think better way
      output.at(i).longitudinal_velocity_mps =
        std::abs(prev_output_point.longitudinal_velocity_mps);
      output.at(i).acceleration_mps2 = prev_output_point.acceleration_mps2;
    }
  }
}

void MotionVelocitySmootherNode::publishStopDistance(
  const TrajectoryPoints & trajectory, const size_t closest) const
{
  // stop distance calculation
  const double stop_dist_lim{50.0};
  double stop_dist{stop_dist_lim};
  const auto stop_idx{autoware_utils::searchZeroVelocityIndex(trajectory)};
  if (stop_idx) {
    stop_dist = trajectory_utils::calcArcLength(trajectory, closest, *stop_idx);
    stop_dist = closest > *stop_idx ? stop_dist : -stop_dist;
  } else {
    stop_dist = closest > 0 ? stop_dist : -stop_dist;
  }
  Float32Stamped dist_to_stopline{};
  dist_to_stopline.stamp = this->now();
  dist_to_stopline.data = std::max(-stop_dist_lim, std::min(stop_dist_lim, stop_dist));
  pub_dist_to_stopline_->publish(dist_to_stopline);
}

std::tuple<double, double, MotionVelocitySmootherNode::InitializeType>
MotionVelocitySmootherNode::calcInitialMotion(
  const TrajectoryPoints & input_traj, const size_t input_closest,
  const TrajectoryPoints & prev_traj) const
{
  const double vehicle_speed{std::fabs(current_odometry_ptr_->twist.twist.linear.x)};
  const double target_vel{std::fabs(input_traj.at(input_closest).longitudinal_velocity_mps)};

  double initial_vel{};
  double initial_acc{};
  InitializeType type{};

  // first time
  if (prev_traj.empty()) {
    initial_vel = vehicle_speed;
    initial_acc = 0.0;
    type = InitializeType::INIT;
    return std::make_tuple(initial_vel, initial_acc, type);
  }

  const auto prev_output_closest_point =
    trajectory_utils::calcInterpolatedTrajectoryPoint(prev_traj, input_traj.at(input_closest).pose);

  // when velocity tracking deviation is large
  const double desired_vel{prev_output_closest_point.longitudinal_velocity_mps};
  const double vel_error{vehicle_speed - std::fabs(desired_vel)};
  if (std::fabs(vel_error) > node_param_.replan_vel_deviation) {
    type = InitializeType::LARGE_DEVIATION_REPLAN;
    initial_vel = vehicle_speed;  // use current vehicle speed
    initial_acc = prev_output_closest_point.acceleration_mps2;
    RCLCPP_DEBUG(
      get_logger(),
      "calcInitialMotion : Large deviation error for speed control. Use current speed for "
      "initial value, desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
      desired_vel, vehicle_speed, vel_error, node_param_.replan_vel_deviation);
    return std::make_tuple(initial_vel, initial_acc, type);
  }

  // if current vehicle velocity is low && base_desired speed is high,
  // use engage_velocity for engage vehicle
  const double engage_vel_thr = node_param_.engage_velocity * node_param_.engage_exit_ratio;
  if (vehicle_speed < engage_vel_thr) {
    if (target_vel >= node_param_.engage_velocity) {
      const auto idx = autoware_utils::searchZeroVelocityIndex(input_traj);
      const double stop_dist =
        idx ? autoware_utils::calcDistance2d(input_traj.at(*idx), input_traj.at(input_closest))
            : 0.0;
      if (!idx || stop_dist > node_param_.stop_dist_to_prohibit_engage) {
        type = InitializeType::ENGAGING;
        initial_vel = node_param_.engage_velocity;
        initial_acc = node_param_.engage_acceleration;
        RCLCPP_DEBUG(
          get_logger(),
          "calcInitialMotion : vehicle speed is low (%.3f), and desired speed is high (%.3f). Use "
          "engage speed (%.3f) until vehicle speed reaches engage_vel_thr (%.3f). stop_dist = %.3f",
          vehicle_speed, target_vel, node_param_.engage_velocity, engage_vel_thr, stop_dist);
        return std::make_tuple(initial_vel, initial_acc, type);
      } else {
        RCLCPP_DEBUG(
          get_logger(), "calcInitialMotion : stop point is close (%.3f[m]). no engage.", stop_dist);
      }
    } else if (target_vel > 0.0) {
      auto clock{rclcpp::Clock{RCL_ROS_TIME}};
      RCLCPP_WARN_THROTTLE(
        get_logger(), clock, 3000,
        "calcInitialMotion : target velocity(%.3f[m/s]) is lower than engage velocity(%.3f[m/s]). ",
        target_vel, node_param_.engage_velocity);
    }
  }

  // normal update: use closest in prev_output
  type = InitializeType::NORMAL;
  initial_vel = prev_output_closest_point.longitudinal_velocity_mps;
  initial_acc = prev_output_closest_point.acceleration_mps2;
  RCLCPP_DEBUG(
    get_logger(),
    "calcInitialMotion : normal update. v0 = %f, a0 = %f, vehicle_speed = %f, target_vel = %f",
    initial_vel, initial_acc, vehicle_speed, target_vel);
  return std::make_tuple(initial_vel, initial_acc, type);
}

void MotionVelocitySmootherNode::overwriteStopPoint(
  const TrajectoryPoints & input, TrajectoryPoints & output) const
{
  const auto stop_idx = autoware_utils::searchZeroVelocityIndex(input);
  if (!stop_idx) {
    return;
  }

  // Get Closest Point from Output
  const auto nearest_output_point_idx = autoware_utils::findNearestIndex(
    output, input.at(*stop_idx).pose, std::numeric_limits<double>::max(),
    node_param_.delta_yaw_threshold);

  // check over velocity
  bool is_stop_velocity_exceeded{false};
  double input_stop_vel{};
  double output_stop_vel{};
  if (nearest_output_point_idx) {
    double optimized_stop_point_vel =
      output.at(*nearest_output_point_idx).longitudinal_velocity_mps;
    is_stop_velocity_exceeded = (optimized_stop_point_vel > over_stop_velocity_warn_thr_);
    input_stop_vel = input.at(*nearest_output_point_idx).longitudinal_velocity_mps;
    output_stop_vel = output.at(*nearest_output_point_idx).longitudinal_velocity_mps;
    trajectory_utils::applyMaximumVelocityLimit(
      *nearest_output_point_idx, output.size(), 0.0, output);
    RCLCPP_DEBUG(
      get_logger(),
      "replan : input_stop_idx = %lu, stop velocity : input = %f, output = %f, thr = %f",
      *nearest_output_point_idx, input_stop_vel, output_stop_vel, over_stop_velocity_warn_thr_);
  } else {
    input_stop_vel = -1.0;
    output_stop_vel = -1.0;
    RCLCPP_DEBUG(
      get_logger(),
      "replan : input_stop_idx = -1, stop velocity : input = %f, output = %f, thr = %f",
      input_stop_vel, output_stop_vel, over_stop_velocity_warn_thr_);
  }

  {
    StopSpeedExceeded msg{};
    msg.stamp = this->now();
    msg.stop_speed_exceeded = is_stop_velocity_exceeded;
    pub_over_stop_velocity_->publish(msg);
  }
}

void MotionVelocitySmootherNode::applyExternalVelocityLimit(TrajectoryPoints & traj) const
{
  if (traj.size() < 1) {
    return;
  }

  trajectory_utils::applyMaximumVelocityLimit(
    0, traj.size(), max_velocity_with_deceleration_, traj);

  const auto closest_idx = autoware_utils::findNearestIndex(
    traj, current_pose_ptr_->pose, std::numeric_limits<double>::max(),
    node_param_.delta_yaw_threshold);
  if (!closest_idx) {
    return;
  }

  double dist = 0.0;
  for (size_t idx = *closest_idx; idx < traj.size() - 1; ++idx) {
    dist += autoware_utils::calcDistance2d(traj.at(idx), traj.at(idx + 1));
    if (dist > external_velocity_limit_dist_) {
      trajectory_utils::applyMaximumVelocityLimit(
        idx + 1, traj.size(), external_velocity_limit_, traj);
      return;
    }
  }
  traj.back().longitudinal_velocity_mps =
    std::min(traj.back().longitudinal_velocity_mps, static_cast<float>(external_velocity_limit_));
  RCLCPP_DEBUG(get_logger(), "externalVelocityLimit : limit_vel = %.3f", external_velocity_limit_);
}

void MotionVelocitySmootherNode::applyStopApproachingVelocity(TrajectoryPoints & traj) const
{
  const auto stop_idx = autoware_utils::searchZeroVelocityIndex(traj);
  if (!stop_idx) {
    return;  // no stop point.
  }
  double distance_sum = 0.0;
  for (size_t i = *stop_idx - 1; i < traj.size(); --i) {  // search backward
    distance_sum += autoware_utils::calcDistance2d(traj.at(i), traj.at(i + 1));
    if (distance_sum > node_param_.stopping_distance) {
      break;
    }
    if (traj.at(i).longitudinal_velocity_mps > node_param_.stopping_velocity) {
      traj.at(i).longitudinal_velocity_mps = node_param_.stopping_velocity;
    }
  }
}

void MotionVelocitySmootherNode::publishDebugTrajectories(
  const std::vector<TrajectoryPoints> & debug_trajectories) const
{
  if (node_param_.algorithm_type == AlgorithmType::JERK_FILTERED) {
    if (debug_trajectories.size() != 3) {
      RCLCPP_DEBUG(get_logger(), "Size of the debug trajectories is incorrect");
      return;
    }
    const auto & h = base_traj_raw_ptr_->header;
    pub_forward_filtered_trajectory_->publish(toTrajectoryMsg(debug_trajectories.at(0), h));
    pub_backward_filtered_trajectory_->publish(toTrajectoryMsg(debug_trajectories.at(1), h));
    pub_merged_filtered_trajectory_->publish(toTrajectoryMsg(debug_trajectories.at(2), h));
    publishClosestVelocity(
      debug_trajectories.at(2), current_pose_ptr_->pose, pub_closest_merged_velocity_);
  }
}

void MotionVelocitySmootherNode::publishClosestVelocity(
  const TrajectoryPoints & trajectory, const Pose & current_pose,
  const rclcpp::Publisher<Float32Stamped>::SharedPtr pub) const
{
  const auto closest_point =
    trajectory_utils::calcInterpolatedTrajectoryPoint(trajectory, current_pose);

  Float32Stamped vel_data{};
  vel_data.stamp = this->now();
  vel_data.data = std::max(closest_point.longitudinal_velocity_mps, static_cast<float>(0.0));
  pub->publish(vel_data);
}

void MotionVelocitySmootherNode::publishClosestState(const TrajectoryPoint & closest_point)
{
  auto publishFloat = [=](
                        const double data, const rclcpp::Publisher<Float32Stamped>::SharedPtr pub) {
    Float32Stamped msg{};
    msg.stamp = this->now();
    msg.data = data;
    pub->publish(msg);
    return;
  };

  const double curr_vel{closest_point.longitudinal_velocity_mps};
  const double curr_acc{closest_point.acceleration_mps2};
  if (!prev_time_) {
    prev_time_ = std::make_shared<rclcpp::Time>(this->now());
    prev_acc_ = curr_acc;
    return;
  }

  // Calculate jerk
  rclcpp::Time curr_time{this->now()};
  double dt = (curr_time - *prev_time_).seconds();
  double curr_jerk = (curr_acc - prev_acc_) / dt;

  // Publish data
  publishFloat(curr_vel, debug_closest_velocity_);
  publishFloat(curr_acc, debug_closest_acc_);
  publishFloat(curr_jerk, debug_closest_jerk_);

  // Update
  prev_acc_ = curr_acc;
  *prev_time_ = curr_time;
}

MotionVelocitySmootherNode::AlgorithmType MotionVelocitySmootherNode::getAlgorithmType(
  const std::string & algorithm_name) const
{
  if (algorithm_name == "JerkFiltered") {
    return AlgorithmType::JERK_FILTERED;
  }
  if (algorithm_name == "L2") {
    return AlgorithmType::L2;
  }
  if (algorithm_name == "Linf") {
    return AlgorithmType::LINF;
  }
  if (algorithm_name == "Analytical") {
    return AlgorithmType::ANALYTICAL;
  }

  throw std::domain_error("[MotionVelocitySmootherNode] undesired algorithm is selected.");
  return AlgorithmType::INVALID;
}

double MotionVelocitySmootherNode::calcTravelDistance() const
{
  const auto closest_point =
    trajectory_utils::calcInterpolatedTrajectoryPoint(prev_output_, current_pose_ptr_->pose);

  if (prev_closest_point_) {
    const double travel_dist = autoware_utils::calcDistance2d(*prev_closest_point_, closest_point);
    return travel_dist;
  }

  return 0.0;
}

bool MotionVelocitySmootherNode::isEngageStatus(const double target_vel) const
{
  const double vehicle_speed = std::fabs(current_odometry_ptr_->twist.twist.linear.x);
  const double engage_vel_thr = node_param_.engage_velocity * node_param_.engage_exit_ratio;
  return vehicle_speed < engage_vel_thr && target_vel >= node_param_.engage_velocity;
}

Trajectory MotionVelocitySmootherNode::toTrajectoryMsg(
  const TrajectoryPoints & points, const std_msgs::msg::Header & header) const
{
  auto trajectory = autoware_utils::convertToTrajectory(points);
  trajectory.header = header;
  return trajectory;
}

}  // namespace motion_velocity_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motion_velocity_smoother::MotionVelocitySmootherNode)
