// Copyright 2021 The Autoware Foundation
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

#include "gtest/gtest.h"
#include "mpc_lateral_controller/mpc.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_unconstraint_fast.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

using autoware_auto_control_msgs::msg::AckermannLateralCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;

class MPCTest : public ::testing::Test
{
protected:
  MPCParam param;
  // Test inputs
  Trajectory dummy_straight_trajectory;
  Trajectory dummy_right_turn_trajectory;
  SteeringReport neutral_steer;
  Pose pose_zero;
  double default_velocity = 1.0;
  rclcpp::Logger logger = rclcpp::get_logger("mpc_test_logger");

  // Vehicle model parameters
  double wheelbase = 2.7;
  double steer_limit = 1.0;
  double steer_tau = 0.1;
  double mass_fl = 600.0;
  double mass_fr = 600.0;
  double mass_rl = 600.0;
  double mass_rr = 600.0;
  double cf = 155494.663;
  double cr = 155494.663;

  // Filters parameter
  double steering_lpf_cutoff_hz = 3.0;
  double error_deriv_lpf_cutoff_hz = 5.0;

  // Test Parameters
  double admissible_position_error = 5.0;
  double admissible_yaw_error_rad = M_PI_2;
  double steer_lim = 0.610865;      // 35 degrees
  double steer_rate_lim = 2.61799;  // 150 degrees
  double ctrl_period = 0.03;

  bool use_steer_prediction = true;

  TrajectoryFilteringParam trajectory_param;

  TrajectoryPoint makePoint(const double x, const double y, const float vx)
  {
    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.longitudinal_velocity_mps = vx;
    return p;
  }

  void SetUp() override
  {
    param.prediction_horizon = 50;
    param.prediction_dt = 0.1;
    param.zero_ff_steer_deg = 0.5;
    param.input_delay = 0.0;
    param.acceleration_limit = 2.0;
    param.velocity_time_constant = 0.3;
    param.min_prediction_length = 5.0;
    param.steer_tau = 0.1;
    param.nominal_weight.lat_error = 1.0;
    param.nominal_weight.heading_error = 1.0;
    param.nominal_weight.heading_error_squared_vel = 1.0;
    param.nominal_weight.terminal_lat_error = 1.0;
    param.nominal_weight.terminal_heading_error = 0.1;
    param.low_curvature_weight.lat_error = 0.1;
    param.low_curvature_weight.heading_error = 0.0;
    param.low_curvature_weight.heading_error_squared_vel = 0.3;
    param.nominal_weight.steering_input = 1.0;
    param.nominal_weight.steering_input_squared_vel = 0.25;
    param.nominal_weight.lat_jerk = 0.0;
    param.nominal_weight.steer_rate = 0.0;
    param.nominal_weight.steer_acc = 0.000001;
    param.low_curvature_weight.steering_input = 1.0;
    param.low_curvature_weight.steering_input_squared_vel = 0.25;
    param.low_curvature_weight.lat_jerk = 0.0;
    param.low_curvature_weight.steer_rate = 0.0;
    param.low_curvature_weight.steer_acc = 0.000001;
    param.low_curvature_thresh_curvature = 0.0;

    trajectory_param.traj_resample_dist = 0.1;
    trajectory_param.path_filter_moving_ave_num = 35;
    trajectory_param.curvature_smoothing_num_traj = 1;
    trajectory_param.curvature_smoothing_num_ref_steer = 35;
    trajectory_param.enable_path_smoothing = true;
    trajectory_param.extend_trajectory_for_end_yaw_control = true;

    dummy_straight_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(1.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(2.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(3.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(4.0, 0.0, 1.0f));

    dummy_right_turn_trajectory.points.push_back(makePoint(-1.0, -1.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(1.0, -1.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(2.0, -2.0, 1.0f));

    neutral_steer.steering_tire_angle = 0.0;
    pose_zero.position.x = 0.0;
    pose_zero.position.y = 0.0;
  }

  void initializeMPC(mpc_lateral_controller::MPC & mpc)
  {
    mpc.m_param = param;
    mpc.m_admissible_position_error = admissible_position_error;
    mpc.m_admissible_yaw_error_rad = admissible_yaw_error_rad;
    mpc.m_steer_lim = steer_lim;
    mpc.m_steer_rate_lim_map_by_curvature.emplace_back(0.0, steer_rate_lim);
    mpc.m_steer_rate_lim_map_by_velocity.emplace_back(0.0, steer_rate_lim);
    mpc.m_ctrl_period = ctrl_period;
    mpc.m_use_steer_prediction = use_steer_prediction;

    mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
    mpc.initializeSteeringPredictor();

    // Init trajectory
    const auto current_kinematics =
      makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
    mpc.setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);
  }

  nav_msgs::msg::Odometry makeOdometry(const geometry_msgs::msg::Pose & pose, const double velocity)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.pose.pose = pose;
    odometry.twist.twist.linear.x = velocity;
    return odometry;
  }
};  // class MPCTest

/* cppcheck-suppress syntaxError */
TEST_F(MPCTest, InitializeAndCalculate)
{
  MPC mpc;
  EXPECT_FALSE(mpc.hasVehicleModel());
  EXPECT_FALSE(mpc.hasQPSolver());

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(mpc);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, InitializeAndCalculateRightTurn)
{
  MPC mpc;
  EXPECT_FALSE(mpc.hasVehicleModel());
  EXPECT_FALSE(mpc.hasQPSolver());

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc.setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, OsqpCalculate)
{
  MPC mpc;
  initializeMPC(mpc);
  const auto current_kinematics = makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
  mpc.setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverOSQP>(logger);
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  EXPECT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, OsqpCalculateRightTurn)
{
  MPC mpc;
  initializeMPC(mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc.setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverOSQP>(logger);
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, KinematicsNoDelayCalculate)
{
  MPC mpc;
  initializeMPC(mpc);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase, steer_limit);
  mpc.setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Init filters
  mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
  // Init trajectory
  const auto current_kinematics = makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
  mpc.setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);
  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, KinematicsNoDelayCalculateRightTurn)
{
  MPC mpc;
  initializeMPC(mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc.setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase, steer_limit);
  mpc.setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Init filters
  mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, DynamicCalculate)
{
  MPC mpc;
  initializeMPC(mpc);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<DynamicsBicycleModel>(wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
  mpc.setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc.hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc.hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, MultiSolveWithBuffer)
{
  MPC mpc;
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr);
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);

  // Init parameters and reference trajectory
  initializeMPC(mpc);

  mpc.m_input_buffer = {0.0, 0.0, 0.0};
  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);

  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc.m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc.m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc.m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc.m_input_buffer.size(), size_t(3));
}

TEST_F(MPCTest, FailureCases)
{
  MPC mpc;
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc.setVehicleModel(vehicle_model_ptr);
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc.setQPSolver(qpsolver_ptr);

  // Init parameters and reference trajectory
  initializeMPC(mpc);

  // Calculate MPC with a pose too far from the trajectory
  Pose pose_far;
  pose_far.position.x = pose_zero.position.x - admissible_position_error - 1.0;
  pose_far.position.y = pose_zero.position.y - admissible_position_error - 1.0;
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_far, default_velocity);
  EXPECT_FALSE(mpc.calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));

  // Calculate MPC with a fast velocity to make the prediction go further than the reference path
  EXPECT_FALSE(mpc.calculateMPC(
    neutral_steer, makeOdometry(pose_far, default_velocity + 10.0), ctrl_cmd, pred_traj, diag));
}
}  // namespace autoware::motion::control::mpc_lateral_controller
