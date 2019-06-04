/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mpc_follower/mpc_follower_core.h"

#define DEBUG_INFO(...) { if (show_debug_info_) { ROS_INFO(__VA_ARGS__); }}

MPCFollower::MPCFollower()
    : nh_(""), pnh_("~"), my_position_ok_(false), my_velocity_ok_(false), my_steering_ok_(false)
{
  pnh_.param("show_debug_info", show_debug_info_, bool(false));
  pnh_.param("publish_debug_values", publish_debug_values_, bool(true));
  pnh_.param("ctrl_period", ctrl_period_, double(0.03));
  pnh_.param("enable_path_smoothing", enable_path_smoothing_, bool(true));
  pnh_.param("enable_yaw_recalculation", enable_yaw_recalculation_, bool(false));
  pnh_.param("path_filter_moving_ave_num", path_filter_moving_ave_num_, int(35));
  pnh_.param("path_smoothing_times", path_smoothing_times_, int(1));
  pnh_.param("curvature_smoothing_num", curvature_smoothing_num_, int(35));
  pnh_.param("traj_resample_dist", traj_resample_dist_, double(0.1)); // [m]
  pnh_.param("admisible_position_error", admisible_position_error_, double(5.0));
  pnh_.param("admisible_yaw_error_deg", admisible_yaw_error_deg_, double(90.0));
  pnh_.param("output_interface", output_interface_, std::string("all"));

  /* mpc parameters */
  pnh_.param("mpc_prediction_horizon", mpc_param_.prediction_horizon, int(70));
  pnh_.param("mpc_prediction_sampling_time", mpc_param_.prediction_sampling_time, double(0.1));
  pnh_.param("mpc_weight_lat_error", mpc_param_.weight_lat_error, double(1.0));
  pnh_.param("mpc_weight_heading_error", mpc_param_.weight_heading_error, double(0.0));
  pnh_.param("mpc_weight_heading_error_squared_vel_coeff", mpc_param_.weight_heading_error_squared_vel_coeff, double(0.3));
  pnh_.param("mpc_weight_steering_input", mpc_param_.weight_steering_input, double(1.0));
  pnh_.param("mpc_weight_steering_input_squared_vel_coeff", mpc_param_.weight_steering_input_squared_vel_coeff, double(0.25));
  pnh_.param("mpc_weight_lat_jerk", mpc_param_.weight_lat_jerk, double(0.0));
  pnh_.param("mpc_weight_terminal_lat_error", mpc_param_.weight_terminal_lat_error, double(1.0));
  pnh_.param("mpc_weight_terminal_heading_error", mpc_param_.weight_terminal_heading_error, double(0.1));
  pnh_.param("mpc_zero_ff_steer_deg", mpc_param_.zero_ff_steer_deg, double(2.0));

  pnh_.param("steer_lim_deg", steer_lim_deg_, double(35.0));
  pnh_.param("steering_gear_ratio", steering_gear_ratio_, double(19.0));
  pnh_.param("vehicle_model_wheelbase", wheelbase_, double(2.9));

  /* vehicle model setup */
  pnh_.param("vehicle_model_type", vehicle_model_type_, std::string("kinematics"));
  if (vehicle_model_type_ == "kinematics")
  {
    double steer_tau;
    pnh_.param("vehicle_model_steer_tau", steer_tau, double(0.1));

    vehicle_model_ptr_ = std::make_shared<KinematicsBicycleModel>(wheelbase_, amathutils::deg2rad(steer_lim_deg_), steer_tau);
    ROS_INFO("[MPC] set vehicle_model = kinematics");
  }
  else if (vehicle_model_type_ == "kinematics_no_delay")
  {
    vehicle_model_ptr_ = std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase_, amathutils::deg2rad(steer_lim_deg_));
    ROS_INFO("[MPC] set vehicle_model = kinematics_no_delay");
  }
  else if (vehicle_model_type_ == "dynamics")
  {
    double mass_fl, mass_fr, mass_rl, mass_rr, cf, cr;
    pnh_.param("mass_fl", mass_fl, double(600));
    pnh_.param("mass_fr", mass_fr, double(600));
    pnh_.param("mass_rl", mass_rl, double(600));
    pnh_.param("mass_rr", mass_rr, double(600));
    pnh_.param("cf", cf, double(155494.663));
    pnh_.param("cr", cr, double(155494.663));

    vehicle_model_ptr_ = std::make_shared<DynamicsBicycleModel>(wheelbase_, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
    ROS_INFO("[MPC] set vehicle_model = dynamics");
  }
  else
  {
    ROS_ERROR("[MPC] vehicle_model_type is undefined");
  }

  /* QP solver setup */
  std::string qp_solver_type_;
  pnh_.param("qp_solver_type", qp_solver_type_, std::string("unconstraint_fast"));
  if (qp_solver_type_ == "unconstraint")
  {
    qpsolver_ptr_ = std::make_shared<QPSolverEigenLeastSquare>();
    ROS_INFO("[MPC] set qp solver = unconstraint");
  }
  else if (qp_solver_type_ == "unconstraint_fast")
  {
    qpsolver_ptr_ = std::make_shared<QPSolverEigenLeastSquareLLT>();
    ROS_INFO("[MPC] set qp solver = unconstraint_fast");
  }
  // else if (qp_solver_type_ == "qpoases_hotstart")
  // {
  //   int max_iter = 200;
  //   qpsolver_ptr_ = std::make_shared<QPSolverQpoasesHotstart>(max_iter);
  //   ROS_INFO("[MPC] set qp solver = qpoases_hotstart");
  // }
  else
  {
    ROS_ERROR("[MPC] qp_solver_type is undefined");
  }

  steer_cmd_prev_ = 0.0;
  lateral_error_prev_ = 0.0;
  yaw_error_prev_ = 0.0;

  /* initialize lowpass filter */
  double steering_lpf_cutoff_hz, error_deriv_lpf_curoff_hz;
  pnh_.param("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz, double(3.0));
  pnh_.param("error_deriv_lpf_curoff_hz", error_deriv_lpf_curoff_hz, double(5.0));
  lpf_steering_cmd_.initialize(ctrl_period_, steering_lpf_cutoff_hz);
  lpf_lateral_error_.initialize(ctrl_period_, error_deriv_lpf_curoff_hz);
  lpf_yaw_error_.initialize(ctrl_period_, error_deriv_lpf_curoff_hz);

  /* set up ros system */
  timer_control_ = nh_.createTimer(ros::Duration(ctrl_period_), &MPCFollower::timerCallback, this);
  std::string out_twist, out_vehicle_cmd, in_vehicle_status, in_waypoints, in_selfpose;
  pnh_.param("out_twist_name", out_twist, std::string("/twist_raw"));
  pnh_.param("out_vehicle_cmd_name", out_vehicle_cmd, std::string("/ctrl_cmd"));
  pnh_.param("in_waypoints_name", in_waypoints, std::string("/base_waypoints"));
  pnh_.param("in_selfpose_name", in_selfpose, std::string("/current_pose"));
  pnh_.param("in_vehicle_status_name", in_vehicle_status, std::string("/vehicle_status"));
  pub_twist_cmd_ = nh_.advertise<geometry_msgs::TwistStamped>(out_twist, 1);
  pub_steer_vel_ctrl_cmd_ = nh_.advertise<autoware_msgs::ControlCommandStamped>(out_vehicle_cmd, 1);
  sub_ref_path_ = nh_.subscribe(in_waypoints, 1, &MPCFollower::callbackRefPath, this);
  sub_pose_ = nh_.subscribe(in_selfpose, 1, &MPCFollower::callbackPose, this);
  sub_vehicle_status_ = nh_.subscribe(in_vehicle_status, 1, &MPCFollower::callbackVehicleStatus, this);

  /* for debug */
  pub_debug_filtered_traj_ = pnh_.advertise<visualization_msgs::Marker>("debug/filtered_traj", 1);
  pub_debug_predicted_traj_ = pnh_.advertise<visualization_msgs::Marker>("debug/predicted_traj", 1);
  pub_debug_mpc_calc_time_ = pnh_.advertise<std_msgs::Float32>("debug/mpc_calc_time", 1);
  
  if (publish_debug_values_)
  {
    pub_debug_values_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug/debug_values", 1);
    sub_estimate_twist_ = nh_.subscribe("/estimate_twist", 1, &MPCFollower::callbackEstimateTwist, this);
    pub_debug_steer_cmd_ = pnh_.advertise<std_msgs::Float32>("debug/steer_cmd", 1);
    pub_debug_steer_cmd_ff_ = pnh_.advertise<std_msgs::Float32>("debug/steer_cmd_ff", 1);
    pub_debug_steer_cmd_raw_ = pnh_.advertise<std_msgs::Float32>("debug/steer_cmd_raw", 1);
    pub_debug_steer_ = pnh_.advertise<std_msgs::Float32>("debug/steer", 1);
    pub_debug_laterr_ = pnh_.advertise<std_msgs::Float32>("debug/laterr", 1);
    pub_debug_yawerr_ = pnh_.advertise<std_msgs::Float32>("debug/yawerr", 1);
    pub_debug_current_vel_ = pnh_.advertise<std_msgs::Float32>("debug/current_vel", 1);
    pub_debug_vel_cmd_ = pnh_.advertise<std_msgs::Float32>("debug/vel_cmd", 1);
    pub_debug_angvel_cmd_ = pnh_.advertise<std_msgs::Float32>("debug/angvel_cmd", 1);
    pub_debug_angvel_steer_ = pnh_.advertise<std_msgs::Float32>("debug/angvel_from_steer", 1);
    pub_debug_angvel_cmd_ff_ = pnh_.advertise<std_msgs::Float32>("debug/angvel_cmd_ff", 1);
    pub_debug_angvel_estimatetwist_ = pnh_.advertise<std_msgs::Float32>("debug/angvel_estimatetwist", 1);
  }
};

void MPCFollower::timerCallback(const ros::TimerEvent &te)
{

  /* guard */
  if (vehicle_model_ptr_ == nullptr || qpsolver_ptr_ == nullptr)
  {
    DEBUG_INFO("[MPC] vehicle_model = %d, qp_solver = %d", !(vehicle_model_ptr_ == nullptr), !(qpsolver_ptr_ == nullptr));
    publishControlCommands(0.0, 0.0, steer_cmd_prev_, 0.0); // publish brake
    return;
  }

  if (ref_traj_.size() == 0 || !my_position_ok_ || !my_velocity_ok_ || !my_steering_ok_)
  {
    DEBUG_INFO("[MPC] MPC is not solved. ref_traj_.size() = %d, my_position_ok_ = %d,  my_velocity_ok_ = %d,  my_steering_ok_ = %d",
               ref_traj_.size(), my_position_ok_, my_velocity_ok_, my_steering_ok_);
    publishControlCommands(0.0, 0.0, steer_cmd_prev_, 0.0); // publish brake
    return;
  }

  /* control command */
  double vel_cmd = 0.0;
  double acc_cmd = 0.0;
  double steer_cmd = 0.0;
  double steer_vel_cmd = 0.0;

  /* solve MPC */
  auto start = std::chrono::system_clock::now();
  const bool mpc_solved = calculateMPC(vel_cmd, acc_cmd, steer_cmd, steer_vel_cmd);
  double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() * 1.0e-6;
  DEBUG_INFO("[MPC] timerCallback: MPC calculating time = %f [ms]\n", elapsed_ms);

  /* publish computing time */
  std_msgs::Float32 mpc_calc_time_msg;
  mpc_calc_time_msg.data = elapsed_ms;
  pub_debug_mpc_calc_time_.publish(mpc_calc_time_msg);

  if (!mpc_solved)
  {
    ROS_WARN("[MPC] MPC is not solved. publish 0 velocity.");
    vel_cmd = 0.0;
    acc_cmd = 0.0;
    steer_cmd = steer_cmd_prev_;
    steer_vel_cmd = 0.0;
  }

  publishControlCommands(vel_cmd, acc_cmd, steer_cmd, steer_vel_cmd);
};

bool MPCFollower::calculateMPC(double &vel_cmd, double &acc_cmd, double &steer_cmd, double &steer_vel_cmd)
{
  const int N = mpc_param_.prediction_horizon;
  const double DT = mpc_param_.prediction_sampling_time;
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  const double current_yaw = tf2::getYaw(vehicle_status_.pose.orientation);

  /* calculate nearest point on reference trajectory (used as initial state) */
  unsigned int nearest_index = 0;
  double yaw_err, dist_err, nearest_traj_time;
  geometry_msgs::Pose nearest_pose;
  if (!MPCUtils::calcNearestPoseInterp(ref_traj_, vehicle_status_.pose, nearest_pose, nearest_index, dist_err, yaw_err, nearest_traj_time))
  {
    ROS_WARN("[MPC] calculateMPC: error in calculating nearest pose. stop mpc.");
    return false;
  };

  /* check if lateral error is not too large */
  if (dist_err > admisible_position_error_ || std::fabs(yaw_err) > amathutils::deg2rad(admisible_yaw_error_deg_ ))
  {
    ROS_WARN("[MPC] error is over limit, stop mpc. (pos: error = %f[m], limit: %f[m], yaw: error = %f[deg], limit %f[deg])",
             dist_err, admisible_position_error_, amathutils::rad2deg(yaw_err), admisible_yaw_error_deg_);
    return false;
  }

  /* set mpc initial time */
  const double mpc_start_time = nearest_traj_time;

  /* check trajectory length */
  const double mpc_end_time = mpc_start_time + (N - 1) * DT;
  if (mpc_end_time > ref_traj_.relative_time.back())
  {
    ROS_WARN("[MPC] path is too short for prediction. path end: %f[s], mpc end time: %f[s]", ref_traj_.relative_time.back(), mpc_end_time);
    return false;
  }

  /* convert tracking x,y error to lat error */
  const double err_x = vehicle_status_.pose.position.x - nearest_pose.position.x;
  const double err_y = vehicle_status_.pose.position.y - nearest_pose.position.y;
  const double sp_yaw = tf2::getYaw(nearest_pose.orientation);
  const double err_lat = -sin(sp_yaw) * err_x + cos(sp_yaw) * err_y;

  /* get steering angle */
  const double steer = vehicle_status_.tire_angle_rad;

  /* define initial state for error dynamics */
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(DIM_X);
  if (vehicle_model_type_ == "kinematics")
  {
    x0 << err_lat, yaw_err, steer;
  }
  else if (vehicle_model_type_ == "kinematics_no_delay")
  {
    x0 << err_lat, yaw_err;
  }
  else if (vehicle_model_type_ == "dynamics")
  {
    double dot_err_lat = (err_lat - lateral_error_prev_) / ctrl_period_;
    double dot_err_yaw = (yaw_err - yaw_error_prev_) / ctrl_period_;
    DEBUG_INFO("[MPC] (before lpf) dot_err_lat = %f, dot_err_yaw = %f", dot_err_lat, dot_err_yaw);
    lateral_error_prev_ = err_lat;
    yaw_error_prev_ = yaw_err;
    dot_err_lat = lpf_lateral_error_.filter(dot_err_lat);
    dot_err_yaw = lpf_yaw_error_.filter(dot_err_yaw);
    DEBUG_INFO("[MPC] (after lpf) dot_err_lat = %f, dot_err_yaw = %f", dot_err_lat, dot_err_yaw);
    x0 << err_lat, dot_err_lat, yaw_err, dot_err_yaw;
  }
  else
  {
    ROS_ERROR("vehicle_model_type is undefined");
    return false;
  }
  DEBUG_INFO("[MPC] selfpose.x = %f, y = %f, yaw = %f", vehicle_status_.pose.position.x, vehicle_status_.pose.position.y, current_yaw);
  DEBUG_INFO("[MPC] nearpose.x = %f, y = %f, yaw = %f", nearest_pose.position.x, nearest_pose.position.y, tf2::getYaw(nearest_pose.orientation));
  DEBUG_INFO("[MPC] nearest_index = %d, nearest_traj_time = %f", nearest_index, nearest_traj_time);
  DEBUG_INFO("[MPC] lat error = %f, yaw error = %f, steer = %f, sp_yaw = %f, my_yaw = %f", err_lat, yaw_err, steer, sp_yaw, current_yaw);

  /////////////// generate mpc matrix  ///////////////
  /*
   * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
   * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * Rex * (Uex - Urefex)
   * Qex = diag([Q,Q,...]), Rex = diag([R,R,...])
   */

  Eigen::MatrixXd Aex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_X);
  Eigen::MatrixXd Bex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_U * N);
  Eigen::MatrixXd Wex = Eigen::MatrixXd::Zero(DIM_X * N, 1);
  Eigen::MatrixXd Cex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  Eigen::MatrixXd Qex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  Eigen::MatrixXd Rex = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd Urefex = Eigen::MatrixXd::Zero(DIM_U * N, 1);

  /* weight matrix depends on the vehicle model */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Eigen::MatrixXd Q_adaptive = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R_adaptive = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Q(0, 0) = mpc_param_.weight_lat_error;
  Q(1, 1) = mpc_param_.weight_heading_error;
  R(0, 0) = mpc_param_.weight_steering_input;

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);
  Eigen::MatrixXd Uref(DIM_U, 1);

  /* resample ref_traj with mpc sampling time */
  std::vector<double> mpc_time_v;
  for (int i = 0; i < N; ++i)
  {
    mpc_time_v.push_back(mpc_start_time + i * DT);
  }
  MPCTrajectory mpc_resampled_ref_traj;
  if (!MPCUtils::interp1dMPCTraj(ref_traj_.relative_time, ref_traj_, mpc_time_v, mpc_resampled_ref_traj))
  {
    ROS_WARN("[MPC] calculateMPC: mpc resample error, stop mpc calculation. check code!");
    return false;
  }

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i)
  {
    const double ref_k = mpc_resampled_ref_traj.k[i];
    const double ref_vx = mpc_resampled_ref_traj.vx[i];
    const double ref_vx_squared = ref_vx * ref_vx;

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(ref_vx);
    vehicle_model_ptr_->setCurvature(ref_k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1)
    {
      Q_adaptive(0, 0) = mpc_param_.weight_terminal_lat_error;
      Q_adaptive(1, 1) = mpc_param_.weight_terminal_heading_error;
    }
    Q_adaptive(1, 1) += ref_vx_squared * mpc_param_.weight_heading_error_squared_vel_coeff;
    R_adaptive(0, 0) += ref_vx_squared * mpc_param_.weight_steering_input_squared_vel_coeff;

    /* update mpc matrix */
    int idx_x_i = i * DIM_X;
    int idx_x_i_prev = (i - 1) * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0)
    {
      Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      Wex.block(0, 0, DIM_X, 1) = Wd;
    }
    else
    {
      Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j)
      {
        int idx_u_j = j * DIM_U;
        Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) = Ad * Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    Rex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    /* get reference input (feed-forward) */
    vehicle_model_ptr_->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < amathutils::deg2rad(mpc_param_.zero_ff_steer_deg))
    {
      Uref(0, 0) = 0.0; // ignore curvature noise
    }

    Urefex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  /* add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2 */
  for (int i = 0; i < N - 1; ++i)
  {
    const double v = mpc_resampled_ref_traj.vx[i];
    const double lateral_jerk_weight = v * v * mpc_param_.weight_lat_jerk;
    Rex(i, i) += lateral_jerk_weight;
    Rex(i + 1, i) -= lateral_jerk_weight;
    Rex(i, i + 1) -= lateral_jerk_weight;
    Rex(i + 1, i + 1) += lateral_jerk_weight;
  }

  if (Aex.array().isNaN().any() || Bex.array().isNaN().any() ||
      Cex.array().isNaN().any() || Wex.array().isNaN().any())
  {
    ROS_WARN("[MPC] calculateMPC: model matrix includes NaN, stop MPC.");
    return false;
  }

  /////////////// optimization ///////////////
  /*
   * solve quadratic optimization.
   * cost function: 1/2 * Uex' * H * Uex + f' * Uex
   */
  const Eigen::MatrixXd CB = Cex * Bex;
  const Eigen::MatrixXd QCB = Qex * CB;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB; // NOTE: This calculation is very heavy. searching for a good way...
  H.triangularView<Eigen::Upper>() += Rex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  Eigen::MatrixXd f = (Cex * (Aex * x0 + Wex)).transpose() * QCB - Urefex.transpose() * Rex;

  /* constraint matrix : lb < U < ub, lbA < A*U < ubA */
  const double u_lim = amathutils::deg2rad(steer_lim_deg_);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd lbA = Eigen::MatrixXd::Zero(DIM_U * N, 1);
  Eigen::MatrixXd ubA = Eigen::MatrixXd::Zero(DIM_U * N, 1);
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(DIM_U * N, -u_lim); // min steering angle
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(DIM_U * N, u_lim);  // max steering angle

  auto start = std::chrono::system_clock::now();
  Eigen::VectorXd Uex;
  if (!qpsolver_ptr_->solve(H, f.transpose(), A, lb, ub, lbA, ubA, Uex))
  {
    ROS_WARN("[MPC] qp solver error");
    return false;
  }
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() * 1.0e-6;
  DEBUG_INFO("[MPC] calculateMPC: qp solver calculation time = %f [ms]", elapsed);

  if (Uex.array().isNaN().any())
  {
    ROS_WARN("[MPC] calculateMPC: model Uex includes NaN, stop MPC. ");
    return false;
  }

  /* saturation */
  const double u_sat = std::max(std::min(Uex(0), u_lim), -u_lim);

  /* filtering */
  const double u_filtered = lpf_steering_cmd_.filter(u_sat);

  /* set steering command */
  steer_cmd = u_filtered;
  steer_vel_cmd = (Uex(1) - Uex(0)) / DT;

  /* Velocity control: for simplicity, now we calculate steer and speed separately */
  vel_cmd = ref_traj_.vx[0];
  acc_cmd = (ref_traj_.vx[1] - ref_traj_.vx[0]) / DT;

  steer_cmd_prev_ = steer_cmd;

  DEBUG_INFO("[MPC] calculateMPC: mpc steer command raw = %f, filtered = %f, steer_vel_cmd = %f", Uex(0, 0), u_filtered, steer_vel_cmd);
  DEBUG_INFO("[MPC] calculateMPC: mpc vel command = %f, acc_cmd = %f", vel_cmd, acc_cmd);

  ////////////////// DEBUG ///////////////////

  /* calculate predicted trajectory */
  Eigen::VectorXd Xex = Aex * x0 + Bex * Uex + Wex;
  MPCTrajectory debug_mpc_predicted_traj;
  for (int i = 0; i < N; ++i)
  {
    const double lat_error = Xex(i * DIM_X);
    const double yaw_error = Xex(i * DIM_X + 1);
    const double x = mpc_resampled_ref_traj.x[i] - std::sin(mpc_resampled_ref_traj.yaw[i]) * lat_error;
    const double y = mpc_resampled_ref_traj.y[i] + std::cos(mpc_resampled_ref_traj.yaw[i]) * lat_error;
    const double z = mpc_resampled_ref_traj.z[i];
    debug_mpc_predicted_traj.push_back(x, y, z, mpc_resampled_ref_traj.yaw[i] + yaw_error, 0, 0, 0);
  }

  /* publish for visualization */
  visualization_msgs::Marker marker;
  convertTrajToMarker(debug_mpc_predicted_traj, marker, "predicted_traj", 0.99, 0.99, 0.99, 0.2);
  pub_debug_predicted_traj_.publish(marker);

  /* publish debug values */
  if (publish_debug_values_)
  {
    std_msgs::Float32 steer_cmd_msg; // final steering command (MPC + LPF)
    steer_cmd_msg.data = steer_cmd;
    pub_debug_steer_cmd_.publish(steer_cmd_msg);

    std_msgs::Float32 steer_cmd_raw_msg; // mpc calculation result
    steer_cmd_raw_msg.data = u_sat;
    pub_debug_steer_cmd_raw_.publish(steer_cmd_raw_msg);

    std_msgs::Float32 steer_cmd_ff_msg; // feedforward steering value
    steer_cmd_ff_msg.data = Urefex(0);
    pub_debug_steer_cmd_ff_.publish(steer_cmd_ff_msg);

    std_msgs::Float32 steer_act_msg; // current steering angle
    steer_act_msg.data = steer;
    pub_debug_steer_.publish(steer_act_msg);

    std_msgs::Float32 err_lat_msg; // lateral error
    err_lat_msg.data = err_lat;
    pub_debug_laterr_.publish(err_lat_msg);

    std_msgs::Float32 err_yaw_msg; // yaw error
    err_yaw_msg.data = yaw_err;
    pub_debug_yawerr_.publish(err_yaw_msg);

    std_msgs::Float32 current_vel_msg; //current velocity
    current_vel_msg.data = estimate_twist_.twist.angular.z;
    pub_debug_current_vel_.publish(current_vel_msg);

    std_msgs::Float32 vel_cmd_msg; // velocity command
    vel_cmd_msg.data = vel_cmd;
    pub_debug_vel_cmd_.publish(vel_cmd_msg);

    std_msgs::Float32 angvel_converted_from_steer_cmd_msg; // angular velocity calculated by steering command with kinematics model
    angvel_converted_from_steer_cmd_msg.data = vehicle_status_.twist.linear.x * tan(steer_cmd) / wheelbase_;
    pub_debug_angvel_cmd_.publish(angvel_converted_from_steer_cmd_msg);

    std_msgs::Float32 angvel_converted_from_steer_act_msg; // angular velocity calculated by current steering with kinematics model
    angvel_converted_from_steer_act_msg.data = vehicle_status_.twist.linear.x * tan(steer) / wheelbase_;
    pub_debug_angvel_steer_.publish(angvel_converted_from_steer_act_msg);

    std_msgs::Float32 angvel_ff_msg; // angular velocity calculated by steering feedforward with kinematics model
    const double nearest_curvature = mpc_resampled_ref_traj.k[0];
    angvel_ff_msg.data = nearest_curvature * vehicle_status_.twist.linear.x;
    pub_debug_angvel_cmd_ff_.publish(angvel_ff_msg);

    std_msgs::Float32 angvel_estimatetwist_msg; // estimate twist angular velocity
    angvel_estimatetwist_msg.data = estimate_twist_.twist.angular.z;
    pub_debug_angvel_estimatetwist_.publish(angvel_estimatetwist_msg);
  }

  return true;
};

void MPCFollower::callbackRefPath(const autoware_msgs::Lane::ConstPtr &msg)
{
  current_waypoints_ = *msg;
  DEBUG_INFO("[MPC] path callback: received path size = %lu", current_waypoints_.waypoints.size());

  MPCTrajectory traj;

  /* calculate relative time */
  std::vector<double> relative_time;
  MPCUtils::calcPathRelativeTime(current_waypoints_, relative_time);
  DEBUG_INFO("[MPC] path callback: relative_time.size() = %lu, front() = %f, back() = %f",
             relative_time.size(), relative_time.front(), relative_time.back());

  /* resampling */
  MPCUtils::convertWaypointsToMPCTrajWithDistanceResample(current_waypoints_, relative_time, traj_resample_dist_, traj);
  MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  DEBUG_INFO("[MPC] path callback: resampled traj size() = %lu", traj.relative_time.size());

  /* path smoothing */
  if (enable_path_smoothing_)
  {
    for (int i = 0; i < path_smoothing_times_; ++i)
    {
      if (!MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.x) ||
          !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.y) ||
          !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.yaw) ||
          !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.vx))
      {
        ROS_WARN("[MPC] path callback: filtering error. stop filtering");
        return;
      }
    }
  }

  /* calculate yaw angle */
  if (enable_yaw_recalculation_)
  {
    MPCUtils::calcTrajectoryYawFromXY(traj);
    MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  }

  /* calculate curvature */
  MPCUtils::calcTrajectoryCurvature(traj, curvature_smoothing_num_);
  const double max_k = *max_element(traj.k.begin(), traj.k.end());
  const double min_k = *min_element(traj.k.begin(), traj.k.end());
  DEBUG_INFO("[MPC] path callback: trajectory curvature : max_k = %f, min_k = %f", max_k, min_k);

  /* add end point with vel=0 on traj for mpc prediction */
  const double mpc_predict_time_length = (mpc_param_.prediction_horizon + 1) * mpc_param_.prediction_sampling_time;
  const double end_velocity = 0.0;
  traj.vx.back() = end_velocity; // also for end point
  traj.push_back(traj.x.back(), traj.y.back(), traj.z.back(), traj.yaw.back(),
                 end_velocity, traj.k.back(), traj.relative_time.back() + mpc_predict_time_length);

  if (!traj.size())
  {
    ROS_ERROR("[MPC] path callback: trajectory size is undesired.");
    DEBUG_INFO("size: x=%lu, y=%lu, z=%lu, yaw=%lu, v=%lu,k=%lu,t=%lu", traj.x.size(), traj.y.size(),
               traj.z.size(), traj.yaw.size(), traj.vx.size(), traj.k.size(), traj.relative_time.size());
    return;
  }

  ref_traj_ = traj;

  /* publish trajectory for visualize */
  visualization_msgs::Marker markers;
  convertTrajToMarker(ref_traj_, markers, "ref_traj", 0.0, 0.5, 1.0, 0.05);
  pub_debug_filtered_traj_.publish(markers);
};

void MPCFollower::convertTrajToMarker(const MPCTrajectory &traj, visualization_msgs::Marker &marker,
                                      std::string ns, double r, double g, double b, double z)
{
  marker.points.clear();
  marker.header.frame_id = current_waypoints_.header.frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.15;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 0.9;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  for (unsigned int i = 0; i < traj.x.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = traj.x.at(i);
    p.y = traj.y.at(i);
    p.z = traj.z.at(i) + z;
    marker.points.push_back(p);
  }
}

void MPCFollower::callbackPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  vehicle_status_.header = msg->header;
  vehicle_status_.pose = msg->pose;
  my_position_ok_ = true;
};

void MPCFollower::callbackVehicleStatus(const autoware_msgs::VehicleStatus &msg)
{
  vehicle_status_.tire_angle_rad = amathutils::deg2rad(msg.angle) / steering_gear_ratio_;
  vehicle_status_.twist.linear.x = amathutils::kmph2mps(msg.speed);
  my_steering_ok_ = true;
  my_velocity_ok_ = true;
};

void MPCFollower::publishControlCommands(const double &vel_cmd, const double &acc_cmd,
                                         const double &steer_cmd, const double &steer_vel_cmd)
{
  const double omega_cmd = vehicle_status_.twist.linear.x * std::tan(steer_cmd) / wheelbase_;
  if (output_interface_ == "twist")
  {
    publishTwist(vel_cmd, omega_cmd);
  }
  else if (output_interface_ == "ctrl_cmd")
  {
    publishCtrlCmd(vel_cmd, acc_cmd, steer_cmd);
  }
  else if (output_interface_ == "all")
  {
    publishTwist(vel_cmd, omega_cmd);
    publishCtrlCmd(vel_cmd, acc_cmd, steer_cmd);
  }
  else
  {
    ROS_WARN("[MPC] control command interface is not appropriate");
  }
}

void MPCFollower::publishTwist(const double &vel_cmd, const double &omega_cmd)
{
  /* convert steering to twist */
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "/base_link";
  twist.header.stamp = ros::Time::now();
  twist.twist.linear.x = vel_cmd;
  twist.twist.linear.y = 0.0;
  twist.twist.linear.z = 0.0;
  twist.twist.angular.x = 0.0;
  twist.twist.angular.y = 0.0;
  twist.twist.angular.z = omega_cmd;
  pub_twist_cmd_.publish(twist);
}

void MPCFollower::publishCtrlCmd(const double &vel_cmd, const double &acc_cmd, const double &steer_cmd)
{
  autoware_msgs::ControlCommandStamped cmd;
  cmd.header.frame_id = "/base_link";
  cmd.header.stamp = ros::Time::now();
  cmd.cmd.linear_velocity = vel_cmd;
  cmd.cmd.linear_acceleration = acc_cmd;
  cmd.cmd.steering_angle = steer_cmd;
  pub_steer_vel_ctrl_cmd_.publish(cmd);
}

MPCFollower::~MPCFollower()
{
  ROS_INFO("Publish 0 twist before I died.");
  double vel_cmd = 0.0;
  double acc_cmd = 0.0;
  double steer_cmd = 0.0;
  double steer_vel_cmd = 0.0;
  if (my_steering_ok_)
    steer_cmd = vehicle_status_.tire_angle_rad;
  publishControlCommands(vel_cmd, acc_cmd, steer_cmd, steer_vel_cmd);
};
