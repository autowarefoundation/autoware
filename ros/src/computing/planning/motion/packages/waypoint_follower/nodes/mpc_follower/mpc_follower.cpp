#include "mpc_follower/mpc_follower.h"

#define DEBUG_INFO(...) { if (show_debug_info_) { ROS_INFO(__VA_ARGS__); }}

MPCFollower::MPCFollower()
    : nh_(""), pnh_("~"), my_position_ok_(false), my_velocity_ok_(false), my_steering_ok_(false)
{
  pnh_.param("show_debug_info", show_debug_info_, bool(false));
  pnh_.param("ctrl_period", ctrl_period_, double(0.1));
  pnh_.param("use_path_smoothing", use_path_smoothing_, bool(true));
  pnh_.param("path_filter_moving_ave_num", path_filter_moving_ave_num_, int(5));
  pnh_.param("path_smoothing_times", path_smoothing_times_, int(1));
  pnh_.param("curvature_smoothing_num", curvature_smoothing_num_, int(10));
  pnh_.param("traj_resample_dist", traj_resample_dist_, double(0.1)); // [m]

  pnh_.param("admisible_position_error", admisible_position_error_, double(5.0));
  pnh_.param("admisible_yaw_error_deg", admisible_yaw_error_deg_, double(45.0));

  /* mpc parameters */
  pnh_.param("mpc_n", mpc_param_.n, int(50));
  pnh_.param("mpc_dt", mpc_param_.dt, double(0.1));
  pnh_.param("mpc_weight_lat_error", mpc_param_.weight_lat_error, double(1.0));
  pnh_.param("mpc_weight_heading_error", mpc_param_.weight_heading_error, double(1.0));
  pnh_.param("mpc_weight_steering_input", mpc_param_.weight_steering_input, double(1.0));
  pnh_.param("mpc_weight_steering_input_vel_coeff", mpc_param_.weight_steering_input_vel_coeff, double(0.0));
  pnh_.param("mpc_delay_compensation_time", mpc_param_.delay_compensation_time, double(0.05));
  pnh_.param("mpc_zero_curvature_range", mpc_param_.zero_curvature_range, double(0.03));
  
  pnh_.param("steer_lim_deg", steer_lim_deg_, double(35.0));
  pnh_.param("vehicle_model_wheelbase", wheelbase_, double(2.9));

  /* vehicle model initialize */
  double steer_tau;
  pnh_.param("vehicle_model_steer_tau", steer_tau, double(0.1));
  vehicle_model_.setParams(wheelbase_, steer_tau, steer_lim_deg_);

  /* set control command interface */
  std::string ctrl_cmd_interface_string;
  pnh_.param("ctrl_cmd_interface", ctrl_cmd_interface_string, std::string("all"));
  if (ctrl_cmd_interface_string == "twist")
    ctrl_cmd_interface_ = CtrlCmdInterface::TWIST;
  else if (ctrl_cmd_interface_string == "steer_and_vel")
    ctrl_cmd_interface_ = CtrlCmdInterface::STEER_AND_VEL;
  else if (ctrl_cmd_interface_string == "steer")
    ctrl_cmd_interface_ = CtrlCmdInterface::STEER;
  else if (ctrl_cmd_interface_string == "all")
    ctrl_cmd_interface_ = CtrlCmdInterface::ALL;
  else
  {
    ROS_ERROR("output interface is inappropriate");
  }

  /* initialize lowpass filter */
  double steering_lpf_cutoff_hz;
  pnh_.param("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz, double(3.0));
  lpf_steering_cmd_.initialize(ctrl_period_, steering_lpf_cutoff_hz);

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
  pub_debug_values_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug/debug_values", 1);
};

void MPCFollower::timerCallback(const ros::TimerEvent &te)
{

  /* check flags */
  if (ref_traj_.size() == 0 || !my_position_ok_ || !my_velocity_ok_ || !my_steering_ok_)
  {
    DEBUG_INFO("MPC is not solved. ref_traj_.size() = %d, my_position_ok_ = %d,  my_velocity_ok_ = %d,  my_steering_ok_ = %d",
               ref_traj_.size(), my_position_ok_, my_velocity_ok_, my_steering_ok_);
    return;
  }

  /* control command */
  double vel_cmd(0.0), steer_cmd(0.0);

  /* solve MPC */
  auto start = std::chrono::system_clock::now();
  const bool mpc_solved = calculateMPC(vel_cmd, steer_cmd);
  auto end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  DEBUG_INFO("[timerCallback] MPC calculating time = %f [ms]\n", elapsed * 1.0e-6);

  if (!mpc_solved)
  {
    ROS_WARN("MPC is not solved. publish 0 velocity.");
    vel_cmd = 0.0;
    steer_cmd = vehicle_status_.tire_angle_rad;
  }

  publishControlCommands(vel_cmd, steer_cmd);
};

bool MPCFollower::calculateMPC(double &vel_cmd, double &steer_cmd)
{
  /* parameter definition */
  static const double RAD2DEG = 180.0 / M_PI;
  static const double DEG2RAD = M_PI / 180.0;
  const int N = mpc_param_.n;
  const int DIM_X = vehicle_model_.getDimX();
  const int DIM_U = vehicle_model_.getDimU();
  const int DIM_Y = vehicle_model_.getDimY();

  geometry_msgs::Pose self_pose;
  geometry_msgs::Pose nearest_pose;
  double current_yaw = tf2::getYaw(vehicle_status_.pose.orientation);

  /* calculate nearest point on reference trajectory (used as initial state) */
  unsigned int nearest_index = 0;
  double nearest_yaw_error = std::numeric_limits<double>::max();
  double nearest_dist_error = std::numeric_limits<double>::max();
  double nearest_traj_time(0.0), nearest_ref_k(0.0);
  MPCUtils::calcNearestPoseInterp(ref_traj_, vehicle_status_.pose, nearest_pose, nearest_index, nearest_dist_error, nearest_yaw_error, nearest_traj_time);
  DEBUG_INFO("[calculateMPC] selfpose.x = %f, y = %f, yaw = %f", vehicle_status_.pose.position.x, vehicle_status_.pose.position.y, current_yaw);
  DEBUG_INFO("[calculateMPC] nearpose.x = %f, y = %f, yaw = %f", nearest_pose.position.x, nearest_pose.position.y, tf2::getYaw(nearest_pose.orientation));
  DEBUG_INFO("[calculateMPC] nearest_index = %d, nearest_dist_error = %f", nearest_index, nearest_dist_error);

  /* check if lateral error is not too large */
  if (nearest_dist_error > admisible_position_error_ || std::fabs(nearest_yaw_error) > admisible_yaw_error_deg_ * DEG2RAD)
    {
      ROS_WARN("[calculateMPC] error is over limit, stop mpc. (pos-error: %f[m], pos-limit: %f[m], yaw-error: %f[deg], yaw-limit: %f[deg])",
               nearest_dist_error, admisible_position_error_, nearest_yaw_error * RAD2DEG, admisible_yaw_error_deg_);
      return false;
    }

  /* set mpc initial time */
  double mpc_start_time = nearest_traj_time; /* as initialize */
  DEBUG_INFO("[calculateMPC] nearest_traj_time = %f", nearest_traj_time);

  /* check trajectory length */
  const double mpc_end_time = mpc_start_time + (N - 1) * mpc_param_.dt;
  if (mpc_end_time > ref_traj_.relative_time.back())
  {
    ROS_WARN("[calculateMPC] path is too short to predict dynamics. ");
    ROS_WARN("[calculateMPC] path end time: %f, mpc end time: %f", ref_traj_.relative_time.back(), mpc_end_time);
    return false;
  }

  /* convert tracking x,y error to lat error */
  const double err_x = vehicle_status_.pose.position.x - nearest_pose.position.x;
  const double err_y = vehicle_status_.pose.position.y - nearest_pose.position.y;
  const double sp_yaw = tf2::getYaw(nearest_pose.orientation);
  const double err_lat = -sin(sp_yaw) * err_x + cos(sp_yaw) * err_y;

  /* calculate yaw error, convert into range [-pi to pi] */
  const double err_yaw = MPCUtils::intoSemicircle(nearest_yaw_error);

  /* get steering angle */
  const double steer = vehicle_status_.tire_angle_rad;

  /* define initial state for error dynamics */
  Eigen::VectorXd x0(DIM_X, 1);
  x0 << err_lat, err_yaw, steer;

  DEBUG_INFO("[calculateMPC] lat error = %f, yaw error = %f, steer = %f, sp_yaw = %f, my_yaw = %f", err_lat, err_yaw, steer, sp_yaw, current_yaw);

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
  Eigen::VectorXd MPC_T = Eigen::VectorXd::Zero(N);

  /* weight matrix depends on the vehicle model */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Q(0, 0) = mpc_param_.weight_lat_error;
  Q(1, 1) = mpc_param_.weight_heading_error;
  R(0, 0) = mpc_param_.weight_steering_input + mpc_param_.weight_steering_input_vel_coeff * std::fabs(vehicle_status_.twist.linear.x);

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);
  Eigen::MatrixXd Uref(DIM_U, 1);

  /* resample ref_traj with mpc sampling time */
  MPCTrajectory mpc_resampled_ref_traj;
  std::vector<double> mpc_time_v;
  double mpc_time_tmp = mpc_start_time;
  for (int i = 0;i < N; ++i) {
    mpc_time_tmp += mpc_param_.dt;
    mpc_time_v.push_back(mpc_time_tmp);
    MPC_T[i] = mpc_time_tmp;
  }
  if (!MPCUtils::interp1dMPCTraj(ref_traj_.relative_time, ref_traj_, mpc_time_v, mpc_resampled_ref_traj)) {
    ROS_WARN("[calculateMPC] mpc resample error, stop mpc calculation. check code!");
    return false;
  }

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i)
  {
    const double ref_vx = mpc_resampled_ref_traj.vx[i];
    const double ref_k = mpc_resampled_ref_traj.k[i];

    /* get discrete state matrix A, B, C, W */
    vehicle_model_.setVel(ref_vx);
    vehicle_model_.setCurvature(ref_k);
    vehicle_model_.calculateDiscreteMatrix(Ad, Bd, Cd, Wd, mpc_param_.dt);

    /* update mpc matrix */
    if (i == 0)
    {
      nearest_ref_k = ref_k;
      Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      Wex.block(0, 0, DIM_X, 1) = Wd;
      Cex.block(0, 0, DIM_Y, DIM_X) = Cd;
      Qex.block(0, 0, DIM_Y, DIM_Y) = Q;
      Rex.block(0, 0, DIM_U, DIM_U) = R;
    }
    else
    {
      int idx_x_i = i * DIM_X;
      int idx_x_i_prev = (i - 1) * DIM_X;
      int idx_u_i = i * DIM_U;
      int idx_y_i = i * DIM_Y;
      Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j)
      {
        int idx_u_j = j * DIM_U;
        Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) = Ad * Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
      Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
      Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
      Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q;
      Rex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R;
    }

    /* get reference input (feed-forward) */
    if (std::fabs(ref_k) < mpc_param_.zero_curvature_range)
      vehicle_model_.calculateReferenceInput(Uref, 0.0); // with 0 curvature
    else
      vehicle_model_.calculateReferenceInput(Uref); // with curvature set above

    Urefex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  /////////////// optimization ///////////////
  /*
   * solve quadratic optimization.
   * cost function: Uex' * H * Uex + f' * Uex
   */
  const Eigen::MatrixXd CB = Cex * Bex;
  const Eigen::MatrixXd QCB = Qex * CB;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB;
  H.triangularView<Eigen::Upper>() += Rex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  const Eigen::MatrixXd f = (Cex * (Aex * x0 + Wex)).transpose() * QCB - Urefex.transpose() * Rex;
  Eigen::VectorXd Uex;

  auto start = std::chrono::system_clock::now();
  if (!qpsolver::solveEigenLeastSquareLLT(H, f, Uex)){
    ROS_WARN("qp solver error");
    return false;
  }
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
  DEBUG_INFO("[calculateMPC] qp solver calculation time = %f [ms]", elapsed * 1.0e-6);  

  /* time delay compensation, look ahead delay_compensation_time for optimized input vector*/
  double u_delay_comped;
  double total_delay = (ros::Time::now() - vehicle_status_.header.stamp).toSec();
  DEBUG_INFO("[calculateMPC] total delay time = %f [s]", total_delay);
  // if (!MPCUtils::interp1d(MPC_T, Uex, nearest_traj_time + mpc_param_.delay_compensation_time, u_delay_comped))
  if (!MPCUtils::interp1dX<std::vector<double>, Eigen::VectorXd>(mpc_time_v, Uex, nearest_traj_time + mpc_param_.delay_compensation_time, u_delay_comped))
  {
    ROS_ERROR("invalid interpolation for u_delay");
  }
  DEBUG_INFO("[calculateMPC] mpc steering angle command = %f [deg] (no delay comp = %f)", u_delay_comped * RAD2DEG, Uex(0) * RAD2DEG);

  /* saturation */
  double u_sat = std::max(std::min(u_delay_comped, steer_lim_deg_ * DEG2RAD), -steer_lim_deg_ * DEG2RAD);

  /* filtering */
  double u_filtered = lpf_steering_cmd_.filter(u_sat);

  /* set steering command */
  steer_cmd = u_filtered;

  /////////////// velocity control ////////////////
  /* For simplicity, now we calculate steer and speed separately */
  double lookahead_time = 1.0;                                  // [s]
  double cmd_vel_ref_time = nearest_traj_time + lookahead_time; // get ahead reference velocity
  if (!MPCUtils::interp1d(ref_traj_.relative_time, ref_traj_.vx, cmd_vel_ref_time, vel_cmd))
  {
    ROS_ERROR("invalid interpolation for vel_cmd");
  }
  DEBUG_INFO("[calculateMPC] velocitycommand = %f [m/s]", vel_cmd);



  ////////////////// DEBUG ///////////////////

  /* calculate predicted trajectory */
  Eigen::VectorXd Xex = Aex * x0 + Bex * Uex + Wex;

  /* calcuate trajectory from reference trajectory and tracking error */
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
  convertTrajToMarker(debug_mpc_predicted_traj, marker, "predicted_traj", 1.0, 0.0, 0.0);
  pub_debug_predicted_traj_.publish(marker);

  /* publish debug values */
  const double input_curvature = tan(steer_cmd) / wheelbase_;
  std_msgs::Float64MultiArray debug_values;
  debug_values.data.clear();
  debug_values.data.push_back(u_sat);
  debug_values.data.push_back(u_filtered);
  debug_values.data.push_back(err_lat);
  debug_values.data.push_back(err_yaw);
  debug_values.data.push_back(steer);
  debug_values.data.push_back(nearest_ref_k);
  debug_values.data.push_back(input_curvature);
  debug_values.data.push_back(MPCUtils::intoSemicircle(current_yaw));
  debug_values.data.push_back(MPCUtils::intoSemicircle(sp_yaw));
  pub_debug_values_.publish(debug_values);
  return true;
};

void MPCFollower::publishControlCommands(const double &vel_cmd, const double &steer_cmd)
{
  const double omega_cmd = vehicle_status_.twist.linear.x * std::tan(steer_cmd) / wheelbase_;
  switch (ctrl_cmd_interface_)
  {
  case CtrlCmdInterface::TWIST:
    publishAsTwist(vel_cmd, omega_cmd);
    break;
  case CtrlCmdInterface::STEER:
    // TODO: write me
    ROS_WARN("control command interface STEER is not implemented");
    break;
  case CtrlCmdInterface::STEER_AND_VEL:
    publishSteerAndVel(vel_cmd, steer_cmd);
    break;
  case CtrlCmdInterface::ALL:
    publishAsTwist(vel_cmd, omega_cmd);
    publishSteerAndVel(vel_cmd, steer_cmd);
    break;
  default:
    ROS_WARN("control command interface is not appropriate");
    break;
  }
}

void MPCFollower::publishAsTwist(const double &vel_cmd, const double &omega_cmd)
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

void MPCFollower::publishSteerAndVel(const double &vel_cmd, const double &steer_cmd)
{
  autoware_msgs::ControlCommandStamped cmd;
  cmd.header.frame_id = "/base_link";
  cmd.header.stamp = ros::Time::now();
  cmd.cmd.linear_velocity = vel_cmd;
  cmd.cmd.linear_acceleration = (vel_cmd - vehicle_status_.twist.linear.x) / ctrl_period_;
  cmd.cmd.steering_angle = steer_cmd;
  pub_steer_vel_ctrl_cmd_.publish(cmd);
}

void MPCFollower::callbackRefPath(const autoware_msgs::Lane::ConstPtr &msg)
{
  const auto start = std::chrono::system_clock::now();

  current_waypoints_ = *msg;
  DEBUG_INFO("[path callback] received path size = %lu", current_waypoints_.waypoints.size());

  MPCTrajectory traj;

  /* calculate relative time */
  std::vector<double> relative_time;
  MPCUtils::calcPathRelativeTime(current_waypoints_, relative_time);
  DEBUG_INFO("[path callback] relative_time.size() = %lu, front() = %f, back() = %f",
             relative_time.size(), relative_time.front(), relative_time.back());

  /* resampling */
  MPCUtils::resamplePathToTrajByDistance(current_waypoints_, relative_time, traj_resample_dist_, traj);
  MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  DEBUG_INFO("[path callback] resampled traj size() = %lu", traj.relative_time.size());

  /* path smoothing */
  if (use_path_smoothing_)
  {
    for (int i = 0; i < path_smoothing_times_; ++i) {
    MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.x);
    MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.y);
    // MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.z);
    MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.yaw);
    // MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.k);
    }
  }

  /* calculate yaw angle */
  // MPCUtils::calcTrajectoryYawFromXY(traj);

  /* calculate curvature */
  MPCUtils::calcTrajectoryCurvature(traj, curvature_smoothing_num_);
  DEBUG_INFO("[path callback] trajectory curvature : max_k = %f, min_k = %f", *max_element(traj.k.begin(), traj.k.end()), *min_element(traj.k.begin(), traj.k.end()));

  /* add end point with vel=0 on traj for mpc prediction */
  const double mpc_predict_time_length = mpc_param_.n * mpc_param_.dt;
  const double end_velocity = 0.0;
  traj.vx.back() = 0.0; // also for end point
  traj.push_back(traj.x.back(), traj.y.back(), traj.z.back(), traj.yaw.back(),
                 end_velocity, traj.k.back(), traj.relative_time.back() + mpc_predict_time_length);

  if (!traj.size())
  {
    ROS_ERROR("[callbackRefPath] trajectory size is undesired.");
    DEBUG_INFO("size: x=%lu, y=%lu, z=%lu, yaw=%lu, v=%lu,k=%lu,t=%lu", traj.x.size(), traj.y.size(), traj.z.size(), traj.yaw.size(), traj.vx.size(), traj.k.size(), traj.relative_time.size());
  }

  ref_traj_ = traj;

  const auto end = std::chrono::system_clock::now();
  const double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  DEBUG_INFO("[path callback] path modification total time = %f [ms]", elapsed * 1.0e-6);

  /* publish trajectory for visualize */
  visualization_msgs::Marker markers;
  convertTrajToMarker(ref_traj_, markers, "ref_traj", 0.0, 0.0, 1.0);
  pub_debug_filtered_traj_.publish(markers);
};

void MPCFollower::convertTrajToMarker(const MPCTrajectory &traj, visualization_msgs::Marker &marker,
                                      std::string ns, double r, double g, double b)
{
  marker.points.clear();
  marker.header.frame_id = current_waypoints_.header.frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  for (unsigned int i = 0; i < traj.x.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = traj.x.at(i);
    p.y = traj.y.at(i);
    p.z = traj.z.at(i);
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
  vehicle_status_.tire_angle_rad = msg.angle / 60.0;
  vehicle_status_.twist.linear.x = msg.speed * 1000.0 / 3600.0;
  my_steering_ok_ = true;
  my_velocity_ok_ = true;
};

MPCFollower::~MPCFollower()
{
  ROS_INFO("Publish 0 twist before I died.");
  double vel_cmd = 0.0;
  double steer_cmd = 0.0;
  if (my_steering_ok_)
    steer_cmd = vehicle_status_.tire_angle_rad;
  publishControlCommands(vel_cmd, steer_cmd);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_follower");
  MPCFollower obj;
  ros::spin();
  return 0;
};
