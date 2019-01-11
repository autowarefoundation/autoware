#include <vector>
#include <iostream>
#include <limits>
#include <chrono>

#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"
#include "utils.h"
#include "lowpass_filter.h"
#include "mpc_trajectory.h"

using vector = Eigen::VectorXd;
using matrix = Eigen::MatrixXd;


#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

#define MPC_DEBUG_VERBOSE
#ifdef MPC_DEBUG_VERBOSE
#define MPC_INFO(...) printf(__VA_ARGS__)
#else
#define MPC_INFO(...)
#endif




class MPCFollower {
public:
  MPCFollower();
  ~MPCFollower();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_steering_, pub_twist_cmd_, pub_debug_filtered_traj_, pub_debug_predicted_traj_, pub_debug_values_;
  ros::Subscriber sub_ref_path_, sub_twist_, sub_pose_, sub_steering_, sub_sim_odom_;
  ros::Timer timer_control_;
  std::string path_frame_id_;


  autoware_msgs::Lane current_ref_path_;
  MPCTrajectory ref_traj_;
  Butterworth2d lpf_steering_cmd_;

  enum CtrlCmdInterface {
    TWIST = 0,
    STEER = 1,
    STEER_AND_VEL = 2,
  };

  /* parameters */
  std::string ctrl_cmd_interface_string_; // currentlly, twist or steer_and_vel
  CtrlCmdInterface ctrl_cmd_interface_;
  double ctrl_dt_; // deside control frequency
  bool use_path_smoothing_; // for path smoothing
  double traj_resample_dt_; // used for resample path
  double moving_filter_num_; // for path smoothing
  double steering_lpf_cutoff_hz_; // for steering command smoothing
  double admisible_lateral_error_; //  stop mpc calculation when lateral error is large than this value.

  struct MPCParam {
    int n;
    double dt;
    double weight_lat_error;
    double weight_heading_error;
    double weight_steering_input;
    double delay_compensation_time;
    int dim_state_;
    int dim_input_;
    int dim_output_;
  };
  MPCParam mpc_param_; // for mpc design

  struct VehicleModel {
    double steer_tau;
    double wheelbase;
    double steer_lim_deg;
  };
  VehicleModel vehicle_model_; // vehicle physical parameters

  struct VehicleStatus {
    std::string frame_id_pos;
    double posx;
    double posy;
    double posz;
    double yaw;
    double vx;
    double wz;
    double steer_rad;
  };
  VehicleStatus vehicle_status_; // updated by topic callback






  void timerCallback(const ros::TimerEvent&);
  void callbackRefPath(const autoware_msgs::Lane::ConstPtr&);
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr&);
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr&);
  void callbackSteering(const std_msgs::Float64 &);
  void callbackSimOdom(const nav_msgs::Odometry::ConstPtr&);

  void calcRelativeTimeForPath(const autoware_msgs::Lane&, std::vector<double> &);
  void resamplePathToTraj(const autoware_msgs::Lane &path, const std::vector<double> &time, const double &resample_dt, MPCTrajectory &ref_traj_);
  void convertTraj2Marker(const MPCTrajectory &traj, visualization_msgs::Marker &markers, std::string ns, double r, double g, double b);
  void calcTrajectoryCurvature(MPCTrajectory &traj);
  void publishAsTwist(double &vel_cmd, double &steer_cmd);

  bool calculateMPC(double &vel_cmd, double &steer_cmd);
  void getErrorDynamicsStateMatrix(const double &dt, const double &ref_vx,
                                   const double &wheelbase,
                                   const double &steer_tau,
                                   const double &ref_curvature, matrix &Ad,
                                   matrix &Bd, matrix &Wd,
                                   matrix &Cd);
};

MPCFollower::MPCFollower()
    : nh_(""), pnh_("~"), path_frame_id_(""){

  pnh_.param("ctrl_cmd_interface", ctrl_cmd_interface_string_, std::string("twist"));
  pnh_.param("ctrl_dt", ctrl_dt_, double(0.05));
  pnh_.param("use_path_smoothing", use_path_smoothing_, bool(true));
  pnh_.param("traj_resample_dt", traj_resample_dt_, double(0.1));
  pnh_.param("moving_filter_num", moving_filter_num_, double(5));
  pnh_.param("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz_, double(5.0));
  pnh_.param("admisible_lateral_error", admisible_lateral_error_, double(5.0));

  /* mpc parameters */
  pnh_.param("mpc_n", mpc_param_.n, int(100));
  pnh_.param("mpc_dt", mpc_param_.dt, double(0.05));
  pnh_.param("mpc_weight_lat_error", mpc_param_.weight_lat_error, double(1.0));
  pnh_.param("mpc_weight_heading_error", mpc_param_.weight_heading_error, double(1.0));
  pnh_.param("mpc_weight_steering_input", mpc_param_.weight_steering_input, double(1.0));
  pnh_.param("mpc_delay_compensation_time", mpc_param_.delay_compensation_time, double(0.02));
  pnh_.param("mpc_dim_state", mpc_param_.dim_state_, int(3));
  pnh_.param("mpc_dim_input", mpc_param_.dim_input_, int(1));
  pnh_.param("mpc_dim_output", mpc_param_.dim_output_, int(2));

  /*/ vehicle model */
  pnh_.param("vehicle_model_steer_tau", vehicle_model_.steer_tau, double(0.2));
  pnh_.param("vehicle_model_wheelbase", vehicle_model_.wheelbase, double(2.9));
  pnh_.param("vehicle_model_steer_lim_deg", vehicle_model_.steer_lim_deg, double(30.0));


  /* initialize for vehicle status */
  vehicle_status_.frame_id_pos = "";
  vehicle_status_.posx = 0.0;
  vehicle_status_.posy = 0.0;
  vehicle_status_.posz = 0.0;
  vehicle_status_.yaw = 0.0;
  vehicle_status_.vx = 0.0;
  vehicle_status_.wz = 0.0;
  vehicle_status_.steer_rad = 0.0;

  /* set control command interface */
  if (ctrl_cmd_interface_string_ == "twist") {
    ctrl_cmd_interface_ = CtrlCmdInterface::TWIST;
  } else if (ctrl_cmd_interface_string_ == "steer_and_vel") {
    ctrl_cmd_interface_ = CtrlCmdInterface::STEER_AND_VEL;
  } else if (ctrl_cmd_interface_string_ == "steer") {
    ctrl_cmd_interface_ = CtrlCmdInterface::STEER;
  }


  timer_control_ = nh_.createTimer(ros::Duration(ctrl_dt_), &MPCFollower::timerCallback, this);

  pub_steering_  = nh_.advertise<autoware_msgs::ControlCommandStamped>("/ctrl_cmd", 1);
  pub_twist_cmd_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 1);
  pub_debug_filtered_traj_ = pnh_.advertise<visualization_msgs::Marker>("debug/filtered_traj", 1);
  pub_debug_predicted_traj_ = pnh_.advertise<visualization_msgs::Marker>("debug/predicted_traj", 1);
  pub_debug_values_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug/debug_values", 1);

  sub_ref_path_ = nh_.subscribe("/base_waypoints", 1, &MPCFollower::callbackRefPath, this);
  sub_twist_ = nh_.subscribe("/vehicle_info/twist", 1, &MPCFollower::callbackTwist, this);
  sub_pose_ = nh_.subscribe("/current_pose", 1, &MPCFollower::callbackPose, this);
  sub_steering_ = nh_.subscribe("/vehicle_info/steering_angle", 1, &MPCFollower::callbackSteering, this);

  lpf_steering_cmd_.initialize(ctrl_dt_, steering_lpf_cutoff_hz_);

  /* for DEBUG */
  // sub_sim_odom_ = nh_.subscribe("/autoware_gazebo/diff_drive_controller/odom", 1, &MPCFollower::callbackSimOdom, this);

};

void MPCFollower::timerCallback(const ros::TimerEvent& te) {

  /* control command */
  double vel_cmd(0.0), steer_cmd(0.0);

  /* solve MPC */
  auto start = std::chrono::system_clock::now();
  bool mpc_solved = calculateMPC(vel_cmd, steer_cmd);
  auto end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  printf("MPC calculating time = %f [ms]\n", elapsed * 1.0e-6);

  if (!mpc_solved) {
    ROS_WARN("MPC is not solved\n");
    // TODO: this is very temporal code. Stop when mpc error occurs.
    vel_cmd = 0.0;
    steer_cmd = 0.0;
  }

  /* publish control command */
  switch (ctrl_cmd_interface_) {
  case CtrlCmdInterface::TWIST:
    publishAsTwist(vel_cmd, steer_cmd);
    break;
  case CtrlCmdInterface::STEER:
    // TODO: write me
    break;
  case CtrlCmdInterface::STEER_AND_VEL:
    // TODO: write me
    break;
  default:
    // TODO: write me
    break;
  }
};

bool MPCFollower::calculateMPC(double &vel_cmd, double &steer_cmd) {

  if (ref_traj_.size() == 0) {
    return false;
  }

  /* parameter definition */
  static const double rad2deg = 180.0 / M_PI;
  static const double deg2rad = M_PI / 180.0;
  const int N = mpc_param_.n;
  const int DIM_X = mpc_param_.dim_state_;
  const int DIM_U = mpc_param_.dim_input_;
  const int DIM_Y = mpc_param_.dim_output_;


  /* calculate nearest point on reference trajectory (used as initial state) */
  uint nearest_index = 0;
  double min_dist_squared = std::numeric_limits<double>::max();
  for (uint i = 0; i < ref_traj_.size(); ++i) {
    const double dx = vehicle_status_.posx - ref_traj_.x[i];
    const double dy = vehicle_status_.posy - ref_traj_.y[i];
    const double dist_squared = dx * dx + dy * dy;
    if (dist_squared < min_dist_squared) {

      /* ignore when yaw error is large, for crossing path */
      const double err_yaw = intoSemicircle(vehicle_status_.yaw - ref_traj_.yaw[i]);
      if (fabs(err_yaw) < (M_PI / 2.0)) {

        /* save nearest index */
        min_dist_squared = dist_squared;
        nearest_index = i;
      }
    }
  }
  MPC_INFO("[calculateMPC] nearest_index = %d, min_dist_squared = %f\n",nearest_index, min_dist_squared);

  /* check if lateral error is not too large */
  if (std::sqrt(min_dist_squared) > admisible_lateral_error_) {
    ROS_WARN("[calculateMPC] lateral error is large than admisible error, stop "
             "mpc. (error: %f[m], admisible: %f[m]\n", std::sqrt(min_dist_squared), admisible_lateral_error_);
    return false;
  }

  /* set mpc initial time */
  const double nearest_traj_time = ref_traj_.relative_time[nearest_index];
  double mpc_time = nearest_traj_time; /* as initialize */

  /* check trajectory length */
  const double mpc_end_time = mpc_time + (N-1)*mpc_param_.dt;
  if (mpc_end_time > ref_traj_.relative_time.back()) {
    ROS_WARN("[calculateMPC] path is too short to predict dynamics. \n");
    ROS_WARN("[calculateMPC] path end time: %f, mpc end time: %f\n", ref_traj_.relative_time.back(), mpc_end_time);
    return false;
  }

  /* convert tracking x,y error to lat error */
  const double err_x = vehicle_status_.posx - ref_traj_.x[nearest_index];
  const double err_y = vehicle_status_.posy - ref_traj_.y[nearest_index];
  const double sp_yaw = ref_traj_.yaw[nearest_index];
  const double err_lat = -sin(sp_yaw) * err_x + cos(sp_yaw) * err_y;

  /* calculate yaw error, convert into range [-pi to pi] */
  const double my_yaw = vehicle_status_.yaw;
  const double err_yaw = intoSemicircle(my_yaw - sp_yaw);

  /* get steering angle */
  const double steer = vehicle_status_.steer_rad;

  /* define initial state for error dynamics */
  matrix x0(DIM_X,1);
  x0 << err_lat, err_yaw, steer;

  MPC_INFO("lat error = %f, yaw error = %f, steer = %f, sp_yaw = %f, my_yaw = %f\n", err_lat, err_yaw, steer, sp_yaw, my_yaw);



  /////////////// generate mpc matrix  ///////////////

  /*
   * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
   * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * Rex * (Uex - Uref)
   * Qex = diag([Q,Q,...]), Rex = diag([R,R,...])
   */

  matrix Aex = matrix::Zero(DIM_X * N, DIM_X);
  matrix Bex = matrix::Zero(DIM_X * N, DIM_U * N);
  matrix Wex = matrix::Zero(DIM_X * N, 1);
  matrix Cex = matrix::Zero(DIM_Y * N, DIM_X * N);
  matrix Qex = matrix::Zero(DIM_Y * N, DIM_Y * N);
  matrix Rex = matrix::Zero(DIM_U * N, DIM_U * N);
  matrix Uref = matrix::Zero(DIM_U * N, 1);
  vector MPC_T = vector::Zero(N);

  matrix Q = matrix::Zero(DIM_Y, DIM_Y);
  matrix R = matrix::Zero(DIM_U, DIM_U);
  Q(0, 0) = mpc_param_.weight_lat_error;
  Q(1, 1) = mpc_param_.weight_heading_error;
  R(0, 0) = mpc_param_.weight_steering_input;


  matrix Ad(DIM_X, DIM_X);
  matrix Bd(DIM_X, DIM_U);
  matrix Wd(DIM_X, 1);
  matrix Cd(DIM_Y, DIM_X);

  MPCTrajectory debug_ref_vec; /* DEBUG: to calculate predicted trajectory */


  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i) {
    // MPC_INFO("mpc time = %f\n",mpc_time );
    MPC_T[i] = mpc_time;

    /* get reference information at mpc_time by interpolation */
    double ref_vx; /* reference velocity at current mpc time */
    double ref_k;  /* reference curvature at current mpc time */
    if (!interp1d(ref_traj_.relative_time, ref_traj_.vx, mpc_time, ref_vx) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.k, mpc_time, ref_k)) {
      ROS_ERROR("invalid interpolation\n");
      return false;
    }

    /* DEBUG:  to predict trajectory */
    double debug_ref_x_tmp, debug_ref_y_tmp, debug_ref_z_tmp, debug_ref_yaw_tmp;
    if (!interp1d(ref_traj_.relative_time, ref_traj_.x, mpc_time, debug_ref_x_tmp) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.y, mpc_time, debug_ref_y_tmp) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.z, mpc_time, debug_ref_z_tmp) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.yaw, mpc_time, debug_ref_yaw_tmp)) {
      ROS_ERROR("invalid interpolation\n");
      return false;
    }
    debug_ref_vec.push_back(debug_ref_x_tmp, debug_ref_y_tmp, debug_ref_z_tmp, debug_ref_yaw_tmp, 0, 0, 0);
    // MPC_INFO("\ndebug_ref_x_tmp = %f, debug_ref_y_tmp = %f, debug_ref_z_tmp = %f, debug_ref_yaw_tmp = %f\n", debug_ref_x_tmp, debug_ref_y_tmp, debug_ref_z_tmp, debug_ref_yaw_tmp);

    /* get discrete state matrix */
    getErrorDynamicsStateMatrix(mpc_param_.dt, ref_vx, vehicle_model_.wheelbase,
                                vehicle_model_.steer_tau, ref_k, Ad, Bd, Wd, Cd);
    /* update mpc matrix */
    if (i == 0) {
      Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      Wex.block(0, 0, DIM_X, 1) = Wd;
      Cex.block(0, 0, DIM_Y, DIM_X) = Cd;
      Qex.block(0, 0, DIM_Y, DIM_Y) = Q;
      Rex.block(0, 0, DIM_U, DIM_U) = R;
    } else {
      int idx_x_i = i * DIM_X;
      int idx_x_i_prev = (i - 1) * DIM_X;
      int idx_u_i = i * DIM_U;
      int idx_y_i = i * DIM_Y;
      Aex.block(idx_x_i, 0, DIM_X, DIM_X) =
          Ad * Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * DIM_U;
        Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) =
            Ad * Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
      Wex.block(idx_x_i, 0, DIM_X, 1) =
          Ad * Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
      Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
      Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q;
      Rex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R;
    }
    Uref(i * DIM_U, 0) = std::atan(vehicle_model_.wheelbase * ref_k);
    // printf("i = %d, ref_k = %f, uref = %f\n", i, ref_k, Uref(i*DIM_U, 0));

    /* update mpc time */
    mpc_time += mpc_param_.dt;
  }



  /////////////// optimization ///////////////
  /*
   * solve quadratic optimization.
   * cost function: Uex' * H * Uex + f' * Uex
   */
  matrix H = (Cex * Bex).transpose() * Qex * Cex * Bex + Rex;
  matrix f = (Cex * (Aex * x0 + Wex)).transpose() * Qex * Cex * Bex - Uref.transpose() * Rex;
  vector Uex = -H.inverse() * f.transpose(); /* least square */
  // PRINT_MAT(-H.inverse() * f.transpose());

  /* time delay compensation, look ahead delay_compensation_time for optimized input vector*/
  double u_delay_comped;
  interp1d(MPC_T, Uex, nearest_traj_time + mpc_param_.delay_compensation_time, u_delay_comped);
  MPC_INFO("mpc steering angle command = %f [deg] (no delay comp = %f)\n", u_delay_comped * rad2deg, Uex(0) * rad2deg);

  /* saturation */
  double u_sat = std::max(std::min(u_delay_comped, vehicle_model_.steer_lim_deg * deg2rad), -vehicle_model_.steer_lim_deg * deg2rad);

  /* filtering */
  double u_filtered = lpf_steering_cmd_.filter(u_sat);

  /* set steering command */
  steer_cmd = u_filtered;




  ////////// calculating velocity //////////
  // NOTE: this is temporal for velocity control
  double lookahead_time = 1.0; // [s]
  double cmd_vel_ref_time = nearest_traj_time + lookahead_time; // get ahead reference velocity
  interp1d(ref_traj_.relative_time, ref_traj_.vx, cmd_vel_ref_time, vel_cmd);




  ////////// DEBUG: calculate predicted trajectory //////////
  vector Xex = Aex * x0 + Bex * Uex + Wex;

  /* calcuate trajectory from reference trajectory and tracking error */
  MPCTrajectory debug_mpc_predicted_traj;
  for (int i = 0; i < N; ++i) {
    const double yaw_ref = debug_ref_vec.yaw[i];
    const double lat_error = Xex(i * DIM_X);
    const double yaw_error = Xex(i * DIM_X + 1);
    const double x = debug_ref_vec.x[i] - std::sin(yaw_ref) * lat_error;
    const double y = debug_ref_vec.y[i] + std::cos(yaw_ref) * lat_error;
    const double z = debug_ref_vec.z[i];
    debug_mpc_predicted_traj.push_back(x, y, z, yaw_ref + yaw_error, 0, 0, 0);
  }

  /* publish for visualization */
  visualization_msgs::Marker marker;
  convertTraj2Marker(debug_mpc_predicted_traj, marker, "predicted_traj", 1.0, 0.0, 0.0);
  pub_debug_predicted_traj_.publish(marker);




  ////////// publish debug values //////////
  std_msgs::Float64MultiArray debug_values;
  double nearest_ref_k = ref_traj_.k[nearest_index];
  debug_values.data.clear();
  debug_values.data.push_back(u_sat);
  debug_values.data.push_back(u_filtered);
  debug_values.data.push_back(err_lat);
  debug_values.data.push_back(err_yaw);
  debug_values.data.push_back(steer);
  debug_values.data.push_back(nearest_ref_k);
  pub_debug_values_.publish(debug_values);


  return true;

};

void MPCFollower::getErrorDynamicsStateMatrix(
    const double &dt, const double &v, const double &wheelbase,
    const double &steer_tau, const double &ref_curvature, matrix &Ad,
    matrix &Bd, matrix &Wd, matrix &Cd) {


  int DIM_X = 3;
  int DIM_U = 1;
  int DIM_Y = 2;

  auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

  /* linearization around delta = delta_r */
  double delta_r = atan(wheelbase * ref_curvature);
  if (abs(delta_r) >= vehicle_model_.steer_lim_deg / 180.0 * M_PI)
    delta_r = (vehicle_model_.steer_lim_deg / 180.0 * M_PI) * (double)sign(delta_r);

  double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

  /* difine continuous model */
  matrix A(DIM_X, DIM_X);
  matrix B(DIM_X, DIM_U);
  matrix W(DIM_X, 1);
  matrix C(DIM_Y, DIM_X);
  A << 0.0, v, 0.0,
       0.0, 0.0, v / wheelbase * cos_delta_r_squared_inv,
       0.0, 0.0, -1.0 / steer_tau;

  B << 0.0, 0.0, 1.0 / steer_tau;
  C << 1.0, 0.0, 0.0,
       0.0, 1.0, 0.0;
  W << 0.0,
      -v * ref_curvature + v / wheelbase * (tan(delta_r) - delta_r * cos_delta_r_squared_inv),
      0.0;

  /* bilinear discretization */
  // Ad = eye(3) + A * dt;
  matrix I = matrix::Identity(DIM_X, DIM_X);
  Ad = (I - dt * 0.5 * A).inverse() *  (I + dt * 0.5 * A);
  Bd = B * dt;
  Cd = C;
  Wd = W * dt;
}




void MPCFollower::publishAsTwist(double &vel_cmd, double &steer_cmd) {

  /* convert steering to twist */
  double omega = vel_cmd * std::tan(steer_cmd) / vehicle_model_.wheelbase;

  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "/base_link";
  twist.header.stamp = ros::Time::now();
  twist.twist.linear.x = vel_cmd;
  twist.twist.linear.y = 0.0;
  twist.twist.linear.z = 0.0;
  twist.twist.angular.x = 0.0;
  twist.twist.angular.y = 0.0;
  twist.twist.angular.z = omega;
  pub_twist_cmd_.publish(twist);
}




void MPCFollower::callbackRefPath(const autoware_msgs::Lane::ConstPtr& msg){
  current_ref_path_ = *msg;
  path_frame_id_ = msg->header.frame_id;
  // MPC_INFO("[path callback] current_ref_path_.size() = %d\n",current_ref_path_.waypoints.size());

  MPCTrajectory traj;

  /* calculate relative time */
  std::vector<double> relative_time;
  MPCFollower::calcRelativeTimeForPath(current_ref_path_, relative_time);
  // MPC_INFO("[path callback] relative_time.front() = %f, relative_time.back() = %f\n", ref_traj_.relative_time.front(), ref_traj_.relative_time.back());
  // MPC_INFO("[path callback] relative_time.size() = %d\n",relative_time.size());

  /* resampling */
  MPCFollower::resamplePathToTraj(current_ref_path_, relative_time, traj_resample_dt_, traj);
  convertEulerAngleToMonotonic(traj.yaw);
  // MPC_INFO("[path callback] resampled traj.relative_time.size() = %d\n",traj.relative_time.size());

  /* calculate curvature */
  MPCFollower::calcTrajectoryCurvature(traj);
  double max_k = *max_element(traj.k.begin(), traj.k.end());
  double min_k = *min_element(traj.k.begin(), traj.k.end());
  // MPC_INFO("[path callback] curvature (raw): max_k = %f, min_k = %f\n", max_k, min_k);


  /* low pass filter */
  auto start = std::chrono::system_clock::now();
  if (use_path_smoothing_) {
    filteringMovingAverate(traj.x, moving_filter_num_);
    filteringMovingAverate(traj.y, moving_filter_num_);
    filteringMovingAverate(traj.z, moving_filter_num_);
    filteringMovingAverate(traj.yaw, moving_filter_num_);
    filteringMovingAverate(traj.k, moving_filter_num_);
  }
  auto end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  // MPC_INFO("[path callback] curvature (filtered): max_k = %f, min_k = %f\n", max_k, min_k);
  // MPC_INFO("[path callback] path filtering time = %f [ms]\n", elapsed * 1.0e-6);

  if (!traj.size()) {
    ROS_ERROR("[callbackRefPath] trajectory size is undesired.\n");
    MPC_INFO("size: x=%lu, y=%lu, z=%lu, yaw=%lu, v=%lu,k=%lu,t=%lu\n",traj.x.size(), traj.y.size(), traj.z.size(), traj.yaw.size(), traj.vx.size(), traj.k.size(), traj.relative_time.size());
  }

  ref_traj_ = traj;

  /* publish trajectory for visualize */
  visualization_msgs::Marker markers;
  convertTraj2Marker(ref_traj_, markers, "ref_traj", 0.0, 0.0, 1.0);
  pub_debug_filtered_traj_.publish(markers);
};






void MPCFollower::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg){
  vehicle_status_.frame_id_pos = msg->header.frame_id;
  vehicle_status_.vx = msg->twist.linear.x;
  vehicle_status_.wz = msg->twist.angular.z;
};

void MPCFollower::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
  vehicle_status_.posx = msg->pose.position.x;
  vehicle_status_.posy = msg->pose.position.y;
  vehicle_status_.posz = msg->pose.position.z;
  vehicle_status_.yaw = tf2::getYaw(msg->pose.orientation);
};

void MPCFollower::callbackSteering(const std_msgs::Float64 &msg){
  vehicle_status_.steer_rad = msg.data;
};



void MPCFollower::calcRelativeTimeForPath(const autoware_msgs::Lane &path,
                                          std::vector<double> &path_time) {

  double t = 0.0;
  path_time.clear();
  path_time.push_back(t);
  for (int i = 0; i < (int)path.waypoints.size() - 1; ++i) {
    const double x0 = path.waypoints.at(i).pose.pose.position.x;
    const double y0 = path.waypoints.at(i).pose.pose.position.y;
    const double z0 = path.waypoints.at(i).pose.pose.position.z;
    const double x1 = path.waypoints.at(i+1).pose.pose.position.x;
    const double y1 = path.waypoints.at(i+1).pose.pose.position.y;
    const double z1 = path.waypoints.at(i+1).pose.pose.position.z;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;
    const double dist = sqrt(dx * dx + dy * dy + dz * dz);
    double v = std::max(std::fabs(path.waypoints.at(i).twist.twist.linear.x), 1.0);
    t += (dist / v);
    path_time.push_back(t);
    // MPC_INFO("i = %d, t = %f, dist = %f, v = %f, dx = %f, dy = %f, dz = %f\n",i,t,dist,v,dx,dy,dz);
  }

  // for (int i = 0; i < path_time.size(); i++) {
  //    printf("%f, ", path_time[i]);
  //  }
 // printf("\n");
 // printf("path_time.size() = %d\n", path_time.size());
 // printf("path_time.end() = %f\n", path_time.back());
 // printf("path_time.begin() = %f\n", path_time.front());
 //  printf("calcRelativeTimeForPath end\n");
}

void MPCFollower::resamplePathToTraj(const autoware_msgs::Lane &path, const std::vector<double> &time, const double &dt, MPCTrajectory &ref_traj_){


  ref_traj_.clear();
  double t = 0.0;

  while (t < time.back()) {
    uint j = 1;
    while (t > time.at(j)) {
      ++j;
      if (j > time.size() - 1){
        ROS_ERROR("[resamplePathToTraj] sampling time is not monotonically increasing");
        printf("t = %f, time.at(j-1)=%f\n", t, time.at(j-1));
        return;
      }
    }

    const double a = t - time.at(j - 1);
    const double path_dt_j = time.at(j) - time.at(j-1);
    const geometry_msgs::Pose pos0 = path.waypoints.at(j-1).pose.pose;
    const geometry_msgs::Pose pos1 = path.waypoints.at(j).pose.pose;
    const geometry_msgs::Twist twist0 = path.waypoints.at(j-1).twist.twist;
    const geometry_msgs::Twist twist1 = path.waypoints.at(j).twist.twist;
    const double x = ((path_dt_j - a) * pos0.position.x + a * pos1.position.x) / path_dt_j;
    const double y = ((path_dt_j - a) * pos0.position.y + a * pos1.position.y) / path_dt_j;
    const double z = ((path_dt_j - a) * pos0.position.z + a * pos1.position.z) / path_dt_j;

    /* for singular point of euler angle */
    const double yaw0 = tf2::getYaw(pos0.orientation);
    const double dyaw = intoSemicircle(tf2::getYaw(pos1.orientation) - yaw0);
    const double yaw1 = yaw0 + dyaw;
    const double yaw = ((path_dt_j - a) * yaw0 + a * yaw1) / path_dt_j;
    // const double yaw = ((path_dt_j - a) * tf2::getYaw(pos0.orientation) +
    //                 a * tf2::getYaw(pos1.orientation)) / path_dt_j;
    const double vx = ((path_dt_j - a) * twist0.linear.x + a * twist1.linear.x) / path_dt_j;
    const double curvature_tmp = 0.0;
    // MPC_INFO("t = %f, j = %d, a = %f, dt = %f", t, j, a, path_dt_j);
    // MPC_INFO("x0 = %f, x = %f, x1 = %f, ", pos0.position.x, x, pos1.position.x);
    // MPC_INFO("y0 = %f, y = %f, y1 = %f, ", pos0.position.y, y, pos1.position.y);
    // MPC_INFO("z0 = %f, z = %f, z1 = %f, ", pos0.position.z, z, pos1.position.z);
    // MPC_INFO("yaw0 = %f, yaw = %f, yaw1 = %f\n", tf2::getYaw(pos0.orientation), yaw, tf2::getYaw(pos1.orientation));
    ref_traj_.push_back(x, y, z, yaw, vx, curvature_tmp, t);
    t += dt;
  }
  // MPC_INFO("x = ");
  // for (int i = 0; i < ref_traj_.x.size(); ++i){
  //   MPC_INFO("%f, ", ref_traj_.x[i]);
  // }
  // MPC_INFO("\n");
  // MPC_INFO("y = ");
  // for (int i = 0; i < ref_traj_.y.size(); ++i){
  //   MPC_INFO("%f, ", ref_traj_.y[i]);
  // }
  // MPC_INFO("\n");

}

void MPCFollower::calcTrajectoryCurvature(MPCTrajectory &traj) {
  traj.k.clear();

  auto dist = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
    return std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
  };
  // MPC_INFO("k = ");

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::Point p1, p2, p3;
  int smoothing_num = 5;
  for (uint i = smoothing_num; i < traj.x.size() - smoothing_num; ++i) {
    p1.x = traj.x[i - smoothing_num];
    p2.x = traj.x[i];
    p3.x = traj.x[i + smoothing_num];
    p1.y = traj.y[i - smoothing_num];
    p2.y = traj.y[i];
    p3.y = traj.y[i + smoothing_num];
    const double curvature =
        2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) /
        (dist(p1, p2) * dist(p2, p3) * dist(p3, p1));
    traj.k.push_back(curvature);
    // MPC_INFO("%f, ", curvature);
  }

  /* first and last curvature is copied from next value */
  for (int i = 0; i < smoothing_num; ++i){
    traj.k.insert(traj.k.begin(), traj.k.front());
    traj.k.push_back(traj.k.back());
  }
}



void MPCFollower::convertTraj2Marker(const MPCTrajectory &traj, visualization_msgs::Marker &marker, std::string ns, double r, double g, double b){
  marker.points.clear();
  marker.header.frame_id = path_frame_id_;
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
  for (uint i = 0; i < traj.x.size(); ++i){
    geometry_msgs::Point p;
    p.x = traj.x.at(i);
    p.y = traj.y.at(i);
    p.z = traj.z.at(i);
    marker.points.push_back(p);
  }
}


void MPCFollower::callbackSimOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  double vx_ideal = msg->twist.twist.linear.x;
  double wz_ideal = msg->twist.twist.angular.z;

  printf("vx = %f, wz = %f, steer = %f\n", vx_ideal, wz_ideal, vehicle_status_.steer_rad);
}



MPCFollower::~MPCFollower(){};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpc_follower");
  MPCFollower obj;

  ros::spin();

  return 0;
};
