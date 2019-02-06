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
#include <pacmod_msgs/SystemRptFloat.h>
#include <pacmod_msgs/WheelSpeedRpt.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <glog/logging.h>

#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"

#include "utils.h"
#include "lowpass_filter.h"
#include "mpc_trajectory.h"
#include "kinematics_bicycle_model.h"

using vector = Eigen::VectorXd;
using matrix = Eigen::MatrixXd;

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

#define MPC_DEBUG_VERBOSE
#ifdef MPC_DEBUG_VERBOSE
#define MPC_INFO(...) { if (show_debug_info_) { printf(__VA_ARGS__); } }
#else
#define MPC_INFO(...)
#endif

class MPCFollower
{
public:
  MPCFollower();
  ~MPCFollower();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_steer_vel_ctrl_cmd_, pub_twist_cmd_;
  ros::Subscriber sub_ref_path_, sub_twist_, sub_pose_, sub_steering_, sub_vel_;
  ros::Timer timer_control_;

  autoware_msgs::Lane current_waypoints_; // current received waypoints
  MPCTrajectory ref_traj_;                // reference trajectory for mpc
  Butterworth2d lpf_steering_cmd_;        // steering command lowpass filter

  std::string robot_interface_; // set interface to receive velosity and steering data (pacmod / gazebo)
  std::string path_frame_id_;
  double previous_steering_command_;

  KinematicsBicycleModel vehicle_model_;

  /* set what is published to vehicle */
  enum CtrlCmdInterface
  {
    TWIST = 0,
    STEER = 1,
    STEER_AND_VEL = 2,
    ALL = 3,
  };
  CtrlCmdInterface ctrl_cmd_interface_;
  std::string ctrl_cmd_interface_string_; // currentlly, twist or steer_and_vel

  /* parameters */
  double ctrl_dt_;                    // deside control frequency
  bool use_path_smoothing_;           // flag for path smoothing
  double path_smoothing_cutoff_hz_;   // path smoothing cutoff frequency [Hz]
  int path_smoothing_moving_ave_num_; // path smoothing moving average number
  int curvature_smoothing_num_;       // for smoothing curvature calculation
  double traj_resample_dl_;           // path resample distance span
  double steering_lpf_cutoff_hz_;     // for steering command smoothing
  double admisible_position_error_;   // stop mpc calculation when lateral error is large than this value.
  double admisible_yaw_error_deg_;    // stop mpc calculation when yaw error is large than this value.
  double steer_cmd_lim_;              // steering command limit [rad]
  double wheelbase_;                  // only used to convert steering to twist

  struct MPCParam
  {
    int n;                          // prediction horizon step
    double dt;                      // prediction horizon period
    double weight_lat_error;        // for weight matrix Q
    double weight_heading_error;    // for weight matrix Q
    double weight_steering_input;   // for weight matrix R
    double delay_compensation_time; // use interpolation for time delay compensation
  };
  MPCParam mpc_param_; // for mpc design

  struct VehicleStatus
  {
    std::string frame_id_pos;
    double time;
    double posx;
    double posy;
    double posz;
    double yaw;
    double vx;
    double wz;
    double steer_rad;
  };
  VehicleStatus vehicle_status_; // updated by topic callback

  /* flags */
  bool my_position_ok_;
  bool my_velocity_ok_;
  bool my_steering_ok_;

  /* debug */
  ros::Publisher pub_debug_filtered_traj_, pub_debug_predicted_traj_, pub_debug_values_;
  ros::Subscriber sub_ndt_twist_;
  bool show_debug_info_;
  geometry_msgs::TwistStamped estimate_twist_;
  double estimated_curvature_;

  void timerCallback(const ros::TimerEvent &);
  void callbackRefPath(const autoware_msgs::Lane::ConstPtr &);
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr &);
  void callbackVelocity(const pacmod_msgs::WheelSpeedRpt &);
  void callbackSteering(const pacmod_msgs::SystemRptFloat &);
  void callbackVelocityGazebo(const std_msgs::Float64 &);
  void callbackSteeringGazebo(const std_msgs::Float64 &);

  void calcRelativeTimeForPath(const autoware_msgs::Lane &, std::vector<double> &);
  void resamplePathToTrajByTime(const autoware_msgs::Lane &path, const std::vector<double> &time, const double &resample_dt, MPCTrajectory &ref_traj_);
  void resamplePathToTrajByDistance(const autoware_msgs::Lane &path, const std::vector<double> &time, const double &resample_dl, MPCTrajectory &ref_traj_);
  void calcTrajectoryCurvature(MPCTrajectory &traj);
  void publishControlCommands(const double &vel_cmd, const double &steer_cmd);
  void publishAsTwist(const double &vel_cmd, const double &steer_cmd);
  void publishSteerAndVel(const double &vel_cmd, const double &steer_cmd);

  bool calculateMPC(double &vel_cmd, double &steer_cmd);
  void calculateNearestPose(unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error,
                            const double &my_x, const double &my_y, const double &my_yaw);
  void calcNearestPoseInterp(unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error,
                             double &nearest_x, double &nearest_y, double &nearest_yaw, double &nearest_time,
                             const double &my_x, const double &my_y, const double &my_yaw);
  void getErrorDynamicsStateMatrix(const double &dt, const double &ref_vx, const double &wheelbase,
                                   const double &steer_tau, const double &ref_curvature,
                                   matrix &Ad, matrix &Bd, matrix &Wd, matrix &Cd);

  /* debug */
  void convertTrajToMarker(const MPCTrajectory &traj, visualization_msgs::Marker &markers, std::string ns, double r, double g, double b);
  void callbackEstimateTwist(const geometry_msgs::TwistStamped::ConstPtr &msg);
};

MPCFollower::MPCFollower()
    : nh_(""), pnh_("~"), path_frame_id_(""), previous_steering_command_(0.0),
      my_position_ok_(false), my_velocity_ok_(false), my_steering_ok_(false),
      estimated_curvature_(0.0)
{

  pnh_.param("robot_interface", robot_interface_, std::string("pacmod"));
  pnh_.param("ctrl_cmd_interface", ctrl_cmd_interface_string_, std::string("all"));
  pnh_.param("ctrl_dt", ctrl_dt_, double(0.1));
  pnh_.param("use_path_smoothing", use_path_smoothing_, bool(true));
  pnh_.param("path_smoothing_cutoff_hz", path_smoothing_cutoff_hz_, double(1.0));
  pnh_.param("path_smoothing_moving_ave_num", path_smoothing_moving_ave_num_, int(5));
  pnh_.param("curvature_smoothing_num", curvature_smoothing_num_, int(10));
  pnh_.param("traj_resample_dl", traj_resample_dl_, double(0.1)); // [m]
  pnh_.param("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz_, double(3.0));
  pnh_.param("admisible_position_error", admisible_position_error_, double(5.0));
  pnh_.param("admisible_yaw_error_deg", admisible_yaw_error_deg_, double(45.0));
  pnh_.param("steer_cmd_lim", steer_cmd_lim_, double(35.0 * 3.1415 / 180.0));

  /* mpc parameters */
  pnh_.param("mpc_n", mpc_param_.n, int(50));
  pnh_.param("mpc_dt", mpc_param_.dt, double(0.1));
  pnh_.param("mpc_weight_lat_error", mpc_param_.weight_lat_error, double(1.0));
  pnh_.param("mpc_weight_heading_error", mpc_param_.weight_heading_error, double(1.0));
  pnh_.param("mpc_weight_steering_input", mpc_param_.weight_steering_input, double(1.0));
  pnh_.param("mpc_delay_compensation_time", mpc_param_.delay_compensation_time, double(0.05));

  pnh_.param("show_debug_info", show_debug_info_, bool(false));

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
  if (ctrl_cmd_interface_string_ == "twist")
    ctrl_cmd_interface_ = CtrlCmdInterface::TWIST;
  else if (ctrl_cmd_interface_string_ == "steer_and_vel")
    ctrl_cmd_interface_ = CtrlCmdInterface::STEER_AND_VEL;
  else if (ctrl_cmd_interface_string_ == "steer")
    ctrl_cmd_interface_ = CtrlCmdInterface::STEER;
  else if (ctrl_cmd_interface_string_ == "all")
    ctrl_cmd_interface_ = CtrlCmdInterface::ALL;

  /* initialize lowpass filter */
  lpf_steering_cmd_.initialize(ctrl_dt_, steering_lpf_cutoff_hz_);

  /* set up ros system */
  timer_control_ = nh_.createTimer(ros::Duration(ctrl_dt_), &MPCFollower::timerCallback, this);
  pub_twist_cmd_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 1);
  pub_steer_vel_ctrl_cmd_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("/ctrl_cmd", 1);
  sub_ref_path_ = nh_.subscribe("/base_waypoints", 1, &MPCFollower::callbackRefPath, this);
  sub_pose_ = nh_.subscribe("/current_pose", 1, &MPCFollower::callbackPose, this);

  /* setup can interface */
  if (robot_interface_ == "pacmod")
  {
    sub_vel_ = nh_.subscribe("/pacmod/parsed_tx/wheel_speed_rpt", 1, &MPCFollower::callbackVelocity, this);
    sub_steering_ = nh_.subscribe("/pacmod/parsed_tx/steer_rpt", 1, &MPCFollower::callbackSteering, this);
  }
  else if (robot_interface_ == "sim")
  {
    sub_vel_ = nh_.subscribe("/vehicle_info/velocity", 1, &MPCFollower::callbackVelocityGazebo, this);
    sub_steering_ = nh_.subscribe("/vehicle_info/steering_angle", 1, &MPCFollower::callbackSteeringGazebo, this);
  }

  /* for debug */
  pub_debug_filtered_traj_ = pnh_.advertise<visualization_msgs::Marker>("debug/filtered_traj", 1);
  pub_debug_predicted_traj_ = pnh_.advertise<visualization_msgs::Marker>("debug/predicted_traj", 1);
  pub_debug_values_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug/debug_values", 1);
  sub_ndt_twist_ = nh_.subscribe("/estimate_twist", 1, &MPCFollower::callbackEstimateTwist, this);
};

void MPCFollower::timerCallback(const ros::TimerEvent &te)
{

  /* check flags */
  if (ref_traj_.size() == 0 || !my_position_ok_ || !my_velocity_ok_ || !my_steering_ok_)
  {
    MPC_INFO("MPC is not solved. ref_traj_.size() = %d, my_position_ok_ = %d,  my_velocity_ok_ = %d,  my_steering_ok_ = %d\n",
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
  MPC_INFO("[timerCallback] MPC calculating time = %f [ms]\n", elapsed * 1.0e-6);

  if (!mpc_solved)
  {
    ROS_WARN("MPC is not solved. publish 0 velocity.\n");
    vel_cmd = 0.0;
    steer_cmd = previous_steering_command_;
  }

  publishControlCommands(vel_cmd, steer_cmd);

  previous_steering_command_ = steer_cmd;
};

bool MPCFollower::calculateMPC(double &vel_cmd, double &steer_cmd)
{

  /* parameter definition */
  static const double rad2deg = 180.0 / M_PI;
  static const double deg2rad = M_PI / 180.0;
  const int N = mpc_param_.n;
  const int DIM_X = vehicle_model_.getDimX();
  const int DIM_U = vehicle_model_.getDimU();
  const int DIM_Y = vehicle_model_.getDimY();

  double my_x = vehicle_status_.posx;
  double my_y = vehicle_status_.posy;
  double my_yaw = vehicle_status_.yaw;

  /* calculate nearest point on reference trajectory (used as initial state) */
  uint nearest_index = 0;
  double nearest_yaw_error = std::numeric_limits<double>::max();
  double nearest_dist_error = std::numeric_limits<double>::max();
  double nearest_x(0.0), nearest_y(0.0), nearest_yaw(0.0), nearest_traj_time(0.0), nearest_ref_k(0.0);
  calcNearestPoseInterp(nearest_index, nearest_dist_error, nearest_yaw_error,
                        nearest_x, nearest_y, nearest_yaw, nearest_traj_time, my_x, my_y, my_yaw);
  MPC_INFO("[calculateMPC] nearest_index = %d, nearest_dist_error = %f\n", nearest_index, nearest_dist_error);

  /* check if lateral error is not too large */
  if (nearest_dist_error > admisible_position_error_)
  {
    ROS_WARN("[calculateMPC] position error over limit, stop mpc. (error: %f[m], limit: %f[m])",
             nearest_dist_error, admisible_position_error_);
    return false;
  }
  if (std::fabs(nearest_yaw_error) > admisible_yaw_error_deg_ * deg2rad)
  {
    ROS_WARN("[calculateMPC] yaw error over limit, stop mpc.(error: %f[m], admisible: %f[m])",
             nearest_yaw_error * rad2deg, admisible_yaw_error_deg_);
    return false;
  }

  /* set mpc initial time */
  double mpc_time = nearest_traj_time; /* as initialize */
  MPC_INFO("[calculateMPC] nearest_traj_time = %f\n", nearest_traj_time);

  /* check trajectory length */
  const double mpc_end_time = mpc_time + (N - 1) * mpc_param_.dt;
  if (mpc_end_time > ref_traj_.relative_time.back())
  {
    ROS_WARN("[calculateMPC] path is too short to predict dynamics. ");
    ROS_WARN("[calculateMPC] path end time: %f, mpc end time: %f", ref_traj_.relative_time.back(), mpc_end_time);
    return false;
  }

  /* convert tracking x,y error to lat error */
  const double err_x = my_x - nearest_x;
  const double err_y = my_y - nearest_y;
  const double sp_yaw = nearest_yaw;
  const double err_lat = -sin(sp_yaw) * err_x + cos(sp_yaw) * err_y;

  /* calculate yaw error, convert into range [-pi to pi] */
  const double err_yaw = intoSemicircle(nearest_yaw_error);

  /* get steering angle */
  const double steer = vehicle_status_.steer_rad;

  /* define initial state for error dynamics */
  matrix x0(DIM_X, 1);
  x0 << err_lat, err_yaw, steer;

  MPC_INFO("[calculateMPC] lat error = %f, yaw error = %f, steer = %f, sp_yaw = %f, my_yaw = %f\n", err_lat, err_yaw, steer, sp_yaw, my_yaw);

  /////////////// generate mpc matrix  ///////////////
  /*
   * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
   * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * Rex * (Uex - Urefex)
   * Qex = diag([Q,Q,...]), Rex = diag([R,R,...])
   */

  matrix Aex = matrix::Zero(DIM_X * N, DIM_X);
  matrix Bex = matrix::Zero(DIM_X * N, DIM_U * N);
  matrix Wex = matrix::Zero(DIM_X * N, 1);
  matrix Cex = matrix::Zero(DIM_Y * N, DIM_X * N);
  matrix Qex = matrix::Zero(DIM_Y * N, DIM_Y * N);
  matrix Rex = matrix::Zero(DIM_U * N, DIM_U * N);
  matrix Urefex = matrix::Zero(DIM_U * N, 1);
  vector MPC_T = vector::Zero(N);

  /* weight matrix depends on the vehicle model */
  matrix Q = matrix::Zero(DIM_Y, DIM_Y);
  matrix R = matrix::Zero(DIM_U, DIM_U);
  Q(0, 0) = mpc_param_.weight_lat_error;
  Q(1, 1) = mpc_param_.weight_heading_error;
  R(0, 0) = mpc_param_.weight_steering_input;

  matrix Ad(DIM_X, DIM_X);
  matrix Bd(DIM_X, DIM_U);
  matrix Wd(DIM_X, 1);
  matrix Cd(DIM_Y, DIM_X);
  matrix Uref(DIM_U, 0);

  MPCTrajectory debug_ref_vec; /* DEBUG: to calculate predicted trajectory */

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i)
  {
    // MPC_INFO("mpc time = %f\n",mpc_time );
    MPC_T[i] = mpc_time;

    /* get reference information at mpc_time by interpolation */
    double ref_vx; /* reference velocity at current mpc time */
    double ref_k;  /* reference curvature at current mpc time */
    if (!interp1d(ref_traj_.relative_time, ref_traj_.vx, mpc_time, ref_vx) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.k, mpc_time, ref_k))
    {
      ROS_ERROR("invalid interpolatio for ref_vx, ref_k\n");
      return false;
    }

    /* DEBUG:  to predict trajectory */
    double debug_ref_x_tmp, debug_ref_y_tmp, debug_ref_z_tmp, debug_ref_yaw_tmp;
    if (!interp1d(ref_traj_.relative_time, ref_traj_.x, mpc_time, debug_ref_x_tmp) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.y, mpc_time, debug_ref_y_tmp) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.z, mpc_time, debug_ref_z_tmp) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.yaw, mpc_time, debug_ref_yaw_tmp))
    {
      ROS_ERROR("invalid interpolation for ref_x, y, z, yaw\n");
      return false;
    }
    debug_ref_vec.push_back(debug_ref_x_tmp, debug_ref_y_tmp, debug_ref_z_tmp, debug_ref_yaw_tmp, 0, 0, 0);

    /* get discrete state matrix A, B, C, W */
    vehicle_model_.calculateDiscreteMatrix(Ad, Bd, Cd, Wd, mpc_param_.dt, ref_k, ref_vx);

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
    vehicle_model_.calculateReferenceInput(Uref, ref_k);
    Urefex.block(i * DIM_U, 0, DIM_U, 0) = Uref;
    // MPC_INFO("i = %d, ref_k = %f, uref = %f\n", i, ref_k, Urefex(i*DIM_U, 0));

    /* update mpc time */
    mpc_time += mpc_param_.dt;
  }

  /////////////// optimization ///////////////
  
  /*
   * solve quadratic optimization.
   * cost function: Uex' * H * Uex + f' * Uex
   */
  matrix H = (Cex * Bex).transpose() * Qex * Cex * Bex + Rex;
  matrix f = (Cex * (Aex * x0 + Wex)).transpose() * Qex * Cex * Bex - Urefex.transpose() * Rex;
  vector Uex = -H.inverse() * f.transpose(); /* least square */
  // PRINT_MAT(-H.inverse() * f.transpose());

  /* time delay compensation, look ahead delay_compensation_time for optimized input vector*/
  double u_delay_comped;
  double total_delay = ros::Time::now().toSec() - vehicle_status_.time;
  MPC_INFO("[calculateMPC] total delay time = %f [s]\n", total_delay);
  if (!interp1d(MPC_T, Uex, nearest_traj_time + mpc_param_.delay_compensation_time, u_delay_comped))
  {
    ROS_ERROR("invalid interpolation for u_delay\n");
  }
  MPC_INFO("[calculateMPC] mpc steering angle command = %f [deg] (no delay comp = %f)\n", u_delay_comped * rad2deg, Uex(0) * rad2deg);

  /* saturation */
  double u_sat = std::max(std::min(u_delay_comped, steer_cmd_lim_), -steer_cmd_lim_);

  /* filtering */
  double u_filtered = lpf_steering_cmd_.filter(u_sat);

  /* set steering command */
  steer_cmd = u_filtered;


  /////////////// velocity control ////////////////
  /* For simplicity, now we calculate steer and speed separately */
  double lookahead_time = 1.0;                                  // [s]
  double cmd_vel_ref_time = nearest_traj_time + lookahead_time; // get ahead reference velocity
  if (!interp1d(ref_traj_.relative_time, ref_traj_.vx, cmd_vel_ref_time, vel_cmd))
  {
    ROS_ERROR("invalid interpolation for vel_cmd\n");
  }
  MPC_INFO("[calculateMPC] velocitycommand = %f [m/s]\n", vel_cmd);


  ////////////////// DEBUG ///////////////////

  /* calculate predicted trajectory */
  vector Xex = Aex * x0 + Bex * Uex + Wex;

  /* calcuate trajectory from reference trajectory and tracking error */
  MPCTrajectory debug_mpc_predicted_traj;
  for (int i = 0; i < N; ++i)
  {
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
  debug_values.data.push_back(estimated_curvature_);
  debug_values.data.push_back(intoSemicircle(my_yaw));
  debug_values.data.push_back(intoSemicircle(sp_yaw));
  pub_debug_values_.publish(debug_values);

  return true;
};

void MPCFollower::calculateNearestPose(unsigned int &nearest_index,
                                       double &min_dist_error,
                                       double &nearest_yaw_error,
                                       const double &my_x, const double &my_y,
                                       const double &my_yaw)
{
  nearest_index = 0;
  nearest_yaw_error = std::numeric_limits<double>::max();
  double min_dist_squared = std::numeric_limits<double>::max();
  for (uint i = 0; i < ref_traj_.size(); ++i)
  {
    const double dx = my_x - ref_traj_.x[i];
    const double dy = my_y - ref_traj_.y[i];
    const double dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const double err_yaw = intoSemicircle(my_yaw - ref_traj_.yaw[i]);
    if (fabs(err_yaw) < (M_PI / 2.0))
    {

      if (dist_squared < min_dist_squared)
      {
        /* save nearest index */
        min_dist_squared = dist_squared;
        nearest_yaw_error = err_yaw;
        nearest_index = i;
      }
    }
  }

  min_dist_error = std::sqrt(min_dist_squared);
};

void MPCFollower::calcNearestPoseInterp(
    unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_x, double &nearest_y,
    double &nearest_yaw, double &nearest_time, const double &my_x, const double &my_y, const double &my_yaw)
{

  if (ref_traj_.size() == 0)
  {
    ROS_WARN("[calcNearestPoseInterp] trajectory size is zero");
    return;
  }

  nearest_index = 0;
  double min_dist_squared = std::numeric_limits<double>::max();
  for (uint i = 0; i < ref_traj_.size(); ++i)
  {
    const double dx = my_x - ref_traj_.x[i];
    const double dy = my_y - ref_traj_.y[i];
    const double dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const double err_yaw = intoSemicircle(my_yaw - ref_traj_.yaw[i]);
    if (fabs(err_yaw) < (M_PI / 2.0))
    {
      if (dist_squared < min_dist_squared)
      {
        /* save nearest index */
        min_dist_squared = dist_squared;
        nearest_index = i;
      }
    }
  }

  if (ref_traj_.size() == 1)
  {
    nearest_x = ref_traj_.x[nearest_index];
    nearest_y = ref_traj_.y[nearest_index];
    nearest_yaw = ref_traj_.yaw[nearest_index];
    nearest_time = ref_traj_.relative_time[nearest_index];
    min_dist_error = std::sqrt(min_dist_squared);
    nearest_yaw_error = intoSemicircle(my_yaw - ref_traj_.yaw[nearest_index]);
    return;
  }

  /* get second nearest index, next to nearest_index */
  int second_nearest_index = 0;
  if (nearest_index == ref_traj_.size() - 1)
    second_nearest_index = nearest_index - 1;
  else if (nearest_index == 0)
    second_nearest_index = 1;
  else
  {
    double dx1, dy1, dist_squared1, dx2, dy2, dist_squared2;
    dx1 = my_x - ref_traj_.x[nearest_index + 1];
    dy1 = my_y - ref_traj_.y[nearest_index + 1];
    dist_squared1 = dx1 * dx1 + dy1 * dy1;
    dx2 = my_x - ref_traj_.x[nearest_index - 1];
    dy2 = my_y - ref_traj_.y[nearest_index - 1];
    dist_squared2 = dx2 * dx2 + dy2 * dy2;
    if (dist_squared1 < dist_squared2)
      second_nearest_index = nearest_index + 1;
    else
      second_nearest_index = nearest_index - 1;
  }

  const double a_sq = min_dist_squared;

  /* distance between my position and second nearest position */
  const double dx2 = my_x - ref_traj_.x[second_nearest_index];
  const double dy2 = my_y - ref_traj_.y[second_nearest_index];
  const double b_sq = dx2 * dx2 + dy2 * dy2;

  /* distance between first and second nearest position */
  const double dx3 = ref_traj_.x[nearest_index] - ref_traj_.x[second_nearest_index];
  const double dy3 = ref_traj_.y[nearest_index] - ref_traj_.y[second_nearest_index];
  const double c_sq = dx3 * dx3 + dy3 * dy3;

  /* if distance between two points are too close */
  if (c_sq < 1.0E-5)
  {
    nearest_x = ref_traj_.x[nearest_index];
    nearest_y = ref_traj_.y[nearest_index];
    nearest_yaw = ref_traj_.yaw[nearest_index];
    nearest_time = ref_traj_.relative_time[nearest_index];
    min_dist_error = std::sqrt(min_dist_squared);
    nearest_yaw_error = intoSemicircle(my_yaw - ref_traj_.yaw[nearest_index]);
    return;
  }

  /* linear interpolation */
  const double alpha = 0.5 * (c_sq - a_sq + b_sq) / c_sq;
  nearest_x = alpha * ref_traj_.x[nearest_index] + (1 - alpha) * ref_traj_.x[second_nearest_index];
  nearest_y = alpha * ref_traj_.y[nearest_index] + (1 - alpha) * ref_traj_.y[second_nearest_index];
  nearest_yaw = alpha * ref_traj_.yaw[nearest_index] + (1 - alpha) * ref_traj_.yaw[second_nearest_index];
  nearest_time = alpha * ref_traj_.relative_time[nearest_index] + (1 - alpha) * ref_traj_.relative_time[second_nearest_index];
  min_dist_error = std::sqrt(b_sq - c_sq * alpha * alpha);
  nearest_yaw_error = intoSemicircle(my_yaw - nearest_yaw);
  return;
}

void MPCFollower::publishControlCommands(const double &vel_cmd, const double &steer_cmd)
{
  switch (ctrl_cmd_interface_)
  {
  case CtrlCmdInterface::TWIST:
    publishAsTwist(vel_cmd, steer_cmd);
    break;
  case CtrlCmdInterface::STEER:
    // TODO: write me
    ROS_WARN("control command interface STEER is not implemented");
    break;
  case CtrlCmdInterface::STEER_AND_VEL:
    publishSteerAndVel(vel_cmd, steer_cmd);
    break;
  case CtrlCmdInterface::ALL:
    publishAsTwist(vel_cmd, steer_cmd);
    publishSteerAndVel(vel_cmd, steer_cmd);
    break;
  default:
    ROS_WARN("control command interface is not appropriate");
    break;
  }
}

void MPCFollower::publishAsTwist(const double &vel_cmd, const double &steer_cmd)
{
  /* convert steering to twist */
  double omega = vel_cmd * std::tan(steer_cmd) / wheelbase_;

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

void MPCFollower::publishSteerAndVel(const double &vel_cmd, const double &steer_cmd)
{
  autoware_msgs::ControlCommandStamped cmd;
  cmd.header.frame_id = "/base_link";
  cmd.header.stamp = ros::Time::now();
  cmd.cmd.linear_velocity = vel_cmd;
  cmd.cmd.linear_acceleration = (vel_cmd - vehicle_status_.vx) / ctrl_dt_;
  cmd.cmd.steering_angle = steer_cmd;
  pub_steer_vel_ctrl_cmd_.publish(cmd);
}

// double MPCFollower::convertHandleToTireAngle(double &handle_angle_rad, double &vel) {
//   const double a0 = 15.713;
//   const double a1 = 0.053;
//   const double a2 = -0.042;
//   const double N = a0 + a1 * vel * vel + a2 * handle_angle_rad;
//   return handle_angle_rad / N;
// };
// double MPCFollower::convertTireToHandleAngle(double &tire_angle_rad, double &vel) {
//   const double a0 = 15.713;
//   const double a1 = 0.053;
//   const double a2 = -0.042;
//   return tire_angle_rad * (a0 + a1 * vel * vel) / (1.0 - a2 * tire_angle_rad);
// };

void MPCFollower::callbackRefPath(const autoware_msgs::Lane::ConstPtr &msg)
{
  const auto start = std::chrono::system_clock::now();

  current_waypoints_ = *msg;
  path_frame_id_ = msg->header.frame_id;
  MPC_INFO("[path callback] received path size = %lu\n", current_waypoints_.waypoints.size());

  MPCTrajectory traj;

  /* calculate relative time */
  std::vector<double> relative_time;
  MPCFollower::calcRelativeTimeForPath(current_waypoints_, relative_time);
  MPC_INFO("[path callback] relative_time.size() = %lu, front() = %f, back() = %f\n",
           relative_time.size(), relative_time.front(), relative_time.back());

  /* resampling */
  MPCFollower::resamplePathToTrajByDistance(current_waypoints_, relative_time, traj_resample_dl_, traj);
  convertEulerAngleToMonotonic(traj.yaw);
  MPC_INFO("[path callback] resampled traj size() = %lu\n", traj.relative_time.size());

  /* path smoothing */
  if (use_path_smoothing_)
  {
    filteringMovingAverate(traj.x, path_smoothing_moving_ave_num_);
    filteringMovingAverate(traj.y, path_smoothing_moving_ave_num_);
    filteringMovingAverate(traj.z, path_smoothing_moving_ave_num_);
    filteringMovingAverate(traj.yaw, path_smoothing_moving_ave_num_);
    filteringMovingAverate(traj.k, path_smoothing_moving_ave_num_);
  }

  /* calculate curvature */
  MPCFollower::calcTrajectoryCurvature(traj);
  MPC_INFO("[path callback] trajectory curvature : max_k = %f, min_k = %f\n",
           *max_element(traj.k.begin(), traj.k.end()),
           *min_element(traj.k.begin(), traj.k.end()));

  /* add end point with vel=0 on traj for mpc prediction */
  const double mpc_predict_time_length = mpc_param_.n * mpc_param_.dt;
  const double end_velocity = 0.0;
  traj.vx.back() = 0.0; // for end point
  traj.push_back(traj.x.back(), traj.y.back(), traj.z.back(), traj.yaw.back(),
                 end_velocity, traj.k.back(), traj.relative_time.back() + mpc_predict_time_length);

  if (!traj.size())
  {
    ROS_ERROR("[callbackRefPath] trajectory size is undesired.\n");
    MPC_INFO("size: x=%lu, y=%lu, z=%lu, yaw=%lu, v=%lu,k=%lu,t=%lu\n", 
             traj.x.size(), traj.y.size(), traj.z.size(), traj.yaw.size(), traj.vx.size(), traj.k.size(), traj.relative_time.size());
  }

  ref_traj_ = traj;

  const auto end = std::chrono::system_clock::now();
  const double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  MPC_INFO("[path callback] path modification total time = %f [ms]\n", elapsed * 1.0e-6);

  /* publish trajectory for visualize */
  visualization_msgs::Marker markers;
  convertTrajToMarker(ref_traj_, markers, "ref_traj", 0.0, 0.0, 1.0);
  pub_debug_filtered_traj_.publish(markers);
};

void MPCFollower::calcRelativeTimeForPath(const autoware_msgs::Lane &path,
                                          std::vector<double> &path_time)
{
  double t = 0.0;
  path_time.clear();
  path_time.push_back(t);
  for (int i = 0; i < (int)path.waypoints.size() - 1; ++i)
  {
    const double x0 = path.waypoints.at(i).pose.pose.position.x;
    const double y0 = path.waypoints.at(i).pose.pose.position.y;
    const double z0 = path.waypoints.at(i).pose.pose.position.z;
    const double x1 = path.waypoints.at(i + 1).pose.pose.position.x;
    const double y1 = path.waypoints.at(i + 1).pose.pose.position.y;
    const double z1 = path.waypoints.at(i + 1).pose.pose.position.z;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;
    const double dist = sqrt(dx * dx + dy * dy + dz * dz);
    double v = std::max(std::fabs(path.waypoints.at(i).twist.twist.linear.x), 1.0);
    t += (dist / v);
    path_time.push_back(t);
  }
}

void MPCFollower::resamplePathToTrajByTime(const autoware_msgs::Lane &path, const std::vector<double> &time,
                                           const double &dt, MPCTrajectory &ref_traj_)
{

  ref_traj_.clear();
  double t = 0.0;

  while (t < time.back())
  {
    uint j = 1;
    while (t > time.at(j))
    {
      ++j;
      if (j > time.size() - 1)
      {
        ROS_ERROR("[resamplePathToTraj] sampling time is not monotonically increasing");
        printf("t = %f, time.at(j-1)=%f\n", t, time.at(j - 1));
        return;
      }
    }

    const double a = t - time.at(j - 1);
    const double path_dt_j = time.at(j) - time.at(j - 1);
    const geometry_msgs::Pose pos0 = path.waypoints.at(j - 1).pose.pose;
    const geometry_msgs::Pose pos1 = path.waypoints.at(j).pose.pose;
    const geometry_msgs::Twist twist0 = path.waypoints.at(j - 1).twist.twist;
    const geometry_msgs::Twist twist1 = path.waypoints.at(j).twist.twist;
    const double x = ((path_dt_j - a) * pos0.position.x + a * pos1.position.x) / path_dt_j;
    const double y = ((path_dt_j - a) * pos0.position.y + a * pos1.position.y) / path_dt_j;
    const double z = ((path_dt_j - a) * pos0.position.z + a * pos1.position.z) / path_dt_j;

    /* for singular point of euler angle */
    const double yaw0 = tf2::getYaw(pos0.orientation);
    const double dyaw = intoSemicircle(tf2::getYaw(pos1.orientation) - yaw0);
    const double yaw1 = yaw0 + dyaw;
    const double yaw = ((path_dt_j - a) * yaw0 + a * yaw1) / path_dt_j;
    const double vx = ((path_dt_j - a) * twist0.linear.x + a * twist1.linear.x) / path_dt_j;
    const double curvature_tmp = 0.0;
    ref_traj_.push_back(x, y, z, yaw, vx, curvature_tmp, t);
    t += dt;
  }
}

void MPCFollower::resamplePathToTrajByDistance(const autoware_msgs::Lane &path, const std::vector<double> &time,
                                               const double &dl, MPCTrajectory &ref_traj_)
{

  ref_traj_.clear();
  double t = 0.0;
  double dist = 0.0;
  std::vector<double> dists;
  dists.push_back(0.0);

  for (int i = 1; i < time.size(); ++i)
  {
    double dx = path.waypoints.at(i).pose.pose.position.x - path.waypoints.at(i - 1).pose.pose.position.x;
    double dy = path.waypoints.at(i).pose.pose.position.y - path.waypoints.at(i - 1).pose.pose.position.y;
    dist += sqrt(dx * dx + dy * dy);
    dists.push_back(dist);
  }

  double l = 0.0;
  while (l < dists.back())
  {
    uint j = 1;
    while (l > dists.at(j))
    {
      ++j;
      if (j > dists.size() - 1)
      {
        ROS_ERROR("[resamplePathToTraj] sampling time is not monotonically increasing");
        printf("l = %f, dists.at(j-1)=%f\n", l, dists.at(j - 1));
        return;
      }
    }

    const double a = l - dists.at(j - 1);
    const double path_dl_j = dists.at(j) - dists.at(j - 1);
    const geometry_msgs::Pose pos0 = path.waypoints.at(j - 1).pose.pose;
    const geometry_msgs::Pose pos1 = path.waypoints.at(j).pose.pose;
    const geometry_msgs::Twist twist0 = path.waypoints.at(j - 1).twist.twist;
    const geometry_msgs::Twist twist1 = path.waypoints.at(j).twist.twist;
    const double x = ((path_dl_j - a) * pos0.position.x + a * pos1.position.x) / path_dl_j;
    const double y = ((path_dl_j - a) * pos0.position.y + a * pos1.position.y) / path_dl_j;
    const double z = ((path_dl_j - a) * pos0.position.z + a * pos1.position.z) / path_dl_j;

    /* for singular point of euler angle */
    const double yaw0 = tf2::getYaw(pos0.orientation);
    const double dyaw = intoSemicircle(tf2::getYaw(pos1.orientation) - yaw0);
    const double yaw1 = yaw0 + dyaw;
    const double yaw = ((path_dl_j - a) * yaw0 + a * yaw1) / path_dl_j;
    const double vx = ((path_dl_j - a) * twist0.linear.x + a * twist1.linear.x) / path_dl_j;
    const double curvature_tmp = 0.0;
    const double t = ((path_dl_j - a) * time.at(j - 1) + a * time.at(j)) / path_dl_j;
    ref_traj_.push_back(x, y, z, yaw, vx, curvature_tmp, t);
    l += dl;
  }
}

void MPCFollower::calcTrajectoryCurvature(MPCTrajectory &traj)
{
  traj.k.clear();

  auto dist = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
    return std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
  };
  // MPC_INFO("k = ");

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::Point p1, p2, p3;
  for (uint i = curvature_smoothing_num_; i < traj.x.size() - curvature_smoothing_num_; ++i)
  {
    p1.x = traj.x[i - curvature_smoothing_num_];
    p2.x = traj.x[i];
    p3.x = traj.x[i + curvature_smoothing_num_];
    p1.y = traj.y[i - curvature_smoothing_num_];
    p2.y = traj.y[i];
    p3.y = traj.y[i + curvature_smoothing_num_];
    const double curvature =
        2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) /
        (dist(p1, p2) * dist(p2, p3) * dist(p3, p1));
    traj.k.push_back(curvature);
    // MPC_INFO("curvature = %f, ", curvature);
    // printf("x1 = %f, x2 = %f, x3 = %f, y1 = %f, y2 = %f, y3 = %f, ", p1.x, p2.x, p3.x, p1.y, p2.y, p3.y);
    // printf("(p2.x - p1.x) = %f, (p3.y - p1.y) = %f, (p2.y - p1.y) = %f, (p3.x - p1.x) = %f\n", (p2.x - p1.x) , (p3.y - p1.y), (p2.y - p1.y), (p3.x - p1.x));
  }

  /* first and last curvature is copied from next value */
  for (int i = 0; i < curvature_smoothing_num_; ++i)
  {
    traj.k.insert(traj.k.begin(), traj.k.front());
    traj.k.push_back(traj.k.back());
  }
}

void MPCFollower::convertTrajToMarker(const MPCTrajectory &traj, visualization_msgs::Marker &marker, std::string ns, double r, double g, double b)
{
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
  for (uint i = 0; i < traj.x.size(); ++i)
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
  vehicle_status_.frame_id_pos = msg->header.frame_id;
  vehicle_status_.posx = msg->pose.position.x;
  vehicle_status_.posy = msg->pose.position.y;
  vehicle_status_.posz = msg->pose.position.z;
  vehicle_status_.yaw = tf2::getYaw(msg->pose.orientation);
  vehicle_status_.time = msg->header.stamp.toSec();
  my_position_ok_ = true;
};

void MPCFollower::callbackVelocity(const pacmod_msgs::WheelSpeedRpt &msg)
{
  static const double tire_radius = 0.77 / 2.0;
  static const double wheel_speed_rads_to_ms = tire_radius;
  vehicle_status_.vx = (msg.rear_left_wheel_speed + msg.rear_right_wheel_speed) / 2.0 * wheel_speed_rads_to_ms;
  my_velocity_ok_ = true;
};

void MPCFollower::callbackSteering(const pacmod_msgs::SystemRptFloat &msg)
{
  vehicle_status_.steer_rad = msg.output;
  my_steering_ok_ = true;
};

void MPCFollower::callbackVelocityGazebo(const std_msgs::Float64 &msg)
{
  vehicle_status_.vx = msg.data;
  my_velocity_ok_ = true;
};

void MPCFollower::callbackSteeringGazebo(const std_msgs::Float64 &msg)
{
  vehicle_status_.steer_rad = msg.data;
  my_steering_ok_ = true;
};

void MPCFollower::callbackEstimateTwist(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  estimate_twist_ = *msg;
  if (fabs(msg->twist.linear.x) < 0.01)
    estimated_curvature_ = 0.0;
  else
    estimated_curvature_ = msg->twist.angular.z / msg->twist.linear.x;
}

MPCFollower::~MPCFollower(){};

int main(int argc, char **argv)
{

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "mpc_follower");
  MPCFollower obj;

  ros::spin();

  return 0;
};
