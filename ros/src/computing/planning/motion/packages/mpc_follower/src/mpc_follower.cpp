#include <vector>
#include <iostream>
#include <limits>


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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

using vector = std::vector<double>;
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl


class MPCFollower {
public:
  MPCFollower();
  ~MPCFollower();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_steering_, pub_twist_cmd_, pub_debug_filtered_traj_, pub_debug_predicted_traj_;
  ros::Subscriber sub_ref_path_, sub_twist_, sub_pose_, sub_steering_;
  ros::Timer timer_control_;
  std::string path_frame_id_;


  /* algorithm */
  geometry_msgs::TwistStamped current_twist_;
  geometry_msgs::PoseStamped current_pose_;
  double current_steer_;
  autoware_msgs::Lane current_ref_path_;
  class Trajectory {
  public:
    vector x;
    vector y;
    vector z;
    vector yaw;
    vector vx;
    vector k;
    vector relative_time;
    void push_back(const double &xp, const double &yp, const double &zp,
                   const double &yawp, const double &vxp, const double &kp, const double &tp) {
      x.push_back(xp);
      y.push_back(yp);
      z.push_back(zp);
      yaw.push_back(yawp);
      vx.push_back(vxp);
      k.push_back(kp);
      relative_time.push_back(tp);
    };
    void clear() {
      x.clear();
      y.clear();
      z.clear();
      yaw.clear();
      vx.clear();
      k.clear();
      relative_time.clear();
    };
    uint size() {
      uint a = x.size();
      if (a == y.size() && a == z.size() && a == yaw.size() &&
          a == vx.size() && a == k.size() && a == relative_time.size()) {
        return a;
      } else {
        printf("trajectory size is inappropriate\n");
        return 0;
      }
    }
  };
  MPCFollower::Trajectory ref_traj_;
  Butter2D path_filter_;

  /* parameters */
  double traj_dt_;
  double ctrl_dt_;
  double path_lowpass_cutoff_hz_;

  int dim_x_;
  int dim_u_;
  int dim_y_;

  int mpc_n_;
  double mpc_dt_;
  double mpc_steer_tau_;
  double mpc_wheelbase_;


  /* flags */
  bool use_path_smoothing_;

  void timerCallback(const ros::TimerEvent&);

  void callbackRefPath(const autoware_msgs::Lane::ConstPtr&);
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr&);
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr&);
  void callbackSteering(const std_msgs::Float64 &);


  void calcRelativeTimeForPath(const autoware_msgs::Lane&, std::vector<double> &);
  void resamplePath2Traj(const autoware_msgs::Lane &path, const vector &time, const double &resample_dt, MPCFollower::Trajectory &ref_traj_);
  void convertTraj2Marker(const Trajectory &traj, visualization_msgs::Marker &markers, std::string ns, double r, double g, double b);
  void calcTrajectoryCurvature(Trajectory &traj);

  bool generateMPCMatrix(Eigen::MatrixXd &x0, Eigen::MatrixXd &Aex,
                         Eigen::MatrixXd &Bex, Eigen::MatrixXd &Wex,
                         Eigen::MatrixXd &Cex, Eigen::MatrixXd &Qex,
                         Eigen::MatrixXd &Rex);
  void getErrorDynamicsStateMatrix(const double &mpc_dt_, const double &ref_vx,
                                   const double &mpc_wheelbase_,
                                   const double &mpc_steer_tau_,
                                   const double &ref_k, Eigen::MatrixXd &Ad,
                                   Eigen::MatrixXd &Bd, Eigen::MatrixXd &Wd,
                                   Eigen::MatrixXd &Cd);
};

MPCFollower::MPCFollower()
    : nh_(""), pnh_("~"), path_frame_id_(""), dim_x_(3), dim_u_(1), dim_y_(2) {

  pnh_.param("use_path_smoothing", use_path_smoothing_, bool(true));
  pnh_.param("traj_dt", traj_dt_, double(0.1));
  pnh_.param("ctrl_dt", ctrl_dt_, double(0.1));
  pnh_.param("path_lowpass_cutoff_hz", path_lowpass_cutoff_hz_, double(2));
  mpc_steer_tau_ = 0.2;
  mpc_wheelbase_ = 2.9;
  current_steer_ = 0.0;

  // mpc parameters
  pnh_.param("mpc_n", mpc_n_, int(30));
  pnh_.param("mpc_dt", mpc_dt_, double(0.1));

  timer_control_ = nh_.createTimer(ros::Duration(ctrl_dt_), &MPCFollower::timerCallback, this);

  pub_steering_  = nh_.advertise<autoware_msgs::ControlCommandStamped>("/ctrl_cmd", 1);
  pub_twist_cmd_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 1);
  pub_debug_filtered_traj_ = pnh_.advertise<visualization_msgs::Marker>("debug/filtered_traj", 1);
  pub_debug_predicted_traj_ = pnh_.advertise<visualization_msgs::Marker>("debug/predicted_traj", 1);

  sub_ref_path_ = nh_.subscribe("/base_waypoints", 1, &MPCFollower::callbackRefPath, this);
  sub_twist_ = nh_.subscribe("/vehicle_info/twist", 1, &MPCFollower::callbackTwist, this);
  sub_pose_ = nh_.subscribe("/current_pose", 1, &MPCFollower::callbackPose, this);
  sub_steering_ = nh_.subscribe("/vehicle_info/steering_angle", 1, &MPCFollower::callbackSteering, this);

  path_filter_.initialize(traj_dt_, path_lowpass_cutoff_hz_);

};

void MPCFollower::timerCallback(const ros::TimerEvent& te) {

  uint nearest_index = 0;
  Eigen::MatrixXd x0(dim_x_,1);
  Eigen::MatrixXd Aex = Eigen::MatrixXd::Zero(dim_x_*mpc_n_, dim_x_);
  Eigen::MatrixXd Bex = Eigen::MatrixXd::Zero(dim_x_*mpc_n_, dim_u_*mpc_n_);
  Eigen::MatrixXd Wex = Eigen::MatrixXd::Zero(dim_x_*mpc_n_, 1);
  Eigen::MatrixXd Cex = Eigen::MatrixXd::Zero(dim_y_*mpc_n_, dim_x_*mpc_n_);
  Eigen::MatrixXd Qex = Eigen::MatrixXd::Zero(dim_y_*mpc_n_, dim_y_*mpc_n_);
  Eigen::MatrixXd Rex = Eigen::MatrixXd::Zero(dim_u_*mpc_n_, dim_u_*mpc_n_);

////TODO
  generateMPCMatrix(x0, Aex, Bex, Wex, Cex, Qex, Rex);

//////////////////////////////////////////////////////////////////////////////////

// calc predicted trajectory




};

bool MPCFollower::generateMPCMatrix(Eigen::MatrixXd &x0, Eigen::MatrixXd &Aex, Eigen::MatrixXd &Bex,
                                    Eigen::MatrixXd &Wex, Eigen::MatrixXd &Cex,
                                    Eigen::MatrixXd &Qex,
                                    Eigen::MatrixXd &Rex) {
  if (ref_traj_.size() == 0) {
    return false;
  }

  // calculate nearest point (used as initial state)
  uint nearest_index = 0;
  double min_dist_squared = std::numeric_limits<double>::max();
  printf("ref_traj_.size() = %d\n", ref_traj_.size());
  for (uint i = 0; i < ref_traj_.size(); ++i) {
    const double dx = current_pose_.pose.position.x - ref_traj_.x[i];
    const double dy = current_pose_.pose.position.y - ref_traj_.y[i];
    const double dist_squared = dx * dx + dy * dy;
    if (dist_squared < min_dist_squared) {

      // ignore when yaw error is large, for cross path
      double err_yaw = tf2::getYaw(current_pose_.pose.orientation) - ref_traj_.yaw[i];
      while (!(-2*M_PI <= err_yaw && err_yaw <= 2*M_PI)) {
          if (err_yaw >= 2*M_PI) {
              err_yaw = err_yaw - 2*M_PI;
          } else if (err_yaw <= -2*M_PI) {
              err_yaw = err_yaw + 2*M_PI;
          }
      }
      if (err_yaw > M_PI) {
          err_yaw = err_yaw - 2*M_PI;
      } else if (err_yaw < -M_PI) {
          err_yaw = err_yaw + 2*M_PI;
      }
      // dyaw = fmod(dyaw, 2.0 * M_PI);
      // dyaw -= 2.0 * M_PI * ((dyaw > M_PI) - (dyaw < -M_PI));
      if (fabs(err_yaw) < M_PI / 2.0) {

        // save nearest index
        min_dist_squared = dist_squared;
        nearest_index = i;
      }
    }
  }
  double distance = std::sqrt(min_dist_squared);
  printf("nearest_index = %d, min_dist_squared = %f\n",nearest_index, min_dist_squared);

  // convert x,y error to lat error
  const double err_x = current_pose_.pose.position.x - ref_traj_.x[nearest_index];
  const double err_y = current_pose_.pose.position.y - ref_traj_.y[nearest_index];
  const double sp_yaw = ref_traj_.yaw[nearest_index];
  const double err_lat = -sin(sp_yaw) * err_x + cos(sp_yaw) * err_y;


  // calculate yaw error, convert into range [-pi to pi]
  double my_yaw = tf2::getYaw(current_pose_.pose.orientation);
  double err_yaw = my_yaw - sp_yaw;
  double err_yaw2 = fmod(err_yaw, 2.0 * M_PI);
  err_yaw2 -= 2.0 * M_PI * ((err_yaw2 > M_PI) - (err_yaw2 < -M_PI));
  while (!(-2*M_PI <= err_yaw && err_yaw <= 2*M_PI)) {
      if (err_yaw >= 2*M_PI) {
          err_yaw = err_yaw - 2*M_PI;
      } else if (err_yaw <= -2*M_PI) {
          err_yaw = err_yaw + 2*M_PI;
      }
  }
  if (err_yaw > M_PI) {
      err_yaw = err_yaw - 2*M_PI;
  } else if (err_yaw < -M_PI) {
      err_yaw = err_yaw + 2*M_PI;
  }
  if (fabs(err_yaw - err_yaw2) > 0.001) {
    printf("\n\n\n\n\n err_yaw = %f\nerr_yaw2 = %f\n\n\n\n\n",err_yaw, err_yaw2);
  }

  // get steering angle
  double steer = current_steer_;

  // initial state for error dynamics
  x0 << err_lat, err_yaw, steer;

printf("lat error = %f, yaw error = %f, steer = %f, sp_yaw = %f, my_yaw = %f\n", err_lat, err_yaw, steer, sp_yaw, my_yaw);

  /////////////// generate mpc matrix  ///////////////

  // TODO
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(dim_y_, dim_y_);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(dim_u_, dim_u_);


  Eigen::MatrixXd Ad(dim_x_, dim_x_);
  Eigen::MatrixXd Bd(dim_x_, dim_u_);
  Eigen::MatrixXd Wd(dim_x_, 1);
  Eigen::MatrixXd Cd(dim_y_, dim_x_);

  Trajectory debug_ref_xyz_vec; // to calculate predicted trajectory

  const double nearest_traj_time = ref_traj_.relative_time[nearest_index];
  double mpc_time = nearest_traj_time; // as initialize;

  printf("ts = %f, tf = %f\n", ref_traj_.relative_time.front(), ref_traj_.relative_time.back());

  // predict dynamics for mpc_n_ times
  for (int i = 0; i < mpc_n_; ++i) {
    // printf("mpc time = %f\n",mpc_time );

    // check for out of range
    if (mpc_time > ref_traj_.relative_time.back()) {
      mpc_time = ref_traj_.relative_time.back();
      printf("[MPC] path is too short to predict dynamics. i = %d\n", i);
    }

    // get reference information at mpc_time by interpolation
    double ref_vx; // reference velocity at current mpc time
    double ref_k; // reference curvature at current mpc time
    if (!interp1d(ref_traj_.relative_time, ref_traj_.vx, mpc_time, ref_vx) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.k, mpc_time, ref_k)) {
      ROS_ERROR("invalid interpolation\n");
      return false;
    }

    // for debug, to predict trajectory
    double debug_ref_x_tmp, debug_ref_y_tmp, debug_ref_z_tmp, debug_ref_yaw_tmp;
    if (!interp1d(ref_traj_.relative_time, ref_traj_.x, mpc_time, debug_ref_x_tmp) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.y, mpc_time, debug_ref_y_tmp) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.z, mpc_time, debug_ref_z_tmp) ||
        !interp1d(ref_traj_.relative_time, ref_traj_.yaw, mpc_time, debug_ref_yaw_tmp)) {
      ROS_ERROR("invalid interpolation\n");
      return false;
    }
    // for (int i=0;i<ref_traj_.size(); ++i){
    //   printf("%f, ",ref_traj_.x[i]);
    // }
    debug_ref_xyz_vec.push_back(debug_ref_x_tmp, debug_ref_y_tmp, debug_ref_z_tmp, debug_ref_yaw_tmp, 0, 0, 0);
    // printf("\ndebug_ref_x_tmp = %f, debug_ref_y_tmp = %f, debug_ref_z_tmp = %f, debug_ref_yaw_tmp = %f\n", debug_ref_x_tmp, debug_ref_y_tmp, debug_ref_z_tmp, debug_ref_yaw_tmp);

    // get discrete state matrix
    getErrorDynamicsStateMatrix(mpc_dt_, ref_vx, mpc_wheelbase_, mpc_steer_tau_,
                                ref_k, Ad, Bd, Wd, Cd);
    // printf("i = %d\n",i);
    // PRINT_MAT(Ad);
    // PRINT_MAT(Bd);
    // PRINT_MAT(Wd);
    // PRINT_MAT(Cd);


    // update mpc matrix
    if (i == 0) {
      Aex.block(0, 0, dim_x_, dim_x_) = Ad;
      Bex.block(0, 0, dim_x_, dim_u_) = Bd;
      Wex.block(0, 0, dim_x_, 1) = Wd;
      Cex.block(0, 0, dim_y_, dim_x_) = Cd;
      Qex.block(0, 0, dim_y_, dim_y_) = Q;
      Rex.block(0, 0, dim_u_, dim_u_) = R;
    } else {
      int idx_x_i = i * dim_x_;
      int idx_x_i_prev = (i - 1) * dim_x_;
      int idx_u_i = i * dim_u_;
      int idx_y_i = i * dim_y_;
      Aex.block(idx_x_i, 0, dim_x_, dim_x_) =
          Ad * Aex.block(idx_x_i_prev, 0, dim_x_, dim_x_);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * dim_u_;
        Bex.block(idx_x_i, idx_u_j, dim_x_, dim_u_) =
            Ad * Bex.block(idx_x_i_prev, idx_u_j, dim_x_, dim_u_);
      }
      Bex.block(idx_x_i, idx_u_i, dim_x_, dim_u_) = Bd;
      Wex.block(idx_x_i, 0, dim_x_, 1) =
          Ad * Wex.block(idx_x_i_prev, 0, dim_x_, 1) + Wd;
      Cex.block(idx_y_i, idx_x_i, dim_y_, dim_x_) = Cd;
      Qex.block(idx_y_i, idx_y_i, dim_y_, dim_y_) = Q;
      Rex.block(idx_u_i, idx_u_i, dim_u_, dim_u_) = R;
    }

    // update mpc time
    mpc_time += mpc_dt_;
  }
// PRINT_MAT(Aex);
// PRINT_MAT(Bex);
// PRINT_MAT(Wex);
// PRINT_MAT(Cex);
// PRINT_MAT(Qex);
// PRINT_MAT(Rex);
// PRINT_MAT(x0);

  /////////////// optimization ///////////////
  Eigen::MatrixXd H = Bex.transpose() * Cex.transpose() * Qex * Cex * Bex + Rex;
  Eigen::MatrixXd f = (Cex * (Aex * x0 + Wex)).transpose() * Qex * Cex * Bex;
  Eigen::MatrixXd Uex = -H.inverse() * f.transpose();
  // PRINT_MAT(H);
  // PRINT_MAT(f);
  // PRINT_MAT(Uex);
  printf("mpc steering angle command = %f [deg]\n", Uex(0,0) * 180.0 / M_PI);

  ////////// calculate predicted trajectory //////////
  Eigen::MatrixXd Xex = Aex * x0 + Bex * Uex + Wex;
  // PRINT_MAT(Xex);

  Trajectory debug_mpc_predicted_traj;
  for (int i = 0; i < mpc_n_; ++i) {
    const double yaw_ref = debug_ref_xyz_vec.yaw[i];
    const double lat_error = Xex(i * dim_x_, 0);
    const double yaw_error = Xex(i * dim_x_ + 1, 0);
    const double x = debug_ref_xyz_vec.x[i] - std::sin(yaw_ref) * lat_error;
    const double y = debug_ref_xyz_vec.y[i] + std::cos(yaw_ref) * lat_error;
    const double z = debug_ref_xyz_vec.z[i];
    debug_mpc_predicted_traj.push_back(x, y, z, yaw_ref + yaw_error, 0, 0, 0);
  }

  visualization_msgs::Marker marker;
  convertTraj2Marker(debug_mpc_predicted_traj, marker, "predicted_traj", 1.0, 0.0, 0.0);
  pub_debug_predicted_traj_.publish(marker);




  ////// convert steering to twist //////
  auto convertSteerToOmega = [](double &steer, double &vel, double &wheelbase) {
    return vel * std::tan(steer) / wheelbase;
  };
  double deg2rad = M_PI / 180.0;
  double u_sat = std::max(std::min(Uex(0,0), 30. *deg2rad), -30.*deg2rad);
  double omega = convertSteerToOmega(u_sat, ref_traj_.vx[nearest_index], mpc_wheelbase_);
  double command_vel_time = nearest_traj_time + 1.0; // get ahead reference velocity
  double vel_ref;
  interp1d(ref_traj_.relative_time, ref_traj_.vx, command_vel_time, vel_ref);
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "/base_link";
  twist.header.stamp = ros::Time::now();
  twist.twist.linear.x = vel_ref + 2.0;
  twist.twist.linear.y = 0.0;
  twist.twist.linear.z = 0.0;
  twist.twist.angular.x = 0.0;
  twist.twist.angular.y = 0.0;
  twist.twist.angular.z = omega;
  pub_twist_cmd_.publish(twist);



};

void MPCFollower::getErrorDynamicsStateMatrix(
    const double &dt, const double &v, const double &wheelbase,
    const double &tau, const double &ref_k, Eigen::MatrixXd &Ad,
    Eigen::MatrixXd &Bd, Eigen::MatrixXd &Wd, Eigen::MatrixXd &Cd) {

  auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

  // linearization around delta = delta_ref
  double delta_r = atan(wheelbase * ref_k);
  if (abs(delta_r) >= 40.0 / 180.0 * M_PI)
    delta_r = (40.0 / 180.0 * M_PI) * (double)sign(delta_r);

  double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

  // difinition for continuous model
  Eigen::MatrixXd A(dim_x_, dim_x_);
  Eigen::MatrixXd B(dim_x_, dim_u_);
  Eigen::MatrixXd W(dim_x_, 1);
  Eigen::MatrixXd C(dim_y_, dim_x_);
  A << 0.0, v, 0.0,
       0.0, 0.0, v / wheelbase * cos_delta_r_squared_inv,
       0.0, 0.0, -1.0 / tau;

  B << 0.0, 0.0, 1.0 / tau;
  C << 1.0, 0.0, 0.0,
       0.0, 1.0, 0.0;
  W << 0.0,
      -v * ref_k + v / wheelbase * (tan(delta_r) - delta_r * cos_delta_r_squared_inv),
      0.0;

  // discretization
  // Ad = eye(3) + A * dt;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  Ad = (I - dt * 0.5 * A).inverse() *  (I + dt * 0.5 * A);
  Bd = B * dt;
  Cd = C;
  Wd = W * dt;
}



void MPCFollower::callbackRefPath(const autoware_msgs::Lane::ConstPtr& msg){
  current_ref_path_ = *msg;
  path_frame_id_ = msg->header.frame_id;
  printf("callbackRefPath start\n");
  printf("current_ref_path_.size() = %d\n",current_ref_path_.waypoints.size());

  Trajectory traj;

  // calculate relative time
  vector relative_time;
  MPCFollower::calcRelativeTimeForPath(current_ref_path_, relative_time);
  printf("relative_time.end() = %f\n", relative_time.back());
  printf("relative_time.size() = %d\n",relative_time.size());

  // resampling
  MPCFollower::resamplePath2Traj(current_ref_path_, relative_time, traj_dt_, traj);
  printf("traj.relative_time.size() = %d\n",traj.relative_time.size());

  // low pass filter
  ros::Time ts = ros::Time::now();
  if (use_path_smoothing_) {
    path_filter_.initialize(traj_dt_, path_lowpass_cutoff_hz_);
    path_filter_.filtfilt_vector(traj.relative_time, traj.x);
    path_filter_.filtfilt_vector(traj.relative_time, traj.y);
    path_filter_.filtfilt_vector(traj.relative_time, traj.z);
    path_filter_.filtfilt_vector(traj.relative_time, traj.yaw);
  }
  ros::Time tf = ros::Time::now();
  printf("path filtering time = %f [ms]\n", (tf-ts).toSec() * 1000.0);

  MPCFollower::calcTrajectoryCurvature(traj);
  double max_k = *max_element(traj.k.begin(), traj.k.end());
  double min_k = *min_element(traj.k.begin(), traj.k.end());
  printf("max_k = %f, min_k = %f\n", max_k, min_k);

  if (!traj.size()) {
    printf("[callbackRefPath] trajectory size is wrong\n");
    printf("size: x=%d, y=%d, z=%d, yaw=%d, v=%d,k=%d,t=%d\n",traj.x.size(), traj.y.size(), traj.z.size(), traj.yaw.size(), traj.vx.size(), traj.k.size(), traj.relative_time.size());
  }

  ref_traj_ = traj;

  // publish trajectory for visualize
  visualization_msgs::Marker markers;
  convertTraj2Marker(ref_traj_, markers, "ref_traj", 0.0, 0.0, 1.0);
  pub_debug_filtered_traj_.publish(markers);
};






void MPCFollower::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg){
  current_twist_ = *msg;
};

void MPCFollower::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose_ = *msg;
};

void MPCFollower::callbackSteering(const std_msgs::Float64 &msg){
  printf("callback steering !!!!! steer = %f\n", msg.data);

  current_steer_ = msg.data;
};



void MPCFollower::calcRelativeTimeForPath(const autoware_msgs::Lane &path,
                                          std::vector<double> &path_time) {
printf("calcRelativeTimeForPath start\n");
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
    // printf("i = %d, t = %f, dist = %f, v = %f, dx = %f, dy = %f, dz = %f\n",i,t,dist,v,dx,dy,dz);
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

void MPCFollower::resamplePath2Traj(const autoware_msgs::Lane &path, const vector &time, const double &dt, MPCFollower::Trajectory &ref_traj_){
  printf("resamplePath2Traj start\n");
  // relative time should be monotonicly incresing
  ref_traj_.clear();
  double t = 0.0;
  printf("time = [%f ,~ %f]\n", time.front(), time.back());

  while (t < time.back()) {
    uint j = 1;
    while (t > time.at(j)) {
      ++j;
      if (j > time.size() - 1){
        printf("\n\n\n[MPC resampleByDt] something wrong\n\n\n");
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
    const double yaw = (a * tf2::getYaw(pos0.orientation) +
                        (path_dt_j - a) * tf2::getYaw(pos1.orientation)) / path_dt_j;
    const double vx = ((path_dt_j - a) * twist0.linear.x + a * twist1.linear.x) / path_dt_j;
    const double curvature_tmp = 0.0;
    // printf("t = %f, j = %d, a = %f, dt = %f", t, j, a, path_dt_j);
    // printf("x0 = %f, x = %f, x1 = %f, ", pos0.position.x, x, pos1.position.x);
    // printf("y0 = %f, y = %f, y1 = %f, ", pos0.position.y, y, pos1.position.y);
    // printf("z0 = %f, z = %f, z1 = %f, ", pos0.position.z, z, pos1.position.z);
    // printf("yaw0 = %f, yaw = %f, yaw1 = %f\n", tf2::getYaw(pos0.orientation), yaw, tf2::getYaw(pos1.orientation));
    ref_traj_.push_back(x, y, z, yaw, vx, curvature_tmp, t);
    t += dt;
  }
  // printf("x = ");
  // for (int i = 0; i < ref_traj_.x.size(); ++i){
  //   printf("%f, ", ref_traj_.x[i]);
  // }
  // printf("\n");
  // printf("y = ");
  // for (int i = 0; i < ref_traj_.y.size(); ++i){
  //   printf("%f, ", ref_traj_.y[i]);
  // }
  // printf("\n");
  printf("resamplePath2Traj end\n");
}

void MPCFollower::calcTrajectoryCurvature(Trajectory &traj) {
  traj.k.clear();

  auto dist = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
    return std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
  };
// printf("k = ");
  geometry_msgs::Point p1, p2, p3;
  int smoothing_num = 3;
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
    // printf("%f, ", curvature);
  }

  // first and last curvature is copied from next value
  for (int i = 0; i < smoothing_num; ++i){
    traj.k.insert(traj.k.begin(), traj.k.front());
    traj.k.push_back(traj.k.back());
  }
}



void MPCFollower::convertTraj2Marker(const Trajectory &traj, visualization_msgs::Marker &marker, std::string ns, double r, double g, double b){
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




MPCFollower::~MPCFollower(){};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpc_follower");
  MPCFollower obj;

  ros::spin();

  return 0;
};
