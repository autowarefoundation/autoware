#include <deque>
#include <iostream>
#include <vector>
#include <chrono>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

#include <autoware_msgs/VehicleStatus.h>
#include <tf/transform_broadcaster.h>

#include "kalman_filter/kalman_filter.h"

#include <pacmod_msgs/SystemRptFloat.h>
#include <pacmod_msgs/WheelSpeedRpt.h>

#define PRINT_MAT(X) std::cout << #X << ":\n"    \
                               << X << std::endl \
                               << std::endl

#define EKF_DEBUG_VERBOSE
#ifdef EKF_DEBUG_VERBOSE
#define EKF_INFO(...)      \
  {                        \
    if (show_debug_info_)  \
    {                      \
      printf(__VA_ARGS__); \
    }                      \
  }
#else
#define EKF_INFO(...)
#endif

class KalmanFilterNode
{
public:
  KalmanFilterNode();
  ~KalmanFilterNode();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_pose_, pub_pose_cov_, pub_pose_array_, pub_debug_, pub_ndt_pose_;
  ros::Subscriber sub_ndt_pose_, sub_vehicle_status_, sub_imu_, sub1_, sub2_;
  ros::Timer timer_control_, timer_tf_;
  tf::TransformBroadcaster tf_br_;
  bool show_debug_info_;

  KalmanFilter kf_;

  bool initial_pose_received_;
  bool initial_twist_received_;

  /* parameters */
  double predict_frequency_; // kalman filter predict rate
  double predict_dt_;        // = 1 / predict_frequency_
  double tf_dt_;             // tf publish rate
  double wheelbase_;         // to convert steering angle to angular velocity
  double additional_delay_;  // compensated ndt delay time = (ndt.header.stamp - now) + additional_delay [s]
  int dim_x_;                // dimension of kalman state
  int extend_state_step_;    // for time delay compensation
  int dim_x_ex_;             // dimension of extended kalman state (dim_x_ * extended_state_step)

  /* process noise standard deviation for continuous model*/
  double stddev_proc_x_c_;       // for dot x
  double stddev_proc_y_c_;       // for dot y
  double stddev_proc_yaw_c_;     // for dot yaw
  double stddev_proc_wz_bias_c_; // for wz_bias
  double stddev_vx_c_;           // for vx

  /* process noise variance for discrete model */
  double cov_proc_x_d_;
  double cov_proc_y_d_;
  double cov_proc_yaw_d_;
  double cov_proc_wz_bias_d_;

  /* measurement noise for time uncertainty */
  double measure_time_uncertainty_; // added for measurement covariance

  enum class Idx {
    X = 0,
    Y,
    YAW,
    WZ_BIAS,
    YAW_BIAS,
  };

  /* for model prediction */
  autoware_msgs::VehicleStatus current_vehicle_status_;
  geometry_msgs::TwistStamped current_twist_;
  geometry_msgs::PoseStamped current_ndt_pose_;
  sensor_msgs::Imu current_imu_;
  geometry_msgs::PoseStamped current_kf_pose_;

  void timerCallback(const ros::TimerEvent &e);
  void timerTFCallback(const ros::TimerEvent &e);
  void callbackNDTPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void callbackVehicleStatus(const autoware_msgs::VehicleStatus &msg);
  void callbackIMU(const sensor_msgs::Imu &msg);
  void callbackTwist(const geometry_msgs::TwistStamped &msg);

  void initKalmanFilter();
  void predictKinematicsModel();
  void measurementUpdateNDTPose(const geometry_msgs::PoseStamped &ndt_pose);
  void measurementUpdateIMU(const sensor_msgs::Imu &msg);

  void publishEstimatedPose();

  void callback1(const pacmod_msgs::SystemRptFloat &msg);
  void callback2(const pacmod_msgs::WheelSpeedRpt &msg);
};

KalmanFilterNode::KalmanFilterNode()
    : nh_(""), pnh_("~"), initial_pose_received_(false), initial_twist_received_(false)
{

  pnh_.param("show_debug_info", show_debug_info_, bool(false));
  pnh_.param("predict_frequency", predict_frequency_, double(100.0));
  pnh_.param("tf_dt", tf_dt_, double(0.05));
  pnh_.param("wheelbase", wheelbase_, double(2.79));
  pnh_.param("additional_delay", additional_delay_, double(0.15));
  pnh_.param("extend_state_step", extend_state_step_, int(50));

  pnh_.param("stddev_proc_x_c", stddev_proc_x_c_, double(1.0));
  pnh_.param("stddev_proc_y_c", stddev_proc_y_c_, double(1.0));
  pnh_.param("stddev_proc_yaw_c", stddev_proc_yaw_c_, double(1.0));
  pnh_.param("stddev_proc_wz_bias_c", stddev_proc_wz_bias_c_, double(0.001));
  pnh_.param("stddev_vx_c", stddev_vx_c_, double(1.0));

  pnh_.param("measure_time_uncertainty", measure_time_uncertainty_, double(0.01));

  predict_dt_ = 1.0 / std::max(predict_frequency_, 0.1);
  cov_proc_x_d_ = stddev_proc_x_c_ * stddev_proc_x_c_ * predict_dt_ * predict_dt_;
  cov_proc_y_d_ = stddev_proc_y_c_ * stddev_proc_y_c_ * predict_dt_ * predict_dt_;
  cov_proc_yaw_d_ = stddev_proc_yaw_c_ * stddev_proc_yaw_c_ * predict_dt_ * predict_dt_;
  cov_proc_wz_bias_d_ = stddev_proc_wz_bias_c_ * stddev_proc_wz_bias_c_ * predict_dt_ * predict_dt_;

  timer_control_ = nh_.createTimer(ros::Duration(predict_dt_), &KalmanFilterNode::timerCallback, this);
  timer_tf_ = nh_.createTimer(ros::Duration(tf_dt_), &KalmanFilterNode::timerTFCallback, this);
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/kf_pose_old", 1);
  pub_pose_cov_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/kf_pose_with_covariance", 1);
  pub_ndt_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("/my_ndt_pose", 1);
  pub_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("/kalman_filter_pose_array", 1);
  pub_debug_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug", 1);
  sub_ndt_pose_ = nh_.subscribe("/ndt_pose", 1, &KalmanFilterNode::callbackNDTPose, this);
  sub_vehicle_status_ = nh_.subscribe("/vehicle_status", 1, &KalmanFilterNode::callbackVehicleStatus, this);
  sub_imu_ = nh_.subscribe("/imu_raw", 1, &KalmanFilterNode::callbackIMU, this);

  dim_x_ = 5; // x, y, yaw, omega_bias
  dim_x_ex_ = dim_x_ * extend_state_step_;

  initKalmanFilter();

  // sub1_ = nh_.subscribe("/pacmod/parsed_tx/steer_rpt", 1, &KalmanFilterNode::callback1, this);
  // sub2_ = nh_.subscribe("/pacmod/parsed_tx/wheel_speed_rpt", 1, &KalmanFilterNode::callback2, this);
};

KalmanFilterNode::~KalmanFilterNode(){};

/*
 * timerCallback
 */
void KalmanFilterNode::timerCallback(const ros::TimerEvent &e)
{

  /* check flags */
  if (!initial_pose_received_ || !initial_twist_received_)
  {
    ROS_WARN("initial info is not received. pose = %d, odom = %d",
             initial_pose_received_, initial_twist_received_);
    return;
  }

  /* predict model in kalman filter */
  auto start = std::chrono::system_clock::now();
  predictKinematicsModel();
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
  EKF_INFO("[kalman filter] predictEKF calculation time = %f [ms]\n", elapsed * 1.0e-6);

  publishEstimatedPose();
}

/*
 * timerTFCallback
 */
void KalmanFilterNode::timerTFCallback(const ros::TimerEvent &e)
{
  tf::Transform t;
  t.setOrigin(tf::Vector3(current_kf_pose_.pose.position.x,
                          current_kf_pose_.pose.position.y, current_kf_pose_.pose.position.z));
  t.setRotation(tf::Quaternion(current_kf_pose_.pose.orientation.x, current_kf_pose_.pose.orientation.y,
                               current_kf_pose_.pose.orientation.z, current_kf_pose_.pose.orientation.w));
  tf_br_.sendTransform(tf::StampedTransform(t, ros::Time::now(), current_kf_pose_.header.frame_id, "kf_pose"));
}

/*
 * callbackNDTPose
 */
void KalmanFilterNode::callbackNDTPose(
    const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  current_ndt_pose_ = *msg;
  initial_pose_received_ = true;

  auto start = std::chrono::system_clock::now();
  measurementUpdateNDTPose(current_ndt_pose_);
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
  EKF_INFO("[kalman filter] updateEKF calculation time = %f [ms]\n", elapsed * 1.0e-6);
};

/*
 * callbackVehicleStatus
 */
void KalmanFilterNode::callbackVehicleStatus(const autoware_msgs::VehicleStatus &msg)
{
  const double kmph2mps = 1000.0 / 3600.0;
  current_vehicle_status_ = msg;
  current_twist_.twist.linear.x = msg.speed * kmph2mps;
  current_twist_.twist.angular.z = current_twist_.twist.linear.x * tan(msg.angle) / wheelbase_;
  initial_twist_received_ = true;

  // // TEMP
  // current_twist_.twist.linear.x = msg.speed / 3.6;
  // current_twist_.twist.angular.z =
  //     (msg.speed / 3.6) * tan(msg.angle * 3.1415 / 180.0 / 17.85) / 2.95 / 1.09;
  // initial_twist_received_ = true;
};

/* temp */
  void KalmanFilterNode::callback1(const pacmod_msgs::SystemRptFloat &msg){
    double adaptive_gear_ratio_ = 15.713 + 0.053 * std::pow(current_twist_.twist.linear.x, 2) + 0.042 * msg.output;
    current_twist_.twist.angular.z = current_twist_.twist.linear.x * std::tan(msg.output / adaptive_gear_ratio_) / 2.79;
  };
  void KalmanFilterNode::callback2(const pacmod_msgs::WheelSpeedRpt &msg) {
    initial_twist_received_ = true;
    current_twist_.twist.linear.x = (msg.rear_left_wheel_speed + msg.rear_right_wheel_speed) * 0.39 / 2;
  };

/*
 * callbackTwist
 */
void KalmanFilterNode::callbackTwist(const geometry_msgs::TwistStamped &msg)
{
  // current_twist_ = msg;
  // initial_twist_received_ = true;
};

/*
 * callbackIMU
 */
void KalmanFilterNode::callbackIMU(const sensor_msgs::Imu &msg)
{
  current_imu_ = msg;
  measurementUpdateIMU(current_imu_);
}

/*
 * initKalmanFilter
 */
void KalmanFilterNode::initKalmanFilter()
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_ex_, dim_x_ex_) * 1.0E9;
  for (int i = 0; i + 3 < dim_x_ex_; i += dim_x_)
  {
    P(i + 3, i + 3) = 1.0E-6; // for omega bias
    P(i + 4, i + 4) = 1.0E-5; // for omega bias
  }
  kf_.init(X, P);
}

/*
 * predictKinematicsModel
 */
void KalmanFilterNode::predictKinematicsModel()
{
  /*  == Nonlinear model ==
   *
   * x_{k+1} = x_k + vx * cos(yaw_k + yaw_bias_k) * dt
   * y_{k+1} = y_k + vx * sin(yaw_k + yaw_bias_k) * dt
   * yaw_{k+1} = yaw_k + (wz + wz_bias_k) * dt
   * wz_bias_{k+1} = wz_bias_k
   * yaw_bias_{k+1} = yaw_bias_k
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, -vx*sin(yaw + yaw_bias_k)*dt, 0 , -vx*sin(yaw + yaw_bias_k)*dt]
   *     [ 0, 1, vx*cos(yaw + yaw_bias_k)*dt,  0,   vx*cos(yaw + yaw_bias_k)*dt]
   *     [ 0, 0, 1,                           dt,   0]
   *     [ 0, 0, 0,                           1,    0]
   *     [ 0, 0, 0,                           0,    1]
   */

  Eigen::MatrixXd X_curr(dim_x_ex_, 1); // curent state
  Eigen::MatrixXd X_next(dim_x_ex_, 1); // predicted state
  kf_.getX(X_curr);

  Eigen::MatrixXd P_curr, P_next;
  kf_.getP(P_curr);

  const int d_dim_x = dim_x_ex_ - dim_x_;
  const double vx = current_twist_.twist.linear.x;
  const double wz = current_twist_.twist.angular.z;
  const double yaw = X_curr(2);
  const double wz_bias = X_curr(3);
  const double yaw_bias = X_curr(4);
  const double dt = predict_dt_;

  EKF_INFO("X_curr = %f, %f, %f, %f, %f\n", X_curr(0), X_curr(1), X_curr(2), X_curr(3), X_curr(4));

  /* update for latest state */
  X_next(0) = X_curr(0) + vx * cos(yaw + yaw_bias) * dt; // dx = v * cos(yaw)
  X_next(1) = X_curr(1) + vx * sin(yaw + yaw_bias) * dt; // dy = v * sin(yaw)
  X_next(2) = X_curr(2) + (wz + wz_bias) * dt;           // dyaw = omega + omega_bias
  X_next(3) = wz_bias;                                   // d_omega_bias = 0;
  X_next(4) = yaw_bias;
  while (std::fabs(X_next(2)) > M_PI)
  {
    X_next(2) -= 2.0 * M_PI * ((X_next(2) > 0) - (X_next(2) < 0));
  }
  EKF_INFO("X_next = %f, %f, %f, %f, %f\n", X_next(0), X_next(1), X_next(2), X_next(3), X_next(4));

  /* slide states in the time direction */
  X_next.block(dim_x_, 0, d_dim_x, 1) = X_curr.block(0, 0, d_dim_x, 1);

  /* set A matrix for latest state */
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(0, 2) = -vx * sin(yaw + yaw_bias) * dt;
  A(1, 2) = vx * cos(yaw + yaw_bias) * dt;
  A(2, 3) = dt;
  A(0, 4) = -vx * sin(yaw + yaw_bias) * dt;
  A(1, 4) = vx * cos(yaw + yaw_bias) * dt;

  /* set covariance matrix Q for process noise 
      calc Q by velocity and yaw angle covariance :
        dx = Ax + J*w -> Q = J*w_cov*J' */
  Eigen::MatrixXd J(2, 2); // coeff of deviation of vx & yaw
  J << cos(yaw), -vx * sin(yaw),
      sin(yaw), vx * cos(yaw);
  Eigen::MatrixXd Q_vx_yaw = Eigen::MatrixXd::Zero(2, 2); // cov of vx and yaw
  Q_vx_yaw(0, 0) = stddev_vx_c_ * stddev_vx_c_ * dt * dt; // covariance of vx
  Q_vx_yaw(1, 1) = P_curr(2, 2) * dt * dt;                // covariance of yaw
  Eigen::MatrixXd Q_ex = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  Q_ex.block(0, 0, 2, 2) = J * Q_vx_yaw * J.transpose(); // for pos_x & pos_y
  Q_ex(2, 2) = cov_proc_yaw_d_;                          // for yaw
  Q_ex(3, 3) = cov_proc_wz_bias_d_;                      // for yaw bias
  Q_ex(4, 4) = 0.0001 * 0.0001;
#if 0

  /* update P directly (slow for large dimension) */

  Eigen::MatrixXd A_ex = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  A_ex.block(0, 0, dim_x_, dim_x_) = A;
  A_ex.block(dim_x_, 0, dim_d, dim_d) = Eigen::MatrixXd::Identity(dim_d, dim_d);
  kf_.predictEKF(X_next, A_ex, Q_ex);

#else

  /* update P with A matrix structure (fast) */
  P_next = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  P_next.block(0, 0, dim_x_, dim_x_) = A * P_curr.block(0, 0, dim_x_, dim_x_) * A.transpose();
  P_next.block(0, dim_x_, dim_x_, d_dim_x) = A * P_curr.block(0, 0, dim_x_, d_dim_x);
  P_next.block(dim_x_, 0, d_dim_x, dim_x_) = P_curr.block(0, 0, d_dim_x, dim_x_) * A.transpose();
  P_next.block(dim_x_, dim_x_, d_dim_x, d_dim_x) = P_curr.block(0, 0, d_dim_x, d_dim_x);
  P_next += Q_ex;
  kf_.predictXandP(X_next, P_next);

#endif
}

/*
 * measurementUpdateNDTPose
 */
void KalmanFilterNode::measurementUpdateNDTPose(const geometry_msgs::PoseStamped &ndt_pose)
{
  const int dim_y = 3; // pos_x, pos_y, yaw, depending on NDT output
  const ros::Time t_curr = ros::Time::now();

  /* calculate delay step */
  double delay_time = (t_curr - ndt_pose.header.stamp).toSec() + additional_delay_;
  if (delay_time < 0)
  {
    delay_time = 0;
    ROS_WARN("time stamp is inappropriate. delay = %f, now() = %f, ndt.stamp = %f",
             delay_time, t_curr.toSec(), ndt_pose.header.stamp.toSec());
  };

  int delay_step = std::roundf(delay_time / predict_dt_);
  if (delay_step > extend_state_step_ - 1)
  {
    delay_step = extend_state_step_ - 1;
    ROS_WARN("NDT pose delay time: %f[s] exceeds the allowable limit: extend_state_step * predict_dt_ = %f [s]",
             delay_time, extend_state_step_ * predict_dt_);
  }

  EKF_INFO("predict_dt_ = %f, delay_time = %f, delay_step = %d, extend_state_step_ = %d\n",
           predict_dt_, delay_time, delay_step, extend_state_step_);

  /* set measurement matrix */
  Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
  C_ex(0, dim_x_ * delay_step) = 1.0;     // for pos x
  C_ex(1, dim_x_ * delay_step + 1) = 1.0; // for pos y
  C_ex(2, dim_x_ * delay_step + 2) = 1.0; // for yaw

  /* set measurement noise covariancs : NOTE this should be set by NDT reliability */
  const double kf_yaw = kf_.getXelement((unsigned int)Idx::YAW);
  const double cov_pos_x = std::pow(measure_time_uncertainty_ * current_twist_.twist.linear.x * cos(kf_yaw), 2);
  const double cov_pos_y = std::pow(measure_time_uncertainty_ * current_twist_.twist.linear.x * sin(kf_yaw), 2);
  const double cov_yaw = std::pow(measure_time_uncertainty_ * current_twist_.twist.angular.z, 2);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  R(0, 0) = std::pow(0.05, 2) + cov_pos_x; // pos_x
  R(1, 1) = std::pow(0.05, 2) + cov_pos_y; // pos_y
  R(2, 2) = std::pow(0.035, 2) + cov_yaw;  // yaw
  // PRINT_MAT(R);

  const double yaw_curr = kf_.getXelement((unsigned int)(delay_step * dim_x_ + 2));
  double yaw = tf::getYaw(ndt_pose.pose.orientation);
  while (std::fabs(yaw - yaw_curr) > M_PI)
  {
    yaw -= 2.0 * M_PI * ((yaw > yaw_curr) - (yaw < yaw_curr));
  }

  /* measurement update */
  Eigen::MatrixXd y(dim_y, 1);
  y << ndt_pose.pose.position.x, ndt_pose.pose.position.y, yaw;

  kf_.update(y, C_ex, R);
}

/*
 * measurementUpdateIMU
 */
void KalmanFilterNode::measurementUpdateIMU(const sensor_msgs::Imu &msg) {}

/*
 * publishEstimatedPose
 */
void KalmanFilterNode::publishEstimatedPose()
{
  Eigen::MatrixXd X(dim_x_ex_, 1);
  Eigen::MatrixXd P(dim_x_ex_, dim_x_ex_);
  kf_.getX(X);
  kf_.getP(P);

  /* publish latest pose */
  geometry_msgs::PoseStamped kf_pose;
  kf_pose.header.stamp = ros::Time::now();
  kf_pose.header.frame_id = current_ndt_pose_.header.frame_id;
  kf_pose.pose.position.x = X(0);
  kf_pose.pose.position.y = X(1);
  kf_pose.pose.position.z = current_ndt_pose_.pose.position.z;
  kf_pose.pose.orientation = tf::createQuaternionMsgFromYaw(X(2, 0) + X(4, 0));
  pub_pose_.publish(kf_pose);

  geometry_msgs::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = ros::Time::now();
  pose_cov.header.frame_id = current_ndt_pose_.header.frame_id;
  pose_cov.pose.pose = kf_pose.pose;
  for (int i = 0; i < 36; ++i)
  {
    pose_cov.pose.covariance[i] = 0.0;
  }
  pose_cov.pose.covariance[0] = P(0, 0);  // x, x
  pose_cov.pose.covariance[1] = P(0, 1);  // x, y
  pose_cov.pose.covariance[5] = P(0, 2);  // x, yaw
  pose_cov.pose.covariance[6] = P(1, 0);  // y, x
  pose_cov.pose.covariance[7] = P(1, 1);  // y, y
  pose_cov.pose.covariance[11] = P(1, 2); // y, yaw
  pose_cov.pose.covariance[30] = P(2, 0); // yaw, x
  pose_cov.pose.covariance[31] = P(2, 1); // yaw, y
  pose_cov.pose.covariance[35] = P(2, 2); // yaw, yaw
  pub_pose_cov_.publish(pose_cov);
  current_kf_pose_.header = pose_cov.header;
  current_kf_pose_.pose = pose_cov.pose.pose;

  /* publish pose array */
  // geometry_msgs::PoseArray pose_array;
  // geometry_msgs::Pose pose;
  // for (unsigned int i = 0; i < dim_x_ex_; i += dim_x_) {
  //   pose.position.x = X(i, 0);
  //   pose.position.y = X(i + 1, 0);
  //   pose.position.z = current_ndt_pose_.pose.position.z;
  //   pose.orientation = tf::createQuaternionMsgFromYaw(X(i + 2, 0));
  //   pose_array.poses.push_back(pose);
  // }
  // pose_array.header.stamp = ros::Time::now();
  // pose_array.header.frame_id = current_ndt_pose_.header.frame_id;
  // pub_pose_array_.publish(pose_array);

  /* debug my ndt */
  geometry_msgs::PoseStamped p;
  p = current_ndt_pose_;
  p.header.stamp = ros::Time::now();
  pub_ndt_pose_.publish(p);

  /* debug publish */
  double RAD2DEG = 180.0 / 3.141592;
  std_msgs::Float64MultiArray msg;
  msg.data.push_back(current_twist_.twist.linear.x);                            // [0] vx
  msg.data.push_back(current_twist_.twist.angular.z);                           // [1] wz (omega)
  msg.data.push_back(X(2) * RAD2DEG);                                           // [2] yaw angle
  msg.data.push_back(tf::getYaw(current_ndt_pose_.pose.orientation) * RAD2DEG); // [3] NDT yaw angle
  msg.data.push_back(X(3) * RAD2DEG);                                           // [4] omega bias
  msg.data.push_back((current_twist_.twist.angular.z + X(3)));                  // [5] omega + omega_bias
  msg.data.push_back(X(4));                                                     // [6] yaw_bias
  pub_debug_.publish(msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "kalman_filter_old");
  KalmanFilterNode obj;

  ros::spin();

  return 0;
};
