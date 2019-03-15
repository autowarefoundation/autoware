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

#include "kalman_filter/kalman_filter_localizer.h"

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

#define DEBUG_INFO(...) { if (show_debug_info_) { ROS_INFO(__VA_ARGS__); } }

KalmanFilterNode::KalmanFilterNode() : nh_(""), pnh_("~"), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)
{
  pnh_.param("show_debug_info", show_debug_info_, bool(false));
  pnh_.param("predict_frequency", kf_rate_, double(100.0));
  kf_dt_ = 1.0 / std::max(kf_rate_, 0.1);
  pnh_.param("tf_rate", tf_rate_, double(10.0));
  pnh_.param("enable_yaw_bias_estimation", enable_yaw_bias_estimation_, bool(true));
  pnh_.param("extend_state_step", extend_state_step_, int(100));
  pnh_.param("pose_frame_id", pose_frame_id_, std::string("map"));

  /* NDT measurement */
  pnh_.param("ndt_additional_delay", ndt_additional_delay_, double(0.15));
  pnh_.param("ndt_measure_uncertainty_time", ndt_measure_uncertainty_time_, double(0.01));
  pnh_.param("ndt_rate", ndt_rate_, double(10.0));             // used for covariance calculation
  pnh_.param("ndt_gate_dist", ndt_gate_dist_, double(1000.0)); // Mahalanobis limit
  pnh_.param("ndt_stddev_x", ndt_stddev_x_, double(0.05));
  pnh_.param("ndt_stddev_y", ndt_stddev_y_, double(0.05));
  pnh_.param("ndt_stddev_yaw", ndt_stddev_yaw_, double(0.035));

  /* twist measurement */
  pnh_.param("twist_additional_delay", twist_additional_delay_, double(0.0));
  pnh_.param("twist_rate", twist_rate_, double(10.0));             // used for covariance calculation
  pnh_.param("twist_gate_dist", twist_gate_dist_, double(1000.0)); // Mahalanobis limit
  pnh_.param("twist_stddev_vx", twist_stddev_vx_, double(0.3));
  pnh_.param("twist_stddev_wz", twist_stddev_wz_, double(0.3));

  /* process noise */
  double stddev_proc_yaw_c, stddev_proc_yaw_bias_c, stddev_proc_vx_c, stddev_proc_wz_c;
  pnh_.param("stddev_proc_yaw_c", stddev_proc_yaw_c, double(0.03));
  pnh_.param("stddev_proc_yaw_bias_c", stddev_proc_yaw_bias_c, double(0.001));
  pnh_.param("stddev_proc_vx_c", stddev_proc_vx_c, double(10.0));
  pnh_.param("stddev_proc_wz_c", stddev_proc_wz_c, double(10.0));
  if (!enable_yaw_bias_estimation_) {
    stddev_proc_yaw_bias_c = 0.0;
  }

  /* convert to continuous to discrete */
  cov_proc_vx_d_ = std::pow(stddev_proc_vx_c * kf_dt_, 2.0);
  cov_proc_wz_d_ = std::pow(stddev_proc_wz_c * kf_dt_, 2.0);
  cov_proc_yaw_d_ = std::pow(stddev_proc_yaw_c * kf_dt_, 2.0);
  cov_proc_yaw_bias_d_ = std::pow(stddev_proc_yaw_bias_c * kf_dt_, 2.0);

  /* initialize ros system */
  std::string in_ndt_pose, in_twist, out_pose, out_twist, out_pose_with_covariance;
  pnh_.param("input_ndt_pose_name", in_ndt_pose, std::string("/ndt_pose"));
  pnh_.param("input_twist_name", in_twist, std::string("/can_twist"));
  pnh_.param("output_pose_name", out_pose, std::string("/kf_pose"));
  pnh_.param("output_twist_name", out_twist, std::string("/kf_twist"));
  pnh_.param("output_pose_with_covariance_name", out_pose_with_covariance, std::string("/kf_pose_with_covariance"));
  timer_control_ = nh_.createTimer(ros::Duration(kf_dt_), &KalmanFilterNode::timerCallback, this);
  timer_tf_ = nh_.createTimer(ros::Duration(1.0 / tf_rate_), &KalmanFilterNode::timerTFCallback, this);
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(out_pose, 1);
  pub_pose_cov_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(out_pose_with_covariance, 1);
  pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(out_twist, 1);
  pub_yaw_bias_ = pnh_.advertise<std_msgs::Float64>("estimated_yaw_bias", 1);
  sub_initialpose_ = nh_.subscribe("/initialpose", 1, &KalmanFilterNode::callbackInitialPose, this);
  sub_ndt_pose_ = nh_.subscribe(in_ndt_pose, 1, &KalmanFilterNode::callbackNDTPose, this);
  sub_twist_ = nh_.subscribe(in_twist, 1, &KalmanFilterNode::callbackTwist, this);

  dim_x_ex_ = dim_x_ * extend_state_step_;

  initKalmanFilter();

  /* debug */
  pub_debug_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug", 1);
  pub_ndt_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("my_ndt_pose", 1);
};

KalmanFilterNode::~KalmanFilterNode(){};

/*
 * timerCallback
 */
void KalmanFilterNode::timerCallback(const ros::TimerEvent &e)
{
  DEBUG_INFO("===== timer called =====");

  /* predict model in kalman filter */
  auto start = std::chrono::system_clock::now();
  DEBUG_INFO("----- start prediction -----");
  predictKinematicsModel();
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
  DEBUG_INFO("[kalman filter] predictKinematicsModel calculation time = %f [ms]", elapsed * 1.0e-6);
  DEBUG_INFO("----- end prediction -----\n");

  if (current_ndt_pose_ptr_ != nullptr)
  {
    DEBUG_INFO("----- start NDT -----");
    start = std::chrono::system_clock::now();
    measurementUpdateNDTPose(*current_ndt_pose_ptr_);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
    DEBUG_INFO("[kalman filter] measurementUpdateNDTPose calculation time = %f [ms]", elapsed * 1.0e-6);
    DEBUG_INFO("----- end NDT -----\n");
  }

  if (current_twist_ptr_ != nullptr)
  {
    DEBUG_INFO("----- start twist -----");
    start = std::chrono::system_clock::now();
    measurementUpdateTwist(*current_twist_ptr_);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
    DEBUG_INFO("[kalman filter] measurementUpdateTwist calculation time = %f [ms]", elapsed * 1.0e-6);
    DEBUG_INFO("----- end twist -----\n");
  }

  /* set current pose, twist */
  setCurrentResult();

  publishEstimatedPose();
}

void KalmanFilterNode::setCurrentResult()
{

  current_kf_pose_.header.frame_id = "world";
  current_kf_pose_.header.stamp = ros::Time::now();
  current_kf_pose_.pose.position.x = kf_.getXelement(IDX::X);
  current_kf_pose_.pose.position.y = kf_.getXelement(IDX::Y);
  double yaw = kf_.getXelement(IDX::YAW) + kf_.getXelement(IDX::YAWB);

  double roll, pitch;
  if (current_ndt_pose_ptr_ != nullptr)
  {
    current_kf_pose_.pose.position.z = current_ndt_pose_ptr_->pose.position.z;
    tf::Quaternion q_tf;
    quaternionMsgToTF(current_ndt_pose_ptr_->pose.orientation, q_tf); /* use NDT pitch and roll */
    tf::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  }
  else
  {
    current_kf_pose_.pose.position.z = 0.0;
    roll = 0;
    pitch = 0;
  }
  current_kf_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  current_kf_twist_.header.frame_id = "base_link";
  current_kf_twist_.header.stamp = ros::Time::now();
  current_kf_twist_.twist.linear.x = kf_.getXelement(IDX::VX);
  current_kf_twist_.twist.angular.z = kf_.getXelement(IDX::WZ);
}

/*
 * timerTFCallback
 */
void KalmanFilterNode::timerTFCallback(const ros::TimerEvent &e)
{
  if (current_kf_pose_.header.frame_id == "") 
    return;

  tf::Transform t;
  t.setOrigin(tf::Vector3(current_kf_pose_.pose.position.x, current_kf_pose_.pose.position.y, current_kf_pose_.pose.position.z));
  t.setRotation(tf::Quaternion(current_kf_pose_.pose.orientation.x, current_kf_pose_.pose.orientation.y,
                               current_kf_pose_.pose.orientation.z, current_kf_pose_.pose.orientation.w));
  tf_br_.sendTransform(tf::StampedTransform(t, ros::Time::now(), current_kf_pose_.header.frame_id, "kf_pose"));
}

void KalmanFilterNode::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped &initialpose)
{
  if (initialpose.header.frame_id != "world") {
    ROS_WARN_DELAYED_THROTTLE(5, "twist frame_id must be world");
  }

  Eigen::MatrixXd X(dim_x_,1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  X(IDX::X) = initialpose.pose.pose.position.x;
  X(IDX::Y) = initialpose.pose.pose.position.y;
  X(IDX::YAW) = tf2::getYaw(initialpose.pose.pose.orientation);
  X(IDX::YAWB) = 0.0;
  X(IDX::VX) = 0.0;
  X(IDX::WZ) = 0.0;

  P(IDX::X, IDX::X) = initialpose.pose.covariance[0];
  P(IDX::Y, IDX::Y) = initialpose.pose.covariance[6+1];
  P(IDX::YAW, IDX::YAW) = initialpose.pose.covariance[6*5+5];
  P(IDX::YAWB, IDX::YAWB) = 0.0001;
  P(IDX::VX, IDX::VX) = 0.01;
  P(IDX::WZ, IDX::WZ) = 0.01;

  kf_.init(X, P, extend_state_step_);

};

/*
 * callbackNDTPose
 */
void KalmanFilterNode::callbackNDTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  current_ndt_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(*msg);
};

/*
 * callbackTwist
 */
void KalmanFilterNode::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  current_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
};

/*
 * initKalmanFilter
 */
void KalmanFilterNode::initKalmanFilter()
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E3;
  P(IDX::YAWB, IDX::YAWB) = cov_proc_yaw_bias_d_; // for yaw bias

  kf_.init(X, P, extend_state_step_);
}

/*
 * predictKinematicsModel
 */
void KalmanFilterNode::predictKinematicsModel()
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * b_{k+1}   = b_k
   * vx_{k+1}  = vz_k
   * wz_{k+1}  = wz_k   
   * 
   * (b_k : yaw_bias_k)
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, -vx*sin(yaw+b)*dt, -vx*sin(yaw+b)*dt, cos(yaw+b)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw+b)*dt,  vx*cos(yaw+b)*dt, sin(yaw+b)*dt,  0]
   *     [ 0, 0,                 1,                 0,             0, dt]
   *     [ 0, 0,                 0,                 1,             0,  0]
   *     [ 0, 0,                 0,                 0,             1,  0]
   *     [ 0, 0,                 0,                 0,             0,  1]
   */

  Eigen::MatrixXd X_curr(dim_x_, 1); // curent state
  Eigen::MatrixXd X_next(dim_x_, 1); // predicted state
  kf_.getCurrentX(X_curr);

  Eigen::MatrixXd P_curr;
  kf_.getCurrentP(P_curr);

  const int d_dim_x = dim_x_ex_ - dim_x_;
  const double yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double vx = X_curr(IDX::VX);
  const double wz = X_curr(IDX::WZ);
  const double dt = kf_dt_;

  /* Update for latest state */
  X_next(IDX::X) = X_curr(IDX::X) + vx * cos(yaw + yaw_bias) * dt; // dx = v * cos(yaw)
  X_next(IDX::Y) = X_curr(IDX::Y) + vx * sin(yaw + yaw_bias) * dt; // dy = v * sin(yaw)
  X_next(IDX::YAW) = X_curr(IDX::YAW) + (wz)*dt;                   // dyaw = omega + omega_bias
  X_next(IDX::YAWB) = yaw_bias;
  X_next(IDX::VX) = vx;
  X_next(IDX::WZ) = wz;

  while (std::fabs(X_next(IDX::YAW)) > M_PI)
  {
    X_next(IDX::YAW) -= 2.0 * M_PI * ((X_next(IDX::YAW) > 0) - (X_next(IDX::YAW) < 0));
  }

  /* Set A matrix for latest state */
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(IDX::X, IDX::YAW) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::YAWB) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::VX) = cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAW) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAWB) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::VX) = sin(yaw + yaw_bias) * dt;
  A(IDX::YAW, IDX::WZ) = dt;

  /* Set covariance matrix Q for process noise. Calc Q by velocity and yaw angle covariance :
     dx = Ax + J*w -> Q = J*w_cov*J'          */
  Eigen::MatrixXd J(2, 2); // coeff of deviation of vx & yaw
  J << cos(yaw), -vx * sin(yaw),
      sin(yaw), vx * cos(yaw);
  Eigen::MatrixXd Q_vx_yaw = Eigen::MatrixXd::Zero(2, 2); // cov of vx and yaw
  Q_vx_yaw(0, 0) = P_curr(IDX::VX, IDX::VX) * dt * dt;    // covariance of vx
  Q_vx_yaw(1, 1) = P_curr(IDX::YAW, IDX::YAW) * dt * dt;  // covariance of yaw
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
  Q.block(0, 0, 2, 2) = J * Q_vx_yaw * J.transpose(); // for pos_x & pos_y
  Q(IDX::YAW, IDX::YAW) = cov_proc_yaw_d_;            // for yaw
  Q(IDX::YAWB, IDX::YAWB) = cov_proc_yaw_bias_d_;     // for yaw bias
  Q(IDX::VX, IDX::VX) = cov_proc_vx_d_;               // for vx
  Q(IDX::WZ, IDX::WZ) = cov_proc_wz_d_;               // for wz

  kf_.predictWithDelay(X_next, A, Q);
}

int KalmanFilterNode::getDelayStep(const double &delay_time, const int &extend_state_step, const double &kf_dt)
{

  if (delay_time < 0)
  {
    ROS_WARN("time stamp is inappropriate, set 0. delay = %f", delay_time);
    return 0;
  };

  int delay_step = std::roundf(delay_time / kf_dt);
  if (delay_step > extend_state_step - 1)
  {
    delay_step = extend_state_step - 1;
    ROS_WARN_DELAYED_THROTTLE(5.0, "delay time: %f[s] exceeds the allowable limit: extend_state_step * kf_dt = %f [s]", delay_time, extend_state_step * kf_dt);
  }

  return delay_step;
}

/*
 * measurementUpdateNDTPose
 */
void KalmanFilterNode::measurementUpdateNDTPose(const geometry_msgs::PoseStamped &ndt_pose)
{
  // if (ndt_pose.header.frame_id != "world") {
  //   ROS_WARN_DELAYED_THROTTLE(5, "twist frame_id must be world");
  // }

  const int dim_y = 3; // pos_x, pos_y, yaw, depending on NDT output
  const ros::Time t_curr = ros::Time::now();

  /* Calculate delay step */
  double delay_time = (t_curr - ndt_pose.header.stamp).toSec() + ndt_additional_delay_;
  int delay_step = getDelayStep(delay_time, extend_state_step_, kf_dt_);

  /* Set yaw */
  const double yaw_curr = kf_.getXelement((unsigned int)(delay_step * dim_x_ + IDX::YAW));
  double yaw = tf::getYaw(ndt_pose.pose.orientation);
  while (std::fabs(yaw - yaw_curr) > M_PI)
  {
    yaw -= 2.0 * M_PI * ((yaw > yaw_curr) - (yaw < yaw_curr));
  }

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << ndt_pose.pose.position.x, ndt_pose.pose.position.y, yaw;

  /* Gate */
  Eigen::MatrixXd y_kf(dim_y, 1);
  y_kf << kf_.getXelement(delay_step * dim_x_ + IDX::X), kf_.getXelement(delay_step * dim_x_ + IDX::Y), kf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  Eigen::MatrixXd P_curr, P_y;
  kf_.getCurrentP(P_curr);
  P_y = P_curr.block(0, 0, dim_y, dim_y);
  if (!mahalanobisGate(ndt_gate_dist_, y_kf, y, P_y))
  {
    ROS_WARN_DELAYED_THROTTLE(2.0, "[kalman filter] measurement update, mahalanobis distance is larger than limit. ignore NDT measurement data.");
    return;
  }

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::X) = 1.0;   // for pos x
  C(1, IDX::Y) = 1.0;   // for pos y
  C(2, IDX::YAW) = 1.0; // for yaw

  /* Set measurement noise covariancs : NOTE this should be set by NDT reliability */
  const double kf_yaw = kf_.getXelement(IDX::YAW);
  const double vx = kf_.getXelement(IDX::VX);
  const double wz = kf_.getXelement(IDX::WZ);
  const double cov_pos_x = std::pow(ndt_measure_uncertainty_time_ * vx * cos(kf_yaw), 2.0);
  const double cov_pos_y = std::pow(ndt_measure_uncertainty_time_ * vx * sin(kf_yaw), 2.0);
  const double cov_yaw = std::pow(ndt_measure_uncertainty_time_ * wz, 2.0);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  R(0, 0) = std::pow(ndt_stddev_x_, 2) + cov_pos_x; // pos_x
  R(1, 1) = std::pow(ndt_stddev_y_, 2) + cov_pos_y; // pos_y
  R(2, 2) = std::pow(ndt_stddev_yaw_, 2) + cov_yaw; // yaw

  /* In order to avoid a large change at the time of updating, measuremeent update is performed by dividing at every step. */
  R *= (kf_rate_ / ndt_rate_);

  kf_.updateWithDelay(y, C, R, delay_step);
}

/*
 * measurementUpdateTwist
 */
void KalmanFilterNode::measurementUpdateTwist(const geometry_msgs::TwistStamped &twist)
{
  if (twist.header.frame_id != "base_link") {
    ROS_WARN_DELAYED_THROTTLE(3, "twist frame_id must be base_link");
  }

  const int dim_y = 2; // vx, wz
  const ros::Time t_curr = ros::Time::now();

  /* Calculate delay step */
  double delay_time = (t_curr - twist.header.stamp).toSec() + twist_additional_delay_;
  int delay_step = getDelayStep(delay_time, extend_state_step_, kf_dt_);

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << twist.twist.linear.x, twist.twist.angular.z;

  /* Gate */
  Eigen::MatrixXd y_kf(dim_y, 1);
  y_kf << kf_.getXelement(delay_step * dim_x_ + IDX::VX), kf_.getXelement(delay_step * dim_x_ + IDX::WZ);
  Eigen::MatrixXd P_curr;
  kf_.getCurrentP(P_curr);
  if (!mahalanobisGate(ndt_gate_dist_, y_kf, y, P_curr.block(4, 4, dim_y, dim_y)))
  {
    ROS_WARN_DELAYED_THROTTLE(2.0, "[kalman filter] twist measurement update, mahalanobis distance is larger than limit. ignore twist measurement data.");
    return;
  }

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::VX) = 1.0; // for vx
  C(1, IDX::WZ) = 1.0; // for wz

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  R(0, 0) = twist_stddev_vx_ * twist_stddev_vx_ * kf_dt_ * kf_dt_; // for vx
  R(1, 1) = twist_stddev_wz_ * twist_stddev_wz_ * kf_dt_ * kf_dt_; // for wz

  /* In order to avoid a large change by update, measurement update is performed by dividing at every step. */
  R *= (kf_rate_ / twist_rate_);

  kf_.updateWithDelay(y, C, R, delay_step);
};

/*
 * mahalanobisGate
 */
bool KalmanFilterNode::mahalanobisGate(const double &dist_max, const Eigen::MatrixXd &x,
                                       const Eigen::MatrixXd &obj_x, const Eigen::MatrixXd &cov)
{
  Eigen::MatrixXd mahalanobis_squared = (x - obj_x).transpose() * cov.inverse() * (x - obj_x);
  DEBUG_INFO("measurement update: mahalanobis = %f, gate limit = %f", std::sqrt(mahalanobis_squared(0)), dist_max);
  if (mahalanobis_squared(0) > dist_max * dist_max)
  {
    return false;
  }

  return true;
}

/*
 * publishEstimatedPose
 */
void KalmanFilterNode::publishEstimatedPose()
{
  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P(dim_x_, dim_x_);
  kf_.getCurrentX(X);
  kf_.getCurrentP(P);

  /* publish latest pose */
  pub_pose_.publish(current_kf_pose_);

  /* publish latest pose with covariance */
  geometry_msgs::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = ros::Time::now();
  pose_cov.header.frame_id = current_kf_pose_.header.frame_id;
  pose_cov.pose.pose = current_kf_pose_.pose;
  for (int i = 0; i < 36; ++i)
  {
    pose_cov.pose.covariance[i] = 0.0;
  }
  pose_cov.pose.covariance[0] = P(IDX::X, IDX::X);      // x, x
  pose_cov.pose.covariance[1] = P(IDX::X, IDX::Y);      // x, y
  pose_cov.pose.covariance[5] = P(IDX::X, IDX::YAW);    // x, yaw
  pose_cov.pose.covariance[6] = P(IDX::Y, IDX::X);      // y, x
  pose_cov.pose.covariance[7] = P(IDX::Y, IDX::Y);      // y, y
  pose_cov.pose.covariance[11] = P(IDX::Y, IDX::YAW);   // y, yaw
  pose_cov.pose.covariance[30] = P(IDX::YAW, IDX::X);   // yaw, x
  pose_cov.pose.covariance[31] = P(IDX::YAW, IDX::Y);   // yaw, y
  pose_cov.pose.covariance[35] = P(IDX::YAW, IDX::YAW); // yaw, yaw
  pub_pose_cov_.publish(pose_cov);

  /* publish latest twist */
  pub_twist_.publish(current_kf_twist_);

  /* publish yaw bias */
  std_msgs::Float64 yawb;
  yawb.data = X(IDX::YAWB);
  pub_yaw_bias_.publish(yawb);

  /* debug my ndt */
  if (current_ndt_pose_ptr_ != nullptr)
  {
    geometry_msgs::PoseStamped p;
    p = *current_ndt_pose_ptr_;
    p.header.stamp = ros::Time::now();
    pub_ndt_pose_.publish(p);
  }

  /* debug publish */
  double RAD2DEG = 180.0 / 3.141592;
  double ndt_yaw = 0.0;
  if (current_ndt_pose_ptr_ != nullptr)
    ndt_yaw = tf::getYaw(current_ndt_pose_ptr_->pose.orientation) * RAD2DEG;
  
  std_msgs::Float64MultiArray msg;
  msg.data.push_back(X(IDX::YAW) * RAD2DEG);  // [0] yaw angle
  msg.data.push_back(ndt_yaw);                // [1] NDT yaw angle
  msg.data.push_back(X(IDX::YAWB) * RAD2DEG); // [2] yaw bias
  pub_debug_.publish(msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "kalman_filter");
  KalmanFilterNode obj;

  ros::spin();

  return 0;
};
