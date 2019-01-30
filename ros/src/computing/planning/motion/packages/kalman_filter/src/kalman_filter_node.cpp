#include <deque>
#include <iostream>
#include <vector>

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

#include "kalman_filter/kalman_filter.h"

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define EKF_INFO(...)                                                          \
  {                                                                            \
    if (show_debug_info_) {                                                    \
      printf(__VA_ARGS__);                                                     \
    }                                                                          \
  }

class KalmanFilterNode {
public:
  KalmanFilterNode();
  ~KalmanFilterNode();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_pose_, pub_pose_array_, pub_debug_;
  ros::Subscriber sub_ndt_pose_, sub_vehicle_status_, sub_imu_;
  ros::Timer timer_control_;
  bool show_debug_info_;

  KalmanFilter kf_;
  bool initial_pose_received_;
  bool initial_twist_received_;

  /* parameters */
  double predict_frequency_;
  double predict_dt_;
  double wheelbase_;
  unsigned int dim_x_;
  unsigned int dim_x_ex_;
  int extend_state_step_;

  /* for model prediction */
  std::deque<double> vel_array_;
  std::deque<double> omega_array_;

  autoware_msgs::VehicleStatus current_vehicle_status_;
  geometry_msgs::TwistStamped current_twist_;
  geometry_msgs::PoseStamped current_ndt_pose_;
  sensor_msgs::Imu current_imu_;

  void timerCallback(const ros::TimerEvent &e);
  void callbackNDTPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void callbackVehicleStatus(const autoware_msgs::VehicleStatus &msg);
  void callbackIMU(const sensor_msgs::Imu &msg);
  void callbackTwist(const geometry_msgs::TwistStamped &msg);

  void initKalmanFilter();
  void predictKinematicsModel();
  void measurementUpdateNDTPose(const geometry_msgs::PoseStamped &ndt_pose);
  void measurementUpdateIMU(const sensor_msgs::Imu &msg);

  void publishEstimatedPose();
};

KalmanFilterNode::KalmanFilterNode()
    : nh_(""), pnh_("~"), initial_pose_received_(false),
      initial_twist_received_(false) {

  pnh_.param("show_debug_info", show_debug_info_, bool(false));
  pnh_.param("predict_frequency", predict_frequency_, double(5.0));
  pnh_.param("wheelbase", wheelbase_, double(2.79));
  pnh_.param("extend_state_step", extend_state_step_, int(50));
  predict_dt_ = 1.0 / std::max(predict_frequency_, 0.1);
  ROS_ERROR("predict_frequency = %f", predict_frequency_);
  ROS_ERROR("predict_dt_ = %f", predict_dt_);

  timer_control_ = nh_.createTimer(ros::Duration(predict_dt_), &KalmanFilterNode::timerCallback, this);
  pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/kf_estimated_pose", 1);
  pub_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("/kalman_filter_pose_array", 1);
  pub_debug_ = pnh_.advertise<std_msgs::Float64MultiArray>("debug", 1);
  sub_ndt_pose_ = nh_.subscribe("/ndt_pose", 1, &KalmanFilterNode::callbackNDTPose, this);
  sub_vehicle_status_ = nh_.subscribe("/vehicle_status", 1, &KalmanFilterNode::callbackVehicleStatus, this);
  sub_imu_ = nh_.subscribe("/imu_raw", 1, &KalmanFilterNode::callbackIMU, this);

  std::deque<double> zero(extend_state_step_, 0.0);
  vel_array_ = zero;
  omega_array_ = zero;

  dim_x_ = 3;
  dim_x_ex_ = dim_x_ * extend_state_step_;

  initKalmanFilter();
};

KalmanFilterNode::~KalmanFilterNode(){};

/*
 * timerCallback
 */
void KalmanFilterNode::timerCallback(const ros::TimerEvent &e) {
  if (!initial_pose_received_ || !initial_twist_received_) {
    ROS_WARN("initial info is not received. pose = %d, odom = %d",
             initial_pose_received_, initial_twist_received_);
    return;
  }

  Eigen::MatrixXd X, Xp;
  kf_.getX(X);

  /* predict model in kalman filter */
  predictKinematicsModel();

  /* set current angular and linear velocity vector */
  vel_array_.push_front(current_twist_.twist.linear.x);
  vel_array_.pop_back();
  omega_array_.push_front(current_twist_.twist.angular.z);
  omega_array_.pop_back();

  publishEstimatedPose();

  /* debug */
  // kf_.getX(Xp);
  // printf("[-------- predict --------]\n");
  // printf("before predict, pos_x: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", X(i));
  // }
  // printf("\n");
  // printf("after predict, pos_x: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", Xp(i));
  // }
  // printf("\n\n");
  // printf("before predict, pos_y: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", X(i+1));
  // }
  // printf("\n");
  // printf("after predict, pos_y: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", Xp(i+1));
  // }
  // printf("\n\n");
  // printf("before predict, yaw: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", X(i+2));
  // }
  // printf("\n");
  // printf("after predict, yaw: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", Xp(i+2));
  // }
  // printf("\n");

  kf_.getX(X);
  EKF_INFO("[predict] pos_x = %f, pos_y = %f, yaw = %f", X(0), X(1), X(2));
  std_msgs::Float64MultiArray msg;
  msg.data.push_back(current_twist_.twist.linear.x);
  msg.data.push_back(current_twist_.twist.angular.z);
  pub_debug_.publish(msg);
}

/*
 * callbackNDTPose
 */
void KalmanFilterNode::callbackNDTPose(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  current_ndt_pose_ = *msg;
  initial_pose_received_ = true;

  measurementUpdateNDTPose(current_ndt_pose_);
};

/*
 * callbackVehicleStatus
 */
void KalmanFilterNode::callbackVehicleStatus(
    const autoware_msgs::VehicleStatus &msg) {
  current_vehicle_status_ = msg;
  // current_twist_.linear.x = msg->speed;
  // current_twist_.angular.z = msg->speed * tan(msg->angle) / wheelbase_;

  // TEMP
  current_twist_.twist.linear.x = msg.speed / 3.6;
  current_twist_.twist.angular.z =
      (msg.speed / 3.6) * tan(msg.angle * 3.1415 / 180.0 / 17.85) / 2.95;
  initial_twist_received_ = true;
};

/*
 * callbackTwist
 */
void KalmanFilterNode::callbackTwist(const geometry_msgs::TwistStamped &msg) {
  current_twist_ = msg;
  initial_twist_received_ = true;
};

/*
 * callbackIMU
 */
void KalmanFilterNode::callbackIMU(const sensor_msgs::Imu &msg) {
  current_imu_ = msg;
  measurementUpdateIMU(current_imu_);
}

/*
 * initKalmanFilter
 */
void KalmanFilterNode::initKalmanFilter() {
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_ex_, dim_x_ex_);
  P *= 1000000.0; // initial covariance
  kf_.init(X, P);
}

/*
 * predictKinematicsModel
 */
void KalmanFilterNode::predictKinematicsModel() {
  /* parameters */
  const double sigma_proc_x = std::pow(0.1, 2);
  const double sigma_proc_y = std::pow(0.1, 2);
  const double sigma_proc_yaw = std::pow(0.05, 2);

  /* predict state with nonlinear dynamics */
  Eigen::MatrixXd X(dim_x_ex_, 1);
  kf_.getX(X);

  Eigen::MatrixXd X_next(dim_x_ex_, 1);
  Eigen::MatrixXd A_ex = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  Eigen::MatrixXd Q_ex = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  const double vx = current_twist_.twist.linear.x;
  const double wz = current_twist_.twist.angular.z;
  // printf("vx = %f, wz = %f\n", vx, wz);
  const double yaw = X(2);
  X_next(0) = X(0) + vx * cos(yaw) * predict_dt_; // dx = v * cos(yaw)
  X_next(1) = X(1) + vx * sin(yaw) * predict_dt_; // dy = v * sin(yaw)
  X_next(2) = X(2) + wz * predict_dt_;            // dyaw = omega
  for (int i = dim_x_; i < dim_x_ex_; ++i) {
    X_next(i) = X(i - dim_x_);
  }
  A_ex.block(0, 0, dim_x_, dim_x_) = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A_ex(0, 2) = -vx * sin(yaw) * predict_dt_;
  A_ex(1, 2) = vx * cos(yaw) * predict_dt_;
  const int dim_d = dim_x_ex_ - dim_x_;
  A_ex.block(dim_x_, 0, dim_d, dim_d) = Eigen::MatrixXd::Identity(dim_d, dim_d);

  Q_ex(0, 0) = sigma_proc_x;
  Q_ex(1, 1) = sigma_proc_y;
  Q_ex(2, 2) = sigma_proc_yaw;

  // PRINT_MAT(A_ex);
  // PRINT_MAT(X.transpose());
  // PRINT_MAT(X_next.transpose());
  // PRINT_MAT(Q_ex);

  kf_.predictEKF(X_next, A_ex, Q_ex);
  Eigen::MatrixXd TMP;

  kf_.getX(TMP);
  printf("X : %f -> %f, ", X(0), TMP(0));


  /* debug */
  // PRINT_MAT(X);
  // PRINT_MAT(X_next);
  // PRINT_MAT(A_ex);
  // PRINT_MAT(Q_ex);
}

/*
 * measurementUpdateNDTPose
 */
void KalmanFilterNode::measurementUpdateNDTPose(
    const geometry_msgs::PoseStamped &ndt_pose) {
  /* calculate delay step */
  double delay_time = (ros::Time::now() - ndt_pose.header.stamp).toSec();
  if (delay_time < 0) {
    ROS_WARN("time is wrong. delay = %f, now() = %f, ndt.stamp = %f",
             delay_time, ros::Time::now().toSec(),
             ndt_pose.header.stamp.toSec());
    delay_time = 0;
  };

  int delay_step = std::roundf(delay_time / predict_dt_);
  if (delay_step > extend_state_step_ - 1) {
    delay_step = extend_state_step_ - 1;
    ROS_WARN("NDT pose delay time: %f[s] exceeds the allowable limit: "
             "extend_state_step * predict_dt_ = %f [s]",
             delay_time, extend_state_step_ * predict_dt_);
  }

  EKF_INFO("predict_dt_ = %f\n", predict_dt_);
  // printf("delay_time = %f\n", delay_time);
  // printf("delay_step = %d\n", delay_step);
  EKF_INFO("extend_state_step_ = %d\n", extend_state_step_);

  /* set measurement matrix */
  const int dim_y = 3; // pos_x, pos_y, yaw, depending ndt output
  Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
  C_ex.block(0, dim_x_ * delay_step, dim_y, dim_y) =
      Eigen::MatrixXd::Identity(dim_y, dim_y);

  /* set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  R(0, 0) = std::pow(10.0 / 1.0, 2); // pos_x
  R(1, 1) = std::pow(10.0 / 1.0, 2); // pos_y
  R(2, 2) = std::pow(5.0 / 1.0, 2);  // yaw

  /* measurement update */
  Eigen::MatrixXd y(dim_y, 1);
  y << ndt_pose.pose.position.x, ndt_pose.pose.position.y,
      tf::getYaw(ndt_pose.pose.orientation);

  Eigen::MatrixXd X, P, Xp;
  kf_.getX(X);
  kf_.getP(P);
  EKF_INFO("[Update] predicted pos_x = %f, pos_y = %f, yaw = %f\n", X(0, 0),
           X(1, 0), X(2, 0));
  EKF_INFO("[Update] covariance pos_x = %f, pos_y = %f, yaw = %f\n", P(0, 0),
           P(1, 1), P(2, 2));
  // PRINT_MAT(P);
  // PRINT_MAT(R);
  // PRINT_MAT(C_ex);

  kf_.update(y, C_ex, R);
  kf_.getX(Xp);
  kf_.getP(P);
  EKF_INFO("[Update] updated pos_x = %f, pos_y = %f, yaw = %f\n", Xp(0, 0),
           Xp(1, 0), Xp(2, 0));
  EKF_INFO("[Update] updated covariance pos_x = %f, pos_y = %f, yaw = %f\n",
           P(0, 0), P(1, 1), P(2, 2));
  EKF_INFO("[Update] X diff = %f, %f, %f\n", X(0) - Xp(0), X(1) - Xp(1),
           X(2) - Xp(2));

  // printf("[******** measurement update ********]\n");
  // printf("before update, pos_x: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", X(i));
  // }
  // printf("\n");
  // printf("after update, pos_x: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", Xp(i));
  // }
  // printf("\n\n");
  // printf("before update, pos_y: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", X(i+1));
  // }
  // printf("\n");
  // printf("after update, pos_y: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", Xp(i+1));
  // }
  // printf("\n\n");
  // printf("before update, yaw: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", X(i+2));
  // }
  // printf("\n");
  // printf("after update, yaw: ");
  // for (int i = 0; i < dim_x_ex_; i+=dim_x_) {
  //   printf("%f, ", Xp(i+2));
  // }
  // printf("\n");
}

/*
 * measurementUpdateIMU
 */
void KalmanFilterNode::measurementUpdateIMU(const sensor_msgs::Imu &msg) {}

/*
 * publishEstimatedPose
 */
void KalmanFilterNode::publishEstimatedPose() {
  Eigen::MatrixXd X(dim_x_ex_, 1);
  Eigen::MatrixXd P(dim_x_ex_, dim_x_ex_);
  kf_.getX(X);
  kf_.getP(P);
  // PRINT_MAT(P);

  /* publish latest pose */
  geometry_msgs::PoseWithCovarianceStamped pose_curr;
  pose_curr.header.stamp = ros::Time::now();
  pose_curr.header.frame_id = current_ndt_pose_.header.frame_id;
  pose_curr.pose.pose.position.x = X(0);
  pose_curr.pose.pose.position.y = X(1);
  pose_curr.pose.pose.position.z = current_ndt_pose_.pose.position.z;
  pose_curr.pose.pose.orientation = tf::createQuaternionMsgFromYaw(X(2, 0));
  for (int i = 0; i < 36; ++i) {
    pose_curr.pose.covariance[i] = 0.0;
  }
  pose_curr.pose.covariance[0] = P(0, 0);
  pose_curr.pose.covariance[1] = P(0, 1);
  pose_curr.pose.covariance[5] = P(0, 2);
  pose_curr.pose.covariance[6] = P(1, 0);
  pose_curr.pose.covariance[7] = P(1, 1);
  pose_curr.pose.covariance[11] = P(1, 2);
  pose_curr.pose.covariance[30] = P(2, 1);
  pose_curr.pose.covariance[31] = P(2, 1);
  pose_curr.pose.covariance[35] = P(2, 2);
  static double ppp = 0.0;
  printf("pub_x = %f, diff = %f\n", pose_curr.pose.pose.position.x, pose_curr.pose.pose.position.x-ppp);
  ppp = pose_curr.pose.pose.position.x;
  pub_pose_.publish(pose_curr);

  /* publish pose array */
  geometry_msgs::PoseArray pose_array;
  geometry_msgs::Pose pose;
  for (unsigned int i = 0; i < dim_x_ex_; i += dim_x_) {
    pose.position.x = X(i, 0);
    pose.position.y = X(i + 1, 0);
    pose.position.z = current_ndt_pose_.pose.position.z;
    pose.orientation = tf::createQuaternionMsgFromYaw(X(i + 2, 0));
    pose_array.poses.push_back(pose);
  }
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = current_ndt_pose_.header.frame_id;
  pub_pose_array_.publish(pose_array);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "kalman_filter_node");
  KalmanFilterNode obj;

  ros::spin();

  return 0;
};
