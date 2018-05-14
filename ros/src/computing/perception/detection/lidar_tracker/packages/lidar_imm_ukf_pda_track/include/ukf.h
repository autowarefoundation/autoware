#ifndef OBJECT_TRACKING_UKF_H
#define OBJECT_TRACKING_UKF_H

#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <jsk_recognition_msgs/BoundingBox.h>

class UKF
{
public:
  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  //    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_merge_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_cv_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_ctrv_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_rm_;

  //    ///* state covariance matrix
  Eigen::MatrixXd p_merge_;

  ///* state covariance matrix
  Eigen::MatrixXd p_cv_;

  ///* state covariance matrix
  Eigen::MatrixXd p_ctrv_;

  ///* state covariance matrix
  Eigen::MatrixXd p_rm_;

  ///* predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_cv_;

  ///* predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_ctrv_;

  ///* predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_rm_;

  ///* time when the state is true, in us
  long long time_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_cv_;
  double std_a_ctrv_;
  double std_a_rm_;


  // CTRV
  double std_ctrv_yawdd_;
  // CV
  double std_cv_yawdd_;

  double std_rm_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Weights of sigma points
  Eigen::VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* Augmented sigma point spreading parameter
  double lambda_aug_;

  int count_;
  int count_empty_;

  double mode_match_prob_cv2cv_;
  double mode_match_prob_ctrv2cv_;
  double mode_match_prob_rm2cv_;

  double mode_match_prob_cv2ctrv_;
  double mode_match_prob_ctrv2ctrv_;
  double mode_match_prob_rm2ctrv_;

  double mode_match_prob_cv2rm_;
  double mode_match_prob_ctrv2rm_;
  double mode_match_prob_rm2rm_;

  double mode_match_prob_cv_;

  double mode_match_prob_ctrv_;

  double mode_match_prob_rm_;

  double mode_prob_cv_;
  double mode_prob_ctrv_;
  double mode_prob_rm_;

  std::vector<double> ini_u_;

  std::vector<double> p1_;

  std::vector<double> p2_;

  std::vector<double> p3_;

  Eigen::VectorXd z_pred_cv_;
  Eigen::VectorXd z_pred_ctrv_;
  Eigen::VectorXd z_pred_rm_;

  Eigen::MatrixXd s_cv_;
  Eigen::MatrixXd s_ctrv_;
  Eigen::MatrixXd s_rm_;

  Eigen::MatrixXd k_cv_;
  Eigen::MatrixXd k_ctrv_;
  Eigen::MatrixXd k_rm_;

  double pd_;
  double pg_;

  int lifetime_;
  bool is_static_;

  // bounding box params
  bool is_vis_bb_;

  jsk_recognition_msgs::BoundingBox jsk_bb_;
  jsk_recognition_msgs::BoundingBox best_jsk_bb_;

  bool is_best_jsk_bb_empty_;

  double best_yaw_;
  double bb_yaw_;
  double bb_area_;
  std::vector<double> bb_yaw_history_;
  std::vector<double> bb_vel_history_;
  std::vector<double> bb_area_history_;

  // for env classification
  Eigen::VectorXd init_meas_;
  double dist_from_init_;

  std::vector<Eigen::VectorXd> local2local_;
  std::vector<double> local2localYawVec_;

  double x_merge_yaw_;

  int tracking_num_;

  /**
   * Constructor
   */
  UKF();

  void updateYawWithHighProb();

  void initialize(const Eigen::VectorXd& z, const double timestamp);

  void updateModeProb(const std::vector<double>& lambda_vec);

  void mergeEstimationAndCovariance();

  void mixingProbability();

  void interaction();

  void predictionIMMUKF(const double dt);

  void updateIMMUKF(const std::vector<double>& lambda_vec);

  void ctrv(const double p_x, const double p_y, const double v, const double yaw, const double yawd, const double nu_a,
            const double nu_yawdd, const double delta_t, std::vector<double>& state);

  void cv(const double p_x, const double p_y, const double v, const double yaw, const double yawd, const double nu_a,
          const double nu_yawdd, const double delta_t, std::vector<double>& state);

  void randomMotion(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
                    const double nu_a, const double nu_yawdd, const double delta_t, std::vector<double>& state);

  void prediction(const double delta_t, const int model_ind);

  void updateLidar(const int model_ind);

};

#endif /* UKF_H */
