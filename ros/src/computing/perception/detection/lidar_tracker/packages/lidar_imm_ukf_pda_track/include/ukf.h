/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OBJECT_TRACKING_UKF_H
#define OBJECT_TRACKING_UKF_H

#include "Eigen/Dense"
#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <geometry_msgs/PoseStamped.h>
#include <jsk_recognition_msgs/BoundingBox.h>

#include "autoware_msgs/DetectedObject.h"

enum TrackingState : int
{
  Die = 0,     // No longer tracking
  Init = 1,    // Start tracking
  Stable = 4,  // Stable tracking
  Lost = 10,   // About to lose target
};

class UKF
{
  /*
  cv: Constant Velocity
  ctrv: Constatnt Turn Rate and Velocity
  rm: Random Motion
  */

public:
  int ukf_id_;

  //* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  //* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_merge_;

  //* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_cv_;

  //* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_ctrv_;

  //* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_rm_;

  //* state covariance matrix
  Eigen::MatrixXd p_merge_;

  //* state covariance matrix
  Eigen::MatrixXd p_cv_;

  //* state covariance matrix
  Eigen::MatrixXd p_ctrv_;

  //* state covariance matrix
  Eigen::MatrixXd p_rm_;

  //* predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_cv_;

  //* predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_ctrv_;

  //* predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_rm_;

  //* time when the state is true, in us
  long long time_;

  //* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_cv_;
  double std_a_ctrv_;
  double std_a_rm_;

  // CTRV
  double std_ctrv_yawdd_;
  // CV
  double std_cv_yawdd_;

  double std_rm_yawdd_;

  //* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  //* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  //* Weights of sigma points
  Eigen::VectorXd weights_c_;
  Eigen::VectorXd weights_s_;

  //* State dimension
  int n_x_;

  //* Sigma point spreading parameter
  double lambda_;

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
  std::vector<double> vel_history_;

  std::vector<Eigen::VectorXd> local2local_;
  std::vector<double> local2localYawVec_;

  double x_merge_yaw_;

  int tracking_num_;

  Eigen::VectorXd cv_meas_;
  Eigen::VectorXd ctrv_meas_;
  Eigen::VectorXd rm_meas_;

  Eigen::MatrixXd q_cv_;
  Eigen::MatrixXd q_ctrv_;
  Eigen::MatrixXd q_rm_;

  Eigen::MatrixXd r_cv_;
  Eigen::MatrixXd r_ctrv_;
  Eigen::MatrixXd r_rm_;

  double nis_cv_;
  double nis_ctrv_;
  double nis_rm_;

  Eigen::MatrixXd new_x_sig_cv_;
  Eigen::MatrixXd new_x_sig_ctrv_;
  Eigen::MatrixXd new_x_sig_rm_;

  Eigen::MatrixXd new_z_sig_cv_;
  Eigen::MatrixXd new_z_sig_ctrv_;
  Eigen::MatrixXd new_z_sig_rm_;

  Eigen::VectorXd new_z_pred_cv_;
  Eigen::VectorXd new_z_pred_ctrv_;
  Eigen::VectorXd new_z_pred_rm_;

  Eigen::MatrixXd new_s_cv_;
  Eigen::MatrixXd new_s_ctrv_;
  Eigen::MatrixXd new_s_rm_;

  /**
   * Constructor
   */
  UKF();

  void updateYawWithHighProb();

  void initialize(const Eigen::VectorXd& z, const double timestamp, const int target_ind);

  void updateModeProb(const std::vector<double>& lambda_vec);

  void mergeEstimationAndCovariance();

  void mixingProbability();

  void interaction();

  void predictionSUKF(const double dt);

  void predictionIMMUKF(const double dt);

  void findMaxZandS(Eigen::VectorXd& max_det_z, Eigen::MatrixXd& max_det_s);

  void updateLikelyMeasurementForCTRV(const std::vector<autoware_msgs::DetectedObject>& object_vec);

  void updateEachMotion(const double detection_probability, const double gate_probability, const double gating_thres,
                        const std::vector<autoware_msgs::DetectedObject>& object_vec, std::vector<double>& lambda_vec);

  void updateSUKF(const std::vector<autoware_msgs::DetectedObject>& object_vec);

  void updateIMMUKF(const double detection_probability, const double gate_probability, const double gating_thres,
                    const std::vector<autoware_msgs::DetectedObject>& object_vec);

  void ctrv(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
            const double delta_t, std::vector<double>& state);

  void cv(const double p_x, const double p_y, const double v, const double yaw, const double yawd, const double delta_t,
          std::vector<double>& state);

  void randomMotion(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
                    const double delta_t, std::vector<double>& state);

  void initCovarQs(const double dt, const double yaw);

  void prediction(const double delta_t, const int model_ind);

  void updateLidar(const int model_ind);
};

#endif /* UKF_H */
