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

#include "ukf.h"

/**
* Initializes Unscented Kalman filter
*/
UKF::UKF()
{
  // initial state vector
  x_merge_ = Eigen::MatrixXd(5, 1);

  // initial state vector
  x_cv_ = Eigen::MatrixXd(5, 1);

  // initial state vector
  x_ctrv_ = Eigen::MatrixXd(5, 1);

  // initial state vector
  x_rm_ = Eigen::MatrixXd(5, 1);

  // initial covariance matrix
  p_merge_ = Eigen::MatrixXd(5, 5);

  // initial covariance matrix
  p_cv_ = Eigen::MatrixXd(5, 5);

  // initial covariance matrix
  p_ctrv_ = Eigen::MatrixXd(5, 5);

  // initial covariance matrix
  p_rm_ = Eigen::MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_cv_ = 2;
  std_a_ctrv_ = 2;
  std_a_rm_ = 3;
  std_ctrv_yawdd_ = 2;
  std_cv_yawdd_ = 2;
  std_rm_yawdd_ = 3;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;
  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // time when the state is true, in us
  time_ = 0.0;

  // state dimension
  n_x_ = 5;

  // predicted sigma points matrix
  x_sig_pred_cv_ = Eigen::MatrixXd(n_x_, 2 * n_x_ + 1);

  // predicted sigma points matrix
  x_sig_pred_ctrv_ = Eigen::MatrixXd(n_x_, 2 * n_x_ + 1);

  // predicted sigma points matrix
  x_sig_pred_rm_ = Eigen::MatrixXd(n_x_, 2 * n_x_ + 1);

  // create vector for weights
  weights_c_ = Eigen::VectorXd(2 * n_x_ + 1);
  weights_s_ = Eigen::VectorXd(2 * n_x_ + 1);

  // transition probability
  p1_.push_back(0.9);
  p1_.push_back(0.05);
  p1_.push_back(0.05);

  p2_.push_back(0.05);
  p2_.push_back(0.9);
  p2_.push_back(0.05);

  p3_.push_back(0.05);
  p3_.push_back(0.05);
  p3_.push_back(0.9);

  mode_match_prob_cv2cv_ = 0;
  mode_match_prob_ctrv2cv_ = 0;
  mode_match_prob_rm2cv_ = 0;

  mode_match_prob_cv2ctrv_ = 0;
  mode_match_prob_ctrv2ctrv_ = 0;
  mode_match_prob_rm2ctrv_ = 0;

  mode_match_prob_cv2rm_ = 0;
  mode_match_prob_ctrv2rm_ = 0;
  mode_match_prob_rm2rm_ = 0;

  mode_prob_cv_ = 0.33;
  mode_prob_ctrv_ = 0.33;
  mode_prob_rm_ = 0.33;

  z_pred_cv_ = Eigen::VectorXd(2);
  z_pred_ctrv_ = Eigen::VectorXd(2);
  z_pred_rm_ = Eigen::VectorXd(2);

  s_cv_ = Eigen::MatrixXd(2, 2);
  s_ctrv_ = Eigen::MatrixXd(2, 2);
  s_rm_ = Eigen::MatrixXd(2, 2);

  k_cv_ = Eigen::MatrixXd(5, 2);
  k_ctrv_ = Eigen::MatrixXd(5, 2);
  k_rm_ = Eigen::MatrixXd(5, 2);

  pd_ = 0.9;
  pg_ = 0.99;

  // tracking parameter
  lifetime_ = 0;
  is_static_ = false;

  // bounding box params
  is_best_jsk_bb_empty_ = false;
  is_vis_bb_ = false;
  jsk_bb_.dimensions.x = 1.0;
  jsk_bb_.dimensions.y = 1.0;
  best_yaw_ = 100;
  bb_yaw_ = 0;
  bb_area_ = 0;

  // for static classification
  init_meas_ = Eigen::VectorXd(2);

  x_merge_yaw_ = 0;

  // for raukf
  cv_meas_ = Eigen::VectorXd(2);
  ctrv_meas_ = Eigen::VectorXd(2);
  rm_meas_ = Eigen::VectorXd(2);

  r_cv_ = Eigen::MatrixXd(2, 2);
  r_ctrv_ = Eigen::MatrixXd(2, 2);
  r_rm_ = Eigen::MatrixXd(2, 2);

  q_cv_ = Eigen::MatrixXd(5, 5);
  q_ctrv_ = Eigen::MatrixXd(5, 5);
  q_rm_ = Eigen::MatrixXd(5, 5);

  nis_cv_ = 0;
  nis_ctrv_ = 0;
  nis_rm_ = 0;

  new_x_sig_cv_ = Eigen::MatrixXd(n_x_, 2 * n_x_ + 1);
  new_x_sig_ctrv_ = Eigen::MatrixXd(n_x_, 2 * n_x_ + 1);
  new_x_sig_rm_ = Eigen::MatrixXd(n_x_, 2 * n_x_ + 1);

  new_z_sig_cv_ = Eigen::MatrixXd(2, 2 * n_x_ + 1);
  new_z_sig_ctrv_ = Eigen::MatrixXd(2, 2 * n_x_ + 1);
  new_z_sig_rm_ = Eigen::MatrixXd(2, 2 * n_x_ + 1);

  new_z_pred_cv_ = Eigen::VectorXd(2);
  new_z_pred_ctrv_ = Eigen::VectorXd(2);
  new_z_pred_rm_ = Eigen::VectorXd(2);

  new_s_cv_ = Eigen::MatrixXd(2, 2);
  new_s_ctrv_ = Eigen::MatrixXd(2, 2);
  new_s_rm_ = Eigen::MatrixXd(2, 2);
}

void UKF::initialize(const Eigen::VectorXd& z, const double timestamp, const int target_id)
{
  ukf_id_ = target_id;

  // first measurement
  x_merge_ << 0, 0, 0, 0, 0.1;

  // init covariance matrix by hardcoding since no clue about initial state covrariance
  p_merge_ << 0.5, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 1;

  // set weights
  // reference from "The Unscented Kalman Filter for Nonlinear Estimation, Eric A. Wan and Rudolph van der Merwe, 2000"
  // alpha = 0.0025, beta = 2, k = 0
  double alpha = 0.0025;
  double beta = 2;
  double k = 0;
  lambda_ = alpha * alpha * (n_x_ + k) - n_x_;
  double weight_s_0 = lambda_ / (lambda_ + n_x_);
  double weight_c_0 = lambda_ / (lambda_ + n_x_) + (1 - alpha * alpha + beta);
  weights_s_(0) = weight_s_0;
  weights_c_(0) = weight_c_0;
  for (int i = 1; i < 2 * n_x_ + 1; i++)
  {  // 2n+1 weights
    double weight = 0.5 / (n_x_ + lambda_);
    weights_s_(i) = weight;
    weights_c_(i) = weight;
  }

  // init timestamp
  time_ = timestamp;

  x_merge_(0) = z(0);
  x_merge_(1) = z(1);

  z_pred_cv_(0) = z(0);
  z_pred_cv_(1) = z(1);

  z_pred_ctrv_(0) = z(0);
  z_pred_ctrv_(1) = z(1);

  z_pred_rm_(0) = z(0);
  z_pred_rm_(1) = z(1);

  x_cv_ = x_ctrv_ = x_rm_ = x_merge_;
  p_cv_ = p_ctrv_ = p_rm_ = p_merge_;

  s_cv_ << 1, 0, 0, 1;
  s_ctrv_ << 1, 0, 0, 1;
  s_rm_ << 1, 0, 0, 1;

  // initialize R covariance
  r_cv_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
  r_ctrv_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
  r_rm_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

  // init tracking num
  tracking_num_ = 1;
}

void UKF::updateModeProb(const std::vector<double>& lambda_vec)
{
  double cvGauss = lambda_vec[0];
  double ctrvGauss = lambda_vec[1];
  double rmGauss = lambda_vec[2];
  double sumGauss = cvGauss * mode_prob_cv_ + ctrvGauss * mode_prob_ctrv_ + rmGauss * mode_prob_rm_;
  mode_prob_cv_ = (cvGauss * mode_prob_cv_) / sumGauss;
  mode_prob_ctrv_ = (ctrvGauss * mode_prob_ctrv_) / sumGauss;
  mode_prob_rm_ = (rmGauss * mode_prob_rm_) / sumGauss;
  // prevent each prob from becoming 0
  if (fabs(mode_prob_cv_) < 0.0001)
    mode_prob_cv_ = 0.0001;
  if (fabs(mode_prob_ctrv_) < 0.0001)
    mode_prob_ctrv_ = 0.0001;
  if (fabs(mode_prob_rm_) < 0.0001)
    mode_prob_rm_ = 0.0001;
}

void UKF::updateYawWithHighProb()
{
  if (mode_prob_cv_ > mode_prob_ctrv_)
  {
    if (mode_prob_cv_ > mode_prob_rm_)
    {
      x_merge_yaw_ = x_cv_(3);
    }
    else
    {
      x_merge_yaw_ = x_rm_(3);
    }
  }
  else
  {
    if (mode_prob_ctrv_ > mode_prob_rm_)
    {
      x_merge_yaw_ = x_ctrv_(3);
    }
    else
    {
      x_merge_yaw_ = x_rm_(3);
    }
  }
  x_merge_(3) = x_merge_yaw_;
}

void UKF::mergeEstimationAndCovariance()
{
  x_merge_ = mode_prob_cv_ * x_cv_ + mode_prob_ctrv_ * x_ctrv_ + mode_prob_rm_ * x_rm_;
  while (x_merge_(3) > M_PI)
    x_merge_(3) -= 2. * M_PI;
  while (x_merge_(3) < -M_PI)
    x_merge_(3) += 2. * M_PI;

  // not interacting yaw(-pi ~ pi)
  updateYawWithHighProb();

  p_merge_ = mode_prob_cv_ * (p_cv_ + (x_cv_ - x_merge_) * (x_cv_ - x_merge_).transpose()) +
             mode_prob_ctrv_ * (p_ctrv_ + (x_ctrv_ - x_merge_) * (x_ctrv_ - x_merge_).transpose()) +
             mode_prob_rm_ * (p_rm_ + (x_rm_ - x_merge_) * (x_rm_ - x_merge_).transpose());
}

void UKF::mixingProbability()
{
  double sumProb1 = mode_prob_cv_ * p1_[0] + mode_prob_ctrv_ * p2_[0] + mode_prob_rm_ * p3_[0];
  double sumProb2 = mode_prob_cv_ * p1_[1] + mode_prob_ctrv_ * p2_[1] + mode_prob_rm_ * p3_[1];
  double sumProb3 = mode_prob_cv_ * p1_[2] + mode_prob_ctrv_ * p2_[2] + mode_prob_rm_ * p3_[2];
  mode_match_prob_cv2cv_ = mode_prob_cv_ * p1_[0] / sumProb1;
  mode_match_prob_ctrv2cv_ = mode_prob_ctrv_ * p2_[0] / sumProb1;
  mode_match_prob_rm2cv_ = mode_prob_rm_ * p3_[0] / sumProb1;

  mode_match_prob_cv2ctrv_ = mode_prob_cv_ * p1_[1] / sumProb2;
  mode_match_prob_ctrv2ctrv_ = mode_prob_ctrv_ * p2_[1] / sumProb2;
  mode_match_prob_rm2ctrv_ = mode_prob_rm_ * p3_[1] / sumProb2;

  mode_match_prob_cv2rm_ = mode_prob_cv_ * p1_[2] / sumProb3;
  mode_match_prob_ctrv2rm_ = mode_prob_ctrv_ * p2_[2] / sumProb3;
  mode_match_prob_rm2rm_ = mode_prob_rm_ * p3_[2] / sumProb3;
}

void UKF::interaction()
{
  Eigen::MatrixXd x_pre_cv = x_cv_;
  Eigen::MatrixXd x_pre_ctrv = x_ctrv_;
  Eigen::MatrixXd x_pre_rm = x_rm_;
  Eigen::MatrixXd p_pre_cv = p_cv_;
  Eigen::MatrixXd p_pre_ctrv = p_ctrv_;
  Eigen::MatrixXd p_pre_rm = p_rm_;
  x_cv_ = mode_match_prob_cv2cv_ * x_pre_cv + mode_match_prob_ctrv2cv_ * x_pre_ctrv + mode_match_prob_rm2cv_ * x_pre_rm;
  x_ctrv_ = mode_match_prob_cv2ctrv_ * x_pre_cv + mode_match_prob_ctrv2ctrv_ * x_pre_ctrv +
            mode_match_prob_rm2ctrv_ * x_pre_rm;
  x_rm_ = mode_match_prob_cv2rm_ * x_pre_cv + mode_match_prob_ctrv2rm_ * x_pre_ctrv + mode_match_prob_rm2rm_ * x_pre_rm;

  // not interacting yaw(-pi ~ pi)
  x_cv_(3) = x_pre_cv(3);
  x_ctrv_(3) = x_pre_ctrv(3);
  x_rm_(3) = x_pre_rm(3);

  // normalizing angle
  while (x_cv_(3) > M_PI)
    x_cv_(3) -= 2. * M_PI;
  while (x_cv_(3) < -M_PI)
    x_cv_(3) += 2. * M_PI;
  while (x_ctrv_(3) > M_PI)
    x_ctrv_(3) -= 2. * M_PI;
  while (x_ctrv_(3) < -M_PI)
    x_ctrv_(3) += 2. * M_PI;
  while (x_rm_(3) > M_PI)
    x_rm_(3) -= 2. * M_PI;
  while (x_rm_(3) < -M_PI)
    x_rm_(3) += 2. * M_PI;

  p_cv_ = mode_match_prob_cv2cv_ * (p_pre_cv + (x_pre_cv - x_cv_) * (x_pre_cv - x_cv_).transpose()) +
          mode_match_prob_ctrv2cv_ * (p_pre_ctrv + (x_pre_ctrv - x_cv_) * (x_pre_ctrv - x_cv_).transpose()) +
          mode_match_prob_rm2cv_ * (p_pre_rm + (x_pre_rm - x_cv_) * (x_pre_rm - x_cv_).transpose());
  p_ctrv_ = mode_match_prob_cv2ctrv_ * (p_pre_cv + (x_pre_cv - x_ctrv_) * (x_pre_cv - x_ctrv_).transpose()) +
            mode_match_prob_ctrv2ctrv_ * (p_pre_ctrv + (x_pre_ctrv - x_ctrv_) * (x_pre_ctrv - x_ctrv_).transpose()) +
            mode_match_prob_rm2ctrv_ * (p_pre_rm + (x_pre_rm - x_ctrv_) * (x_pre_rm - x_ctrv_).transpose());
  p_rm_ = mode_match_prob_cv2rm_ * (p_pre_cv + (x_pre_cv - x_rm_) * (x_pre_cv - x_rm_).transpose()) +
          mode_match_prob_ctrv2rm_ * (p_pre_ctrv + (x_pre_ctrv - x_rm_) * (x_pre_ctrv - x_rm_).transpose()) +
          mode_match_prob_rm2rm_ * (p_pre_rm + (x_pre_rm - x_rm_) * (x_pre_rm - x_rm_).transpose());
}

void UKF::predictionSUKF(const double dt)
{
  /*****************************************************************************
  *  Init covariance Q if it is necessary
  ****************************************************************************/
  initCovarQs(dt, x_merge_(3));
  /*****************************************************************************
  *  Prediction
  ****************************************************************************/
  prediction(dt, MotionModel::CTRV);
  /*****************************************************************************
  *  Update
  ****************************************************************************/
  updateLidar(MotionModel::CTRV);
}

void UKF::predictionIMMUKF(const double dt)
{
  /*****************************************************************************
  *  Init covariance Q if it is needed
  ****************************************************************************/
  initCovarQs(dt, x_merge_(3));
  /*****************************************************************************
  *  IMM Mixing and Interaction
  ****************************************************************************/
  mixingProbability();
  interaction();
  /*****************************************************************************
  *  Prediction
  ****************************************************************************/
  prediction(dt, MotionModel::CV);
  prediction(dt, MotionModel::CTRV);
  prediction(dt, MotionModel::RM);
  /*****************************************************************************
  *  Update
  ****************************************************************************/
  updateLidar(MotionModel::CV);
  updateLidar(MotionModel::CTRV);
  updateLidar(MotionModel::RM);
}

void UKF::findMaxZandS(Eigen::VectorXd& max_det_z, Eigen::MatrixXd& max_det_s)
{
  double cv_det = s_cv_.determinant();
  double ctrv_det = s_ctrv_.determinant();
  double rm_det = s_rm_.determinant();

  if (cv_det > ctrv_det)
  {
    if (cv_det > rm_det)
    {
      max_det_z = z_pred_cv_;
      max_det_s = s_cv_;
    }
    else
    {
      max_det_z = z_pred_rm_;
      max_det_s = s_rm_;
    }
  }
  else
  {
    if (ctrv_det > rm_det)
    {
      max_det_z = z_pred_ctrv_;
      max_det_s = s_ctrv_;
    }
    else
    {
      max_det_z = z_pred_rm_;
      max_det_s = s_rm_;
    }
  }
}

void UKF::updateEachMotion(const double detection_probability, const double gate_probability, const double gating_thres,
                           const std::vector<autoware_msgs::DetectedObject>& object_vec,
                           std::vector<double>& lambda_vec)
{
  // calculating association probability
  double num_meas = object_vec.size();
  double b = 2 * num_meas * (1 - detection_probability * gate_probability) / (gating_thres * detection_probability);
  double e_cv_sum = 0;
  double e_ctrv_sum = 0;
  double e_rm_sum = 0;

  std::vector<double> e_cv_vec;
  std::vector<double> e_ctrv_vec;
  std::vector<double> e_rm_vec;

  std::vector<Eigen::VectorXd> diff_cv_vec;
  std::vector<Eigen::VectorXd> diff_ctrv_vec;
  std::vector<Eigen::VectorXd> diff_rm_vec;

  std::vector<Eigen::VectorXd> meas_vec;

  for (size_t i = 0; i < num_meas; i++)
  {
    Eigen::VectorXd meas = Eigen::VectorXd(2);
    meas(0) = object_vec[i].pose.position.x;
    meas(1) = object_vec[i].pose.position.y;
    meas_vec.push_back(meas);

    Eigen::VectorXd diff_cv = meas - z_pred_cv_;
    Eigen::VectorXd diff_ctrv = meas - z_pred_ctrv_;
    Eigen::VectorXd diff_rm = meas - z_pred_rm_;

    diff_cv_vec.push_back(diff_cv);
    diff_ctrv_vec.push_back(diff_ctrv);
    diff_rm_vec.push_back(diff_rm);

    double e_cv = exp(-0.5 * diff_cv.transpose() * s_cv_.inverse() * diff_cv);
    double e_ctrv = exp(-0.5 * diff_ctrv.transpose() * s_ctrv_.inverse() * diff_ctrv);
    double e_rm = exp(-0.5 * diff_rm.transpose() * s_rm_.inverse() * diff_rm);

    e_cv_vec.push_back(e_cv);
    e_ctrv_vec.push_back(e_ctrv);
    e_rm_vec.push_back(e_rm);

    e_cv_sum += e_cv;
    e_ctrv_sum += e_ctrv;
    e_rm_sum += e_rm;
  }
  double beta_cv_zero = b / (b + e_cv_sum);
  double beta_ctrv_zero = b / (b + e_ctrv_sum);
  double beta_rm_zero = b / (b + e_rm_sum);

  std::vector<double> beta_cv;
  std::vector<double> beta_ctrv;
  std::vector<double> beta_rm;

  if (num_meas != 0)
  {
    std::vector<double>::iterator max_cv_iter = std::max_element(e_cv_vec.begin(), e_cv_vec.end());
    std::vector<double>::iterator max_ctrv_iter = std::max_element(e_ctrv_vec.begin(), e_ctrv_vec.end());
    std::vector<double>::iterator max_rm_iter = std::max_element(e_rm_vec.begin(), e_rm_vec.end());
    int max_cv_ind = std::distance(e_cv_vec.begin(), max_cv_iter);
    int max_ctrv_ind = std::distance(e_ctrv_vec.begin(), max_ctrv_iter);
    int max_rm_ind = std::distance(e_rm_vec.begin(), max_rm_iter);
    cv_meas_ = meas_vec[max_cv_ind];
    ctrv_meas_ = meas_vec[max_ctrv_ind];
    rm_meas_ = meas_vec[max_rm_ind];
  }

  for (size_t i = 0; i < num_meas; i++)
  {
    double temp_cv = e_cv_vec[i] / (b + e_cv_sum);
    double temp_ctrv = e_ctrv_vec[i] / (b + e_ctrv_sum);
    double temp_rm = e_rm_vec[i] / (b + e_rm_sum);

    beta_cv.push_back(temp_cv);
    beta_ctrv.push_back(temp_ctrv);
    beta_rm.push_back(temp_rm);
  }
  Eigen::VectorXd sigma_x_cv;
  Eigen::VectorXd sigma_x_ctrv;
  Eigen::VectorXd sigma_x_rm;
  sigma_x_cv.setZero(2);
  sigma_x_ctrv.setZero(2);
  sigma_x_rm.setZero(2);

  for (size_t i = 0; i < num_meas; i++)
  {
    sigma_x_cv += beta_cv[i] * diff_cv_vec[i];
    sigma_x_ctrv += beta_ctrv[i] * diff_ctrv_vec[i];
    sigma_x_rm += beta_rm[i] * diff_rm_vec[i];
  }

  Eigen::MatrixXd sigma_p_cv;
  Eigen::MatrixXd sigma_p_ctrv;
  Eigen::MatrixXd sigma_p_rm;
  sigma_p_cv.setZero(2, 2);
  sigma_p_ctrv.setZero(2, 2);
  sigma_p_rm.setZero(2, 2);

  for (size_t i = 0; i < num_meas; i++)
  {
    sigma_p_cv += (beta_cv[i] * diff_cv_vec[i] * diff_cv_vec[i].transpose() - sigma_x_cv * sigma_x_cv.transpose());
    sigma_p_ctrv +=
        (beta_ctrv[i] * diff_ctrv_vec[i] * diff_ctrv_vec[i].transpose() - sigma_x_ctrv * sigma_x_ctrv.transpose());
    sigma_p_rm += (beta_rm[i] * diff_rm_vec[i] * diff_rm_vec[i].transpose() - sigma_x_rm * sigma_x_rm.transpose());
  }

  // update x and P
  x_cv_ = x_cv_ + k_cv_ * sigma_x_cv;
  x_ctrv_ = x_ctrv_ + k_ctrv_ * sigma_x_ctrv;
  x_rm_ = x_rm_ + k_rm_ * sigma_x_rm;

  while (x_cv_(3) > M_PI)
    x_cv_(3) -= 2. * M_PI;
  while (x_cv_(3) < -M_PI)
    x_cv_(3) += 2. * M_PI;
  while (x_ctrv_(3) > M_PI)
    x_ctrv_(3) -= 2. * M_PI;
  while (x_ctrv_(3) < -M_PI)
    x_ctrv_(3) += 2. * M_PI;
  while (x_rm_(3) > M_PI)
    x_rm_(3) -= 2. * M_PI;
  while (x_rm_(3) < -M_PI)
    x_rm_(3) += 2. * M_PI;

  Eigen::MatrixXd p_pre_cv = p_cv_;
  Eigen::MatrixXd p_pre_ctrv = p_ctrv_;
  Eigen::MatrixXd p_pre_rm = p_rm_;

  if (num_meas != 0)
  {
    p_cv_ = beta_cv_zero * p_pre_cv + (1 - beta_cv_zero) * (p_pre_cv - k_cv_ * s_cv_ * k_cv_.transpose()) +
            k_cv_ * sigma_p_cv * k_cv_.transpose();
    p_ctrv_ = beta_ctrv_zero * p_pre_ctrv +
              (1 - beta_ctrv_zero) * (p_pre_ctrv - k_ctrv_ * s_ctrv_ * k_ctrv_.transpose()) +
              k_ctrv_ * sigma_p_ctrv * k_ctrv_.transpose();
    p_rm_ = beta_rm_zero * p_pre_rm + (1 - beta_rm_zero) * (p_pre_rm - k_rm_ * s_rm_ * k_rm_.transpose()) +
            k_rm_ * sigma_p_rm * k_rm_.transpose();
  }
  else
  {
    p_cv_ = p_pre_cv - k_cv_ * s_cv_ * k_cv_.transpose();
    p_ctrv_ = p_pre_ctrv - k_ctrv_ * s_ctrv_ * k_ctrv_.transpose();
    p_rm_ = p_pre_rm - k_rm_ * s_rm_ * k_rm_.transpose();
  }

  Eigen::VectorXd max_det_z;
  Eigen::MatrixXd max_det_s;

  findMaxZandS(max_det_z, max_det_s);
  double Vk = M_PI * sqrt(gating_thres * max_det_s.determinant());

  double lambda_cv, lambda_ctrv, lambda_rm;
  if (num_meas != 0)
  {
    lambda_cv =
        (1 - gate_probability * detection_probability) / pow(Vk, num_meas) +
        detection_probability * pow(Vk, 1 - num_meas) * e_cv_sum / (num_meas * sqrt(2 * M_PI * s_cv_.determinant()));
    lambda_ctrv = (1 - gate_probability * detection_probability) / pow(Vk, num_meas) +
                  detection_probability * pow(Vk, 1 - num_meas) * e_ctrv_sum /
                      (num_meas * sqrt(2 * M_PI * s_ctrv_.determinant()));
    lambda_rm =
        (1 - gate_probability * detection_probability) / pow(Vk, num_meas) +
        detection_probability * pow(Vk, 1 - num_meas) * e_rm_sum / (num_meas * sqrt(2 * M_PI * s_rm_.determinant()));
  }
  else
  {
    lambda_cv = (1 - gate_probability * detection_probability);
    lambda_ctrv = (1 - gate_probability * detection_probability);
    lambda_rm = (1 - gate_probability * detection_probability);
  }

  lambda_vec.push_back(lambda_cv);
  lambda_vec.push_back(lambda_ctrv);
  lambda_vec.push_back(lambda_rm);
}

void UKF::updateLikelyMeasurementForCTRV(const std::vector<autoware_msgs::DetectedObject>& object_vec)
{
  double num_meas = object_vec.size();
  std::vector<double> e_ctrv_vec;
  std::vector<Eigen::VectorXd> meas_vec;

  for (size_t i = 0; i < num_meas; i++)
  {
    Eigen::VectorXd meas = Eigen::VectorXd(2);
    meas(0) = object_vec[i].pose.position.x;
    meas(1) = object_vec[i].pose.position.y;
    meas_vec.push_back(meas);
    Eigen::VectorXd diff_ctrv = meas - z_pred_ctrv_;
    double e_ctrv = exp(-0.5 * diff_ctrv.transpose() * s_ctrv_.inverse() * diff_ctrv);
    e_ctrv_vec.push_back(e_ctrv);
  }
  // for noise estimation
  if (num_meas != 0)
  {
    std::vector<double>::iterator max_ctrv_iter = std::max_element(e_ctrv_vec.begin(), e_ctrv_vec.end());
    int max_ctrv_ind = std::distance(e_ctrv_vec.begin(), max_ctrv_iter);
    ctrv_meas_ = meas_vec[max_ctrv_ind];
  }
}

void UKF::updateSUKF(const std::vector<autoware_msgs::DetectedObject>& object_vec)
{
  if (object_vec.size() == 0)
  {
    return;
  }

  // get most likely measurement ctrv_meas_
  updateLikelyMeasurementForCTRV(object_vec);

  Eigen::VectorXd z = Eigen::VectorXd(2);
  z << ctrv_meas_(0), ctrv_meas_(1);

  Eigen::VectorXd x = x_ctrv_.col(0);
  Eigen::VectorXd z_pred = z_pred_ctrv_;
  Eigen::MatrixXd k = k_ctrv_;
  Eigen::MatrixXd p = p_ctrv_;
  Eigen::MatrixXd s = s_ctrv_;

  x_ctrv_.col(0) = x + k * (z - z_pred);
  p_ctrv_ = p - k * s_ctrv_ * k.transpose();
  x_merge_.col(0) = x_ctrv_.col(0);
}

void UKF::updateIMMUKF(const double detection_probability, const double gate_probability, const double gating_thres,
                       const std::vector<autoware_msgs::DetectedObject>& object_vec)
{
  /*****************************************************************************
  *  IMM Update
  ****************************************************************************/
  // update each motion's x and p
  std::vector<double> lambda_vec;
  updateEachMotion(detection_probability, gate_probability, gating_thres, object_vec, lambda_vec);
  /*****************************************************************************
  *  IMM Merge Step
  ****************************************************************************/
  updateModeProb(lambda_vec);
  mergeEstimationAndCovariance();
}

void UKF::ctrv(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
               const double delta_t, std::vector<double>& state)
{
  // predicted state values
  double px_p, py_p;

  // avoid division by zero
  if (fabs(yawd) > 0.001)
  {
    px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
    py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
  }
  else
  {
    px_p = p_x + v * delta_t * cos(yaw);
    py_p = p_y + v * delta_t * sin(yaw);
  }
  double v_p = v;
  double yaw_p = yaw + yawd * delta_t;
  double yawd_p = yawd;

  while (yaw_p > M_PI)
    yaw_p -= 2. * M_PI;
  while (yaw_p < -M_PI)
    yaw_p += 2. * M_PI;

  state[0] = px_p;
  state[1] = py_p;
  state[2] = v_p;
  state[3] = yaw_p;
  state[4] = yawd_p;
}

void UKF::cv(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
             const double delta_t, std::vector<double>& state)
{
  // predicted state values
  double px_p = p_x + v * cos(yaw) * delta_t;
  double py_p = p_y + v * sin(yaw) * delta_t;

  double v_p = v;
  // not sure which one, works better in curve by using yaw
  double yaw_p = yaw;

  double yawd_p = yawd;

  state[0] = px_p;
  state[1] = py_p;
  state[2] = v_p;
  state[3] = yaw_p;
  state[4] = yawd_p;
}

void UKF::randomMotion(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
                       const double delta_t, std::vector<double>& state)
{
  double px_p = p_x;
  double py_p = p_y;
  double v_p = v * 0.9;  // aim to converge velocity for static objects
  // double v_p = 0.0;

  double yaw_p = yaw;
  double yawd_p = yawd;

  state[0] = px_p;
  state[1] = py_p;
  state[2] = v_p;
  state[3] = yaw_p;
  state[4] = yawd_p;
}

void UKF::initCovarQs(const double dt, const double yaw)
{
  if (tracking_num_ != TrackingState::Init)
  {
    return;
  }
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double cos_2_yaw = cos(yaw) * cos(yaw);
  double sin_2_yaw = sin(yaw) * sin(yaw);
  double cos_sin = cos_yaw * sin_yaw;

  double cv_var_a = std_a_cv_ * std_a_cv_;
  double cv_var_yawdd = std_cv_yawdd_ * std_cv_yawdd_;

  double ctrv_var_a = std_a_ctrv_ * std_a_ctrv_;
  double ctrv_var_yawdd = std_ctrv_yawdd_ * std_ctrv_yawdd_;

  double rm_var_a = std_a_rm_ * std_a_rm_;
  double rm_var_yawdd = std_rm_yawdd_ * std_rm_yawdd_;

  q_cv_ << 0.5 * 0.5 * dt_4 * cos_2_yaw * cv_var_a, 0.5 * 0.5 * dt_4 * cos_sin * cv_var_a,
      0.5 * dt_3 * cos_yaw * cv_var_a, 0, 0, 0.5 * 0.5 * dt_4 * cos_sin * cv_var_a,
      0.5 * 0.5 * dt_4 * sin_2_yaw * cv_var_a, 0.5 * dt_3 * sin_yaw * cv_var_a, 0, 0, 0.5 * dt_3 * cos_yaw * cv_var_a,
      0.5 * dt_3 * sin_yaw * cv_var_a, dt_2 * cv_var_a, 0, 0, 0, 0, 0, 0.5 * 0.5 * dt_4 * cv_var_yawdd,
      0.5 * dt_3 * cv_var_yawdd, 0, 0, 0, 0.5 * dt_3 * cv_var_yawdd, dt_2 * cv_var_yawdd;
  q_ctrv_ << 0.5 * 0.5 * dt_4 * cos_2_yaw * ctrv_var_a, 0.5 * 0.5 * dt_4 * cos_sin * ctrv_var_a,
      0.5 * dt_3 * cos_yaw * ctrv_var_a, 0, 0, 0.5 * 0.5 * dt_4 * cos_sin * ctrv_var_a,
      0.5 * 0.5 * dt_4 * sin_2_yaw * ctrv_var_a, 0.5 * dt_3 * sin_yaw * ctrv_var_a, 0, 0,
      0.5 * dt_3 * cos_yaw * ctrv_var_a, 0.5 * dt_3 * sin_yaw * ctrv_var_a, dt_2 * ctrv_var_a, 0, 0, 0, 0, 0,
      0.5 * 0.5 * dt_4 * ctrv_var_yawdd, 0.5 * dt_3 * ctrv_var_yawdd, 0, 0, 0, 0.5 * dt_3 * ctrv_var_yawdd,
      dt_2 * ctrv_var_yawdd;
  q_rm_ << 0.5 * 0.5 * dt_4 * cos_2_yaw * rm_var_a, 0.5 * 0.5 * dt_4 * cos_sin * rm_var_a,
      0.5 * dt_3 * cos_yaw * rm_var_a, 0, 0, 0.5 * 0.5 * dt_4 * cos_sin * rm_var_a,
      0.5 * 0.5 * dt_4 * sin_2_yaw * rm_var_a, 0.5 * dt_3 * sin_yaw * rm_var_a, 0, 0, 0.5 * dt_3 * cos_yaw * rm_var_a,
      0.5 * dt_3 * sin_yaw * rm_var_a, dt_2 * rm_var_a, 0, 0, 0, 0, 0, 0.5 * 0.5 * dt_4 * rm_var_yawdd,
      0.5 * dt_3 * rm_var_yawdd, 0, 0, 0, 0.5 * dt_3 * rm_var_yawdd, dt_2 * rm_var_yawdd;
}

void UKF::prediction(const double delta_t, const int model_ind)
{
  /*****************************************************************************
 *  Initialize model parameters
 ****************************************************************************/
  Eigen::MatrixXd x(x_cv_.rows(), 1);
  Eigen::MatrixXd p(p_cv_.rows(), p_cv_.cols());
  Eigen::MatrixXd q(p_cv_.rows(), p_cv_.cols());
  Eigen::MatrixXd x_sig_pred(x_sig_pred_cv_.rows(), x_sig_pred_cv_.cols());
  if (model_ind == MotionModel::CV)
  {
    x = x_cv_.col(0);
    p = p_cv_;
    q = q_cv_;
    x_sig_pred = x_sig_pred_cv_;
  }
  else if (model_ind == MotionModel::CTRV)
  {
    x = x_ctrv_.col(0);
    p = p_ctrv_;
    q = q_ctrv_;
    x_sig_pred = x_sig_pred_ctrv_;
  }
  else
  {
    x = x_rm_.col(0);
    p = p_rm_;
    q = q_rm_;
    x_sig_pred = x_sig_pred_rm_;
  }

  /*****************************************************************************
  *  Create Sigma Points
  ****************************************************************************/

  Eigen::MatrixXd x_sig = Eigen::MatrixXd(n_x_, 2 * n_x_ + 1);

  // create square root matrix
  Eigen::MatrixXd L = p.llt().matrixL();

  // create augmented sigma points
  x_sig.col(0) = x;
  for (int i = 0; i < n_x_; i++)
  {
    Eigen::VectorXd pred1 = x + sqrt(lambda_ + n_x_) * L.col(i);
    Eigen::VectorXd pred2 = x - sqrt(lambda_ + n_x_) * L.col(i);

    while (pred1(3) > M_PI)
      pred1(3) -= 2. * M_PI;
    while (pred1(3) < -M_PI)
      pred1(3) += 2. * M_PI;

    while (pred2(3) > M_PI)
      pred2(3) -= 2. * M_PI;
    while (pred2(3) < -M_PI)
      pred2(3) += 2. * M_PI;

    x_sig.col(i + 1) = pred1;
    x_sig.col(i + 1 + n_x_) = pred2;
  }

  /*****************************************************************************
  *  Predict Sigma Points
  ****************************************************************************/
  // predict sigma points
  for (int i = 0; i < 2 * n_x_ + 1; i++)
  {
    // extract values for better readability
    double p_x = x_sig(0, i);
    double p_y = x_sig(1, i);
    double v = x_sig(2, i);
    double yaw = x_sig(3, i);
    double yawd = x_sig(4, i);

    std::vector<double> state(5);
    if (model_ind == MotionModel::CV)
      cv(p_x, p_y, v, yaw, yawd, delta_t, state);
    else if (model_ind == MotionModel::CTRV)
      ctrv(p_x, p_y, v, yaw, yawd, delta_t, state);
    else
      randomMotion(p_x, p_y, v, yaw, yawd, delta_t, state);

    // write predicted sigma point into right column
    x_sig_pred(0, i) = state[0];
    x_sig_pred(1, i) = state[1];
    x_sig_pred(2, i) = state[2];
    x_sig_pred(3, i) = state[3];
    x_sig_pred(4, i) = state[4];
  }

  /*****************************************************************************
  *  Convert Predicted Sigma Points to Mean/Covariance
  ****************************************************************************/
  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_x_ + 1; i++)
  {  // iterate over sigma points
    x = x + weights_s_(i) * x_sig_pred.col(i);
  }

  while (x(3) > M_PI)
    x(3) -= 2. * M_PI;
  while (x(3) < -M_PI)
    x(3) += 2. * M_PI;
  // predicted state covariance matrix
  p.fill(0.0);
  for (int i = 0; i < 2 * n_x_ + 1; i++)
  {  // iterate over sigma points
    // state difference
    Eigen::VectorXd x_diff = x_sig_pred.col(i) - x;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    p = p + weights_c_(i) * x_diff * x_diff.transpose();
  }

  p = p + q;

  /*****************************************************************************
  *  Update model parameters
  ****************************************************************************/
  if (model_ind == MotionModel::CV)
  {
    x_cv_.col(0) = x;
    p_cv_ = p;
    x_sig_pred_cv_ = x_sig_pred;
  }
  else if (model_ind == MotionModel::CTRV)
  {
    x_ctrv_.col(0) = x;
    p_ctrv_ = p;
    x_sig_pred_ctrv_ = x_sig_pred;
  }
  else
  {
    x_rm_.col(0) = x;
    p_rm_ = p;
    x_sig_pred_rm_ = x_sig_pred;
  }
}

void UKF::updateLidar(const int model_ind)
{
  /*****************************************************************************
 *  Initialize model parameters
 ****************************************************************************/
  Eigen::VectorXd x(x_cv_.rows());
  Eigen::MatrixXd P(p_cv_.rows(), p_cv_.cols());
  Eigen::MatrixXd r(2, 2);
  Eigen::MatrixXd x_sig_pred(x_sig_pred_cv_.rows(), x_sig_pred_cv_.cols());
  if (model_ind == MotionModel::CV)
  {
    x = x_cv_.col(0);
    r = r_cv_;
    x_sig_pred = x_sig_pred_cv_;
  }
  else if (model_ind == MotionModel::CTRV)
  {
    x = x_ctrv_.col(0);
    r = r_ctrv_;
    x_sig_pred = x_sig_pred_ctrv_;
  }
  else
  {
    x = x_rm_.col(0);
    r = r_rm_;
    x_sig_pred = x_sig_pred_rm_;
  }

  // set measurement dimension, lidar can measure p_x and p_y
  int n_z = 2;

  // create matrix for sigma points in measurement space
  Eigen::MatrixXd z_sig = Eigen::MatrixXd(n_z, 2 * n_x_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_x_ + 1; i++)
  {  // 2n+1 simga points
    // extract values for better readibility
    double p_x = x_sig_pred(0, i);
    double p_y = x_sig_pred(1, i);

    // measurement model
    z_sig(0, i) = p_x;
    z_sig(1, i) = p_y;
  }

  // mean predicted measurement
  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_x_ + 1; i++)
  {
    z_pred = z_pred + weights_s_(i) * z_sig.col(i);
  }

  // measurement covariance matrix S
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_x_ + 1; i++)
  {  // 2n+1 simga points
    // residual
    Eigen::VectorXd z_diff = z_sig.col(i) - z_pred;
    S = S + weights_c_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  S = S + r;

  // create matrix for cross correlation Tc
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);

  /*****************************************************************************
  *  UKF Update for Lidar
  ****************************************************************************/
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_x_ + 1; i++)
  {  // 2n+1 simga points
    // residual
    Eigen::VectorXd z_diff = z_sig.col(i) - z_pred;
    // state difference
    Eigen::VectorXd x_diff = x_sig_pred.col(i) - x;

    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;

    Tc = Tc + weights_c_(i) * x_diff * z_diff.transpose();
  }

  Eigen::MatrixXd K = Tc * S.inverse();

  /*****************************************************************************
  *  Update model parameters
  ****************************************************************************/
  if (model_ind == MotionModel::CV)
  {
    x_cv_.col(0) = x;
    x_sig_pred_cv_ = x_sig_pred;
    z_pred_cv_ = z_pred;
    s_cv_ = S;
    k_cv_ = K;
  }
  else if (model_ind == MotionModel::CTRV)
  {
    x_ctrv_.col(0) = x;
    x_sig_pred_ctrv_ = x_sig_pred;
    z_pred_ctrv_ = z_pred;
    s_ctrv_ = S;
    k_ctrv_ = K;
  }
  else
  {
    x_rm_.col(0) = x;
    x_sig_pred_rm_ = x_sig_pred;
    z_pred_rm_ = z_pred;
    s_rm_ = S;
    k_rm_ = K;
  }
}
