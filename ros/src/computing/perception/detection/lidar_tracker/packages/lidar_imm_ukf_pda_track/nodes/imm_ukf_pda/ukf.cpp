
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

  //------------------
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

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // Augmented sigma point spreading parameter
  lambda_aug_ = 3 - n_aug_;

  // predicted sigma points matrix
  x_sig_pred_cv_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  // predicted sigma points matrix
  x_sig_pred_ctrv_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  // predicted sigma points matrix
  x_sig_pred_rm_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  // create vector for weights
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);

  count_ = 0;
  count_empty_ = 0;

  ini_u_.push_back(0.33);
  ini_u_.push_back(0.33);
  ini_u_.push_back(0.33);

  // different from paper, might be wrong
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

  k_cv_ = Eigen::MatrixXd(2, 2);
  k_ctrv_ = Eigen::MatrixXd(2, 2);
  k_rm_ = Eigen::MatrixXd(2, 2);

  // gamma_g_ = 9.21;
  pd_ = 0.9;
  pg_ = 0.99;

  // track parameter
  lifetime_ = 0;
  is_static_ = false;

  // bounding box params
  is_best_jsk_bb_empty_ = false;
  is_vis_bb_ = false;
  best_yaw_ = 100;
  bb_yaw_ = 0;
  bb_area_ = 0;

  // for env classification
  init_meas_ = Eigen::VectorXd(2);
  dist_from_init_ = 0;

  x_merge_yaw_ = 0;
}

void UKF::initialize(const Eigen::VectorXd& z, const double timestamp)
{
  // first measurement
  x_merge_ << 1, 1, 0, 0, 0.1;

  // init covariance matrix
  p_merge_ << 0.5, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 1;

  // set weights
  double weight_0 = lambda_aug_ / (lambda_aug_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++)
  {  // 2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_aug_);
    weights_(i) = weight;
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

  // init tracking num
  tracking_num_ = 1;

  // prevent transform pose error, if the condition meets, target_.jskBB_ would be updated
  jsk_bb_.pose.orientation.x = 1.0;
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


void UKF::predictionIMMUKF(const double dt)
{
  /*****************************************************************************
  *  IMM Mixing and Interaction
  ****************************************************************************/
  mixingProbability();
  interaction();
  /*****************************************************************************
  *  Prediction
  ****************************************************************************/
  prediction(dt, 0);
  prediction(dt, 1);
  prediction(dt, 2);

  /*****************************************************************************
  *  Update
  ****************************************************************************/
  updateLidar(0);
  updateLidar(1);
  updateLidar(2);
}

void UKF::updateIMMUKF(const std::vector<double>& lambda_vec)
{
  /*****************************************************************************
  *  IMM Merge Step
  ****************************************************************************/
  updateModeProb(lambda_vec);
  mergeEstimationAndCovariance();
}

void UKF::ctrv(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
               const double nu_a, const double nu_yawdd, const double delta_t, std::vector<double>& state)
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

  // add noise
  px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
  py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
  v_p = v_p + nu_a * delta_t;

  yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
  yawd_p = yawd_p + nu_yawdd * delta_t;

  state[0] = px_p;
  state[1] = py_p;
  state[2] = v_p;
  state[3] = yaw_p;
  state[4] = yawd_p;
}

void UKF::cv(const double p_x, const double p_y, const double v, const double yaw, const double yawd, const double nu_a,
             const double nu_yawdd, const double delta_t, std::vector<double>& state)
{
  // predicted state values
  double px_p = p_x + v * cos(yaw) * delta_t;
  double py_p = p_y + v * sin(yaw) * delta_t;

  double v_p = v;
  // not sure which one, works better in curve by using yaw
  double yaw_p = yaw;

  double yawd_p = yawd;

  // add noise
  px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
  py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
  v_p = v_p + nu_a * delta_t;

  yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
  yawd_p = yawd_p + nu_yawdd * delta_t;

  state[0] = px_p;
  state[1] = py_p;
  state[2] = v_p;
  state[3] = yaw_p;
  state[4] = yawd_p;
}

void UKF::randomMotion(const double p_x, const double p_y, const double v, const double yaw, const double yawd,
                       const double nu_a, const double nu_yawdd, const double delta_t, std::vector<double>& state)
{
  double px_p = p_x;
  double py_p = p_y;
  double v_p = v;

  double yaw_p = yaw;
  double yawd_p = yawd;

  state[0] = px_p;
  state[1] = py_p;
  state[2] = v_p;
  state[3] = yaw_p;
  state[4] = yawd_p;
}


void UKF::prediction(const double delta_t, const int model_ind)
{
  /*****************************************************************************
 *  Initialize model parameters
 ****************************************************************************/
  double std_yawdd, std_a;
  Eigen::MatrixXd x_(x_cv_.rows(), 1);
  Eigen::MatrixXd p_(p_cv_.rows(), p_cv_.cols());
  Eigen::MatrixXd x_sig_pred_(x_sig_pred_cv_.rows(), x_sig_pred_cv_.cols());
  if (model_ind == 0)
  {
    x_ = x_cv_.col(0);
    p_ = p_cv_;
    x_sig_pred_ = x_sig_pred_cv_;
    std_yawdd = std_cv_yawdd_;
    std_a = std_a_cv_;
  }
  else if (model_ind == 1)
  {
    x_ = x_ctrv_.col(0);
    p_ = p_ctrv_;
    x_sig_pred_ = x_sig_pred_ctrv_;
    std_yawdd = std_ctrv_yawdd_;
    std_a = std_a_ctrv_;
  }
  else
  {
    x_ = x_rm_.col(0);
    p_ = p_rm_;
    x_sig_pred_ = x_sig_pred_rm_;
    std_yawdd = std_rm_yawdd_;
    std_a = std_a_rm_;
  }

  /*****************************************************************************
  *  Augment Sigma Points
  ****************************************************************************/
  // create augmented mean vector
  Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);

  // create augmented state covariance
  Eigen::MatrixXd p_aug = Eigen::MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  Eigen::MatrixXd x_sig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  p_aug.fill(0.0);
  p_aug.topLeftCorner(5, 5) = p_;
  p_aug(5, 5) = std_a * std_a;
  p_aug(6, 6) = std_yawdd * std_yawdd;

  // create square root matrix
  Eigen::MatrixXd L = p_aug.llt().matrixL();

  // create augmented sigma points
  x_sig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    x_sig_aug.col(i + 1) = x_aug + sqrt(lambda_aug_ + n_aug_) * L.col(i);
    x_sig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_aug_ + n_aug_) * L.col(i);
  }

  /*****************************************************************************
  *  Predict Sigma Points
  ****************************************************************************/
  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // extract values for better readability
    double p_x = x_sig_aug(0, i);
    double p_y = x_sig_aug(1, i);
    double v = x_sig_aug(2, i);
    double yaw = x_sig_aug(3, i);
    double yawd = x_sig_aug(4, i);
    double nu_a = x_sig_aug(5, i);
    double nu_yawdd = x_sig_aug(6, i);

    std::vector<double> state(5);
    if (model_ind == 0)
      cv(p_x, p_y, v, yaw, yawd, nu_a, nu_yawdd, delta_t, state);
    else if (model_ind == 1)
      ctrv(p_x, p_y, v, yaw, yawd, nu_a, nu_yawdd, delta_t, state);
    else
      randomMotion(p_x, p_y, v, yaw, yawd, nu_a, nu_yawdd, delta_t, state);

    // write predicted sigma point into right column
    x_sig_pred_(0, i) = state[0];
    x_sig_pred_(1, i) = state[1];
    x_sig_pred_(2, i) = state[2];
    x_sig_pred_(3, i) = state[3];
    x_sig_pred_(4, i) = state[4];
  }

  /*****************************************************************************
  *  Convert Predicted Sigma Points to Mean/Covariance
  ****************************************************************************/
  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {  // iterate over sigma points
    x_ = x_ + weights_(i) * x_sig_pred_.col(i);
  }

  while (x_(3) > M_PI)
    x_(3) -= 2. * M_PI;
  while (x_(3) < -M_PI)
    x_(3) += 2. * M_PI;
  // predicted state covariance matrix
  p_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {  // iterate over sigma points
    // state difference
    Eigen::VectorXd x_diff = x_sig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    p_ = p_ + weights_(i) * x_diff * x_diff.transpose();
  }

  /*****************************************************************************
  *  Update model parameters
  ****************************************************************************/
  if (model_ind == 0)
  {
    x_cv_.col(0) = x_;
    p_cv_ = p_;
    x_sig_pred_cv_ = x_sig_pred_;
  }
  else if (model_ind == 1)
  {
    x_ctrv_.col(0) = x_;
    p_ctrv_ = p_;
    x_sig_pred_ctrv_ = x_sig_pred_;
  }
  else
  {
    x_rm_.col(0) = x_;
    p_rm_ = p_;
    x_sig_pred_rm_ = x_sig_pred_;
  }
}


void UKF::updateLidar(const int model_ind)
{
  /*****************************************************************************
 *  Initialize model parameters
 ****************************************************************************/
  Eigen::VectorXd x(x_cv_.rows());
  Eigen::MatrixXd P(p_cv_.rows(), p_cv_.cols());
  Eigen::MatrixXd x_sig_pred(x_sig_pred_cv_.rows(), x_sig_pred_cv_.cols());
  if (model_ind == 0)
  {
    x = x_cv_.col(0);
    P = p_cv_;
    x_sig_pred = x_sig_pred_cv_;
  }
  else if (model_ind == 1)
  {
    x = x_ctrv_.col(0);
    P = p_ctrv_;
    x_sig_pred = x_sig_pred_ctrv_;
  }
  else
  {
    x = x_rm_.col(0);
    P = p_rm_;
    x_sig_pred = x_sig_pred_rm_;
  }

  // set measurement dimension, lidar can measure p_x and p_y
  int n_z = 2;

  // create matrix for sigma points in measurement space
  Eigen::MatrixXd z_sig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
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
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * z_sig.col(i);
  }

  // measurement covariance matrix S
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {  // 2n+1 simga points
    // residual
    Eigen::VectorXd z_diff = z_sig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
  S = S + R;

  // create matrix for cross correlation Tc
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);

  /*****************************************************************************
  *  UKF Update for Lidar
  ****************************************************************************/
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {  // 2n+1 simga points
    // residual
    Eigen::VectorXd z_diff = z_sig.col(i) - z_pred;
    // state difference
    Eigen::VectorXd x_diff = x_sig_pred.col(i) - x;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  Eigen::MatrixXd K = Tc * S.inverse();

  /*****************************************************************************
  *  Update model parameters
  ****************************************************************************/
  if (model_ind == 0)
  {
    x_cv_.col(0) = x;
    p_cv_ = P;
    x_sig_pred_cv_ = x_sig_pred;
    z_pred_cv_ = z_pred;
    s_cv_ = S;
    k_cv_ = K;
  }
  else if (model_ind == 1)
  {
    x_ctrv_.col(0) = x;
    p_ctrv_ = P;
    x_sig_pred_ctrv_ = x_sig_pred;
    z_pred_ctrv_ = z_pred;
    s_ctrv_ = S;
    k_ctrv_ = K;
  }
  else
  {
    x_rm_.col(0) = x;
    p_rm_ = P;
    x_sig_pred_rm_ = x_sig_pred;
    z_pred_rm_ = z_pred;
    s_rm_ = S;
    k_rm_ = K;
  }
}
