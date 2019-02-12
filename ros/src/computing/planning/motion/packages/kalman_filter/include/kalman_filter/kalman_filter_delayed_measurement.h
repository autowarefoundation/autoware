#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "kalman_filter/kalman_filter.h"

#include <iostream>
#define PRINT_MAT(X) std::cout << #X << ":\n"    \
                               << X << std::endl \
                               << std::endl

class KalmanFilterDelayedMeasurement : public KalmanFilter
{
public:
  KalmanFilterDelayedMeasurement();

  void init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P0, const int max_delay_step);

  void getCurrentX(Eigen::MatrixXd &x);
  void getCurrentP(Eigen::MatrixXd &P);

  void predictDelayedEKF(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A,
                         const Eigen::MatrixXd &Q);
  void updateDelayedEKF(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
                        const Eigen::MatrixXd &R, const int delay_step);

private:
  int max_delay_step_;
  int dim_x_;
  int dim_x_ex_;
  int dim_y_;
};

KalmanFilterDelayedMeasurement::KalmanFilterDelayedMeasurement() {}

void KalmanFilterDelayedMeasurement::init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P0,
                                          const int max_delay_step)
{
   
  max_delay_step_ = max_delay_step;
  dim_x_ = x.rows();
  printf("dim_x_ = %d\n",dim_x_);
  dim_x_ex_ = dim_x_ * max_delay_step;

     
  x_ = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
  P_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
   
  for (int i = 0; i < max_delay_step_; ++i)
  {
    x_.block(i * dim_x_, 0, dim_x_, 1) = x;
    P_.block(i * dim_x_, i * dim_x_, dim_x_, dim_x_) = P0;
  }

  PRINT_MAT(x_);
  PRINT_MAT(P_);
   
};

void KalmanFilterDelayedMeasurement::getCurrentX(Eigen::MatrixXd &x) { x = x_.block(0, 0, dim_x_, 1); };
void KalmanFilterDelayedMeasurement::getCurrentP(Eigen::MatrixXd &P) { P = P_.block(0, 0, dim_x_, dim_x_); };

void KalmanFilterDelayedMeasurement::predictDelayedEKF(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A,
                                                       const Eigen::MatrixXd &Q)
{ 
  int d_dim_x = dim_x_ex_ - dim_x_;

  /* slide states in the time direction */
  Eigen::MatrixXd x_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
  x_tmp.block(0, 0, dim_x_, 1) = x_next;
  x_tmp.block(dim_x_, 0, d_dim_x, 1) = x_.block(0, 0, d_dim_x, 1);
  x_ = x_tmp;

  Eigen::MatrixXd Q_ex = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  Q_ex.block(0, 0, dim_x_, dim_x_) = Q;
 
  /* update P with A matrix structure */
  Eigen::MatrixXd P_next = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  P_next.block(0, 0, dim_x_, dim_x_) = A * P_.block(0, 0, dim_x_, dim_x_) * A.transpose();
  P_next.block(0, dim_x_, dim_x_, d_dim_x) = A * P_.block(0, 0, dim_x_, d_dim_x);
  P_next.block(dim_x_, 0, d_dim_x, dim_x_) = P_.block(0, 0, d_dim_x, dim_x_) * A.transpose();
  P_next.block(dim_x_, dim_x_, d_dim_x, d_dim_x) = P_.block(0, 0, d_dim_x, d_dim_x);
  P_next += Q_ex;
  P_ = P_next;

   
};

void KalmanFilterDelayedMeasurement::updateDelayedEKF(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
                                                      const Eigen::MatrixXd &R, const int delay_step)
{

  const int dim_y = y.rows();

  /* set measurement matrix */
  Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
  C_ex.block(0, dim_x_ * delay_step, dim_y, dim_x_) = C;

  Eigen::MatrixXd y_pred = C_ex * x_;

  /* update */
  const Eigen::MatrixXd S_inv = (R + C_ex * P_ * C_ex.transpose()).inverse();
  const Eigen::MatrixXd K = P_ * C_ex.transpose() * S_inv;
  x_ = x_ + K * (y - y_pred);
  P_ = P_ - (P_ * C_ex.transpose()) * S_inv * (C_ex * P_);
};


