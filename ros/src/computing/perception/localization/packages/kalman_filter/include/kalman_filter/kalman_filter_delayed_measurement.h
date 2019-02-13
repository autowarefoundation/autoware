#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "kalman_filter/kalman_filter.h"

class KalmanFilterDelayedMeasurement : public KalmanFilter
{
public:
  KalmanFilterDelayedMeasurement();

  void init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P0, const int max_delay_step);

  void getCurrentX(Eigen::MatrixXd &x);
  void getCurrentP(Eigen::MatrixXd &P);

  void predictDelayedEKF(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A,
                         const Eigen::MatrixXd &Q);
  bool updateDelayedEKF(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
                        const Eigen::MatrixXd &R, const int delay_step);

private:
  int max_delay_step_;
  int dim_x_;
  int dim_x_ex_;
};