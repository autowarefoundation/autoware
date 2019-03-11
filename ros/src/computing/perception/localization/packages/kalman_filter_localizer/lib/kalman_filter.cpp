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

#include "kalman_filter/kalman_filter.h"

KalmanFilter::KalmanFilter() {}
KalmanFilter::KalmanFilter(const Eigen::MatrixXd &x, const Eigen::MatrixXd &A,
                           const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
                           const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                           const Eigen::MatrixXd &P)
{
  init(x, A, B, C, Q, R, P);
}
KalmanFilter::~KalmanFilter() {}
void KalmanFilter::init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &A,
                        const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
                        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                        const Eigen::MatrixXd &P)
{
  x_ = x;
  A_ = A;
  B_ = B;
  C_ = C;
  Q_ = Q;
  R_ = R;
  P_ = P;
}
void KalmanFilter::init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P0)
{
  x_ = x;
  P_ = P0;
}

void KalmanFilter::setA(const Eigen::MatrixXd &A) { A_ = A; }
void KalmanFilter::setB(const Eigen::MatrixXd &B) { B_ = B; }
void KalmanFilter::setC(const Eigen::MatrixXd &C) { C_ = C; }
void KalmanFilter::setQ(const Eigen::MatrixXd &Q) { Q_ = Q; }
void KalmanFilter::setR(const Eigen::MatrixXd &R) { R_ = R; }
void KalmanFilter::getX(Eigen::MatrixXd &x) { x = x_; };
void KalmanFilter::getP(Eigen::MatrixXd &P) { P = P_; };
double KalmanFilter::getXelement(unsigned int i) { return x_(i); };

void KalmanFilter::predictXandP(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P)
{
  x_ = x;
  P_ = P;
}
bool KalmanFilter::predict(const Eigen::MatrixXd &x_next,
                           const Eigen::MatrixXd &A,
                           const Eigen::MatrixXd &Q)
{
  if (x_.cols() != x_next.cols() || A.rows() != P_.cols() ||
      Q.cols() != Q.rows() || A.cols() != Q.cols())
  {
    return false;
  }
  x_ = x_next;
  P_ = A * P_ * A.transpose() + Q;
  return true;
}
bool KalmanFilter::predict(const Eigen::MatrixXd &x_next,
                           const Eigen::MatrixXd &A)
{
  return predict(x_next, A, Q_);
}

bool KalmanFilter::predict(const Eigen::MatrixXd &u, const Eigen::MatrixXd &A,
                           const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q)
{
  if (A.rows() != x_.cols() || B.rows() != u.cols())
  {
    return false;
  }
  const Eigen::MatrixXd x_next = A * x_ + B * u;
  return predict(x_next, A, Q);
}
bool KalmanFilter::predict(const Eigen::MatrixXd &u) { return predict(u, A_, B_, Q_); }

bool KalmanFilter::update(const Eigen::MatrixXd &y,
                          const Eigen::MatrixXd &y_pred,
                          const Eigen::MatrixXd &C,
                          const Eigen::MatrixXd &R)
{
  if (P_.rows() != C.rows() || R.rows() != R.cols() || R.rows() != C.cols() ||
      y.cols() != y_pred.cols() || y.cols() != C.cols())
  {
    return false;
  }
  const Eigen::MatrixXd PCT = P_ * C.transpose();
  const Eigen::MatrixXd K = PCT * ((R + C * PCT).inverse());

  if (isnan(K.array()).any() || isinf(K.array()).any()) {
    return false;
  };

  x_ = x_ + K * (y - y_pred);
  P_ = P_ - K * (C * P_);
  return true;
}
// bool KalmanFilter::update(const Eigen::MatrixXd &y,
//                           const Eigen::MatrixXd &y_pred,
//                           const Eigen::MatrixXd &C)
// {
//   return update(y, y_pred, C, R_);
// }
bool KalmanFilter::update(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
                          const Eigen::MatrixXd &R)
{
  if (C.rows() != x_.cols())
  {
    return false;
  }
  const Eigen::MatrixXd y_pred = C * x_;
  return update(y, y_pred, C, R);
}
bool KalmanFilter::update(const Eigen::MatrixXd &y) { return update(y, C_, R_); }
