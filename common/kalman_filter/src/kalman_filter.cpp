// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kalman_filter/kalman_filter.hpp"

KalmanFilter::KalmanFilter() {}
KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & x, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & P)
{
  init(x, A, B, C, Q, R, P);
}
KalmanFilter::~KalmanFilter() {}
bool KalmanFilter::init(
  const Eigen::MatrixXd & x, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & P)
{
  if (
    x.cols() == 0 || x.rows() == 0 || A.cols() == 0 || A.rows() == 0 || B.cols() == 0 ||
    B.rows() == 0 || C.cols() == 0 || C.rows() == 0 || Q.cols() == 0 || Q.rows() == 0 ||
    R.cols() == 0 || R.rows() == 0 || P.cols() == 0 || P.rows() == 0) {
    return false;
  }
  x_ = x;
  A_ = A;
  B_ = B;
  C_ = C;
  Q_ = Q;
  R_ = R;
  P_ = P;
  return true;
}
bool KalmanFilter::init(const Eigen::MatrixXd & x, const Eigen::MatrixXd & P0)
{
  if (x.cols() == 0 || x.rows() == 0 || P0.cols() == 0 || P0.rows() == 0) {
    return false;
  }
  x_ = x;
  P_ = P0;
  return true;
}

void KalmanFilter::setA(const Eigen::MatrixXd & A) { A_ = A; }
void KalmanFilter::setB(const Eigen::MatrixXd & B) { B_ = B; }
void KalmanFilter::setC(const Eigen::MatrixXd & C) { C_ = C; }
void KalmanFilter::setQ(const Eigen::MatrixXd & Q) { Q_ = Q; }
void KalmanFilter::setR(const Eigen::MatrixXd & R) { R_ = R; }
void KalmanFilter::getX(Eigen::MatrixXd & x) { x = x_; }
void KalmanFilter::getP(Eigen::MatrixXd & P) { P = P_; }
double KalmanFilter::getXelement(unsigned int i) { return x_(i); }

bool KalmanFilter::predict(
  const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  if (
    x_.rows() != x_next.rows() || A.cols() != P_.rows() || Q.cols() != Q.rows() ||
    A.rows() != Q.cols()) {
    return false;
  }
  x_ = x_next;
  P_ = A * P_ * A.transpose() + Q;
  return true;
}
bool KalmanFilter::predict(const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A)
{
  return predict(x_next, A, Q_);
}

bool KalmanFilter::predict(
  const Eigen::MatrixXd & u, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & Q)
{
  if (A.cols() != x_.rows() || B.cols() != u.rows()) {
    return false;
  }
  const Eigen::MatrixXd x_next = A * x_ + B * u;
  return predict(x_next, A, Q);
}
bool KalmanFilter::predict(const Eigen::MatrixXd & u) { return predict(u, A_, B_, Q_); }

bool KalmanFilter::update(
  const Eigen::MatrixXd & y, const Eigen::MatrixXd & y_pred, const Eigen::MatrixXd & C,
  const Eigen::MatrixXd & R)
{
  if (
    P_.cols() != C.cols() || R.rows() != R.cols() || R.rows() != C.rows() ||
    y.rows() != y_pred.rows() || y.rows() != C.rows()) {
    return false;
  }
  const Eigen::MatrixXd PCT = P_ * C.transpose();
  const Eigen::MatrixXd K = PCT * ((R + C * PCT).inverse());

  if (isnan(K.array()).any() || isinf(K.array()).any()) {
    return false;
  }

  x_ = x_ + K * (y - y_pred);
  P_ = P_ - K * (C * P_);
  return true;
}

bool KalmanFilter::update(
  const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R)
{
  if (C.cols() != x_.rows()) {
    return false;
  }
  const Eigen::MatrixXd y_pred = C * x_;
  return update(y, y_pred, C, R);
}
bool KalmanFilter::update(const Eigen::MatrixXd & y) { return update(y, C_, R_); }
