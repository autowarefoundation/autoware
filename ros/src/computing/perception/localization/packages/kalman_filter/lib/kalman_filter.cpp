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
void KalmanFilter::predictEKF(const Eigen::MatrixXd &x_next,
                              const Eigen::MatrixXd &A,
                              const Eigen::MatrixXd &Q)
{
  x_ = x_next;
  P_ = A * P_ * A.transpose() + Q;
}
void KalmanFilter::predictEKF(const Eigen::MatrixXd &x_next,
                              const Eigen::MatrixXd &A)
{
  predictEKF(x_next, A, Q_);
}

void KalmanFilter::predict(const Eigen::MatrixXd &u, const Eigen::MatrixXd &A,
                           const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q)
{
  const Eigen::MatrixXd x_next = A * x_ + B * u;
  predictEKF(x_next, A, Q);
}
void KalmanFilter::predict(const Eigen::MatrixXd &u) { predict(u, A_, B_, Q_); }

void KalmanFilter::updateEKF(const Eigen::MatrixXd &y,
                             const Eigen::MatrixXd &y_pred,
                             const Eigen::MatrixXd &C,
                             const Eigen::MatrixXd &R)
{
  // const Eigen::MatrixXd S_inv = (R + C * P_ * C.transpose()).inverse();
  // const Eigen::MatrixXd K = P_ * C.transpose() * S_inv;
  // x_ = x_ + K * (y - y_pred);
  // const int dim = P_.cols();
  // P_ = P_ - (P_ * C.transpose()) * S_inv * (C * P_);  
  const Eigen::MatrixXd PCT = P_ * C.transpose();
  const Eigen::MatrixXd K = PCT * ((R + C * PCT).inverse());
  x_ = x_ + K * (y - y_pred);
  P_ = P_ - K * (C * P_);
}
void KalmanFilter::updateEKF(const Eigen::MatrixXd &y,
                             const Eigen::MatrixXd &y_pred,
                             const Eigen::MatrixXd &C)
{
  updateEKF(y, y_pred, C, R_);
}
void KalmanFilter::update(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
                          const Eigen::MatrixXd &R)
{
  const Eigen::MatrixXd y_pred = C * x_;
  updateEKF(y, y_pred, C, R);
}
void KalmanFilter::update(const Eigen::MatrixXd &y) { update(y, C_, R_); }
