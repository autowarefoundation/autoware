#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

class KalmanFilterDelay {
  KalmanFilterDelay();
  ~KalmanFilterDelay();

private:
  KalmanFilter kf_;
  double dim_x_;
  double dim_u_;
  double dim_y_;
  double dim_x_ex_;
  double dim_u_ex_;
  unsigned int step_num_;
  Eigen::MatrixXd A_ex_;
  Eigen::MatrixXd B_ex_;
  Eigen::MatrixXd C_ex_;


  void predict();
  void update();
};

KalmanFilterDelay() {};
KalmanFilterDelay(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
             const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q,
             const Eigen::MatrixXd &R, const Eigen::MatrixXd &P,
             const double &dt) {
  init(A, B, C, Q, R, P, step_num);
}
void KalmanFilterDelay::init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &A,
                        const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
                        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                        const Eigen::MatrixXd &P) {
    dim_x_ = A.cols();
    dim_y_ = C.cols();
    dim_u_ = B.rows();
    dim_x_ex_ = dim_x_ * step_num;
    dim_u_ex_ = dim_u_ * step_num_;
    A_ex_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
    B_ex_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_u_ex_);
    C_ex_ = Eigen::MatrixXd::Zero(dim_y_, dim_x_ex_);
}

void KalmanFilterDelay::UpdateExtendedMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) {
  for (unsigned int i = 1; i < step_num; ++i) {
    A_ex_.block(i, i, dim_x_, dim_x_) = A_ex_.block(i - dim_x_, i - dim_x_, dim_x_, dim_x_);
  }
};


void predict(const Eigen::MatrixXd &u) {
  x_ = A_ * x_ + B_ * u;
  P_ = A_ * P_ * A_.transpose() + Q_;
}

void update(const Eigen::MatrixXd &y) {
  const Eigen::MatrixXd S = R_ + C_ * P_ * C_.transpose();
  const Eigen::MatrixXd K = P_ * C_.transpose() * S.inverce();
  x_ = x_ + K * (y - C_ * x_);
  const int dim = P_.cols();
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim, dim);
  P_ = (I - K * C_) * P_ * ((I - K * C_).transpose()) + K * P_ * K;
}
