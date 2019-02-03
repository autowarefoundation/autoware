#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

class KalmanFilter
{
public:
  KalmanFilter();
  KalmanFilter(const Eigen::MatrixXf &x, const Eigen::MatrixXf &A,
               const Eigen::MatrixXf &B, const Eigen::MatrixXf &C,
               const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R,
               const Eigen::MatrixXf &P);
  ~KalmanFilter();

  void init(const Eigen::MatrixXf &x, const Eigen::MatrixXf &A,
            const Eigen::MatrixXf &B, const Eigen::MatrixXf &C,
            const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R,
            const Eigen::MatrixXf &P);
  void init(const Eigen::MatrixXf &x, const Eigen::MatrixXf &P0);
  void setA(const Eigen::MatrixXf &A);
  void setB(const Eigen::MatrixXf &B);
  void setC(const Eigen::MatrixXf &C);
  void setQ(const Eigen::MatrixXf &Q);
  void setR(const Eigen::MatrixXf &R);
  void getX(Eigen::MatrixXf &x);
  void getP(Eigen::MatrixXf &P);

  void predictXandP(const Eigen::MatrixXf &x, const Eigen::MatrixXf &P);
  void predictEKF(const Eigen::MatrixXf &x_next, const Eigen::MatrixXf &A,
                  const Eigen::MatrixXf &Q);
  void predictEKF(const Eigen::MatrixXf &x_next, const Eigen::MatrixXf &A);
  void predict(const Eigen::MatrixXf &u, const Eigen::MatrixXf &A,
               const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q);
  void predict(const Eigen::MatrixXf &u);

  void updateEKF(const Eigen::MatrixXf &y, const Eigen::MatrixXf &y_pred,
                 const Eigen::MatrixXf &C, const Eigen::MatrixXf &R);
  void updateEKF(const Eigen::MatrixXf &y, const Eigen::MatrixXf &y_pred,
                 const Eigen::MatrixXf &C);
  void update(const Eigen::MatrixXf &y, const Eigen::MatrixXf &C,
              const Eigen::MatrixXf &R);
  void update(const Eigen::MatrixXf &y);


private:
  Eigen::MatrixXf x_;
  Eigen::MatrixXf A_;
  Eigen::MatrixXf B_;
  Eigen::MatrixXf C_;
  Eigen::MatrixXf Q_;
  Eigen::MatrixXf R_;
  Eigen::MatrixXf P_;
};

KalmanFilter::KalmanFilter() {}
KalmanFilter::KalmanFilter(const Eigen::MatrixXf &x, const Eigen::MatrixXf &A,
                           const Eigen::MatrixXf &B, const Eigen::MatrixXf &C,
                           const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R,
                           const Eigen::MatrixXf &P)
{
  init(x, A, B, C, Q, R, P);
}
KalmanFilter::~KalmanFilter() {}
void KalmanFilter::init(const Eigen::MatrixXf &x, const Eigen::MatrixXf &A,
                        const Eigen::MatrixXf &B, const Eigen::MatrixXf &C,
                        const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R,
                        const Eigen::MatrixXf &P)
{
  x_ = x;
  A_ = A;
  B_ = B;
  C_ = C;
  Q_ = Q;
  R_ = R;
  P_ = P;
}
void KalmanFilter::init(const Eigen::MatrixXf &x, const Eigen::MatrixXf &P0)
{
  x_ = x;
  P_ = P0;
}

void KalmanFilter::setA(const Eigen::MatrixXf &A) { A_ = A; }
void KalmanFilter::setB(const Eigen::MatrixXf &B) { B_ = B; }
void KalmanFilter::setC(const Eigen::MatrixXf &C) { C_ = C; }
void KalmanFilter::setQ(const Eigen::MatrixXf &Q) { Q_ = Q; }
void KalmanFilter::setR(const Eigen::MatrixXf &R) { R_ = R; }
void KalmanFilter::getX(Eigen::MatrixXf &x) { x = x_; };
void KalmanFilter::getP(Eigen::MatrixXf &P) { P = P_; };

void KalmanFilter::predictXandP(const Eigen::MatrixXf &x, const Eigen::MatrixXf &P)
{
  x_ = x;
  P_ = P;
}
void KalmanFilter::predictEKF(const Eigen::MatrixXf &x_next,
                              const Eigen::MatrixXf &A,
                              const Eigen::MatrixXf &Q)
{
  x_ = x_next;
  P_ = A * P_ * A.transpose() + Q;
}
void KalmanFilter::predictEKF(const Eigen::MatrixXf &x_next,
                              const Eigen::MatrixXf &A)
{
  predictEKF(x_next, A, Q_);
}

void KalmanFilter::predict(const Eigen::MatrixXf &u, const Eigen::MatrixXf &A,
                           const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q)
{
  const Eigen::MatrixXf x_next = A * x_ + B * u;
  predictEKF(x_next, A, Q);
}
void KalmanFilter::predict(const Eigen::MatrixXf &u) { predict(u, A_, B_, Q_); }

void KalmanFilter::updateEKF(const Eigen::MatrixXf &y,
                             const Eigen::MatrixXf &y_pred,
                             const Eigen::MatrixXf &C,
                             const Eigen::MatrixXf &R)
{
  const Eigen::MatrixXf S_inv = (R + C * P_ * C.transpose()).inverse();
  const Eigen::MatrixXf K = P_ * C.transpose() * S_inv;
  x_ = x_ + K * (y - y_pred);
  const int dim = P_.cols();
  P_ = P_ - (P_ * C.transpose()) * S_inv * (C * P_);  
}
void KalmanFilter::updateEKF(const Eigen::MatrixXf &y,
                             const Eigen::MatrixXf &y_pred,
                             const Eigen::MatrixXf &C)
{
  updateEKF(y, y_pred, C, R_);
}
void KalmanFilter::update(const Eigen::MatrixXf &y, const Eigen::MatrixXf &C,
                          const Eigen::MatrixXf &R)
{
  const Eigen::MatrixXf y_pred = C * x_;
  updateEKF(y, y_pred, C, R);
}
void KalmanFilter::update(const Eigen::MatrixXf &y) { update(y, C_, R_); }
