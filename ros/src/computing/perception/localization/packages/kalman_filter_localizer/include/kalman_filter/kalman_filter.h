#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

class KalmanFilter
{
public:
  KalmanFilter();
  KalmanFilter(const Eigen::MatrixXd &x, const Eigen::MatrixXd &A,
               const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
               const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
               const Eigen::MatrixXd &P);
  ~KalmanFilter();

  void init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &A,
            const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
            const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
            const Eigen::MatrixXd &P);
  void init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P0);
  void setA(const Eigen::MatrixXd &A);
  void setB(const Eigen::MatrixXd &B);
  void setC(const Eigen::MatrixXd &C);
  void setQ(const Eigen::MatrixXd &Q);
  void setR(const Eigen::MatrixXd &R);
  void getX(Eigen::MatrixXd &x);
  void getP(Eigen::MatrixXd &P);
  double getXelement(unsigned int i);

  void predictXandP(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P);
  void predictEKF(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A,
                  const Eigen::MatrixXd &Q);
  void predictEKF(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A);
  void predict(const Eigen::MatrixXd &u, const Eigen::MatrixXd &A,
               const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q);
  void predict(const Eigen::MatrixXd &u);

  void updateEKF(const Eigen::MatrixXd &y, const Eigen::MatrixXd &y_pred,
                 const Eigen::MatrixXd &C, const Eigen::MatrixXd &R);
  void updateEKF(const Eigen::MatrixXd &y, const Eigen::MatrixXd &y_pred,
                 const Eigen::MatrixXd &C);
  void update(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
              const Eigen::MatrixXd &R);
  void update(const Eigen::MatrixXd &y);


protected:
  Eigen::MatrixXd x_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd C_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd P_;
};