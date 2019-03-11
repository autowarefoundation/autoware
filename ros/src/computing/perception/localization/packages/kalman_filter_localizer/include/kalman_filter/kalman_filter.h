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

  bool init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &A,
            const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
            const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
            const Eigen::MatrixXd &P);
  bool init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P0);
  void setA(const Eigen::MatrixXd &A);
  void setB(const Eigen::MatrixXd &B);
  void setC(const Eigen::MatrixXd &C);
  void setQ(const Eigen::MatrixXd &Q);
  void setR(const Eigen::MatrixXd &R);
  void getX(Eigen::MatrixXd &x);
  void getP(Eigen::MatrixXd &P);
  double getXelement(unsigned int i);

  void predictXandP(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P);
  bool predict(const Eigen::MatrixXd &u, const Eigen::MatrixXd &A,
               const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q);
  bool predict(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A,
               const Eigen::MatrixXd &Q);
  bool predict(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A);
  bool predict(const Eigen::MatrixXd &u);

  bool update(const Eigen::MatrixXd &y, const Eigen::MatrixXd &y_pred,
              const Eigen::MatrixXd &C, const Eigen::MatrixXd &R);
  // bool update(const Eigen::MatrixXd &y, const Eigen::MatrixXd &y_pred,
  //             const Eigen::MatrixXd &C);
  bool update(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
              const Eigen::MatrixXd &R);
  bool update(const Eigen::MatrixXd &y);

protected:
  Eigen::MatrixXd x_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd C_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd P_;
};