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

/**
 * @file kalman_filter.h
 * @brief kalman filter class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

class KalmanFilter
{
public:

  /**
   * @brief No initialization constructor.
   */
  KalmanFilter();

  /**
   * @brief constructor with initialization
   * @param x initial state
   * @param A coefficient matrix of x for process model
   * @param B coefficient matrix of u for process model
   * @param C coefficient matrix of x for measurement model
   * @param Q covariace matrix for process model
   * @param R covariance matrix for measurement model
   * @param P initial covariance of estimated state
   */
  KalmanFilter(const Eigen::MatrixXd &x, const Eigen::MatrixXd &A,
               const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
               const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
               const Eigen::MatrixXd &P);

  /**
   * @brief destructor
   */
  ~KalmanFilter();

  /**
   * @brief initialization of kalman filter
   * @param x initial state
   * @param A coefficient matrix of x for process model
   * @param B coefficient matrix of u for process model
   * @param C coefficient matrix of x for measurement model
   * @param Q covariace matrix for process model
   * @param R covariance matrix for measurement model
   * @param P initial covariance of estimated state
   */
  bool init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &A,
            const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
            const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
            const Eigen::MatrixXd &P);

  /**
   * @brief initialization of kalman filter
   * @param x initial state
   * @param P initial covariance of estimated state
   */
  bool init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P0);

  /**
   * @brief set A of process model
   * @param A coefficient matrix of x for process model
   */
  void setA(const Eigen::MatrixXd &A);

  /**
   * @brief set B of process model
   * @param B coefficient matrix of u for process model
   */
  void setB(const Eigen::MatrixXd &B);

  /**
   * @brief set C of measurement model
   * @param C coefficient matrix of x for measurement model
   */
  void setC(const Eigen::MatrixXd &C);

  /**
   * @brief set covariace matrix Q for process model
   * @param Q covariace matrix for process model
   */
  void setQ(const Eigen::MatrixXd &Q);

  /**
   * @brief set covariance matrix R for measurement model
   * @param R covariance matrix for measurement model
   */
  void setR(const Eigen::MatrixXd &R);

  /**
   * @brief get current kalman filter state
   * @param x kalman filter state
   */
  void getX(Eigen::MatrixXd &x);

  /**
   * @brief get current kalman filter covariance
   * @param P kalman filter covariance
   */
  void getP(Eigen::MatrixXd &P);

  /**
   * @brief get component of current kalman filter state
   * @param i index of kalman filter state
   * @return value of i's component of the kalman filter state x[i]
   */
  double getXelement(unsigned int i);

  /**
   * @brief calculate kalman filter state and covariance by prediction model with A, B, Q matrix. This is mainly for EKF with variable matrix.
   * @param u input for model
   * @param A coefficient matrix of x for process model
   * @param B coefficient matrix of u for process model
   * @param Q covariace matrix for process model
   * @return bool to check matrix operations are being performed properly
   */
  bool predict(const Eigen::MatrixXd &u, const Eigen::MatrixXd &A,
               const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q);

  /**
   * @brief calculate kalman filter covariance with prediction model with x, A, Q matrix. This is mainly for EKF with variable matrix.
   * @param x_next predicted state
   * @param A coefficient matrix of x for process model
   * @param Q covariace matrix for process model
   * @return bool to check matrix operations are being performed properly
   */
  bool predict(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A,
               const Eigen::MatrixXd &Q);

  /**
   * @brief calculate kalman filter covariance with prediction model with x, A, Q matrix. This is mainly for EKF with variable matrix.
   * @param x_next predicted state
   * @param A coefficient matrix of x for process model
   * @return bool to check matrix operations are being performed properly
   */
  bool predict(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A);

  /**
   * @brief calculate kalman filter state by prediction model with A, B and Q being class menber variables.
   * @param u input for the model
   * @return bool to check matrix operations are being performed properly
   */
  bool predict(const Eigen::MatrixXd &u);

  /**
   * @brief calculate kalman filter state by measurement model with y_pred, C and R matrix. This is mainly for EKF with variable matrix.
   * @param y measured values
   * @param y output values expected from measurement model
   * @param C coefficient matrix of x for measurement model
   * @param R covariance matrix for measurement model
   * @return bool to check matrix operations are being performed properly
   */
  bool update(const Eigen::MatrixXd &y, const Eigen::MatrixXd &y_pred,
              const Eigen::MatrixXd &C, const Eigen::MatrixXd &R);

  /**
   * @brief calculate kalman filter state by measurement model with C and R matrix. This is mainly for EKF with variable matrix.
   * @param y measured values
   * @param C coefficient matrix of x for measurement model
   * @param R covariance matrix for measurement model
   * @return bool to check matrix operations are being performed properly
   */
  bool update(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
              const Eigen::MatrixXd &R);

  /**
   * @brief calculate kalman filter state by measurement model with C and R being class menber variables.
   * @param y measured values
   * @return bool to check matrix operations are being performed properly
   */
  bool update(const Eigen::MatrixXd &y);

protected:
  Eigen::MatrixXd x_; //!< @brief current estimated state
  Eigen::MatrixXd A_; //!< @brief coefficient matrix of x for process model x[k+1] = A*x[k] + B*u[k]
  Eigen::MatrixXd B_; //!< @brief coefficient matrix of u for process model x[k+1] = A*x[k] + B*u[k]
  Eigen::MatrixXd C_; //!< @brief coefficient matrix of x for measurement model y[k] = C * x[k]
  Eigen::MatrixXd Q_; //!< @brief covariace matrix for process model x[k+1] = A*x[k] + B*u[k]
  Eigen::MatrixXd R_; //!< @brief covariance matrix for measurement model y[k] = C * x[k]
  Eigen::MatrixXd P_; //!< @brief covariance of estimated state
};