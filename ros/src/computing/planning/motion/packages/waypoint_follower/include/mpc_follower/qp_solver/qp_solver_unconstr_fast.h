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
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <cmath>
#include "mpc_follower/qp_solver/qp_solver_interface.h"

class QPSolverEigenLeastSquareLLT : public QPSolverInterface
{
public:
  QPSolverEigenLeastSquareLLT();
  ~QPSolverEigenLeastSquareLLT();
  bool solve(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, const Eigen::MatrixXd &A,
             const Eigen::VectorXd &lb, const Eigen::VectorXd &ub, const Eigen::MatrixXd &lbA,
             const Eigen::MatrixXd &ubA, Eigen::VectorXd &U) override;
};
