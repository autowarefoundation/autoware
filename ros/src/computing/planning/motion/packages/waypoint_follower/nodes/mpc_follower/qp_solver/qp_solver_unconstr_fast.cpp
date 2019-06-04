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

#include "mpc_follower/qp_solver/qp_solver_unconstr_fast.h"

QPSolverEigenLeastSquareLLT::QPSolverEigenLeastSquareLLT(){};
QPSolverEigenLeastSquareLLT::~QPSolverEigenLeastSquareLLT(){};
bool QPSolverEigenLeastSquareLLT::solve(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, const Eigen::MatrixXd &A,
                                        const Eigen::VectorXd &lb, const Eigen::VectorXd &ub, const Eigen::MatrixXd &lbA,
                                        const Eigen::MatrixXd &ubA, Eigen::VectorXd &U)
{
     if (std::fabs(Hmat.determinant()) < 1.0E-9)
          return false;

     U = -Hmat.llt().solve(fvec);

     return true;
};
