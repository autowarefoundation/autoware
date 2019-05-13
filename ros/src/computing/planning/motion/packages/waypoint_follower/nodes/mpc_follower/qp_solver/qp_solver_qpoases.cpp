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

#include "mpc_follower/qp_solver/qp_solver_qpoases.h"

QPSolverQpoasesHotstart::QPSolverQpoasesHotstart(const int max_iter)
{
     max_iter_ = max_iter;
};
QPSolverQpoasesHotstart::~QPSolverQpoasesHotstart(){};

bool QPSolverQpoasesHotstart::solve(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, const Eigen::MatrixXd &A,
                                    const Eigen::VectorXd &lb, const Eigen::VectorXd &ub, const Eigen::MatrixXd &lbA,
                                    const Eigen::MatrixXd &ubA, Eigen::VectorXd &U)
{
     USING_NAMESPACE_QPOASES

     int max_iter = max_iter_;

     const int kNumOfMatrixElements = Hmat.rows() * Hmat.cols();
     double h_matrix[kNumOfMatrixElements];

     const int kNumOfoffsetRows = fvec.rows();
     double g_matrix[kNumOfoffsetRows];

     double lower_bound[kNumOfoffsetRows];
     double upper_bound[kNumOfoffsetRows];

     double result[kNumOfoffsetRows];
     U = Eigen::VectorXd::Zero(kNumOfoffsetRows);

     Eigen::MatrixXd Aconstraint = Eigen::MatrixXd::Identity(kNumOfoffsetRows, kNumOfoffsetRows);
     double a_constraint_matirx[kNumOfMatrixElements];

     int index = 0;

     for (int r = 0; r < Hmat.rows(); ++r)
     {
          g_matrix[r] = fvec(r, 0);
          for (int c = 0; c < Hmat.cols(); ++c)
          {
               h_matrix[index] = Hmat(r, c);
               a_constraint_matirx[index] = Aconstraint(r, c);
               index++;
          }
     }

     for (int i = 0; i < kNumOfoffsetRows; ++i)
     {
          lower_bound[i] = lb[i];
          upper_bound[i] = ub[i];
     }

     solver_.setPrintLevel(qpOASES::PL_NONE);

     if (is_init_)
     {
          solver_ = qpOASES::SQProblem(kNumOfoffsetRows, kNumOfoffsetRows);
          auto ret = solver_.init(h_matrix, g_matrix, a_constraint_matirx, lower_bound, upper_bound, lower_bound, upper_bound, max_iter);
          if (ret != SUCCESSFUL_RETURN)
          {
               printf("[QPOASES] not successfully solved in init()\n");
               return false;
          }

          is_init_ = false;
     }
     else
     {
          auto ret = solver_.hotstart(h_matrix, g_matrix, a_constraint_matirx, lower_bound, upper_bound, lower_bound, upper_bound, max_iter);
          if (ret != SUCCESSFUL_RETURN)
          {
               printf("[QPOASES] not successfully solved in hotstart()");
               return false;
          }
     }

     solver_.getPrimalSolution(result);

     for (int i = 0; i < kNumOfoffsetRows; ++i)
     {
          U(i) = result[i];
     }

     return true;
};
