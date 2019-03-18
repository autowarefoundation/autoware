#include "mpc_follower/qp_solver/qp_solver.h"
#include <chrono>
#include <iostream>

namespace qpsolver
{

bool solveEigenLeastSquareInverse(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec,
                                  Eigen::VectorXd &U)
{
     if (std::fabs(Hmat.determinant()) < 1.0E-9)
          return false;

     U = -Hmat.inverse() * fvec.transpose();

     return true;
}

bool solveEigenLeastSquareLLT(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec,
                              Eigen::VectorXd &U)
{
     if (std::fabs(Hmat.determinant()) < 1.0E-9)
          return false;

     U = -Hmat.llt().solve(fvec.transpose());

     return true;
}

bool solveQpoases(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, Eigen::VectorXd &U)
{
     USING_NAMESPACE_QPOASES
     int mpc_n = 50;

     QProblemB qpoases(mpc_n);
     qpoases.setPrintLevel(PL_NONE);
     int nWSR = 200;

     const auto kNumOfMatrixElements = Hmat.rows() * Hmat.cols();
     double h_matrix[kNumOfMatrixElements];

     const auto kNumOfoffsetRows = fvec.rows();
     double g_matrix[kNumOfoffsetRows];

     double lower_bound[mpc_n];
     double upper_bound[mpc_n];

     double result[mpc_n];
     U = Eigen::VectorXd::Zero(50);

     int index = 0;

     for(int r=0; r<Hmat.rows(); ++r)
     {
          g_matrix[r] = fvec(r, 0);
          for (int c=0; c<Hmat.cols(); ++c)
          {
               h_matrix[index++] = Hmat(r, c);
          }
     }

     for(int i=0; i<mpc_n; ++i)
     {
          lower_bound[i] = -60*M_PI/180;
          upper_bound[i] = 60*M_PI/180;
     }

     auto ret = qpoases.init(h_matrix, g_matrix, lower_bound, upper_bound, nWSR);
     
     if(ret != SUCCESSFUL_RETURN)
          return false;

     qpoases.getPrimalSolution(result);

     for(int i=0; i<mpc_n; ++i)
     {
          U(i) = result[i];
     }
     printf("output will be %f\n", U(0));

     return true;
}

bool solveByHotstart(qpOASES::SQProblem& solver, const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, Eigen::VectorXd &U, int count)
{
     USING_NAMESPACE_QPOASES
     int mpc_n = 50;

     int nWSR = 200;

     const auto kNumOfMatrixElements = Hmat.rows() * Hmat.cols();
     double h_matrix[kNumOfMatrixElements];

     const auto kNumOfoffsetRows = fvec.rows();
     double g_matrix[kNumOfoffsetRows];

     double lower_bound[mpc_n];
     double upper_bound[mpc_n];

     double result[mpc_n];
     U = Eigen::VectorXd::Zero(50);

     Eigen::MatrixXd Aconstraint = Eigen::MatrixXd::Identity(50, 50);
     double a_constraint_matirx[mpc_n*mpc_n];

     int index = 0;

     for(int r=0; r<Hmat.rows(); ++r)
     {
          g_matrix[r] = fvec(r, 0);
          for (int c=0; c<Hmat.cols(); ++c)
          {
               h_matrix[index] = Hmat(r, c);
               a_constraint_matirx[index] = Aconstraint(r, c);
               index++;
          }
     }

     for(int i=0; i<mpc_n; ++i)
     {
          lower_bound[i] = -60*M_PI/180;
          upper_bound[i] = 60*M_PI/180;
     }

     printf("Count Number is %d\n", count);
     if(count == 0 )
     {
          auto ret = solver.init(h_matrix, g_matrix, a_constraint_matirx, lower_bound, upper_bound, lower_bound, upper_bound, nWSR);
          if(ret != SUCCESSFUL_RETURN)
               return false;
     }
     else
     {
          auto ret = solver.hotstart(h_matrix, g_matrix, a_constraint_matirx, lower_bound, upper_bound, lower_bound, upper_bound, nWSR);
          if(ret != SUCCESSFUL_RETURN)
               return false;
     }

     solver.getPrimalSolution(result);

     for(int i=0; i<mpc_n; ++i)
     {
          U(i) = result[i];
     }
     printf("output will be %f\n", U(0));

     return true;
}

} // namespace qpsolver