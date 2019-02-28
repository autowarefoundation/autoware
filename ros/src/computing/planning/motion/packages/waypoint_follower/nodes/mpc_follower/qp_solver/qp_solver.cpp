#include "mpc_follower/qp_solver/qp_solver.h"
#include <chrono>

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
} // namespace qpsolver