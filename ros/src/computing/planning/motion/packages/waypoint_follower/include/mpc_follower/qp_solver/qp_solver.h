#pragma once

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/LU>

namespace qpsolver
{
bool solveEigenLeastSquare(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec,
                           Eigen::VectorXd &U);
bool solveEigenLeastSquareLLT(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec,
                              Eigen::VectorXd &U);
}