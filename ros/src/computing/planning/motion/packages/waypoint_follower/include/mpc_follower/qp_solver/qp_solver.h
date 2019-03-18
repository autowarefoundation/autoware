#pragma once

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <qpOASES.hpp>
#include <cmath>

namespace qpsolver
{
bool solveEigenLeastSquare(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec,
                           Eigen::VectorXd &U);
bool solveEigenLeastSquareLLT(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec,
                              Eigen::VectorXd &U);
bool solveQpoases(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, Eigen::VectorXd &U);

bool solveByHotstart(qpOASES::SQProblem& solver, const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, Eigen::VectorXd &U, int count);
}