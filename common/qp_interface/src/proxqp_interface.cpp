// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "qp_interface/proxqp_interface.hpp"

namespace autoware::common
{
using proxsuite::proxqp::QPSolverOutput;

ProxQPInterface::ProxQPInterface(
  const bool enable_warm_start, const double eps_abs, const double eps_rel, const bool verbose)
: QPInterface(enable_warm_start)
{
  settings_.eps_abs = eps_abs;
  settings_.eps_rel = eps_rel;
  settings_.verbose = verbose;
}

void ProxQPInterface::initializeProblemImpl(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  const size_t variables_num = q.size();
  const size_t constraints_num = l.size();

  const bool enable_warm_start = [&]() {
    if (
      !enable_warm_start_  // Warm start is designated
      || !qp_ptr_          // QP pointer is initialized
      // The number of variables is the same as the previous one.
      || !variables_num_ ||
      *variables_num_ != variables_num
      // The number of constraints is the same as the previous one
      || !constraints_num_ || *constraints_num_ != constraints_num) {
      return false;
    }
    return true;
  }();

  if (!enable_warm_start) {
    qp_ptr_ = std::make_shared<proxsuite::proxqp::sparse::QP<double, int>>(
      variables_num, 0, constraints_num);
  }

  settings_.initial_guess =
    enable_warm_start ? proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT
                      : proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;

  qp_ptr_->settings = settings_;

  Eigen::SparseMatrix<double> P_sparse(variables_num, constraints_num);
  P_sparse = P.sparseView();

  // NOTE: const std vector cannot be converted to eigen vector
  std::vector<double> non_const_q = q;
  Eigen::VectorXd eigen_q =
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(non_const_q.data(), non_const_q.size());
  std::vector<double> l_std_vec = l;
  Eigen::VectorXd eigen_l =
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(l_std_vec.data(), l_std_vec.size());
  std::vector<double> u_std_vec = u;
  Eigen::VectorXd eigen_u =
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(u_std_vec.data(), u_std_vec.size());

  if (enable_warm_start) {
    qp_ptr_->update(
      P_sparse, eigen_q, proxsuite::nullopt, proxsuite::nullopt, A.sparseView(), eigen_l, eigen_u);
  } else {
    qp_ptr_->init(
      P_sparse, eigen_q, proxsuite::nullopt, proxsuite::nullopt, A.sparseView(), eigen_l, eigen_u);
  }
}

void ProxQPInterface::updateEpsAbs(const double eps_abs)
{
  settings_.eps_abs = eps_abs;
}

void ProxQPInterface::updateEpsRel(const double eps_rel)
{
  settings_.eps_rel = eps_rel;
}

void ProxQPInterface::updateVerbose(const bool is_verbose)
{
  settings_.verbose = is_verbose;
}

bool ProxQPInterface::isSolved() const
{
  if (qp_ptr_) {
    return qp_ptr_->results.info.status == QPSolverOutput::PROXQP_SOLVED;
  }
  return false;
}

int ProxQPInterface::getIterationNumber() const
{
  if (qp_ptr_) {
    return qp_ptr_->results.info.iter;
  }
  return 0;
}

std::string ProxQPInterface::getStatus() const
{
  if (qp_ptr_) {
    if (qp_ptr_->results.info.status == QPSolverOutput::PROXQP_SOLVED) {
      return "PROXQP_SOLVED";
    }
    if (qp_ptr_->results.info.status == QPSolverOutput::PROXQP_MAX_ITER_REACHED) {
      return "PROXQP_MAX_ITER_REACHED";
    }
    if (qp_ptr_->results.info.status == QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE) {
      return "PROXQP_PRIMAL_INFEASIBLE";
    }
    // if (qp_ptr_->results.info.status == QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE) {
    //   return "PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE";
    // }
    if (qp_ptr_->results.info.status == QPSolverOutput::PROXQP_DUAL_INFEASIBLE) {
      return "PROXQP_DUAL_INFEASIBLE";
    }
    if (qp_ptr_->results.info.status == QPSolverOutput::PROXQP_NOT_RUN) {
      return "PROXQP_NOT_RUN";
    }
  }
  return "None";
}

std::vector<double> ProxQPInterface::optimizeImpl()
{
  qp_ptr_->solve();

  std::vector<double> result;
  for (Eigen::Index i = 0; i < qp_ptr_->results.x.size(); ++i) {
    result.push_back(qp_ptr_->results.x[i]);
  }
  return result;
}
}  // namespace autoware::common
