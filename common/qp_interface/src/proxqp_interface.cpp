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

namespace qp
{
ProxQPInterface::ProxQPInterface(const bool enable_warm_start, const double eps_abs)
: QPInterface(enable_warm_start)
{
  m_settings.eps_abs = eps_abs;
}

void ProxQPInterface::initializeProblemImpl(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  const size_t variables_num = q.size();
  const size_t constraints_num = l.size();

  const bool enable_warm_start = [&]() {
    if (
      !m_enable_warm_start  // Warm start is designated
      || !m_qp_ptr          // QP pointer is initialized
      // The number of variables is the same as the previous one.
      || !m_variables_num ||
      *m_variables_num != variables_num
      // The number of constraints is the same as the previous one
      || !m_constraints_num || *m_constraints_num != constraints_num) {
      return false;
    }
    return true;
  }();

  if (!enable_warm_start) {
    m_qp_ptr = std::make_shared<proxsuite::proxqp::sparse::QP<double, int>>(
      variables_num, 0, constraints_num);
  }

  m_settings.initial_guess =
    enable_warm_start ? proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT
                      : proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;

  m_qp_ptr->settings = m_settings;

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
    m_qp_ptr->update(
      P_sparse, eigen_q, proxsuite::nullopt, proxsuite::nullopt, A.sparseView(), eigen_l, eigen_u);
  } else {
    m_qp_ptr->init(
      P_sparse, eigen_q, proxsuite::nullopt, proxsuite::nullopt, A.sparseView(), eigen_l, eigen_u);
  }
}

void ProxQPInterface::updateEpsAbs(const double eps_abs)
{
  m_settings.eps_abs = eps_abs;
}

void ProxQPInterface::updateEpsRel(const double eps_rel)
{
  m_settings.eps_rel = eps_rel;
}

void ProxQPInterface::updateVerbose(const bool is_verbose)
{
  m_settings.verbose = is_verbose;
}

int ProxQPInterface::getIteration() const
{
  if (m_qp_ptr) {
    return m_qp_ptr->results.info.iter;
  }
  return 0;
}

int ProxQPInterface::getStatus() const
{
  if (m_qp_ptr) {
    return static_cast<int>(m_qp_ptr->results.info.status);
  }
  return 0;
}

std::vector<double> ProxQPInterface::optimizeImpl()
{
  m_qp_ptr->solve();

  std::vector<double> result;
  for (Eigen::Index i = 0; i < m_qp_ptr->results.x.size(); ++i) {
    result.push_back(m_qp_ptr->results.x[i]);
  }
  return result;
}
}  // namespace qp
