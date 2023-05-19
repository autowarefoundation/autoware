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

#include "qp_interface/qp_interface.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace qp
{
void QPInterface::initializeProblem(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  // check if arguments are valid
  std::stringstream ss;
  if (P.rows() != P.cols()) {
    ss << "P.rows() and P.cols() are not the same. P.rows() = " << P.rows()
       << ", P.cols() = " << P.cols();
    throw std::invalid_argument(ss.str());
  }
  if (P.rows() != static_cast<int>(q.size())) {
    ss << "P.rows() and q.size() are not the same. P.rows() = " << P.rows()
       << ", q.size() = " << q.size();
    throw std::invalid_argument(ss.str());
  }
  if (P.rows() != A.cols()) {
    ss << "P.rows() and A.cols() are not the same. P.rows() = " << P.rows()
       << ", A.cols() = " << A.cols();
    throw std::invalid_argument(ss.str());
  }
  if (A.rows() != static_cast<int>(l.size())) {
    ss << "A.rows() and l.size() are not the same. A.rows() = " << A.rows()
       << ", l.size() = " << l.size();
    throw std::invalid_argument(ss.str());
  }
  if (A.rows() != static_cast<int>(u.size())) {
    ss << "A.rows() and u.size() are not the same. A.rows() = " << A.rows()
       << ", u.size() = " << u.size();
    throw std::invalid_argument(ss.str());
  }

  initializeProblemImpl(P, A, q, l, u);

  m_variables_num = q.size();
  m_constraints_num = l.size();
}

std::vector<double> QPInterface::optimize(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  initializeProblem(P, A, q, l, u);
  const auto result = optimizeImpl();

  return result;
}
}  // namespace qp
