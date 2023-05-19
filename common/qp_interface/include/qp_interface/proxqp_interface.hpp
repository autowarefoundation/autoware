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

#ifndef QP_INTERFACE__PROXQP_INTERFACE_HPP_
#define QP_INTERFACE__PROXQP_INTERFACE_HPP_

#include "qp_interface/qp_interface.hpp"

#include <proxsuite/helpers/optional.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>

#include <limits>
#include <memory>
#include <vector>

namespace qp
{
class ProxQPInterface : public QPInterface
{
public:
  explicit ProxQPInterface(
    const bool enable_warm_start = false,
    const double eps_abs = std::numeric_limits<double>::epsilon());

  int getIteration() const override;
  int getStatus() const override;

  void updateEpsAbs(const double eps_abs) override;
  void updateEpsRel(const double eps_rel) override;
  void updateVerbose(const bool verbose) override;

private:
  proxsuite::proxqp::Settings<double> m_settings;
  std::shared_ptr<proxsuite::proxqp::sparse::QP<double, int>> m_qp_ptr;

  void initializeProblemImpl(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u) override;

  std::vector<double> optimizeImpl() override;
};
}  // namespace qp

#endif  // QP_INTERFACE__PROXQP_INTERFACE_HPP_
