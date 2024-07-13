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

#ifndef QP_INTERFACE__QP_INTERFACE_HPP_
#define QP_INTERFACE__QP_INTERFACE_HPP_

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

namespace autoware::common
{
class QPInterface
{
public:
  explicit QPInterface(const bool enable_warm_start) : enable_warm_start_(enable_warm_start) {}

  std::vector<double> optimize(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u);

  virtual bool isSolved() const = 0;
  virtual int getIterationNumber() const = 0;
  virtual std::string getStatus() const = 0;

  virtual void updateEpsAbs([[maybe_unused]] const double eps_abs) = 0;
  virtual void updateEpsRel([[maybe_unused]] const double eps_rel) = 0;
  virtual void updateVerbose([[maybe_unused]] const bool verbose) {}

protected:
  bool enable_warm_start_{false};

  void initializeProblem(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u);

  virtual void initializeProblemImpl(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u) = 0;

  virtual std::vector<double> optimizeImpl() = 0;

  std::optional<size_t> variables_num_{std::nullopt};
  std::optional<size_t> constraints_num_{std::nullopt};
};
}  // namespace autoware::common

#endif  // QP_INTERFACE__QP_INTERFACE_HPP_
