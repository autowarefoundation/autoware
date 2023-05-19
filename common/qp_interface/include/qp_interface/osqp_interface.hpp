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

#ifndef QP_INTERFACE__OSQP_INTERFACE_HPP_
#define QP_INTERFACE__OSQP_INTERFACE_HPP_

#include "osqp/osqp.h"
#include "qp_interface/osqp_csc_matrix_conv.hpp"
#include "qp_interface/qp_interface.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace qp
{
constexpr c_float INF = 1e30;

class OSQPInterface : public QPInterface
{
public:
  /// \brief Constructor without problem formulation
  OSQPInterface(
    const bool enable_warm_start = false,
    const c_float eps_abs = std::numeric_limits<c_float>::epsilon(), const bool polish = true);
  /// \brief Constructor with problem setup
  /// \param P: (n,n) matrix defining relations between parameters.
  /// \param A: (m,n) matrix defining parameter constraints relative to the lower and upper bound.
  /// \param q: (n) vector defining the linear cost of the problem.
  /// \param l: (m) vector defining the lower bound problem constraint.
  /// \param u: (m) vector defining the upper bound problem constraint.
  /// \param eps_abs: Absolute convergence tolerance.
  OSQPInterface(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u,
    const bool enable_warm_start = false,
    const c_float eps_abs = std::numeric_limits<c_float>::epsilon());
  OSQPInterface(
    const CSC_Matrix & P, const CSC_Matrix & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u,
    const bool enable_warm_start = false,
    const c_float eps_abs = std::numeric_limits<c_float>::epsilon());
  ~OSQPInterface();

  static void OSQPWorkspaceDeleter(OSQPWorkspace * ptr) noexcept;

  std::vector<double> optimize(
    CSC_Matrix P, CSC_Matrix A, const std::vector<double> & q, const std::vector<double> & l,
    const std::vector<double> & u);

  int getIteration() const override;
  int getStatus() const override;
  int getPolishStatus() const;
  std::vector<double> getDualSolution() const;

  void updateEpsAbs(const double eps_abs) override;
  void updateEpsRel(const double eps_rel) override;
  void updateVerbose(const bool verbose) override;

  // Updates problem parameters while keeping solution in memory.
  //
  // Args:
  //   P_new: (n,n) matrix defining relations between parameters.
  //   A_new: (m,n) matrix defining parameter constraints relative to the lower and upper bound.
  //   q_new: (n) vector defining the linear cost of the problem.
  //   l_new: (m) vector defining the lower bound problem constraint.
  //   u_new: (m) vector defining the upper bound problem constraint.
  void updateP(const Eigen::MatrixXd & P_new);
  void updateCscP(const CSC_Matrix & P_csc);
  void updateA(const Eigen::MatrixXd & A_new);
  void updateCscA(const CSC_Matrix & A_csc);
  void updateQ(const std::vector<double> & q_new);
  void updateL(const std::vector<double> & l_new);
  void updateU(const std::vector<double> & u_new);
  void updateBounds(const std::vector<double> & l_new, const std::vector<double> & u_new);

  void updateMaxIter(const int iter);
  void updateRhoInterval(const int rho_interval);
  void updateRho(const double rho);
  void updateAlpha(const double alpha);
  void updateScaling(const int scaling);
  void updatePolish(const bool polish);
  void updatePolishRefinementIteration(const int polish_refine_iter);
  void updateCheckTermination(const int check_termination);

  /// \brief Get the number of iteration taken to solve the problem
  inline int64_t getTakenIter() const { return static_cast<int64_t>(m_latest_work_info.iter); }
  /// \brief Get the status message for the latest problem solved
  inline std::string getStatusMessage() const
  {
    return static_cast<std::string>(m_latest_work_info.status);
  }
  /// \brief Get the runtime of the latest problem solved
  inline double getRunTime() const { return m_latest_work_info.run_time; }
  /// \brief Get the objective value the latest problem solved
  inline double getObjVal() const { return m_latest_work_info.obj_val; }
  /// \brief Returns flag asserting interface condition (Healthy condition: 0).
  inline int64_t getExitFlag() const { return m_exitflag; }

  void logUnsolvedStatus(const std::string & prefix_message = "") const;

  // Setter functions for warm start
  bool setWarmStart(
    const std::vector<double> & primal_variables, const std::vector<double> & dual_variables);
  bool setPrimalVariables(const std::vector<double> & primal_variables);
  bool setDualVariables(const std::vector<double> & dual_variables);

private:
  std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>> m_work;
  std::unique_ptr<OSQPSettings> m_settings;
  std::unique_ptr<OSQPData> m_data;
  // store last work info since work is cleaned up at every execution to prevent memory leak.
  OSQPInfo m_latest_work_info;
  // Number of parameters to optimize
  int64_t m_param_n;
  // Flag to check if the current work exists
  bool m_work_initialized = false;
  // Exitflag
  int64_t m_exitflag;

  void initializeProblemImpl(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u) override;

  void initializeCSCProblemImpl(
    CSC_Matrix P, CSC_Matrix A, const std::vector<double> & q, const std::vector<double> & l,
    const std::vector<double> & u);

  std::vector<double> optimizeImpl() override;
};
}  // namespace qp

#endif  // QP_INTERFACE__OSQP_INTERFACE_HPP_
