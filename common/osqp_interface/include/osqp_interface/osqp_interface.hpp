// Copyright 2021 The Autoware Foundation
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

#ifndef OSQP_INTERFACE__OSQP_INTERFACE_HPP_
#define OSQP_INTERFACE__OSQP_INTERFACE_HPP_

#include "eigen3/Eigen/Core"
#include "osqp/osqp.h"
#include "osqp_interface/csc_matrix_conv.hpp"
#include "osqp_interface/visibility_control.hpp"

#include <rclcpp/rclcpp.hpp>

#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace autoware
{
namespace common
{
namespace osqp
{
constexpr c_float INF = 1e30;

/**
 * Implementation of a native C++ interface for the OSQP solver.
 *
 */
class OSQP_INTERFACE_PUBLIC OSQPInterface
{
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

  // Runs the solver on the stored problem.
  std::tuple<std::vector<double>, std::vector<double>, int64_t, int64_t, int64_t> solve();

  static void OSQPWorkspaceDeleter(OSQPWorkspace * ptr) noexcept;

public:
  /// \brief Constructor without problem formulation
  explicit OSQPInterface(
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
    const std::vector<double> & l, const std::vector<double> & u, const c_float eps_abs);
  OSQPInterface(
    const CSC_Matrix & P, const CSC_Matrix & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u, const c_float eps_abs);
  ~OSQPInterface();

  /****************
   * OPTIMIZATION
   ****************/
  /// \brief Solves the stored convex quadratic program (QP) problem using the OSQP solver.
  //
  /// \return The function returns a tuple containing the solution as two float vectors.
  /// \return The first element of the tuple contains the 'primal' solution.
  /// \return The second element contains the 'lagrange multiplier' solution.
  /// \return The third element contains an integer with solver polish status information.

  /// \details How to use:
  /// \details   1. Generate the Eigen matrices P, A and vectors q, l, u according to the problem.
  /// \details   2. Initialize the interface and set up the problem.
  /// \details        osqp_interface = OSQPInterface(P, A, q, l, u, 1e-6);
  /// \details   3. Call the optimization function.
  /// \details        std::tuple<std::vector<double>, std::vector<double>> result;
  /// \details        result = osqp_interface.optimize();
  /// \details   4. Access the optimized parameters.
  /// \details        std::vector<float> param = std::get<0>(result);
  /// \details        double x_0 = param[0];
  /// \details        double x_1 = param[1];
  std::tuple<std::vector<double>, std::vector<double>, int64_t, int64_t, int64_t> optimize();

  /// \brief Solves convex quadratic programs (QPs) using the OSQP solver.
  /// \return The function returns a tuple containing the solution as two float vectors.
  /// \return The first element of the tuple contains the 'primal' solution.
  /// \return The second element contains the 'lagrange multiplier' solution.
  /// \return The third element contains an integer with solver polish status information.
  /// \details How to use:
  /// \details   1. Generate the Eigen matrices P, A and vectors q, l, u according to the problem.
  /// \details   2. Initialize the interface.
  /// \details        osqp_interface = OSQPInterface(1e-6);
  /// \details   3. Call the optimization function with the problem formulation.
  /// \details        std::tuple<std::vector<double>, std::vector<double>> result;
  /// \details        result = osqp_interface.optimize(P, A, q, l, u, 1e-6);
  /// \details   4. Access the optimized parameters.
  /// \details        std::vector<float> param = std::get<0>(result);
  /// \details        double x_0 = param[0];
  /// \details        double x_1 = param[1];
  std::tuple<std::vector<double>, std::vector<double>, int64_t, int64_t, int64_t> optimize(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u);

  /// \brief Converts the input data and sets up the workspace object.
  /// \param P (n,n) matrix defining relations between parameters.
  /// \param A (m,n) matrix defining parameter constraints relative to the lower and upper bound.
  /// \param q (n) vector defining the linear cost of the problem.
  /// \param l (m) vector defining the lower bound problem constraint.
  /// \param u (m) vector defining the upper bound problem constraint.
  int64_t initializeProblem(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u);
  int64_t initializeProblem(
    CSC_Matrix P, CSC_Matrix A, const std::vector<double> & q, const std::vector<double> & l,
    const std::vector<double> & u);

  // Setter functions for warm start
  bool setWarmStart(
    const std::vector<double> & primal_variables, const std::vector<double> & dual_variables);
  bool setPrimalVariables(const std::vector<double> & primal_variables);
  bool setDualVariables(const std::vector<double> & dual_variables);

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
  void updateEpsAbs(const double eps_abs);
  void updateEpsRel(const double eps_rel);
  void updateMaxIter(const int iter);
  void updateVerbose(const bool verbose);
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
  /// \brief Get the status value for the latest problem solved
  inline int64_t getStatus() const { return static_cast<int64_t>(m_latest_work_info.status_val); }
  /// \brief Get the status polish for the latest problem solved
  inline int64_t getStatusPolish() const
  {
    return static_cast<int64_t>(m_latest_work_info.status_polish);
  }
  /// \brief Get the runtime of the latest problem solved
  inline double getRunTime() const { return m_latest_work_info.run_time; }
  /// \brief Get the objective value the latest problem solved
  inline double getObjVal() const { return m_latest_work_info.obj_val; }
  /// \brief Returns flag asserting interface condition (Healthy condition: 0).
  inline int64_t getExitFlag() const { return m_exitflag; }

  void logUnsolvedStatus(const std::string & prefix_message = "") const;
};

}  // namespace osqp
}  // namespace common
}  // namespace autoware

#endif  // OSQP_INTERFACE__OSQP_INTERFACE_HPP_
