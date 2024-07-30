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

#include "qp_interface/osqp_interface.hpp"

#include "qp_interface/osqp_csc_matrix_conv.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware::common
{
OSQPInterface::OSQPInterface(
  const bool enable_warm_start, const int max_iteration, const c_float eps_abs,
  const c_float eps_rel, const bool polish, const bool verbose)
: QPInterface(enable_warm_start), work_{nullptr, OSQPWorkspaceDeleter}
{
  settings_ = std::make_unique<OSQPSettings>();
  data_ = std::make_unique<OSQPData>();

  if (settings_) {
    osqp_set_default_settings(settings_.get());
    settings_->alpha = 1.6;  // Change alpha parameter
    settings_->eps_rel = eps_rel;
    settings_->eps_abs = eps_abs;
    settings_->eps_prim_inf = 1.0E-4;
    settings_->eps_dual_inf = 1.0E-4;
    settings_->warm_start = enable_warm_start;
    settings_->max_iter = max_iteration;
    settings_->verbose = verbose;
    settings_->polish = polish;
  }
  exitflag_ = 0;
}

OSQPInterface::OSQPInterface(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const bool enable_warm_start,
  const c_float eps_abs)
: OSQPInterface(enable_warm_start, eps_abs)
{
  initializeProblem(P, A, q, l, u);
}

OSQPInterface::OSQPInterface(
  const CSC_Matrix & P, const CSC_Matrix & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const bool enable_warm_start,
  const c_float eps_abs)
: OSQPInterface(enable_warm_start, eps_abs)
{
  initializeCSCProblemImpl(P, A, q, l, u);
}

OSQPInterface::~OSQPInterface()
{
  if (data_->P) free(data_->P);
  if (data_->A) free(data_->A);
}

void OSQPInterface::initializeProblemImpl(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
  CSC_Matrix A_csc = calCSCMatrix(A);
  initializeCSCProblemImpl(P_csc, A_csc, q, l, u);
}

void OSQPInterface::initializeCSCProblemImpl(
  CSC_Matrix P_csc, CSC_Matrix A_csc, const std::vector<double> & q, const std::vector<double> & l,
  const std::vector<double> & u)
{
  // Dynamic float arrays
  std::vector<double> q_tmp(q.begin(), q.end());
  std::vector<double> l_tmp(l.begin(), l.end());
  std::vector<double> u_tmp(u.begin(), u.end());
  double * q_dyn = q_tmp.data();
  double * l_dyn = l_tmp.data();
  double * u_dyn = u_tmp.data();

  /**********************
   * OBJECTIVE FUNCTION
   **********************/
  param_n_ = static_cast<int>(q.size());
  data_->m = static_cast<int>(l.size());

  /*****************
   * POPULATE DATA
   *****************/
  data_->n = param_n_;
  if (data_->P) free(data_->P);
  data_->P = csc_matrix(
    data_->n, data_->n, static_cast<c_int>(P_csc.vals_.size()), P_csc.vals_.data(),
    P_csc.row_idxs_.data(), P_csc.col_idxs_.data());
  data_->q = q_dyn;
  if (data_->A) free(data_->A);
  data_->A = csc_matrix(
    data_->m, data_->n, static_cast<c_int>(A_csc.vals_.size()), A_csc.vals_.data(),
    A_csc.row_idxs_.data(), A_csc.col_idxs_.data());
  data_->l = l_dyn;
  data_->u = u_dyn;

  // Setup workspace
  OSQPWorkspace * workspace;
  exitflag_ = osqp_setup(&workspace, data_.get(), settings_.get());
  work_.reset(workspace);
  work__initialized = true;
}

void OSQPInterface::OSQPWorkspaceDeleter(OSQPWorkspace * ptr) noexcept
{
  if (ptr != nullptr) {
    osqp_cleanup(ptr);
  }
}

void OSQPInterface::updateEpsAbs(const double eps_abs)
{
  settings_->eps_abs = eps_abs;  // for default setting
  if (work__initialized) {
    osqp_update_eps_abs(work_.get(), eps_abs);  // for current work
  }
}

void OSQPInterface::updateEpsRel(const double eps_rel)
{
  settings_->eps_rel = eps_rel;  // for default setting
  if (work__initialized) {
    osqp_update_eps_rel(work_.get(), eps_rel);  // for current work
  }
}

void OSQPInterface::updateMaxIter(const int max_iter)
{
  settings_->max_iter = max_iter;  // for default setting
  if (work__initialized) {
    osqp_update_max_iter(work_.get(), max_iter);  // for current work
  }
}

void OSQPInterface::updateVerbose(const bool is_verbose)
{
  settings_->verbose = is_verbose;  // for default setting
  if (work__initialized) {
    osqp_update_verbose(work_.get(), is_verbose);  // for current work
  }
}

void OSQPInterface::updateRhoInterval(const int rho_interval)
{
  settings_->adaptive_rho_interval = rho_interval;  // for default setting
}

void OSQPInterface::updateRho(const double rho)
{
  settings_->rho = rho;
  if (work__initialized) {
    osqp_update_rho(work_.get(), rho);
  }
}

void OSQPInterface::updateAlpha(const double alpha)
{
  settings_->alpha = alpha;
  if (work__initialized) {
    osqp_update_alpha(work_.get(), alpha);
  }
}

void OSQPInterface::updateScaling(const int scaling)
{
  settings_->scaling = scaling;
}

void OSQPInterface::updatePolish(const bool polish)
{
  settings_->polish = polish;
  if (work__initialized) {
    osqp_update_polish(work_.get(), polish);
  }
}

void OSQPInterface::updatePolishRefinementIteration(const int polish_refine_iter)
{
  if (polish_refine_iter < 0) {
    std::cerr << "Polish refinement iterations must be positive" << std::endl;
    return;
  }

  settings_->polish_refine_iter = polish_refine_iter;
  if (work__initialized) {
    osqp_update_polish_refine_iter(work_.get(), polish_refine_iter);
  }
}

void OSQPInterface::updateCheckTermination(const int check_termination)
{
  if (check_termination < 0) {
    std::cerr << "Check termination must be positive" << std::endl;
    return;
  }

  settings_->check_termination = check_termination;
  if (work__initialized) {
    osqp_update_check_termination(work_.get(), check_termination);
  }
}

bool OSQPInterface::setWarmStart(
  const std::vector<double> & primal_variables, const std::vector<double> & dual_variables)
{
  return setPrimalVariables(primal_variables) && setDualVariables(dual_variables);
}

bool OSQPInterface::setPrimalVariables(const std::vector<double> & primal_variables)
{
  if (!work__initialized) {
    return false;
  }

  const auto result = osqp_warm_start_x(work_.get(), primal_variables.data());
  if (result != 0) {
    std::cerr << "Failed to set primal variables for warm start" << std::endl;
    return false;
  }

  return true;
}

bool OSQPInterface::setDualVariables(const std::vector<double> & dual_variables)
{
  if (!work__initialized) {
    return false;
  }

  const auto result = osqp_warm_start_y(work_.get(), dual_variables.data());
  if (result != 0) {
    std::cerr << "Failed to set dual variables for warm start" << std::endl;
    return false;
  }

  return true;
}

void OSQPInterface::updateP(const Eigen::MatrixXd & P_new)
{
  /*
  // Transform 'P' into an 'upper trapezoidal matrix'
  Eigen::MatrixXd P_trap = P_new.triangularView<Eigen::Upper>();
  // Transform 'P' into a sparse matrix and extract data as dynamic arrays
  Eigen::SparseMatrix<double> P_sparse = P_trap.sparseView();
  double *P_val_ptr = P_sparse.valuePtr();
  // Convert dynamic 'int' arrays to 'c_int' arrays (OSQP input type)
  c_int P_elem_N = P_sparse.nonZeros();
  */
  CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P_new);
  osqp_update_P(work_.get(), P_csc.vals_.data(), OSQP_NULL, static_cast<c_int>(P_csc.vals_.size()));
}

void OSQPInterface::updateCscP(const CSC_Matrix & P_csc)
{
  osqp_update_P(work_.get(), P_csc.vals_.data(), OSQP_NULL, static_cast<c_int>(P_csc.vals_.size()));
}

void OSQPInterface::updateA(const Eigen::MatrixXd & A_new)
{
  /*
  // Transform 'A' into a sparse matrix and extract data as dynamic arrays
  Eigen::SparseMatrix<double> A_sparse = A_new.sparseView();
  double *A_val_ptr = A_sparse.valuePtr();
  // Convert dynamic 'int' arrays to 'c_int' arrays (OSQP input type)
  c_int A_elem_N = A_sparse.nonZeros();
  */
  CSC_Matrix A_csc = calCSCMatrix(A_new);
  osqp_update_A(work_.get(), A_csc.vals_.data(), OSQP_NULL, static_cast<c_int>(A_csc.vals_.size()));
  return;
}

void OSQPInterface::updateCscA(const CSC_Matrix & A_csc)
{
  osqp_update_A(work_.get(), A_csc.vals_.data(), OSQP_NULL, static_cast<c_int>(A_csc.vals_.size()));
}

void OSQPInterface::updateQ(const std::vector<double> & q_new)
{
  std::vector<double> q_tmp(q_new.begin(), q_new.end());
  double * q_dyn = q_tmp.data();
  osqp_update_lin_cost(work_.get(), q_dyn);
}

void OSQPInterface::updateL(const std::vector<double> & l_new)
{
  std::vector<double> l_tmp(l_new.begin(), l_new.end());
  double * l_dyn = l_tmp.data();
  osqp_update_lower_bound(work_.get(), l_dyn);
}

void OSQPInterface::updateU(const std::vector<double> & u_new)
{
  std::vector<double> u_tmp(u_new.begin(), u_new.end());
  double * u_dyn = u_tmp.data();
  osqp_update_upper_bound(work_.get(), u_dyn);
}

void OSQPInterface::updateBounds(
  const std::vector<double> & l_new, const std::vector<double> & u_new)
{
  std::vector<double> l_tmp(l_new.begin(), l_new.end());
  std::vector<double> u_tmp(u_new.begin(), u_new.end());
  double * l_dyn = l_tmp.data();
  double * u_dyn = u_tmp.data();
  osqp_update_bounds(work_.get(), l_dyn, u_dyn);
}

int OSQPInterface::getIterationNumber() const
{
  return work_->info->iter;
}

std::string OSQPInterface::getStatus() const
{
  return "OSQP_SOLVED";
}

bool OSQPInterface::isSolved() const
{
  return latest_work_info_.status_val == 1;
}

int OSQPInterface::getPolishStatus() const
{
  return static_cast<int>(latest_work_info_.status_polish);
}

std::vector<double> OSQPInterface::getDualSolution() const
{
  double * sol_y = work_->solution->y;
  std::vector<double> dual_solution(sol_y, sol_y + data_->m);
  return dual_solution;
}

std::vector<double> OSQPInterface::optimizeImpl()
{
  osqp_solve(work_.get());

  double * sol_x = work_->solution->x;
  double * sol_y = work_->solution->y;
  std::vector<double> sol_primal(sol_x, sol_x + param_n_);
  std::vector<double> sol_lagrange_multiplier(sol_y, sol_y + data_->m);

  latest_work_info_ = *(work_->info);

  if (!enable_warm_start_) {
    work_.reset();
    work__initialized = false;
  }

  return sol_primal;
}

std::vector<double> OSQPInterface::optimize(
  CSC_Matrix P, CSC_Matrix A, const std::vector<double> & q, const std::vector<double> & l,
  const std::vector<double> & u)
{
  initializeCSCProblemImpl(P, A, q, l, u);
  const auto result = optimizeImpl();

  // show polish status if not successful
  const int status_polish = static_cast<int>(latest_work_info_.status_polish);
  if (status_polish != 1) {
    const auto msg = status_polish == 0    ? "unperformed"
                     : status_polish == -1 ? "unsuccessful"
                                           : "unknown";
    std::cerr << "osqp polish process failed : " << msg << ". The result may be inaccurate"
              << std::endl;
  }

  return result;
}

}  // namespace autoware::common
