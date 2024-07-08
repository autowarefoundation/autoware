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

#include "autoware/path_optimizer/state_equation_generator.hpp"

#include "autoware/path_optimizer/mpt_optimizer.hpp"

namespace autoware::path_optimizer
{
// state equation: x = B u + W (u includes x_0)
// NOTE: Originally, x_t+1 = Ad x_t + Bd u + Wd.
StateEquationGenerator::Matrix StateEquationGenerator::calcMatrix(
  const std::vector<ReferencePoint> & ref_points) const
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const size_t D_x = vehicle_model_ptr_->getDimX();
  const size_t D_u = vehicle_model_ptr_->getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  // matrices for whole state equation
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_x, N_x);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(N_x, N_u);
  Eigen::VectorXd W = Eigen::VectorXd::Zero(N_x);

  // matrices for one-step state equation
  Eigen::MatrixXd Ad(D_x, D_x);
  Eigen::MatrixXd Bd(D_x, D_u);
  Eigen::MatrixXd Wd(D_x, 1);

  A.block(0, 0, D_x, D_x) = Eigen::MatrixXd::Identity(D_x, D_x);

  // calculate one-step state equation considering kinematics N_ref times
  for (size_t i = 1; i < N_ref; ++i) {
    // get discrete kinematics matrix A, B, W
    const auto & p = ref_points.at(i - 1);

    // TODO(murooka) use curvature by stabling optimization
    // Currently, when using curvature, the optimization result is weird with sample_map.
    // vehicle_model_ptr_->calculateStateEquationMatrix(Ad, Bd, Wd, p.curvature,
    // p.delta_arc_length);
    vehicle_model_ptr_->calculateStateEquationMatrix(Ad, Bd, Wd, 0.0, p.delta_arc_length);

    A.block(i * D_x, (i - 1) * D_x, D_x, D_x) = Ad;
    B.block(i * D_x, (i - 1) * D_u, D_x, D_u) = Bd;
    W.segment(i * D_x, D_x) = Wd;
  }

  return Matrix{A, B, W};
}

Eigen::VectorXd StateEquationGenerator::predict(
  const StateEquationGenerator::Matrix & mat, const Eigen::VectorXd U) const
{
  return mat.B * U + mat.W;
}
}  // namespace autoware::path_optimizer
