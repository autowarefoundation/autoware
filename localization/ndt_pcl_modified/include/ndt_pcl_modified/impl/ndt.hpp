// Copyright 2020 Tier IV, Inc.
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
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_NDT_MODIFIED_IMPL_H_
#define PCL_REGISTRATION_NDT_MODIFIED_IMPL_H_

#include <algorithm>
#include <vector>

template <typename PointSource, typename PointTarget>
void pcl::NormalDistributionsTransformModified<PointSource, PointTarget>::computeTransformation(
  PointCloudSource & output, const Eigen::Matrix4f & guess)
{
  nr_iterations_ = 0;
  converged_ = false;

  double gauss_c1, gauss_c2, gauss_d3;

  // Initializes the gaussian fitting parameters (eq. 6.8) [Magnusson 2009]
  gauss_c1 = 10 * (1 - outlier_ratio_);
  gauss_c2 = outlier_ratio_ / pow(resolution_, 3);
  gauss_d3 = -log(gauss_c2);
  gauss_d1_ = -log(gauss_c1 + gauss_c2) - gauss_d3;
  gauss_d2_ = -2 * log((-log(gauss_c1 * exp(-0.5) + gauss_c2) - gauss_d3) / gauss_d1_);

  if (guess != Eigen::Matrix4f::Identity()) {
    // Initialise final transformation to the guessed one
    final_transformation_ = guess;
    // Apply guessed transformation prior to search for neighbours
    transformPointCloud(output, output, guess);
  }

  // Initialize Point Gradient and Hessian
  point_gradient_.setZero();
  point_gradient_.block(0, 0, 3, 3).setIdentity();
  point_hessian_.setZero();

  Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
  eig_transformation.matrix() = final_transformation_;

  // Convert initial guess matrix to 6 element transformation vector
  Eigen::Matrix<double, 6, 1> p, delta_p, score_gradient;
  Eigen::Vector3f init_translation = eig_transformation.translation();
  Eigen::Vector3f init_rotation = eig_transformation.rotation().eulerAngles(0, 1, 2);
  p << init_translation(0), init_translation(1), init_translation(2), init_rotation(0),
    init_rotation(1), init_rotation(2);

  Eigen::Matrix<double, 6, 6> hessian;

  double score = 0;
  double delta_p_norm;
  // Calculate derivatives of initial transform vector, subsequent derivative calculations are done
  // in the step length determination.
  score = NormalDistributionsTransformModified<PointSource, PointTarget>::computeDerivatives(
    score_gradient, hessian, output, p);

  transformation_array_.clear();
  transformation_array_.push_back(final_transformation_);
  bool converged_rotation = false;
  while (!converged_) {
    // Store previous transformation
    previous_transformation_ = transformation_;

    // Solve for decent direction using newton method, line 23 in Algorithm 2 [Magnusson 2009]
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> sv(
      hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Negative for maximization as opposed to minimization
    delta_p = sv.solve(-score_gradient);

    // Calculate step length with guaranteed sufficient decrease [More, Thuente 1994]
    delta_p_norm = delta_p.norm();

    if (delta_p_norm == 0 || delta_p_norm != delta_p_norm) {
      trans_probability_ = score / static_cast<double>(input_->points.size());
      converged_ = delta_p_norm == delta_p_norm;
      return;
    }

    Eigen::Matrix<double, 6, 1> delta_p_rotation = delta_p;
    delta_p_rotation(0) = delta_p_rotation(1) = delta_p_rotation(2) = 0;
    double delta_p_rotation_norm = delta_p_rotation.norm();

    Eigen::Matrix<double, 6, 1> score_gradient_rotation = score_gradient;
    score_gradient_rotation(0) = score_gradient_rotation(1) = score_gradient_rotation(2) = 0;

    delta_p.normalize();
    delta_p_rotation.normalize();

    if (!converged_rotation && delta_p_rotation_norm > 0.001 && nr_iterations_ < 10) {
      delta_p = delta_p_rotation;
      delta_p_norm = delta_p_rotation_norm;
      score_gradient = score_gradient_rotation;
      step_size_ = 0.01;
      transformation_epsilon_ = 0.001;
      delta_p_norm = computeStepLengthMT(
        p, delta_p, delta_p_norm, step_size_, transformation_epsilon_ / 2, score, score_gradient,
        hessian, output);
    } else {
      converged_rotation = true;
      transformation_epsilon_ = 0.01;
      step_size_ = 0.1;
      delta_p_norm = computeStepLengthMT(
        p, delta_p, delta_p_norm, step_size_, transformation_epsilon_ / 2, score, score_gradient,
        hessian, output);
    }

    delta_p *= delta_p_norm;

    transformation_ =
      (Eigen::Translation<float, 3>(
         static_cast<float>(delta_p(0)), static_cast<float>(delta_p(1)),
         static_cast<float>(delta_p(2))) *
       Eigen::AngleAxis<float>(static_cast<float>(delta_p(3)), Eigen::Vector3f::UnitX()) *
       Eigen::AngleAxis<float>(static_cast<float>(delta_p(4)), Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxis<float>(static_cast<float>(delta_p(5)), Eigen::Vector3f::UnitZ()))
        .matrix();

    transformation_array_.push_back(final_transformation_);
    p = p + delta_p;

    // Update Visualizer (untested)
    if (update_visualizer_ != 0) {
      update_visualizer_(output, std::vector<int>(), *target_, std::vector<int>());
    }

    if (
      nr_iterations_ > max_iterations_ || (converged_rotation && nr_iterations_ &&
                                           (std::fabs(delta_p_norm) < transformation_epsilon_))) {
      converged_ = true;
    }

    nr_iterations_++;
  }

  // Store transformation probability.  The realtive differences within each scan registration are
  // accurate but the normalization constants need to be modified for it to be globally accurate
  trans_probability_ = score / static_cast<double>(input_->points.size());

  hessian_ = hessian;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
double pcl::NormalDistributionsTransformModified<PointSource, PointTarget>::computeStepLengthMT(
  const Eigen::Matrix<double, 6, 1> & x, Eigen::Matrix<double, 6, 1> & step_dir, double step_init,
  double step_max, double step_min, double & score, Eigen::Matrix<double, 6, 1> & score_gradient,
  Eigen::Matrix<double, 6, 6> & hessian, PointCloudSource & trans_cloud)
{
  // Set the value of phi(0), Equation 1.3 [More, Thuente 1994]
  double phi_0 = -score;
  // Set the value of phi'(0), Equation 1.3 [More, Thuente 1994]
  double d_phi_0 = -(score_gradient.dot(step_dir));

  Eigen::Matrix<double, 6, 1> x_t;

  if (d_phi_0 >= 0) {
    // Not a decent direction
    if (d_phi_0 == 0) {
      return 0;
    } else {
      // Reverse step direction and calculate optimal step.
      d_phi_0 *= -1;
      step_dir *= -1;
    }
  }

  // The Search Algorithm for T(mu) [More, Thuente 1994]

  int max_step_iterations = 10;
  int step_iterations = 0;

  // Sufficient decrease constant, Equation 1.1 [More, Thuete 1994]
  double mu = 1.e-4;
  // Curvature condition constant, Equation 1.2 [More, Thuete 1994]
  double nu = 0.9;

  // Initial endpoints of Interval I,
  double a_l = 0, a_u = 0;

  // Auxiliary function psi is used until I is determined ot be a closed interval, Equation 2.1
  // [More, Thuente 1994]
  double f_l =
    NormalDistributionsTransformModified<PointSource, PointTarget>::auxilaryFunction_PsiMT(
      a_l, phi_0, phi_0, d_phi_0, mu);
  double g_l =
    NormalDistributionsTransformModified<PointSource, PointTarget>::auxilaryFunction_dPsiMT(
      d_phi_0, d_phi_0, mu);

  double f_u =
    NormalDistributionsTransformModified<PointSource, PointTarget>::auxilaryFunction_PsiMT(
      a_u, phi_0, phi_0, d_phi_0, mu);
  double g_u =
    NormalDistributionsTransformModified<PointSource, PointTarget>::auxilaryFunction_dPsiMT(
      d_phi_0, d_phi_0, mu);

  // Check used to allow More-Thuente step length calculation to be skipped by making step_min ==
  // step_max
  bool interval_converged = (step_max - step_min) > 0, open_interval = true;

  double a_t = step_init;
  a_t = std::min(a_t, step_max);
  a_t = std::max(a_t, step_min);

  x_t = x + step_dir * a_t;

  final_transformation_ =
    (Eigen::Translation<float, 3>(
       static_cast<float>(x_t(0)), static_cast<float>(x_t(1)), static_cast<float>(x_t(2))) *
     Eigen::AngleAxis<float>(static_cast<float>(x_t(3)), Eigen::Vector3f::UnitX()) *
     Eigen::AngleAxis<float>(static_cast<float>(x_t(4)), Eigen::Vector3f::UnitY()) *
     Eigen::AngleAxis<float>(static_cast<float>(x_t(5)), Eigen::Vector3f::UnitZ()))
      .matrix();

  // New transformed point cloud
  transformPointCloud(*input_, trans_cloud, final_transformation_);

  // Updates score, gradient and hessian.  Hessian calculation is unnecessary but testing showed
  // that most step calculations use the initial step suggestion and recalculation the reusable
  // portions of the hessian would intail more computation time.
  score = NormalDistributionsTransformModified<PointSource, PointTarget>::computeDerivatives(
    score_gradient, hessian, trans_cloud, x_t, true);

  // Calculate phi(alpha_t)
  double phi_t = -score;
  // Calculate phi'(alpha_t)
  double d_phi_t = -(score_gradient.dot(step_dir));

  // Calculate psi(alpha_t)
  double psi_t =
    NormalDistributionsTransformModified<PointSource, PointTarget>::auxilaryFunction_PsiMT(
      a_t, phi_t, phi_0, d_phi_0, mu);
  // Calculate psi'(alpha_t)
  double d_psi_t =
    NormalDistributionsTransformModified<PointSource, PointTarget>::auxilaryFunction_dPsiMT(
      d_phi_t, d_phi_0, mu);

  // Iterate until max number of iterations, interval convergence or a value satisfies the
  // sufficient decrease, Equation 1.1, and curvature condition, Equation 1.2 [More, Thuente 1994]

  while (!interval_converged && step_iterations < max_step_iterations &&
         !(psi_t <= 0 /*Sufficient Decrease*/ && d_phi_t <= -nu * d_phi_0 /*Curvature Condition*/))
  // while (!interval_converged && step_iterations < max_step_iterations && !(psi_t <= 0))
  {
    // Use auxiliary function if interval I is not closed
    if (open_interval) {
      a_t = NormalDistributionsTransformModified<PointSource, PointTarget>::trialValueSelectionMT(
        a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
    } else {
      a_t = NormalDistributionsTransformModified<PointSource, PointTarget>::trialValueSelectionMT(
        a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
    }

    a_t = std::min(a_t, step_max);
    a_t = std::max(a_t, step_min);

    x_t = x + step_dir * a_t;

    final_transformation_ =
      (Eigen::Translation<float, 3>(
         static_cast<float>(x_t(0)), static_cast<float>(x_t(1)), static_cast<float>(x_t(2))) *
       Eigen::AngleAxis<float>(static_cast<float>(x_t(3)), Eigen::Vector3f::UnitX()) *
       Eigen::AngleAxis<float>(static_cast<float>(x_t(4)), Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxis<float>(static_cast<float>(x_t(5)), Eigen::Vector3f::UnitZ()))
        .matrix();

    // New transformed point cloud
    // Done on final cloud to prevent wasted computation
    transformPointCloud(*input_, trans_cloud, final_transformation_);

    // Updates score, gradient. Values stored to prevent wasted computation.
    score = NormalDistributionsTransformModified<PointSource, PointTarget>::computeDerivatives(
      score_gradient, hessian, trans_cloud, x_t, false);

    // Calculate phi(alpha_t+)
    phi_t = -score;
    // Calculate phi'(alpha_t+)
    d_phi_t = -(score_gradient.dot(step_dir));

    // Calculate psi(alpha_t+)
    psi_t = NormalDistributionsTransformModified<PointSource, PointTarget>::auxilaryFunction_PsiMT(
      a_t, phi_t, phi_0, d_phi_0, mu);
    // Calculate psi'(alpha_t+)
    d_psi_t =
      NormalDistributionsTransformModified<PointSource, PointTarget>::auxilaryFunction_dPsiMT(
        d_phi_t, d_phi_0, mu);

    // Check if I is now a closed interval
    if (open_interval && (psi_t <= 0 && d_psi_t >= 0)) {
      open_interval = false;

      // Converts f_l and g_l from psi to phi
      f_l = f_l + phi_0 - mu * d_phi_0 * a_l;
      g_l = g_l + mu * d_phi_0;

      // Converts f_u and g_u from psi to phi
      f_u = f_u + phi_0 - mu * d_phi_0 * a_u;
      g_u = g_u + mu * d_phi_0;
    }

    if (open_interval) {
      // Update interval end points using Updating Algorithm [More, Thuente 1994]
      interval_converged =
        NormalDistributionsTransformModified<PointSource, PointTarget>::updateIntervalMT(
          a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
    } else {
      // Update interval end points using Modified Updating Algorithm [More, Thuente 1994]
      interval_converged =
        NormalDistributionsTransformModified<PointSource, PointTarget>::updateIntervalMT(
          a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
    }

    step_iterations++;
  }

  // If inner loop was run then hessian needs to be calculated.
  // Hessian is unnecessary for step length determination but gradients are required
  // so derivative and transform data is stored for the next iteration.
  if (step_iterations) {
    NormalDistributionsTransformModified<PointSource, PointTarget>::computeHessian(
      hessian, trans_cloud, x_t);
  }

  return a_t;
}

#endif  // PCL_REGISTRATION_NDT_IMPL_MODIFIED_H_
