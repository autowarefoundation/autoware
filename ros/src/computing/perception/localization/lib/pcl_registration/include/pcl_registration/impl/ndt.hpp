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

template<typename PointSource, typename PointTarget> void
pcl::NormalDistributionsTransformModified<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess)
{
  nr_iterations_ = 0;
  converged_ = false;

  double gauss_c1, gauss_c2, gauss_d3;

  // Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
  gauss_c1 = 10 * (1 - outlier_ratio_);
  gauss_c2 = outlier_ratio_ / pow (resolution_, 3);
  gauss_d3 = -log (gauss_c2);
  gauss_d1_ = -log ( gauss_c1 + gauss_c2 ) - gauss_d3;
  gauss_d2_ = -2 * log ((-log ( gauss_c1 * exp ( -0.5 ) + gauss_c2 ) - gauss_d3) / gauss_d1_);

  if (guess != Eigen::Matrix4f::Identity ())
  {
    // Initialise final transformation to the guessed one
    final_transformation_ = guess;
    // Apply guessed transformation prior to search for neighbours
    transformPointCloud (output, output, guess);
  }

  // Initialize Point Gradient and Hessian
  point_gradient_.setZero ();
  point_gradient_.block(0, 0, 3, 3).setIdentity ();
  point_hessian_.setZero ();

  Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
  eig_transformation.matrix () = final_transformation_;

  // Convert initial guess matrix to 6 element transformation vector
  Eigen::Matrix<double, 6, 1> p, delta_p, score_gradient;
  Eigen::Vector3f init_translation = eig_transformation.translation ();
  Eigen::Vector3f init_rotation = eig_transformation.rotation ().eulerAngles (0, 1, 2);
  p << init_translation (0), init_translation (1), init_translation (2),
  init_rotation (0), init_rotation (1), init_rotation (2);

  Eigen::Matrix<double, 6, 6> hessian;

  double score = 0;
  double delta_p_norm;

  // Calculate derivates of initial transform vector, subsequent derivative calculations are done in the step length determination.
  score = NormalDistributionsTransformModified<PointSource, PointTarget>::computeDerivatives (score_gradient, hessian, output, p);

  while (!converged_)
  {
    // Store previous transformation
    previous_transformation_ = transformation_;

    // Solve for decent direction using newton method, line 23 in Algorithm 2 [Magnusson 2009]
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6> > sv (hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Negative for maximization as opposed to minimization
    delta_p = sv.solve (-score_gradient);

    //Calculate step length with guarnteed sufficient decrease [More, Thuente 1994]
    delta_p_norm = delta_p.norm ();

    if (delta_p_norm == 0 || delta_p_norm != delta_p_norm)
    {
      trans_probability_ = score / static_cast<double> (input_->points.size ());
      converged_ = delta_p_norm == delta_p_norm;
      return;
    }

    delta_p.normalize ();
    delta_p_norm = NormalDistributionsTransformModified<PointSource, PointTarget>::computeStepLengthMT (p, delta_p, delta_p_norm, step_size_, transformation_epsilon_ / 2, score, score_gradient, hessian, output);
    delta_p *= delta_p_norm;


    transformation_ = (Eigen::Translation<float, 3> (static_cast<float> (delta_p (0)), static_cast<float> (delta_p (1)), static_cast<float> (delta_p (2))) *
                       Eigen::AngleAxis<float> (static_cast<float> (delta_p (3)), Eigen::Vector3f::UnitX ()) *
                       Eigen::AngleAxis<float> (static_cast<float> (delta_p (4)), Eigen::Vector3f::UnitY ()) *
                       Eigen::AngleAxis<float> (static_cast<float> (delta_p (5)), Eigen::Vector3f::UnitZ ())).matrix ();


    p = p + delta_p;

    // Update Visualizer (untested)
    if (update_visualizer_ != 0)
      update_visualizer_ (output, std::vector<int>(), *target_, std::vector<int>() );

    if (nr_iterations_ > max_iterations_ ||
        (nr_iterations_ && (std::fabs (delta_p_norm) < transformation_epsilon_)))
    {
      converged_ = true;
    }

    nr_iterations_++;

  }

  // Store transformation probability.  The realtive differences within each scan registration are accurate
  // but the normalization constants need to be modified for it to be globally accurate
  trans_probability_ = score / static_cast<double> (input_->points.size ());

  hessian_ = hessian;
}


#endif // PCL_REGISTRATION_NDT_IMPL_MODIFIED_H_
