/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#ifndef FAST_PCL_REGISTRATION_DEFAULT_CONVERGENCE_CRITERIA_HPP_
#define FAST_PCL_REGISTRATION_DEFAULT_CONVERGENCE_CRITERIA_HPP_

#include <pcl/console/print.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename Scalar> bool
pcl::registration::DefaultConvergenceCriteria<Scalar>::hasConverged ()
{
  convergence_state_ = CONVERGENCE_CRITERIA_NOT_CONVERGED;

  PCL_DEBUG ("[pcl::DefaultConvergenceCriteria::hasConverged] Iteration %d out of %d.\n", iterations_, max_iterations_);
  // 1. Number of iterations has reached the maximum user imposed number of iterations
  if (iterations_ >= max_iterations_)
  {
    if (failure_after_max_iter_)
      return (false);
    else
    {
      convergence_state_ = CONVERGENCE_CRITERIA_ITERATIONS;
      return (true);
    }
    return (failure_after_max_iter_ ? false : true);
  }

  // 2. The epsilon (difference) between the previous transformation and the current estimated transformation
  double cos_angle = 0.5 * (transformation_.coeff (0, 0) + transformation_.coeff (1, 1) + transformation_.coeff (2, 2) - 1);
  double translation_sqr = transformation_.coeff (0, 3) * transformation_.coeff (0, 3) +
                           transformation_.coeff (1, 3) * transformation_.coeff (1, 3) +
                           transformation_.coeff (2, 3) * transformation_.coeff (2, 3);
  PCL_DEBUG ("[pcl::DefaultConvergenceCriteria::hasConverged] Current transformation gave %f rotation (cosine) and %f translation.\n", cos_angle, translation_sqr);

  if (cos_angle >= rotation_threshold_ && translation_sqr <= translation_threshold_)
  {
    if (iterations_similar_transforms_ < max_iterations_similar_transforms_)
    {
      // Increment the number of transforms that the thresholds are allowed to be similar
      ++iterations_similar_transforms_;
      return (false);
    }
    else
    {
      iterations_similar_transforms_ = 0;
      convergence_state_ = CONVERGENCE_CRITERIA_TRANSFORM;
      return (true);
    }
  }

  correspondences_cur_mse_ = calculateMSE (correspondences_);
  PCL_DEBUG ("[pcl::DefaultConvergenceCriteria::hasConverged] Previous / Current MSE for correspondences distances is: %f / %f.\n", correspondences_prev_mse_, correspondences_cur_mse_);

  // 3. The relative sum of Euclidean squared errors is smaller than a user defined threshold
  // Absolute
  if (fabs (correspondences_cur_mse_ - correspondences_prev_mse_) < mse_threshold_absolute_)
  {
    if (iterations_similar_transforms_ < max_iterations_similar_transforms_)
    {
      // Increment the number of transforms that the thresholds are allowed to be similar
      ++iterations_similar_transforms_;
      return (false);
    }
    else
    {
      iterations_similar_transforms_ = 0;
      convergence_state_ = CONVERGENCE_CRITERIA_ABS_MSE;
      return (true);
    }
  }
  
  // Relative
  if (fabs (correspondences_cur_mse_ - correspondences_prev_mse_) / correspondences_prev_mse_ < mse_threshold_relative_)
  {
    if (iterations_similar_transforms_ < max_iterations_similar_transforms_)
    {
      // Increment the number of transforms that the thresholds are allowed to be similar
      ++iterations_similar_transforms_;
      return (false);
    }
    else
    {
      iterations_similar_transforms_ = 0;
      convergence_state_ = CONVERGENCE_CRITERIA_REL_MSE;
      return (true);
    }
  }

  correspondences_prev_mse_ = correspondences_cur_mse_;

  return (false);
}

#endif    // FAST_PCL_REGISTRATION_DEFAULT_CONVERGENCE_CRITERIA_HPP_
