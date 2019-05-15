/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_REGISTRATION_NDT_MODIFIED_H_
#define PCL_REGISTRATION_NDT_MODIFIED_H_

#include <pcl/registration/ndt.h>
#include <unsupported/Eigen/NonLinearOptimization>

namespace pcl
{
  template<typename PointSource, typename PointTarget>
  class NormalDistributionsTransformModified : public NormalDistributionsTransform<PointSource, PointTarget>
  {
    protected:
      typedef typename Registration<PointSource, PointTarget>::PointCloudSource PointCloudSource;

    public:
      virtual void
      computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess) override;

      inline const Eigen::Matrix<double, 6, 6>
      getHessian() const
      {
          return hessian_;
      }

    protected:
      using Registration<PointSource, PointTarget>::input_;
      using Registration<PointSource, PointTarget>::target_;
      using Registration<PointSource, PointTarget>::nr_iterations_;
      using Registration<PointSource, PointTarget>::max_iterations_;
      using Registration<PointSource, PointTarget>::previous_transformation_;
      using Registration<PointSource, PointTarget>::final_transformation_;
      using Registration<PointSource, PointTarget>::transformation_;
      using Registration<PointSource, PointTarget>::transformation_epsilon_;
      using Registration<PointSource, PointTarget>::converged_;
      using Registration<PointSource, PointTarget>::update_visualizer_;

      using NormalDistributionsTransform<PointSource, PointTarget>::outlier_ratio_;
      using NormalDistributionsTransform<PointSource, PointTarget>::resolution_;
      using NormalDistributionsTransform<PointSource, PointTarget>::step_size_;
      using NormalDistributionsTransform<PointSource, PointTarget>::gauss_d1_;
      using NormalDistributionsTransform<PointSource, PointTarget>::gauss_d2_;
      using NormalDistributionsTransform<PointSource, PointTarget>::point_gradient_;
      using NormalDistributionsTransform<PointSource, PointTarget>::point_hessian_;
      using NormalDistributionsTransform<PointSource, PointTarget>::trans_probability_;

      Eigen::Matrix<double, 6, 6> hessian_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}

#include <pcl_registration/impl/ndt.hpp>

#endif // PCL_REGISTRATION_NDT_MODIFIED_H_
