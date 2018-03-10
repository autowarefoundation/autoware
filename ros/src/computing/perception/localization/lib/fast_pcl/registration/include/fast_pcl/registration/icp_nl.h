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

#ifndef FAST_PCL_ICP_NL_H_
#define FAST_PCL_ICP_NL_H_

// PCL includes
//#include <pcl/registration/icp.h>
//#include <pcl/registration/transformation_estimation_lm.h>
#include "fast_pcl/registration/icp.h"
#include "fast_pcl/registration/transformation_estimation_lm.h"

namespace pcl
{
  /** \brief @b IterativeClosestPointNonLinear is an ICP variant that uses Levenberg-Marquardt optimization 
    * backend. The resultant transformation is optimized as a quaternion.
    *
    * The algorithm has several termination criteria:
    *
    * <ol>
    * <li>Number of iterations has reached the maximum user imposed number of iterations 
    *     (via \ref setMaximumIterations)</li>
    * <li>The epsilon (difference) between the previous transformation and the current estimated transformation is 
    *     smaller than an user imposed value (via \ref setTransformationEpsilon)</li>
    * <li>The sum of Euclidean squared errors is smaller than a user defined threshold 
    *     (via \ref setEuclideanFitnessEpsilon)</li>
    * </ol>
    *
    * \author Radu B. Rusu, Michael Dixon
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget, typename Scalar = float>
  class IterativeClosestPointNonLinear : public IterativeClosestPoint<PointSource, PointTarget, Scalar>
  {
    using IterativeClosestPoint<PointSource, PointTarget, Scalar>::min_number_correspondences_;
    using IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
    using IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformation_estimation_;
    using IterativeClosestPoint<PointSource, PointTarget, Scalar>::computeTransformation;

    public:

      typedef boost::shared_ptr< IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> > Ptr;
      typedef boost::shared_ptr< const IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> > ConstPtr;

      typedef typename Registration<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;

      /** \brief Empty constructor. */
      IterativeClosestPointNonLinear ()
      {
        min_number_correspondences_ = 4;
        reg_name_ = "IterativeClosestPointNonLinear";

        transformation_estimation_.reset (new pcl::registration::TransformationEstimationLM<PointSource, PointTarget, Scalar>);
      }
  };
}

#endif  //#ifndef FAST_PCL_ICP_NL_H_
