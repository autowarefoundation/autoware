/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef FAST_PCL_WARP_POINT_RIGID_H_
#define FAST_PCL_WARP_POINT_RIGID_H_

//#include <pcl/registration/eigen.h>
#include "fast_pcl/registration/eigen.h"

namespace pcl
{
  namespace registration
  {
    /** \brief Base warp point class. 
      * 
      * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
      * \author Radu B. Rusu
      * \ingroup registration
      */
    template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
    class WarpPointRigid
    {
      public:
        typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
        typedef Eigen::Matrix<Scalar, 4, 1> Vector4;

        typedef boost::shared_ptr<WarpPointRigid<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPointRigid<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor
          * \param[in] nr_dim the number of dimensions
          */
        WarpPointRigid (int nr_dim)
          : nr_dim_ (nr_dim)
          , transform_matrix_ (Matrix4::Zero ())
        {
          transform_matrix_ (3, 3) = 1.0;
        };

        /** \brief Destructor. */
        virtual ~WarpPointRigid () {};

        /** \brief Set warp parameters. Pure virtual.
          * \param[in] p warp parameters 
          */
        virtual void 
        setParam (const VectorX& p) = 0;

        /** \brief Warp a point given a transformation matrix
          * \param[in] pnt_in the point to warp (transform)
          * \param[out] pnt_out the warped (transformed) point
          */
        inline void 
        warpPoint (const PointSourceT& pnt_in, PointSourceT& pnt_out) const
        {
          pnt_out.x = static_cast<float> (transform_matrix_ (0, 0) * pnt_in.x + transform_matrix_ (0, 1) * pnt_in.y + transform_matrix_ (0, 2) * pnt_in.z + transform_matrix_ (0, 3));
          pnt_out.y = static_cast<float> (transform_matrix_ (1, 0) * pnt_in.x + transform_matrix_ (1, 1) * pnt_in.y + transform_matrix_ (1, 2) * pnt_in.z + transform_matrix_ (1, 3));
          pnt_out.z = static_cast<float> (transform_matrix_ (2, 0) * pnt_in.x + transform_matrix_ (2, 1) * pnt_in.y + transform_matrix_ (2, 2) * pnt_in.z + transform_matrix_ (2, 3));
          //pnt_out.getVector3fMap () = transform_matrix_.topLeftCorner (3, 3) * 
          //                            pnt_in.getVector3fMap () + 
          //                            transform_matrix_.block (0, 3, 3, 1);
          //pnt_out.data[3] = pnt_in.data[3];
        }

        /** \brief Warp a point given a transformation matrix
          * \param[in] pnt_in the point to warp (transform)
          * \param[out] pnt_out the warped (transformed) point
          */
        inline void 
        warpPoint (const PointSourceT& pnt_in, Vector4& pnt_out) const
        {
          pnt_out[0] = static_cast<Scalar> (transform_matrix_ (0, 0) * pnt_in.x + transform_matrix_ (0, 1) * pnt_in.y + transform_matrix_ (0, 2) * pnt_in.z + transform_matrix_ (0, 3));
          pnt_out[1] = static_cast<Scalar> (transform_matrix_ (1, 0) * pnt_in.x + transform_matrix_ (1, 1) * pnt_in.y + transform_matrix_ (1, 2) * pnt_in.z + transform_matrix_ (1, 3));
          pnt_out[2] = static_cast<Scalar> (transform_matrix_ (2, 0) * pnt_in.x + transform_matrix_ (2, 1) * pnt_in.y + transform_matrix_ (2, 2) * pnt_in.z + transform_matrix_ (2, 3));
          pnt_out[3] = 0.0;
        }

        /** \brief Get the number of dimensions. */
        inline int 
        getDimension () const { return (nr_dim_); }

        /** \brief Get the Transform used. */
        inline const Matrix4& 
        getTransform () const { return (transform_matrix_); }
        
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      protected:
        int nr_dim_;
        Matrix4 transform_matrix_;
    };
  } // namespace registration
} // namespace pcl

#endif
