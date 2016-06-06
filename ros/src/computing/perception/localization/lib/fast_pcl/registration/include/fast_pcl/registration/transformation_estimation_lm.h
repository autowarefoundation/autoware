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
#ifndef FAST_PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_H_
#define FAST_PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_H_

//#include <pcl/registration/transformation_estimation.h>
//#include <pcl/registration/warp_point_rigid.h>
//#include <pcl/registration/distances.h>
#include "fast_pcl/registration/transformation_estimation.h"
#include "fast_pcl/registration/warp_point_rigid.h"
#include "fast_pcl/registration/distances.h"

namespace pcl
{
  namespace registration
  {
    /** @b TransformationEstimationLM implements Levenberg Marquardt-based
      * estimation of the transformation aligning the given correspondences.
      *
      * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
      * \author Radu B. Rusu
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename MatScalar = float>
    class TransformationEstimationLM : public TransformationEstimation<PointSource, PointTarget, MatScalar>
    {
      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      public:
        typedef boost::shared_ptr<TransformationEstimationLM<PointSource, PointTarget, MatScalar> > Ptr;
        typedef boost::shared_ptr<const TransformationEstimationLM<PointSource, PointTarget, MatScalar> > ConstPtr;

        typedef Eigen::Matrix<MatScalar, Eigen::Dynamic, 1> VectorX;
        typedef Eigen::Matrix<MatScalar, 4, 1> Vector4;
        typedef typename TransformationEstimation<PointSource, PointTarget, MatScalar>::Matrix4 Matrix4;
        
        /** \brief Constructor. */
        TransformationEstimationLM ();

        /** \brief Copy constructor. 
          * \param[in] src the TransformationEstimationLM object to copy into this 
          */
        TransformationEstimationLM (const TransformationEstimationLM &src) : 
          tmp_src_ (src.tmp_src_), 
          tmp_tgt_ (src.tmp_tgt_), 
          tmp_idx_src_ (src.tmp_idx_src_), 
          tmp_idx_tgt_ (src.tmp_idx_tgt_), 
          warp_point_ (src.warp_point_)
        {};

        /** \brief Copy operator. 
          * \param[in] src the TransformationEstimationLM object to copy into this 
          */
        TransformationEstimationLM&
        operator = (const TransformationEstimationLM &src)
        {
          tmp_src_ = src.tmp_src_; 
          tmp_tgt_ = src.tmp_tgt_; 
          tmp_idx_src_ = src.tmp_idx_src_;
          tmp_idx_tgt_ = src.tmp_idx_tgt_; 
          warp_point_ = src.warp_point_;
        }

         /** \brief Destructor. */
        virtual ~TransformationEstimationLM () {};

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix) const;

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix) const;

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from 
          * \a indices_src
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Matrix4 &transformation_matrix) const;

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] correspondences the vector of correspondences between source and target point cloud
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const pcl::Correspondences &correspondences,
            Matrix4 &transformation_matrix) const;

        /** \brief Set the function we use to warp points. Defaults to rigid 6D warp.
          * \param[in] warp_fcn a shared pointer to an object that warps points
          */
        void
        setWarpFunction (const boost::shared_ptr<WarpPointRigid<PointSource, PointTarget, MatScalar> > &warp_fcn)
        {
          warp_point_ = warp_fcn;
        }

      protected:
        /** \brief Compute the distance between a source point and its corresponding target point
          * \param[in] p_src The source point
          * \param[in] p_tgt The target point
          * \return The distance between \a p_src and \a p_tgt
          *
          * \note Older versions of PCL used this method internally for calculating the
          * optimization gradient. Since PCL 1.7, a switch has been made to the 
          * computeDistance method using Vector4 types instead. This method is only 
          * kept for API compatibility reasons.
          */
        virtual MatScalar
        computeDistance (const PointSource &p_src, const PointTarget &p_tgt) const
        {
          Vector4 s (p_src.x, p_src.y, p_src.z, 0);
          Vector4 t (p_tgt.x, p_tgt.y, p_tgt.z, 0);
          return ((s - t).norm ());
        }

        /** \brief Compute the distance between a source point and its corresponding target point
          * \param[in] p_src The source point
          * \param[in] p_tgt The target point
          * \return The distance between \a p_src and \a p_tgt
          *
          * \note A different distance function can be defined by creating a subclass of 
          * TransformationEstimationLM and overriding this method. 
          * (See \a TransformationEstimationPointToPlane)
          */
        virtual MatScalar
        computeDistance (const Vector4 &p_src, const PointTarget &p_tgt) const
        {
          Vector4 t (p_tgt.x, p_tgt.y, p_tgt.z, 0);
          return ((p_src - t).norm ());
        }

        /** \brief Temporary pointer to the source dataset. */
        mutable const PointCloudSource *tmp_src_;

        /** \brief Temporary pointer to the target dataset. */
        mutable const PointCloudTarget  *tmp_tgt_;

        /** \brief Temporary pointer to the source dataset indices. */
        mutable const std::vector<int> *tmp_idx_src_;

        /** \brief Temporary pointer to the target dataset indices. */
        mutable const std::vector<int> *tmp_idx_tgt_;

        /** \brief The parameterized function used to warp the source to the target. */
        boost::shared_ptr<pcl::registration::WarpPointRigid<PointSource, PointTarget, MatScalar> > warp_point_;
        
        /** Base functor all the models that need non linear optimization must
          * define their own one and implement operator() (const Eigen::VectorXd& x, Eigen::VectorXd& fvec)
          * or operator() (const Eigen::VectorXf& x, Eigen::VectorXf& fvec) dependening on the choosen _Scalar
          */
        template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
        struct Functor
        {
          typedef _Scalar Scalar;
          enum 
          {
            InputsAtCompileTime = NX,
            ValuesAtCompileTime = NY
          };
          typedef Eigen::Matrix<_Scalar,InputsAtCompileTime,1> InputType;
          typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,1> ValueType;
          typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

          /** \brief Empty Construtor. */
          Functor () : m_data_points_ (ValuesAtCompileTime) {}

          /** \brief Constructor
            * \param[in] m_data_points number of data points to evaluate.
            */
          Functor (int m_data_points) : m_data_points_ (m_data_points) {}
        
          /** \brief Destructor. */
          virtual ~Functor () {}

          /** \brief Get the number of values. */ 
          int
          values () const { return (m_data_points_); }

          protected:
            int m_data_points_;
        };

        struct OptimizationFunctor : public Functor<MatScalar>
        {
          using Functor<MatScalar>::values;

          /** Functor constructor
            * \param[in] m_data_points the number of data points to evaluate
            * \param[in,out] estimator pointer to the estimator object
            */
          OptimizationFunctor (int m_data_points, 
                               const TransformationEstimationLM *estimator) 
            :  Functor<MatScalar> (m_data_points), estimator_ (estimator) 
          {}

          /** Copy constructor
            * \param[in] src the optimization functor to copy into this
            */
          inline OptimizationFunctor (const OptimizationFunctor &src) : 
            Functor<MatScalar> (src.m_data_points_), estimator_ ()
          {
            *this = src;
          }

          /** Copy operator
            * \param[in] src the optimization functor to copy into this
            */
          inline OptimizationFunctor& 
          operator = (const OptimizationFunctor &src) 
          { 
            Functor<MatScalar>::operator=(src);
            estimator_ = src.estimator_; 
            return (*this); 
          }

          /** \brief Destructor. */
          virtual ~OptimizationFunctor () {}

          /** Fill fvec from x. For the current state vector x fill the f values
            * \param[in] x state vector
            * \param[out] fvec f values vector
            */
          int 
          operator () (const VectorX &x, VectorX &fvec) const;

          const TransformationEstimationLM<PointSource, PointTarget, MatScalar> *estimator_;
        };

        struct OptimizationFunctorWithIndices : public Functor<MatScalar>
        {
          using Functor<MatScalar>::values;

          /** Functor constructor
            * \param[in] m_data_points the number of data points to evaluate
            * \param[in,out] estimator pointer to the estimator object
            */
          OptimizationFunctorWithIndices (int m_data_points, 
                                          const TransformationEstimationLM *estimator) 
            : Functor<MatScalar> (m_data_points), estimator_ (estimator) 
          {}

          /** Copy constructor
            * \param[in] src the optimization functor to copy into this
            */
          inline OptimizationFunctorWithIndices (const OptimizationFunctorWithIndices &src)
            : Functor<MatScalar> (src.m_data_points_), estimator_ ()
          {
            *this = src;
          }

          /** Copy operator
            * \param[in] src the optimization functor to copy into this
            */
          inline OptimizationFunctorWithIndices& 
          operator = (const OptimizationFunctorWithIndices &src) 
          { 
            Functor<MatScalar>::operator=(src);
            estimator_ = src.estimator_; 
            return (*this); 
          }

          /** \brief Destructor. */
          virtual ~OptimizationFunctorWithIndices () {}

          /** Fill fvec from x. For the current state vector x fill the f values
            * \param[in] x state vector
            * \param[out] fvec f values vector
            */
          int 
          operator () (const VectorX &x, VectorX &fvec) const;

          const TransformationEstimationLM<PointSource, PointTarget, MatScalar> *estimator_;
        };
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

//#include <pcl/registration/impl/transformation_estimation_lm.hpp>
#include "fast_pcl/registration/impl/transformation_estimation_lm.hpp"

#endif /* FAST_PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_H_ */

