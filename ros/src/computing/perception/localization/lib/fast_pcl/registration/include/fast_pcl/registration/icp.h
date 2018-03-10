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

#ifndef FAST_PCL_ICP_H_
#define FAST_PCL_ICP_H_

// PCL includes
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
//#include <pcl/registration/registration.h>
//#include <pcl/registration/transformation_estimation_svd.h>
//#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
//#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/default_convergence_criteria.h>
#include "fast_pcl/registration/registration.h"
#include "fast_pcl/registration/transformation_estimation_svd.h"
#include "fast_pcl/registration/transformation_estimation_point_to_plane_lls.h"
#include "fast_pcl/registration/correspondence_estimation.h"
#include "fast_pcl/registration/default_convergence_criteria.h"


namespace pcl
{
  /** \brief @b IterativeClosestPoint provides a base implementation of the Iterative Closest Point algorithm. 
    * The transformation is estimated based on Singular Value Decomposition (SVD).
    *
    * The algorithm has several termination criteria:
    *
    * <ol>
    * <li>Number of iterations has reached the maximum user imposed number of iterations (via \ref setMaximumIterations)</li>
    * <li>The epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value (via \ref setTransformationEpsilon)</li>
    * <li>The sum of Euclidean squared errors is smaller than a user defined threshold (via \ref setEuclideanFitnessEpsilon)</li>
    * </ol>
    *
    *
    * Usage example:
    * \code
    * IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    * // Set the input source and target
    * icp.setInputCloud (cloud_source);
    * icp.setInputTarget (cloud_target);
    *
    * // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    * icp.setMaxCorrespondenceDistance (0.05);
    * // Set the maximum number of iterations (criterion 1)
    * icp.setMaximumIterations (50);
    * // Set the transformation epsilon (criterion 2)
    * icp.setTransformationEpsilon (1e-8);
    * // Set the euclidean distance difference epsilon (criterion 3)
    * icp.setEuclideanFitnessEpsilon (1);
    *
    * // Perform the alignment
    * icp.align (cloud_source_registered);
    *
    * // Obtain the transformation that aligned cloud_source to cloud_source_registered
    * Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    * \endcode
    *
    * \author Radu B. Rusu, Michael Dixon
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget, typename Scalar = float>
  class IterativeClosestPoint : public Registration<PointSource, PointTarget, Scalar>
  {
    public:
      typedef typename Registration<PointSource, PointTarget, Scalar>::PointCloudSource PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef typename Registration<PointSource, PointTarget, Scalar>::PointCloudTarget PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      typedef boost::shared_ptr<IterativeClosestPoint<PointSource, PointTarget, Scalar> > Ptr;
      typedef boost::shared_ptr<const IterativeClosestPoint<PointSource, PointTarget, Scalar> > ConstPtr;

      using Registration<PointSource, PointTarget, Scalar>::reg_name_;
      using Registration<PointSource, PointTarget, Scalar>::getClassName;
      using Registration<PointSource, PointTarget, Scalar>::input_;
      using Registration<PointSource, PointTarget, Scalar>::indices_;
      using Registration<PointSource, PointTarget, Scalar>::target_;
      using Registration<PointSource, PointTarget, Scalar>::nr_iterations_;
      using Registration<PointSource, PointTarget, Scalar>::max_iterations_;
      using Registration<PointSource, PointTarget, Scalar>::previous_transformation_;
      using Registration<PointSource, PointTarget, Scalar>::final_transformation_;
      using Registration<PointSource, PointTarget, Scalar>::transformation_;
      using Registration<PointSource, PointTarget, Scalar>::transformation_epsilon_;
      using Registration<PointSource, PointTarget, Scalar>::converged_;
      using Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;
      using Registration<PointSource, PointTarget, Scalar>::inlier_threshold_;
      using Registration<PointSource, PointTarget, Scalar>::min_number_correspondences_;
      using Registration<PointSource, PointTarget, Scalar>::update_visualizer_;
      using Registration<PointSource, PointTarget, Scalar>::euclidean_fitness_epsilon_;
      using Registration<PointSource, PointTarget, Scalar>::correspondences_;
      using Registration<PointSource, PointTarget, Scalar>::transformation_estimation_;
      using Registration<PointSource, PointTarget, Scalar>::correspondence_estimation_;
      using Registration<PointSource, PointTarget, Scalar>::correspondence_rejectors_;

      typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr convergence_criteria_;
      typedef typename Registration<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;

      /** \brief Empty constructor. */
      IterativeClosestPoint () 
        : x_idx_offset_ (0)
        , y_idx_offset_ (0)
        , z_idx_offset_ (0)
        , nx_idx_offset_ (0)
        , ny_idx_offset_ (0)
        , nz_idx_offset_ (0)
        , use_reciprocal_correspondence_ (false)
        , source_has_normals_ (false)
        , target_has_normals_ (false)
      {
        reg_name_ = "IterativeClosestPoint";
        transformation_estimation_.reset (new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar> ());
        correspondence_estimation_.reset (new pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>);
        convergence_criteria_.reset(new pcl::registration::DefaultConvergenceCriteria<Scalar> (nr_iterations_, transformation_, *correspondences_));
      };

      /** \brief Empty destructor */
      virtual ~IterativeClosestPoint () {}

      /** \brief Returns a pointer to the DefaultConvergenceCriteria used by the IterativeClosestPoint class.
        * This allows to check the convergence state after the align() method as well as to configure
        * DefaultConvergenceCriteria's parameters not available through the ICP API before the align()
        * method is called. Please note that the align method sets max_iterations_,
        * euclidean_fitness_epsilon_ and transformation_epsilon_ and therefore overrides the default / set
        * values of the DefaultConvergenceCriteria instance.
        * \return Pointer to the IterativeClosestPoint's DefaultConvergenceCriteria.
        */
      inline typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr
      getConvergeCriteria ()
      {
        return convergence_criteria_;
      }

      /** \brief Provide a pointer to the input source 
        * (e.g., the point cloud that we want to align to the target)
        *
        * \param[in] cloud the input point cloud source
        */
      virtual void
      setInputSource (const PointCloudSourceConstPtr &cloud)
      {
        Registration<PointSource, PointTarget, Scalar>::setInputSource (cloud);
        std::vector<pcl::PCLPointField> fields;
        pcl::getFields (*cloud, fields);
        source_has_normals_ = false;
        for (size_t i = 0; i < fields.size (); ++i)
        {
          if      (fields[i].name == "x") x_idx_offset_ = fields[i].offset;
          else if (fields[i].name == "y") y_idx_offset_ = fields[i].offset;
          else if (fields[i].name == "z") z_idx_offset_ = fields[i].offset;
          else if (fields[i].name == "normal_x") 
          {
            source_has_normals_ = true;
            nx_idx_offset_ = fields[i].offset;
          }
          else if (fields[i].name == "normal_y") 
          {
            source_has_normals_ = true;
            ny_idx_offset_ = fields[i].offset;
          }
          else if (fields[i].name == "normal_z") 
          {
            source_has_normals_ = true;
            nz_idx_offset_ = fields[i].offset;
          }
        }
      }
      
      /** \brief Provide a pointer to the input target 
        * (e.g., the point cloud that we want to align to the target)
        *
        * \param[in] cloud the input point cloud target
        */
      virtual void
      setInputTarget (const PointCloudTargetConstPtr &cloud)
      {
        Registration<PointSource, PointTarget, Scalar>::setInputTarget (cloud);
        std::vector<pcl::PCLPointField> fields;
        pcl::getFields (*cloud, fields);
        target_has_normals_ = false;
        for (size_t i = 0; i < fields.size (); ++i)
        {
          if (fields[i].name == "normal_x" || fields[i].name == "normal_y" || fields[i].name == "normal_z") 
          {
            target_has_normals_ = true;
            break;
          }
        }
      }

      /** \brief Set whether to use reciprocal correspondence or not
        *
        * \param[in] use_reciprocal_correspondence whether to use reciprocal correspondence or not
        */
      inline void
      setUseReciprocalCorrespondences (bool use_reciprocal_correspondence)
      {
        use_reciprocal_correspondence_ = use_reciprocal_correspondence;
      }

      /** \brief Obtain whether reciprocal correspondence are used or not */
      inline bool
      getUseReciprocalCorrespondences () const
      {
        return (use_reciprocal_correspondence_);
      }

    protected:

      /** \brief Apply a rigid transform to a given dataset. Here we check whether whether
        * the dataset has surface normals in addition to XYZ, and rotate normals as well.
        * \param[in] input the input point cloud
        * \param[out] output the resultant output point cloud
        * \param[in] transform a 4x4 rigid transformation
        * \note Can be used with cloud_in equal to cloud_out
        */
      virtual void 
      transformCloud (const PointCloudSource &input, 
                      PointCloudSource &output, 
                      const Matrix4 &transform);

      /** \brief Rigid transformation computation method  with initial guess.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        * \param guess the initial guess of the transformation to compute
        */
      virtual void 
      computeTransformation (PointCloudSource &output, const Matrix4 &guess);

      /** \brief Looks at the Estimators and Rejectors and determines whether their blob-setter methods need to be called */
      virtual void
      determineRequiredBlobData ();

      /** \brief XYZ fields offset. */
      size_t x_idx_offset_, y_idx_offset_, z_idx_offset_;

      /** \brief Normal fields offset. */
      size_t nx_idx_offset_, ny_idx_offset_, nz_idx_offset_;

      /** \brief The correspondence type used for correspondence estimation. */
      bool use_reciprocal_correspondence_;

      /** \brief Internal check whether source dataset has normals or not. */
      bool source_has_normals_;
      /** \brief Internal check whether target dataset has normals or not. */
      bool target_has_normals_;

      /** \brief Checks for whether estimators and rejectors need various data */
      bool need_source_blob_, need_target_blob_;
  };

  /** \brief @b IterativeClosestPointWithNormals is a special case of
    * IterativeClosestPoint, that uses a transformation estimated based on
    * Point to Plane distances by default.
    *
    * \author Radu B. Rusu
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget, typename Scalar = float>
  class IterativeClosestPointWithNormals : public IterativeClosestPoint<PointSource, PointTarget, Scalar>
  {
    public:
      typedef typename IterativeClosestPoint<PointSource, PointTarget, Scalar>::PointCloudSource PointCloudSource;
      typedef typename IterativeClosestPoint<PointSource, PointTarget, Scalar>::PointCloudTarget PointCloudTarget;
      typedef typename IterativeClosestPoint<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;

      using IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
      using IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformation_estimation_;
      using IterativeClosestPoint<PointSource, PointTarget, Scalar>::correspondence_rejectors_;

      typedef boost::shared_ptr<IterativeClosestPoint<PointSource, PointTarget, Scalar> > Ptr;
      typedef boost::shared_ptr<const IterativeClosestPoint<PointSource, PointTarget, Scalar> > ConstPtr;

      /** \brief Empty constructor. */
      IterativeClosestPointWithNormals () 
      {
        reg_name_ = "IterativeClosestPointWithNormals";
        transformation_estimation_.reset (new pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget, Scalar> ());
        //correspondence_rejectors_.add
      };
      
      /** \brief Empty destructor */
      virtual ~IterativeClosestPointWithNormals () {}

    protected:

      /** \brief Apply a rigid transform to a given dataset
        * \param[in] input the input point cloud
        * \param[out] output the resultant output point cloud
        * \param[in] transform a 4x4 rigid transformation
        * \note Can be used with cloud_in equal to cloud_out
        */
      virtual void 
      transformCloud (const PointCloudSource &input, 
                      PointCloudSource &output, 
                      const Matrix4 &transform);
  };
}

//#include <pcl/registration/impl/icp.hpp>
#include "fast_pcl/registration/impl/icp.hpp"

#endif  //#ifndef FAST_PCL_ICP_H_
