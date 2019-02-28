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

#ifndef PCL_OMP_REGISTRATION_H_
#define PCL_OMP_REGISTRATION_H_

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/boost.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>

namespace pcl_omp
{
  /** \brief @b Registration represents the base registration class for general purpose, ICP-like methods.
    * \author Radu B. Rusu, Michael Dixon
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget, typename Scalar = float>
  class Registration : public pcl::PCLBase<PointSource>
  {
    public:
      typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;

      // using pcl::PCLBase<PointSource>::initCompute;
      using pcl::PCLBase<PointSource>::deinitCompute;
      using pcl::PCLBase<PointSource>::input_;
      using pcl::PCLBase<PointSource>::indices_;

      typedef boost::shared_ptr< Registration<PointSource, PointTarget, Scalar> > Ptr;
      typedef boost::shared_ptr< const Registration<PointSource, PointTarget, Scalar> > ConstPtr;

      typedef typename pcl::registration::CorrespondenceRejector::Ptr CorrespondenceRejectorPtr;
      typedef pcl::search::KdTree<PointTarget> KdTree;
      typedef typename pcl::search::KdTree<PointTarget>::Ptr KdTreePtr;

      typedef pcl::search::KdTree<PointSource> KdTreeReciprocal;
      typedef typename KdTreeReciprocal::Ptr KdTreeReciprocalPtr;

      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

      typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

      typedef typename pcl::registration::TransformationEstimation<PointSource, PointTarget, Scalar> TransformationEstimation;
      typedef typename TransformationEstimation::Ptr TransformationEstimationPtr;
      typedef typename TransformationEstimation::ConstPtr TransformationEstimationConstPtr;

      typedef typename pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> CorrespondenceEstimation;
      typedef typename CorrespondenceEstimation::Ptr CorrespondenceEstimationPtr;
      typedef typename CorrespondenceEstimation::ConstPtr CorrespondenceEstimationConstPtr;

      /** \brief Empty constructor. */
      Registration ()
        : reg_name_ ()
        , tree_ (new KdTree)
        , tree_reciprocal_ (new KdTreeReciprocal)
        , nr_iterations_ (0)
        , max_iterations_ (10)
        , ransac_iterations_ (0)
        , target_ ()
        , final_transformation_ (Matrix4::Identity ())
        , transformation_ (Matrix4::Identity ())
        , previous_transformation_ (Matrix4::Identity ())
        , transformation_epsilon_ (0.0)
        , euclidean_fitness_epsilon_ (-std::numeric_limits<double>::max ())
        , corr_dist_threshold_ (std::sqrt (std::numeric_limits<double>::max ()))
        , inlier_threshold_ (0.05)
        , converged_ (false)
        , min_number_correspondences_ (3)
        , correspondences_ (new pcl::Correspondences)
        , transformation_estimation_ ()
        , correspondence_estimation_ ()
        , correspondence_rejectors_ ()
        , target_cloud_updated_ (true)
        , source_cloud_updated_ (true)
        , force_no_recompute_ (false)
        , force_no_recompute_reciprocal_ (false)
        , update_visualizer_ (NULL)
        , point_representation_ ()
      {
      }

      /** \brief destructor. */
      virtual ~Registration () {}

      /** \brief Provide a pointer to the transformation estimation object.
        * (e.g., SVD, point to plane etc.)
        *
        * \param[in] te is the pointer to the corresponding transformation estimation object
        *
        * Code example:
        *
        * \code
        * TransformationEstimationPointToPlaneLLS<PointXYZ, PointXYZ>::Ptr trans_lls (new TransformationEstimationPointToPlaneLLS<PointXYZ, PointXYZ>);
        * icp.setTransformationEstimation (trans_lls);
        * // or...
        * TransformationEstimationSVD<PointXYZ, PointXYZ>::Ptr trans_svd (new TransformationEstimationSVD<PointXYZ, PointXYZ>);
        * icp.setTransformationEstimation (trans_svd);
        * \endcode
        */
      void
      setTransformationEstimation (const TransformationEstimationPtr &te) { transformation_estimation_ = te; }

      /** \brief Provide a pointer to the correspondence estimation object.
        * (e.g., regular, reciprocal, normal shooting etc.)
        *
        * \param[in] ce is the pointer to the corresponding correspondence estimation object
        *
        * Code example:
        *
        * \code
        * CorrespondenceEstimation<PointXYZ, PointXYZ>::Ptr ce (new CorrespondenceEstimation<PointXYZ, PointXYZ>);
        * ce->setInputSource (source);
        * ce->setInputTarget (target);
        * icp.setCorrespondenceEstimation (ce);
        * // or...
        * CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormal, PointNormal>);
        * ce->setInputSource (source);
        * ce->setInputTarget (target);
        * ce->setSourceNormals (source);
        * ce->setTargetNormals (target);
        * icp.setCorrespondenceEstimation (cens);
        * \endcode
        */
      void
      setCorrespondenceEstimation (const CorrespondenceEstimationPtr &ce) { correspondence_estimation_ = ce; }

      /** \brief Provide a pointer to the input source
        * (e.g., the point cloud that we want to align to the target)
        *
        * \param[in] cloud the input point cloud source
        */
      PCL_DEPRECATED ("[pcl::registration::Registration::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.")
      void
      setInputCloud (const PointCloudSourceConstPtr &cloud);

      /** \brief Get a pointer to the input point cloud dataset target. */
      PCL_DEPRECATED ("[pcl::registration::Registration::getInputCloud] getInputCloud is deprecated. Please use getInputSource instead.")
      PointCloudSourceConstPtr const
      getInputCloud ();

      /** \brief Provide a pointer to the input source
        * (e.g., the point cloud that we want to align to the target)
        *
        * \param[in] cloud the input point cloud source
        */
      virtual void
      setInputSource (const PointCloudSourceConstPtr &cloud)
      {
        source_cloud_updated_ = true;
        pcl::PCLBase<PointSource>::setInputCloud (cloud);
      }

      /** \brief Get a pointer to the input point cloud dataset target. */
      inline PointCloudSourceConstPtr const
      getInputSource () { return (input_ ); }

      /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)
        * \param[in] cloud the input point cloud target
        */
      virtual inline void
      setInputTarget (const PointCloudTargetConstPtr &cloud);

      /** \brief Get a pointer to the input point cloud dataset target. */
      inline PointCloudTargetConstPtr const
      getInputTarget () { return (target_ ); }


      /** \brief Provide a pointer to the search object used to find correspondences in
        * the target cloud.
        * \param[in] tree a pointer to the spatial search object.
        * \param[in] force_no_recompute If set to true, this tree will NEVER be
        * recomputed, regardless of calls to setInputTarget. Only use if you are
        * confident that the tree will be set correctly.
        */
      inline void
      setSearchMethodTarget (const KdTreePtr &tree,
                             bool force_no_recompute = false)
      {
        tree_ = tree;
        if (force_no_recompute)
        {
          force_no_recompute_ = true;
        }
        // Since we just set a new tree, we need to check for updates
        target_cloud_updated_ = true;
      }

      /** \brief Get a pointer to the search method used to find correspondences in the
        * target cloud. */
      inline KdTreePtr
      getSearchMethodTarget () const
      {
        return (tree_);
      }

      /** \brief Provide a pointer to the search object used to find correspondences in
        * the source cloud (usually used by reciprocal correspondence finding).
        * \param[in] tree a pointer to the spatial search object.
        * \param[in] force_no_recompute If set to true, this tree will NEVER be
        * recomputed, regardless of calls to setInputSource. Only use if you are
        * extremely confident that the tree will be set correctly.
        */
      inline void
      setSearchMethodSource (const KdTreeReciprocalPtr &tree,
                             bool force_no_recompute = false)
      {
        tree_reciprocal_ = tree;
        if ( force_no_recompute )
        {
          force_no_recompute_reciprocal_ = true;
        }
        // Since we just set a new tree, we need to check for updates
        source_cloud_updated_ = true;
      }

      /** \brief Get a pointer to the search method used to find correspondences in the
        * source cloud. */
      inline KdTreeReciprocalPtr
      getSearchMethodSource () const
      {
        return (tree_reciprocal_);
      }

      /** \brief Get the final transformation matrix estimated by the registration method. */
      inline Matrix4
      getFinalTransformation () { return (final_transformation_); }

      /** \brief Get the last incremental transformation matrix estimated by the registration method. */
      inline Matrix4
      getLastIncrementalTransformation () { return (transformation_); }

      /** \brief Set the maximum number of iterations the internal optimization should run for.
        * \param[in] nr_iterations the maximum number of iterations the internal optimization should run for
        */
      inline void
      setMaximumIterations (int nr_iterations) { max_iterations_ = nr_iterations; }

      /** \brief Get the maximum number of iterations the internal optimization should run for, as set by the user. */
      inline int
      getMaximumIterations () { return (max_iterations_); }

      /** \brief Set the number of iterations RANSAC should run for.
        * \param[in] ransac_iterations is the number of iterations RANSAC should run for
        */
      inline void
      setRANSACIterations (int ransac_iterations) { ransac_iterations_ = ransac_iterations; }

      /** \brief Get the number of iterations RANSAC should run for, as set by the user. */
      inline double
      getRANSACIterations () { return (ransac_iterations_); }

      /** \brief Set the inlier distance threshold for the internal RANSAC outlier rejection loop.
        *
        * The method considers a point to be an inlier, if the distance between the target data index and the transformed
        * source index is smaller than the given inlier distance threshold.
        * The value is set by default to 0.05m.
        * \param[in] inlier_threshold the inlier distance threshold for the internal RANSAC outlier rejection loop
        */
      inline void
      setRANSACOutlierRejectionThreshold (double inlier_threshold) { inlier_threshold_ = inlier_threshold; }

      /** \brief Get the inlier distance threshold for the internal outlier rejection loop as set by the user. */
      inline double
      getRANSACOutlierRejectionThreshold () { return (inlier_threshold_); }

      /** \brief Set the maximum distance threshold between two correspondent points in source <-> target. If the
        * distance is larger than this threshold, the points will be ignored in the alignment process.
        * \param[in] distance_threshold the maximum distance threshold between a point and its nearest neighbor
        * correspondent in order to be considered in the alignment process
        */
      inline void
      setMaxCorrespondenceDistance (double distance_threshold) { corr_dist_threshold_ = distance_threshold; }

      /** \brief Get the maximum distance threshold between two correspondent points in source <-> target. If the
        * distance is larger than this threshold, the points will be ignored in the alignment process.
        */
      inline double
      getMaxCorrespondenceDistance () { return (corr_dist_threshold_); }

      /** \brief Set the transformation epsilon (maximum allowable difference between two consecutive
        * transformations) in order for an optimization to be considered as having converged to the final
        * solution.
        * \param[in] epsilon the transformation epsilon in order for an optimization to be considered as having
        * converged to the final solution.
        */
      inline void
      setTransformationEpsilon (double epsilon) { transformation_epsilon_ = epsilon; }

      /** \brief Get the transformation epsilon (maximum allowable difference between two consecutive
        * transformations) as set by the user.
        */
      inline double
      getTransformationEpsilon () { return (transformation_epsilon_); }

      /** \brief Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before
        * the algorithm is considered to have converged.
        * The error is estimated as the sum of the differences between correspondences in an Euclidean sense,
        * divided by the number of correspondences.
        * \param[in] epsilon the maximum allowed distance error before the algorithm will be considered to have
        * converged
        */

      inline void
      setEuclideanFitnessEpsilon (double epsilon) { euclidean_fitness_epsilon_ = epsilon; }

      /** \brief Get the maximum allowed distance error before the algorithm will be considered to have converged,
        * as set by the user. See \ref setEuclideanFitnessEpsilon
        */
      inline double
      getEuclideanFitnessEpsilon () { return (euclidean_fitness_epsilon_); }

      /** \brief Provide a boost shared pointer to the PointRepresentation to be used when comparing points
        * \param[in] point_representation the PointRepresentation to be used by the k-D tree
        */
      inline void
      setPointRepresentation (const PointRepresentationConstPtr &point_representation)
      {
        point_representation_ = point_representation;
      }

      /** \brief Register the user callback function which will be called from registration thread
       * in order to update point cloud obtained after each iteration
       * \param[in] visualizerCallback reference of the user callback function
       */
      template<typename FunctionSignature> inline bool
      registerVisualizationCallback (boost::function<FunctionSignature> &visualizerCallback)
      {
        if (visualizerCallback != NULL)
        {
          update_visualizer_ = visualizerCallback;
          return (true);
        }
        else
          return (false);
      }

      /** \brief Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
        * \param[in] max_range maximum allowable distance between a point and its correspondence in the target
        * (default: double::max)
        */
      inline double
      getFitnessScore (double max_range = std::numeric_limits<double>::max ());


      /** \brief Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
        * from two sets of correspondence distances (distances between source and target points)
        * \param[in] distances_a the first set of distances between correspondences
        * \param[in] distances_b the second set of distances between correspondences
        */
      inline double
      getFitnessScore (const std::vector<float> &distances_a, const std::vector<float> &distances_b);

      /** \brief Return the state of convergence after the last align run */
      inline bool
      hasConverged () { return (converged_); }

      /** \brief Call the registration algorithm which estimates the transformation and returns the transformed source
        * (input) as \a output.
        * \param[out] output the resultant input transfomed point cloud dataset
        */
      inline void
      align (PointCloudSource &output);

      /** \brief Call the registration algorithm which estimates the transformation and returns the transformed source
        * (input) as \a output.
        * \param[out] output the resultant input transfomed point cloud dataset
        * \param[in] guess the initial gross estimation of the transformation
        */
      inline void
      align (PointCloudSource &output, const Matrix4& guess);

      /** \brief Abstract class get name method. */
      inline const std::string&
      getClassName () const { return (reg_name_); }

      /** \brief Internal computation initalization. */
      bool
      initCompute ();

      /** \brief Internal computation when reciprocal lookup is needed */
      bool
      initComputeReciprocal ();

      /** \brief Add a new correspondence rejector to the list
        * \param[in] rejector the new correspondence rejector to concatenate
        *
        * Code example:
        *
        * \code
        * CorrespondenceRejectorDistance rej;
        * rej.setInputCloud<PointXYZ> (keypoints_src);
        * rej.setInputTarget<PointXYZ> (keypoints_tgt);
        * rej.setMaximumDistance (1);
        * rej.setInputCorrespondences (all_correspondences);
        *
        * // or...
        *
        * \endcode
        */
      inline void
      addCorrespondenceRejector (const CorrespondenceRejectorPtr &rejector)
      {
        correspondence_rejectors_.push_back (rejector);
      }

      /** \brief Get the list of correspondence rejectors. */
      inline std::vector<CorrespondenceRejectorPtr>
      getCorrespondenceRejectors ()
      {
        return (correspondence_rejectors_);
      }

      /** \brief Remove the i-th correspondence rejector in the list
        * \param[in] i the position of the correspondence rejector in the list to remove
        */
      inline bool
      removeCorrespondenceRejector (unsigned int i)
      {
        if (i >= correspondence_rejectors_.size ())
          return (false);
        correspondence_rejectors_.erase (correspondence_rejectors_.begin () + i);
        return (true);
      }

      /** \brief Clear the list of correspondence rejectors. */
      inline void
      clearCorrespondenceRejectors ()
      {
        correspondence_rejectors_.clear ();
      }

    protected:
      /** \brief The registration method name. */
      std::string reg_name_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief A pointer to the spatial search object of the source. */
      KdTreeReciprocalPtr tree_reciprocal_;

      /** \brief The number of iterations the internal optimization ran for (used internally). */
      int nr_iterations_;

      /** \brief The maximum number of iterations the internal optimization should run for.
        * The default value is 10.
        */
      int max_iterations_;

      /** \brief The number of iterations RANSAC should run for. */
      int ransac_iterations_;

      /** \brief The input point cloud dataset target. */
      PointCloudTargetConstPtr target_;

      /** \brief The final transformation matrix estimated by the registration method after N iterations. */
      Matrix4 final_transformation_;

      /** \brief The transformation matrix estimated by the registration method. */
      Matrix4 transformation_;

      /** \brief The previous transformation matrix estimated by the registration method (used internally). */
      Matrix4 previous_transformation_;

      /** \brief The maximum difference between two consecutive transformations in order to consider convergence
        * (user defined).
        */
      double transformation_epsilon_;

      /** \brief The maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the
        * algorithm is considered to have converged. The error is estimated as the sum of the differences between
        * correspondences in an Euclidean sense, divided by the number of correspondences.
        */
      double euclidean_fitness_epsilon_;

      /** \brief The maximum distance threshold between two correspondent points in source <-> target. If the
        * distance is larger than this threshold, the points will be ignored in the alignement process.
        */
      double corr_dist_threshold_;

      /** \brief The inlier distance threshold for the internal RANSAC outlier rejection loop.
        * The method considers a point to be an inlier, if the distance between the target data index and the transformed
        * source index is smaller than the given inlier distance threshold. The default value is 0.05.
        */
      double inlier_threshold_;

      /** \brief Holds internal convergence state, given user parameters. */
      bool converged_;

      /** \brief The minimum number of correspondences that the algorithm needs before attempting to estimate the
        * transformation. The default value is 3.
        */
      int min_number_correspondences_;

      /** \brief The set of correspondences determined at this ICP step. */
      pcl::CorrespondencesPtr correspondences_;

      /** \brief A TransformationEstimation object, used to calculate the 4x4 rigid transformation. */
      TransformationEstimationPtr transformation_estimation_;

      /** \brief A CorrespondenceEstimation object, used to estimate correspondences between the source and the target cloud. */
      CorrespondenceEstimationPtr correspondence_estimation_;

      /** \brief The list of correspondence rejectors to use. */
      std::vector<CorrespondenceRejectorPtr> correspondence_rejectors_;

      /** \brief Variable that stores whether we have a new target cloud, meaning we need to pre-process it again.
       * This way, we avoid rebuilding the kd-tree for the target cloud every time the determineCorrespondences () method
       * is called. */
      bool target_cloud_updated_;
      /** \brief Variable that stores whether we have a new source cloud, meaning we need to pre-process it again.
       * This way, we avoid rebuilding the reciprocal kd-tree for the source cloud every time the determineCorrespondences () method
       * is called. */
      bool source_cloud_updated_;
      /** \brief A flag which, if set, means the tree operating on the target cloud
       * will never be recomputed*/
      bool force_no_recompute_;

      /** \brief A flag which, if set, means the tree operating on the source cloud
       * will never be recomputed*/
      bool force_no_recompute_reciprocal_;

      /** \brief Callback function to update intermediate source point cloud position during it's registration
        * to the target point cloud.
        */
      boost::function<void(const pcl::PointCloud<PointSource> &cloud_src,
                           const std::vector<int> &indices_src,
                           const pcl::PointCloud<PointTarget> &cloud_tgt,
                           const std::vector<int> &indices_tgt)> update_visualizer_;

      /** \brief Search for the closest nearest neighbor of a given point.
        * \param cloud the point cloud dataset to use for nearest neighbor search
        * \param index the index of the query point
        * \param indices the resultant vector of indices representing the k-nearest neighbors
        * \param distances the resultant distances from the query point to the k-nearest neighbors
        */
      inline bool
      searchForNeighbors (const PointCloudSource &cloud, int index,
                          std::vector<int> &indices, std::vector<float> &distances)
      {
        int k = tree_->nearestKSearch (cloud, index, 1, indices, distances);
        if (k == 0)
          return (false);
        return (true);
      }

      /** \brief Abstract transformation computation method with initial guess */
      virtual void
      computeTransformation (PointCloudSource &output, const Matrix4& guess) = 0;

    private:
      /** \brief The point representation used (internal). */
      PointRepresentationConstPtr point_representation_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
}

#include "pcl_omp_registration/impl/registration.hpp"

#endif  //#ifndef PCL_OMP_REGISTRATION_H_
