/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc
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

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl_omp::Registration<PointSource, PointTarget, Scalar>::setInputCloud (
    const typename pcl_omp::Registration<PointSource, PointTarget, Scalar>::PointCloudSourceConstPtr &cloud)
{
  setInputSource (cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> typename pcl_omp::Registration<PointSource, PointTarget, Scalar>::PointCloudSourceConstPtr const
pcl_omp::Registration<PointSource, PointTarget, Scalar>::getInputCloud ()
{
  return (getInputSource ());
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl_omp::Registration<PointSource, PointTarget, Scalar>::setInputTarget (const PointCloudTargetConstPtr &cloud)
{
  if (cloud->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }
  target_ = cloud;
  target_cloud_updated_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl_omp::Registration<PointSource, PointTarget, Scalar>::initCompute ()
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  // Only update target kd-tree if a new target cloud was set
  if (target_cloud_updated_ && !force_no_recompute_)
  {
    tree_->setInputCloud (target_);
    target_cloud_updated_ = false;
  }


  // Update the correspondence estimation
  if (correspondence_estimation_)
  {
    correspondence_estimation_->setSearchMethodTarget (tree_, force_no_recompute_);
    correspondence_estimation_->setSearchMethodSource (tree_reciprocal_, force_no_recompute_reciprocal_);
  }

  // Note: we /cannot/ update the search method on all correspondence rejectors, because we know
  // nothing about them. If they should be cached, they must be cached individually.

  return (pcl::PCLBase<PointSource>::initCompute ());
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl_omp::Registration<PointSource, PointTarget, Scalar>::initComputeReciprocal ()
{
  if (!input_)
  {
    PCL_ERROR ("[pcl::registration::%s::compute] No input source dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  if (source_cloud_updated_ && !force_no_recompute_reciprocal_)
  {
    tree_reciprocal_->setInputCloud (input_);
    source_cloud_updated_ = false;
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline double
pcl_omp::Registration<PointSource, PointTarget, Scalar>::getFitnessScore (
    const std::vector<float> &distances_a,
    const std::vector<float> &distances_b)
{
  unsigned int nr_elem = static_cast<unsigned int> (std::min (distances_a.size (), distances_b.size ()));
  Eigen::VectorXf map_a = Eigen::VectorXf::Map (&distances_a[0], nr_elem);
  Eigen::VectorXf map_b = Eigen::VectorXf::Map (&distances_b[0], nr_elem);
  return (static_cast<double> ((map_a - map_b).sum ()) / static_cast<double> (nr_elem));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline double
pcl_omp::Registration<PointSource, PointTarget, Scalar>::getFitnessScore (double max_range)
{

  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  PointCloudSource input_transformed;
  // transformPointCloud (*input_, input_transformed, final_transformation_);
  input_transformed.resize (input_->size ());

#ifdef _OPENMP
  int nr = 0;
#pragma omp parallel
  {
#pragma omp for
#endif
  for (size_t i = 0; i < input_->size (); ++i)
  {
    const PointSource &src = input_->points[i];
    PointTarget &tgt = input_transformed.points[i];
    tgt.x = static_cast<float> (final_transformation_ (0, 0) * src.x + final_transformation_ (0, 1) * src.y + final_transformation_ (0, 2) * src.z + final_transformation_ (0, 3));
    tgt.y = static_cast<float> (final_transformation_ (1, 0) * src.x + final_transformation_ (1, 1) * src.y + final_transformation_ (1, 2) * src.z + final_transformation_ (1, 3));
    tgt.z = static_cast<float> (final_transformation_ (2, 0) * src.x + final_transformation_ (2, 1) * src.y + final_transformation_ (2, 2) * src.z + final_transformation_ (2, 3));
  }

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
#ifndef _OPENMP
  int nr = 0;
#endif
#ifdef _OPENMP
#pragma omp for private(nn_dists, nn_indices) reduction(+:fitness_score)
#endif
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] <= max_range)
    {
      // Add to the fitness score
      fitness_score += nn_dists[0];
      nr++;
    }
  }
#ifdef _OPENMP
  }
#endif

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl_omp::Registration<PointSource, PointTarget, Scalar>::align (PointCloudSource &output)
{
  align (output, Matrix4::Identity ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl_omp::Registration<PointSource, PointTarget, Scalar>::align (PointCloudSource &output, const Matrix4& guess)
{
  if (!initCompute ())
    return;

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Copy the header
  output.header   = input_->header;
  // Check if the output will be computed for all points or only a subset
  if (indices_->size () != input_->points.size ())
  {
    output.width    = static_cast<uint32_t> (indices_->size ());
    output.height   = 1;
  }
  else
  {
    output.width    = static_cast<uint32_t> (input_->width);
    output.height   = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Copy the point data to output
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i] = input_->points[(*indices_)[i]];

  // Set the internal point representation of choice unless otherwise noted
  if (point_representation_ && !force_no_recompute_)
    tree_->setPointRepresentation (point_representation_);

  // Perform the actual transformation computation
  converged_ = false;
  final_transformation_ = transformation_ = previous_transformation_ = Matrix4::Identity ();

  // Right before we estimate the transformation, we set all the point.data[3] values to 1 to aid the rigid
  // transformation
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i].data[3] = 1.0;

  computeTransformation (output, guess);

  deinitCompute ();
}
