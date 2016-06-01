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
#ifndef FAST_PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_SVD_HPP_
#define FAST_PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_SVD_HPP_

#include <pcl/common/eigen.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix) const
{
  size_t nr_points = cloud_src.points.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationSVD::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", nr_points, cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix) const
{
  if (indices_src.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::TransformationSVD::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Matrix4 &transformation_matrix) const
{
  if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::TransformationEstimationSVD::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), indices_tgt.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, indices_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const pcl::Correspondences &correspondences,
    Matrix4 &transformation_matrix) const
{
  ConstCloudIterator<PointSource> source_it (cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, correspondences, false);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    ConstCloudIterator<PointSource>& source_it,
    ConstCloudIterator<PointTarget>& target_it,
    Matrix4 &transformation_matrix) const
{
  // Convert to Eigen format
  const int npts = static_cast <int> (source_it.size ());



  if (use_umeyama_)
  {
    Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src (3, npts);
    Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt (3, npts);

    for (int i = 0; i < npts; ++i)
    {
      cloud_src (0, i) = source_it->x;
      cloud_src (1, i) = source_it->y;
      cloud_src (2, i) = source_it->z;
      ++source_it;

      cloud_tgt (0, i) = target_it->x;
      cloud_tgt (1, i) = target_it->y;
      cloud_tgt (2, i) = target_it->z;
      ++target_it;
    }
    
    // Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
    transformation_matrix = pcl::umeyama (cloud_src, cloud_tgt, false);
  }
  else
  {
    source_it.reset (); target_it.reset ();
    // <cloud_src,cloud_src> is the source dataset
    transformation_matrix.setIdentity ();

    Eigen::Matrix<Scalar, 4, 1> centroid_src, centroid_tgt;
    // Estimate the centroids of source, target
    compute3DCentroid (source_it, centroid_src);
    compute3DCentroid (target_it, centroid_tgt);
    source_it.reset (); target_it.reset ();

    // Subtract the centroids from source, target
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean, cloud_tgt_demean;
    demeanPointCloud (source_it, centroid_src, cloud_src_demean);
    demeanPointCloud (target_it, centroid_tgt, cloud_tgt_demean);

    getTransformationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::getTransformationFromCorrelation (
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_src_demean,
    const Eigen::Matrix<Scalar, 4, 1> &centroid_src,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_tgt_demean,
    const Eigen::Matrix<Scalar, 4, 1> &centroid_tgt,
    Matrix4 &transformation_matrix) const
{
  transformation_matrix.setIdentity ();

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix<Scalar, 3, 3> H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner (3, 3);

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3> > svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<Scalar, 3, 3> u = svd.matrixU ();
  Eigen::Matrix<Scalar, 3, 3> v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  Eigen::Matrix<Scalar, 3, 3> R = v * u.transpose ();

  // Return the correct transformation
  transformation_matrix.topLeftCorner (3, 3) = R;
  const Eigen::Matrix<Scalar, 3, 1> Rc (R * centroid_src.head (3));
  transformation_matrix.block (0, 3, 3, 1) = centroid_tgt.head (3) - Rc;
}

//#define PCL_INSTANTIATE_TransformationEstimationSVD(T,U) template class PCL_EXPORTS pcl::registration::TransformationEstimationSVD<T,U>;

#endif /* FAST_PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_SVD_HPP_ */
