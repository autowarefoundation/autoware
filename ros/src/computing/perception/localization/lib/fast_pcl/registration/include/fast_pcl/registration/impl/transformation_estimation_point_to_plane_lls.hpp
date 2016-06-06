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
#ifndef FAST_PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_LLS_HPP_
#define FAST_PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_LLS_HPP_
#include <pcl/cloud_iterator.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             Matrix4 &transformation_matrix) const
{
  size_t nr_points = cloud_src.points.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationPointToPlaneLLS::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", nr_points, cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);  
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const std::vector<int> &indices_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             Matrix4 &transformation_matrix) const
{
  size_t nr_points = indices_src.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationPointToPlaneLLS::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);  
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const std::vector<int> &indices_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             const std::vector<int> &indices_tgt,
                             Matrix4 &transformation_matrix) const
{
  size_t nr_points = indices_src.size ();
  if (indices_tgt.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationPointToPlaneLLS::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), indices_tgt.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, indices_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);  
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                             const pcl::PointCloud<PointTarget> &cloud_tgt,
                             const pcl::Correspondences &correspondences,
                             Matrix4 &transformation_matrix) const
{
  ConstCloudIterator<PointSource> source_it (cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, correspondences, false);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget, Scalar>::
constructTransformationMatrix (const double & alpha, const double & beta, const double & gamma,
                               const double & tx,    const double & ty,   const double & tz,
                               Matrix4 &transformation_matrix) const
{
  // Construct the transformation matrix from rotation and translation 
  transformation_matrix = Eigen::Matrix<Scalar, 4, 4>::Zero ();
  transformation_matrix (0, 0) = static_cast<Scalar> ( cos (gamma) * cos (beta));
  transformation_matrix (0, 1) = static_cast<Scalar> (-sin (gamma) * cos (alpha) + cos (gamma) * sin (beta) * sin (alpha));
  transformation_matrix (0, 2) = static_cast<Scalar> ( sin (gamma) * sin (alpha) + cos (gamma) * sin (beta) * cos (alpha));
  transformation_matrix (1, 0) = static_cast<Scalar> ( sin (gamma) * cos (beta));
  transformation_matrix (1, 1) = static_cast<Scalar> ( cos (gamma) * cos (alpha) + sin (gamma) * sin (beta) * sin (alpha));
  transformation_matrix (1, 2) = static_cast<Scalar> (-cos (gamma) * sin (alpha) + sin (gamma) * sin (beta) * cos (alpha));
  transformation_matrix (2, 0) = static_cast<Scalar> (-sin (beta));
  transformation_matrix (2, 1) = static_cast<Scalar> ( cos (beta) * sin (alpha));
  transformation_matrix (2, 2) = static_cast<Scalar> ( cos (beta) * cos (alpha));

  transformation_matrix (0, 3) = static_cast<Scalar> (tx);
  transformation_matrix (1, 3) = static_cast<Scalar> (ty);
  transformation_matrix (2, 3) = static_cast<Scalar> (tz);
  transformation_matrix (3, 3) = static_cast<Scalar> (1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationPointToPlaneLLS<PointSource, PointTarget, Scalar>::
estimateRigidTransformation (ConstCloudIterator<PointSource>& source_it, ConstCloudIterator<PointTarget>& target_it, Matrix4 &transformation_matrix) const
{
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  Matrix6d ATA;
  Vector6d ATb;
  ATA.setZero ();
  ATb.setZero ();

  // Approximate as a linear least squares problem
  while (source_it.isValid () && target_it.isValid ())
  {
    if (!pcl_isfinite (source_it->x) ||
        !pcl_isfinite (source_it->y) ||
        !pcl_isfinite (source_it->z) ||
        !pcl_isfinite (target_it->x) ||
        !pcl_isfinite (target_it->y) ||
        !pcl_isfinite (target_it->z) ||
        !pcl_isfinite (target_it->normal_x) ||
        !pcl_isfinite (target_it->normal_y) ||
        !pcl_isfinite (target_it->normal_z))
    {
      ++target_it;
      ++source_it;    
      continue;
    }

    const float & sx = source_it->x;
    const float & sy = source_it->y;
    const float & sz = source_it->z;
    const float & dx = target_it->x;
    const float & dy = target_it->y;
    const float & dz = target_it->z;
    const float & nx = target_it->normal[0];
    const float & ny = target_it->normal[1];
    const float & nz = target_it->normal[2];

    double a = nz*sy - ny*sz;
    double b = nx*sz - nz*sx; 
    double c = ny*sx - nx*sy;
   
    //    0  1  2  3  4  5
    //    6  7  8  9 10 11
    //   12 13 14 15 16 17
    //   18 19 20 21 22 23
    //   24 25 26 27 28 29
    //   30 31 32 33 34 35
   
    ATA.coeffRef (0) += a * a;
    ATA.coeffRef (1) += a * b;
    ATA.coeffRef (2) += a * c;
    ATA.coeffRef (3) += a * nx;
    ATA.coeffRef (4) += a * ny;
    ATA.coeffRef (5) += a * nz;
    ATA.coeffRef (7) += b * b;
    ATA.coeffRef (8) += b * c;
    ATA.coeffRef (9) += b * nx;
    ATA.coeffRef (10) += b * ny;
    ATA.coeffRef (11) += b * nz;
    ATA.coeffRef (14) += c * c;
    ATA.coeffRef (15) += c * nx;
    ATA.coeffRef (16) += c * ny;
    ATA.coeffRef (17) += c * nz;
    ATA.coeffRef (21) += nx * nx;
    ATA.coeffRef (22) += nx * ny;
    ATA.coeffRef (23) += nx * nz;
    ATA.coeffRef (28) += ny * ny;
    ATA.coeffRef (29) += ny * nz;
    ATA.coeffRef (35) += nz * nz;

    double d = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
    ATb.coeffRef (0) += a * d;
    ATb.coeffRef (1) += b * d;
    ATb.coeffRef (2) += c * d;
    ATb.coeffRef (3) += nx * d;
    ATb.coeffRef (4) += ny * d;
    ATb.coeffRef (5) += nz * d;

    ++target_it;
    ++source_it;    
  }
  ATA.coeffRef (6) = ATA.coeff (1);
  ATA.coeffRef (12) = ATA.coeff (2);
  ATA.coeffRef (13) = ATA.coeff (8);
  ATA.coeffRef (18) = ATA.coeff (3);
  ATA.coeffRef (19) = ATA.coeff (9);
  ATA.coeffRef (20) = ATA.coeff (15);
  ATA.coeffRef (24) = ATA.coeff (4);
  ATA.coeffRef (25) = ATA.coeff (10);
  ATA.coeffRef (26) = ATA.coeff (16);
  ATA.coeffRef (27) = ATA.coeff (22);
  ATA.coeffRef (30) = ATA.coeff (5);
  ATA.coeffRef (31) = ATA.coeff (11);
  ATA.coeffRef (32) = ATA.coeff (17);
  ATA.coeffRef (33) = ATA.coeff (23);
  ATA.coeffRef (34) = ATA.coeff (29);

  // Solve A*x = b
  Vector6d x = static_cast<Vector6d> (ATA.inverse () * ATb);
  
  // Construct the transformation matrix from x
  constructTransformationMatrix (x (0), x (1), x (2), x (3), x (4), x (5), transformation_matrix);
}
#endif /* FAST_PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_LLS_HPP_ */
