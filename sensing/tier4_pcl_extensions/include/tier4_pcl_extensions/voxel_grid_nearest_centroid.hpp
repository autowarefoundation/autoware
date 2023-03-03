// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#ifndef TIER4_PCL_EXTENSIONS__VOXEL_GRID_NEAREST_CENTROID_HPP_
#define TIER4_PCL_EXTENSIONS__VOXEL_GRID_NEAREST_CENTROID_HPP_

#include <pcl/pcl_config.h>

#if PCL_VERSION < PCL_VERSION_CALC(1, 12, 0)
#include <pcl/filters/boost.h>
#endif

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <map>
#include <vector>

namespace pcl
{
/** \brief A searchable voxel strucure containing the mean and covariance of the data.
 * \note For more information please see
 * <b>Magnusson, M. (2009). The Three-Dimensional Normal-Distributions Transform —
 * an Efficient Representation for Registration, Surface Analysis, and Loop Detection.
 * PhD thesis, Orebro University. Orebro Studies in Technology 36</b>
 * \author Brian Okorn (Space and Naval Warfare Systems Center Pacific)
 */
template <typename PointT>
class VoxelGridNearestCentroid : public VoxelGrid<PointT>
{
protected:
  using VoxelGrid<PointT>::filter_name_;
  using VoxelGrid<PointT>::getClassName;
  using VoxelGrid<PointT>::input_;
  using VoxelGrid<PointT>::indices_;
  using VoxelGrid<PointT>::filter_limit_negative_;
  using VoxelGrid<PointT>::filter_limit_min_;
  using VoxelGrid<PointT>::filter_limit_max_;
  using VoxelGrid<PointT>::filter_field_name_;

  using VoxelGrid<PointT>::downsample_all_data_;
  using VoxelGrid<PointT>::leaf_layout_;
  using VoxelGrid<PointT>::save_leaf_layout_;
  using VoxelGrid<PointT>::leaf_size_;
  using VoxelGrid<PointT>::min_b_;
  using VoxelGrid<PointT>::max_b_;
  using VoxelGrid<PointT>::inverse_leaf_size_;
  using VoxelGrid<PointT>::div_b_;
  // cspell: ignore divb
  using VoxelGrid<PointT>::divb_mul_;

  typedef typename pcl::traits::fieldList<PointT>::type FieldList;
  typedef typename Filter<PointT>::PointCloud PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

public:
  typedef pcl::shared_ptr<VoxelGrid<PointT>> Ptr;
  typedef pcl::shared_ptr<const VoxelGrid<PointT>> ConstPtr;

  /** \brief Simple structure to hold a centroid, covariance and the number of points in a leaf.
   * Inverse covariance, eigen vectors and eigen values are precomputed. */
  struct Leaf
  {
    /** \brief Constructor.
     * Sets \ref nr_points, \ref icov_, \ref mean_ and \ref evals_ to 0 and \ref cov_ and \ref
     * evecs_ to the identity matrix
     */
    Leaf()
    : nr_points(0),
      // mean_ (Eigen::Vector3d::Zero ()),
      centroid()
    // cov_ (Eigen::Matrix3d::Identity ()),
    // icov_ (Eigen::Matrix3d::Zero ()),
    // evecs_ (Eigen::Matrix3d::Identity ()),
    // evals_ (Eigen::Vector3d::Zero ())
    {
    }

    /** \brief Get the voxel covariance.
     * \return covariance matrix
     */
    // Eigen::Matrix3d
    // getCov () const
    // {
    //   return (cov_);
    // }

    /** \brief Get the inverse of the voxel covariance.
     * \return inverse covariance matrix
     */
    // Eigen::Matrix3d
    // getInverseCov () const
    // {
    //   return (icov_);
    // }

    /** \brief Get the voxel centroid.
     * \return centroid
     */
    // Eigen::Vector3d
    // getMean () const
    // {
    //   return (mean_);
    // }

    /** \brief Get the eigen vectors of the voxel covariance.
     * \note Order corresponds with \ref getEvals
     * \return matrix whose columns contain eigen vectors
     */
    // Eigen::Matrix3d
    // getEvecs () const
    // {
    //   return (evecs_);
    // }

    /** \brief Get the eigen values of the voxel covariance.
     * \note Order corresponds with \ref getEvecs
     * \return vector of eigen values
     */
    // Eigen::Vector3d
    // getEvals () const
    // {
    //   return (evals_);
    // }

    /** \brief Get the number of points contained by this voxel.
     * \return number of points
     */
    int getPointCount() const { return nr_points; }

    /** \brief Number of points contained by voxel */
    int nr_points;

    /** \brief 3D voxel centroid */
    // Eigen::Vector3d mean_;

    /** \brief Nd voxel centroid
     * \note Differs from \ref mean_ when color data is used
     */
    Eigen::VectorXf centroid;

    /** \brief Voxel covariance matrix */
    // Eigen::Matrix3d cov_;

    /** \brief Inverse of voxel covariance matrix */
    // Eigen::Matrix3d icov_;

    /** \brief Eigen vectors of voxel covariance matrix */
    // Eigen::Matrix3d evecs_;

    /** \brief Eigen values of voxel covariance matrix */
    // Eigen::Vector3d evals_;

    PointCloud points;
  };

  /** \brief Pointer to VoxelGridNearestCentroid leaf structure */
  typedef Leaf * LeafPtr;

  /** \brief Const pointer to VoxelGridNearestCentroid leaf structure */
  typedef const Leaf * LeafConstPtr;

public:
  /** \brief Constructor.
   * Sets \ref leaf_size_ to 0 and \ref searchable_ to false.
   */
  VoxelGridNearestCentroid()
  : searchable_(true),
    // min_points_per_voxel_ (6),
    min_points_per_voxel_(1),
    // min_covar_eigenvalue_mult_ (0.01),
    leaves_(),
    voxel_centroids_(),
    voxel_centroids_leaf_indices_(),
    kdtree_()
  {
    downsample_all_data_ = false;
    save_leaf_layout_ = false;
    leaf_size_.setZero();
    min_b_.setZero();
    max_b_.setZero();
    filter_name_ = "VoxelGridNearestCentroid";
  }

  /** \brief Set the minimum number of points required for a cell to be used
   *        (must be 3 or greater for covariance calculation).
   * \param[in] min_points_per_voxel the minimum number of points for required
   *                                 for a voxel to be used
   */
  inline void setMinPointPerVoxel(int min_points_per_voxel)
  {
    // if(min_points_per_voxel > 2)
    if (min_points_per_voxel > 1) {
      min_points_per_voxel_ = min_points_per_voxel;
    } else {
      // PCL_WARN ("%s: Covariance calculation requires at least 3 points, setting Min Point per
      // Voxel to 3 ", this->getClassName ().c_str ());
      // min_points_per_voxel_ = 3;
      PCL_WARN(
        "%s: Covariance calculation requires at least 1 points, setting Min Point per Voxel to 3 ",
        this->getClassName().c_str());
      min_points_per_voxel_ = 1;
    }
  }

  /** \brief Get the minimum number of points required for a cell to be used.
   * \return the minimum number of points for required for a voxel to be used
   */
  inline int getMinPointPerVoxel() { return min_points_per_voxel_; }

  /** \brief Set the minimum allowable ratio between eigenvalues to prevent singular covariance
   * matrices. \param[in] min_covar_eigenvalue_mult the minimum allowable ratio between eigenvalues
   */
  // inline void
  // setCovEigValueInflationRatio (double min_covar_eigenvalue_mult)
  // {
  //   min_covar_eigenvalue_mult_ = min_covar_eigenvalue_mult;
  // }

  /** \brief Get the minimum allowable ratio between eigenvalues to prevent singular covariance
   * matrices. \return the minimum allowable ratio between eigenvalues
   */
  // inline double
  // getCovEigValueInflationRatio ()
  // {
  //   return min_covar_eigenvalue_mult_;
  // }

  /** \brief Filter cloud and initializes voxel structure.
   * \param[out] output cloud containing centroids of voxels containing a sufficient number of
   * points \param[in] searchable flag if voxel structure is searchable, if true then kdtree is
   * built
   */
  inline void filter(PointCloud & output, bool searchable = false)
  {
    searchable_ = searchable;
    applyFilter(output);

    voxel_centroids_ = PointCloudPtr(new PointCloud(output));

    if (searchable_ && voxel_centroids_->size() > 0) {
      // Initiates kdtree of the centroids of voxels containing a sufficient number of points
      kdtree_.setInputCloud(voxel_centroids_);
    }
  }

  /** \brief Initializes voxel structure.
   * \param[in] searchable flag if voxel structure is searchable, if true then kdtree is built
   */
  inline void filter(bool searchable = false)
  {
    searchable_ = searchable;
    voxel_centroids_ = PointCloudPtr(new PointCloud);
    applyFilter(*voxel_centroids_);

    if (searchable_ && voxel_centroids_->size() > 0) {
      // Initiates kdtree of the centroids of voxels containing a sufficient number of points
      kdtree_.setInputCloud(voxel_centroids_);
    }
  }

  /** \brief Get the voxel containing point p.
   * \param[in] index the index of the leaf structure node
   * \return const pointer to leaf structure
   */
  inline LeafConstPtr getLeaf(int index)
  {
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(index);
    if (leaf_iter != leaves_.end()) {
      LeafConstPtr ret(&(leaf_iter->second));
      return ret;
    } else {
      return NULL;
    }
  }

  /** \brief Get the voxel containing point p.
   * \param[in] p the point to get the leaf structure at
   * \return const pointer to leaf structure
   */
  inline LeafConstPtr getLeaf(PointT & p)
  {
    // Generate index associated with p
    int ijk0 = static_cast<int>(floor(p.x * inverse_leaf_size_[0]) - min_b_[0]);
    int ijk1 = static_cast<int>(floor(p.y * inverse_leaf_size_[1]) - min_b_[1]);
    int ijk2 = static_cast<int>(floor(p.z * inverse_leaf_size_[2]) - min_b_[2]);

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    // Find leaf associated with index
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(idx);
    if (leaf_iter != leaves_.end()) {
      // If such a leaf exists return the pointer to the leaf structure
      LeafConstPtr ret(&(leaf_iter->second));
      return ret;
    } else {
      return NULL;
    }
  }

  /** \brief Get the voxel containing point p.
   * \param[in] p the point to get the leaf structure at
   * \return const pointer to leaf structure
   */
  inline LeafConstPtr getLeaf(Eigen::Vector3f & p)
  {
    // Generate index associated with p
    int ijk0 = static_cast<int>(floor(p[0] * inverse_leaf_size_[0]) - min_b_[0]);
    int ijk1 = static_cast<int>(floor(p[1] * inverse_leaf_size_[1]) - min_b_[1]);
    int ijk2 = static_cast<int>(floor(p[2] * inverse_leaf_size_[2]) - min_b_[2]);

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    // Find leaf associated with index
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(idx);
    if (leaf_iter != leaves_.end()) {
      // If such a leaf exists return the pointer to the leaf structure
      LeafConstPtr ret(&(leaf_iter->second));
      return ret;
    } else {
      return NULL;
    }
  }

  /** \brief Get the voxels surrounding point p, not including the voxel containing point p.
   * \note Only voxels containing a sufficient number of points are used (slower than radius search
   * in practice). \param[in] reference_point the point to get the leaf structure at \param[out]
   * neighbors \return number of neighbors found
   */
  // int
  // getNeighborhoodAtPoint (const PointT& reference_point, std::vector<LeafConstPtr> &neighbors);

  /** \brief Get the leaf structure map
   * \return a map containing all leaves
   */
  inline const std::map<size_t, Leaf> & getLeaves() { return leaves_; }

  /** \brief Get a pointcloud containing the voxel centroids
   * \note Only voxels containing a sufficient number of points are used.
   * \return a map containing all leaves
   */
  inline PointCloudPtr getCentroids() { return voxel_centroids_; }

  /** \brief Get a cloud to visualize each voxels normal distribution.
   * \param[out] cell_cloud a cloud created by sampling the normal distributions of each voxel
   */
  // void
  // getDisplayCloud (pcl::PointCloud<PointXYZ>& cell_cloud);

  /** \brief Search for the k-nearest occupied voxels for the given query point.
   * \note Only voxels containing a sufficient number of points are used.
   * \param[in] point the given query point
   * \param[in] k the number of neighbors to search for
   * \param[out] k_leaves the resultant leaves of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
   * \return number of neighbors found
   */
  int nearestKSearch(
    const PointT & point, int k, std::vector<LeafConstPtr> & k_leaves,
    std::vector<float> & k_sqr_distances)
  {
    k_leaves.clear();

    // Check if kdtree has been built
    if (!searchable_) {
      PCL_WARN("%s: Not Searchable", this->getClassName().c_str());
      return 0;
    }

    // Find k-nearest neighbors in the occupied voxel centroid cloud
    std::vector<int> k_indices;
    k = kdtree_.nearestKSearch(point, k, k_indices, k_sqr_distances);

    // Find leaves corresponding to neighbors
    k_leaves.reserve(k);
    for (std::vector<int>::iterator iter = k_indices.begin(); iter != k_indices.end(); iter++) {
      k_leaves.push_back(&leaves_[voxel_centroids_leaf_indices_[*iter]]);
    }
    return k;
  }

  /** \brief Search for the k-nearest occupied voxels for the given query point.
   * \note Only voxels containing a sufficient number of points are used.
   * \param[in] cloud the given query point
   * \param[in] index the index
   * \param[in] k the number of neighbors to search for
   * \param[out] k_leaves the resultant leaves of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
   * \return number of neighbors found
   */
  inline int nearestKSearch(
    const PointCloud & cloud, int index, int k, std::vector<LeafConstPtr> & k_leaves,
    std::vector<float> & k_sqr_distances)
  {
    if (index >= static_cast<int>(cloud.points.size()) || index < 0) {
      return 0;
    }
    return nearestKSearch(cloud.points[index], k, k_leaves, k_sqr_distances);
  }

  /** \brief Search for all the nearest occupied voxels of the query point in a given radius.
   * \note Only voxels containing a sufficient number of points are used.
   * \param[in] point the given query point
   * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
   * \param[out] k_leaves the resultant leaves of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
   * \param[in] max_nn
   * \return number of neighbors found
   */
  int radiusSearch(
    const PointT & point, double radius, std::vector<LeafConstPtr> & k_leaves,
    std::vector<float> & k_sqr_distances, unsigned int max_nn = 0)
  {
    k_leaves.clear();

    // Check if kdtree has been built
    if (!searchable_) {
      PCL_WARN("%s: Not Searchable", this->getClassName().c_str());
      return 0;
    }

    // Find neighbors within radius in the occupied voxel centroid cloud
    std::vector<int> k_indices;
    int k = kdtree_.radiusSearch(point, radius, k_indices, k_sqr_distances, max_nn);

    // Find leaves corresponding to neighbors
    k_leaves.reserve(k);
    for (std::vector<int>::iterator iter = k_indices.begin(); iter != k_indices.end(); iter++) {
      k_leaves.push_back(&leaves_[voxel_centroids_leaf_indices_[*iter]]);
    }
    return k;
  }

  /** \brief Search for all the nearest occupied voxels of the query point in a given radius.
   * \note Only voxels containing a sufficient number of points are used.
   * \param[in] cloud the given query point
   * \param[in] index a valid index in cloud representing a valid (i.e., finite) query point
   * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
   * \param[out] k_leaves the resultant leaves of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
   * \param[in] max_nn
   * \return number of neighbors found
   */
  inline int radiusSearch(
    const PointCloud & cloud, int index, double radius, std::vector<LeafConstPtr> & k_leaves,
    std::vector<float> & k_sqr_distances, unsigned int max_nn = 0)
  {
    if (index >= static_cast<int>(cloud.points.size()) || index < 0) {
      return 0;
    }
    return radiusSearch(cloud.points[index], radius, k_leaves, k_sqr_distances, max_nn);
  }

protected:
  /** \brief Filter cloud and initializes voxel structure.
   * \param[out] output cloud containing centroids of voxels containing a sufficient
   *                    number of points
   */
  void applyFilter(PointCloud & output);

  /** \brief Flag to determine if voxel structure is searchable. */
  bool searchable_;

  /** \brief Minimum points contained with in a voxel to allow it to be useable. */
  int min_points_per_voxel_;

  /** \brief Minimum allowable ratio between eigenvalues to prevent singular covariance
   *         matrices. */
  // double min_covar_eigenvalue_mult_;

  /** \brief Voxel structure containing all leaf nodes (includes voxels with less than
   *         a sufficient number of points). */
  std::map<size_t, Leaf> leaves_;

  /** \brief Point cloud containing centroids of voxels containing atleast
   *         minimum number of points. */
  PointCloudPtr voxel_centroids_;

  /** \brief Indices of leaf structures associated with each point in \ref voxel_centroids_
   *         (used for searching). */
  std::vector<int> voxel_centroids_leaf_indices_;

  /** \brief KdTree generated using \ref voxel_centroids_ (used for searching). */
  KdTreeFLANN<PointT> kdtree_;
};
}  // namespace pcl

#ifdef PCL_NO_PRECOMPILE
//  #include <pcl/filters/impl/voxel_grid_covariance.hpp>
#include <voxel_grid_approxi.hpp>
#endif

#endif  // TIER4_PCL_EXTENSIONS__VOXEL_GRID_NEAREST_CENTROID_HPP_
