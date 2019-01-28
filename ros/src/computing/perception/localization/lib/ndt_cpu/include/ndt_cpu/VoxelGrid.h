/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
 */

#ifndef CPU_VGRID_H_
#define CPU_VGRID_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <float.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "Octree.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace cpu {

template <typename PointSourceType>
class VoxelGrid {
public:
	VoxelGrid();

	/* Set input points */
	void setInput(typename pcl::PointCloud<PointSourceType>::Ptr input);

	/* For each input point, search for voxels whose distance between their centroids and
	 * the input point are less than radius.
	 * The output is a list of candidate voxel ids */
	void radiusSearch(PointSourceType query_point, float radius, std::vector<int> &voxel_ids, int max_nn = INT_MAX);

	int getVoxelNum() const;

	float getMaxX() const;
	float getMaxY() const;
	float getMaxZ() const;

	float getMinX() const;
	float getMinY() const;
	float getMinZ() const;

	float getVoxelX() const;
	float getVoxelY() const;
	float getVoxelZ() const;

	int getMaxBX() const;
	int getMaxBY() const;
	int getMaxBZ() const;

	int getMinBX() const;
	int getMinBY() const;
	int getMinBZ() const;

	int getVgridX() const;
	int getVgridY() const;
	int getVgridZ() const;

	void setLeafSize(float voxel_x, float voxel_y, float voxel_z);

	/* Searching for the nearest point of each input query point.
	 * Return the distance between the query point and its nearest neighbor.
	 * If the distance is larger than max_range, then return DBL_MAX. */

	double nearestNeighborDistance(PointSourceType query_point, float max_range);

	Eigen::Vector3d getCentroid(int voxel_id) const;
	Eigen::Matrix3d getCovariance(int voxel_id) const;
	Eigen::Matrix3d getInverseCovariance(int voxel_id) const;

	void update(typename pcl::PointCloud<PointSourceType>::Ptr new_cloud);

private:

	typedef struct {
		int x, y, z;
	} OctreeDim;

	/* Construct the voxel grid and the build the octree. */
	void initialize();

	/* Put points into voxels */
	void scatterPointsToVoxelGrid();

	/* Compute centroids and covariances of voxels. */
	void computeCentroidAndCovariance();

	/* Find boundaries of input point cloud and compute
	 * the number of necessary voxels as well as boundaries
	 * measured in number of leaf size */
	void findBoundaries();

	void findBoundaries(typename pcl::PointCloud<PointSourceType>::Ptr input_cloud,
							float &max_x, float &max_y, float &max_z,
							float &min_x, float &min_y, float &min_z);

	int voxelId(PointSourceType p);

	int voxelId(PointSourceType p,
				float voxel_x, float voxel_y, float voxel_z,
				int min_b_x, int min_b_y, int min_b_z,
				int vgrid_x, int vgrid_y, int vgrid_z);

	int voxelId(int idx, int idy, int idz,
				int min_b_x, int min_b_y, int min_b_z,
				int size_x, int size_y, int size_z);

	/* Private methods for merging new point cloud to the current point cloud */
	void updateBoundaries(float max_x, float max_y, float max_z,
							float min_x, float min_y, float min_z);

	void updateVoxelContent(typename pcl::PointCloud<PointSourceType>::Ptr new_cloud);

	int nearestVoxel(PointSourceType query_point, Eigen::Matrix<float, 6, 1> boundaries, float max_range);

	int roundUp(int input, int factor);

	int roundDown(int input, int factor);

	int div(int input, int divisor);

	//Coordinate of input points
	typename pcl::PointCloud<PointSourceType>::Ptr source_cloud_;

	int voxel_num_;						// Number of voxels
	float max_x_, max_y_, max_z_;		// Upper bounds of the grid (maximum coordinate)
	float min_x_, min_y_, min_z_;		// Lower bounds of the grid (minimum coordinate)
	float voxel_x_, voxel_y_, voxel_z_;	// Leaf size, a.k.a, size of each voxel

	int max_b_x_, max_b_y_, max_b_z_;	// Upper bounds of the grid, measured in number of voxels
	int min_b_x_, min_b_y_, min_b_z_;	// Lower bounds of the grid, measured in number of voxels
	int vgrid_x_, vgrid_y_, vgrid_z_;	// Size of the voxel grid, measured in number of voxels
	int min_points_per_voxel_;			// Minimum number of points per voxel. If the number of points
										// per voxel is less than this number, then the voxel is ignored
										// during computation (treated like it contains no point)

	boost::shared_ptr<std::vector<Eigen::Vector3d> > centroid_;			// 3x1 Centroid vectors of voxels
	boost::shared_ptr<std::vector<Eigen::Matrix3d> > icovariance_;		// Inverse covariance matrixes of voxel
	boost::shared_ptr<std::vector<std::vector<int> > > points_id_;		// Indexes of points belong to each voxel
	boost::shared_ptr<std::vector<int> > points_per_voxel_;				// Number of points belong to each voxel
													// (may differ from size of each vector in points_id_
													// because of changes made during computing covariances
	boost::shared_ptr<std::vector<Eigen::Vector3d> > tmp_centroid_;
	boost::shared_ptr<std::vector<Eigen::Matrix3d> > tmp_cov_;

	int real_max_bx_, real_max_by_, real_max_bz_;
	int real_min_bx_, real_min_by_, real_min_bz_;

	Octree<PointSourceType> octree_;

	static const int MAX_BX_ = 16;
	static const int MAX_BY_ = 16;
	static const int MAX_BZ_ = 8;
};
}

#endif
