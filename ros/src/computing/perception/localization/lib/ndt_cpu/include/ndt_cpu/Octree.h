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

#ifndef OCTREE_H_
#define OCTREE_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <float.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

/* The octree is built on top of a voxel grid to fasten the nearest neighbor search */
namespace cpu {
template <typename PointSourceType>
class Octree {
public:

	Octree();

	/* Input is a vector of boundaries and ptsum of the voxel grid
	 * Those boundaries is needed since the number of actually occupied voxels may be
	 * much smaller than reserved number of voxels */
	void setInput(std::vector<Eigen::Vector3i> occupied_voxels, typename pcl::PointCloud<PointSourceType>::Ptr point_cloud);

	void update(std::vector<Eigen::Vector3i> new_voxels, typename pcl::PointCloud<PointSourceType>::Ptr new_cloud);

	Eigen::Matrix<float, 6, 1> nearestOctreeNode(PointSourceType q);

private:
	typedef struct {
		float lx, ux;
		float ly, uy;
		float lz, uz;
		Eigen::Vector3d centroid;
		int point_num;
	} OctreeNode;

	typedef struct {
		int lower_x, upper_x;
		int lower_y, upper_y;
		int lower_z, upper_z;
	} OctreeLevelBoundaries;

	typedef struct {
		int x, y, z;
	} OctreeLevelDim;

	// Convert from 3d indexes and level of the tree node to the actual index in the array
	int index2id(int idx, int idy, int idz, int level);
	int index2id(int idx, int idy, int idz, OctreeLevelBoundaries bounds, OctreeLevelDim dims);

	// Convert from the index in the array to the 3d indexes of the tree node
	Eigen::Vector3i id2index(int id, int level);
	Eigen::Vector3i id2index(int id, OctreeLevelBoundaries bounds, OctreeLevelDim dims);

	void buildLevel(int level);

	bool isOccupied(int node_id, int level);

	bool isOccupied(std::vector<unsigned int> occupancy, int node_id);

	void setOccupied(int node_id, int level);

	void setOccupied(std::vector<unsigned int> &occupancy, int node_id);

	void updateBoundaries(std::vector<Eigen::Vector3i> new_voxels);

	int roundUp(int input, int factor);
	int roundDown(int input, int factor);

	int div(int input, int divisor);

	void updateOctreeContent(std::vector<Eigen::Vector3i> new_voxels, typename pcl::PointCloud<PointSourceType>::Ptr new_cloud);

	double dist(OctreeNode node, PointSourceType q);

	/* Three functions to search for the nearest neighbor of a point */

	void initRange(PointSourceType q, double &min_range, int &current_nn_voxel);

	void goUp(Eigen::Matrix<int, 4, 1 > tree_node, PointSourceType q, double &min_range, int &current_nn_voxel);

	void goDown(Eigen::Matrix<int, 4, 1> tree_node, PointSourceType q, double &min_range, int &current_nn_voxel);

	boost::shared_ptr<std::vector<std::vector<OctreeNode> > > octree_;
	boost::shared_ptr<std::vector<OctreeLevelBoundaries> > reserved_size_;
	boost::shared_ptr<std::vector<OctreeLevelDim> > dimension_;

	/* Used for checking if an octree node is occupied or not
	 * If an octree node is occupied (containing some points),
	 * then the corresponding bit is set
	 */
	boost::shared_ptr<std::vector<std::vector<unsigned int> > > occupancy_check_;

	int leaf_x_, leaf_y_, leaf_z_;		// Number of voxels contained in each leaf

	static const int MAX_BX_ = 8;
	static const int MAX_BY_ = 8;
	static const int MAX_BZ_ = 4;
};
}

#endif
