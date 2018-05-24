#include "ndt_cpu/Octree.h"
#include "ndt_cpu/debug.h"
#include <math.h>
#include <limits>
#include <inttypes.h>

#include <vector>
#include <cmath>

#include <stdio.h>
#include <sys/time.h>
#include <iostream>

#include <bitset>

namespace cpu {

template <typename PointSourceType>
Octree<PointSourceType>::Octree()
{
	leaf_x_ = 4; //16;
	leaf_y_ = 4; //16;
	leaf_z_ = 2; //4;

	octree_.reset();
	reserved_size_.reset();
	dimension_.reset();
	occupancy_check_.reset();
}

template <typename PointSourceType>
int Octree<PointSourceType>::roundUp(int input, int factor)
{
	return (input < 0) ? (-((-input) / factor) * factor) : ((input  + factor - 1 ) / factor) * factor;
}

template <typename PointSourceType>
int Octree<PointSourceType>::roundDown(int input, int factor)
{
	return (input < 0) ? (-(-input + factor - 1) / factor) * factor : (input / factor) * factor;
}

template <typename PointSourceType>
int Octree<PointSourceType>::div(int input, int divisor)
{
	return (input < 0) ? (-(-input + divisor - 1) / divisor) : (input / divisor);
}

template <typename PointSourceType>
void Octree<PointSourceType>::setInput(std::vector<Eigen::Vector3i> occupied_voxels, typename pcl::PointCloud<PointSourceType>::Ptr point_cloud)
{
	octree_.reset();
	reserved_size_.reset();
	dimension_.reset();
	occupancy_check_.reset();

	octree_ = boost::make_shared<std::vector<std::vector<OctreeNode> > >();
	reserved_size_ = boost::make_shared<std::vector<OctreeLevelBoundaries> >();
	dimension_ = boost::make_shared<std::vector<OctreeLevelDim> >();
	occupancy_check_ = boost::make_shared<std::vector<std::vector<unsigned int> > >();

	// Reserve memory
	int max_b_x, max_b_y, max_b_z;
	int min_b_x, min_b_y, min_b_z;

	max_b_x = max_b_y = max_b_z = INT_MIN;
	min_b_x = min_b_y = min_b_z = INT_MAX;

	for (int i = 0; i < occupied_voxels.size(); i++) {
		Eigen::Vector3i vid = occupied_voxels[i];

		if (min_b_x > vid(0)) {
			min_b_x = vid(0);
		}

		if (max_b_x < vid(0)) {
			max_b_x = vid(0);
		}

		if (min_b_y > vid(1)) {
			min_b_y = vid(1);
		}

		if (max_b_y < vid(1)) {
			max_b_y = vid(1);
		}

		if (min_b_z > vid(2)) {
			min_b_z = vid(2);
		}

		if (max_b_z < vid(2)) {
			max_b_z = vid(2);
		}
	}

	OctreeLevelBoundaries level;

	int node_number;

	OctreeLevelDim level_size;

	level.lower_x = roundDown(div(min_b_x, leaf_x_), MAX_BX_);
	level.lower_y = roundDown(div(min_b_y, leaf_y_), MAX_BY_);
	level.lower_z = roundDown(div(min_b_z, leaf_z_), MAX_BZ_);

	level.upper_x = roundUp(div(max_b_x, leaf_x_), MAX_BX_);
	level.upper_y = roundUp(div(max_b_y, leaf_y_), MAX_BY_);
	level.upper_z = roundUp(div(max_b_z, leaf_z_), MAX_BZ_);

	level_size.x = level.upper_x - level.lower_x + 1;
	level_size.y = level.upper_y - level.lower_y + 1;
	level_size.z = level.upper_z - level.lower_z + 1;

	node_number = level_size.x * level_size.y * level_size.z;

	// construct bottom

	std::vector<OctreeNode> level0(node_number);
	std::vector<unsigned int> occupancy0(static_cast<int>((node_number + sizeof(unsigned int) * 8 - 1) / (sizeof(unsigned int) * 8)), 0);

	(*octree_).push_back(level0);
	(*reserved_size_).push_back(level);
	(*dimension_).push_back(level_size);
	(*occupancy_check_).push_back(occupancy0);

	int tree_level = 0;



	while (node_number > 8) {
		level.lower_x = div(level.lower_x, 2);
		level.lower_y = div(level.lower_y, 2);
		level.lower_z = div(level.lower_z, 2);

		level.upper_x = div(level.upper_x, 2);
		level.upper_y = div(level.upper_y, 2);
		level.upper_z = div(level.upper_z, 2);

		level_size.x = level.upper_x - level.lower_x + 1;
		level_size.y = level.upper_y - level.lower_y + 1;
		level_size.z = level.upper_z - level.lower_z + 1;

		node_number = level_size.x * level_size.y * level_size.z;

		std::vector<OctreeNode> new_level(node_number);
		std::vector<unsigned int> new_occupancy_check(static_cast<int>((node_number + sizeof(unsigned int) * 8 - 1) / (sizeof(unsigned int) * 8)), 0);

		(*octree_).push_back(new_level);
		(*reserved_size_).push_back(level);
		(*dimension_).push_back(level_size);
		(*occupancy_check_).push_back(new_occupancy_check);
	}

	// Setup the lowest level
	for (int i = 0; i < occupied_voxels.size(); i++) {
		Eigen::Vector3i vid = occupied_voxels[i];

		// Indexes of the octree node that contains the current voxel
		int nidx = div(vid(0), leaf_x_);
		int nidy = div(vid(1), leaf_y_);
		int nidz = div(vid(2), leaf_z_);

		int nid = index2id(nidx, nidy, nidz, 0);

		std::vector<OctreeNode> &current_level = (*octree_)[0];
		OctreeLevelBoundaries curb = (*reserved_size_)[0];
		OctreeNode &current_node = current_level[nid];
		PointSourceType p = point_cloud->points[i];

		// Update boundaries inside the node
		if (!isOccupied(nid, 0)) {
			/* If the current octree node is empty,
			 * just set boundaries inside the node
			 */
			current_node.lx = current_node.ux = p.x;
			current_node.ly = current_node.uy = p.y;
			current_node.lz = current_node.uz = p.z;
			current_node.point_num = 1;
			current_node.centroid(0) = p.x;
			current_node.centroid(1) = p.y;
			current_node.centroid(2) = p.z;

			// Update occupancy status
			setOccupied(nid, 0);

		} else {
			/* Otherwise, update boundaries inside the node */
			if (p.x < current_node.lx) {
				current_node.lx = p.x;
			}

			if (p.y < current_node.ly) {
				current_node.ly = p.y;
			}

			if (p.z < current_node.lz) {
				current_node.lz = p.z;
			}

			if (p.x > current_node.ux) {
				current_node.ux = p.x;
			}

			if (p.y > current_node.uy) {
				current_node.uy = p.y;
			}

			if (p.z > current_node.uz) {
				current_node.uz = p.z;
			}

			Eigen::Vector3d new_node(p.x, p.y, p.z);

			current_node.centroid = current_node.centroid * current_node.point_num + new_node;
			current_node.point_num++;
			current_node.centroid /= current_node.point_num;

		}
	}

	// Build higher levels
	OctreeLevelBoundaries child, parent;

	for (int level = 1; level < (*octree_).size(); level++) {
		buildLevel(level);
	}
}

template <typename PointSourceType>
bool Octree<PointSourceType>::isOccupied(int node_id, int level)
{
	std::vector<unsigned int> &current_occ = (*occupancy_check_)[level];

	int val_loc = node_id / (sizeof(int) * 8);
	int bit_loc = node_id % (sizeof(int) * 8);

	return ((current_occ[val_loc] & (1 << bit_loc)) == (1 << bit_loc));
}

template <typename PointSourceType>
bool Octree<PointSourceType>::isOccupied(std::vector<unsigned int> occupancy, int node_id)
{
	int val_loc = node_id / (sizeof(int) * 8);
	int bit_loc = node_id % (sizeof(int) * 8);

	return ((occupancy[val_loc] & (1 << bit_loc)) == (1 << bit_loc));
}

template <typename PointSourceType>
void Octree<PointSourceType>::setOccupied(int node_id, int level)
{
	std::vector<unsigned int> &current_occ = (*occupancy_check_)[level];
	int val_loc = node_id / (sizeof(int) * 8);
	int bit_loc = node_id % (sizeof(int) * 8);

	current_occ[val_loc] |= (1 << bit_loc);
}

template <typename PointSourceType>
void Octree<PointSourceType>::setOccupied(std::vector<unsigned int> &occupancy, int node_id)
{
	int val_loc = node_id / (sizeof(int) * 8);
	int bit_loc = node_id % (sizeof(int) * 8);

	occupancy[val_loc] |= (1 << bit_loc);
}

template <typename PointSourceType>
void Octree<PointSourceType>::buildLevel(int level)
{
	// Parent list
	std::vector<OctreeNode> &parent = (*octree_)[level];
	std::vector<unsigned int> &poccupancy = (*occupancy_check_)[level];

	// Child list
	std::vector<OctreeNode> &child = (*octree_)[level - 1];
	std::vector<unsigned int> &coccupancy = (*occupancy_check_)[level - 1];
	OctreeLevelBoundaries cbounds = (*reserved_size_)[level - 1];

	for (int i = cbounds.lower_x; i <= cbounds.upper_x; i++) {
		for (int j = cbounds.lower_y; j <= cbounds.upper_y; j++) {
			for (int k = cbounds.lower_z; k <= cbounds.upper_z; k++) {
				int cid = index2id(i, j, k, level - 1);		// Child id
				int pidx = div(i, 2);
				int pidy = div(j, 2);
				int pidz = div(k, 2);
				int pid = index2id(pidx, pidy, pidz, level);	// Parent id
				OctreeNode &cnode = child[cid];
				OctreeNode &pnode = parent[pid];

				if (isOccupied(cid, level - 1)) {
					if (isOccupied(pid, level)) {
						if (pnode.lx > cnode.lx) {
							pnode.lx = cnode.lx;
						}

						if (pnode.ly > cnode.ly) {
							pnode.ly = cnode.ly;
						}

						if (pnode.lz > cnode.lz) {
							pnode.lz = cnode.lz;
						}

						if (pnode.ux < cnode.ux) {
							pnode.ux = cnode.ux;
						}

						if (pnode.uy < cnode.uy) {
							pnode.uy = cnode.uy;
						}

						if (pnode.uz < cnode.uz) {
							pnode.uz = cnode.uz;
						}

						// If the parent node is already occupied, update its centroid
						pnode.centroid = pnode.centroid * pnode.point_num + cnode.centroid * cnode.point_num;
						pnode.point_num += cnode.point_num;
						pnode.centroid /= pnode.point_num;
					} else {
						// Otherwise, set cnode to pnode and mark the pnode as occupied
						pnode = cnode;

						setOccupied(pid, level);
					}
				}
			}
		}
	}
}

template <typename PointSourceType>
void Octree<PointSourceType>::update(std::vector<Eigen::Vector3i> new_voxels, typename pcl::PointCloud<PointSourceType>::Ptr new_cloud)
{
	// First, update boundaries
	updateBoundaries(new_voxels);

	updateOctreeContent(new_voxels, new_cloud);
}

template <typename PointSourceType>
void Octree<PointSourceType>::updateBoundaries(std::vector<Eigen::Vector3i> new_voxels)
{
	int new_min_bx, new_min_by, new_min_bz;
	int new_max_bx, new_max_by, new_max_bz;

	new_min_bx = new_min_by = new_min_bz = INT_MAX;
	new_max_bx = new_max_by = new_max_bz = INT_MIN;

	for (int i = 0; i < new_voxels.size(); i++) {
		Eigen::Vector3i vid = new_voxels[i];

		if (new_min_bx > vid(0)) {
			new_min_bx = vid(0);
		}

		if (new_max_bx < vid(0)) {
			new_max_bx = vid(0);
		}

		if (new_min_by > vid(1)) {
			new_min_by = vid(1);
		}

		if (new_max_by < vid(1)) {
			new_max_by = vid(1);
		}

		if (new_min_bz > vid(2)) {
			new_min_bz = vid(2);
		}

		if (new_max_bz < vid(2)) {
			new_max_bz = vid(2);
		}
	}

	OctreeLevelBoundaries dst_bounds;

	dst_bounds.lower_x = roundDown(div(new_min_bx, leaf_x_), MAX_BX_);
	dst_bounds.lower_y = roundDown(div(new_min_by, leaf_y_), MAX_BY_);
	dst_bounds.lower_z = roundDown(div(new_min_bz, leaf_z_), MAX_BZ_);

	dst_bounds.upper_x = roundUp(div(new_max_bx, leaf_x_), MAX_BX_);
	dst_bounds.upper_y = roundUp(div(new_max_by, leaf_y_), MAX_BY_);
	dst_bounds.upper_z = roundUp(div(new_max_bz, leaf_z_), MAX_BZ_);

	/* Compare dst_bounds with boundaries of the lowest level
	 * of the octree to check if we need to reallocate the octree
	 */
	OctreeLevelBoundaries src_bounds = (*reserved_size_)[0];

	if (dst_bounds.lower_x < src_bounds.lower_x || dst_bounds.lower_y < src_bounds.lower_y || dst_bounds.lower_z < src_bounds.lower_z ||
			dst_bounds.upper_x > src_bounds.upper_x || dst_bounds.upper_y > src_bounds.upper_y || dst_bounds.upper_z > src_bounds.upper_z) {
		// If the base voxel grid expanded, then we need expand the octree as well
		if (dst_bounds.lower_x > src_bounds.lower_x) {
			dst_bounds.lower_x = src_bounds.lower_x;
		}

		if (dst_bounds.lower_y > src_bounds.lower_y) {
			dst_bounds.lower_y = src_bounds.lower_y;
		}

		if (dst_bounds.lower_z > src_bounds.lower_z) {
			dst_bounds.lower_z = src_bounds.lower_z;
		}

		if (dst_bounds.upper_x < src_bounds.upper_x) {
			dst_bounds.upper_x = src_bounds.upper_x;
		}

		if (dst_bounds.upper_y < src_bounds.upper_y) {
			dst_bounds.upper_y = src_bounds.upper_y;
		}

		if (dst_bounds.upper_z < src_bounds.upper_z) {
			dst_bounds.upper_z = src_bounds.upper_z;
		}

		OctreeLevelDim dst_dim;

		dst_dim.x = dst_bounds.upper_x - dst_bounds.lower_x + 1;
		dst_dim.y = dst_bounds.upper_y - dst_bounds.lower_y + 1;
		dst_dim.z = dst_bounds.upper_z - dst_bounds.lower_z + 1;
		int node_number = dst_dim.x * dst_dim.y * dst_dim.z;

		// Backup old octree
		boost::shared_ptr<std::vector<std::vector<OctreeNode> > > old_octree = octree_;
		boost::shared_ptr<std::vector<std::vector<unsigned int> > > old_occupancy_check = occupancy_check_;
		boost::shared_ptr<std::vector<OctreeLevelBoundaries> > old_reserved_size = reserved_size_;
		boost::shared_ptr<std::vector<OctreeLevelDim> > old_dimension = dimension_;

		// Reserve space for the new octree
		octree_ = boost::make_shared<std::vector<std::vector<OctreeNode> > >();
		occupancy_check_ = boost::make_shared<std::vector<std::vector<unsigned int> > >();
		reserved_size_ = boost::make_shared<std::vector<OctreeLevelBoundaries> >();
		dimension_ = boost::make_shared<std::vector<OctreeLevelDim> >();

		//Setup level0

		std::vector<OctreeNode> new_level0(node_number);
		std::vector<unsigned int> new_occupy0((node_number + sizeof(unsigned int) * 8 - 1) / (sizeof(unsigned int) * 8), 0);

		(*octree_).push_back(new_level0);
		(*occupancy_check_).push_back(new_occupy0);
		(*reserved_size_).push_back(dst_bounds);
		(*dimension_).push_back(dst_dim);

		while (node_number > 8) {
			dst_bounds.lower_x = div(dst_bounds.lower_x, 2);
			dst_bounds.lower_y = div(dst_bounds.lower_y, 2);
			dst_bounds.lower_z = div(dst_bounds.lower_z, 2);
			dst_bounds.upper_x = div(dst_bounds.upper_x, 2);
			dst_bounds.upper_y = div(dst_bounds.upper_y, 2);
			dst_bounds.upper_z = div(dst_bounds.upper_z, 2);

			dst_dim.x = dst_bounds.upper_x - dst_bounds.lower_x + 1;
			dst_dim.y = dst_bounds.upper_y - dst_bounds.lower_y + 1;
			dst_dim.z = dst_bounds.upper_z - dst_bounds.lower_z + 1;

			node_number = dst_dim.x * dst_dim.y * dst_dim.z;

			std::vector<OctreeNode> new_level(node_number);
			std::vector<unsigned int> new_occupancy(static_cast<int>((node_number + sizeof(unsigned int) * 8 - 1) / (sizeof(unsigned int) * 8)), 0);

			(*octree_).push_back(new_level);
			(*occupancy_check_).push_back(new_occupancy);
			(*reserved_size_).push_back(dst_bounds);
			(*dimension_).push_back(dst_dim);
		}


		// Copy the old octree to the new one
		int level = 0;

		for (; level < (*old_octree).size(); level++) {
			std::vector<unsigned int> &dst_occupancy = (*occupancy_check_)[level];
			std::vector<OctreeNode> &dst_level = (*octree_)[level];
			OctreeLevelBoundaries dst_bounds = (*reserved_size_)[level];
			OctreeLevelDim dst_dim = (*dimension_)[level];


			std::vector<unsigned int> &src_occupancy = (*old_occupancy_check)[level];
			std::vector<OctreeNode> &src_level = (*old_octree)[level];
			OctreeLevelBoundaries src_bounds = (*old_reserved_size)[level];
			OctreeLevelDim src_dim = (*old_dimension)[level];

			for (int i = 0; i < src_occupancy.size(); i++) {
				unsigned int stat32 = src_occupancy[i];

				if (stat32 != 0) {
					for (int j = 0; j < sizeof(unsigned int) * 8; j++) {
						if ((stat32 & (1 << j)) == (1 << j)) {
							int src_id = j + i * sizeof(unsigned int) * 8;
							Eigen::Vector3i vid = id2index(src_id, src_bounds, src_dim);
							int dst_id = index2id(vid(0), vid(1), vid(2), dst_bounds, dst_dim);

							dst_level[dst_id] = src_level[src_id];
							setOccupied(dst_occupancy, dst_id);
						}
					}
				}
			}
		}

		// Build upper levels
		for (; level < (*octree_).size(); level++) {
			buildLevel(level);
		}
	}

}

template <typename PointSourceType>
int Octree<PointSourceType>::index2id(int idx, int idy, int idz, int level)
{
	OctreeLevelBoundaries bound = (*reserved_size_)[level];
	OctreeLevelDim dim = (*dimension_)[level];

	return ((idx - bound.lower_x) + (idy - bound.lower_y) * dim.x + (idz - bound.lower_z) * dim.x * dim.y);
}


template <typename PointSourceType>
int Octree<PointSourceType>::index2id(int idx, int idy, int idz, OctreeLevelBoundaries bounds, OctreeLevelDim dim)
{
	return ((idx - bounds.lower_x) + (idy - bounds.lower_y) * dim.x + (idz - bounds.lower_z) * dim.x * dim.y);
}

template <typename PointSourceType>
Eigen::Vector3i Octree<PointSourceType>::id2index(int id, int level)
{
	Eigen::Vector3i output;

	OctreeLevelBoundaries bound = (*reserved_size_)[level];
	OctreeLevelDim dim = (*dimension_)[level];

	output(2) = id / (dim.x * dim.y);
	id -= output(2) * dim.x * dim.y;
	output(1) = id / dim.x;
	output(0) = id - output(1) * dim.x;


	output(0) += bound.lower_x;
	output(1) += bound.lower_y;
	output(2) += bound.lower_z;

	return output;
}

template <typename PointSourceType>
Eigen::Vector3i Octree<PointSourceType>::id2index(int id, OctreeLevelBoundaries bounds, OctreeLevelDim dims)
{
	Eigen::Vector3i output;

	output(2) = id / (dims.x * dims.y);
	id -= output(2) * dims.x * dims.y;
	output(1) = id / dims.x;
	output(0) = id - output(1) * dims.x;

	output(0) += bounds.lower_x;
	output(1) += bounds.lower_y;
	output(2) += bounds.lower_z;

	return output;
}

template <typename PointSourceType>
void Octree<PointSourceType>::updateOctreeContent(std::vector<Eigen::Vector3i> new_voxels, typename pcl::PointCloud<PointSourceType>::Ptr new_cloud)
{
	for (int i = 0; i < new_voxels.size(); i++) {
		Eigen::Vector3i vid = new_voxels[i];
		int node_idx = div(vid(0), leaf_x_);
		int node_idy = div(vid(1), leaf_y_);
		int node_idz = div(vid(2), leaf_z_);
		PointSourceType p = new_cloud->points[i];
		Eigen::Vector3d point(p.x, p.y, p.z);

		// Go from bottom to top to update tree nodes
		for (int level = 0; level < (*octree_).size(); level++) {
			int nid = index2id(node_idx, node_idy, node_idz, level);
			std::vector<OctreeNode> &cur_level = (*octree_)[level];
			OctreeNode &node = cur_level[nid];

			if (isOccupied(nid, level)) {
				// If the node is occupied, update the current content
				if (p.x < node.lx) {
					node.lx = p.x;
				}

				if (p.y < node.ly) {
					node.ly = p.y;
				}

				if (p.z < node.lz) {
					node.lz = p.z;
				}

				if (p.x > node.ux) {
					node.ux = p.x;
				}

				if (p.y > node.uy) {
					node.uy = p.y;
				}

				if (p.z > node.uz) {
					node.uz = p.z;
				}


				node.centroid = node.centroid * node.point_num + point;
				node.point_num++;
				node.centroid /= node.point_num;

			} else {
				node.lx = node.ux = p.x;
				node.ly = node.uy = p.y;
				node.lz = node.uz = p.z;

				node.centroid = point;
				node.point_num = 1;

				setOccupied(nid, level);
			}

			node_idx = div(node_idx, 2);
			node_idy = div(node_idy, 2);
			node_idz = div(node_idz, 2);
		}
	}
}


template <typename PointSourceType>
Eigen::Matrix<float, 6, 1> Octree<PointSourceType>::nearestOctreeNode(PointSourceType q)
{
	double min_range = DBL_MAX;
	int current_nn_vid = -1;

	initRange(q, min_range, current_nn_vid);

	Eigen::Vector3i cur_vid = id2index(current_nn_vid, 0);
	Eigen::Matrix<int, 4, 1> cur_nn_node(cur_vid(0), cur_vid(1), cur_vid(2), 0);

	goUp(cur_nn_node, q, min_range, current_nn_vid);

	std::vector<OctreeNode> &bottom = (*octree_)[0];

	OctreeNode out_node = bottom[current_nn_vid];

	Eigen::Matrix<float, 6, 1> output;

	output(0) = out_node.lx;
	output(1) = out_node.ly;
	output(2) = out_node.lz;

	output(3) = out_node.ux;
	output(4) = out_node.uy;
	output(5) = out_node.uz;



	return output;
}

template <typename PointSourceType>
double Octree<PointSourceType>::dist(OctreeNode node, PointSourceType p)
{
	double dx, dy, dz;


	if (p.x < node.lx) {
		dx = node.lx - p.x;
	} else if (p.x > node.ux) {
		dx = p.x - node.ux;
	} else {
		dx = 0;
	}

	if (p.y < node.ly) {
		dy = node.ly - p.y;
	} else if (p.y > node.uy) {
		dy = p.y - node.uy;
	} else {
		dy = 0;
	}

	if (p.z < node.lz) {
		dz = node.lz - p.z;
	} else if (p.z > node.uz) {
		dz = p.z - node.uz;
	} else {
		dz = 0;
	}

	return sqrt(dx * dx + dy * dy + dz * dz);
}

template <typename PointSourceType>
void Octree<PointSourceType>::initRange(PointSourceType q, double &min_range, int &nn_id)
{
	// Init min_range at maximum double value
	min_range = DBL_MAX;

	// Check top
	int top_id;
	int top_lv = (*octree_).size() - 1;
	std::vector<OctreeNode> &top = (*octree_)[top_lv];

	for (int i = 0; i < top.size(); i++) {
		if (isOccupied(i, top_lv)) {
			OctreeNode node = top[i];

			double cur_dist = dist(node, q);

			if (cur_dist < min_range) {
				min_range = cur_dist;
				top_id = i;
			}
		}
	}


	Eigen::Vector3i top_indexes = id2index(top_id, top_lv);
	int nidx = top_indexes(0);
	int nidy = top_indexes(1);
	int nidz = top_indexes(2);
	int lower_x, upper_x, lower_y, upper_y, lower_z, upper_z;

	// Go down to leaf
	for (int level = (*octree_).size() - 2; level >= 1; level--) {
		OctreeLevelBoundaries bounds = (*reserved_size_)[level];
		std::vector<OctreeNode> &cur_level = (*octree_)[level];

		lower_x = (nidx * 2 < bounds.lower_x) ? bounds.lower_x : nidx * 2;
		upper_x = (nidx * 2 + 1 > bounds.upper_x) ? bounds.upper_x : nidx * 2 + 1;

		lower_y = (nidy * 2 < bounds.lower_y) ? bounds.lower_y : nidy * 2;
		upper_y = (nidy * 2 + 1 > bounds.upper_y) ? bounds.upper_y : nidy * 2 + 1;

		lower_z = (nidz * 2 < bounds.lower_z) ? bounds.lower_z : nidz * 2;
		upper_z = (nidz * 2 + 1 > bounds.upper_z) ? bounds.upper_z : nidz * 2 + 1;

		min_range = DBL_MAX;

		for (int i = lower_x; i <= upper_x; i++) {
			for (int j = lower_y; j <= upper_y; j++) {
				for (int k = lower_z; k <= upper_z; k++) {
					int node_id = index2id(i, j, k, level);

					if (isOccupied(node_id, level)) {
						OctreeNode node = cur_level[node_id];
						double cur_dist = dist(node, q);

						if (cur_dist < min_range) {
							min_range = cur_dist;
							nidx = i;
							nidy = j;
							nidz = k;
						}
					}
				}
			}
		}
	}

	// Inspect leaves
	OctreeLevelBoundaries bounds = (*reserved_size_)[0];
	std::vector<OctreeNode> &bottom = (*octree_)[0];

	lower_x = (nidx * 2 < bounds.lower_x) ? bounds.lower_x : nidx * 2;
	upper_x = (nidx * 2 + 1 > bounds.upper_x) ? bounds.upper_x : nidx * 2 + 1;

	lower_y = (nidy * 2 < bounds.lower_y) ? bounds.lower_y : nidy * 2;
	upper_y = (nidy * 2 + 1 > bounds.upper_y) ? bounds.upper_y : nidy * 2 + 1;

	lower_z = (nidz * 2 < bounds.lower_z) ? bounds.lower_z : nidz * 2;
	upper_z = (nidz * 2 + 1 > bounds.upper_z) ? bounds.upper_z : nidz * 2 + 1;

	min_range = DBL_MAX;

	for (int i = lower_x; i <= upper_x; i++) {
		for (int j = lower_y; j <= upper_y; j++) {
			for (int k = lower_z; k <= upper_z; k++) {
				int node_id = index2id(i, j, k, 0);

				if (isOccupied(node_id, 0)) {
					OctreeNode node = bottom[node_id];
					Eigen::Vector3d c = node.centroid;
					double cur_dist = sqrt((c(0) - q.x) * (c(0) - q.x) + (c(1) - q.y) * (c(1) - q.y) + (c(2) - q.z) * (c(2) - q.z));

					if (cur_dist < min_range) {
						min_range = cur_dist;
						nidx = i;
						nidy = j;
						nidz = k;
					}
				}
			}
		}
	}

	nn_id = index2id(nidx, nidy, nidz, 0);
}

// Check all childrens
template <typename PointSourceType>
void Octree<PointSourceType>::goDown(Eigen::Matrix<int, 4, 1> node, PointSourceType q, double &min_range, int &current_nn_voxel)
{
	int id = index2id(node(0), node(1), node(2), node(3));

	if (!isOccupied(id, node(3))) {
		return;
	}

	std::vector<OctreeNode> &cur_level = (*octree_)[node(3)];
	OctreeNode cur_node = cur_level[id];

	if (node(3) == 0) {
		Eigen::Vector3d c = cur_node.centroid;
		double cur_dist = sqrt((c(0) - q.x) * (c(0) - q.x) + (c(1) - q.y) * (c(1) - q.y) + (c(2) - q.z) * (c(2) - q.z));

		if (cur_dist < min_range) {
			min_range = cur_dist;
			current_nn_voxel = id;
		}
	}

	double cur_dist = dist(cur_node, q);

	if (cur_dist > min_range) {
		return;
	}

	// Check children
	int level = node(3) - 1;
	OctreeLevelBoundaries bounds = (*reserved_size_)[level];

	int lower_x = (node(0) * 2 < bounds.lower_x) ? bounds.lower_x : node(0) * 2;
	int upper_x = (node(0) * 2 + 1 > bounds.upper_x) ? bounds.upper_x : node(0) * 2 + 1;

	int lower_y = (node(1) * 2 < bounds.lower_y) ? bounds.lower_y : node(1) * 2;
	int upper_y = (node(1) * 2 + 1 > bounds.upper_y) ? bounds.upper_y : node(1) * 2 + 1;

	int lower_z = (node(2) * 2 < bounds.lower_z) ? bounds.lower_z : node(2) * 2;
	int upper_z = (node(2) * 2 + 1 > bounds.upper_z) ? bounds.upper_z : node(2) * 2 + 1;

	for (int i = lower_x; i <= upper_x; i++) {
		for (int j = lower_y; j <= upper_y; j++) {
			for (int k = lower_z; k <= upper_z; k++) {
				Eigen::Matrix<int, 4, 1> child(i, j, k, level);

				goDown(child, q, min_range, current_nn_voxel);
			}
		}
	}
}

// Check siblings then go up to parent
template <typename PointSourceType>
void Octree<PointSourceType>::goUp(Eigen::Matrix<int, 4, 1> node, PointSourceType q, double &min_range, int &current_nn_voxel)
{
	if (node(3) == (*octree_).size() - 1) {
		// If reaching top
		int top_lv = (*octree_).size() - 1;
		OctreeLevelBoundaries top_bound = (*reserved_size_)[top_lv];

		for (int i = top_bound.lower_x; i <= top_bound.upper_x; i++) {
			for (int j = top_bound.lower_y; j <= top_bound.upper_y; j++) {
				for (int k = top_bound.lower_z; k <= top_bound.upper_z; k++) {
					Eigen::Matrix<int, 4, 1> sib(i, j, k, top_lv);

					if (sib != node) {
						goDown(sib, q, min_range, current_nn_voxel);
					}
				}
			}
		}

		return;
	}

	int px = div(node(0), 2);
	int py = div(node(1), 2);
	int pz = div(node(2), 2);
	int level = node(3);
	OctreeLevelBoundaries bounds = (*reserved_size_)[level];
	int lower_x = (px * 2 < bounds.lower_x) ? bounds.lower_x : px * 2;
	int upper_x = (px * 2 + 1 > bounds.upper_x) ? bounds.upper_x : px * 2 + 1;
	int lower_y = (py * 2 < bounds.lower_y) ? bounds.lower_y : py * 2;
	int upper_y = (py * 2 + 1 > bounds.upper_y) ? bounds.upper_y : py * 2 + 1;
	int lower_z = (pz * 2 < bounds.lower_z) ? bounds.lower_z : pz * 2;
	int upper_z = (pz * 2 + 1 > bounds.upper_z) ? bounds.upper_z : pz * 2 + 1;

	for (int i = lower_x; i <= upper_x; i++) {
		for (int j = lower_y; j <= upper_y; j++) {
			for (int k = lower_z; k <= upper_z; k++) {
				Eigen::Matrix<int, 4, 1> sib(i, j, k, level);

				if (sib != node) {
					goDown(sib, q, min_range, current_nn_voxel);
				}
			}
		}
	}


	Eigen::Matrix<int, 4, 1> parent(px, py, pz, level + 1);

	goUp(parent, q, min_range, current_nn_voxel);
}

template class Octree<pcl::PointXYZ>;
template class Octree<pcl::PointXYZI>;

}

