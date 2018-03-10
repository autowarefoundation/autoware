#include "fast_pcl/ndt_cpu/VoxelGrid.h"
#include "fast_pcl/ndt_cpu/debug.h"
#include <math.h>
#include <limits>
#include <inttypes.h>

#include <vector>
#include <cmath>

#include <stdio.h>
#include <sys/time.h>

#include "fast_pcl/ndt_cpu/SymmetricEigenSolver.h"

namespace cpu {
template <typename PointSourceType>
VoxelGrid<PointSourceType>::VoxelGrid():
	voxel_num_(0),
	max_x_(FLT_MIN),
	max_y_(FLT_MIN),
	max_z_(FLT_MIN),
	min_x_(FLT_MAX),
	min_y_(FLT_MAX),
	min_z_(FLT_MAX),
	voxel_x_(0),
	voxel_y_(0),
	voxel_z_(0),
	max_b_x_(0),
	max_b_y_(0),
	max_b_z_(0),
	min_b_x_(0),
	min_b_y_(0),
	min_b_z_(0),
	vgrid_x_(0),
	vgrid_y_(0),
	vgrid_z_(0),
	min_points_per_voxel_(6)
{
	centroid_.clear();
	covariance_.clear();
	icovariance_.clear();
	points_id_.clear();
	points_per_voxel_.clear();
	octree_centroids_.clear();
	points_per_node_.clear();
};

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::initialize()
{
	centroid_.resize(voxel_num_);

	covariance_.resize(voxel_num_);

	icovariance_.resize(voxel_num_);

	points_id_.resize(voxel_num_);

	points_per_voxel_.resize(voxel_num_);

}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getVoxelNum() const
{
	return voxel_num_;
}

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::getMaxX() const
{
	return max_x_;
}

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::getMaxY() const
{
	return max_y_;
}

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::getMaxZ() const
{
	return max_z_;
}

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::getMinX() const
{
	return min_x_;
}

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::getMinY() const
{
	return min_y_;
}

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::getMinZ() const
{
	return min_z_;
}

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::getVoxelX() const
{
	return voxel_x_;
}

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::getVoxelY() const
{
	return voxel_y_;
}

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::getVoxelZ() const
{
	return voxel_z_;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getMaxBX() const
{
	return max_b_x_;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getMaxBY() const
{
	return max_b_y_;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getMaxBZ() const
{
	return max_b_z_;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getMinBX() const
{
	return min_b_x_;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getMinBY() const
{
	return min_b_y_;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getMinBZ() const
{
	return min_b_z_;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getVgridX() const
{
	return vgrid_x_;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getVgridY() const
{
	return vgrid_y_;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::getVgridZ() const
{
	return vgrid_z_;
}

template <typename PointSourceType>
Eigen::Vector3d VoxelGrid<PointSourceType>::getCentroid(int voxel_id) const
{
	return centroid_[voxel_id];
}

template <typename PointSourceType>
Eigen::Matrix3d VoxelGrid<PointSourceType>::getCovariance(int voxel_id) const
{
	return covariance_[voxel_id];
}

template <typename PointSourceType>
Eigen::Matrix3d VoxelGrid<PointSourceType>::getInverseCovariance(int voxel_id) const
{
	return icovariance_[voxel_id];
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::setLeafSize(float voxel_x, float voxel_y, float voxel_z)
{
	voxel_x_ = voxel_x;
	voxel_y_ = voxel_y;
	voxel_z_ = voxel_z;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::voxelId(PointSourceType p)
{
	int idx = static_cast<int>(floor(p.x / voxel_x_) - static_cast<float>(min_b_x_));
	int idy = static_cast<int>(floor(p.y / voxel_y_) - static_cast<float>(min_b_y_));
	int idz = static_cast<int>(floor(p.z / voxel_z_) - static_cast<float>(min_b_z_));

	return (idx + idy * vgrid_x_ + idz * vgrid_x_ * vgrid_y_);
}


template <typename PointSourceType>
void VoxelGrid<PointSourceType>::computeCentroidAndCovariance()
{
	for (int i = 0; i < voxel_num_; i++) {
		int ipoint_num = points_id_[i].size();
		double point_num = static_cast<double>(ipoint_num);
		Eigen::Vector3d pt_sum = centroid_[i];

		if (ipoint_num > 0) {
			centroid_[i] /= point_num;
		}

		if (ipoint_num >= min_points_per_voxel_) {

			covariance_[i] = (covariance_[i] - 2 * (pt_sum * centroid_[i].transpose())) / point_num + centroid_[i] * centroid_[i].transpose();
			covariance_[i] *= (point_num - 1.0) / point_num;

			SymmetricEigensolver3x3 sv(covariance_[i]);

			sv.compute();
			Eigen::Matrix3d evecs = sv.eigenvectors();
			Eigen::Matrix3d evals = sv.eigenvalues().asDiagonal();

			if (evals(0, 0) < 0 || evals(1, 1) < 0 || evals(2, 2) <= 0) {
				points_per_voxel_[i] = -1;
				continue;
			}

			double min_cov_eigvalue = evals(2, 2) * 0.01;

			if (evals(0, 0) < min_cov_eigvalue) {
				evals(0, 0) = min_cov_eigvalue;

				if (evals(1, 1) < min_cov_eigvalue) {
					evals(1, 1) = min_cov_eigvalue;
				}

				covariance_[i] = evecs * evals * evecs.inverse();
			}

			icovariance_[i] = covariance_[i].inverse();
		}
	}
}

//Input are supposed to be in device memory
template <typename PointSourceType>
void VoxelGrid<PointSourceType>::setInput(typename pcl::PointCloud<PointSourceType>::Ptr input_cloud)
{
	if (input_cloud->points.size() > 0) {
		source_cloud_ = input_cloud;

		findBoundaries();

		initialize();

		scatterPointsToVoxelGrid();

		computeCentroidAndCovariance();

		buildOctree();
	}
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::findBoundaries()
{

	for (int i = 0; i < source_cloud_->points.size(); i++) {
		float x = source_cloud_->points[i].x;
		float y = source_cloud_->points[i].y;
		float z = source_cloud_->points[i].z;

		if (i == 0) {
			max_x_ = min_x_ = x;
			max_y_ = min_y_ = y;
			max_z_ = min_z_ = z;
		} else {

			max_x_ = (max_x_ > x) ? max_x_ : x;
			max_y_ = (max_y_ > y) ? max_y_ : y;
			max_z_ = (max_z_ > z) ? max_z_ : z;

			min_x_ = (min_x_ < x) ? min_x_ : x;
			min_y_ = (min_y_ < y) ? min_y_ : y;
			min_z_ = (min_z_ < z) ? min_z_ : z;
		}
	}

	max_b_x_ = static_cast<int> (floor(max_x_ / voxel_x_));
	max_b_y_ = static_cast<int> (floor(max_y_ / voxel_y_));
	max_b_z_ = static_cast<int> (floor(max_z_ / voxel_z_));

	min_b_x_ = static_cast<int> (floor(min_x_ / voxel_x_));
	min_b_y_ = static_cast<int> (floor(min_y_ / voxel_y_));
	min_b_z_ = static_cast<int> (floor(min_z_ / voxel_z_));

	vgrid_x_ = max_b_x_ - min_b_x_ + 1;
	vgrid_y_ = max_b_y_ - min_b_y_ + 1;
	vgrid_z_ = max_b_z_ - min_b_z_ + 1;

	voxel_num_ = vgrid_x_ * vgrid_y_ * vgrid_z_;
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::radiusSearch(PointSourceType p, float radius, std::vector<int> &voxel_ids, int max_nn)
{
	float t_x = p.x;
	float t_y = p.y;
	float t_z = p.z;

	int max_id_x = static_cast<int>(floor((t_x + radius) / voxel_x_));
	int max_id_y = static_cast<int>(floor((t_y + radius) / voxel_y_));
	int max_id_z = static_cast<int>(floor((t_z + radius) / voxel_z_));

	int min_id_x = static_cast<int>(floor((t_x - radius) / voxel_x_));
	int min_id_y = static_cast<int>(floor((t_y - radius) / voxel_y_));
	int min_id_z = static_cast<int>(floor((t_z - radius) / voxel_z_));

	/* Find intersection of the cube containing
	 * the NN sphere of the point and the voxel grid
	 */
	max_id_x = (max_id_x > max_b_x_) ? max_b_x_ - min_b_x_ : max_id_x - min_b_x_;
	max_id_y = (max_id_y > max_b_y_) ? max_b_y_ - min_b_y_ : max_id_y - min_b_y_;
	max_id_z = (max_id_z > max_b_z_) ? max_b_z_ - min_b_z_ : max_id_z - min_b_z_;

	min_id_x = (min_id_x < min_b_x_) ? 0 : min_id_x - min_b_x_;
	min_id_y = (min_id_y < min_b_y_) ? 0 : min_id_y - min_b_y_;
	min_id_z = (min_id_z < min_b_z_) ? 0 : min_id_z - min_b_z_;

	int nn = 0;

	for (int idx = min_id_x; idx <= max_id_x && nn < max_nn; idx++) {
		for (int idy = min_id_y; idy <= max_id_y && nn < max_nn; idy++) {
			for (int idz = min_id_z; idz <= max_id_z && nn < max_nn; idz++) {
				int vid = idx + idy * vgrid_x_ + idz * vgrid_x_ * vgrid_y_;

				if (points_per_voxel_[vid] >= min_points_per_voxel_) {
					double cx = centroid_[vid](0) - static_cast<double>(t_x);
					double cy = centroid_[vid](1) - static_cast<double>(t_y);
					double cz = centroid_[vid](2) - static_cast<double>(t_z);

					double distance = sqrt(cx * cx + cy * cy + cz * cz);

					if (distance < radius) {
						nn++;
						voxel_ids.push_back(vid);
					}
				}
			}
		}
	}
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::scatterPointsToVoxelGrid()
{

	for (int pid = 0; pid < source_cloud_->points.size(); pid++) {
		int vid = voxelId(source_cloud_->points[pid]);
		PointSourceType p = source_cloud_->points[pid];

		Eigen::Vector3d p3d(p.x, p.y, p.z);

		if (points_id_[vid].size() == 0) {
			centroid_[vid].setZero();
			covariance_[vid].setIdentity();
			icovariance_[vid].setZero();
			points_per_voxel_[vid] = 0;
		}

		centroid_[vid] += p3d;
		covariance_[vid] += p3d * p3d.transpose();
		points_id_[vid].push_back(pid);
		points_per_voxel_[vid]++;
	}
}

/* Build parent nodes from child nodes of the octree */
template <typename PointSourceType>
void VoxelGrid<PointSourceType>::buildParent(std::vector<Eigen::Vector3d> &child_centroids, std::vector<int> &points_per_child, OctreeGridSize child_size,
												std::vector<Eigen::Vector3d> &parent_centroids, std::vector<int> &points_per_parent, OctreeGridSize parent_size)
{

	for (int px = 0; px < parent_size.size_x; px++) {
		for (int py = 0; py < parent_size.size_y; py++) {
			for (int pz = 0; pz < parent_size.size_z; pz++) {
				int pid = px + py * parent_size.size_x + pz * parent_size.size_x * parent_size.size_y;
				int ppoints_num = 0;

				parent_centroids[pid].setZero();

				for (int cx = px * 2; cx < px * 2 + 2 && cx < child_size.size_x; cx++) {
					for (int cy = py * 2; cy < py * 2 + 2 && cy < child_size.size_y; cy++) {
						for (int cz = pz * 2; cz < pz * 2 + 2 && cz < child_size.size_z; cz++) {
							int cid = cx + cy * child_size.size_x + cz * child_size.size_x * child_size.size_y;
							int cpoints_num = points_per_child[cid];

							if (cpoints_num > 0) {
								//std::cout << "Child centroid = " << child_centroids[cid] << std::endl;
								parent_centroids[pid] += static_cast<double>(cpoints_num) * child_centroids[cid];
								ppoints_num += cpoints_num;
							}
						}
					}
				}

				if (ppoints_num > 0) {
					parent_centroids[pid] /= static_cast<double>(ppoints_num);
				}

				points_per_parent[pid] = ppoints_num;
			}
		}
	}
}


template <typename PointSourceType>
void VoxelGrid<PointSourceType>::buildOctree()
{
	octree_centroids_.clear();
	points_per_node_.clear();
	octree_size_of_level_.clear();

	//Push leafs to the bottom of the tree
	octree_centroids_.push_back(centroid_);

	std::vector<int> points_per_octree_node(points_id_.size());

	for (int i = 0; i < points_id_.size(); i++) {
		points_per_octree_node[i] = points_id_[i].size();
	}

	points_per_node_.push_back(points_per_octree_node);
	OctreeGridSize grid_size;

	grid_size.size_x = vgrid_x_;
	grid_size.size_y = vgrid_y_;
	grid_size.size_z = vgrid_z_;

	octree_size_of_level_.push_back(grid_size);

	int node_number = voxel_num_;
	OctreeGridSize child_size, parent_size;

	int i = 0;

	while (node_number > 8) {
		child_size = octree_size_of_level_[i];
		parent_size.size_x = (child_size.size_x - 1) / 2 + 1;
		parent_size.size_y = (child_size.size_y - 1) / 2 + 1;
		parent_size.size_z = (child_size.size_z - 1) / 2 + 1;

		node_number = parent_size.size_x * parent_size.size_y * parent_size.size_z;

		std::vector<Eigen::Vector3d> parent_centroids(node_number);
		std::vector<int> points_per_parent(node_number);

		std::vector<Eigen::Vector3d> &child_centroids = octree_centroids_[i];
		std::vector<int> &points_per_child = points_per_node_[i];

		buildParent(child_centroids, points_per_child, child_size, parent_centroids, points_per_parent, parent_size);

		octree_centroids_.push_back(parent_centroids);
		points_per_node_.push_back(points_per_parent);

		octree_size_of_level_.push_back(parent_size);

		i++;
	}
}

/* Search for the nearest octree node */
template <typename PointSourceType>
void VoxelGrid<PointSourceType>::nearestOctreeNodeSearch(PointSourceType p, Eigen::Vector3d &node_id, int tree_level)
{
	struct timeval start, end;

	int vx = node_id(0);
	int vy = node_id(1);
	int vz = node_id(2);
	double min_dist = DBL_MAX;
	double t_x = static_cast<double>(p.x);
	double t_y = static_cast<double>(p.y);
	double t_z = static_cast<double>(p.z);
	double cur_dist;


	int vgrid_x = octree_size_of_level_[tree_level].size_x;
	int vgrid_y = octree_size_of_level_[tree_level].size_y;
	int vgrid_z = octree_size_of_level_[tree_level].size_z;

	std::vector<Eigen::Vector3d> &current_centroids = octree_centroids_[tree_level];

	int out_x, out_y, out_z;

	double tmp_x, tmp_y, tmp_z;

	std::vector<int> &current_level_points_per_node = points_per_node_[tree_level];

	for (int j = vx * 2; j < vx * 2 + 2 && j < vgrid_x; j++) {
		for (int k = vy * 2; k < vy * 2 + 2 && k < vgrid_y; k++) {
			for (int l = vz * 2; l < vz * 2 + 2 && l < vgrid_z; l++) {
				int nid = j + k * vgrid_x + l * vgrid_x * vgrid_y;
				int points = current_level_points_per_node[nid];
				Eigen::Vector3d node_centr = current_centroids[nid];

				if (points > 0) {
					tmp_x = node_centr(0) - t_x;
					tmp_y = node_centr(1) - t_y;
					tmp_z = node_centr(2) - t_z;

					cur_dist = sqrt(tmp_x * tmp_x + tmp_y * tmp_y + tmp_z * tmp_z);

					if (cur_dist < min_dist) {
						out_x = j;
						out_y = k;
						out_z = l;

						min_dist = cur_dist;
					}
				}
			}
		}
	}

	node_id(0) = out_x;
	node_id(1) = out_y;
	node_id(2) = out_z;
}

template <typename PointSourceType>
double VoxelGrid<PointSourceType>::nearestNeighborDistance(PointSourceType query_point, float max_range)
{
	Eigen::Vector3d node_id;

	node_id.setZero();

	// Go through top of the octree to the bottom
	for (int i = octree_centroids_.size() - 1; i >= 0; i--) {
		nearestOctreeNodeSearch(query_point, node_id, i);
	}

	int voxel_id = node_id(0) + node_id(1) * vgrid_x_ + node_id(2) * vgrid_x_ * vgrid_y_;

	std::vector<int> &pid_list = points_id_[voxel_id];

	float min_dist = FLT_MAX;
	float qx = query_point.x;
	float qy = query_point.y;
	float qz = query_point.z;

	//std::cout << "Voxel id = " << voxel_id << std::endl;
	//std::cout << "PID LIST SIZE = " << pid_list.size() << std::endl;

	for (int i = 0; i < pid_list.size(); i++) {
		float tx = source_cloud_->points[pid_list[i]].x - qx;
		float ty = source_cloud_->points[pid_list[i]].y - qy;
		float tz = source_cloud_->points[pid_list[i]].z - qz;
		float cur_dist = sqrt(tx * tx + ty * ty + tz * tz);

		if (cur_dist < min_dist) {
			min_dist = cur_dist;
		}
	}

	if (min_dist > max_range)
		min_dist = DBL_MAX;

	return static_cast<double>(min_dist);
}

template class VoxelGrid<pcl::PointXYZI>;
template class VoxelGrid<pcl::PointXYZ>;

}
