#include "ndt_cpu/VoxelGrid.h"
#include "ndt_cpu/debug.h"
#include <math.h>
#include <limits>
#include <inttypes.h>

#include <vector>
#include <cmath>

#include <stdio.h>
#include <sys/time.h>

#include "ndt_cpu/SymmetricEigenSolver.h"

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
	min_points_per_voxel_(6),
	real_max_bx_(INT_MIN),
	real_max_by_(INT_MIN),
	real_max_bz_(INT_MIN),
	real_min_bx_(INT_MAX),
	real_min_by_(INT_MAX),
	real_min_bz_(INT_MAX)
{
	centroid_.reset();
	icovariance_.reset();
	points_id_.reset();
	points_per_voxel_.reset();
	tmp_centroid_.reset();
	tmp_cov_.reset();
};

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::roundUp(int input, int factor)
{
	return (input < 0) ? -((-input) / factor) * factor : ((input + factor - 1) / factor) * factor;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::roundDown(int input, int factor)
{
	return (input < 0) ? -((-input + factor - 1) / factor) * factor : (input / factor) * factor;
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::div(int input, int divisor)
{
	return (input < 0) ? -((-input + divisor - 1) / divisor) : input / divisor;
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::initialize()
{
	centroid_.reset();
	centroid_ = boost::make_shared<std::vector<Eigen::Vector3d> >(voxel_num_);

	icovariance_.reset();
	icovariance_ = boost::make_shared<std::vector<Eigen::Matrix3d> >(voxel_num_);

	points_id_.reset();
	points_id_ = boost::make_shared<std::vector<std::vector<int> > >(voxel_num_);

	points_per_voxel_.reset();
	points_per_voxel_ = boost::make_shared<std::vector<int> >(voxel_num_, 0);

	tmp_centroid_.reset();
	tmp_centroid_ = boost::make_shared<std::vector<Eigen::Vector3d> >(voxel_num_);

	tmp_cov_.reset();
	tmp_cov_ = boost::make_shared<std::vector<Eigen::Matrix3d> >(voxel_num_);
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
	return (*centroid_)[voxel_id];
}

template <typename PointSourceType>
Eigen::Matrix3d VoxelGrid<PointSourceType>::getInverseCovariance(int voxel_id) const
{
	return (*icovariance_)[voxel_id];
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
	int idx = static_cast<int>(floor(p.x / voxel_x_)) - min_b_x_;
	int idy = static_cast<int>(floor(p.y / voxel_y_)) - min_b_y_;
	int idz = static_cast<int>(floor(p.z / voxel_z_)) - min_b_z_;

	return (idx + idy * vgrid_x_ + idz * vgrid_x_ * vgrid_y_);
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::voxelId(PointSourceType p,
											float voxel_x, float voxel_y, float voxel_z,
											int min_b_x, int min_b_y, int min_b_z,
											int vgrid_x, int vgrid_y, int vgrid_z)
{
	int idx = static_cast<int>(floor(p.x / voxel_x)) - min_b_x;
	int idy = static_cast<int>(floor(p.y / voxel_y)) - min_b_y;
	int idz = static_cast<int>(floor(p.z / voxel_z)) - min_b_z;

	return (idx + idy * vgrid_x + idz * vgrid_x * vgrid_y);
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::voxelId(int idx, int idy, int idz,
										int min_b_x, int min_b_y, int min_b_z,
										int size_x, int size_y, int size_z)
{
	return (idx - min_b_x) + (idy - min_b_y) * size_x + (idz - min_b_z) * size_x * size_y;
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::computeCentroidAndCovariance()
{
	for (int idx = real_min_bx_; idx <= real_max_bx_; idx++)
		for (int idy = real_min_by_; idy <= real_max_by_; idy++)
			for (int idz = real_min_bz_; idz <= real_max_bz_; idz++) {
				int i = voxelId(idx, idy, idz, min_b_x_, min_b_y_, min_b_z_, vgrid_x_, vgrid_y_, vgrid_z_);
				int ipoint_num = (*points_id_)[i].size();
				double point_num = static_cast<double>(ipoint_num);
				Eigen::Vector3d pt_sum = (*tmp_centroid_)[i];

				if (ipoint_num > 0) {
					(*centroid_)[i] = pt_sum / point_num;
				}

				Eigen::Matrix3d covariance;

				if (ipoint_num >= min_points_per_voxel_) {
					covariance = ((*tmp_cov_)[i] - 2.0 * (pt_sum * (*centroid_)[i].transpose())) / point_num + (*centroid_)[i] * (*centroid_)[i].transpose();
					covariance *= (point_num - 1.0) / point_num;

					SymmetricEigensolver3x3 sv(covariance);

					sv.compute();
					Eigen::Matrix3d evecs = sv.eigenvectors();
					Eigen::Matrix3d evals = sv.eigenvalues().asDiagonal();

					if (evals(0, 0) < 0 || evals(1, 1) < 0 || evals(2, 2) <= 0) {
						(*points_per_voxel_)[i] = -1;
						continue;
					}

					double min_cov_eigvalue = evals(2, 2) * 0.01;

					if (evals(0, 0) < min_cov_eigvalue) {
						evals(0, 0) = min_cov_eigvalue;

						if (evals(1, 1) < min_cov_eigvalue) {
							evals(1, 1) = min_cov_eigvalue;
						}

						covariance = evecs * evals * evecs.inverse();
					}

					(*icovariance_)[i] = covariance.inverse();
				}
			}
}

//Input are supposed to be in device memory
template <typename PointSourceType>
void VoxelGrid<PointSourceType>::setInput(typename pcl::PointCloud<PointSourceType>::Ptr input_cloud)
{
	if (input_cloud->points.size() > 0) {
		/* If no voxel grid was created, then
		 * build the initial voxel grid and octree
		 */
		source_cloud_ = input_cloud;

		findBoundaries();

		std::vector<Eigen::Vector3i> voxel_ids(input_cloud->points.size());

		for (int i = 0; i < input_cloud->points.size(); i++) {
			Eigen::Vector3i &vid = voxel_ids[i];
			PointSourceType p = input_cloud->points[i];

			vid(0) = static_cast<int>(floor(p.x / voxel_x_));
			vid(1) = static_cast<int>(floor(p.y / voxel_y_));
			vid(2) = static_cast<int>(floor(p.z / voxel_z_));
		}

		octree_.setInput(voxel_ids, input_cloud);

		voxel_ids.clear();

		initialize();

		scatterPointsToVoxelGrid();

		computeCentroidAndCovariance();
	}
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::findBoundaries()
{

	findBoundaries(source_cloud_, max_x_, max_y_, max_z_, min_x_, min_y_, min_z_);

	real_max_bx_ = max_b_x_ = static_cast<int> (floor(max_x_ / voxel_x_));
	real_max_by_ = max_b_y_ = static_cast<int> (floor(max_y_ / voxel_y_));
	real_max_bz_ = max_b_z_ = static_cast<int> (floor(max_z_ / voxel_z_));

	real_min_bx_ = min_b_x_ = static_cast<int> (floor(min_x_ / voxel_x_));
	real_min_by_ = min_b_y_ = static_cast<int> (floor(min_y_ / voxel_y_));
	real_min_bz_ = min_b_z_ = static_cast<int> (floor(min_z_ / voxel_z_));

	/* Allocate a poll of memory that is larger than the requested memory so
	 * we do not have to reallocate buffers when the target cloud is set
	 */
	/* Max bounds round toward plus infinity */
	max_b_x_ = roundUp(max_b_x_, MAX_BX_);
	max_b_y_ = roundUp(max_b_y_, MAX_BY_);
	max_b_z_ = roundUp(max_b_z_, MAX_BZ_);

	/* Min bounds round toward minus infinity */
	min_b_x_ = roundDown(min_b_x_, MAX_BX_);
	min_b_y_ = roundDown(min_b_y_, MAX_BY_);
	min_b_z_ = roundDown(min_b_z_, MAX_BZ_);

	vgrid_x_ = max_b_x_ - min_b_x_ + 1;
	vgrid_y_ = max_b_y_ - min_b_y_ + 1;
	vgrid_z_ = max_b_z_ - min_b_z_ + 1;

	if (vgrid_x_ > 0 && vgrid_y_ > 0 && vgrid_z_ > 0) {
		voxel_num_ = vgrid_x_ * vgrid_y_ * vgrid_z_;
	} else {
		voxel_num_ = 0;
	}
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::findBoundaries(typename pcl::PointCloud<PointSourceType>::Ptr input_cloud,
													float &max_x, float &max_y, float &max_z,
													float &min_x, float &min_y, float &min_z)
{

	max_x = max_y = max_z = -FLT_MAX;
	min_x = min_y = min_z = FLT_MAX;

	for (int i = 0; i < input_cloud->points.size(); i++) {
		float x = input_cloud->points[i].x;
		float y = input_cloud->points[i].y;
		float z = input_cloud->points[i].z;

		max_x = (max_x > x) ? max_x : x;
		max_y = (max_y > y) ? max_y : y;
		max_z = (max_z > z) ? max_z : z;

		min_x = (min_x < x) ? min_x : x;
		min_y = (min_y < y) ? min_y : y;
		min_z = (min_z < z) ? min_z : z;
	}
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
	max_id_x = (max_id_x > real_max_bx_) ? real_max_bx_ : max_id_x;
	max_id_y = (max_id_y > real_max_by_) ? real_max_by_ : max_id_y;
	max_id_z = (max_id_z > real_max_bz_) ? real_max_bz_ : max_id_z;

	min_id_x = (min_id_x < real_min_bx_) ? real_min_bx_ : min_id_x;
	min_id_y = (min_id_y < real_min_by_) ? real_min_by_ : min_id_y;
	min_id_z = (min_id_z < real_min_bz_) ? real_min_bz_ : min_id_z;
	int nn = 0;

	for (int idx = min_id_x; idx <= max_id_x && nn < max_nn; idx++) {
		for (int idy = min_id_y; idy <= max_id_y && nn < max_nn; idy++) {
			for (int idz = min_id_z; idz <= max_id_z && nn < max_nn; idz++) {
				int vid = voxelId(idx, idy, idz,
									min_b_x_, min_b_y_, min_b_z_,
									vgrid_x_, vgrid_y_, vgrid_z_);

				if ((*points_per_voxel_)[vid] >= min_points_per_voxel_) {
					double cx = (*centroid_)[vid](0) - static_cast<double>(t_x);
					double cy = (*centroid_)[vid](1) - static_cast<double>(t_y);
					double cz = (*centroid_)[vid](2) - static_cast<double>(t_z);

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

		if ((*points_id_)[vid].size() == 0) {
			(*centroid_)[vid].setZero();
			(*points_per_voxel_)[vid] = 0;
			(*tmp_centroid_)[vid].setZero();
			(*tmp_cov_)[vid].setIdentity();
		}

		(*tmp_centroid_)[vid] += p3d;
		(*tmp_cov_)[vid] += p3d * p3d.transpose();
		(*points_id_)[vid].push_back(pid);
		(*points_per_voxel_)[vid]++;
	}
}

template <typename PointSourceType>
int VoxelGrid<PointSourceType>::nearestVoxel(PointSourceType query_point, Eigen::Matrix<float, 6, 1> boundaries, float max_range)
{
	// Index of the origin of the circle (query point)
	float qx = query_point.x;
	float qy = query_point.y;
	float qz = query_point.z;

	int lower_x = static_cast<int>(floor(boundaries(0) / voxel_x_));
	int lower_y = static_cast<int>(floor(boundaries(1) / voxel_y_));
	int lower_z = static_cast<int>(floor(boundaries(2) / voxel_z_));

	int upper_x = static_cast<int>(floor(boundaries(3) / voxel_x_));
	int upper_y = static_cast<int>(floor(boundaries(4) / voxel_y_));
	int upper_z = static_cast<int>(floor(boundaries(5) / voxel_z_));

	double min_dist = DBL_MAX;
	int nn_vid = -1;

	for (int i = lower_x; i <= upper_x; i++) {
		for (int j = lower_y; j <= upper_y; j++) {
			for (int k = lower_z; k <= upper_z; k++) {
				int vid = voxelId(i, j, k, min_b_x_, min_b_y_, min_b_z_, vgrid_x_, vgrid_y_, vgrid_z_);
				Eigen::Vector3d c = (*centroid_)[vid];

				if ((*points_id_)[vid].size() > 0) {
					double cur_dist = sqrt((qx - c(0)) * (qx - c(0)) + (qy - c(1)) * (qy - c(1)) + (qz - c(2)) * (qz - c(2)));

					if (cur_dist < min_dist) {
						min_dist = cur_dist;
						nn_vid = vid;
					}
				}
			}
		}
	}

	return nn_vid;
}

template <typename PointSourceType>
double VoxelGrid<PointSourceType>::nearestNeighborDistance(PointSourceType q, float max_range)
{
	Eigen::Matrix<float, 6, 1> nn_node_bounds;

	nn_node_bounds = octree_.nearestOctreeNode(q);

	int nn_vid = nearestVoxel(q, nn_node_bounds, max_range);

	Eigen::Vector3d c = (*centroid_)[nn_vid];
	double min_dist = sqrt((q.x - c(0)) * (q.x - c(0)) + (q.y - c(1)) * (q.y - c(1)) + (q.z - c(2)) * (q.z - c(2)));

	if (min_dist >= max_range) {
		return DBL_MAX;
	}

	return min_dist;

}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::updateBoundaries(float max_x, float max_y, float max_z,
													float min_x, float min_y, float min_z)
{

	float new_max_x, new_max_y, new_max_z;
	float new_min_x, new_min_y, new_min_z;

	new_max_x = (max_x_ >= max_x) ? max_x_ : max_x;
	new_max_y = (max_y_ >= max_y) ? max_y_ : max_y;
	new_max_z = (max_z_ >= max_z) ? max_z_ : max_z;

	new_min_x = (min_x_ <= min_x) ? min_x_ : min_x;
	new_min_y = (min_y_ <= min_y) ? min_y_ : min_y;
	new_min_z = (min_z_ <= min_z) ? min_z_ : min_z;

	/* If the boundaries change, then we need to extend the list of voxels */
	if (new_max_x > max_x_ || new_max_y > max_y_ || new_max_z > max_z_ ||
			new_min_x < min_x_ || new_min_y < min_y_ || new_min_z < min_z_) {

		int max_b_x = static_cast<int> (floor(new_max_x / voxel_x_));
		int max_b_y = static_cast<int> (floor(new_max_y / voxel_y_));
		int max_b_z = static_cast<int> (floor(new_max_z / voxel_z_));

		int min_b_x = static_cast<int> (floor(new_min_x / voxel_x_));
		int min_b_y = static_cast<int> (floor(new_min_y / voxel_y_));
		int min_b_z = static_cast<int> (floor(new_min_z / voxel_z_));

		int real_max_bx = max_b_x;
		int real_max_by = max_b_y;
		int real_max_bz = max_b_z;

		int real_min_bx = min_b_x;
		int real_min_by = min_b_y;
		int real_min_bz = min_b_z;

		/* Max bounds round toward plus infinity */
		max_b_x = roundUp(max_b_x, MAX_BX_);
		max_b_y = roundUp(max_b_y, MAX_BY_);
		max_b_z = roundUp(max_b_z, MAX_BZ_);

		/* Min bounds round toward minus infinity */
		min_b_x = roundDown(min_b_x, MAX_BX_);
		min_b_y = roundDown(min_b_y, MAX_BY_);
		min_b_z = roundDown(min_b_z, MAX_BZ_);

		if (max_b_x > max_b_x_ || max_b_y > max_b_y_ || max_b_z > max_b_z_ ||
				min_b_x < min_b_x_ || min_b_y < min_b_y_ || min_b_z < min_b_z_) {
			int vgrid_x = max_b_x - min_b_x + 1;
			int vgrid_y = max_b_y - min_b_y + 1;
			int vgrid_z = max_b_z - min_b_z + 1;

			int voxel_num = vgrid_x * vgrid_y * vgrid_z;

			boost::shared_ptr<std::vector<Eigen::Vector3d> > old_centroid = centroid_;
			boost::shared_ptr<std::vector<Eigen::Matrix3d> > old_icovariance = icovariance_;
			boost::shared_ptr<std::vector<std::vector<int> > > old_points_id = points_id_;
			boost::shared_ptr<std::vector<int> > old_points_per_voxel = points_per_voxel_;

			boost::shared_ptr<std::vector<Eigen::Vector3d> > old_tmp_centroid = tmp_centroid_;
			boost::shared_ptr<std::vector<Eigen::Matrix3d> > old_tmp_cov = tmp_cov_;

			centroid_ = boost::make_shared<std::vector<Eigen::Vector3d> >(voxel_num);
			icovariance_ = boost::make_shared<std::vector<Eigen::Matrix3d> >(voxel_num);
			points_id_ = boost::make_shared<std::vector<std::vector<int> > >(voxel_num);
			points_per_voxel_ = boost::make_shared<std::vector<int> >(voxel_num, 0);
			tmp_centroid_ = boost::make_shared<std::vector<Eigen::Vector3d> >(voxel_num);
			tmp_cov_ = boost::make_shared<std::vector<Eigen::Matrix3d> >(voxel_num);

			// Move the old non-empty voxels to the new list of voxels

			int idx, idy, idz;
			int old_id, new_id;

			for (int idx = real_min_bx_; idx <= real_max_bx_; idx++) {
				for (int idy = real_min_by_; idy <= real_max_by_; idy++) {
					for (int idz = real_min_bz_; idz <= real_max_bz_; idz++) {
						old_id = voxelId(idx, idy, idz,
											min_b_x_, min_b_y_, min_b_z_,
											vgrid_x_, vgrid_y_, vgrid_z_);
						new_id = voxelId(idx, idy, idz,
											min_b_x, min_b_y, min_b_z,
											vgrid_x, vgrid_y, vgrid_z);

						if ((*old_points_id)[old_id].size() > 0) {
							(*points_per_voxel_)[new_id] = (*old_points_per_voxel)[old_id];
							(*centroid_)[new_id] = (*old_centroid)[old_id];
							(*icovariance_)[new_id] = (*old_icovariance)[old_id];
							(*points_id_)[new_id] = (*old_points_id)[old_id];

							(*tmp_centroid_)[new_id] = (*old_tmp_centroid)[old_id];
							(*tmp_cov_)[new_id] = (*old_tmp_cov)[old_id];
						}
					}
				}
			}

			// Update boundaries of voxels
			max_b_x_ = max_b_x;
			max_b_y_ = max_b_y;
			max_b_z_ = max_b_z;

			min_b_x_ = min_b_x;
			min_b_y_ = min_b_y;
			min_b_z_ = min_b_z;

			vgrid_x_ = vgrid_x;
			vgrid_y_ = vgrid_y;
			vgrid_z_ = vgrid_z;

			voxel_num_ = voxel_num;

		}
		// Update actual voxel boundaries
		real_min_bx_ = real_min_bx;
		real_min_by_ = real_min_by;
		real_min_bz_ = real_min_bz;

		real_max_bx_ = real_max_bx;
		real_max_by_ = real_max_by;
		real_max_bz_ = real_max_bz;


		//Update boundaries of points
		max_x_ = new_max_x;
		max_y_ = new_max_y;
		max_z_ = new_max_z;

		min_x_ = new_min_x;
		min_y_ = new_min_y;
		min_z_ = new_min_z;

	}
}


template <typename PointSourceType>
void VoxelGrid<PointSourceType>::update(typename pcl::PointCloud<PointSourceType>::Ptr new_cloud)
{
	if (new_cloud->points.size() <= 0) {
		return;
	}

	float new_max_x, new_max_y, new_max_z;
	float new_min_x, new_min_y, new_min_z;

	// Find boundaries of the new point cloud
	findBoundaries(new_cloud, new_max_x, new_max_y, new_max_z, new_min_x, new_min_y, new_min_z);

	/* Update current boundaries of the voxel grid
	 * Also allocate buffer for new voxel grid and
	 * octree and move the current voxel grid and
	 * octree to the new buffer if necessary
	 */
	updateBoundaries(new_max_x, new_max_y, new_max_z, new_min_x, new_min_y, new_min_z);

	/* Update changed voxels (voxels that contains new points).
	 * Update centroids of voxels and their covariance matrixes
	 * as well as inverse covariance matrixes */
	updateVoxelContent(new_cloud);

	/* Update octree */
	std::vector<Eigen::Vector3i> new_voxel_id(new_cloud->points.size());

	for (int i = 0; i < new_cloud->points.size(); i++) {
		Eigen::Vector3i &vid = new_voxel_id[i];
		PointSourceType p = new_cloud->points[i];

		vid(0) = static_cast<int>(floor(p.x / voxel_x_));
		vid(1) = static_cast<int>(floor(p.y / voxel_y_));
		vid(2) = static_cast<int>(floor(p.z / voxel_z_));
	}

	octree_.update(new_voxel_id, new_cloud);

	*source_cloud_ += *new_cloud;
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::updateVoxelContent(typename pcl::PointCloud<PointSourceType>::Ptr new_cloud)
{
	int total_points_num = source_cloud_->points.size();

	for (int i = 0; i < new_cloud->points.size(); i++) {
		PointSourceType p = new_cloud->points[i];
		Eigen::Vector3d p3d(p.x, p.y, p.z);
		int vx = static_cast<int>(floor(p.x / voxel_x_));
		int vy = static_cast<int>(floor(p.y / voxel_y_));
		int vz = static_cast<int>(floor(p.z / voxel_z_));
		int vid = voxelId(vx, vy, vz, min_b_x_, min_b_y_, min_b_z_, vgrid_x_, vgrid_y_, vgrid_z_);
		std::vector<int> &tmp_pid = (*points_id_)[vid];

		if (tmp_pid.size() == 0) {
			(*centroid_)[vid].setZero();
			(*tmp_cov_)[vid].setIdentity();
			(*tmp_centroid_)[vid].setZero();
		}

		(*tmp_centroid_)[vid] += p3d;
		(*tmp_cov_)[vid] += p3d * p3d.transpose();
		(*points_id_)[vid].push_back(total_points_num + i);

		// Update centroids
		int ipoint_num = (*points_id_)[vid].size();
		(*centroid_)[vid] = (*tmp_centroid_)[vid] / static_cast<double>(ipoint_num);
		(*points_per_voxel_)[vid] = ipoint_num;


		// Update covariance
		double point_num = static_cast<double>(ipoint_num);
		Eigen::Vector3d pt_sum = (*tmp_centroid_)[vid];

		// Update  centroids
		(*centroid_)[vid] = (*tmp_centroid_)[vid] / point_num;

		if (ipoint_num >= min_points_per_voxel_) {
			Eigen::Matrix3d covariance;
			covariance = ((*tmp_cov_)[vid] - 2.0 * (pt_sum * (*centroid_)[vid].transpose())) / point_num + (*centroid_)[vid] * (*centroid_)[vid].transpose();
			covariance *= (point_num - 1.0) / point_num;

			SymmetricEigensolver3x3 sv(covariance);

			sv.compute();
			Eigen::Matrix3d evecs = sv.eigenvectors();
			Eigen::Matrix3d evals = sv.eigenvalues().asDiagonal();

			if (evals(0, 0) < 0 || evals(1, 1) < 0 || evals(2, 2) <= 0) {
				(*points_per_voxel_)[vid] = -1;
				continue;
			}

			double min_cov_eigvalue = evals(2, 2) * 0.01;

			if (evals(0, 0) < min_cov_eigvalue) {
				evals(0, 0) = min_cov_eigvalue;

				if (evals(1, 1) < min_cov_eigvalue) {
					evals(1, 1) = min_cov_eigvalue;
				}

				covariance = evecs * evals * evecs.inverse();
			}

			(*icovariance_)[vid] = covariance.inverse();
		}
	}
}

template class VoxelGrid<pcl::PointXYZI>;
template class VoxelGrid<pcl::PointXYZ>;

}
