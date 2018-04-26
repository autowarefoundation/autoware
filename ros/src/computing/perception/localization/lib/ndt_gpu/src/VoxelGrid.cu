#include "ndt_gpu/VoxelGrid.h"
#include "ndt_gpu/debug.h"
#include "ndt_gpu/common.h"
#include <math.h>
#include <limits>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/scan.h>
#include <thrust/fill.h>
#include <inttypes.h>

#include <vector>
#include <cmath>

#include <stdio.h>
#include <sys/time.h>

#include "ndt_gpu/SymmetricEigenSolver.h"

namespace gpu {

GVoxelGrid::GVoxelGrid():
	x_(NULL),
	y_(NULL),
	z_(NULL),
	points_num_(0),
	centroid_(NULL),
	covariance_(NULL),
	inverse_covariance_(NULL),
	points_per_voxel_(NULL),
	voxel_num_(0),
	max_x_(FLT_MAX),
	max_y_(FLT_MAX),
	max_z_(FLT_MAX),
	min_x_(FLT_MIN),
	min_y_(FLT_MIN),
	min_z_(FLT_MIN),
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
	starting_point_ids_(NULL),
	point_ids_(NULL),
	is_copied_(false)
{

};

GVoxelGrid::GVoxelGrid(const GVoxelGrid &other)
{
	x_ = other.x_;
	y_ = other.y_;
	z_ = other.z_;

	points_num_ = other.points_num_;

	centroid_ = other.centroid_;
	covariance_ = other.covariance_;
	inverse_covariance_ = other.inverse_covariance_;
	points_per_voxel_ = other.points_per_voxel_;

	voxel_num_ = other.voxel_num_;
	max_x_ = other.max_x_;
	max_y_ = other.max_y_;
	max_z_ = other.max_z_;

	min_x_ = other.min_x_;
	min_y_ = other.min_y_;
	min_z_ = other.min_z_;

	voxel_x_ = other.voxel_x_;
	voxel_y_ = other.voxel_y_;
	voxel_z_ = other.voxel_z_;

	max_b_x_ = other.max_b_x_;
	max_b_y_ = other.max_b_y_;
	max_b_z_ = other.max_b_z_;

	min_b_x_ = other.min_b_x_;
	min_b_y_ = other.min_b_y_;
	min_b_z_ = other.min_b_z_;

	vgrid_x_ = other.vgrid_x_;
	vgrid_y_ = other.vgrid_y_;
	vgrid_z_ = other.vgrid_z_;

	min_points_per_voxel_ = other.min_points_per_voxel_;

	starting_point_ids_ = other.starting_point_ids_;
	point_ids_ = other.point_ids_;


	is_copied_ = true;
}


GVoxelGrid::~GVoxelGrid() {
	if (!is_copied_) {

		for (unsigned int i = 1; i < octree_centroids_.size(); i++) {
			if (octree_centroids_[i] != NULL) {
				checkCudaErrors(cudaFree(octree_centroids_[i]));
				octree_centroids_[i] = NULL;
			}
			if (octree_points_per_node_[i] != NULL) {
				checkCudaErrors(cudaFree(octree_points_per_node_[i]));
				octree_points_per_node_[i] = NULL;
			}
		}

		octree_centroids_.clear();
		octree_points_per_node_.clear();
		octree_grid_size_.clear();

		if (starting_point_ids_ != NULL) {
			checkCudaErrors(cudaFree(starting_point_ids_));
			starting_point_ids_ = NULL;
		}

		if (point_ids_ != NULL) {
			checkCudaErrors(cudaFree(point_ids_));
			point_ids_ = NULL;
		}

		if (centroid_ != NULL) {
			checkCudaErrors(cudaFree(centroid_));
			centroid_ = NULL;
		}

		if (covariance_ != NULL) {
			checkCudaErrors(cudaFree(covariance_));
			covariance_ = NULL;
		}

		if (inverse_covariance_ != NULL) {
			checkCudaErrors(cudaFree(inverse_covariance_));
			inverse_covariance_ = NULL;
		}

		if (points_per_voxel_ != NULL) {
			checkCudaErrors(cudaFree(points_per_voxel_));
			points_per_voxel_ = NULL;
		}
	}
}


void GVoxelGrid::initialize()
{
	if (centroid_ != NULL) {
		checkCudaErrors(cudaFree(centroid_));
		centroid_ = NULL;
	}

	if (covariance_ != NULL) {
		checkCudaErrors(cudaFree(covariance_));
		covariance_ = NULL;
	}

	if (inverse_covariance_ != NULL) {
		checkCudaErrors(cudaFree(inverse_covariance_));
		inverse_covariance_ = NULL;
	}

	if (points_per_voxel_ != NULL) {
		checkCudaErrors(cudaFree(points_per_voxel_));
		points_per_voxel_ = NULL;
	}

	checkCudaErrors(cudaMalloc(&centroid_, sizeof(double) * 3 * voxel_num_));
	checkCudaErrors(cudaMalloc(&covariance_, sizeof(double) * 9 * voxel_num_));
	checkCudaErrors(cudaMalloc(&inverse_covariance_, sizeof(double) * 9 * voxel_num_));
	checkCudaErrors(cudaMalloc(&points_per_voxel_, sizeof(int) * voxel_num_));

	checkCudaErrors(cudaMemset(inverse_covariance_, 0, sizeof(double) * 9 * voxel_num_));
	checkCudaErrors(cudaMemset(points_per_voxel_, 0, sizeof(int) * voxel_num_));
	checkCudaErrors(cudaDeviceSynchronize());
}

int GVoxelGrid::getVoxelNum() const
{
	return voxel_num_;
}


float GVoxelGrid::getMaxX() const
{
	return max_x_;
}
float GVoxelGrid::getMaxY() const
{
	return max_y_;
}
float GVoxelGrid::getMaxZ() const
{
	return max_z_;
}


float GVoxelGrid::getMinX() const
{
	return min_x_;
}
float GVoxelGrid::getMinY() const
{
	return min_y_;
}
float GVoxelGrid::getMinZ() const
{
	return min_z_;
}


float GVoxelGrid::getVoxelX() const
{
	return voxel_x_;
}
float GVoxelGrid::getVoxelY() const
{
	return voxel_y_;
}
float GVoxelGrid::getVoxelZ() const
{
	return voxel_z_;
}


int GVoxelGrid::getMaxBX() const
{
	return max_b_x_;
}
int GVoxelGrid::getMaxBY() const
{
	return max_b_y_;
}
int GVoxelGrid::getMaxBZ() const
{
	return max_b_z_;
}


int GVoxelGrid::getMinBX() const
{
	return min_b_x_;
}
int GVoxelGrid::getMinBY() const
{
	return min_b_y_;
}
int GVoxelGrid::getMinBZ() const
{
	return min_b_z_;
}


int GVoxelGrid::getVgridX() const
{
	return vgrid_x_;
}
int GVoxelGrid::getVgridY() const
{
	return vgrid_y_;
}
int GVoxelGrid::getVgridZ() const
{
	return vgrid_z_;
}


void GVoxelGrid::setLeafSize(float voxel_x, float voxel_y, float voxel_z)
{
	voxel_x_ = voxel_x;
	voxel_y_ = voxel_y;
	voxel_z_ = voxel_z;
}

double* GVoxelGrid::getCentroidList() const
{
	return centroid_;
}

double* GVoxelGrid::getCovarianceList() const
{
	return covariance_;
}

double* GVoxelGrid::getInverseCovarianceList() const
{
	return inverse_covariance_;
}

int* GVoxelGrid::getPointsPerVoxelList() const
{
	return points_per_voxel_;
}


extern "C" __device__ int voxelId(float x, float y, float z,
									float voxel_x, float voxel_y, float voxel_z,
									int min_b_x, int min_b_y, int min_b_z,
									int vgrid_x, int vgrid_y, int vgrid_z)
{
	int id_x = static_cast<int>(floorf(x / voxel_x) - static_cast<float>(min_b_x));
	int id_y = static_cast<int>(floorf(y / voxel_y) - static_cast<float>(min_b_y));
	int id_z = static_cast<int>(floorf(z / voxel_z) - static_cast<float>(min_b_z));

	return (id_x + id_y * vgrid_x + id_z * vgrid_x * vgrid_y);
}

/* First step to compute centroids and covariances of voxels. */
extern "C" __global__ void initCentroidAndCovariance(float *x, float *y, float *z, int *starting_point_ids, int *point_ids,
														double *centroids, double *covariances, int voxel_num)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = idx; i < voxel_num; i += stride) {
		MatrixDevice centr(3, 1, voxel_num, centroids + i);
		MatrixDevice cov(3, 3, voxel_num, covariances + i);

		double centr0, centr1, centr2;
		double cov00, cov01, cov02, cov11, cov12, cov22;

		centr0 = centr1 = centr2 = 0.0;
		cov00 = cov11 = cov22 = 1.0;
		cov01 = cov02 = cov12 = 0.0;

		for (int j = starting_point_ids[i]; j < starting_point_ids[i + 1]; j++) {
			int pid = point_ids[j];
			double t_x = static_cast<double>(x[pid]);
			double t_y = static_cast<double>(y[pid]);
			double t_z = static_cast<double>(z[pid]);

			centr0 += t_x;
			centr1 += t_y;
			centr2 += t_z;

			cov00 += t_x * t_x;
			cov01 += t_x * t_y;
			cov02 += t_x * t_z;
			cov11 += t_y * t_y;
			cov12 += t_y * t_z;
			cov22 += t_z * t_z;
		}

		centr(0) = centr0;
		centr(1) = centr1;
		centr(2) = centr2;

		cov(0, 0) = cov00;
		cov(0, 1) = cov01;
		cov(0, 2) = cov02;
		cov(1, 1) = cov11;
		cov(1, 2) = cov12;
		cov(2, 2) = cov22;
	}
}

/* Update centroids of voxels. */
extern "C" __global__ void updateVoxelCentroid(double *centroid, int *points_per_voxel, int voxel_num)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int vid = index; vid < voxel_num; vid += stride) {
		MatrixDevice centr(3, 1, voxel_num, centroid + vid);
		double points_num = static_cast<double>(points_per_voxel[vid]);

		if (points_num > 0) {
			centr /= points_num;
		}
	}
}

/* Update covariance of voxels. */
extern "C" __global__ void updateVoxelCovariance(double *centroid, double *pt_sum, double *covariance, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int vid = index; vid < voxel_num; vid += stride) {
		MatrixDevice centr(3, 1, voxel_num, centroid + vid);
		MatrixDevice cov(3, 3, voxel_num, covariance + vid);
		MatrixDevice pts(3, 1, voxel_num, pt_sum + vid);
		double points_num = static_cast<double>(points_per_voxel[vid]);

		double c0 = centr(0);
		double c1 = centr(1);
		double c2 = centr(2);
		double p0 = pts(0);
		double p1 = pts(1);
		double p2 = pts(2);

		points_per_voxel[vid] = (points_num < min_points_per_voxel) ? 0 : points_num;

		if (points_num >= min_points_per_voxel) {

			double mult = (points_num - 1.0) / points_num;

			cov(0, 0) = ((cov(0, 0) - 2.0 * p0 * c0) / points_num + c0 * c0) * mult;
			cov(0, 1) = ((cov(0, 1) - 2.0 * p0 * c1) / points_num + c0 * c1) * mult;
			cov(0, 2) = ((cov(0, 2) - 2.0 * p0 * c2) / points_num + c0 * c2) * mult;
			cov(1, 0) = cov(0, 1);
			cov(1, 1) = ((cov(1, 1) - 2.0 * p1 * c1) / points_num + c1 * c1) * mult;
			cov(1, 2) = ((cov(1, 2) - 2.0 * p1 * c2) / points_num + c1 * c2) * mult;
			cov(2, 0) = cov(0, 2);
			cov(2, 1) = cov(1, 2);
			cov(2, 2) = ((cov(2, 2) - 2.0 * p2 * c2) / points_num + c2 * c2) * mult;
		}
	}
}

extern "C" __global__ void computeInverseEigenvectors(double *inverse_covariance, int *points_per_voxel, int voxel_num, double *eigenvectors, int min_points_per_voxel)
{

	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int vid = index; vid < voxel_num; vid += stride) {
		if (points_per_voxel[vid] >= min_points_per_voxel) {
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + vid);
			MatrixDevice eigen_vectors(3, 3, voxel_num, eigenvectors + vid);

			eigen_vectors.inverse(icov);
		}

		__syncthreads();
	}
}

//eigen_vecs = eigen_vecs * eigen_val

extern "C" __global__ void updateCovarianceS0(int *points_per_voxel, int voxel_num, double *eigenvalues, double *eigenvectors, int min_points_per_voxel)
{

	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int vid = index; vid < voxel_num; vid += stride) {
		if (points_per_voxel[vid] >= min_points_per_voxel) {
			MatrixDevice eigen_vectors(3, 3, voxel_num, eigenvectors + vid);

			double eig_val0 = eigenvalues[vid];
			double eig_val1 = eigenvalues[vid + voxel_num];
			double eig_val2 = eigenvalues[vid + 2 * voxel_num];

			eigen_vectors(0, 0) *= eig_val0;
			eigen_vectors(1, 0) *= eig_val0;
			eigen_vectors(2, 0) *= eig_val0;

			eigen_vectors(0, 1) *= eig_val1;
			eigen_vectors(1, 1) *= eig_val1;
			eigen_vectors(2, 1) *= eig_val1;

			eigen_vectors(0, 2) *= eig_val2;
			eigen_vectors(1, 2) *= eig_val2;
			eigen_vectors(2, 2) *= eig_val2;
		}

		__syncthreads();
	}
}

//cov = new eigen_vecs * eigen_vecs transpose

extern "C" __global__ void updateCovarianceS1(double *covariance, double *inverse_covariance, int *points_per_voxel, int voxel_num, double *eigenvectors, int min_points_per_voxel, int col)
{

	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int vid = index; vid < voxel_num; vid += stride) {
		if (points_per_voxel[vid] >= min_points_per_voxel) {
			MatrixDevice cov(3, 3, voxel_num, covariance + vid);
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + vid);
			MatrixDevice eigen_vectors(3, 3, voxel_num, eigenvectors + vid);

			double tmp0 = icov(0, col);
			double tmp1 = icov(1, col);
			double tmp2 = icov(2, col);

			cov(0, col) = eigen_vectors(0, 0) * tmp0 + eigen_vectors(0, 1) * tmp1 + eigen_vectors(0, 2) * tmp2;
			cov(1, col) = eigen_vectors(1, 0) * tmp0 + eigen_vectors(1, 1) * tmp1 + eigen_vectors(1, 2) * tmp2;
			cov(2, col) = eigen_vectors(2, 0) * tmp0 + eigen_vectors(2, 1) * tmp1 + eigen_vectors(2, 2) * tmp2;
		}

		__syncthreads();
	}
}

extern "C" __global__ void computeInverseCovariance(double *covariance, double *inverse_covariance, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{

	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int vid = index; vid < voxel_num; vid += stride) {
		if (points_per_voxel[vid] >= min_points_per_voxel) {
			MatrixDevice cov(3, 3, voxel_num, covariance + vid);
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + vid);

			cov.inverse(icov);
		}
		__syncthreads();
	}
}


template<typename T>
__global__ void init(T *input, int size, int local_size)
{
	for (int i = threadIdx.x + blockIdx.x * blockDim.x; i < size; i += blockDim.x * gridDim.x) {
		for (int j = 0; j < local_size; j++)
			input[i + j * size] = 1;
	}
}

extern "C" __global__ void initBoolean(bool *input, int size)
{
	for (int i = threadIdx.x + blockIdx.x * blockDim.x; i < size; i += blockDim.x * gridDim.x) {
		input[i] = (i % 2 == 0) ? true : false;
	}
}

/* Normalize input matrices to avoid overflow. */
extern "C" __global__ void normalize(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < voxel_num; id += blockDim.x * gridDim.x) {
		if (points_per_voxel[id] >= min_points_per_voxel) {}
			sv.normalizeInput(id);
		__syncthreads();
	}
}

/* Compute eigenvalues. Eigenvalues are arranged in increasing order.
 * (eigen(0) <= eigen(1) <= eigen(2). */
extern "C" __global__ void computeEigenvalues(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < voxel_num; id += blockDim.x * gridDim.x) {
		if (points_per_voxel[id] >= min_points_per_voxel)
			sv.computeEigenvalues(id);
		__syncthreads();
	}
}

/* First step to compute eigenvector 0 of covariance matrices. */
extern "C" __global__ void computeEvec00(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < voxel_num; id += blockDim.x * gridDim.x) {
		if (points_per_voxel[id] >= min_points_per_voxel)
			sv.computeEigenvector00(id);
		__syncthreads();
	}
}

/* Second step to compute eigenvector 0 of covariance matrices. */
extern "C" __global__ void computeEvec01(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < voxel_num; id += blockDim.x * gridDim.x) {
		if (points_per_voxel[id] >= min_points_per_voxel)
			sv.computeEigenvector01(id);
		__syncthreads();
	}
}

/* First step to compute eigenvector 1 of covariance matrices. */
extern "C" __global__ void computeEvec10(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < voxel_num; id += blockDim.x * gridDim.x) {
		if (points_per_voxel[id] >= min_points_per_voxel)
			sv.computeEigenvector10(id);
		__syncthreads();
	}
}

/* Second step to compute eigenvector 1 of covariance matrices. */
extern "C" __global__ void computeEvec11(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < voxel_num; id += blockDim.x * gridDim.x) {
		if (points_per_voxel[id] >= min_points_per_voxel)
			sv.computeEigenvector11(id);
		__syncthreads();
	}
}

/* Compute eigenvector 2 of covariance matrices. */
extern "C" __global__ void computeEvec2(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < voxel_num; id += blockDim.x * gridDim.x) {
		if (points_per_voxel[id] >= min_points_per_voxel)
			sv.computeEigenvector2(id);
		__syncthreads();
	}
}

/* Final step to compute eigenvalues. */
extern "C" __global__ void updateEval(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < voxel_num; id += blockDim.x * gridDim.x) {
		if (points_per_voxel[id] >= min_points_per_voxel)
			sv.updateEigenvalues(id);
		__syncthreads();
	}
}

/* Update eigenvalues in the case covariance matrix is nearly singular. */
extern "C" __global__ void updateEval2(double *eigenvalues, int *points_per_voxel, int voxel_num, int min_points_per_voxel)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < voxel_num; id += blockDim.x * gridDim.x) {
		if (points_per_voxel[id] >= min_points_per_voxel) {
			MatrixDevice eigen_val(3, 1, voxel_num, eigenvalues + id);
			double ev0 = eigen_val(0);
			double ev1 = eigen_val(1);
			double ev2 = eigen_val(2);


			if (ev0 < 0 || ev1 < 0 || ev2 <= 0) {
				points_per_voxel[id] = 0;
				continue;
			}

			double min_cov_eigvalue = ev2 * 0.01;

			if (ev0 < min_cov_eigvalue) {
				ev0 = min_cov_eigvalue;

				if (ev1 < min_cov_eigvalue) {
					ev1 = min_cov_eigvalue;
				}
			}

			eigen_val(0) = ev0;
			eigen_val(1) = ev1;
			eigen_val(2) = ev2;

			__syncthreads();
		}
	}
}

void GVoxelGrid::computeCentroidAndCovariance()
{
	int block_x = (voxel_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : voxel_num_;
	int grid_x = (voxel_num_ - 1) / block_x + 1;

	initCentroidAndCovariance<<<grid_x, block_x>>>(x_, y_, z_, starting_point_ids_, point_ids_, centroid_, covariance_, voxel_num_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	double *pt_sum;

	checkCudaErrors(cudaMalloc(&pt_sum, sizeof(double) * voxel_num_ * 3));

	checkCudaErrors(cudaMemcpy(pt_sum, centroid_, sizeof(double) * voxel_num_ * 3, cudaMemcpyDeviceToDevice));

	updateVoxelCentroid<<<grid_x, block_x>>>(centroid_, points_per_voxel_, voxel_num_);
	checkCudaErrors(cudaGetLastError());

	updateVoxelCovariance<<<grid_x, block_x>>>(centroid_, pt_sum, covariance_, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaFree(pt_sum));

	double *eigenvalues_dev, *eigenvectors_dev;

	checkCudaErrors(cudaMalloc(&eigenvalues_dev, sizeof(double) * 3 * voxel_num_));
	checkCudaErrors(cudaMalloc(&eigenvectors_dev, sizeof(double) * 9 * voxel_num_));

	// Solving eigenvalues and eigenvectors problem by the GPU.
	SymmetricEigensolver3x3 sv(voxel_num_);

	sv.setInputMatrices(covariance_);
	sv.setEigenvalues(eigenvalues_dev);
	sv.setEigenvectors(eigenvectors_dev);

	normalize<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEigenvalues<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec00<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec01<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec10<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec11<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec2<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	updateEval<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	updateEval2<<<grid_x, block_x>>>(eigenvalues_dev, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeInverseEigenvectors<<<grid_x, block_x>>>(inverse_covariance_, points_per_voxel_, voxel_num_, eigenvectors_dev, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	updateCovarianceS0<<<grid_x, block_x>>>(points_per_voxel_, voxel_num_, eigenvalues_dev, eigenvectors_dev, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	for (int i = 0; i < 3; i++) {
		updateCovarianceS1<<<grid_x, block_x>>>(covariance_, inverse_covariance_, points_per_voxel_, voxel_num_, eigenvectors_dev, min_points_per_voxel_, i);
		checkCudaErrors(cudaGetLastError());
	}
	checkCudaErrors(cudaDeviceSynchronize());

	computeInverseCovariance<<<grid_x, block_x>>>(covariance_, inverse_covariance_, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	checkCudaErrors(cudaDeviceSynchronize());

	sv.memFree();
	checkCudaErrors(cudaFree(eigenvalues_dev));
	checkCudaErrors(cudaFree(eigenvectors_dev));
}

//Input are supposed to be in device memory
void GVoxelGrid::setInput(float *x, float *y, float *z, int points_num)
{
	if (points_num <= 0)
		return;
	x_ = x;
	y_ = y;
	z_ = z;
	points_num_ = points_num;

	findBoundaries();

	voxel_num_ = vgrid_x_ * vgrid_y_ * vgrid_z_;

	initialize();

	scatterPointsToVoxelGrid();

	computeCentroidAndCovariance();

	buildOctree();
}

/* Find the largest coordinate values */
extern "C"  __global__ void findMax(float *x, float *y, float *z, int full_size, int half_size)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = index; i < half_size; i += stride) {
		x[i] = (i + half_size < full_size) ? ((x[i] >= x[i + half_size]) ? x[i] : x[i + half_size]) : x[i];
		y[i] = (i + half_size < full_size) ? ((y[i] >= y[i + half_size]) ? y[i] : y[i + half_size]) : y[i];
		z[i] = (i + half_size < full_size) ? ((z[i] >= z[i + half_size]) ? z[i] : z[i + half_size]) : z[i];
	}
}

/* Find the smallest coordinate values */
extern "C"  __global__ void findMin(float *x, float *y, float *z, int full_size, int half_size)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = index; i < half_size; i += stride) {
		x[i] = (i + half_size < full_size) ? ((x[i] <= x[i + half_size]) ? x[i] : x[i + half_size]) : x[i];
		y[i] = (i + half_size < full_size) ? ((y[i] <= y[i + half_size]) ? y[i] : y[i + half_size]) : y[i];
		z[i] = (i + half_size < full_size) ? ((z[i] <= z[i + half_size]) ? z[i] : z[i + half_size]) : z[i];
	}
}


void GVoxelGrid::findBoundaries()
{
	float *max_x, *max_y, *max_z, *min_x, *min_y, *min_z;

	checkCudaErrors(cudaMalloc(&max_x, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&max_y, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&max_z, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&min_x, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&min_y, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&min_z, sizeof(float) * points_num_));

	checkCudaErrors(cudaMemcpy(max_x, x_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(max_y, y_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(max_z, z_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));

	checkCudaErrors(cudaMemcpy(min_x, x_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(min_y, y_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(min_z, z_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));

	int points_num = points_num_;

	while (points_num > 1) {
		int half_points_num = (points_num - 1) / 2 + 1;
		int block_x = (half_points_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : half_points_num;
		int grid_x = (half_points_num - 1) / block_x + 1;

		findMax<<<grid_x, block_x>>>(max_x, max_y, max_z, points_num, half_points_num);
		checkCudaErrors(cudaGetLastError());

		findMin<<<grid_x, block_x>>>(min_x, min_y, min_z, points_num, half_points_num);
		checkCudaErrors(cudaGetLastError());

		points_num = half_points_num;
	}

	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaMemcpy(&max_x_, max_x, sizeof(float), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&max_y_, max_y, sizeof(float), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&max_z_, max_z, sizeof(float), cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaMemcpy(&min_x_, min_x, sizeof(float), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&min_y_, min_y, sizeof(float), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&min_z_, min_z, sizeof(float), cudaMemcpyDeviceToHost));

	max_b_x_ = static_cast<int> (floor(max_x_ / voxel_x_));
	max_b_y_ = static_cast<int> (floor(max_y_ / voxel_y_));
	max_b_z_ = static_cast<int> (floor(max_z_ / voxel_z_));

	min_b_x_ = static_cast<int> (floor(min_x_ / voxel_x_));
	min_b_y_ = static_cast<int> (floor(min_y_ / voxel_y_));
	min_b_z_ = static_cast<int> (floor(min_z_ / voxel_z_));

	vgrid_x_ = max_b_x_ - min_b_x_ + 1;
	vgrid_y_ = max_b_y_ - min_b_y_ + 1;
	vgrid_z_ = max_b_z_ - min_b_z_ + 1;

	checkCudaErrors(cudaFree(max_x));
	checkCudaErrors(cudaFree(max_y));
	checkCudaErrors(cudaFree(max_z));

	checkCudaErrors(cudaFree(min_x));
	checkCudaErrors(cudaFree(min_y));
	checkCudaErrors(cudaFree(min_z));
}

/* Find indexes idx, idy and idz of candidate voxels */
extern "C"  __global__ void findBoundariesOfCandidateVoxels(float *x, float *y, float *z,
																float radius, int points_num,
																float voxel_x, float voxel_y, float voxel_z,
																int max_b_x, int max_b_y, int max_b_z,
																int min_b_x, int min_b_y, int min_b_z,
																int *max_vid_x, int *max_vid_y, int *max_vid_z,
																int *min_vid_x, int *min_vid_y, int *min_vid_z,
																int *candidate_voxel_per_point)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < points_num; i += stride) {
		float t_x = x[i];
		float t_y = y[i];
		float t_z = z[i];

		int max_id_x = static_cast<int>(floorf((t_x + radius) / voxel_x));
		int max_id_y = static_cast<int>(floorf((t_y + radius) / voxel_y));
		int max_id_z = static_cast<int>(floorf((t_z + radius) / voxel_z));

		int min_id_x = static_cast<int>(floorf((t_x - radius) / voxel_x));
		int min_id_y = static_cast<int>(floorf((t_y - radius) / voxel_y));
		int min_id_z = static_cast<int>(floorf((t_z - radius) / voxel_z));

		/* Find intersection of the cube containing
		 * the NN sphere of the point and the voxel grid
		 */
		max_id_x = (max_id_x > max_b_x) ? max_b_x - min_b_x : max_id_x - min_b_x;
		max_id_y = (max_id_y > max_b_y) ? max_b_y - min_b_y : max_id_y - min_b_y;
		max_id_z = (max_id_z > max_b_z) ? max_b_z - min_b_z : max_id_z - min_b_z;

		min_id_x = (min_id_x < min_b_x) ? 0 : min_id_x - min_b_x;
		min_id_y = (min_id_y < min_b_y) ? 0 : min_id_y - min_b_y;
		min_id_z = (min_id_z < min_b_z) ? 0 : min_id_z - min_b_z;

		int vx = max_id_x - min_id_x + 1;
		int vy = max_id_y - min_id_y + 1;
		int vz = max_id_z - min_id_z + 1;

		candidate_voxel_per_point[i] = (vx > 0 && vy > 0 && vz > 0) ? vx * vy * vz : 0;

		max_vid_x[i] = max_id_x;
		max_vid_y[i] = max_id_y;
		max_vid_z[i] = max_id_z;

		min_vid_x[i] = min_id_x;
		min_vid_y[i] = min_id_y;
		min_vid_z[i] = min_id_z;
	}
}

/* Write id of valid points to the output buffer */
extern "C"  __global__ void collectValidPoints(int *valid_points_mark, int *valid_points_id, int *valid_points_location, int points_num)
{
	for (int index = threadIdx.x + blockIdx.x * blockDim.x; index < points_num; index += blockDim.x * gridDim.x) {
		if (valid_points_mark[index] != 0) {
			valid_points_id[valid_points_location[index]] = index;
		}
	}
}

/* Compute the global index of candidate voxels.
 * global index = idx + idy * grid size x + idz * grid_size x * grid size y */
extern "C"  __global__ void updateCandidateVoxelIds(int points_num,
													int vgrid_x, int vgrid_y, int vgrid_z,
													int *max_vid_x, int *max_vid_y, int *max_vid_z,
													int *min_vid_x, int *min_vid_y, int *min_vid_z,
													int *starting_voxel_id,
													int *candidate_voxel_id)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < points_num; i += stride) {
		int max_id_x = max_vid_x[i];
		int max_id_y = max_vid_y[i];
		int max_id_z = max_vid_z[i];

		int min_id_x = min_vid_x[i];
		int min_id_y = min_vid_y[i];
		int min_id_z = min_vid_z[i];

		int write_location = starting_voxel_id[i];

		for (int j = min_id_x; j <= max_id_x; j++) {
			for (int k = min_id_y; k <= max_id_y; k++) {
				for (int l = min_id_z; l <= max_id_z; l++) {
					candidate_voxel_id[write_location] = j + k * vgrid_x + l * vgrid_x * vgrid_y;
					write_location++;
				}
			}
		}
	}
}


/* Find out which voxels are really inside the radius.
 * This is done by comparing the distance between the centroid
 * of the voxel and the query point with the radius.
 *
 * The valid_voxel_mark store the result of the inspection, which is 0
 * if the centroid is outside the radius and 1 otherwise.
 *
 * The valid_points_mark store the status of the inspection per point.
 * It is 0 if there is no voxels in the candidate list is truly a neighbor
 * of the point, and 1 otherwise.
 *
 * The valid_voxel_count store the number of true neighbor voxels.
 */
extern "C" __global__ void inspectCandidateVoxels(float *x, float *y, float *z,
													float radius, int max_nn, int points_num,
													double *centroid, int *points_per_voxel, int offset,
													int *starting_voxel_id, int *candidate_voxel_id,
													int *valid_voxel_mark, int *valid_voxel_count, int *valid_points_mark)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < points_num; i += stride) {
		double t_x = static_cast<double>(x[i]);
		double t_y = static_cast<double>(y[i]);
		double t_z = static_cast<double>(z[i]);

		int nn = 0;
		for (int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1] && nn <= max_nn; j++) {
			int point_num = points_per_voxel[candidate_voxel_id[j]];
			MatrixDevice centr(3, 1, offset, centroid + candidate_voxel_id[j]);

			double centroid_x = (point_num > 0) ? (t_x - centr(0)) : radius + 1;
			double centroid_y = (point_num > 0) ? (t_y - centr(1)) : 0;
			double centroid_z = (point_num > 0) ? (t_z - centr(2)) : 0;

			bool res = (norm3d(centroid_x, centroid_y, centroid_z) <= radius);

			valid_voxel_mark[j] = (res) ? 1 : 0;
			nn += (res) ? 1 : 0;
		}

		valid_voxel_count[i] = nn;
		valid_points_mark[i] = (nn > 0) ? 1 : 0;

		__syncthreads();
	}
}

/* Write the id of valid voxels to the output buffer */
extern "C"  __global__ void collectValidVoxels(int *valid_voxels_mark, int *candidate_voxel_id, int *output, int *writing_location, int candidate_voxel_num)
{
	for (int index = threadIdx.x + blockIdx.x * blockDim.x; index < candidate_voxel_num; index += blockDim.x * gridDim.x) {
		if (valid_voxels_mark[index] == 1) {
			output[writing_location[index]] = candidate_voxel_id[index];
		}
	}
}

/* Write the number of valid voxel per point to the output buffer */
extern "C" __global__ void collectValidVoxelCount(int *input_valid_voxel_count, int *output_valid_voxel_count, int *writing_location, int points_num)
{
	for (int id = threadIdx.x + blockIdx.x * blockDim.x; id < points_num; id += blockDim.x * gridDim.x) {
		if (input_valid_voxel_count[id] != 0)
			output_valid_voxel_count[writing_location[id]] = input_valid_voxel_count[id];
	}
}

template <typename T>
void GVoxelGrid::ExclusiveScan(T *input, int ele_num, T *sum)
{
	thrust::device_ptr<T> dev_ptr(input);

	thrust::exclusive_scan(dev_ptr, dev_ptr + ele_num, dev_ptr);
	checkCudaErrors(cudaDeviceSynchronize());

	*sum = *(dev_ptr + ele_num - 1);
}

template <typename T>
void GVoxelGrid::ExclusiveScan(T *input, int ele_num)
{
	thrust::device_ptr<T> dev_ptr(input);

	thrust::exclusive_scan(dev_ptr, dev_ptr + ele_num, dev_ptr);
	checkCudaErrors(cudaDeviceSynchronize());
}

void GVoxelGrid::radiusSearch(float *qx, float *qy, float *qz, int points_num, float radius, int max_nn,
										int **valid_points, int **starting_voxel_id, int **valid_voxel_id,
										int *valid_voxel_num, int *valid_points_num)
{
	//Testing input query points
	int block_x = (points_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : points_num;
	int grid_x = (points_num - 1) / block_x + 1;

	//Boundaries of candidate voxels per points
	int *max_vid_x, *max_vid_y, *max_vid_z;
	int *min_vid_x, *min_vid_y, *min_vid_z;

	checkCudaErrors(cudaMalloc(&max_vid_x, sizeof(int) * points_num));
	checkCudaErrors(cudaMalloc(&max_vid_y, sizeof(int) * points_num));
	checkCudaErrors(cudaMalloc(&max_vid_z, sizeof(int) * points_num));

	checkCudaErrors(cudaMalloc(&min_vid_x, sizeof(int) * points_num));
	checkCudaErrors(cudaMalloc(&min_vid_y, sizeof(int) * points_num));
	checkCudaErrors(cudaMalloc(&min_vid_z, sizeof(int) * points_num));

	//Determine the number of candidate voxel per points
	int *candidate_voxel_num_per_point;
	int total_candidate_voxel_num;

	checkCudaErrors(cudaMalloc(&candidate_voxel_num_per_point, sizeof(int) * (points_num + 1)));

	findBoundariesOfCandidateVoxels<<<grid_x, block_x>>>(qx, qy, qz, radius, points_num,
															voxel_x_, voxel_y_, voxel_z_,
															max_b_x_, max_b_y_, max_b_z_,
															min_b_x_, min_b_y_, min_b_z_,
															max_vid_x, max_vid_y, max_vid_z,
															min_vid_x, min_vid_y, min_vid_z,
															candidate_voxel_num_per_point);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());
	//Total candidate voxel num is determined by an exclusive scan on candidate_voxel_num_per_point
	ExclusiveScan(candidate_voxel_num_per_point, points_num + 1, &total_candidate_voxel_num);

	if (total_candidate_voxel_num <= 0) {
		std::cout << "No candidate voxel was found. Exiting..." << std::endl;

		checkCudaErrors(cudaFree(max_vid_x));
		checkCudaErrors(cudaFree(max_vid_y));
		checkCudaErrors(cudaFree(max_vid_z));

		checkCudaErrors(cudaFree(min_vid_x));
		checkCudaErrors(cudaFree(min_vid_y));
		checkCudaErrors(cudaFree(min_vid_z));

		checkCudaErrors(cudaFree(candidate_voxel_num_per_point));

		valid_points = NULL;
		starting_voxel_id = NULL;
		valid_voxel_id = NULL;

		*valid_voxel_num = 0;
		*valid_points_num = 0;

		return;
	}


	//Determine the voxel id of candidate voxels
	int *candidate_voxel_id;

	checkCudaErrors(cudaMalloc(&candidate_voxel_id, sizeof(int) * total_candidate_voxel_num));

	updateCandidateVoxelIds<<<grid_x, block_x>>>(points_num, vgrid_x_, vgrid_y_, vgrid_z_,
													max_vid_x, max_vid_y, max_vid_z,
													min_vid_x, min_vid_y, min_vid_z,
													candidate_voxel_num_per_point, candidate_voxel_id);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	//Go through the candidate voxel id list and find out which voxels are really inside the radius
	int *valid_voxel_mark;

	checkCudaErrors(cudaMalloc(&valid_voxel_mark, sizeof(int) * total_candidate_voxel_num));

	int *valid_voxel_count;

	checkCudaErrors(cudaMalloc(&valid_voxel_count, sizeof(int) * (points_num + 1)));

	int *valid_points_mark;

	checkCudaErrors(cudaMalloc(&valid_points_mark, sizeof(int) * points_num));

	block_x = (total_candidate_voxel_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : total_candidate_voxel_num;
	grid_x = (total_candidate_voxel_num - 1) / block_x + 1;

	///CHECK VALID VOXEL COUNT AGAIN
	inspectCandidateVoxels<<<grid_x, block_x>>>(qx, qy, qz, radius, max_nn, points_num,
													centroid_, points_per_voxel_, voxel_num_,
													candidate_voxel_num_per_point, candidate_voxel_id,
													valid_voxel_mark, valid_voxel_count, valid_points_mark);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	//Collect valid points
	int *valid_points_location;

	checkCudaErrors(cudaMalloc(&valid_points_location, sizeof(int) * (points_num + 1)));
	checkCudaErrors(cudaMemset(valid_points_location, 0, sizeof(int) * (points_num + 1)));
	checkCudaErrors(cudaMemcpy(valid_points_location, valid_points_mark, sizeof(int) * points_num, cudaMemcpyDeviceToDevice));

	//Writing location to the output buffer is determined by an exclusive scan
	ExclusiveScan(valid_points_location, points_num + 1, valid_points_num);

	if (*valid_points_num <= 0) {
		//std::cout << "No valid point was found. Exiting..." << std::endl;
		std::cout << "No valid point was found. Exiting...: " << *valid_points_num << std::endl;

		checkCudaErrors(cudaFree(max_vid_x));
		checkCudaErrors(cudaFree(max_vid_y));
		checkCudaErrors(cudaFree(max_vid_z));

		checkCudaErrors(cudaFree(min_vid_x));
		checkCudaErrors(cudaFree(min_vid_y));
		checkCudaErrors(cudaFree(min_vid_z));

		checkCudaErrors(cudaFree(candidate_voxel_num_per_point));
		checkCudaErrors(cudaFree(candidate_voxel_id));

		checkCudaErrors(cudaFree(valid_voxel_mark));
		checkCudaErrors(cudaFree(valid_voxel_count));
		checkCudaErrors(cudaFree(valid_points_mark));

		checkCudaErrors(cudaFree(valid_points_location));

		valid_points = NULL;
		starting_voxel_id = NULL;
		valid_voxel_id = NULL;

		*valid_voxel_num = 0;
		*valid_points_num = 0;

		return;
	}

	checkCudaErrors(cudaMalloc(valid_points, sizeof(int) * (*valid_points_num)));

	collectValidPoints<<<grid_x, block_x>>>(valid_points_mark, *valid_points, valid_points_location, points_num);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaMalloc(starting_voxel_id, sizeof(int) * (*valid_points_num + 1)));

	collectValidVoxelCount<<<grid_x, block_x>>>(valid_voxel_count, *starting_voxel_id, valid_points_location, points_num);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	//Determine the starting location of voxels per points in the valid points list
	ExclusiveScan(*starting_voxel_id, *valid_points_num + 1, valid_voxel_num);

	//Collect valid voxels
	int *valid_voxel_location;

	checkCudaErrors(cudaMalloc(&valid_voxel_location, sizeof(int) * (total_candidate_voxel_num + 1)));
	checkCudaErrors(cudaMemcpy(valid_voxel_location, valid_voxel_mark, sizeof(int) * total_candidate_voxel_num, cudaMemcpyDeviceToDevice));

	ExclusiveScan(valid_voxel_location, total_candidate_voxel_num + 1, valid_voxel_num);

	if (*valid_voxel_num <= 0) {
		checkCudaErrors(cudaFree(max_vid_x));
		max_vid_x = NULL;
		checkCudaErrors(cudaFree(max_vid_y));
		max_vid_y = NULL;
		checkCudaErrors(cudaFree(max_vid_z));
		max_vid_z = NULL;

		checkCudaErrors(cudaFree(min_vid_x));
		min_vid_x = NULL;
		checkCudaErrors(cudaFree(min_vid_y));
		min_vid_y = NULL;
		checkCudaErrors(cudaFree(min_vid_z));
		min_vid_z = NULL;

		checkCudaErrors(cudaFree(candidate_voxel_num_per_point));
		candidate_voxel_num_per_point = NULL;
		checkCudaErrors(cudaFree(candidate_voxel_id));
		candidate_voxel_id = NULL;

		checkCudaErrors(cudaFree(valid_voxel_mark));
		valid_voxel_mark = NULL;
		checkCudaErrors(cudaFree(valid_voxel_count));
		valid_voxel_count = NULL;
		checkCudaErrors(cudaFree(valid_points_mark));
		valid_points_mark = NULL;

		checkCudaErrors(cudaFree(valid_points_location));
		valid_points_location = NULL;
		checkCudaErrors(cudaFree(valid_voxel_location));
		valid_voxel_location = NULL;

		valid_points = NULL;
		starting_voxel_id = NULL;
		valid_voxel_id = NULL;

		*valid_voxel_num = 0;
		*valid_points_num = 0;
	}

	checkCudaErrors(cudaMalloc(valid_voxel_id, sizeof(int) * (*valid_voxel_num)));

	block_x = (total_candidate_voxel_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : total_candidate_voxel_num;
	grid_x = (total_candidate_voxel_num - 1) / block_x + 1;

	collectValidVoxels<<<grid_x, block_x>>>(valid_voxel_mark, candidate_voxel_id, *valid_voxel_id, valid_voxel_location, total_candidate_voxel_num);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaFree(max_vid_x));
	checkCudaErrors(cudaFree(max_vid_y));
	checkCudaErrors(cudaFree(max_vid_z));

	checkCudaErrors(cudaFree(min_vid_x));
	checkCudaErrors(cudaFree(min_vid_y));
	checkCudaErrors(cudaFree(min_vid_z));

	checkCudaErrors(cudaFree(candidate_voxel_num_per_point));
	checkCudaErrors(cudaFree(candidate_voxel_id));

	checkCudaErrors(cudaFree(valid_voxel_mark));
	checkCudaErrors(cudaFree(valid_points_mark));
	checkCudaErrors(cudaFree(valid_voxel_count));

	checkCudaErrors(cudaFree(valid_points_location));
	checkCudaErrors(cudaFree(valid_voxel_location));
}

/* Build parent nodes from child nodes of the octree */
extern "C" __global__ void buildParent(double *child_centroids, int *points_per_child,
										int child_grid_x, int child_grid_y, int child_grid_z, int child_num,
										double *parent_centroids, int *points_per_parent,
										int parent_grid_x, int parent_grid_y, int parent_grid_z)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int idy = threadIdx.y + blockIdx.y * blockDim.y;
	int idz = threadIdx.z + blockIdx.z * blockDim.z;

	if (idx < parent_grid_x && idy < parent_grid_y && idz < parent_grid_z) {
		int parent_idx = idx + idy * parent_grid_x + idz * parent_grid_x * parent_grid_y;
		MatrixDevice parent_centr(3, 1, parent_grid_x * parent_grid_y * parent_grid_z, parent_centroids + parent_idx);
		double pc0, pc1, pc2;
		int points_num = 0;
		double dpoints_num;

		pc0 = 0.0;
		pc1 = 0.0;
		pc2 = 0.0;

		for (int i = idx * 2; i < idx * 2 + 2 && i < child_grid_x; i++) {
			for (int j = idy * 2; j < idy * 2 + 2 && j < child_grid_y; j++) {
				for (int k = idz * 2; k < idz * 2 + 2 && k < child_grid_z; k++) {
					int child_idx = i + j * child_grid_x + k * child_grid_x * child_grid_y;
					MatrixDevice child_centr(3, 1, child_num, child_centroids + child_idx);
					int child_points = points_per_child[child_idx];
					double dchild_points = static_cast<double>(child_points);


					pc0 += (child_points > 0) ? dchild_points * child_centr(0) : 0.0;
					pc1 += (child_points > 0) ? dchild_points * child_centr(1) : 0.0;
					pc2 += (child_points > 0) ? dchild_points * child_centr(2) : 0.0;
					points_num += (child_points > 0) ? child_points : 0;

					__syncthreads();
				}
			}
		}

		dpoints_num = static_cast<double>(points_num);

		parent_centr(0) = (points_num <= 0) ? DBL_MAX : pc0 / dpoints_num;
		parent_centr(1) = (points_num <= 0) ? DBL_MAX : pc1 / dpoints_num;
		parent_centr(2) = (points_num <= 0) ? DBL_MAX : pc2 / dpoints_num;
		points_per_parent[parent_idx] = points_num;
	}
}

/* Compute the number of points per voxel using atomicAdd */
extern "C"  __global__ void insertPointsToGrid(float *x, float *y, float *z, int points_num,
												int *points_per_voxel, int voxel_num,
												int vgrid_x, int vgrid_y, int vgrid_z,
												float voxel_x, float voxel_y, float voxel_z,
												int min_b_x, int min_b_y, int min_b_z)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = index; i < points_num; i += stride) {
		float t_x = x[i];
		float t_y = y[i];
		float t_z = z[i];
		int voxel_id = voxelId(t_x, t_y, t_z, voxel_x, voxel_y, voxel_z, min_b_x, min_b_y, min_b_z, vgrid_x, vgrid_y, vgrid_z);

		// Update number of points in the voxel
		int ptr_increment = (voxel_id < voxel_num) * voxel_id; // if (voxel_id < voxel_num), then use voxel_id
		int incremental_value = (voxel_id < voxel_num);
		//atomicAdd(points_per_voxel + voxel_id, 1);
		atomicAdd(points_per_voxel + ptr_increment, incremental_value);
	}
}

/* Rearrange points to locations corresponding to voxels */
extern "C" __global__ void scatterPointsToVoxels(float *x, float *y, float *z, int points_num, int voxel_num,
													float voxel_x, float voxel_y, float voxel_z,
													int min_b_x, int min_b_y, int min_b_z,
													int vgrid_x, int vgrid_y, int vgrid_z,
													int *writing_locations, int *point_ids)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = idx; i < points_num; i += stride) {
		int voxel_id = voxelId(x[i], y[i], z[i], voxel_x, voxel_y, voxel_z,
								min_b_x, min_b_y, min_b_z, vgrid_x, vgrid_y, vgrid_z);

		int ptr_increment = (voxel_id < voxel_num) * voxel_id;
		int incremental_value = (voxel_id < voxel_num);
		//int loc = atomicAdd(writing_locations + voxel_id, 1);
		int loc =  atomicAdd(writing_locations + ptr_increment, incremental_value);

		point_ids[loc] = i;
	}
}

void GVoxelGrid::scatterPointsToVoxelGrid()
{
	if (starting_point_ids_ != NULL) {
		checkCudaErrors(cudaFree(starting_point_ids_));
		starting_point_ids_ = NULL;
	}

	if (point_ids_ != NULL) {
		checkCudaErrors(cudaFree(point_ids_));
		point_ids_ = NULL;
	}

	int block_x = (points_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : points_num_;
	int grid_x = (points_num_ - 1) / block_x + 1;

	insertPointsToGrid<<<grid_x, block_x>>>(x_, y_, z_, points_num_, points_per_voxel_, voxel_num_,
												vgrid_x_, vgrid_y_, vgrid_z_,
												voxel_x_, voxel_y_, voxel_z_,
												min_b_x_, min_b_y_, min_b_z_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaMalloc(&starting_point_ids_, sizeof(int) * (voxel_num_ + 1)));

	int *writing_location;

	checkCudaErrors(cudaMalloc(&writing_location, sizeof(int) * voxel_num_));

	checkCudaErrors(cudaMemcpy(starting_point_ids_, points_per_voxel_, sizeof(int) * voxel_num_, cudaMemcpyDeviceToDevice));

	ExclusiveScan(starting_point_ids_, voxel_num_ + 1);

	checkCudaErrors(cudaMemcpy(writing_location, starting_point_ids_, sizeof(int) * voxel_num_, cudaMemcpyDeviceToDevice));

	checkCudaErrors(cudaMalloc(&point_ids_, sizeof(int) * points_num_));

	scatterPointsToVoxels<<<grid_x, block_x>>>(x_, y_, z_, points_num_, voxel_num_,
												voxel_x_, voxel_y_, voxel_z_,
												min_b_x_, min_b_y_, min_b_z_,
												vgrid_x_, vgrid_y_, vgrid_z_,
												writing_location, point_ids_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaFree(writing_location));
}

void GVoxelGrid::buildOctree()
{
	for (unsigned int i = 1; i < octree_centroids_.size(); i++) {
		if (octree_centroids_[i] != NULL) {
			checkCudaErrors(cudaFree(octree_centroids_[i]));
			octree_centroids_[i] = NULL;
		}
		if (octree_points_per_node_[i] != NULL) {
			checkCudaErrors(cudaFree(octree_points_per_node_[i]));
			octree_points_per_node_[i] = NULL;
		}
	}

	octree_centroids_.clear();
	octree_points_per_node_.clear();
	octree_grid_size_.clear();

	//Push leafs to the octree list
	octree_centroids_.push_back(centroid_);
	octree_points_per_node_.push_back(points_per_voxel_);
	OctreeGridSize grid_size;

	grid_size.size_x = vgrid_x_;
	grid_size.size_y = vgrid_y_;
	grid_size.size_z = vgrid_z_;

	octree_grid_size_.push_back(grid_size);

	int node_number = voxel_num_;
	int child_grid_x, child_grid_y, child_grid_z;
	int parent_grid_x, parent_grid_y, parent_grid_z;

	int i = 0;

	while (node_number > 8) {
		child_grid_x = octree_grid_size_[i].size_x;
		child_grid_y = octree_grid_size_[i].size_y;
		child_grid_z = octree_grid_size_[i].size_z;

		parent_grid_x = (child_grid_x - 1) / 2 + 1;
		parent_grid_y = (child_grid_y - 1) / 2 + 1;
		parent_grid_z = (child_grid_z - 1) / 2 + 1;

		node_number = parent_grid_x * parent_grid_y * parent_grid_z;

		double *parent_centroids;
		int *points_per_parent;

		checkCudaErrors(cudaMalloc(&parent_centroids, sizeof(double) * 3 * node_number));
		checkCudaErrors(cudaMalloc(&points_per_parent, sizeof(int) * node_number));

		double *child_centroids = octree_centroids_[i];
		int *points_per_child = octree_points_per_node_[i];

		int block_x = (parent_grid_x > BLOCK_X) ? BLOCK_X : parent_grid_x;
		int block_y = (parent_grid_y > BLOCK_Y) ? BLOCK_Y : parent_grid_y;
		int block_z = (parent_grid_z > BLOCK_Z) ? BLOCK_Z : parent_grid_z;

		int grid_x = (parent_grid_x - 1) / block_x + 1;
		int grid_y = (parent_grid_y - 1) / block_y + 1;
		int grid_z = (parent_grid_z - 1) / block_z + 1;

		dim3 block(block_x, block_y, block_z);
		dim3 grid(grid_x, grid_y, grid_z);

		buildParent<<<grid, block>>>(child_centroids, points_per_child,
										child_grid_x, child_grid_y, child_grid_z, child_grid_x * child_grid_y * child_grid_z,
										parent_centroids, points_per_parent,
										parent_grid_x, parent_grid_y, parent_grid_z);
		checkCudaErrors(cudaGetLastError());
		octree_centroids_.push_back(parent_centroids);
		octree_points_per_node_.push_back(points_per_parent);

		grid_size.size_x = parent_grid_x;
		grid_size.size_y = parent_grid_y;
		grid_size.size_z = parent_grid_z;

		octree_grid_size_.push_back(grid_size);

		i++;
	}

	checkCudaErrors(cudaDeviceSynchronize());
}

/* Search for the nearest octree node */
extern "C" __global__ void nearestOctreeNodeSearch(float *x, float *y, float *z,
													int *vid_x, int *vid_y, int *vid_z,
													int points_num,
													double *centroids, int *points_per_node,
													int vgrid_x, int vgrid_y, int vgrid_z, int node_num)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = idx; i < points_num; i += stride) {
		int vx = vid_x[i];
		int vy = vid_y[i];
		int vz = vid_z[i];
		double min_dist = DBL_MAX;
		double t_x = static_cast<double>(x[i]);
		double t_y = static_cast<double>(y[i]);
		double t_z = static_cast<double>(z[i]);
		double cur_dist;

		int out_x, out_y, out_z;

		out_x = vx;
		out_y = vy;
		out_z = vz;

		double tmp_x, tmp_y, tmp_z;

		for (int j = vx * 2; j < vx * 2 + 2 && j < vgrid_x; j++) {
			for (int k = vy * 2; k < vy * 2 + 2 && k < vgrid_y; k++) {
				for (int l = vz * 2; l < vz * 2 + 2 && l < vgrid_z; l++) {
					int node_id = j + k * vgrid_x + l * vgrid_x * vgrid_y;
					MatrixDevice node_centr(3, 1, node_num, centroids + node_id);
					int points = points_per_node[node_id];

					tmp_x = (points > 0) ? node_centr(0) - t_x : DBL_MAX;
					tmp_y = (points > 0) ? node_centr(1) - t_y : 0.0;
					tmp_z = (points > 0) ? node_centr(2) - t_z : 0.0;

					cur_dist = norm3d(tmp_x, tmp_y, tmp_z);
					bool res = (cur_dist < min_dist);

					out_x = (res) ? j : out_x;
					out_y = (res) ? k : out_y;
					out_z = (res) ? l : out_z;

					min_dist = (res) ? cur_dist : min_dist;
				}
			}
		}

		vid_x[i] = out_x;
		vid_y[i] = out_y;
		vid_z[i] = out_z;
	}
}

/* Search for the nearest point from nearest voxel */
extern "C" __global__ void nearestPointSearch(float *qx, float *qy, float *qz, int qpoints_num,
												float *rx, float *ry, float *rz, int rpoints_num,
												int *vid_x, int *vid_y, int *vid_z,
												int vgrid_x, int vgrid_y, int vgrid_z, int voxel_num,
												int *starting_point_id, int *point_id, double *min_distance)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = idx; i < qpoints_num; i += stride) {
		int voxel_id = vid_x[i] + vid_y[i] * vgrid_x + vid_z[i] * vgrid_x * vgrid_y;
		float cor_qx = qx[i];
		float cor_qy = qy[i];
		float cor_qz = qz[i];
		float min_dist = FLT_MAX;

		for (int j = starting_point_id[voxel_id]; j < starting_point_id[voxel_id + 1]; j++) {
			int pid = point_id[j];
			float cor_rx = rx[pid];
			float cor_ry = ry[pid];
			float cor_rz = rz[pid];

			cor_rx -= cor_qx;
			cor_ry -= cor_qy;
			cor_rz -= cor_qz;

			min_dist = fminf(norm3df(cor_rx, cor_ry, cor_rz), min_dist);
		}

		min_distance[i] = static_cast<double>(min_dist);
	}
}

/* Verify if min distances are really smaller than or equal to max_range */
extern "C" __global__ void verifyDistances(int *valid_distance, double *min_distance, double max_range, int points_num)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = idx; i < points_num; i += stride) {
		bool check = (min_distance[i] <= max_range);

		valid_distance[i] = (check) ? 1 : 0;

		if (!check) {
			min_distance[i] = 0;
		}
	}
}

void GVoxelGrid::nearestNeighborSearch(float *trans_x, float *trans_y, float *trans_z, int point_num, int *valid_distance, double *min_distance, float max_range)
{

	int *vid_x, *vid_y, *vid_z;

	checkCudaErrors(cudaMalloc(&vid_x, sizeof(int) * point_num));
	checkCudaErrors(cudaMalloc(&vid_y, sizeof(int) * point_num));
	checkCudaErrors(cudaMalloc(&vid_z, sizeof(int) * point_num));

	checkCudaErrors(cudaMemset(vid_x, 0, sizeof(int) * point_num));
	checkCudaErrors(cudaMemset(vid_y, 0, sizeof(int) * point_num));
	checkCudaErrors(cudaMemset(vid_z, 0, sizeof(int) * point_num));
	checkCudaErrors(cudaDeviceSynchronize());

	int block_x = (point_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : point_num;
	int grid_x = (point_num - 1) / block_x + 1;

	// Go through top of the octree to the bottom
	for (int i = octree_centroids_.size() - 1; i >= 0; i--) {
		double *centroids = octree_centroids_[i];
		int *points_per_node = octree_points_per_node_[i];
		int vgrid_x = octree_grid_size_[i].size_x;
		int vgrid_y = octree_grid_size_[i].size_y;
		int vgrid_z = octree_grid_size_[i].size_z;
		int node_num = vgrid_x * vgrid_y * vgrid_z;

		nearestOctreeNodeSearch<<<grid_x, block_x>>>(trans_x, trans_y, trans_z,
														vid_x, vid_y, vid_z,
														point_num,
														centroids, points_per_node,
														vgrid_x, vgrid_y, vgrid_z, node_num);
		checkCudaErrors(cudaGetLastError());
	}

	nearestPointSearch<<<grid_x, block_x>>>(trans_x, trans_y, trans_z, point_num,
												x_, y_, z_, points_num_,
												vid_x, vid_y, vid_z,
												vgrid_x_, vgrid_y_, vgrid_z_, voxel_num_,
												starting_point_ids_, point_ids_,
												min_distance);
	checkCudaErrors(cudaGetLastError());

	verifyDistances<<<grid_x, block_x>>>(valid_distance, min_distance, max_range, point_num);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaFree(vid_x));
	checkCudaErrors(cudaFree(vid_y));
	checkCudaErrors(cudaFree(vid_z));
}

}
