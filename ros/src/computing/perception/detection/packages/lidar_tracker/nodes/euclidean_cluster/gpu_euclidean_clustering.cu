/*
 ============================================================================
 Name        : gpu_euclidean_clustering.cu
 Author      : AnhNV91
 Version     : 1.0
 Description : Clustering analysis using Euclidean distance and single linkage
 ============================================================================
 */

#include "gpu_euclidean_clustering.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/scan.h>
#include <thrust/fill.h>

#define MAX_SHARED_SIZE 2048
#define BLOCK_SIZE_X 1024

inline void gassert(cudaError_t err_code, const char *file, int line)
{
	if(err_code!= cudaSuccess)
	{
		fprintf(stderr, "Error: %s %s %d\n", cudaGetErrorString(err_code), file, line);
		cudaDeviceReset();
		exit(EXIT_FAILURE);
	}
}

#define checkCudaErrors(val) gassert(val, __FILE__, __LINE__)

GpuEuclideanCluster::GpuEuclideanCluster()
{
	x_ = NULL;
	y_ = NULL;
	z_ = NULL;

	size_ = 0;
	threshold_ = 0;
	cluster_indices_ = NULL;
	cluster_indices_host_ = NULL;
	min_cluster_pts_ = max_cluster_pts_ = 0;
}

void GpuEuclideanCluster::setInputPoints(float *x, float *y, float *z, int size)
{
	size_ = size;
	checkCudaErrors(cudaMalloc(&x_, size_ * sizeof(float)));
	checkCudaErrors(cudaMalloc(&y_, size_ * sizeof(float)));
	checkCudaErrors(cudaMalloc(&z_, size_ * sizeof(float)));

	checkCudaErrors(cudaMemcpy(x_, x, size_ * sizeof(float), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(y_, y, size_ * sizeof(float), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(z_, z, size_ * sizeof(float), cudaMemcpyHostToDevice));

	checkCudaErrors(cudaMalloc(&cluster_indices_, size_ * sizeof(int)));
	cluster_indices_host_ = (int*)malloc(size_ * sizeof(int));
}

void GpuEuclideanCluster::setThreshold(double threshold)
{
	threshold_ = threshold;
}

void GpuEuclideanCluster::setMinClusterPts(int min_cluster_pts)
{
	min_cluster_pts_ = min_cluster_pts;
}

void GpuEuclideanCluster::setMaxClusterPts(int max_cluster_pts)
{
	max_cluster_pts_ = max_cluster_pts;
}

/* Initially, each point is assigned to an individual cluster.
 *
 */

extern "C" __global__ void pclEuclideanInitialize(int *cluster_indices, int size)
{
	for (int index = threadIdx.x + blockIdx.x * blockDim.x; index < size; index += blockDim.x * gridDim.x)
		cluster_indices[index] = index;
}

/* Connected component labeling points at GPU block thread level.
 * Input list of points is divided into multiple smaller groups.
 * Each group of point is assigned to a block of GPU thread.
 * Each thread in a block handles one point in the group. It iterates over
 * points in the group and compare the distance between the current point A
 * and the point B it has to handle.
 *
 * If the distance between A and B is less than the threshold, then those
 * two points belong to a same connected component and the cluster_changed
 * is marked by 1.
 *
 * A synchronization is called to make sure all thread in the block finish A
 * before moving to the update phase.
 * After finishing checking cluster_changed, threads update the cluster
 * index of all points. If a thread has cluster_changed is 1, then the corresponding
 * cluster of the point it is handling is changed to the cluster of B. Otherwise
 * the original cluster of A remains unchanged.
 *
 * Another synchronization is called before all threads in the block move to
 * other points after done checking A.
 *
 * After this kernel finishes, all points in each block are labeled.
 */
extern "C" __global__ void blockLabelling(float *x, float *y, float *z, int *cluster_indices, int size, float threshold)
{
	int block_start = blockIdx.x * blockDim.x;
	int block_end = (block_start + blockDim.x <= size) ? (block_start + blockDim.x) : size;
	int row = threadIdx.x + block_start;
	__shared__ int local_offset[BLOCK_SIZE_X];
	__shared__ float local_x[BLOCK_SIZE_X];
	__shared__ float local_y[BLOCK_SIZE_X];
	__shared__ float local_z[BLOCK_SIZE_X];
	__shared__ int local_cluster_changed[BLOCK_SIZE_X];

	if (row < block_end) {
		local_offset[threadIdx.x] = threadIdx.x;
		local_x[threadIdx.x] = x[row];
		local_y[threadIdx.x] = y[row];
		local_z[threadIdx.x] = z[row];
		__syncthreads();

		for (int column = block_start; column < block_end; column++) {
			float tmp_x = local_x[threadIdx.x] - local_x[column - block_start];
			float tmp_y = local_y[threadIdx.x] - local_y[column - block_start];
			float tmp_z = local_z[threadIdx.x] - local_z[column - block_start];
			int column_offset = local_offset[column - block_start];
			int row_offset = local_offset[threadIdx.x];

			local_cluster_changed[threadIdx.x] = 0;
			__syncthreads();

			if (row > column && column_offset != row_offset && norm3df(tmp_x, tmp_y, tmp_z) < threshold)
				local_cluster_changed[row_offset] = 1;
			__syncthreads();

			local_offset[threadIdx.x] = (local_cluster_changed[row_offset] == 1) ? column_offset : row_offset;
			__syncthreads();
		}

		__syncthreads();

		int new_cluster = cluster_indices[block_start + local_offset[threadIdx.x]];

		__syncthreads();

		cluster_indices[row] = new_cluster;
	}
}

/* These kernels are used to collect remained clusters after each labeling phase.
 *
 * Basically, in each labeling phases, several clusters are merged together.
 *
 * The first kernel scans over the cluster_indices array and marks the cluster_mark
 * element corresponding with the cluster of the current point by 1. If a cluster
 * does not exists in the current phase (which means it was merged to some other
 * clusters), then its cluster_mark is 0.
 *
 * The second kernel scans over the original cluster_indices again and copy those
 * indices to new location on the target_clusters.
 */
extern "C" __global__ void clusterMark(int *cluster_list, int *cluster_mark, int size)
{
	for (int i = threadIdx.x + blockIdx.x * blockDim.x; i < size; i += blockDim.x * gridDim.x)
		cluster_mark[cluster_list[i]] = 1;
}

extern "C" __global__ void clusterCollector(int *old_cluster_list, int *new_cluster_list, int *cluster_location, int size)
{
	for (int i = threadIdx.x + blockIdx.x * blockDim.x; i < size; i += blockDim.x * gridDim.x)
		new_cluster_list[cluster_location[old_cluster_list[i]]] = old_cluster_list[i];
}

/* Create a cluster matrix.
 *
 * A cluster matrix is to record the relationship between each pair
 * of clusters. If a pair of cluster x and y are connected, then
 * the matrix element [x][y] are 1. Otherwise it is 0. Notice that
 * only the lower half of the matrix is used.
 *
 * To build this matrix, each GPU thread handles one point A, iterates
 * over all points B, and compare distance between A and B. Assuming
 * that A belongs to a cluster x, and B belongs to cluster y. If their
 * distance is less than the threshold, then the matrix element [x][y]
 * is set to 1.
 */
extern "C" __global__ void buildClusterMatrix(float *x, float *y, float *z, int *cluster_indices, int *cluster_matrix, int *cluster_offset, int size, int cluster_num, float threshold)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ float local_x[BLOCK_SIZE_X];
	__shared__ float local_y[BLOCK_SIZE_X];
	__shared__ float local_z[BLOCK_SIZE_X];

	for (int column = index; column < size; column += stride) {
		local_x[threadIdx.x] = x[column];
		local_y[threadIdx.x] = y[column];
		local_z[threadIdx.x] = z[column];
		int column_cluster = cluster_indices[column];
		int cc_offset = cluster_offset[column_cluster];

		__syncthreads();

		for (int row = column + 1; row < size; row++) {
			float tmp_x = x[row] - local_x[threadIdx.x];
			float tmp_y = y[row] - local_y[threadIdx.x];
			float tmp_z = z[row] - local_z[threadIdx.x];
			int row_cluster = cluster_indices[row];
			int rc_offset = cluster_offset[row_cluster];

			__syncthreads();

			if (row_cluster != column_cluster && norm3df(tmp_x, tmp_y, tmp_z) < threshold)
				cluster_matrix[rc_offset * cluster_num + cc_offset] = 1;
		}

	}
}

/* Merge clusters based on the cluster_matrix.
 *
 * This merge process is done per block. The input list of clusters
 * are divided into smaller chunks to be handled by GPU blocks.
 *
 * Each thread in a block handles one row of the matrix and iterates
 * over all columns of the matrix. A synchronization per each iteration
 * is needed to make sure all threads done merging clusters in the
 * current column before moving to the next column.
 *
 * In each iteration, each thread check if the cluster corresponding
 * with the current row is connected to the cluster corresponding to the
 * current column. If so, then the cluster of the row is changed (merged)
 * to the cluster of the column.
 */
extern "C" __global__ void mergeClusters(int *cluster_matrix, int *cluster_list, int cluster_num)
{
	int block_start = blockIdx.x * blockDim.x;
	int block_end = (block_start + blockDim.x <= cluster_num) ? block_start + blockDim.x : cluster_num;
	int row = block_start + threadIdx.x;
	__shared__ int local_cluster_changed[BLOCK_SIZE_X];
	__shared__ int local_offset[BLOCK_SIZE_X];

	if (row < block_end) {
		local_offset[threadIdx.x] = threadIdx.x;

		__syncthreads();

		for (int column = block_start; column < block_end; column++) {
			int row_offset = local_offset[threadIdx.x];
			int column_offset = local_offset[column - block_start];

			local_cluster_changed[threadIdx.x] = 0;
			__syncthreads();

			if (row > column && row_offset != column_offset && (cluster_matrix[row * cluster_num + column] == 1))
				local_cluster_changed[row_offset] = 1;
			__syncthreads();

			local_offset[threadIdx.x] = (local_cluster_changed[row_offset] == 1) ? column_offset : row_offset;
			__syncthreads();
		}

		__syncthreads();

		int new_cluster = cluster_list[block_start + local_offset[threadIdx.x]];

		__syncthreads();
		cluster_list[row] = new_cluster;
	}
}

/* This kernel is to reflex the change in the cluster merging step
 * to cluster indices of all input points.
 *
 * Clusters of input points are changed to the target clusters
 * corresponding with their source clusters.
 */
extern "C" __global__ void reflexClusterChanges(int *cluster_indices, int *cluster_offset, int *cluster_list, int size)
{
	for (int i = threadIdx.x + blockIdx.x * blockDim.x; i < size; i += blockDim.x * gridDim.x)
		cluster_indices[i] = cluster_list[cluster_offset[cluster_indices[i]]];
}

/* Rebuild cluster matrix after merging clusters.
 *
 * After several cluster are merged together, the number of clusters
 * reduces and the cluster matrix needs to be rebuilt.
 *
 * Each thread iterate over rows of one column of the source matrix.
 * If a element [x][y] of the source matrix is 1, then the element
 * [m][n] of the target matrix, in which m and n are the
 * new clusters of x and y, is set to 1.
 */
extern "C" __global__ void rebuildClusterMatrix(int *old_cluster_matrix, int *new_clusters, int *new_cluster_matrix, int *new_cluster_offset, int old_size, int new_size)
{
	for (int column = threadIdx.x + blockIdx.x * blockDim.x; column < old_size; column += blockDim.x * gridDim.x) {
		for (int row = column + 1; row < old_size; row++) {
			int new_row = new_cluster_offset[new_clusters[row]];
			int new_column = new_cluster_offset[new_clusters[column]];

			if (old_cluster_matrix[row * old_size + column] == 1)
				new_cluster_matrix[new_row * new_size + new_column] = 1;
		}
	}
}

/* Perform exclusive scan on the input array using
 * thurst's scan.
 *
 * The variable 'sum' records the last element of
 * the array after being scanned.
 */
void GpuEuclideanCluster::exclusiveScan(int *input, int ele_num, int *sum)
{
	thrust::device_ptr<int> dev_ptr(input);

	thrust::exclusive_scan(dev_ptr, dev_ptr + ele_num, dev_ptr);
	checkCudaErrors(cudaDeviceSynchronize());

	*sum = *(dev_ptr + ele_num - 1);
}


/* Calculate the cluster indices of input points.
 *
 * Initially, the cluster index of the point at index ith
 * is set to i. This method merges cluster indices
 * of points that belong to same clusters.
 *
 * Result of this method is stored at cluster_indices_host_.
 */
void GpuEuclideanCluster::extractClusters()
{
	int block_x, grid_x;

	block_x = (size_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : size_;
	grid_x = (size_ - 1) / block_x + 1;

	int *cluster_offset;
	int cluster_num, old_cluster_num;

	pclEuclideanInitialize<<<grid_x, block_x>>>(cluster_indices_, size_);
	checkCudaErrors(cudaDeviceSynchronize());

	old_cluster_num = cluster_num = size_;

	checkCudaErrors(cudaMalloc(&cluster_offset, (size_ + 1) * sizeof(int)));
	checkCudaErrors(cudaMemset(cluster_offset, 0, (size_ + 1) * sizeof(int)));
	blockLabelling<<<grid_x, block_x>>>(x_, y_, z_, cluster_indices_, size_, threshold_);
	clusterMark<<<grid_x, block_x>>>(cluster_indices_, cluster_offset, size_);
	exclusiveScan(cluster_offset, size_ + 1, &cluster_num);

	int *cluster_list, *new_cluster_list, *tmp;

	checkCudaErrors(cudaMalloc(&cluster_list, cluster_num * sizeof(int)));
	clusterCollector<<<grid_x, block_x>>>(cluster_indices_, cluster_list, cluster_offset, size_);
	checkCudaErrors(cudaDeviceSynchronize());

	int *cluster_matrix;
	int *new_cluster_matrix;

	checkCudaErrors(cudaMalloc(&cluster_matrix, cluster_num * cluster_num * sizeof(int)));
	checkCudaErrors(cudaMemset(cluster_matrix, 0, cluster_num * cluster_num * sizeof(int)));
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaMalloc(&new_cluster_list, cluster_num * sizeof(int)));

	buildClusterMatrix<<<grid_x, block_x>>>(x_, y_, z_, cluster_indices_, cluster_matrix, cluster_offset, size_, cluster_num, threshold_);
	checkCudaErrors(cudaDeviceSynchronize());

	int block_x2 = 0, grid_x2 = 0;


	/* Loop until there is no change in the number of clusters */
	do {
		old_cluster_num = cluster_num;
		block_x2 = (cluster_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : cluster_num;
		grid_x2 = (cluster_num - 1)/block_x2 + 1;

		mergeClusters<<<grid_x2, block_x2>>>(cluster_matrix, cluster_list, cluster_num);
		reflexClusterChanges<<<grid_x, block_x>>>(cluster_indices_, cluster_offset, cluster_list, size_);
		checkCudaErrors(cudaMemset(cluster_offset, 0, (size_ + 1) * sizeof(int)));
		clusterMark<<<grid_x2, block_x2>>>(cluster_list, cluster_offset, cluster_num);
		exclusiveScan(cluster_offset, size_ + 1, &cluster_num);

		if (grid_x2 == 1 && cluster_num == old_cluster_num)
			break;

		clusterCollector<<<grid_x, block_x2>>>(cluster_list, new_cluster_list, cluster_offset, old_cluster_num);
		checkCudaErrors(cudaDeviceSynchronize());

		checkCudaErrors(cudaMalloc(&new_cluster_matrix, cluster_num * cluster_num * sizeof(int)));
		checkCudaErrors(cudaMemset(new_cluster_matrix, 0, cluster_num * cluster_num * sizeof(int)));
		rebuildClusterMatrix<<<grid_x2, block_x2>>>(cluster_matrix, cluster_list, new_cluster_matrix, cluster_offset, old_cluster_num, cluster_num);
		checkCudaErrors(cudaDeviceSynchronize());

		checkCudaErrors(cudaFree(cluster_matrix));
		cluster_matrix = new_cluster_matrix;
		tmp = cluster_list;
		cluster_list = new_cluster_list;
		new_cluster_list = tmp;
	} while (1);

	checkCudaErrors(cudaMemcpy(cluster_indices_host_, cluster_indices_, size_ * sizeof(int), cudaMemcpyDeviceToHost));


	checkCudaErrors(cudaFree(cluster_matrix));
	checkCudaErrors(cudaFree(cluster_list));
	checkCudaErrors(cudaFree(new_cluster_list));
	checkCudaErrors(cudaFree(cluster_offset));
}


/* Collect points that belong to same clusters and put them together.
 *
 * The output is a vector whose each element contains indexes of points
 * that belong to a same clusters.
 */
std::vector<GpuEuclideanCluster::GClusterIndex> GpuEuclideanCluster::getOutput()
{
	std::vector<GClusterIndex> cluster_indices;

	for (int i = 0; i < size_; i++) {
		bool found = false;

		for (unsigned int j = 0; j < cluster_indices.size(); j++) {
			if (cluster_indices_host_[i] == cluster_indices[j].index_value) {
				(cluster_indices[j].points_in_cluster).push_back(i);
				found = true;
				break;
			}
		}

		if (!found) {
			GClusterIndex new_cluster;

			new_cluster.index_value = cluster_indices_host_[i];
			(new_cluster.points_in_cluster).push_back(i);
			cluster_indices.push_back(new_cluster);
		}
	}

	for (unsigned int i = 0; i < cluster_indices.size(); ) {
		int number_of_pts = cluster_indices[i].points_in_cluster.size();

		if (number_of_pts < min_cluster_pts_ || number_of_pts > max_cluster_pts_)
			cluster_indices.erase(cluster_indices.begin() + i);
		else
			i++;
	}

	//For testing
//	float *x = (float*)malloc(sizeof(float) * size_);
//	float *y = (float*)malloc(sizeof(float) * size_);
//	float *z = (float*)malloc(sizeof(float) * size_);
//
//	cudaMemcpy(x, x_, sizeof(float) * size_, cudaMemcpyDeviceToHost);
//	cudaMemcpy(y, y_, sizeof(float) * size_, cudaMemcpyDeviceToHost);
//	cudaMemcpy(z, z_, sizeof(float) * size_, cudaMemcpyDeviceToHost);
//
//	for (int i = 0; i < cluster_indices.size(); i++) {
//		for (int j = 0; j < cluster_indices[i].points_in_cluster.size(); j++) {
//			for (int k = i + 1; k < cluster_indices.size(); k++) {
//				for (int p = 0; p < cluster_indices[k].points_in_cluster.size(); p++) {
//					int left = cluster_indices[i].points_in_cluster[j];
//					int right = cluster_indices[k].points_in_cluster[p];
//					float tmp_x = x[left] - x[right];
//					float tmp_y = y[left] - y[right];
//					float tmp_z = z[left] - z[right];
//
//					if (sqrt(tmp_x * tmp_x + tmp_y * tmp_y + tmp_z * tmp_z) < threshold_)
//						std::cout << "Cluster " << i << " and " << k << " are the same" << std::cout;
//				}
//			}
//		}
//	}
//
//	free(x);
//	free(y);
//	free(z);

	//std::cout << "Number of clusters in GPU is****************** " << cluster_indices.size() << std::endl;

	return cluster_indices;
}

GpuEuclideanCluster::~GpuEuclideanCluster()
{
	checkCudaErrors(cudaFree(x_));
	checkCudaErrors(cudaFree(y_));
	checkCudaErrors(cudaFree(z_));
	checkCudaErrors(cudaFree(cluster_indices_));
	free(cluster_indices_host_);
}

