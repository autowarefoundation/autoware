#ifndef GPU_OCTREE_H_
#define GPU_OCTREE_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "common.h"
#include "MatrixDevice.h"
#include "MatrixHost.h"
#include <float.h>
#include <vector>

namespace gpu {
class GVoxelGrid {
public:
	GVoxelGrid();

	GVoxelGrid(const GVoxelGrid &other);

	/* Set input points */
	void setInput(float *x, float *y, float *z, int points_num);

	void setMinVoxelSize(int size);

	/* For each input point, search for voxels whose distance between their centroids and
	 * the input point are less than radius.
	 * Results of the search are stored into valid_points, starting_voxel_id, and voxel_id.
	 * Valid points: the function return one or more voxels for these points. Other points
	 * are considered as invalid.
	 * Valid voxels: voxels returned from the search.
	 * The number of valid points is stored in valid_points_num.
	 * The total number of valid voxels. If the query of both point A and point B return
	 * voxel X, then the number of valid voxels is 2, not 1. */
	void radiusSearch(float *qx, float *qy, float *qz, int points_num, float radius, int max_nn,
											int **valid_points, int **starting_voxel_id, int **voxel_id,
											int *valid_voxel_num, int *valid_points_num);

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

	/* Get the centroid list. */
	double *getCentroidList() const;

	/* Get the covariance list. */
	double *getCovarianceList() const;

	/* Get the pointer to the inverse covariances list. */
	double *getInverseCovarianceList() const;

	int *getPointsPerVoxelList() const;

	/* Searching for the nearest point of each input query point.
	 * Coordinates of query points are input by trans_x, trans_y, and trans_z.
	 * The ith element of valid_distance array is 1 if the distance between
	 * the ith input point and its nearest neighbor is less than or equal
	 * to max_range. Otherwise, it is 0.
	 * The ith element of min_distance array stores the distance between
	 * the corresponding input point and its nearest neighbor. It is 0 if
	 * the distance is larger than max_range. */
	void nearestNeighborSearch(float *trans_x, float *trans_y, float *trans_z, int point_num, int *valid_distance, double *min_distance, float max_range);

	~GVoxelGrid();
private:

	/* Construct the voxel grid and the build the octree. */
	void initialize();

	/* Compute centroids and covariances of voxels. */
	void computeCentroidAndCovariance();

	/* Find boundaries of input point cloud and compute
	 * the number of necessary voxels as well as boundaries
	 * measured in number of leaf size */
	void findBoundaries();

	/* Put points into voxels */
	void scatterPointsToVoxelGrid();

	/* Build octrees for nearest neighbor search.
	 * Only used for searching one nearest neighbor point.
	 * Cannot used for searching multiple nearest neighbors. */
	void buildOctree();


	/* A wrapper for exclusive scan using thrust.
	 * Output sum of all elements is stored at sum. */
	template <typename T = int>
	void ExclusiveScan(T *input, int ele_num, T *sum);

	/* A wrapper for exclusive scan using thrust.
	 * Output sum is not required. */
	template <typename T = int>
	void ExclusiveScan(T *input, int ele_num);

	/* Size of the octree in each level,
	 * measured in number of tree nodes. */
	typedef struct _OctreeGridSize {
		int size_x;
		int size_y;
		int size_z;
	} OctreeGridSize;

	//Coordinate of input points
	float *x_, *y_, *z_;
	int points_num_;
	double *centroid_; 				// List of 3x1 double vector
	double *covariance_;			// List of 3x3 double matrix
	double *inverse_covariance_;	// List of 3x3 double matrix
	int *points_per_voxel_;

	int voxel_num_;						// Number of voxels
	float max_x_, max_y_, max_z_;		// Upper bounds of the grid (maximum coordinate)
	float min_x_, min_y_, min_z_;		// Lower bounds of the grid (minimum coordinate)
	float voxel_x_, voxel_y_, voxel_z_;	// Leaf size, a.k.a, size of each voxel

	int max_b_x_, max_b_y_, max_b_z_;	// Upper bounds of the grid, measured in number of voxels
	int min_b_x_, min_b_y_, min_b_z_;	// Lower bounds of the grid, measured in number of voxels
	int vgrid_x_, vgrid_y_, vgrid_z_;	// Size of the voxel grid, measured in number of voxels
	int min_points_per_voxel_;

	int *starting_point_ids_;
	int *point_ids_;

	/* Centroids of octree nodes. Each element stores a list
	 * of 3x1 matrices containing centroids of octree nodes. */
	std::vector<double*> octree_centroids_;

	/* The number of points per octree node per level. */
	std::vector<int*> octree_points_per_node_;

	/* The number of octree nodes per level in three dimensions. */
	std::vector<OctreeGridSize> octree_grid_size_;

	bool is_copied_;
};
}

#endif
