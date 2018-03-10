#ifndef CPU_VGRID_H_
#define CPU_VGRID_H_

#include <velodyne_pointcloud/point_types.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <float.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

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
	 * If the distance is larger than max_range, then return 0. */
	double nearestNeighborDistance(PointSourceType query_point, float max_range);


	Eigen::Vector3d getCentroid(int voxel_id) const;
	Eigen::Matrix3d getCovariance(int voxel_id) const;
	Eigen::Matrix3d getInverseCovariance(int voxel_id) const;

private:

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

	/* Size of the octree in each level,
	 * measured in number of tree nodes. */
	typedef struct _OctreeGridSize {
		int size_x;
		int size_y;
		int size_z;
	} OctreeGridSize;

	/* Build octrees for nearest neighbor search.
	 * Only used for searching one nearest neighbor point.
	 * Cannot used for searching multiple nearest neighbors. */
	void buildOctree();

	int voxelId(PointSourceType p);

	void buildParent(std::vector<Eigen::Vector3d> &child_centroids, std::vector<int> &points_per_child, OctreeGridSize child_size,
						std::vector<Eigen::Vector3d> &parent_centroids, std::vector<int> &points_per_parent, OctreeGridSize parent_size);

	void nearestOctreeNodeSearch(PointSourceType q, Eigen::Vector3d &node_id, int tree_level);

	//Coordinate of input points
	typename pcl::PointCloud<PointSourceType>::Ptr source_cloud_;

	int voxel_num_;						// Number of voxels
	float max_x_, max_y_, max_z_;		// Upper bounds of the grid (maximum coordinate)
	float min_x_, min_y_, min_z_;		// Lower bounds of the grid (minimum coordinate)
	float voxel_x_, voxel_y_, voxel_z_;	// Leaf size, a.k.a, size of each voxel

	int max_b_x_, max_b_y_, max_b_z_;	// Upper bounds of the grid, measured in number of voxels
	int min_b_x_, min_b_y_, min_b_z_;	// Lower bounds of the grid, measured in number of voxels
	int vgrid_x_, vgrid_y_, vgrid_z_;	// Size of the voxel grid, measured in number of voxels
	int min_points_per_voxel_;

	std::vector<Eigen::Vector3d> centroid_;
	std::vector<Eigen::Matrix3d> covariance_;
	std::vector<Eigen::Matrix3d> icovariance_;
	std::vector<std::vector<int> > points_id_;
	std::vector<int> points_per_voxel_;

	/* Octree
	 * Each element stores centroids of all voxels in the level */
	std::vector<std::vector<Eigen::Vector3d> > octree_centroids_;
	std::vector<std::vector<int> > points_per_node_;
	std::vector<OctreeGridSize> octree_size_of_level_;
};
}

#endif
