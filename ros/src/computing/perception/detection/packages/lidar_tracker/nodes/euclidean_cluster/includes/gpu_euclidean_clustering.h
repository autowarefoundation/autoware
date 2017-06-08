#ifndef GPU_EUCLIDEAN_H_
#define GPU_EUCLIDEAN_H_

#include <iostream>
#include <vector>

class GpuEuclideanCluster {
public:
	typedef struct {
		int index_value;
		std::vector<int> points_in_cluster;
	} GClusterIndex;

	GpuEuclideanCluster();

	void setInputPoints(float *x, float *y, float *z, int size);
	void setThreshold(double threshold);
	void setMinClusterPts(int min_cluster_pts);
	void setMaxClusterPts(int max_cluster_pts);
	void extractClusters();
	std::vector<GClusterIndex> getOutput();

	~GpuEuclideanCluster();

private:
	float *x_, *y_, *z_;
	int size_;
	double threshold_;
	int *cluster_indices_;
	int *cluster_indices_host_;
	int min_cluster_pts_;
	int max_cluster_pts_;

	void exclusiveScan(int *input, int ele_num, int *sum);
};

#endif
