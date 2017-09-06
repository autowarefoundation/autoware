#ifndef GNDT_H_
#define GNDT_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include "Matrix.h"
#include "MatrixHost.h"
#include "MatrixDevice.h"
#include "common.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace gpu {
class GRegistration {
public:
	GRegistration();
	GRegistration(const GRegistration &other);

	void align(Eigen::Matrix<float, 4, 4> &guess);

	inline void setTransformationEpsilon(double trans_eps)
	{
		transformation_epsilon_ = trans_eps;
	}

	inline void setMaximumIterations(int max_itr)
	{
		max_iterations_ = max_itr;
	}

	inline Eigen::Matrix<float, 4, 4> getFinalTransformation()
	{
		return final_transformation_;
	}

	/* Set input Scanned point cloud.
	 * Copy input points from the main memory to the GPU memory */
	void setInputSource(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
	void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

	/* Set input reference map point cloud.
	 * Copy input points from the main memory to the GPU memory */
	void setInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
	void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

	inline int getFinalNumIteration() { return nr_iterations_; }

	inline bool hasConverged() { return converged_; }

	virtual ~GRegistration();
protected:

	virtual void computeTransformation(Eigen::Matrix<float, 4, 4> &guess) {
		printf("Unsupported by Registration\n");
	}

	double transformation_epsilon_;
	int max_iterations_;

	//Original scanned point clouds
	float *x_, *y_, *z_;
	int points_number_;

	//Transformed point clouds
	float *trans_x_, *trans_y_, *trans_z_;

	bool converged_;
	int nr_iterations_;

	Eigen::Matrix<float, 4, 4> final_transformation_, transformation_, previous_transformation_;

	bool target_cloud_updated_;

	// Reference map point
	float *target_x_, *target_y_, *target_z_;
	int target_points_number_;

	bool is_copied_;
};
}

#endif
